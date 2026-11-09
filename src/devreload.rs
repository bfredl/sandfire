//! USB **dev-mode upload** listener: a background CDC-ACM serial endpoint that
//! receives a framed ELF straight from the host (`cargo deluge run`) and loads +
//! launches it to RAM — the same hand-off the SD `/APPS/` boot path uses, but
//! sourced from USB and with no SD shuffling.
//!
//! Unlike [`crate::usbmsc`], this is **not a mode the user enters**: when dev
//! mode is on, `boot_task` races [`Listener::run`] against the menu selector
//! while the boot menu is shown.  The listener draws nothing until a valid
//! upload header arrives; the moment a complete, CRC-checked image
//! is received it performs the launch itself and never returns.  A bad frame is
//! resynced and listening continues.
//!
//! ## Wire protocol (host → device, little-endian)
//! ```text
//! magic b"DLUP" | version u8 | flags u8 | len u32 | crc32 u32 | <len ELF bytes>
//! ```
//! `crc32` is the shared [`deluge_image::crc32`] of the `len` ELF bytes.  The
//! image is streamed straight to each `PT_LOAD` segment's load / SRAM-staging
//! address as it arrives, checksummed incrementally, and launched once the CRC
//! verifies.

use core::sync::atomic::Ordering;

use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, State};
use log::{info, warn};

use rza1_hal::usb::{Rusb1Driver, disconnect, init_device_mode};

use deluge_image::Crc32;
use deluge_image::elf::{MAX_PHDRS, SDRAM_HI, SDRAM_LO, StreamRouter};

// ── Wire-protocol constants ───────────────────────────────────────────────────

/// Frame magic preceding every upload.
const MAGIC: [u8; 4] = *b"DLUP";
/// Protocol version this loader speaks.
const VERSION: u8 = 1;
/// Bytes of fixed header after the magic: `version | flags | len | crc32`.
const HEADER_TAIL: usize = 1 + 1 + 4 + 4;

/// CDC bulk-endpoint max packet size.  The RUSB1 PHY negotiates high speed, and
/// USB 2.0 requires HS bulk endpoints to advertise 512.
const MAX_PACKET: u16 = 512;

// ── Streaming loader window ───────────────────────────────────────────────────
//
// The upload is routed straight to each segment's load / SRAM-staging address as
// it arrives — there is no whole-image scratch buffer. Only the front matter
// (ELF header + program-header table) is buffered, and the app-segment ceiling
// bounds a sane maximum upload length.
/// Bytes of front matter buffered to build the [`StreamRouter`]: ELF header plus
/// the largest program-header table.
const HEADER_BUF: usize = 52 + MAX_PHDRS * 32;
/// Largest upload accepted (the SDRAM app-region span). Per-segment placement is
/// still validated; this only rejects absurd lengths early.
const MAX_UPLOAD: u32 = SDRAM_HI - SDRAM_LO;

// ── USB descriptor / class `'static` backing storage ──────────────────────────

static mut USB_CONFIG_DESC: [u8; 256] = [0; 256];
static mut USB_BOS_DESC: [u8; 64] = [0; 64];
static mut USB_MSOS_DESC: [u8; 0] = [];
static mut USB_CONTROL_BUF: [u8; 64] = [0; 64];
static mut CDC_STATE: State = State::new();

rza1_hal::bind_interrupts!(struct UsbIrqs {
    USB0 => rza1_hal::usb::DeviceInterruptHandler<rza1_hal::peripherals::USB0>;
});

/// A brought-up dev-upload USB device, ready to listen.
///
/// Created by [`prepare`] **before** the boot menu starts drawing: USB bring-up
/// reconfigures interrupts/clocks, and doing it while an OLED frame DMA (and its
/// PIC chip-select handshake) is in flight can wedge the display so the menu
/// never redraws ([`crate::usbmsc`] likewise builds USB before starting its OLED
/// loop). [`run`](Listener::run) then drives it concurrently
/// with the menu selector.
pub struct Listener {
    device: embassy_usb::UsbDevice<'static, Rusb1Driver<'static>>,
    rx: Receiver<'static, Rusb1Driver<'static>>,
}

/// Bring up the dev-upload CDC device. Call this **before** the menu selector
/// starts drawing (see [`Listener`]), then `.await` [`Listener::run`].
pub fn prepare() -> Listener {
    let (device, cdc) = unsafe { build_usb() };
    info!("DEVRELOAD: CDC listener up (waiting for upload)");
    let (_tx, rx) = cdc.split();
    Listener { device, rx }
}

impl Listener {
    /// Run the CDC device alongside the frame receiver. The receiver only
    /// returns by loading and launching a received image, so this future never
    /// resolves — `boot_task` races it against the menu selector and treats its
    /// completion as "an upload happened".
    pub async fn run(self) -> ! {
        let Listener { mut device, rx } = self;
        join(device.run(), receive(rx)).await;
        // `receive` is `-> !`; the join can never resolve.
        unreachable!()
    }
}

/// Build the USB device in CDC-ACM configuration.  Mirrors `usbmsc::build_usb`
/// but uses a distinct product string so the host can pick the right
/// `/dev/ttyACM*`.
///
/// # Safety
/// Mutates the module's `'static` descriptor buffers; one listener at a time.
unsafe fn build_usb() -> (
    embassy_usb::UsbDevice<'static, Rusb1Driver<'static>>,
    CdcAcmClass<'static, Rusb1Driver<'static>>,
) {
    unsafe {
        // SAFETY: one listener at a time (this function's contract): the previous session's
        // driver is gone, so this session is USB0's only user.
        let usb0 = rza1_hal::peripherals::USB0::steal();
        let (_port, driver) = init_device_mode(usb0, UsbIrqs);
        let mut config = embassy_usb::Config::new(
            deluge_bsp::usb::ids::VID,
            deluge_bsp::usb::ids::PID_APP_LOADER_CDC,
        );
        config.manufacturer = Some("Synthstrom Audible");
        config.product = Some("Deluge Dev Upload");
        config.self_powered = false;
        config.max_power = 250; // 500 mA

        let mut builder = embassy_usb::Builder::new(
            driver,
            config,
            &mut *core::ptr::addr_of_mut!(USB_CONFIG_DESC),
            &mut *core::ptr::addr_of_mut!(USB_BOS_DESC),
            &mut *core::ptr::addr_of_mut!(USB_MSOS_DESC),
            &mut *core::ptr::addr_of_mut!(USB_CONTROL_BUF),
        );

        let cdc = CdcAcmClass::new(
            &mut builder,
            &mut *core::ptr::addr_of_mut!(CDC_STATE),
            MAX_PACKET,
        );

        (builder.build(), cdc)
    }
}

/// Copy a contiguous run of upload bytes starting at file offset `off` to their
/// segment destinations, discarding bytes that fall in gaps.
///
/// # Safety
/// Writes physical RAM at addresses the [`StreamRouter`] validated via
/// `place_segment`; the caller must ensure no live data occupies those regions
/// (true during the boot menu, like the SD loader).
unsafe fn route_bytes(router: &StreamRouter, mut off: u32, mut data: &[u8]) {
    while !data.is_empty() {
        let step = router.route_at(off);
        let take = (step.run as usize).min(data.len());
        if let Some(dst) = step.dst {
            unsafe { core::ptr::copy_nonoverlapping(data.as_ptr(), dst as *mut u8, take) };
        }
        off += take as u32;
        data = &data[take..];
    }
}

/// Receive framed uploads forever.  Returns only by loading and launching an
/// image (so the return type is `!`); a malformed/short/CRC-bad frame is logged,
/// reported on the OLED, and listening resumes.
async fn receive(rx: Receiver<'static, Rusb1Driver<'static>>) -> ! {
    let mut reader = PacketReader::new(rx);
    loop {
        // Resync to the frame magic, then read the fixed header tail.
        reader.sync_to_magic().await;
        warn!("DEVRELOAD: synced");
        let mut tail = [0u8; HEADER_TAIL];
        reader.read_exact(&mut tail).await;
        let version = tail[0];
        let _flags = tail[1];
        let len = u32::from_le_bytes([tail[2], tail[3], tail[4], tail[5]]);
        let expect_crc = u32::from_le_bytes([tail[6], tail[7], tail[8], tail[9]]);
        warn!("DEVRELOAD: thing");

        if version != VERSION || len < 52 || len > MAX_UPLOAD {
            warn!("DEVRELOAD: bad header (version={version}, len={len}); resyncing");
            continue;
        }

        UPLOAD_ACTIVE.store(true, Ordering::Release);
        info!("DEVRELOAD: streaming {len} byte image");

        let mut crc = Crc32::new();

        // 1. Buffer + CRC the front matter (ELF header + program headers).
        let mut prefix = [0u8; HEADER_BUF];
        reader.read_exact(&mut prefix[..52]).await;
        let e_phoff = deluge_image::elf::le32(&prefix, 28);
        let e_phnum = deluge_image::elf::le16(&prefix, 44) as usize;
        if e_phoff < 52 || e_phnum > MAX_PHDRS {
            warn!("DEVRELOAD: bad phdr table (e_phoff={e_phoff}, e_phnum={e_phnum})");
            show_message(b"UPLOAD ERROR", b"BAD LAYOUT").await;
            Timer::after(Duration::from_secs(2)).await;
            UPLOAD_ACTIVE.store(false, Ordering::Release);
            continue;
        }
        let header_end = match (e_phoff as usize).checked_add(e_phnum * 32) {
            Some(h) => h,
            None => {
                warn!("DEVRELOAD: header offset overflow (e_phoff={e_phoff})");
                show_message(b"UPLOAD ERROR", b"BAD LAYOUT").await;
                Timer::after(Duration::from_secs(2)).await;
                UPLOAD_ACTIVE.store(false, Ordering::Release);
                continue;
            }
        };
        if header_end > HEADER_BUF || header_end as u32 > len {
            warn!("DEVRELOAD: phdr table outside header window (header_end={header_end})");
            show_message(b"UPLOAD ERROR", b"BAD LAYOUT").await;
            Timer::after(Duration::from_secs(2)).await;
            UPLOAD_ACTIVE.store(false, Ordering::Release);
            continue;
        }
        if header_end > 52 {
            reader.read_exact(&mut prefix[52..header_end]).await;
        }
        crc.update(&prefix[..header_end]);

        // 2. Build + validate the route plan.
        let router = match StreamRouter::new(&prefix[..header_end]) {
            Ok(r) => r,
            Err(e) => {
                let line2: &[u8] = match ElfError::from(e) {
                    ElfError::BadMagic => b"BAD MAGIC",
                    ElfError::WrongFormat => b"WRONG FORMAT",
                    ElfError::BadLoadAddress => b"BAD LOAD ADDR",
                    ElfError::Unstreamable => b"BAD LAYOUT",
                    _ => b"SEE LOG",
                };
                warn!("DEVRELOAD: image rejected: {e:?}");
                show_message(b"UPLOAD ERROR", line2).await;
                Timer::after(Duration::from_secs(2)).await;
                UPLOAD_ACTIVE.store(false, Ordering::Release);
                continue;
            }
        };

        // 3. Replay the buffered front matter through the router (handles a
        //    first segment whose p_offset is 0), then stream the remainder.
        unsafe { route_bytes(&router, 0, &prefix[..header_end]) };
        let total = len as usize;
        let mut off = header_end as u32;
        let mut remaining = total - header_end;
        let mut last_pct = u8::MAX;
        while remaining > 0 {
            let chunk = reader.next_chunk(remaining).await;
            crc.update(chunk);
            unsafe { route_bytes(&router, off, chunk) };
            off += chunk.len() as u32;
            remaining -= chunk.len();
            let pct = (((total - remaining) as u64) * 100 / total as u64) as u8;
            if pct != last_pct {
                show_progress(b"RECEIVING", pct).await;
                last_pct = pct;
            }
        }

        // 4. Verify integrity before anything irreversible.
        if crc.finalize() != expect_crc {
            warn!("DEVRELOAD: CRC mismatch");
            show_message(b"UPLOAD ERROR", b"BAD CRC").await;
            Timer::after(Duration::from_secs(2)).await;
            UPLOAD_ACTIVE.store(false, Ordering::Release);
            continue;
        }

        // 5. Zero-extend BSS tails and collect SRAM descriptors for handoff.
        let mut sram_descs = [SramSegDesc::default(); MAX_PHDRS];
        let mut n_sram = 0usize;
        for seg in router.segments() {
            let filesz = seg.file_end - seg.file_start;
            let zero_extra = seg.memsz - filesz;
            if zero_extra > 0 {
                unsafe {
                    core::ptr::write_bytes(
                        (seg.write_addr + filesz) as *mut u8,
                        0,
                        zero_extra as usize,
                    )
                };
            }
            if seg.sram {
                sram_descs[n_sram] = SramSegDesc {
                    src: seg.write_addr,
                    dst: seg.final_dst,
                    filesz,
                    zero_extra,
                };
                n_sram += 1;
            }
        }

        let result = LoadResult {
            entry: router.entry(),
            sram_descs,
            n_sram,
        };
        info!("DEVRELOAD: image loaded, entry={:#010x}", result.entry);
        handoff(result).await
    }
}

/// Final handoff: tear down USB cleanly (so the host re-enumerates the app's own
/// usb-log CDC) and launch the loaded image. Mirrors the SD ELF path in
/// `boot_task` (boot logo, disable interrupts, quiesce, launch). Never returns.
async fn handoff(result: LoadResult) -> ! {
    show_message(b"LAUNCHING", b"FROM USB").await;

    // Draw the boot droplet before interrupts/DMA are quiesced (must run while
    // the executor + pic_rx_task are still live — see `crate::show_boot_logo`).
    show_boot_logo().await;

    // Unplug from the host: it sees a clean disconnect and re-enumerates the
    // launched app's own USB stack (e.g. the usb-log CDC for `cargo deluge run
    // --log`).
    // SAFETY: the listener's session is over: nothing drives USB0 past this point.
    disconnect(unsafe { rza1_hal::peripherals::USB0::steal() });

    cortex_ar::interrupt::disable();
    unsafe { quiesce_for_handoff() };

    if result.n_sram > 0 {
        unsafe {
            launch_via_trampoline(&result.sram_descs[..result.n_sram], result.entry)
        }
    } else {
        unsafe { launch(result.entry) }
    }
}

/// Buffered reader over the CDC OUT endpoint: `read_packet` only yields whole
/// packets, so this re-packetises into byte / fixed-length / bulk reads.
struct PacketReader {
    rx: Receiver<'static, Rusb1Driver<'static>>,
    buf: [u8; MAX_PACKET as usize],
    pos: usize,
    fill: usize,
    connected: bool,
}

impl PacketReader {
    fn new(rx: Receiver<'static, Rusb1Driver<'static>>) -> Self {
        Self {
            rx,
            buf: [0u8; MAX_PACKET as usize],
            pos: 0,
            fill: 0,
            connected: false,
        }
    }

    /// Refill the internal buffer with the next non-empty packet, (re)waiting for
    /// the host to open the port across disconnects.
    async fn refill(&mut self) {
        loop {
            if !self.connected {
                warn!("DEVRELOAD: before");
                self.rx.wait_connection().await;
                warn!("DEVRELOAD: after"); // this message gets dropped:D
                self.connected = true;
            }
            match self.rx.read_packet(&mut self.buf).await {
                Ok(n) if n > 0 => {
                    self.pos = 0;
                    self.fill = n;
                    return;
                }
                Ok(_) => {}                       // zero-length packet; keep reading
                Err(_) => self.connected = false, // host went away; rewait
            }
        }
    }

    /// Read one byte.
    async fn byte(&mut self) -> u8 {
        if self.pos >= self.fill {
            self.refill().await;
        }
        let b = self.buf[self.pos];
        self.pos += 1;
        b
    }

    /// Read exactly `dst.len()` bytes.
    async fn read_exact(&mut self, dst: &mut [u8]) {
        let mut i = 0;
        while i < dst.len() {
            if self.pos >= self.fill {
                self.refill().await;
            }
            let take = (self.fill - self.pos).min(dst.len() - i);
            dst[i..i + take].copy_from_slice(&self.buf[self.pos..self.pos + take]);
            self.pos += take;
            i += take;
        }
    }

    /// Slide a 4-byte window until it matches the frame magic.
    async fn sync_to_magic(&mut self) {
        let mut window = [0u8; 4];
        // Prime the window with the first four bytes.
        for slot in window.iter_mut() {
            *slot = self.byte().await;
        }
        while window != MAGIC {
            window.rotate_left(1);
            window[3] = self.byte().await;
        }
    }

    /// Borrow the next run of received bytes (up to `remaining`), refilling from
    /// USB as needed. Never returns empty until the caller has taken `remaining`
    /// bytes across calls.
    async fn next_chunk(&mut self, remaining: usize) -> &[u8] {
        if self.pos >= self.fill {
            self.refill().await;
        }
        let take = (self.fill - self.pos).min(remaining);
        let chunk = &self.buf[self.pos..self.pos + take];
        self.pos += take;
        chunk
    }
}

// launcher.rs

use core::arch::{asm, global_asm};

use rza1_hal::cache;

pub use deluge_image::elf::SegDesc as SramSegDesc;

// ---------------------------------------------------------------------------
// Retention-RAM layout
// ---------------------------------------------------------------------------

/// Base address of the data-retention RAM window.  The trampoline code is
/// copied here at run time.
const RETRAM_CODE: usize = 0x2000_0000;

/// Offset within retention RAM where the `SramSegDesc` table is placed.
/// Chosen to give the trampoline code plenty of room (256 bytes).
const RETRAM_TABLE_OFFSET: usize = 0x100;

// ---------------------------------------------------------------------------
// Trampoline blob (global_asm)
// ---------------------------------------------------------------------------
//
// This is a position-independent ARM32 routine.  It is linked into the
// bootloader's SRAM image, but we *copy* it into retention RAM at
// `RETRAM_CODE` before jumping there.
//
// Calling convention (matching `extern "C"` / AAPCS):
//   R0 = *const SramSegDesc  (table in retention RAM)
//   R1 = count: u32
//   R2 = entry: u32
//
// The function never returns and never touches the stack, because SP still
// points into SRAM which is being overwritten.
global_asm!(
    r#"
    .section .text._trampoline, "ax"
    .code 32
    .global _trampoline_start
    .global _trampoline_end
_trampoline_start:
    @ r0 = descriptor table ptr, r1 = count, r2 = entry point
    @
    @ Disable the MMU and L1 caches before copying.  The RZ/A1L's caches do not
    @ reliably honour cache-maintenance operations (DCCMVAC/DCCMVAU and the
    @ ARM-recommended L1 flush logic) -- see the Deluge firmware's chainload.S,
    @ which copies a new image into RAM the same way.  A cached copy therefore
    @ leaves the upper part of a large image stranded in cache and never written
    @ to physical RAM.  With caches off every store goes straight to memory.
    @ The page table is a flat VA=PA map, so turning the MMU off does not move
    @ this code (it runs from retention RAM at its physical address) and the
    @ copy addresses stay valid.
    cpsid   if                       @ mask IRQ/FIQ for the handoff
    mrc     p15, 0, r9, c1, c0, 0    @ read SCTLR
    bic     r9, r9, #(1 << 12)       @ I:  disable L1 instruction cache
    bic     r9, r9, #(1 <<  2)       @ C:  disable L1 data cache
    bic     r9, r9, #(1 <<  0)       @ M:  disable MMU
    mcr     p15, 0, r9, c1, c0, 0    @ write SCTLR
    dsb
    isb
.Lseg_loop:
    cmp     r1, #0
    beq     .Ldone
    ldr     r3, [r0, #0]      @ src  (SDRAM staging address)
    ldr     r4, [r0, #4]      @ dst  (final SRAM address)
    ldr     r5, [r0, #8]      @ filesz
    ldr     r6, [r0, #12]     @ zero_extra
.Lcopy_loop:
    cmp     r5, #0
    beq     .Lzero_loop
    ldrb    r7, [r3], #1
    strb    r7, [r4], #1
    sub     r5, r5, #1
    b       .Lcopy_loop
.Lzero_loop:
    cmp     r6, #0
    beq     .Lnext_seg
    mov     r7, #0
    strb    r7, [r4], #1
    sub     r6, r6, #1
    b       .Lzero_loop
.Lnext_seg:
    add     r0, r0,  #16      @ advance to next descriptor
    sub     r1, r1,  #1
    b       .Lseg_loop
.Ldone:
    @ Stores went straight to physical RAM (caches off).  Invalidate the I-cache
    @ and branch predictor and barrier before branching so the app entry fetches
    @ the freshly written code, then jump.  The app re-enables its own MMU/cache.
    mov     r7, #0
    mcr     p15, 0, r7, c7, c5, 0    @ ICIALLU: invalidate entire I-cache
    mcr     p15, 0, r7, c7, c5, 6    @ BPIALL:  invalidate branch predictor
    dsb
    isb
    bx      r2                @ jump to app entry point
_trampoline_end:
    "#
);

unsafe extern "C" {
    /// First byte of the trampoline blob in the linked image.
    static _trampoline_start: u8;
    /// First byte past the end of the trampoline blob.
    static _trampoline_end: u8;
}

// ---------------------------------------------------------------------------
// Cache-flush helpers (shared between both launch paths)
// ---------------------------------------------------------------------------

#[inline]
unsafe fn flush_all_caches() {
    unsafe {
        cache::l1_d_clean_inv_all();

        const L2C_BASE: usize = 0x3FFF_F000;
        const L2C_CTRL: *mut u32 = (L2C_BASE + 0x100) as *mut u32;
        const L2C_CLEAN_INV_WAY: *mut u32 = (L2C_BASE + 0x7FC) as *mut u32;
        const L2C_CACHE_SYNC: *mut u32 = (L2C_BASE + 0x730) as *mut u32;
        const L2C_8WAY: u32 = 0xFF;

        L2C_CLEAN_INV_WAY.write_volatile(L2C_8WAY);
        while L2C_CLEAN_INV_WAY.read_volatile() & L2C_8WAY != 0 {}
        // Drain the PL310 eviction write-buffer before disabling L2.
        L2C_CACHE_SYNC.write_volatile(0);
        L2C_CTRL.write_volatile(0);

        cache::l1_i_invalidate_all();

        asm!("dsb", options(nostack));
        asm!("isb", options(nomem, nostack));
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Flush all caches and branch directly to `entry`.
///
/// Use this when the app has no SRAM-targeting segments (i.e.
/// [`crate::elf::LoadResult::n_sram`] is zero).
///
/// # Safety
/// `entry` must be the address of a valid ARM-state function.
pub unsafe fn launch(entry: u32) -> ! {
    unsafe {
        flush_all_caches();
        let f: unsafe extern "C" fn() -> ! = core::mem::transmute(entry as usize);
        f()
    }
}

/// Stage-then-launch: copy the trampoline + descriptors to retention RAM,
/// flush all caches, then branch into the trampoline.
///
/// The trampoline copies each `SramSegDesc` from SDRAM staging to its final
/// SRAM address, then branches to `entry`.
///
/// # Safety
/// - Each descriptor in `descs` must name a fully staged source and a valid
///   SRAM destination (as produced by [`crate::elf::load_from_sd_with_progress`],
///   the dev-upload path, or [`crate::flashboot`]).
/// - Nothing in the retention RAM region (`0x20000000–0x2001FFFF`) must be
///   live; the first-stage bootloader guarantees this.
/// - `entry` must be the address of a valid ARM-state function.
pub unsafe fn launch_via_trampoline(descs: &[SramSegDesc], entry: u32) -> ! {
    unsafe {
        let code_start = core::ptr::addr_of!(_trampoline_start);
        let code_end = core::ptr::addr_of!(_trampoline_end);
        let code_len = code_end as usize - code_start as usize;

        // The descriptor table sits at RETRAM_TABLE_OFFSET; the code must fit
        // below it or the copy below would clobber the table we're about to write.
        debug_assert!(
            code_len <= RETRAM_TABLE_OFFSET,
            "trampoline code overruns the descriptor table offset"
        );

        // Copy trampoline code into retention RAM.
        core::ptr::copy_nonoverlapping(code_start, RETRAM_CODE as *mut u8, code_len);

        // Copy descriptor table right after the code (at RETRAM_TABLE_OFFSET).
        let table_dst = (RETRAM_CODE + RETRAM_TABLE_OFFSET) as *mut SramSegDesc;
        core::ptr::copy_nonoverlapping(descs.as_ptr(), table_dst, descs.len());

        flush_all_caches();

        // Jump into the trampoline in retention RAM.
        // R0 = table, R1 = count, R2 = entry  (AAPCS).
        let trampoline: unsafe extern "C" fn(*const SramSegDesc, u32, u32) -> ! =
            core::mem::transmute(RETRAM_CODE);
        trampoline(table_dst as *const SramSegDesc, descs.len() as u32, entry)
    }
}

// elf.rs

use deluge_image::elf::PlanError;

/// Result returned by a successful [`load_from_sd_with_progress`] call.
pub struct LoadResult {
    /// Application entry point (`e_entry`).
    pub entry: u32,
    /// Descriptors for SRAM segments parked in SDRAM staging.
    pub sram_descs: [SramSegDesc; MAX_PHDRS],
    /// Number of valid entries in `sram_descs`.
    pub n_sram: usize,
}

/// Errors from the ELF loader.
#[derive(Debug, Clone)]
pub enum ElfError {
    /// ELF magic bytes are not `\x7FELF`.
    BadMagic,
    /// Not a 32-bit little-endian ARM executable ELF.
    WrongFormat,
    /// A `PT_LOAD` segment's physical address is outside the permitted regions.
    BadLoadAddress,
    /// The uploaded ELF cannot be stream-loaded: its `PT_LOAD` segments are not
    /// in non-decreasing, non-overlapping file order, so the loader would have
    /// to seek backward to place them.
    Unstreamable,
    /// SD card / FAT read or seek error.
    // Io(#[allow(dead_code)] FatError),
    /// `read` returned 0 before the segment was fully copied.
    UnexpectedEof,
    /// A `PT_LOAD` segment targets a region other than upper SRAM, so the image
    /// cannot be stored as a single-descriptor flash-boot image.
    NotFlashable,
    /// The flattened image is larger than the flash app slot.
    TooLarge,
}

impl From<PlanError> for ElfError {
    fn from(e: PlanError) -> Self {
        match e {
            PlanError::BadMagic => ElfError::BadMagic,
            PlanError::WrongFormat => ElfError::WrongFormat,
            PlanError::BadLoadAddress => ElfError::BadLoadAddress,
            // A truncated image / short program-header table is an I/O-shaped
            // failure the slice loader maps to its EOF variant.
            PlanError::Truncated => ElfError::UnexpectedEof,
            PlanError::Unordered => ElfError::Unstreamable,
        }
    }
}
// ui.rs

use deluge_bsp::oled::text::draw_str;
use deluge_bsp::oled::{self, FrameBuffer, WIDTH};

const VISIBLE_ROWS: usize = 4;
/// Top padding in pixels.  The Deluge OLED panel's top 5 rows sit off the
/// visible area, so all content is shifted down to start at row 5 (matches the
/// demo/controller firmware's `TOPMOST = 5`).
const TOP_PAD: usize = 5;
/// Pixel row of the title bar.
const TITLE_ROW: usize = TOP_PAD;
/// Pixel row of the separator line.
const SEPARATOR_ROW: usize = TOP_PAD + 8;
/// Pixel row of the first entry line.
const ENTRY_START_ROW: usize = TOP_PAD + 11;
/// Height of one entry in pixels (one page = 8 rows).
const ENTRY_HEIGHT: usize = 8;

/// Display a static error or status message centred on the OLED.
pub async fn show_message(line1: &[u8], line2: &[u8]) {
    let mut fb = FrameBuffer::new();
    fb.fill(0x00);
    // Two lines, shifted down by the panel's top padding.
    let x1 = (WIDTH.saturating_sub(line1.len() * 6)) / 2;
    draw_str(&mut fb, x1, TOP_PAD + 16, line1);
    let x2 = (WIDTH.saturating_sub(line2.len() * 6)) / 2;
    draw_str(&mut fb, x2, TOP_PAD + 28, line2);
    oled::send_frame(&fb).await;
}

/// Display a simple progress bar with an app label.
pub async fn show_progress(label: &[u8], percent: u8) {
    let mut fb = FrameBuffer::new();
    fb.fill(0x00);

    let title = b"LOADING APP";
    let title_x = (WIDTH.saturating_sub(title.len() * 6)) / 2;
    draw_str(&mut fb, title_x, TOP_PAD + 8, title);

    let label_x = (WIDTH.saturating_sub(label.len() * 6)) / 2;
    draw_str(&mut fb, label_x, TOP_PAD + 18, label);

    let bar_x = 8usize;
    let bar_y = TOP_PAD + 30;
    let bar_w = WIDTH.saturating_sub(16);
    let bar_h = 10usize;

    for x in bar_x..(bar_x + bar_w) {
        fb.set_pixel(x, bar_y, true);
        fb.set_pixel(x, bar_y + bar_h - 1, true);
    }
    for y in bar_y..(bar_y + bar_h) {
        fb.set_pixel(bar_x, y, true);
        fb.set_pixel(bar_x + bar_w - 1, y, true);
    }

    let pct = core::cmp::min(percent, 100) as usize;
    let fill_w = (bar_w.saturating_sub(2) * pct) / 100;
    for y in (bar_y + 1)..(bar_y + bar_h - 1) {
        for x in (bar_x + 1)..(bar_x + 1 + fill_w) {
            fb.set_pixel(x, y, true);
        }
    }

    oled::send_frame(&fb).await;
}

pub async fn show_boot_logo() {
    oled::send_frame(&oled::boot_logo()).await;
}

/// Set by [`crate::devupload`] once an upload header has arrived, to claim the
/// OLED. While set, [`run_selector`] stops drawing *and* stops acting on input,
/// so the upload progress bar isn't fought over and an accidental encoder bump
/// can't abort an in-flight upload. Cleared again only if the upload is rejected
/// (a successful one launches and never returns).
pub static UPLOAD_ACTIVE: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);


    // main.rs
    //
/// Stop all autonomous peripheral activity before branching to the app.
///
/// Must run after `cortex_ar::interrupt::disable()` and before the launch.
/// The app re-initialises every controller from scratch, so it only needs the
/// hardware to be *quiet*: no DMA channel still transferring into RAM, no timer
/// or GIC source able to fire.  The critical one is the SCIF1/PIC circular
/// receive DMA — see the call site.
pub unsafe fn quiesce_for_handoff() {
    // Every launch passes through here, so this is where the bootloaders get
    // locked: the loader itself runs with the status register writable, which is
    // what lets a J-Link clear `BP` on a unit whose loader can't hand off.
    // #[cfg(feature = "flash-protect")]
    // lock_for_handoff();
    unsafe {
        // Stop + software-reset every DMA channel and clear its DMARS request
        // route (covers PIC RX/TX, SD, and OLED channels). Clearing DMARS lets
        // Linux's rz-dmac claim SCIF1-RX cleanly.
        rza1_hal::dmac::stop_all();
        // Stop the Embassy time-driver timers (OSTM0 free-run + OSTM1 alarm).
        rza1_hal::time_driver::stop();
        // Turn the interrupt controller off entirely.
        rza1_hal::gic::shutdown();
    }
}


