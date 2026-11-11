//! Butchered parts of deluge-sdk/firmwares/app-loader/src, mainly devupload.rs
//! mostly because I could, probably should upstream moving parts from the app-loader
//! to the deluge-sdk crate
//!
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

use deluge_image::elf::LoadResult;

pub struct Reloader {
}

use deluge::usb_upload;
impl usb_upload::UploadMixin for Reloader {
    fn set_upload_active(&self, active: bool) {
        UPLOAD_ACTIVE.store(active, Ordering::Release);
    }
    async fn ui_show_message(&self, line1: &[u8], line2: &[u8]) {
        show_message(line1, line2).await
    }
    async fn ui_show_progress(&self, label: &[u8], percent: u8) {
        show_progress(label, percent).await
    }
    async fn handoff(&self, result: LoadResult) {
        handoff(result).await
    }
}

pub async fn run(listener : usb_upload::Listener) -> ! {
    let x = Reloader {};
    listener.run(&x).await
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

    use deluge_bsp::launcher;
    if result.n_sram > 0 {
        unsafe {
            launcher::launch_via_trampoline(&result.sram_descs[..result.n_sram], result.entry)
        }
    } else {
        unsafe { launcher::launch(result.entry) }
    }
}

// ui.rs

use deluge_bsp::oled::text::draw_str;
use deluge_bsp::oled::{self, FrameBuffer, WIDTH};

const VISIBLE_ROWS: usize = 4;
const TOP_PAD: usize = 5;

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


