//! A Deluge app.

// `no_std`/`no_main` only on the device; `cargo deluge sim` builds the same
// source as a normal std binary for the desktop simulator.
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
// Required by the Embassy task the `#[deluge::app]` macro generates.
#![feature(impl_trait_in_assoc_type)]

mod devreload;

use deluge::prelude::*;
use embassy_time::Timer;
use embassy_futures::select::{select, Either};

use deluge::usb_upload;

#[repr(C)]
struct WaveNetModel;

unsafe extern "C" {
    /// Sum every active voice's additive tone into `out` (mono, `n` samples,
    /// overwritten). See `csrc/additive.cpp` for the full contract.
    fn read_model(
        out: *mut WaveNetModel,
        n: *const u8,
        size: usize,
        max_samples: usize,
    ) -> bool;

    fn process_model(
        model: *const WaveNetModel,
        input: *const f32,
        output: *const f32,
        n_samples: usize,
        history: *mut f32,
        history_count: *mut usize,
        scratch: *mut usize,
        in_gain: f32,
        out_gain: f32
    ) -> bool;

    fn model_get_sizes(
        model: *const WaveNetModel,
        sampel: usize,
        history_size: *mut usize,
        scratch_size: *mut usize,
    );

    static mut the_model: WaveNetModel;
    static model_data: [u8; 7980];
}

#[deluge::app]
async fn main(dlg: Deluge) {
    // tricky: devreload requires oled already set up
    _ = dlg.oled().await;

    let listener = usb_upload::prepare();
    // join doesn't work:p
    select(app_main(dlg), devreload::run(listener)).await;
}

use deluge_bsp::oled::text::draw_str;
use deluge_bsp::oled::{self, FrameBuffer, WIDTH};

const VISIBLE_ROWS: usize = 4;
const TOP_PAD: usize = 5;

pub fn draw_center(fb: &mut FrameBuffer, ypos: usize, title: &[u8]) {
    let title_x = (WIDTH.saturating_sub(title.len() * 6)) / 2;
    draw_str(fb, title_x, ypos, title);
}


async fn app_main(dlg: Deluge) {
    let mut leds = dlg.leds().await;
    let mut led = dlg.sync_led();
    let input = dlg.input();
    let mut flip = true;
    let sampel = 128;

    let mut history : [f64; 44544] = [0.0; 44544];
    let mut scratch : [f64; 384] = [0.0; 384];

    let mut ok_model = false;
    let res = unsafe { read_model(&raw mut the_model, model_data.as_ptr(), 7980, sampel) };
    if res {
        leds.set(controls::button::LOAD, true).await;
        let mut history_size = 0;
        let mut scratch_size = 0;
        unsafe { model_get_sizes(&raw mut the_model, sampel, &raw mut history_size, &raw mut scratch_size); }

        let mut fb = FrameBuffer::new();
        fb.fill(0x00);
        let mut line = [0u8; 24];
        let mut p = 0;
        for &b in b"fooka " {
            push(&mut line, &mut p, b);
        }
        // normal languages: integer
        // rust: .TrY_iNTo().uNWrAP()
        push_u64(&mut line, &mut p, scratch_size.try_into().unwrap());
        draw_center(&mut fb, TOP_PAD+12, &line[..p]);
        oled::send_frame(&fb).await;

        if (history_size <= history.len() && scratch_size <= scratch.len()) {
            ok_model = true;
            leds.set(controls::button::SYNTH, true).await;
        } else {
            leds.set(controls::button::LEARN, true).await;
        }



    } else {
        leds.set(controls::button::SAVE, true).await;
    }

    loop {
        // this resets the timer on input, but whatever
        let res = select(input.next(), Timer::after_millis(500)).await;
        match res {
            Either::First(sel) => match sel {
                // Event::Encoder { .. } => led.toggle(),
                Event::Button { id:31 , pressed:true } => sound_main(&dlg).await,

                _ => {},
            },
            Either::Second(whatevs) => {
                leds.set(controls::button::PLAY, flip).await;
                flip = !flip;
            },
        };
        // warn!("achtung?");
    }
}

/// Cubic soft saturator: `1.5x - 0.5x³`, output in [-1, 1].
#[inline]
fn soft_clip(x: f32) -> f32 {
    let x = x.clamp(-1.0, 1.0);
    1.5 * x - 0.5 * x * x * x
}
use core::sync::atomic::Ordering;
async fn sound_main(dlg: &Deluge) {
    let drive = 2.5;
    dlg.audio() .process(move |block: &mut [StereoFrame]| {
        if devreload::UPLOAD_ACTIVE.load(Ordering::Acquire) {
            for f in block {
                f.l = 0.0;
                f.r = 0.0;
            }
            return;
        }
        for f in block {
            f.l = soft_clip(f.l * drive);
            f.r = soft_clip(f.r * drive);
        }
    }).await
}
#[inline]
fn push(out: &mut [u8], p: &mut usize, b: u8) {
    if *p < out.len() {
        out[*p] = b;
        *p += 1;
    }
}

/// Write a base-10 integer.
fn push_u64(out: &mut [u8], p: &mut usize, mut v: u64) {
    if v == 0 {
        push(out, p, b'0');
        return;
    }
    let mut tmp = [0u8; 20];
    let mut i = 0;
    while v > 0 {
        tmp[i] = b'0' + (v % 10) as u8;
        v /= 10;
        i += 1;
    }
    while i > 0 {
        i -= 1;
        push(out, p, tmp[i]);
    }
}

