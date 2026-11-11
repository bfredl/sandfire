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

#[deluge::app]
async fn main(dlg: Deluge) {
    // tricky: devreload requires oled already set up
    _ = dlg.oled().await;

    let listener = usb_upload::prepare();
    // join doesn't work:p
    select(app_main(dlg), devreload::run(listener)).await;
}

async fn app_main(dlg: Deluge) {
    let mut leds = dlg.leds().await;
    let mut led = dlg.sync_led();
    let input = dlg.input();
    let mut flip = true;
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
async fn sound_main(dlg: &Deluge) {
    let drive = 2.5;
    dlg.audio() .process(move |block: &mut [StereoFrame]| {
        for f in block {
            f.l = soft_clip(f.l * drive);
            f.r = soft_clip(f.r * drive);
        }
    }).await
}
