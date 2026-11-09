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
use embassy_futures::select::select;

#[deluge::app]
async fn main(dlg: Deluge) {
    // tricky: devreload requires oled already set up
    _ = dlg.oled().await;

    let listener = devreload::prepare();
    // join doesn't work:p
    select(app_main(dlg), listener.run()).await;
}

async fn app_main(dlg: Deluge) {
    let mut leds = dlg.leds().await;
    let mut flip = true;
    loop {
        leds.set(controls::button::PLAY, flip).await;
        Timer::after_millis(500).await;
        flip = !flip;
        // warn!("achtung?");
    }
}
