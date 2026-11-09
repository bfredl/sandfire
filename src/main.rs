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

use deluge_bsp::oled::{self, FrameBuffer, WIDTH};
#[deluge::app]
async fn main(dlg: Deluge) {

    // oled::init().await;
    _ = dlg.oled().await;
    let mut first = true;
    loop {
        use embassy_futures::select::{Either, select};

        let listener = devreload::prepare();
        match select(
            app_main(&dlg, first),
            listener.run(),
        ).await
        {
            Either::First(sel) => sel,
            Either::Second(never) => never,
        }
        first = false;
    }

}

async fn app_main(dlg: &Deluge, first: bool) {
    // The platform (heaps, clocks, interrupts, executor, panic handler) is
    // already up. Capabilities are taken from the `dlg` handle:
    //   let mut oled = dlg.oled().await;
    //   let input = dlg.input();
    //   let mut pads = dlg.pads().await;
    let mut leds = dlg.leds().await;
    let mut flip = true;
    loop {
        if first {leds.set(controls::button::PLAY, flip).await }
        Timer::after_millis(200).await;
        flip = !flip;
        // warn!("achtung?");
    }
}
