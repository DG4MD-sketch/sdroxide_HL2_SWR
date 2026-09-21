//! The panadapter's CTR must never park the hardware LO on the VFO.
//!
//! The report this comes from: an AM broadcast distorted "as if tuned 5 kHz
//! off" the moment CTR was clicked, cleared by any mode change, and clean
//! again once CTR was off. CTR keeps the window centred on the dial and asks
//! for that centre with `SetCenter`; taken literally it puts the LO exactly on
//! the VFO. A zero-IF front end has a DC spike there, and the carrier-centred
//! modes have passbands that contain DC — AM's is ±5 kHz — so the spike landed
//! in the demodulated channel and beat against the carrier. SSB and CW never
//! showed it because their passbands start a few hundred hertz up.
//!
//! `lo_guard_hz` already existed to keep the VFO away from the LO; the
//! `SetCenter` path was the one that went around it.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crossbeam_channel::Receiver;
use sdroxide_radio::{Complex32, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{Command, DeviceCaps, Mode, RadioEvent, RxId, Vfo};

const RATE: f64 = 2_000_000.0;
const CENTER: f64 = 13_970_000.0;
const VFO: f64 = 13_720_000.0;
/// What a zero-IF front end asks for: its LO parked this far off the VFO.
const LO_OFFSET: f64 = 500_000.0;
/// What the guard is actually for: DC has to land outside the demodulated
/// channel, and AM's passband is ±5 kHz. Asserted as a range rather than the
/// exact figure because `lo_guard_hz` is 0.6 of the *channel rate*, which the
/// mode and the front end's rate between them decide — the property is what
/// matters, not the arithmetic.
const MIN_CLEARANCE: f64 = 10_000.0;
/// ...and CTR is still centring the window, so the LO must not be thrown back
/// out to the full `lo_offset_hz` either.
const MAX_CLEARANCE: f64 = 60_000.0;

/// A zero-IF front end that takes every tune and remembers where it was sent.
struct ZeroIf {
    center_hz: f64,
    landed: Arc<Mutex<f64>>,
}

impl IqSource for ZeroIf {
    fn sample_rate(&self) -> f64 {
        RATE
    }
    fn center_hz(&self) -> f64 {
        self.center_hz
    }
    fn lo_offset_hz(&self) -> f64 {
        LO_OFFSET
    }
    fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        self.center_hz = hz;
        *self.landed.lock().unwrap() = hz;
        Ok(())
    }
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        std::thread::sleep(Duration::from_millis(5));
        let n = buf.len().min(256);
        buf[..n].fill(Complex32::new(0.0, 0.0));
        Ok(n)
    }
    fn describe(&self) -> String {
        "zero-IF test front end".into()
    }
}

fn caps() -> DeviceCaps {
    DeviceCaps {
        driver: "test".into(),
        label: "test".into(),
        rx_channels: 1,
        sample_rates: vec![RATE],
        freq_ranges_rx: vec![(1_000_000.0, 60_000_000.0)],
        ..DeviceCaps::default()
    }
}

/// The centre the engine settled on, after letting the commands run.
fn settled_center(rx: &Receiver<RadioEvent>, secs: f64) -> f64 {
    let mut center = f64::NAN;
    let deadline = Instant::now() + Duration::from_secs_f64(secs);
    while Instant::now() < deadline {
        while let Ok(ev) = rx.try_recv() {
            if let RadioEvent::State(s) = ev {
                center = s.center_hz;
            }
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    center
}

/// Drive an AM receiver on `VFO`, ask for `want` as the hardware centre, and
/// report where the engine and the front end each ended up.
fn ask_for_center(want: f64) -> (f64, f64) {
    let landed = Arc::new(Mutex::new(CENTER));
    let source = ZeroIf { center_hz: CENTER, landed: Arc::clone(&landed) };
    let mut h = start_engine(Box::new(source), caps(), EngineConfig::default());
    let thread = h.thread.take();

    h.cmd_tx.send(Command::SetMode { rx: RxId::Main, mode: Mode::Am }).unwrap();
    h.cmd_tx.send(Command::SetVfo { vfo: Vfo::A, hz: VFO }).unwrap();
    let _ = settled_center(&h.event_rx, 0.4);
    h.cmd_tx.send(Command::SetCenter(want)).unwrap();
    let center = settled_center(&h.event_rx, 0.6);
    let lo = *landed.lock().unwrap();

    drop(h.cmd_tx);
    if let Some(t) = thread {
        let _ = t.join();
    }
    (center, lo)
}

/// CTR asking for the dial itself is honoured only as far as the guard: the
/// window still moves, the LO stays out of the passband.
#[test]
fn centring_the_window_on_the_dial_keeps_the_lo_off_it() {
    let (center, lo) = ask_for_center(VFO);
    assert!(
        (center - VFO).abs() >= MIN_CLEARANCE,
        "the LO was parked {:.0} Hz from the dial, inside AM's passband",
        (center - VFO).abs()
    );
    assert!(
        (lo - VFO).abs() >= MIN_CLEARANCE,
        "the front end was sent to {lo}, {:.0} Hz from the dial",
        (lo - VFO).abs()
    );
}

/// ...and no further than the guard: CTR is still centring the window, so a
/// request inside it must not be thrown back out to the full LO offset.
#[test]
fn the_window_is_moved_as_near_the_dial_as_the_guard_allows() {
    let (center, _) = ask_for_center(VFO);
    assert!(
        (center - VFO).abs() <= MAX_CLEARANCE,
        "the window was pushed {:.0} Hz from the dial, further than the guard needs",
        (center - VFO).abs()
    );
}

/// A centre outside the guard is nobody's business but the caller's: an
/// ordinary pan of the panadapter goes through untouched.
#[test]
fn a_centre_outside_the_guard_is_honoured_exactly() {
    let want = VFO + 250_000.0;
    let (center, _) = ask_for_center(want);
    assert!((center - want).abs() < 1.0, "asked for {want}, engine settled on {center}");
}
