//! Per-mode settings through the whole engine.
//!
//! The profile vocabulary is held to its own arithmetic in `sdroxide-types`.
//! What only the engine can show is the three things the feature promises:
//! that choosing a mode lays its defaults on the receiver, that what the
//! operator changes while a mode is selected is remembered against *that* mode
//! alone, and that the reset puts the mode's own values back and forgets the
//! overrides for good. The last one is also checked across a restart, because
//! "remembered" that does not survive a launch is not remembered.

use std::time::{Duration, Instant};

use sdroxide_radio::{
    AudioParams, Complex32, EngineConfig, EngineHandles, IqSource, Result, rtrb, start_engine,
};
use sdroxide_types::{
    AgcMode, Command, DeviceCaps, Mode, NrLevel, RadioEvent, RadioState, RxId,
};

const RATE: f64 = 48_000.0;
const DIAL: f64 = 14_074_000.0;

/// A front end with nothing on it. The settings are the whole subject, so the
/// samples only have to keep the loop turning.
struct Quiet;

impl IqSource for Quiet {
    fn sample_rate(&self) -> f64 {
        RATE
    }
    fn center_hz(&self) -> f64 {
        DIAL
    }
    fn set_center_hz(&mut self, _hz: f64) -> Result<()> {
        Ok(())
    }
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        std::thread::sleep(Duration::from_millis(5));
        let n = buf.len().min(480);
        for z in buf[..n].iter_mut() {
            *z = Complex32::default();
        }
        Ok(n)
    }
    fn describe(&self) -> String {
        "quiet mock".into()
    }
}

fn caps() -> DeviceCaps {
    DeviceCaps {
        driver: "mock".into(),
        label: "mock".into(),
        rx_channels: 1,
        sample_rates: vec![RATE],
        freq_ranges_rx: vec![(1_000_000.0, 30_000_000.0)],
        ..DeviceCaps::default()
    }
}

/// A config directory of this test's own, so the profiles file it writes is not
/// the operator's and no session is picked up from one.
///
/// `SDROXIDE_CONFIG_DIR` is process-global, so the tests in this file take
/// [`CONFIG_LOCK`] around it: run in parallel they would each point the process
/// at a different directory and the engines would write over one another.
static CONFIG_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn isolate(name: &str) {
    let root = std::env::temp_dir().join(format!("sdroxide-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(&root).unwrap();
    unsafe { std::env::set_var("SDROXIDE_CONFIG_DIR", &root) };
}

fn start(mode: Mode) -> EngineHandles {
    let (producer, _consumer) = rtrb::RingBuffer::<f32>::new(48_000);
    start_engine(
        Box::new(Quiet),
        caps(),
        EngineConfig {
            audio: Some(AudioParams { producer, out_rate: RATE }),
            initial_mode: Some(mode),
            // The profiles are read and written by an engine that remembers,
            // which is also the only kind that will find the file this test
            // writes. The config directory above keeps it out of the operator's.
            remember_session: true,
            ..Default::default()
        },
    )
}

fn stop(mut h: EngineHandles) {
    let thread = h.thread.take();
    drop(h);
    if let Some(t) = thread {
        let _ = t.join();
    }
}

/// Wait for a state that satisfies `f`, or say what the last one was.
fn wait_for(
    h: &EngineHandles,
    what: &str,
    f: impl Fn(&RadioState) -> bool,
) -> RadioState {
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut last: Option<RadioState> = None;
    while Instant::now() < deadline {
        while let Ok(ev) = h.event_rx.try_recv() {
            if let RadioEvent::State(s) = ev {
                if f(&s) {
                    return s;
                }
                last = Some(s);
            }
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    panic!("the state never showed {what}; last: {:?}", last.map(|s| (s.rx[0].mode, s.rx[0].noise_reduction, s.rx[0].agc)));
}

fn send(h: &EngineHandles, c: Command) {
    h.cmd_tx.send(c).unwrap();
}

/// Choosing a mode lays its own settings on the receiver: an SSB voice mode
/// keeps the stock AGC, a weak-signal digital mode gets a slow one. The noise
/// reduction is off in both, because it is never defaulted on.
#[test]
fn a_mode_change_applies_that_modes_defaults() {
    let _guard = CONFIG_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    isolate("modeprofiles-apply");
    let h = start(Mode::Usb);

    let s = wait_for(&h, "USB's defaults", |s| {
        s.rx[0].mode == Mode::Usb && s.rx[0].agc == AgcMode::Med
    });
    assert_eq!(s.rx[0].noise_reduction, NrLevel::Off);

    send(&h, Command::SetMode { rx: RxId::Main, mode: Mode::Ft8 });
    let s = wait_for(&h, "FT8's defaults", |s| {
        s.rx[0].mode == Mode::Ft8 && s.rx[0].agc == AgcMode::Slow
    });
    assert_eq!(s.rx[0].noise_reduction, NrLevel::Off);

    stop(h);
}

/// What the operator changes is remembered against the mode it was changed in,
/// comes back with that mode, and is forgotten when the mode is reset.
#[test]
fn the_operators_change_is_kept_per_mode_and_reset() {
    let _guard = CONFIG_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    isolate("modeprofiles-remember");
    let h = start(Mode::Ft8);

    // Turn the noise reduction up in FT8. It ships off there, so this is a
    // departure and worth remembering.
    send(&h, Command::SetNoiseReduction { rx: RxId::Main, level: NrLevel::High });
    let _ = wait_for(&h, "FT8's changed NR", |s| {
        s.rx[0].mode == Mode::Ft8 && s.rx[0].noise_reduction == NrLevel::High
    });

    // Usb is not FT8: it comes up with its own default, not the FT8 value.
    send(&h, Command::SetMode { rx: RxId::Main, mode: Mode::Usb });
    let s = wait_for(&h, "Usb's own default", |s| {
        s.rx[0].mode == Mode::Usb && s.rx[0].noise_reduction != NrLevel::High
    });
    assert_eq!(s.rx[0].noise_reduction, NrLevel::Off);

    // ...and FT8 remembers.
    send(&h, Command::SetMode { rx: RxId::Main, mode: Mode::Ft8 });
    let s = wait_for(&h, "FT8's remembered NR", |s| {
        s.rx[0].mode == Mode::Ft8 && s.rx[0].noise_reduction == NrLevel::High
    });
    assert_eq!(s.rx[0].agc, AgcMode::Slow, "the untouched fields are still the mode's");

    // Reset: the mode's own values return...
    send(&h, Command::ResetModeDefaults { mode: Some(Mode::Ft8) });
    let s = wait_for(&h, "FT8 back on its defaults", |s| {
        s.rx[0].mode == Mode::Ft8 && s.rx[0].noise_reduction == NrLevel::Off
    });
    assert_eq!(s.rx[0].agc, AgcMode::Slow);

    // ...and the override is gone, not merely hidden: leaving and returning is
    // still the default.
    send(&h, Command::SetMode { rx: RxId::Main, mode: Mode::Usb });
    let _ = wait_for(&h, "Usb", |s| s.rx[0].mode == Mode::Usb);
    send(&h, Command::SetMode { rx: RxId::Main, mode: Mode::Ft8 });
    let s = wait_for(&h, "FT8 with nothing remembered", |s| {
        s.rx[0].mode == Mode::Ft8 && s.rx[0].noise_reduction == NrLevel::Off
    });
    assert_eq!(s.rx[0].noise_reduction, NrLevel::Off);

    stop(h);
}

/// A reset with no mode forgets every mode's overrides.
#[test]
fn resetting_every_mode_puts_all_of_them_back() {
    let _guard = CONFIG_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    isolate("modeprofiles-reset-all");
    let h = start(Mode::Ft8);

    send(&h, Command::SetSquelch { rx: RxId::Main, db: -80.0 });
    let _ = wait_for(&h, "FT8's squelch", |s| (s.rx[0].squelch_db + 80.0).abs() < 0.01);
    send(&h, Command::SetMode { rx: RxId::Main, mode: Mode::Usb });
    send(&h, Command::SetSquelch { rx: RxId::Main, db: -70.0 });
    let _ = wait_for(&h, "Usb's squelch", |s| {
        s.rx[0].mode == Mode::Usb && (s.rx[0].squelch_db + 70.0).abs() < 0.01
    });

    send(&h, Command::ResetModeDefaults { mode: None });
    let s = wait_for(&h, "Usb open again", |s| {
        s.rx[0].mode == Mode::Usb
            && (s.rx[0].squelch_db - sdroxide_types::SQUELCH_OPEN_DB).abs() < 0.01
    });
    assert!((s.rx[0].squelch_db - sdroxide_types::SQUELCH_OPEN_DB).abs() < 0.01);

    stop(h);
}

/// Remembered means it survives a launch: a fresh engine in the same mode reads
/// the file and applies the override with no command having been sent.
#[test]
fn the_overrides_survive_a_restart() {
    let _guard = CONFIG_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    isolate("modeprofiles-restart");
    {
        let h = start(Mode::Wspr);
        send(&h, Command::SetSquelch { rx: RxId::Main, db: -80.0 });
        let _ = wait_for(&h, "WSPR's squelch", |s| (s.rx[0].squelch_db + 80.0).abs() < 0.01);
        // The write happens in the command handler, so by the state that proves
        // the change it is already on disk.
        stop(h);
    }

    let h = start(Mode::Wspr);
    let s = wait_for(&h, "WSPR's remembered squelch", |s| {
        s.rx[0].mode == Mode::Wspr && (s.rx[0].squelch_db + 80.0).abs() < 0.01
    });
    assert_eq!(s.rx[0].agc, AgcMode::Slow, "the mode's own defaults are still under it");
    stop(h);
}
