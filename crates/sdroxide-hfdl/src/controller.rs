//! Threading wrapper around `xng_mode_hfdl::HfdlChannelDecoder`, mirroring
//! `sdroxide_qo100::Qo100Controller`'s shape: the realtime engine thread ships
//! I/Q blocks to a worker over a bounded channel and drains status updates
//! non-blocking via [`HfdlController::poll`]. All the DSP runs on the worker.
//!
//! Simpler than the QO-100 wrapper in one way — the lane rate is fixed at
//! [`sdroxide_types::HFDL_LANE_RATE_HZ`] and the decoder carries no rolling
//! search window, so a dropped I/Q block costs a few milliseconds of a burst
//! rather than a whole frame, and there is nothing to do but restart. So the
//! bounded queue is the simple version (`try_send`, drop the whole block),
//! not the sequence-numbered restart the QO-100 worker needs.
//!
//! The decoder itself owns the HFDL state that must not be dropped — the LMS
//! equalizer's tap history, the decision-directed carrier loop, the
//! interleaver/Viterbi state — and that is exactly why it lives here rather
//! than being rebuilt per block.

use std::collections::VecDeque;
use std::thread::JoinHandle;

use crossbeam_channel::{Receiver, Sender, bounded, select, unbounded};
use sdroxide_dsp::Complex32;
use sdroxide_types::{HFDL_LOG_DEPTH, HfdlDecode, HfdlSettings, HfdlStatus};
use xng_mode_hfdl::HfdlChannelDecoder;
use xng_mode_hfdl::pdu::{HfdlEvent, gs_name};

/// Realtime data, dropped on backpressure.
struct Iq {
    samples: Vec<Complex32>,
}

/// Control traffic, never dropped.
enum Ctl {
    Config(HfdlSettings),
    Stop,
}

/// Bounded I/Q queue depth. Roughly a couple of seconds of channel-rate air
/// at a typical device read; an ordinary scheduling hiccup must not drop air,
/// and HFDL bursts are short enough that sustained backpressure past this can
/// only cost fragments, not whole frames.
const IQ_QUEUE_DEPTH: usize = 256;

/// How often a status snapshot goes out when nothing has decoded: once every
/// N blocks (about once a second at a normal device read), so the panel's
/// level meter keeps moving without flooding the event channel.
const IDLE_TICK: u64 = 250;

pub struct HfdlController {
    iq_tx: Sender<Iq>,
    ctl_tx: Sender<Ctl>,
    res_rx: Receiver<HfdlStatus>,
    worker: Option<JoinHandle<()>>,
}

impl HfdlController {
    /// `rate_hz` is the rate the engine is feeding (the DDC's actual output,
    /// [`sdroxide_types::HFDL_LANE_RATE_HZ`] or the nearest the DDC can make).
    /// The lane is centred on the channel, so the decoder is built with a
    /// zero offset — the +1440 Hz subcarrier is the decoder's own business.
    pub fn new(rate_hz: f64, cfg: HfdlSettings) -> Self {
        let (iq_tx, iq_rx) = bounded::<Iq>(IQ_QUEUE_DEPTH);
        let (ctl_tx, ctl_rx) = unbounded::<Ctl>();
        let (res_tx, res_rx) = unbounded::<HfdlStatus>();
        let worker = std::thread::Builder::new()
            .name("sdroxide-hfdl".into())
            .spawn({
                move || {
                    let mut cfg = cfg;
                    let mut decoder = match HfdlChannelDecoder::new(rate_hz, 0.0) {
                        Ok(d) => d,
                        Err(e) => {
                            tracing::error!(%e, "HFDL decoder could not start");
                            return;
                        }
                    };
                    let mut log: VecDeque<HfdlDecode> = VecDeque::with_capacity(HFDL_LOG_DEPTH);
                    let (mut bursts, mut decodes) = (0u64, 0u64);
                    let mut tick = 0u64;
                    loop {
                        select! {
                            recv(ctl_rx) -> msg => match msg {
                                Ok(Ctl::Config(next)) => cfg = next,
                                Ok(Ctl::Stop) | Err(_) => break,
                            },
                            recv(iq_rx) -> msg => match msg {
                                Ok(Iq { samples }) => {
                                    tick += 1;
                                    let mut emitted = false;
                                    if cfg.enabled {
                                        let events: Vec<HfdlEvent> = decoder.process(&samples);
                                        bursts += u64::from(!events.is_empty());
                                        decodes += events.len() as u64;
                                        if !events.is_empty() {
                                            let now = now_unix();
                                            for e in events {
                                                if log.len() == HFDL_LOG_DEPTH {
                                                    log.pop_back();
                                                }
                                                log.push_front(mk_decode(&cfg, &e, now));
                                            }
                                            emitted = true;
                                        }
                                    }
                                    // A status always comes back once (tick 1)
                                    // so a fresh client sees `running` and the
                                    // level immediately, then periodically even
                                    // through silence, and immediately whenever
                                    // anything decoded.
                                    if emitted || tick % IDLE_TICK == 0 || tick == 1 {
                                        let _ = res_tx.send(HfdlStatus {
                                            running: cfg.enabled,
                                            level_dbfs: decoder.level_dbfs(),
                                            bursts,
                                            decodes,
                                            log: log.iter().cloned().collect(),
                                        });
                                    }
                                }
                                Err(_) => break,
                            },
                        }
                    }
                }
            })
            .expect("spawn hfdl worker");
        HfdlController {
            iq_tx,
            ctl_tx,
            res_rx,
            worker: Some(worker),
        }
    }

    /// Realtime path: hand a block of lane-rate I/Q to the worker. Non-blocking.
    pub fn on_rx_iq(&self, iq: &[Complex32]) {
        let _ = self.iq_tx.try_send(Iq { samples: iq.to_vec() });
    }

    /// Apply new settings (currently just the channel and the on/off switch)
    /// to the running worker.
    pub fn set_config(&self, cfg: HfdlSettings) {
        let _ = self.ctl_tx.send(Ctl::Config(cfg));
    }

    /// Drain the latest status, non-blocking. Only the newest matters.
    pub fn poll(&self) -> Option<HfdlStatus> {
        let mut out = None;
        while let Ok(s) = self.res_rx.try_recv() {
            out = Some(s);
        }
        out
    }
}

fn now_unix() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0)
}

/// The ground station an event names, as `name (GS n)` — squitters and
/// frequency data always carry a `gs_id`; other kinds (aircraft data) do not.
fn gs_of(e: &HfdlEvent) -> Option<String> {
    let id = e.details.get("gs_id")?.as_u64()? as u8;
    let name = gs_name(id).unwrap_or("unknown");
    Some(format!("{name} (GS {id})"))
}

/// Map one decoded event onto the wire type's log entry.
fn mk_decode(cfg: &HfdlSettings, e: &HfdlEvent, unix: i64) -> HfdlDecode {
    HfdlDecode {
        unix,
        kind: e.kind.clone(),
        gs: gs_of(e),
        freq_khz: (cfg.frequency_hz / 1000.0).round() as u32,
        snr_db: e.snr_db,
        freq_skew_hz: e.freq_skew_hz,
        fec_corrected: e.fec_corrected,
        details: serde_json::to_string(&e.details).unwrap_or_default(),
    }
}

impl Drop for HfdlController {
    fn drop(&mut self) {
        let _ = self.ctl_tx.send(Ctl::Stop);
        if let Some(h) = self.worker.take() {
            let _ = h.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sdroxide_types::HfdlSettings;

    fn now_unix() -> i64 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs() as i64)
            .unwrap_or(0)
    }

    fn wait_for(
        c: &HfdlController,
        pred: impl Fn(&HfdlStatus) -> bool,
    ) -> Option<HfdlStatus> {
        let mut latest = None;
        for _ in 0..500 {
            if let Some(s) = c.poll() {
                let hit = pred(&s);
                latest = Some(s);
                if hit {
                    return latest;
                }
            }
            std::thread::sleep(std::time::Duration::from_millis(20));
        }
        latest
    }

    fn noise(len: usize) -> Vec<Complex32> {
        (0..len)
            .map(|i| {
                let (a, b) = ((i as f32 * 0.7).sin(), (i as f32 * 1.9 + 1.0).sin());
                Complex32::new(a, b)
            })
            .collect()
    }

    #[test]
    fn the_worker_reports_running_and_survives_pure_noise() {
        let c = HfdlController::new(
            24_000.0,
            HfdlSettings { enabled: true, frequency_hz: 21_931_000.0 },
        );
        c.on_rx_iq(&noise(24_000 * 2));
        let s = wait_for(&c, |s| s.running).expect("an idle snapshot should arrive");
        assert!(s.running);
        assert_eq!(s.decodes, 0, "pure noise must never decode");
    }

    /// End to end on the reference off-air capture: the worker, the bounded
    /// queue and the poll path hand a real 21 931 kHz Riverhead squitter back
    /// as a recognised decode — the same chain the engine drives.
    #[test]
    #[ignore = "points at an off-air capture via SDROXIDE_HFDL_SAMPLE"]
    fn an_off_air_capture_decodes_through_the_worker() {
        let path = std::env::var("SDROXIDE_HFDL_SAMPLE")
            .expect("set SDROXIDE_HFDL_SAMPLE to the i16 interleaved complex capture");
        let raw = std::fs::read(&path).expect("read capture");
        let samples: Vec<Complex32> = raw
            .chunks_exact(4)
            .map(|b| Complex32::new(
                i16::from_le_bytes([b[0], b[1]]) as f32 / 32768.0,
                i16::from_le_bytes([b[2], b[3]]) as f32 / 32768.0,
            ))
            .collect();
        let c = HfdlController::new(
            24_000.0,
            HfdlSettings { enabled: true, frequency_hz: 21_931_000.0 },
        );
        let before = now_unix();
        for chunk in samples.chunks(65_536) {
            c.on_rx_iq(chunk);
        }
        let s = wait_for(&c, |s| s.decodes >= 1).expect("a decode should come back");
        let squitter = s.log.iter().find(|d| d.kind == "squitter");
        assert!(squitter.is_some(), "no squitter in {:?}", s.log);
        let sq = squitter.unwrap();
        assert_eq!(sq.freq_khz, 21_931);
        assert!(sq.details.contains("\"gs_id\":4"), "Riverhead: {}", sq.details);
        assert!(sq.unix >= before, "log entry is stamped with a real time");
    }
}