//! `Fsk441Controller` — FSK441, the original meteor-scatter mode, receive only.
//!
//! Slotted, and shaped like the other receive-only decoders here, with two
//! differences that matter:
//!
//! * **The rate.** FSK441's constants — 25 samples a dit at 441 baud — are the
//!   mode, and they only line up at 11 025 Hz. The tap is 48 kHz, so this
//!   resamples to [`sdroxide_dsp::FSK441_RATE`] rather than the 12 kHz the
//!   mfsk-core modes use, and the slot buffer is f32 at that rate.
//! * **The period** is an operator setting ([`Fsk441Period`]), not a property
//!   of the mode, so the slot length follows it.
//!
//! # Why a worker thread
//!
//! A whole-slot ping search is a sliding matched filter over thirty seconds of
//! audio, four tones deep, plus a decode per ping — far from free, and the
//! engine polls the controller on the audio thread. The decode runs on its own
//! thread and the result is drained from [`Self::poll`], one slot in flight at a
//! time, dropping rather than queueing when the machine cannot keep up.
//!
//! Receive only in this build.

use std::sync::mpsc::{Receiver, Sender, TryRecvError};
use std::time::SystemTime;

use sdroxide_dsp::{FSK441_RATE, MonoResampler};
use sdroxide_types::{Decode, DigiConfig, DigiStatus, Fsk441Period, Mode, QsoStep};

use crate::DigiEngine;
use crate::controller::DigiAction;
use crate::modem::decode_fsk441_slot;
use crate::scheduler::SlotScheduler;

/// One slot of audio handed to the decode worker.
struct DecodeJob {
    audio: Vec<f32>,
    slot_utc: i64,
}

/// What the worker sends back.
struct DecodeResult {
    decodes: Vec<Decode>,
}

pub struct Fsk441Controller {
    cfg: DigiConfig,
    /// The period in force, mirroring `cfg.fsk441_period`; re-read whenever the
    /// config changes, because it decides both the slot length and the decode.
    period: Fsk441Period,
    scheduler: SlotScheduler,
    resampler: Option<MonoResampler>,
    /// 11 025 Hz audio accumulated for the slot in progress.
    slot_buf: Vec<f32>,
    /// Whether `slot_buf` began at a slot boundary. It does not after a start,
    /// a reset or a period change part-way through a slot, and a buffer that
    /// began mid-slot is not the slot the decoder is told it is — so such a
    /// slot is dropped rather than decoded.
    buf_aligned: bool,
    tap_scratch: Vec<f32>,
    last_slot_idx: i64,
    audio_hz: f32,

    job_tx: Sender<DecodeJob>,
    res_rx: Receiver<DecodeResult>,
    _worker: std::thread::JoinHandle<()>,
    pending: bool,
    last_count: u32,
    status_dirty: bool,
}

impl Fsk441Controller {
    pub fn new(cfg: DigiConfig, tap_rate: f64) -> Self {
        let period = cfg.fsk441_period;
        let (job_tx, job_rx) = std::sync::mpsc::channel::<DecodeJob>();
        let (res_tx, res_rx) = std::sync::mpsc::channel::<DecodeResult>();
        let worker = std::thread::Builder::new()
            .name("sdroxide-fsk441-decode".into())
            .spawn(move || {
                while let Ok(job) = job_rx.recv() {
                    let decodes = decode_fsk441_slot(&job.audio, job.slot_utc);
                    if res_tx.send(DecodeResult { decodes }).is_err() {
                        break;
                    }
                }
            })
            .expect("spawn fsk441 decode worker");

        Fsk441Controller {
            cfg,
            period,
            scheduler: SlotScheduler::new(period.slot_s(), period.start_delay_s()),
            resampler: MonoResampler::new(tap_rate, FSK441_RATE),
            slot_buf: Vec::new(),
            buf_aligned: false,
            tap_scratch: Vec::new(),
            last_slot_idx: i64::MIN,
            // FSK441's tones sit 882–2205 Hz; the centre is where the cursor
            // belongs and what the decode list plots.
            audio_hz: 1543.5,
            job_tx,
            res_rx,
            _worker: worker,
            pending: false,
            last_count: 0,
            status_dirty: true,
        }
    }

    fn slot_samples(&self) -> usize {
        (self.period.slot_s() * FSK441_RATE) as usize
    }

    fn digi_status(&self) -> DigiStatus {
        let mut s = DigiStatus::idle(self.cfg.clone());
        s.mode = Mode::Fsk441;
        s.step = QsoStep::Idle;
        s.audio_hz = self.audio_hz;
        s
    }
}

impl DigiEngine for Fsk441Controller {
    fn mode(&self) -> Mode {
        Mode::Fsk441
    }

    fn on_rx_audio(&mut self, tap: &[f32]) {
        self.tap_scratch.clear();
        match &mut self.resampler {
            Some(r) => r.push(tap, &mut self.tap_scratch),
            None => self.tap_scratch.extend_from_slice(tap),
        }
        let cap = self.slot_samples() + self.slot_samples() / 8;
        for &s in &self.tap_scratch {
            if self.slot_buf.len() < cap {
                self.slot_buf.push(s.clamp(-1.0, 1.0));
            }
        }
    }

    fn poll(&mut self, now: SystemTime, _dial_hz: f64) -> Vec<DigiAction> {
        let mut actions = Vec::new();

        // Drain the worker.
        loop {
            match self.res_rx.try_recv() {
                Ok(res) => {
                    self.pending = false;
                    self.last_count = res.decodes.len() as u32;
                    self.status_dirty = true;
                    if !res.decodes.is_empty() {
                        actions.push(DigiAction::Decodes(res.decodes));
                    }
                }
                Err(TryRecvError::Empty) => break,
                // The worker is gone and no answer is coming. Release the slot
                // rather than holding `pending` for the rest of the session,
                // which would stop every later slot being dispatched.
                Err(TryRecvError::Disconnected) => {
                    self.pending = false;
                    break;
                }
            }
        }

        let idx = self.scheduler.slot_index(now);
        if idx != self.last_slot_idx {
            // Only a slot whose audio began on its own boundary is decoded, and
            // half a slot of it is the floor: less than that is a stream
            // hiccup, not a transmission.
            let min_samples = (self.period.slot_s() * FSK441_RATE * 0.5) as usize;
            if self.buf_aligned && self.slot_buf.len() >= min_samples && !self.pending {
                let audio = std::mem::take(&mut self.slot_buf);
                // The slot that just ended is the one before this boundary.
                let slot_utc = self.scheduler.slot_start_unix(idx - 1) as i64;
                self.pending = self.job_tx.send(DecodeJob { audio, slot_utc }).is_ok();
            }
            self.slot_buf.clear();
            // The very first poll is not a boundary crossing, only the first
            // look at the clock — part-way through a slot.
            self.buf_aligned = self.last_slot_idx != i64::MIN;
            self.last_slot_idx = idx;
        }
        if self.status_dirty {
            self.status_dirty = false;
            actions.push(DigiAction::Status(self.digi_status()));
        }
        actions
    }

    fn tx_burst_active(&self) -> bool {
        false
    }

    fn fill_tx_block(&mut self, _out: &mut [f32]) -> bool {
        false
    }

    fn on_burst_done(&mut self) {}

    fn abort(&mut self) {
        self.slot_buf.clear();
        self.buf_aligned = false;
        self.status_dirty = true;
    }

    fn abort_tx(&mut self) {}

    fn set_config(&mut self, cfg: DigiConfig) {
        // A period change moves the slot length, so the scheduler, the buffer
        // bound and the decode all have to follow it. Drop the audio in hand:
        // it belongs to the old geometry and decoding it against the new one
        // would look like a signal that is simply not there.
        if cfg.fsk441_period != self.period {
            self.period = cfg.fsk441_period;
            self.scheduler = SlotScheduler::new(self.period.slot_s(), self.period.start_delay_s());
            self.slot_buf.clear();
            self.buf_aligned = false;
            self.last_slot_idx = i64::MIN;
        }
        self.cfg = cfg;
        self.status_dirty = true;
    }

    fn clear_rx(&mut self) {
        self.last_count = 0;
        self.status_dirty = true;
    }

    fn set_audio_hz(&mut self, hz: f32) {
        self.audio_hz = hz.clamp(200.0, 3500.0);
    }

    fn audio_hz(&self) -> f32 {
        self.audio_hz
    }

    fn status(&self) -> DigiStatus {
        self.digi_status()
    }
}
