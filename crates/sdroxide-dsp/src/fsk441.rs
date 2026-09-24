//! FSK441 — the original high-speed meteor-scatter mode, receive side.
//!
//! Four-tone FSK at 441 baud on 882 / 1323 / 1764 / 2205 Hz, carrying the
//! 43-character PUA-43 alphabet three dits to a character. A meteor leaves a
//! short ionised trail, so the receiver hears pings of ten to a few hundred
//! milliseconds at random points through a 30-second (or 15-second) T/R period;
//! the operator transmits the message over and over and the decoder's job is to
//! catch whatever fragment a trail reflects and read the text out of it.
//!
//! # Why this is not mfsk-core
//!
//! Every other weak-signal mode in `sdroxide-digi` is a feature flag over
//! mfsk-core's WSJT-X port. FSK441 is not in mfsk-core, so this is the fork's
//! own decoder — the 4-FSK front end, the ping search and the alphabet. It
//! follows the published definition (K1JT, *Definition and specification of the
//! FSK441 encoding scheme used in WSJT*, 2001) and the MIT-licensed Rust
//! reference [`Nythbran23/FSK441-PLUS`](https://github.com/Nythbran23/FSK441-PLUS)
//! (© 2026 Roger Banks GW4WND), whose constants and search shape this port
//! keeps: 25 samples a dit at 11 025 Hz, the sliding matched-filter detector,
//! the sample-level sync search, and the "tone 3 never starts a character"
//! framing that recovers the character boundary.
//!
//! # The rate
//!
//! 441 baud is exactly 25 samples at 11 025 Hz — the rate WSJT and MSHV both
//! capture at. The engine's clean tap is 48 kHz, so the controller resamples to
//! [`FSK441_RATE`] before this sees a sample, exactly as the other modes
//! resample to 12 kHz. Do not re-derive the constants at another rate: the
//! NSPD window and the tone spacing are the mode.

use std::f32::consts::TAU;

use num_complex::Complex32;
use rustfft::FftPlanner;

/// The decoder's sample rate: 441 baud × 25 samples a dit, exactly.
pub const FSK441_RATE: f64 = 11_025.0;
/// The symbol rate.
pub const FSK441_BAUD: f32 = 441.0;
/// Samples per dit — one tone interval.
pub const FSK441_NSPD: usize = 25;
/// The four tone centre frequencies, Hz.
pub const FSK441_TONES: [f32; 4] = [882.0, 1323.0, 1764.0, 2205.0];
/// The 48-symbol alphabet, `nc = 16·d0 + 4·d1 + d2`. 43 are used; the four
/// single-tone encodings (`000`/`111`/`222`/`333`) are the shorthand messages,
/// and a few entries are reserved. Duplicate spaces are the reserved slots.
pub const FSK441_CHARSET: &[u8; 48] = b" 123456789.,?/# $ABCD FGHIJKLMNOPQRSTUVWXY 0EZ*!";

/// The single-tone shorthand each all-one-tone ping means, indexed by tone.
pub const FSK441_SHORTHAND: [&str; 4] = ["R26", "R27", "RRR", "73"];

/// Longest dit count one ping may carry; bounds the work and the text.
const MAX_NDITS: usize = 441;
/// Longest message one ping may decode to, in characters.
const MAX_MSG_CHARS: usize = 46;
/// FFT size for the coarse frequency refinement.
const SPEC_NFFT: usize = 256;
/// How far off the nominal tones a signal may be and still be read, Hz.
pub const DEFAULT_DFTOL: f32 = 200.0;

/// One decoded ping.
#[derive(Debug, Clone, PartialEq)]
pub struct Fsk441Ping {
    /// The decoded text. The single-tone shorthand comes out as `R26`, `R27`,
    /// `RRR` or `73`.
    pub text: String,
    /// Frequency offset of the tone set from nominal, Hz.
    pub df_hz: f32,
    /// Where the signal's centre sits in the passband, Hz — the middle of the
    /// four tones plus the offset, which is what the decode list plots.
    pub audio_hz: f32,
    /// Time into the slot the ping starts at, seconds.
    pub start_s: f32,
    /// Ping duration, seconds.
    pub duration_s: f32,
    /// An approximate SNR in dB, the ping's tone energy over the slot's own
    /// noise floor. Not calibrated against a 2500 Hz reference; it orders the
    /// rows rather than reporting a measurement.
    pub snr_db: f32,
    /// Mean dit confidence, 0..1 — how cleanly the four tones separated.
    pub confidence: f32,
}

/// The dit indices for a character, or `None` if it is not in the alphabet.
pub fn fsk441_char_to_dits(c: char) -> Option<(u8, u8, u8)> {
    let uc = c.to_ascii_uppercase();
    let pos = FSK441_CHARSET.iter().position(|&b| b as char == uc)?;
    Some(((pos / 16) as u8, ((pos / 4) % 4) as u8, (pos % 4) as u8))
}

/// The character for three dits. Anything outside the used alphabet reads as a
/// space, which keeps a marginal ping legible instead of filling it with `?`.
pub fn fsk441_dits_to_char(d0: u8, d1: u8, d2: u8) -> char {
    let nc = 16 * d0 as usize + 4 * d1 as usize + d2 as usize;
    FSK441_CHARSET.get(nc).map(|&b| b as char).unwrap_or(' ')
}

/// The tone sequence for a message. A character outside the alphabet becomes a
/// space, exactly as the reference's encoder does.
pub fn fsk441_encode_tones(msg: &str) -> Vec<u8> {
    let mut tones = Vec::with_capacity(msg.len() * 3);
    for c in msg.chars() {
        let (d0, d1, d2) = fsk441_char_to_dits(c).unwrap_or((0, 0, 0));
        tones.push(d0);
        tones.push(d1);
        tones.push(d2);
    }
    tones
}

/// Continuous-phase FSK audio for a tone sequence, at [`FSK441_RATE`],
/// normalised to ±1. A signal *generator*, for tests and bench checks — the
/// fork transmits nothing.
pub fn fsk441_generate_audio(tones: &[u8]) -> Vec<f32> {
    let mut samples = Vec::with_capacity(tones.len() * FSK441_NSPD);
    let dt = 1.0_f32 / FSK441_RATE as f32;
    let mut phase = 0.0f32;
    for &t in tones {
        let dpha = TAU * FSK441_TONES[t.min(3) as usize] * dt;
        for _ in 0..FSK441_NSPD {
            samples.push(phase.sin());
            phase += dpha;
            if phase >= TAU {
                phase -= TAU;
            }
        }
    }
    samples
}

/// The sliding matched-filter energy of one tone over `data`: the squared
/// magnitude of the last `NSPD` samples of the tone, sample by sample.
fn detect(data: &[f32], freq: f32) -> Vec<f32> {
    let npts = data.len();
    if npts < FSK441_NSPD {
        return Vec::new();
    }
    let dpha = TAU * freq / FSK441_RATE as f32;
    let c: Vec<Complex32> = data
        .iter()
        .enumerate()
        .map(|(i, &s)| {
            let a = dpha * i as f32;
            Complex32::new(s * a.cos(), -s * a.sin())
        })
        .collect();

    let mut csum: Complex32 = c[..FSK441_NSPD].iter().sum();
    let mut y = Vec::with_capacity(npts - FSK441_NSPD + 1);
    y.push(csum.norm_sqr());
    for i in 1..npts.saturating_sub(FSK441_NSPD - 1) {
        csum = csum - c[i - 1] + c[i + FSK441_NSPD - 1];
        y.push(csum.norm_sqr());
    }
    y
}

/// The sample offset of the dit grid within the first `NSPD` window.
///
/// Every dit position has one tone dominant and the rest quiet; the sample
/// offset where that contrast is strongest, accumulated over the whole ping,
/// is the boundary. A single-bin DFT over the `NSPD` phase bins interpolates
/// it, which is the reference's `find_sync_phase`.
fn find_sync_phase(y: &[[f32; 4]]) -> usize {
    let mut zf = [0.0f32; FSK441_NSPD];
    for (i, e) in y.iter().enumerate() {
        let best = e.iter().copied().fold(f32::NEG_INFINITY, f32::max);
        let second = e
            .iter()
            .copied()
            .filter(|&v| (v - best).abs() > 1e-30)
            .fold(f32::NEG_INFINITY, f32::max)
            .max(0.0);
        zf[i % FSK441_NSPD] += 1e-6 * (best - second);
    }
    let csum: Complex32 = zf
        .iter()
        .enumerate()
        .map(|(j, &z)| {
            let a = TAU * j as f32 / FSK441_NSPD as f32;
            z * Complex32::new(a.cos(), -a.sin())
        })
        .sum();
    let phase = -csum.im.atan2(csum.re);
    ((FSK441_NSPD as f32 * phase / TAU).round() as i32).rem_euclid(FSK441_NSPD as i32) as usize
}

/// The frequency offset that best aligns the four nominal tones with the
/// signal, searched over ±`dftol` Hz. A meteor ping can be mistuned by a few
/// tens of Hz and every tone would otherwise lose energy.
fn refine_frequency(data: &[f32], dftol: f32) -> f32 {
    let nfft = SPEC_NFFT;
    let nh = nfft / 2;
    let df_bin = FSK441_RATE as f32 / nfft as f32;

    let n_windows = data.len() / nfft;
    if n_windows == 0 {
        return 0.0;
    }
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(nfft);
    let mut s = vec![0.0f32; nh];
    for w in 0..n_windows {
        let start = w * nfft;
        let mut buf: Vec<Complex32> =
            data[start..start + nfft].iter().map(|&x| Complex32::new(x, 0.0)).collect();
        fft.process(&mut buf);
        for i in 0..nh {
            s[i] += buf[i].norm_sqr();
        }
    }
    let fac = 1.0 / (100.0 * nfft as f32 * n_windows as f32);
    s.iter_mut().for_each(|x| *x *= fac);

    let nbaud_bins = (FSK441_BAUD / df_bin).round() as i32;
    let ltone = (FSK441_TONES[0] / FSK441_BAUD) as i32;
    let tol_bins = (dftol / df_bin).round() as i32;
    let wgt = [1.0f32, 4.0, 6.0, 4.0, 1.0];

    let mut smax = 0.0f32;
    let mut best_df = 0.0f32;
    let lo = -(ltone * nbaud_bins);
    let hi = tol_bins;
    for offset_bin in lo..=hi {
        let mut sum = 0.0f32;
        for tone in 0..4i32 {
            let centre = (ltone + tone) * nbaud_bins + offset_bin;
            for (k, &w) in wgt.iter().enumerate() {
                let bin = (centre - 2 + k as i32).clamp(0, nh as i32 - 1) as usize;
                sum += w * s[bin];
            }
        }
        if sum > smax {
            smax = sum;
            best_df = offset_bin as f32 * df_bin;
        }
    }
    best_df
}

/// The hard tone of one dit and how cleanly it separated from the runner-up.
fn soft_dit(e: &[f32; 4]) -> (u8, f32) {
    let mut best_idx = 0usize;
    let mut best = f32::NEG_INFINITY;
    for (i, &v) in e.iter().enumerate() {
        if v > best {
            best = v;
            best_idx = i;
        }
    }
    let second = e
        .iter()
        .enumerate()
        .filter(|&(i, _)| i != best_idx)
        .map(|(_, &v)| v)
        .fold(f32::NEG_INFINITY, f32::max)
        .max(0.0);
    let confidence = if best > 1e-30 { ((best - second) / best).clamp(0.0, 1.0) } else { 0.0 };
    (best_idx as u8, confidence)
}

/// The energy matrix for the four tones at a given frequency offset.
fn energy_matrix(data: &[f32], df: f32) -> Vec<[f32; 4]> {
    let raw: [Vec<f32>; 4] = [
        detect(data, FSK441_TONES[0] + df),
        detect(data, FSK441_TONES[1] + df),
        detect(data, FSK441_TONES[2] + df),
        detect(data, FSK441_TONES[3] + df),
    ];
    let n = raw.iter().map(|e| e.len()).min().unwrap_or(0);
    (0..n).map(|i| [raw[0][i], raw[1][i], raw[2][i], raw[3][i]]).collect()
}

/// The hard tone of every dit in a ping, aligned by the sync search and then
/// trimmed to the dits that actually carry signal.
///
/// The ping search pads the run by a block either side, and a real ping fades
/// in and out, so the matrix has dits of near-zero energy at both ends.
/// Counting those would drag the mean confidence down and add junk characters;
/// the trim is to where a tone carries at least a fifth of the ping's peak.
fn extract_dits(mat: &[[f32; 4]]) -> Vec<(u8, f32)> {
    if mat.is_empty() {
        return Vec::new();
    }
    let jpk = find_sync_phase(mat);
    let n_dits = (mat.len().saturating_sub(jpk) / FSK441_NSPD).min(MAX_NDITS);
    if n_dits < 6 {
        return Vec::new();
    }
    let dits: Vec<[f32; 4]> = (0..n_dits).map(|i| mat[jpk + i * FSK441_NSPD]).collect();

    let best: Vec<f32> =
        dits.iter().map(|e| e.iter().copied().fold(f32::NEG_INFINITY, f32::max)).collect();
    let peak = best.iter().copied().fold(0.0f32, f32::max);
    if peak <= 1e-30 {
        return Vec::new();
    }
    let lo = best.iter().position(|&v| v > 0.2 * peak).unwrap_or(0);
    let hi = best.iter().rposition(|&v| v > 0.2 * peak).map(|i| i + 1).unwrap_or(best.len());
    dits[lo..hi].iter().map(soft_dit).collect()
}

/// The dominant tone of a ping's dits and how many carry it.
fn dominant_tone(dits: &[(u8, f32)]) -> (usize, usize) {
    let mut counts = [0usize; 4];
    for &(t, _) in dits {
        counts[t as usize] += 1;
    }
    counts.iter().enumerate().max_by_key(|&(_, &c)| c).map(|(i, &c)| (i, c)).unwrap_or((0, 0))
}

/// The offset of a single tone's spectral peak from its nominal frequency,
/// searched over ±300 Hz. Used for the shorthand, whose whole signal is one
/// tone and so has no tone set for [`refine_frequency`] to align.
fn tone_offset(data: &[f32], dom: usize) -> f32 {
    let n = data.len().min(4096);
    let data = &data[..n];
    if data.is_empty() {
        return 0.0;
    }
    let nominal = FSK441_TONES[dom];
    let mut best = 0.0f32;
    let mut best_df = 0.0f32;
    let mut f = nominal - 300.0;
    while f <= nominal + 300.0 {
        let dpha = TAU * f / FSK441_RATE as f32;
        let (mut re, mut im) = (0.0f32, 0.0f32);
        for (i, &s) in data.iter().enumerate() {
            let a = dpha * i as f32;
            re += s * a.cos();
            im -= s * a.sin();
        }
        let m = re * re + im * im;
        if m > best {
            best = m;
            best_df = f - nominal;
        }
        f += 5.0;
    }
    best_df
}

/// Decode one ping's audio into its text, frequency offset and confidence.
/// `None` when the ping is too short or too unclear to be a real signal.
fn decode_ping(data: &[f32]) -> Option<(String, f32, f32)> {
    let npts = data.len().min(FSK441_RATE as usize);
    if npts < FSK441_NSPD * 6 {
        return None;
    }
    let data = &data[..npts];

    // The single-tone shorthand first, and at the nominal tones: a whole ping
    // on one tone is `R26`/`R27`/`RRR`/`73`, not text, and the four-tone
    // frequency refine has nothing to align against when only one tone is
    // present. Noise does not concentrate on one tone like this, so a high
    // dominant share is the signal, not a coincidence.
    let nominal = extract_dits(&energy_matrix(data, 0.0));
    if nominal.len() >= 8 {
        let (dom, dom_count) = dominant_tone(&nominal);
        if dom_count as f32 >= 0.9 * nominal.len() as f32 {
            let conf = nominal.iter().map(|d| d.1).sum::<f32>() / nominal.len() as f32;
            return Some((FSK441_SHORTHAND[dom].to_string(), tone_offset(data, dom), conf));
        }
    }

    let df = refine_frequency(data, DEFAULT_DFTOL);
    let soft = extract_dits(&energy_matrix(data, df));
    if soft.is_empty() {
        return None;
    }
    let mean_confidence = soft.iter().map(|d| d.1).sum::<f32>() / soft.len() as f32;

    // Tone 3 never starts a character, so the character boundary is the mod-3
    // phase where tone 3 is least often the dominant tone.
    let mut n4 = [0u32; 3];
    for (i, &(t, _)) in soft.iter().enumerate() {
        if t == 3 {
            n4[i % 3] += 1;
        }
    }
    let jsync = n4.iter().enumerate().min_by_key(|&(_, &c)| c).map(|(i, _)| i).unwrap_or(0);

    let n_chars = ((soft.len().saturating_sub(jsync)) / 3).min(MAX_MSG_CHARS);
    let mut text = String::with_capacity(n_chars);
    for i in 0..n_chars {
        let j = jsync + i * 3;
        if j + 2 >= soft.len() {
            break;
        }
        text.push(fsk441_dits_to_char(soft[j].0, soft[j + 1].0, soft[j + 2].0));
    }
    let text = dedup_spaces(text.trim());
    if text.is_empty() {
        return None;
    }
    Some((text, df, mean_confidence))
}

/// Squeeze runs of spaces to one, so a repeated message reads as words.
fn dedup_spaces(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut prev_space = false;
    for c in s.chars() {
        if c == ' ' {
            if !prev_space {
                out.push(' ');
            }
            prev_space = true;
        } else {
            out.push(c);
            prev_space = false;
        }
    }
    out
}

fn median(v: &mut [f32]) -> f32 {
    if v.is_empty() {
        return 0.0;
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    v[v.len() / 2]
}

/// The ping search: block energy, a median noise floor and a run above it.
///
/// Returns the pings found in one slot, in time order. Each carries its text
/// (empty pings are dropped), where it sat and an approximate SNR. This is the
/// offline form of the reference's streaming detector — the controller hands in
/// a whole slot, so there is no cooldown to keep and every ping in the slot is
/// found in one pass.
pub fn fsk441_find_pings(slot: &[f32]) -> Vec<Fsk441Ping> {
    let npts = slot.len();
    // The detector needs the whole matched filter, and the ping below wants a
    // few dits to be worth decoding.
    if npts < FSK441_NSPD * 8 {
        return Vec::new();
    }

    // A tone-selective envelope: at each sample, the strongest of the four
    // matched filters. A broadband noise burst raises the raw power but not
    // this, which is what keeps the search from chasing noise.
    let mut tone_env = vec![0.0f32; npts - FSK441_NSPD + 1];
    for k in 0..4 {
        let y = detect(slot, FSK441_TONES[k]);
        for (i, &v) in y.iter().enumerate() {
            if v > tone_env[i] {
                tone_env[i] = v;
            }
        }
    }

    // 10 ms blocks, the reference's `ping_bounds` resolution.
    let block = (FSK441_RATE as f32 * 0.010) as usize;
    let n_blocks = tone_env.len() / block;
    if n_blocks < 4 {
        return Vec::new();
    }
    let blocks: Vec<f32> = (0..n_blocks)
        .map(|b| tone_env[b * block..(b + 1) * block].iter().copied().fold(0.0f32, f32::max))
        .collect();

    let mut sorted = blocks.clone();
    let base = median(&mut sorted);
    if base <= 0.0 {
        return Vec::new();
    }
    // 4x the median of a max-of-four statistic is a little over 6 dB clear of
    // the noise floor — high enough that a random block does not trip it, low
    // enough that a real ping does.
    let thresh = base * 4.0;

    let mut pings = Vec::new();
    let mut b = 0usize;
    while b < n_blocks {
        if blocks[b] <= thresh {
            b += 1;
            continue;
        }
        let start_b = b;
        while b < n_blocks && blocks[b] > thresh {
            b += 1;
        }
        let end_b = b;
        // 40 ms is the reference's `wmin`; anything shorter is noise or a
        // fragment too short to carry a character.
        if (end_b - start_b) * block < (FSK441_RATE as f32 * 0.04) as usize {
            continue;
        }
        let start = start_b.saturating_sub(1) * block;
        let end = ((end_b + 2) * block).min(npts);
        if end <= start {
            continue;
        }
        let audio = &slot[start..end];
        let Some((text, df, confidence)) = decode_ping(audio) else {
            continue;
        };
        // A run that tripped the detector but decoded with poor tone contrast
        // is far more likely to be a noise burst than a signal; the reference
        // leaves this to its QSO layer, but a decode list has nowhere to put
        // the doubt, so drop it here.
        if confidence < 0.35 {
            continue;
        }
        let peak = blocks[start_b..end_b].iter().copied().fold(0.0f32, f32::max);
        pings.push(Fsk441Ping {
            text,
            df_hz: df,
            audio_hz: (FSK441_TONES[0] + FSK441_TONES[3]) / 2.0 + df,
            start_s: start as f32 / FSK441_RATE as f32,
            duration_s: (end - start) as f32 / FSK441_RATE as f32,
            snr_db: 10.0 * (peak / base).max(1.0).log10(),
            confidence,
        });
    }
    pings
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The alphabet round-trips, including the two reserved-slot duplicates of
    /// space and the shorthand tones.
    #[test]
    fn the_alphabet_round_trips() {
        assert_eq!(fsk441_char_to_dits(' '), Some((0, 0, 0)));
        assert_eq!(fsk441_char_to_dits('A'), Some((1, 0, 1)));
        assert_eq!(fsk441_char_to_dits('G'), Some((1, 1, 3)));
        assert_eq!(fsk441_char_to_dits('W'), Some((2, 1, 3)));
        assert_eq!(fsk441_char_to_dits('0'), Some((2, 2, 3)));
        assert_eq!(fsk441_char_to_dits('z'), Some((2, 3, 1)), "lowercase folds up");
        assert_eq!(fsk441_dits_to_char(0, 0, 0), ' ');
        assert_eq!(fsk441_dits_to_char(1, 0, 1), 'A');
        assert_eq!(fsk441_dits_to_char(3, 3, 3), ' ');

        for c in "ABCXYZ 019.,?/#$*!".chars() {
            let (d0, d1, d2) = fsk441_char_to_dits(c).unwrap();
            assert_eq!(fsk441_dits_to_char(d0, d1, d2), c, "{c}");
        }
    }

    /// The encoder emits three dits a character, in the alphabet's order.
    #[test]
    fn the_encoder_emits_three_dits_a_character() {
        assert_eq!(fsk441_encode_tones("G"), vec![1, 1, 3]);
        assert_eq!(fsk441_encode_tones(" "), vec![0, 0, 0]);
        assert_eq!(fsk441_encode_tones("CQ").len(), 6);
        // A character outside the alphabet becomes a space rather than panicking.
        assert_eq!(fsk441_encode_tones("~"), vec![0, 0, 0]);
    }

    /// A clean single pass of a real message decodes back to itself. This is
    /// the whole chain — ping search, frequency refine, matched filter, sync,
    /// framing and alphabet — over the fork's own generator.
    #[test]
    fn a_clean_ping_decodes_to_its_message() {
        let msg = "CQ DE W1ABC";
        let tones = fsk441_encode_tones(msg);
        let mut slot = vec![0.0f32; 2205]; // 0.2 s of silence
        slot.extend_from_slice(&fsk441_generate_audio(&tones));
        slot.extend(std::iter::repeat(0.0).take(2205));

        let pings = fsk441_find_pings(&slot);
        assert_eq!(pings.len(), 1, "one ping expected, got {pings:?}");
        assert_eq!(pings[0].text, msg);
        assert!(pings[0].confidence > 0.9, "clean signal should be confident");
        // The ping starts where the signal does, within a block.
        assert!((pings[0].start_s - 0.2).abs() < 0.05, "start {}", pings[0].start_s);
    }

    /// A mistuned signal is still found and decoded: the frequency refine has
    /// to absorb an offset, which is what a real ping arrives with.
    #[test]
    fn a_mistuned_ping_is_refined_and_decoded() {
        // Shift the generator's tones by +60 Hz by resynthesising here rather
        // than re-deriving the decoder's constants.
        let msg = "CQ TEST";
        let tones = fsk441_encode_tones(msg);
        let dt = 1.0 / FSK441_RATE as f32;
        let mut phase = 0.0f32;
        let mut audio = Vec::new();
        for &t in &tones {
            let dpha = TAU * (FSK441_TONES[t as usize] + 60.0) * dt;
            for _ in 0..FSK441_NSPD {
                audio.push(phase.sin());
                phase += dpha;
                if phase >= TAU {
                    phase -= TAU;
                }
            }
        }
        let mut slot = vec![0.0f32; 2205];
        slot.extend_from_slice(&audio);
        slot.extend(std::iter::repeat(0.0).take(2205));

        let pings = fsk441_find_pings(&slot);
        assert_eq!(pings.len(), 1, "{pings:?}");
        assert_eq!(pings[0].text, msg);
        assert!((pings[0].df_hz - 60.0).abs() < 30.0, "df {}", pings[0].df_hz);
    }

    /// A pure single tone is the shorthand, not text. `RRR` (tone 2) and `73`
    /// (tone 3) are the two that are not characters in their own right.
    #[test]
    fn a_single_tone_ping_is_the_shorthand() {
        for (tone, want) in [(0usize, "R26"), (1, "R27"), (2, "RRR"), (3, "73")] {
            let tones = vec![tone as u8; 30];
            let mut slot = vec![0.0f32; 2205];
            slot.extend_from_slice(&fsk441_generate_audio(&tones));
            slot.extend(std::iter::repeat(0.0).take(2205));
            let pings = fsk441_find_pings(&slot);
            assert_eq!(pings.len(), 1, "tone {tone}: {pings:?}");
            assert_eq!(pings[0].text, want, "tone {tone}");
        }
    }

    /// A weak ping behind noise still decodes: the detector and the confidence
    /// gate have to leave room for the sort of level meteor scatter delivers.
    #[test]
    fn a_weak_ping_still_decodes() {
        let msg = "W1ABC W9XYZ";
        let audio = fsk441_generate_audio(&fsk441_encode_tones(msg));
        let npts = FSK441_RATE as usize;
        let start = npts / 4;
        let mut slot = vec![0.0f32; npts];
        let mut state = 0x1234_abcdu32;
        for s in slot.iter_mut() {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            *s = ((state >> 8) as f32 / 8_388_608.0 - 1.0) * 0.45;
        }
        for (i, &s) in audio.iter().enumerate() {
            if start + i < npts {
                slot[start + i] += s * 0.6;
            }
        }
        let pings = fsk441_find_pings(&slot);
        assert!(pings.iter().any(|p| p.text == msg), "weak ping not decoded: {pings:?}");
    }

    /// Noise alone must not produce a decode: the confidence and tone-shape
    /// gates are what keep the list honest.
    #[test]
    fn noise_alone_does_not_decode() {
        // A deterministic pseudo-random sequence, so the test cannot flake.
        let mut state = 0x1234_5678u32;
        let mut slot = Vec::with_capacity(55_125);
        for _ in 0..55_125 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            slot.push(((state >> 8) as f32 / 8_388_608.0) - 1.0);
        }
        let pings = fsk441_find_pings(&slot);
        assert!(pings.is_empty(), "noise decoded as {pings:?}");
    }

    /// The text out of a very short fragment is a fragment, not a crash: the
    /// decoder must return what it can and never panic on partial dits.
    #[test]
    fn a_fragment_decodes_without_panicking() {
        let tones = fsk441_encode_tones("W1ABC");
        let audio = fsk441_generate_audio(&tones);
        let _ = decode_ping(&audio[..audio.len() / 2]);
        let _ = fsk441_find_pings(&audio);
    }
}
