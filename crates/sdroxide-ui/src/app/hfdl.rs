//! The HFDL ground-network decoder's window.
//!
//! The choice the other not-a-mode lanes put in a window — enable, tune, then
//! read the decode log — but for one channel instead of a sketch map: HFDL
//! ground stations are spread across the shortwave band one channel each, so
//! the panel is a frequency, a run switch, and the rolling log of what the
//! channel decoded. State comes from [`sdroxide_types::HfdlStatus`] (the
//! engine's re-sent snapshot), settings from [`sdroxide_types::HfdlSettings`]
//! edited as a copy and diffed at the end — the same apply-by-diff convention
//! the AIS and ADS-B windows keep, because the engine echoes the accepted
//! config back and the two copies would otherwise drift.

use eframe::egui::{self, RichText};
use sdroxide_types::{Command, HfdlDecode, HfdlStatus};

use crate::app::SdroxideApp;

/// The HFDL assigned-frequency plan, in kHz — the channels the ground-network
/// stations transmit on, ARINC/AviaSat's published allocation. Riverhead's
/// 21 931 kHz primary is first: it is the default channel and the reference
/// off-air capture that validated the decoder was recorded on it.
const HFDL_PLAN_KHZ: &[u32] = &[
    21_931,
    2_941,
    2_992,
    3_455,
    3_917,
    5_451,
    6_562,
    6_640,
    8_843,
    10_060,
    10_084,
    11_184,
    11_387,
    13_306,
    13_309,
    13_312,
    13_315,
    13_318,
    17_904,
    17_910,
    17_913,
    17_916,
    17_919,
    17_922,
    17_925,
    21_934,
    21_940,
];

/// The HFDL window: run switch, channel, and the decode log.
impl SdroxideApp {
    pub(in crate::app) fn hfdl_window(&mut self, ctx: &egui::Context, cmds: &mut Vec<Command>) {
        if !self.show_hfdl {
            return;
        }
        let mut open = self.show_hfdl;
        // Edited as a copy and diffed at the end, the way the AIS window does: the
        // engine echoes the accepted settings back, so there is no apply step and
        // no way for the two copies to drift.
        let mut cfg = self.state.hfdl;
        let resp = egui::Window::new("HFDL")
            .id(crate::layout::salted_id(ctx, "HfdlWindow"))
            .open(&mut open)
            .frame(crate::chrome::window_frame())
            .resizable(true)
            .default_width(crate::layout::window_w(ctx, 480.0))
            .default_height(crate::layout::window_h(ctx, 420.0))
            .show(ctx, |ui| {
                crate::chrome::window_body_bg(ui);
                hfdl_status_strip(ui, self.hfdl_status.as_ref());
                ui.add_space(4.0);

                ui.horizontal(|ui| {
                    if crate::chrome::chip_enabled(ui, true, cfg.enabled, "LISTEN")
                        .on_hover_text(
                            "Switches the engine's HFDL downconverter and decoder on or \
                             off. Decoding costs a 24 kHz lane and a worker thread whether \
                             the window is open or not.",
                        )
                        .clicked()
                    {
                        cfg.enabled = !cfg.enabled;
                    }
                    let mut khz = (cfg.frequency_hz / 1e3).round();
                    let old_khz = khz;
                    ui.add(
                        egui::DragValue::new(&mut khz)
                            .range(2_800.0..=30_000.0)
                            .speed(1.0)
                            .suffix(" kHz"),
                    )
                    .on_hover_text(
                        "The channel (the assigned frequency) to listen on. Anything in the \
                         band works — HFDL stations transmit on 2.8 to 22 MHz — against a \
                         fixed 24 kHz lane centred here.",
                    );
                    if khz != old_khz {
                        cfg.frequency_hz = khz * 1e3;
                    }
                });
                ui.add_space(4.0);

                // The plan as pick chips, so a spot of a few kHz on the band is a
                // single click away rather than a DragValue spin.
                ui.horizontal_wrapped(|ui| {
                    for &khz in HFDL_PLAN_KHZ {
                        let on = (cfg.frequency_hz / 1e3).round() as u32 == khz;
                        if crate::chrome::chip(ui, on, format!("{:.3} M", khz as f32 / 1e3))
                            .on_hover_text(if khz == 21_931 {
                                "Riverhead (northern Atlantic/north America) — the default, \
                                 and the channel the decoder was validated on."
                            } else {
                                "An HFDL assigned frequency."
                            })
                            .clicked()
                        {
                            cfg.frequency_hz = f64::from(khz) * 1e3;
                        }
                    }
                });

                ui.separator();
                ui.label(
                    RichText::new("DECODES")
                        .size(9.5)
                        .strong()
                        .color(crate::theme::CYAN_DIM()),
                );
                // The log is carried whole in every status snapshot, oldest *last*.
                let log =
                    self.hfdl_status.as_ref().map(|s| s.log.as_slice()).unwrap_or(&[]);
                egui::ScrollArea::vertical()
                    .id_salt("hfdl-log")
                    .auto_shrink([false, false])
                    .max_height(ui.available_height() - 6.0)
                    .show(ui, |ui| {
                        if log.is_empty() {
                            ui.label(
                                RichText::new("Nothing decoded yet — a ground station's squitter \
                                               repeats every ~32 s once one is in the channel.")
                                    .size(10.5)
                                    .color(crate::theme::gray(120)),
                            );
                        }
                        for d in log.iter().rev() {
                            hfdl_log_row(ui, d);
                        }
                    });
            });
        if let Some(r) = &resp {
            crate::chrome::paint_window_border(ctx, &r.response);
        }
        if cfg != self.state.hfdl {
            cmds.push(Command::SetHfdlConfig(cfg));
        }
        self.show_hfdl = open;
    }
}

/// The decoder's own state, as four fixed-width slots so nothing reflows the
/// header when a number grows a digit.
fn hfdl_status_strip(ui: &mut egui::Ui, status: Option<&HfdlStatus>) {
    let (running, level, bursts, decodes) = match status {
        Some(s) => (s.running, Some(s.level_dbfs), s.bursts, s.decodes),
        None => (false, None, 0, 0),
    };
    let (run_text, run_ink) = if running {
        ("RUNNING", crate::theme::GREEN())
    } else {
        ("OFF", crate::theme::gray(150))
    };
    slot(ui, 84.0, run_text, run_ink);
    let level_text = match level {
        Some(l) if l > -90.0 => format!("{l:.0} dBFS"),
        _ => "— dBFS".to_string(),
    };
    slot(ui, 76.0, &level_text, crate::theme::gray(150));
    slot(ui, 64.0, &format!("{} bursts", count(bursts)), crate::theme::CYAN());
    slot(ui, 64.0, &format!("{} decodes", count(decodes)), crate::theme::gray(120));
}

/// One decoded event, as a row of the log: time, kind, the ground station it
/// names, the channel and the burst's signal report.
fn hfdl_log_row(ui: &mut egui::Ui, d: &HfdlDecode) {
    ui.horizontal_wrapped(|ui| {
        ui.label(
            RichText::new(crate::time::utc_clock(d.unix))
                .size(10.5)
                .color(crate::theme::gray(120)),
        );
        ui.label(RichText::new(&d.kind).size(10.5).color(crate::theme::CYAN()));
        if let Some(gs) = &d.gs {
            ui.label(RichText::new(gs).size(10.5).strong());
        }
        ui.label(
            RichText::new(format!("{:.3} MHz", d.freq_khz as f32 / 1e3))
                .size(10.5)
                .color(crate::theme::gray(150)),
        );
        if let Some(snr) = d.snr_db {
            ui.label(
                RichText::new(format!("{snr:.0} dB"))
                    .size(10.5)
                    .color(if snr >= 12.0 { crate::theme::GREEN() } else { crate::theme::YELLOW() }),
            );
        }
        if let Some(fec) = d.fec_corrected {
            if fec > 0 {
                ui.label(
                    RichText::new(format!("{} fixed", count(u64::from(fec))))
                        .size(10.5)
                        .color(crate::theme::gray(120)),
                );
            }
        }
        if !d.details.is_empty() && d.details != "null" {
            ui.label(
                RichText::new(d.details.replace('"', ""))
                    .size(10.0)
                    .color(crate::theme::gray(120)),
            );
        }
    });
}

/// A counter, short enough that it cannot outgrow its slot.
fn count(n: u64) -> String {
    if n < 10_000 {
        n.to_string()
    } else if n < 995_000 {
        format!("{:.0}k", n as f64 / 1e3)
    } else if n < 999_500_000 {
        format!("{:.1}M", n as f64 / 1e6)
    } else {
        format!("{:.2}G", n as f64 / 1e9)
    }
}

/// A readout in a slot of fixed width, so a number that grows a digit cannot
/// re-flow the header.
fn slot(ui: &mut egui::Ui, w: f32, text: &str, color: egui::Color32) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(w, 16.0), egui::Sense::hover());
    if !ui.is_rect_visible(rect) {
        return;
    }
    ui.painter_at(rect).text(
        egui::pos2(rect.left(), rect.center().y),
        egui::Align2::LEFT_CENTER,
        text,
        egui::FontId::monospace(10.5),
        color,
    );
}

/// The window's hot-path helper is exerciseable without a window: rendering
/// concerns belong to egui, but the row text is a pure function of the decode.
fn decode_row(d: &HfdlDecode) -> String {
    let mut parts = vec![crate::time::utc_clock(d.unix), d.kind.clone()];
    if let Some(gs) = &d.gs {
        parts.push(gs.clone());
    }
    parts.push(format!("{:.3}", d.freq_khz as f32 / 1e3));
    if let Some(snr) = d.snr_db {
        parts.push(format!("{snr:.0}"));
    }
    parts.join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;
    use sdroxide_types::HFDL_DEFAULT_HZ;

    fn dec(kind: &str, gs: Option<&str>) -> HfdlDecode {
        HfdlDecode {
            unix: 1_700_000_000,
            kind: kind.to_string(),
            gs: gs.map(|s| s.to_string()),
            freq_khz: 21_931,
            snr_db: Some(14.5),
            freq_skew_hz: None,
            fec_corrected: None,
            details: r#"{"aircraft":"A9C-DM"}"#.to_string(),
        }
    }

    #[test]
    fn row_carries_kind_then_ground_station() {
        let row = decode_row(&dec("squitter", Some("Riverhead (GS 4)")));
        assert!(row.contains("squitter"));
        assert!(row.contains("Riverhead (GS 4)"));
        assert!(row.starts_with(&crate::time::utc_clock(1_700_000_000)));
    }

    #[test]
    fn default_channel_is_in_the_plan() {
        assert!(HFDL_PLAN_KHZ.contains(&(HFDL_DEFAULT_HZ as u32 / 1000)));
    }
}