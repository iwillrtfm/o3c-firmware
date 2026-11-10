//! `o3c run <config.toml>`: run a command for each key press, knob turn and knob push.
//!
//! A key's LED blinks once when its command ends: green for exit status 0, red otherwise.
//! Commands run in the background; a key whose command still runs ignores new presses.
//! The keypad may be unplugged and plugged in again; `run` waits for it.

use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::thread::{self, sleep};
use std::time::Duration;

use hidapi::HidApi;
use o3c_proto::{self as proto, cmd, event, msg};
use serde::Deserialize;

use crate::{req, Dev, Result};

#[derive(Deserialize)]
struct Config {
    #[serde(default)]
    key: Vec<KeyConfig>,
    #[serde(default)]
    knob: KnobConfig,
}

#[derive(Deserialize)]
struct KeyConfig {
    /// 0, 1, 2: left to right.
    index: u8,
    /// Program and arguments.
    run: Vec<String>,
}

#[derive(Deserialize, Default)]
struct KnobConfig {
    /// One run per click clockwise.
    up: Option<Vec<String>>,
    /// One run per click counter-clockwise.
    down: Option<Vec<String>>,
    press: Option<Vec<String>>,
}

const GREEN: [u8; 3] = [0, 255, 0];
const RED: [u8; 3] = [255, 0, 0];

fn log(s: &str) {
    eprintln!("{s}");
}

pub fn run(api: &mut HidApi, path: &str) -> Result<()> {
    let cfg: Config = toml::from_str(&std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?)
        .map_err(|e| format!("{path}: {e}"))?;
    let cfg = Arc::new(cfg);
    let busy: Arc<[AtomicBool; 3]> = Arc::new([AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false)]);
    loop {
        let Ok(dev) = Dev::open(api) else {
            sleep(Duration::from_secs(2));
            continue;
        };
        log("keypad connected");
        if let Err(e) = serve(&cfg, &dev, &busy) {
            log(&format!("keypad lost: {e}"));
        }
        sleep(Duration::from_secs(1));
    }
}

/// Only this loop touches the keypad: it reads events and sends the blinks that command threads queue.
fn serve(cfg: &Arc<Config>, dev: &Dev, busy: &Arc<[AtomicBool; 3]>) -> Result<()> {
    let (blink_tx, blink_rx) = mpsc::channel::<(u8, [u8; 3])>();
    loop {
        while let Ok((led, rgb)) = blink_rx.try_recv() {
            let mut r = req(cmd::BLINK);
            r[1] = led;
            r[2..5].copy_from_slice(&rgb);
            proto::put_u16(&mut r, 5, 250);
            r[7] = 1;
            dev.send(&r)?;
        }
        let Some(m) = dev.recv(10)? else { continue };
        if m[0] != msg::EVENT {
            continue;
        }
        let value = i16::from_le_bytes([m[3], m[4]]);
        match m[1] {
            event::KEY if value != 0 => {
                let k = m[2];
                let Some(key) = cfg.key.iter().find(|c| c.index == k) else { continue };
                if busy[k as usize % 3].swap(true, Ordering::SeqCst) {
                    continue;
                }
                let (argv, blinks, busy) = (key.run.clone(), blink_tx.clone(), busy.clone());
                thread::spawn(move || {
                    let ok = spawn(&argv, true);
                    busy[k as usize % 3].store(false, Ordering::SeqCst);
                    let _ = blinks.send((k, if ok { GREEN } else { RED }));
                });
            }
            event::KNOB => {
                let argv = if value > 0 { &cfg.knob.up } else { &cfg.knob.down };
                if let Some(argv) = argv {
                    for _ in 0..value.unsigned_abs() {
                        spawn(argv, false);
                    }
                }
            }
            event::KNOB_BUTTON if value != 0 => {
                if let Some(argv) = &cfg.knob.press {
                    spawn(argv, false);
                }
            }
            _ => {}
        }
    }
}

/// Runs argv; with `wait`, returns whether it exited with status 0.
fn spawn(argv: &[String], wait: bool) -> bool {
    let Some((prog, args)) = argv.split_first() else { return false };
    match Command::new(prog).args(args).spawn() {
        Ok(mut child) => {
            if wait {
                child.wait().map(|s| s.success()).unwrap_or(false)
            } else {
                thread::spawn(move || {
                    let _ = child.wait();
                });
                true
            }
        }
        Err(e) => {
            log(&format!("{prog}: {e}"));
            false
        }
    }
}
