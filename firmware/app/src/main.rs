//! O3C app firmware. Starts at 0x8000, after the bootloader.
//!
//! USB: composite HID device:
//! - Interface 0: vendor HID for the `o3c` host tool (key/knob events, raw sensor stream,
//!   LEDs, bootloader entry).
//! - Interface 1: native USB HID keyboard (Report ID 1) + consumer media keys (Report ID 2),
//!   modeled after RMK's report engine so the keypad works standalone with minimal latency.
//!
//! One polling loop. Only the LED matrix runs from an interrupt (TIM2).
#![no_std]
#![no_main]

mod input;
mod leds;
mod panic;

use hw::flash::{PageWriter, WriteError};
use hw::gpio;
use o3c_proto::{self as proto, cmd, event, msg, status};
use usbhs::{Descriptors, Interface, Usb};

use hid::HidEngine;
use input::{Key, Knob, KEYS};

const VERSION: u16 = 9;

/// Vendor HID endpoints (Interface 0).
const EP_IN: u8 = 2;
const EP_OUT: u8 = 3;
/// Native Keyboard + Consumer Control HID endpoint (Interface 1).
const EP_KBD_IN: u8 = 1;

/// Optional vendor opcode to mute/unmute native HID (`req[1]`: 0 = disable, 1 = enable).
const CMD_HID_ENABLE: u8 = 0x70;

const VENDOR_REPORT: &[u8] = &[
    0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x40, 0x09, 0x02,
    0x81, 0x02, 0x95, 0x40, 0x09, 0x03, 0x91, 0x02, 0xC0,
];

/// Composite HID report descriptor (RMK-style Report ID multiplexing):
/// - Report ID 1: Standard 6KRO Keyboard (1 + 8 = 9 bytes IN)
/// - Report ID 2: Consumer Control / Media Keys (1 + 2 = 3 bytes IN)
const KBD_REPORT: &[u8] = &[
    // Report ID 1: Keyboard
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x06, // Usage (Keyboard)
    0xA1, 0x01, // Collection (Application)
    0x85, 0x01, //   Report ID (1)
    0x05, 0x07, //   Usage Page (Keyboard/Keypad)
    0x19, 0xE0, //   Usage Minimum (Left Control)
    0x29, 0xE7, //   Usage Maximum (Right GUI)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x08, //   Report Count (8)
    0x81, 0x02, //   Input (Data, Variable, Absolute) - Modifiers
    0x95, 0x01, //   Report Count (1)
    0x75, 0x08, //   Report Size (8)
    0x81, 0x01, //   Input (Constant) - Reserved
    0x95, 0x06, //   Report Count (6)
    0x75, 0x08, //   Report Size (8)
    0x15, 0x00, //   Logical Minimum (0)
    0x26, 0xFF, 0x00, // Logical Maximum (255)
    0x05, 0x07, //   Usage Page (Keyboard/Keypad)
    0x19, 0x00, //   Usage Minimum (0)
    0x29, 0xFF, //   Usage Maximum (255)
    0x81, 0x00, //   Input (Data, Array, Absolute) - Keycodes
    0xC0,       // End Collection
    // Report ID 2: Consumer Control
    0x05, 0x0C, // Usage Page (Consumer)
    0x09, 0x01, // Usage (Consumer Control)
    0xA1, 0x01, // Collection (Application)
    0x85, 0x02, //   Report ID (2)
    0x15, 0x00, //   Logical Minimum (0)
    0x26, 0xFF, 0x03, // Logical Maximum (0x03FF)
    0x19, 0x00, //   Usage Minimum (0)
    0x2A, 0xFF, 0x03, // Usage Maximum (0x03FF)
    0x75, 0x10, //   Report Size (16)
    0x95, 0x01, //   Report Count (1)
    0x81, 0x00, //   Input (Data, Array, Absolute)
    0xC0,       // End Collection
];

const DEVICE: &[u8] = &[
    18, 1, 0x00, 0x02, 0, 0, 0, 64,
    (proto::VID & 0xFF) as u8, (proto::VID >> 8) as u8, (proto::PID & 0xFF) as u8, (proto::PID >> 8) as u8,
    0x00, 0x02, 1, 2, 0, 1,
];

// Total length = 9 (Config) + 32 (Interface 0: Vendor HID) + 25 (Interface 1: Keyboard HID) = 66 bytes
const CONFIG: &[u8] = &[
    9, 2, 66, 0, 2, 1, 0, 0x80, 50, // Configuration (2 interfaces, 66 bytes total)
    // Interface 0: Vendor HID (hid_desc_offset = 18)
    9, 4, 0, 0, 2, 3, 0, 0, 0, //
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, VENDOR_REPORT.len() as u8, 0, //
    7, 5, 0x80 | EP_IN, 3, 64, 0, 1, //
    7, 5, EP_OUT, 3, 64, 0, 1, //
    // Interface 1: Keyboard + Consumer HID (hid_desc_offset = 50)
    9, 4, 1, 0, 1, 3, 0, 0, 0, //
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, KBD_REPORT.len() as u8, 0, //
    7, 5, 0x80 | EP_KBD_IN, 3, 16, 0, 1, // 1000 Hz FS / 8000 Hz HS (bInterval = 1)
];

static DESC: Descriptors = Descriptors {
    device: DEVICE,
    config: CONFIG,
    interfaces: &[
        Interface { report: VENDOR_REPORT, hid_desc_offset: 18 },
        Interface { report: KBD_REPORT, hid_desc_offset: 50 },
    ],
    strings: &[&[4, 3, 0x09, 0x04], usbhs::string_descriptor!("o3c"), usbhs::string_descriptor!("O3C")],
    in_eps: &[EP_IN, EP_KBD_IN],
    out_eps: &[EP_OUT],
};

/// Low-latency single-layer USB HID keyboard & consumer control engine.
mod hid {
    use super::{Usb, EP_KBD_IN, KEYS};

    const REPORT_ID_KBD: u8 = 1;
    const REPORT_ID_CONSUMER: u8 = 2;

    /// Standard USB HID Keyboard/Keypad Usage IDs (Usage Page 0x07).
    #[allow(dead_code)]
    #[repr(u8)]
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub enum KeyCode {
        No = 0x00,
        A = 0x04,
        B = 0x05,
        C = 0x06,
        D = 0x07,
        E = 0x08,
        F = 0x09,
        G = 0x0A,
        H = 0x0B,
        I = 0x0C,
        J = 0x0D,
        K = 0x0E,
        L = 0x0F,
        M = 0x10,
        N = 0x11,
        O = 0x12,
        P = 0x13,
        Q = 0x14,
        R = 0x15,
        S = 0x16,
        T = 0x17,
        U = 0x18,
        V = 0x19,
        W = 0x1A,
        X = 0x1B,
        Y = 0x1C,
        Z = 0x1D,
        Kc1 = 0x1E,
        Kc2 = 0x1F,
        Kc3 = 0x20,
        Kc4 = 0x21,
        Kc5 = 0x22,
        Kc6 = 0x23,
        Kc7 = 0x24,
        Kc8 = 0x25,
        Kc9 = 0x26,
        Kc0 = 0x27,
        Enter = 0x28,
        Escape = 0x29,
        Backspace = 0x2A,
        Tab = 0x2B,
        Space = 0x2C,
        F1 = 0x3A,
        F2 = 0x3B,
        F3 = 0x3C,
        F4 = 0x3D,
        F5 = 0x3E,
        F6 = 0x3F,
        F7 = 0x40,
        F8 = 0x41,
        F9 = 0x42,
        F10 = 0x43,
        F11 = 0x44,
        F12 = 0x45,
        Right = 0x4F,
        Left = 0x50,
        Down = 0x51,
        Up = 0x52,
        LCtrl = 0xE0,
        LShift = 0xE1,
        LAlt = 0xE2,
        LGui = 0xE3,
        RCtrl = 0xE4,
        RShift = 0xE5,
        RAlt = 0xE6,
        RGui = 0xE7,
    }

    /// USB HID Modifier bitmask.
    #[allow(dead_code)]
    pub struct Modifier;
    #[allow(dead_code)]
    impl Modifier {
        pub const NONE: u8 = 0;
        pub const LCTRL: u8 = 1 << 0;
        pub const LSHIFT: u8 = 1 << 1;
        pub const LALT: u8 = 1 << 2;
        pub const LGUI: u8 = 1 << 3;
        pub const RCTRL: u8 = 1 << 4;
        pub const RSHIFT: u8 = 1 << 5;
        pub const RALT: u8 = 1 << 6;
        pub const RGUI: u8 = 1 << 7;
    }

    /// USB HID Consumer Control Usage IDs (Usage Page 0x0C).
    #[allow(dead_code)]
    #[repr(u16)]
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub enum Consumer {
        Mute = 0x00E2,
        VolUp = 0x00E9,
        VolDown = 0x00EA,
        PlayPause = 0x00CD,
        NextTrack = 0x00B5,
        PrevTrack = 0x00B6,
    }

    /// Direct single-layer key/encoder action binding.
    #[allow(dead_code)]
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub enum KeyAction {
        No,
        Key(KeyCode),
        Chord(u8, KeyCode),
        Consumer(Consumer),
    }

    pub struct Keymap {
        pub keys: [KeyAction; KEYS],
        pub knob_cw: KeyAction,
        pub knob_ccw: KeyAction,
        pub knob_sw: KeyAction,
    }

    /// Static keymap: [Z, X, C], Knob = VolDown / VolUp, Knob Push = Mute.
    pub const KEYMAP: Keymap = Keymap {
        keys: [
            KeyAction::Key(KeyCode::Z),
            KeyAction::Key(KeyCode::X),
            KeyAction::Key(KeyCode::C),
        ],
        knob_cw: KeyAction::Consumer(Consumer::VolUp),
        knob_ccw: KeyAction::Consumer(Consumer::VolDown),
        knob_sw: KeyAction::Consumer(Consumer::Mute),
    };

    /// Small ring buffer for edge-triggered consumer media packets (press + release pulses).
    struct ConsumerQueue {
        buf: [[u8; 3]; 16],
        head: usize,
        len: usize,
    }

    impl ConsumerQueue {
        const fn new() -> Self {
            Self {
                buf: [[REPORT_ID_CONSUMER, 0, 0]; 16],
                head: 0,
                len: 0,
            }
        }

        #[inline]
        fn push(&mut self, pkt: [u8; 3]) {
            if self.len < self.buf.len() {
                let idx = (self.head + self.len) % self.buf.len();
                self.buf[idx] = pkt;
                self.len += 1;
            }
        }

        #[inline]
        fn front(&self) -> Option<&[u8; 3]> {
            (self.len > 0).then(|| &self.buf[self.head])
        }

        #[inline]
        fn pop(&mut self) {
            if self.len > 0 {
                self.head = (self.head + 1) % self.buf.len();
                self.len -= 1;
            }
        }

        fn clear(&mut self) {
            self.head = 0;
            self.len = 0;
        }
    }

    /// Tracks active keys and reference-counted modifiers, coalescing keyboard state
    /// into a single latest-state report for zero-queue-delay transmission.
    pub struct HidEngine {
        pub enabled: bool,
        mod_counts: [u8; 8],
        pressed_keys: [u8; 6],
        kbd_report: [u8; 9],
        kbd_dirty: bool,
        consumer_queue: ConsumerQueue,
    }

    impl HidEngine {
        pub const fn new() -> Self {
            Self {
                enabled: true,
                mod_counts: [0; 8],
                pressed_keys: [0; 6],
                kbd_report: [REPORT_ID_KBD, 0, 0, 0, 0, 0, 0, 0, 0],
                kbd_dirty: false,
                consumer_queue: ConsumerQueue::new(),
            }
        }

        pub fn reset(&mut self) {
            self.mod_counts = [0; 8];
            self.pressed_keys = [0; 6];
            self.kbd_report = [REPORT_ID_KBD, 0, 0, 0, 0, 0, 0, 0, 0];
            self.kbd_dirty = false;
            self.consumer_queue.clear();
        }

        /// Immediately flushes the latest keyboard state (highest priority) or next
        /// consumer media report to `EP_KBD_IN`.
        #[inline]
        pub fn flush(&mut self, usb: &mut Usb) {
            if !usb.configured() {
                return;
            }
            if self.kbd_dirty {
                if usb.write(EP_KBD_IN, &self.kbd_report) {
                    self.kbd_dirty = false;
                }
                return;
            }
            if let Some(pkt) = self.consumer_queue.front() {
                let pkt = *pkt;
                if usb.write(EP_KBD_IN, &pkt) {
                    self.consumer_queue.pop();
                }
            }
        }

        #[inline]
        pub fn on_key(&mut self, index: usize, down: bool) {
            if !self.enabled || index >= KEYS {
                return;
            }
            let action = KEYMAP.keys[index];
            if down {
                self.press_action(action);
            } else {
                self.release_action(action);
            }
        }

        #[inline]
        pub fn on_knob_detent(&mut self, detent: i8) {
            if !self.enabled || detent == 0 {
                return;
            }
            let action = if detent > 0 { KEYMAP.knob_cw } else { KEYMAP.knob_ccw };
            let steps = detent.unsigned_abs().min(8);
            for _ in 0..steps {
                self.press_action(action);
                self.release_action(action);
            }
        }

        #[inline]
        pub fn on_knob_switch(&mut self, down: bool) {
            if !self.enabled {
                return;
            }
            if down {
                self.press_action(KEYMAP.knob_sw);
            } else {
                self.release_action(KEYMAP.knob_sw);
            }
        }

        #[inline]
        fn press_action(&mut self, action: KeyAction) {
            match action {
                KeyAction::No => {}
                KeyAction::Key(kc) => {
                    self.register_usage(kc as u8);
                    self.sync_kbd_report();
                }
                KeyAction::Chord(mods, kc) => {
                    self.register_mod_mask(mods);
                    self.register_usage(kc as u8);
                    self.sync_kbd_report();
                }
                KeyAction::Consumer(usage) => {
                    let bytes = (usage as u16).to_le_bytes();
                    self.consumer_queue.push([REPORT_ID_CONSUMER, bytes[0], bytes[1]]);
                }
            }
        }

        #[inline]
        fn release_action(&mut self, action: KeyAction) {
            match action {
                KeyAction::No => {}
                KeyAction::Key(kc) => {
                    self.unregister_usage(kc as u8);
                    self.sync_kbd_report();
                }
                KeyAction::Chord(mods, kc) => {
                    self.unregister_usage(kc as u8);
                    self.unregister_mod_mask(mods);
                    self.sync_kbd_report();
                }
                KeyAction::Consumer(_) => {
                    self.consumer_queue.push([REPORT_ID_CONSUMER, 0, 0]);
                }
            }
        }

        #[inline]
        fn register_mod_mask(&mut self, mask: u8) {
            for bit in 0..8 {
                if mask & (1 << bit) != 0 {
                    self.mod_counts[bit] = self.mod_counts[bit].saturating_add(1);
                }
            }
        }

        #[inline]
        fn unregister_mod_mask(&mut self, mask: u8) {
            for bit in 0..8 {
                if mask & (1 << bit) != 0 {
                    self.mod_counts[bit] = self.mod_counts[bit].saturating_sub(1);
                }
            }
        }

        #[inline]
        fn register_usage(&mut self, code: u8) {
            if code == 0 {
                return;
            }
            if (0xE0..=0xE7).contains(&code) {
                let idx = (code - 0xE0) as usize;
                self.mod_counts[idx] = self.mod_counts[idx].saturating_add(1);
                return;
            }
            if self.pressed_keys.contains(&code) {
                return;
            }
            if let Some(slot) = self.pressed_keys.iter_mut().find(|k| **k == 0) {
                *slot = code;
            }
        }

        #[inline]
        fn unregister_usage(&mut self, code: u8) {
            if code == 0 {
                return;
            }
            if (0xE0..=0xE7).contains(&code) {
                let idx = (code - 0xE0) as usize;
                self.mod_counts[idx] = self.mod_counts[idx].saturating_sub(1);
                return;
            }
            for slot in self.pressed_keys.iter_mut() {
                if *slot == code {
                    *slot = 0;
                }
            }
        }

        #[inline]
        fn sync_kbd_report(&mut self) {
            let mut mods = 0u8;
            for bit in 0..8 {
                if self.mod_counts[bit] > 0 {
                    mods |= 1 << bit;
                }
            }
            let mut report = [REPORT_ID_KBD, mods, 0, 0, 0, 0, 0, 0, 0];
            report[3..9].copy_from_slice(&self.pressed_keys);
            if report != self.kbd_report {
                self.kbd_report = report;
                self.kbd_dirty = true;
            }
        }
    }
}

struct State {
    keys: [Key; KEYS],
    /// Fixed-point Q14.2 EWMA accumulators (`filtered = adc_ewma >> 2`) for Hall sensors.
    adc_ewma: [u32; KEYS],
    pins: u8,
    stream: bool,
    led_override: Option<[[u8; 3]; 3]>,
    brightness: u8,
    /// End of the start-up LED flash (ms).
    hello_until: u64,
    hello_done: bool,
    /// Per LED: (r, g, b), start ms, on/off period ms, blinks.
    blink: [Option<([u8; 3], u64, u16, u8)>; 3],
    reset_at: Option<u64>,
    boot: PageWriter,
    hid: HidEngine,
}

/// Small ring of reports for the vendor IN endpoint.
struct Queue {
    buf: [[u8; 64]; 16],
    head: usize,
    len: usize,
}

impl Queue {
    #[inline]
    fn push(&mut self, r: [u8; 64]) {
        if self.len < self.buf.len() {
            let i = (self.head + self.len) % self.buf.len();
            self.buf[i] = r;
            self.len += 1;
        }
    }
    #[inline]
    fn front(&self) -> Option<&[u8; 64]> {
        (self.len > 0).then(|| &self.buf[self.head])
    }
    #[inline]
    fn pop(&mut self) {
        if self.len > 0 {
            self.head = (self.head + 1) % self.buf.len();
            self.len -= 1;
        }
    }
}

#[inline]
fn event_report(kind: u8, index: u8, value: i16) -> [u8; 64] {
    let mut r = [0u8; 64];
    r[0] = msg::EVENT;
    r[1] = kind;
    r[2] = index;
    r[3..5].copy_from_slice(&value.to_le_bytes());
    r
}

#[inline]
fn now_ms() -> u64 {
    hw::time::ms()
}

/// Built-in LED pattern, (r, g, b) per LED: off, except a short blue-white flash when the
/// host first configures the keypad.
fn led_pattern(s: &State, now: u64) -> [[u8; 3]; 3] {
    if let Some(l) = s.led_override {
        return l;
    }
    let c = |v: u8| (v as u32 * s.brightness as u32 / 255) as u8;
    if now < s.hello_until {
        return [[c(140), c(170), c(255)]; 3];
    }
    let mut out = [[0u8; 3]; 3];
    for led in 0..3 {
        if let Some((rgb, start, period, count)) = s.blink[led] {
            let t = now.saturating_sub(start);
            let period = period.max(1) as u64;
            if t < period * 2 * count as u64 && (t / period) % 2 == 0 {
                out[led] = [c(rgb[0]), c(rgb[1]), c(rgb[2])];
            }
        }
    }
    out
}

fn handle(s: &mut State, q: &mut Queue, req: &[u8; 64]) {
    let ack = |q: &mut Queue, st: u8| {
        let mut r = [0u8; 64];
        r[0] = msg::ACK;
        r[1] = req[0];
        r[2] = st;
        q.push(r);
    };
    match req[0] {
        cmd::INFO => {
            let mut r = [0u8; 64];
            r[0] = msg::INFO;
            r[1] = proto::mode::APP;
            r[2] = proto::PROTO_VERSION;
            proto::put_u16(&mut r, 3, VERSION);
            r[5..17].copy_from_slice(&hw::uid());
            proto::put_u32(&mut r, 17, proto::APP_MAX);
            r[21] = 1;
            q.push(r);
        }
        cmd::LEDS => {
            let mut l = [[0u8; 3]; 3];
            for led in 0..3 {
                for c in 0..3 {
                    l[led][c] = req[1 + led * 3 + c];
                }
            }
            s.led_override = if req[10] != 0 { Some(l) } else { None };
            ack(q, status::OK);
        }
        cmd::BLINK => {
            let led = req[1] as usize;
            if led < 3 {
                let period = proto::get_u16(req, 5);
                s.blink[led] = Some(([req[2], req[3], req[4]], now_ms(), period, req[7].max(1)));
                ack(q, status::OK);
            } else {
                ack(q, status::BAD_ARG);
            }
        }
        cmd::TEXT | cmd::PIXELS | cmd::SCREEN_MODE => ack(q, status::OK),
        cmd::STREAM => {
            s.stream = req[1] != 0;
            ack(q, status::OK);
        }
        cmd::THRESHOLDS => {
            let k = req[1] as usize;
            if k < KEYS {
                let p = proto::get_u16(req, 2);
                let r = proto::get_u16(req, 4);
                if p != 0 {
                    s.keys[k].press = p;
                }
                if r != 0 {
                    s.keys[k].release = r;
                }
                ack(q, status::OK);
            } else {
                ack(q, status::BAD_ARG);
            }
        }
        cmd::BRIGHTNESS => {
            s.brightness = req[1];
            ack(q, status::OK);
        }
        cmd::BOOTLOADER | cmd::RESET => {
            ack(q, status::OK);
            if req[0] == cmd::BOOTLOADER {
                unsafe { (proto::HANDOFF_ADDR as *mut u32).write_volatile(proto::HANDOFF_STAY_IN_BOOT) };
                leds_red(s);
            }
            s.reset_at = Some(now_ms() + 50);
        }
        cmd::PIN => {
            let port = [gpio::A, gpio::B, gpio::C][(req[1] as usize).min(2)];
            let pin = (req[2] & 15) as u32;
            match req[4] {
                0 => gpio::low(port, pin),
                1 => gpio::high(port, pin),
                _ => {}
            }
            gpio::mode(port, pin, (req[3] & 15) as u32);
            let mut r = [0u8; 64];
            r[0] = msg::ACK;
            r[1] = cmd::PIN;
            r[2] = status::OK;
            proto::put_u32(&mut r, 3, gpio::read(port));
            q.push(r);
        }
        cmd::BOOT_ERASE => {
            leds_red(s);
            ack(q, boot_erase(&mut s.boot, proto::get_u32(req, 1)))
        }
        cmd::BOOT_WRITE => {
            let len = (req[5] as usize).min(32);
            ack(q, status_of(s.boot.write(proto::get_u32(req, 1), &req[8..8 + len])))
        }
        cmd::BOOT_COMMIT => {
            let st = boot_commit(&mut s.boot, proto::get_u32(req, 1), proto::get_u32(req, 5));
            s.led_override = None;
            ack(q, st)
        }
        CMD_HID_ENABLE => {
            s.hid.enabled = req[1] != 0;
            if !s.hid.enabled {
                s.hid.reset();
            }
            ack(q, status::OK);
        }
        _ => ack(q, status::UNKNOWN),
    }
}

fn status_of(r: Result<(), WriteError>) -> u8 {
    match r {
        Ok(()) => status::OK,
        Err(WriteError::Order) => status::ORDER,
        Err(WriteError::Flash) => status::FLASH,
    }
}

/// Erases the 4 KiB sectors of [0, len) and starts a new bootloader image there.
fn boot_erase(w: &mut PageWriter, len: u32) -> u8 {
    if len == 0 || len > proto::BOOT_SIZE {
        return status::BAD_ARG;
    }
    let mut a = 0;
    while a < len {
        if !hw::flash::erase_sector(a) {
            return status::FLASH;
        }
        a += hw::flash::SECTOR;
    }
    w.start(0, proto::BOOT_SIZE);
    status::OK
}

fn boot_commit(w: &mut PageWriter, len: u32, crc: u32) -> u8 {
    match w.finish(len) {
        Ok(image) if proto::crc32(image) == crc => status::OK,
        Ok(_) => status::CRC,
        Err(e) => status_of(Err(e)),
    }
}

/// All LEDs red: the keypad is about to flash something.
fn leds_red(s: &mut State) {
    s.led_override = Some([[255, 0, 0]; 3]);
    leds::set(&[[255, 0, 0]; 3]);
}

/// Single-sample ADC read per channel smoothed with a fixed-point Q14.2 EWMA (alpha = 1/4).
#[inline(always)]
fn read_keys_ewma(ewma: &mut [u32; KEYS]) -> [u16; KEYS] {
    const CHANS: [u32; KEYS] = [13, 12, 11];
    let mut out = [0u16; KEYS];
    for k in 0..KEYS {
        let sample = hw::adc1::read(CHANS[k]).unwrap_or(0) as u32;
        ewma[k] = ewma[k] - (ewma[k] >> 2) + sample;
        out[k] = (ewma[k] >> 2) as u16;
    }
    out
}

#[no_mangle]
extern "C" fn rust_trap(mcause: u32) {
    if mcause == 0x8000_0000 | leds::IRQ {
        leds::isr();
    } else {
        panic::fatal_trap(mcause);
    }
}

#[no_mangle]
extern "C" fn rust_main() -> ! {
    hw::rcc::init_144mhz();
    hw::time::init();
    hw::rcc::enable_apb2(hw::rcc::IOPA | hw::rcc::IOPB | hw::rcc::IOPC);

    leds::set(&[[0; 3]; 3]);
    leds::start();
    hw::interrupts_on();

    // Knob: PB12 = A, PB13 = B, PB14 = switch, all pull-up.
    for pin in [12, 13, 14] {
        gpio::pull_up(gpio::B, pin);
    }

    // Hall sensors: PC3 = IN13 (left), PC2 = IN12 (middle), PC1 = IN11 (right).
    for pin in [1, 2, 3] {
        gpio::mode(gpio::C, pin, gpio::ANALOG);
    }
    hw::adc1::init();
    for chan in [11, 12, 13] {
        hw::adc1::sample_time(chan, 3); // 28.5 cycles (~2.5x faster than 71.5 cycles)
    }
    hw::time::delay_ms(10);

    // Discard initial dummy reads so the ADC sample-and-hold capacitor stabilizes.
    for _ in 0..8 {
        for chan in [13u32, 12, 11] {
            let _ = hw::adc1::read(chan);
        }
    }

    let mut s = State {
        keys: [Key::new(); KEYS],
        adc_ewma: [0; KEYS],
        pins: 0,
        stream: false,
        led_override: None,
        brightness: 255,
        hello_until: 0,
        hello_done: false,
        blink: [None; 3],
        reset_at: None,
        boot: PageWriter::new(),
        hid: HidEngine::new(),
    };

    // Rest values & EWMA seed: average of 64 samples.
    let mut sum = [0u32; KEYS];
    for _ in 0..64 {
        for (k, chan) in [13u32, 12, 11].iter().enumerate() {
            sum[k] += hw::adc1::read(*chan).unwrap_or(0) as u32;
        }
    }
    for k in 0..KEYS {
        let avg = sum[k] / 64;
        s.keys[k].rest = avg << 4;
        s.adc_ewma[k] = avg << 2;
    }

    let mut usb = Usb::new(&DESC);
    let mut q = Queue { buf: [[0; 64]; 16], head: 0, len: 0 };
    let mut knob = Knob::new();
    let mut last_ms = now_ms();
    let mut last_sw = false;
    let mut was_configured = false;
    let mut req = [0u8; 64];

    loop {
        usb.poll();

        if usb.configured() != was_configured {
            was_configured = usb.configured();
            if !was_configured {
                s.hid.reset();
            } else if !s.hello_done {
                s.hello_done = true;
                s.hello_until = now_ms() + 250;
            }
        }

        // Fast path 1: scan Hall-effect keys every iteration.
        let raw = read_keys_ewma(&mut s.adc_ewma);
        for k in 0..KEYS {
            if let Some(down) = s.keys[k].update(raw[k]) {
                s.hid.on_key(k, down);
                q.push(event_report(event::KEY, k as u8, down as i16));
            }
        }

        // Fast path 2: sample rotary encoder quadrature every iteration.
        let pins = ((gpio::read(gpio::B) >> 12) & 7) as u8;
        s.pins = pins;
        let detent = knob.update(pins & (1 << input::KNOB_A) != 0, pins & (1 << input::KNOB_B) != 0);
        if detent != 0 {
            s.hid.on_knob_detent(detent);
            q.push(event_report(event::KNOB, 0, detent as i16));
        }

        // Flush native HID endpoint before vendor endpoint traffic.
        s.hid.flush(&mut usb);

        // Vendor HID traffic (Interface 0).
        if usb.read(EP_OUT, &mut req).is_some() {
            handle(&mut s, &mut q, &req);
        }
        if let Some(r) = q.front() {
            let r = *r;
            if usb.write(EP_IN, &r) {
                q.pop();
            }
        }

        // 1 ms housekeeping: knob push switch, LEDs, raw stream, and reset checks.
        let now = now_ms();
        if now != last_ms {
            last_ms = now;
            for k in 0..KEYS {
                s.keys[k].drift();
            }
            let sw = pins & (1 << input::KNOB_SW) == 0;
            if sw != last_sw {
                last_sw = sw;
                s.hid.on_knob_switch(sw);
                q.push(event_report(event::KNOB_BUTTON, 0, sw as i16));
            }
            if now % 5 == 0 {
                leds::set(&led_pattern(&s, now));
            }
            if s.stream && now % 10 == 0 {
                let mut r = [0u8; 64];
                r[0] = msg::RAW;
                for k in 0..KEYS {
                    proto::put_u16(&mut r, 1 + k * 2, s.keys[k].raw);
                    proto::put_u16(&mut r, 7 + k * 2, s.keys[k].rest());
                    proto::put_u16(&mut r, 19 + k * 2, s.keys[k].travel);
                    if s.keys[k].down {
                        r[14] |= 1 << k;
                    }
                }
                r[13] = s.pins;
                proto::put_u32(&mut r, 15, now as u32);
                q.push(r);
            }
            if let Some(t) = s.reset_at {
                if now >= t && q.front().is_none() && !usb.in_busy(EP_IN) {
                    usb.disconnect();
                    hw::time::delay_ms(50);
                    hw::system_reset();
                }
            }
        }
    }
}
