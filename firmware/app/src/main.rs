//! O3C app firmware. Starts at 0x8000, after the bootloader.
//!
//! USB: one vendor HID interface for the `o3c` host tool (key and knob events, raw sensor
//! stream, LEDs, screen, bootloader entry). The keypad does nothing on its own: a program
//! on the host (for example `o3c run`) gives the keys and the knob their meaning.
//!
//! One polling loop. Only the LED matrix runs from an interrupt (TIM2).
#![no_std]
#![no_main]

mod display;
mod input;
mod leds;
mod panic;

use hw::flash::{PageWriter, WriteError};
use hw::gpio;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::raw::RawU16;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use o3c_proto::{self as proto, cmd, event, msg, status};
use usbhs::{Descriptors, Interface, Usb};

use display::Frame;
use input::{Key, Knob, KEYS};

const VERSION: u16 = 8;
const TEXT_LINES: usize = 5;
/// Seconds the diagnostics stay on screen after the host configures the keypad.
const SCREEN_SECS: u64 = 5;
const TEXT_COLS: usize = 26;

const EP_IN: u8 = 2;
const EP_OUT: u8 = 3;

const VENDOR_REPORT: &[u8] = &[
    0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x40, 0x09, 0x02,
    0x81, 0x02, 0x95, 0x40, 0x09, 0x03, 0x91, 0x02, 0xC0,
];

const DEVICE: &[u8] = &[
    18, 1, 0x00, 0x02, 0, 0, 0, 64,
    (proto::VID & 0xFF) as u8, (proto::VID >> 8) as u8, (proto::PID & 0xFF) as u8, (proto::PID >> 8) as u8,
    0x00, 0x02, 1, 2, 0, 1,
];

const CONFIG: &[u8] = &[
    9, 2, 41, 0, 1, 1, 0, 0x80, 50, //
    9, 4, 0, 0, 2, 3, 0, 0, 0, //
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, VENDOR_REPORT.len() as u8, 0, //
    7, 5, 0x80 | EP_IN, 3, 64, 0, 1, //
    7, 5, EP_OUT, 3, 64, 0, 1,
];

static DESC: Descriptors = Descriptors {
    device: DEVICE,
    config: CONFIG,
    interfaces: &[Interface { report: VENDOR_REPORT, hid_desc_offset: 18 }],
    strings: &[&[4, 3, 0x09, 0x04], usbhs::string_descriptor!("o3c"), usbhs::string_descriptor!("O3C")],
    in_eps: &[EP_IN],
    out_eps: &[EP_OUT],
};

/// The screen's frame buffer. Also drawn into by the panic handler.
pub(crate) static mut FRAME: Frame = Frame::new();

pub(crate) fn frame() -> &'static mut Frame {
    unsafe { &mut *(&raw mut FRAME) }
}

struct State {
    keys: [Key; KEYS],
    pins: u8,
    knob: i32,
    knob_sw: bool,
    stream: bool,
    screen_mode: u8,
    text: [[u8; TEXT_COLS]; TEXT_LINES],
    text_len: [u8; TEXT_LINES],
    text_color: [u16; TEXT_LINES],
    led_override: Option<[[u8; 3]; 3]>,
    brightness: u8,
    /// Screen on (picture) or off (panel asleep), as the host set it with BRIGHTNESS.
    screen_on: bool,
    /// Screen follows USB: on while there is no host and for SCREEN_SECS after it
    /// configures, then off. BRIGHTNESS ends this.
    screen_auto: bool,
    /// Screen black and the panel asleep.
    blanked: bool,
    /// End of the start-up LED flash (ms).
    hello_until: u64,
    hello_done: bool,
    /// Per LED: (r, g, b), start ms, on/off period ms, blinks.
    blink: [Option<([u8; 3], u64, u16, u8)>; 3],
    dirty: bool,
    reset_at: Option<u64>,
    boot: PageWriter,
}

/// Small ring of reports for the vendor IN endpoint.
struct Queue {
    buf: [[u8; 64]; 16],
    head: usize,
    len: usize,
}

impl Queue {
    fn push(&mut self, r: [u8; 64]) {
        if self.len < self.buf.len() {
            let i = (self.head + self.len) % self.buf.len();
            self.buf[i] = r;
            self.len += 1;
        }
    }
    fn front(&self) -> Option<&[u8; 64]> {
        (self.len > 0).then(|| &self.buf[self.head])
    }
    fn pop(&mut self) {
        if self.len > 0 {
            self.head = (self.head + 1) % self.buf.len();
            self.len -= 1;
        }
    }
}

fn event_report(kind: u8, index: u8, value: i16) -> [u8; 64] {
    let mut r = [0u8; 64];
    r[0] = msg::EVENT;
    r[1] = kind;
    r[2] = index;
    r[3..5].copy_from_slice(&value.to_le_bytes());
    r
}

fn now_ms() -> u64 {
    hw::time::ms()
}

/// Built-in LED pattern, (r, g, b) per LED: off, except a short blue-white flash when the
/// host first configures the keypad (it started and is connected). The host can set its own.
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

struct Line {
    b: [u8; 32],
    n: usize,
}

impl Line {
    fn new() -> Self {
        Self { b: [0; 32], n: 0 }
    }
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.b[..self.n]).unwrap_or("")
    }
}

impl core::fmt::Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &c in s.as_bytes() {
            if self.n < self.b.len() {
                self.b[self.n] = c;
                self.n += 1;
            }
        }
        Ok(())
    }
}

/// What the diagnostics screen shows about USB.
#[derive(Default)]
struct UsbStatus {
    configured: bool,
    resets: u32,
    setups: u32,
}

impl From<&Usb> for UsbStatus {
    fn from(u: &Usb) -> Self {
        Self { configured: u.configured(), resets: u.resets, setups: u.setups }
    }
}

fn draw_builtin(f: &mut Frame, s: &State, usb: UsbStatus) {
    use core::fmt::Write;
    let _ = f.clear(Rgb565::BLACK);
    for line in 0..TEXT_LINES {
        let len = s.text_len[line] as usize;
        if len == 0 {
            continue;
        }
        let text = core::str::from_utf8(&s.text[line][..len]).unwrap_or("?");
        let color = Rgb565::from(RawU16::new(s.text_color[line]));
        let _ = Text::with_baseline(text, Point::new(1, line as i32 * 10), MonoTextStyle::new(&FONT_6X10, color), Baseline::Top).draw(f);
    }
    let dim = MonoTextStyle::new(&FONT_6X10, Rgb565::new(20, 40, 20));
    let mut l = Line::new();
    let _ = write!(l, "knob {}{} io {:03b}", s.knob, if s.knob_sw { " push" } else { "" }, s.pins);
    let _ = Text::with_baseline(l.as_str(), Point::new(1, 41), dim, Baseline::Top).draw(f);
    let mut l = Line::new();
    let _ = write!(
        l,
        "usb {} r{} s{} t{}",
        if usb.configured { "ok" } else { "--" },
        usb.resets,
        usb.setups,
        now_ms() / 1000
    );
    let _ = Text::with_baseline(l.as_str(), Point::new(1, 51), dim, Baseline::Top).draw(f);
    for k in 0..KEYS {
        let key = &s.keys[k];
        let x = k as i32 * 54;
        let _ = Rectangle::new(Point::new(x, 62), Size::new(52, 18))
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::new(8, 16, 8), 1))
            .draw(f);
        let full = key.travel.max(key.press).max(1) as u32;
        let w = (key.delta() as u32 * 50 / full).min(50);
        let color = if key.down { Rgb565::new(31, 40, 0) } else { Rgb565::new(0, 30, 20) };
        if w > 0 {
            let _ = Rectangle::new(Point::new(x + 1, 63), Size::new(w, 16)).into_styled(PrimitiveStyle::with_fill(color)).draw(f);
        }
    }
}

fn set_line(s: &mut State, line: usize, color: u16, text: &[u8]) {
    if line >= TEXT_LINES {
        return;
    }
    let n = text.len().min(TEXT_COLS);
    s.text[line][..n].copy_from_slice(&text[..n]);
    for c in s.text[line][..n].iter_mut() {
        if !(0x20..0x7F).contains(c) {
            *c = b'?';
        }
    }
    s.text_len[line] = n as u8;
    s.text_color[line] = color;
    s.dirty = true;
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
        cmd::TEXT => {
            let len = (req[4] as usize).min(59);
            set_line(s, req[1] as usize, proto::get_u16(req, 2), &req[5..5 + len]);
            ack(q, status::OK);
        }
        cmd::STREAM => {
            s.stream = req[1] != 0;
            ack(q, status::OK);
        }
        cmd::PIXELS => {
            let first = proto::get_u16(req, 1) as usize;
            let n = (req[3] as usize).min(30);
            let f = frame();
            for i in 0..n {
                f.set_raw(first + i, proto::get_u16(req, 4 + i * 2));
            }
            if s.screen_mode == 1 {
                s.dirty = true;
            }
        }
        cmd::SCREEN_MODE => {
            s.screen_mode = req[1];
            s.dirty = true;
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
            s.screen_on = req[2] != 0;
            s.screen_auto = false;
            s.dirty = true;
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

fn read_keys() -> [u16; KEYS] {
    let mut acc = [0u32; KEYS];
    for _ in 0..4 {
        for (k, chan) in [13u32, 12, 11].iter().enumerate() {
            acc[k] += hw::adc1::read(*chan).unwrap_or(0) as u32;
        }
    }
    [(acc[0] / 4) as u16, (acc[1] / 4) as u16, (acc[2] / 4) as u16]
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
        hw::adc1::sample_time(chan, 6); // 71.5 cycles
    }

    display::init();

    let mut s = State {
        keys: [Key::new(); KEYS],
        pins: 0,
        knob: 0,
        knob_sw: false,
        stream: false,
        screen_mode: 0,
        text: [[b' '; TEXT_COLS]; TEXT_LINES],
        text_len: [0; TEXT_LINES],
        text_color: [0xFFFF; TEXT_LINES],
        led_override: None,
        brightness: 255,
        screen_on: true,
        screen_auto: true,
        blanked: false,
        hello_until: 0,
        hello_done: false,
        blink: [None; 3],
        dirty: true,
        reset_at: None,
        boot: PageWriter::new(),
    };
    set_line(&mut s, 0, 0xFFE0, b"O3C firmware 8");
    set_line(&mut s, 1, 0x07FF, b"no host yet");

    // Rest values: average of 64 samples.
    let mut sum = [0u32; KEYS];
    for _ in 0..64 {
        let r = read_keys();
        for k in 0..KEYS {
            sum[k] += r[k] as u32;
        }
    }
    for k in 0..KEYS {
        s.keys[k].rest = (sum[k] / 64) << 4;
    }

    draw_builtin(frame(), &s, UsbStatus::default());
    display::flush(frame());

    let mut usb = Usb::new(&DESC);
    let mut q = Queue { buf: [[0; 64]; 16], head: 0, len: 0 };
    let mut knob = Knob::new();
    let mut last_ms = now_ms();
    let mut last_draw = 0u64;
    let mut last_sw = false;
    let mut flush_pos: Option<usize> = None;
    let mut was_configured = false;
    let mut configured_at = 0u64;
    let mut req = [0u8; 64];

    loop {
        usb.poll();

        if usb.read(EP_OUT, &mut req).is_some() {
            handle(&mut s, &mut q, &req);
        }

        if let Some(r) = q.front() {
            let r = *r;
            if usb.write(EP_IN, &r) {
                q.pop();
            }
        }

        if usb.configured() != was_configured {
            was_configured = usb.configured();
            configured_at = now_ms();
            s.dirty = true;
            if was_configured && !s.hello_done {
                s.hello_done = true;
                s.hello_until = configured_at + 250;
            }
        }

        let now = now_ms();
        if now != last_ms {
            last_ms = now;
            let raw = read_keys();
            let pins = ((gpio::read(gpio::B) >> 12) & 7) as u8;
            if pins != s.pins {
                s.pins = pins;
                s.dirty = true;
            }
            let detent = knob.update(pins & (1 << input::KNOB_A) != 0, pins & (1 << input::KNOB_B) != 0);
            let sw = pins & (1 << input::KNOB_SW) == 0;
            for k in 0..KEYS {
                if let Some(down) = s.keys[k].update(raw[k]) {
                    q.push(event_report(event::KEY, k as u8, down as i16));
                    s.dirty = true;
                }
            }
            if detent != 0 {
                s.knob += detent as i32;
                q.push(event_report(event::KNOB, 0, detent as i16));
                s.dirty = true;
            }
            if sw != last_sw {
                last_sw = sw;
                s.knob_sw = sw;
                q.push(event_report(event::KNOB_BUTTON, 0, sw as i16));
                s.dirty = true;
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

        // Screen: redraw at most every 33 ms (and once a second for the clock), sent in
        // 1 KiB pieces between USB polls.
        match flush_pos {
            Some(pos) => {
                let next = display::step(frame(), pos, 1024);
                flush_pos = if next >= display::BYTES { None } else { Some(next) };
            }
            None => {
                let on = if s.screen_auto {
                    !usb.configured() || now - configured_at < SCREEN_SECS * 1000
                } else {
                    s.screen_on
                };
                if !on {
                    if !s.blanked {
                        // Black picture, then the panel off and asleep.
                        s.blanked = true;
                        let _ = frame().clear(Rgb565::BLACK);
                        display::flush(frame());
                        display::panel(false);
                    }
                } else if s.blanked || (s.dirty && now - last_draw >= 33) || now - last_draw >= 1000 {
                    if s.blanked {
                        display::panel(true);
                    }
                    s.blanked = false;
                    last_draw = now;
                    s.dirty = false;
                    if s.screen_mode == 0 {
                        draw_builtin(frame(), &s, UsbStatus::from(&usb));
                    }
                    display::begin();
                    flush_pos = Some(0);
                }
            }
        }
    }
}
