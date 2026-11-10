//! Wire protocol between the O3C firmware (bootloader and app) and the `o3c` host tool.
//!
//! Every message is one 64-byte HID report on the vendor interface (usage page 0xFF00, no
//! report ID). Byte 0 is the message type. Host-to-device types are below 0x80, replies and
//! events are 0x80 and up. Multi-byte numbers are little-endian.
#![no_std]

/// pid.codes test VID/PID. The bootloader and the app use the same pair; INFO tells them apart.
pub const VID: u16 = 0x1209;
pub const PID: u16 = 0x0001;
pub const USAGE_PAGE: u16 = 0xFF00;
pub const USAGE: u16 = 0x0001;
pub const REPORT_LEN: usize = 64;
pub const PROTO_VERSION: u8 = 1;

/// Flash layout. Addresses are CPU addresses; the flash controller wants FLASH_BASE + offset.
pub const FLASH_BASE: u32 = 0x0800_0000;
pub const BOOT_SIZE: u32 = 0x8000;
pub const APP_START: u32 = BOOT_SIZE;
/// The last 256-byte page of the app region holds the app header.
pub const APP_END: u32 = 0x3_0000;
pub const HEADER_ADDR: u32 = APP_END - PAGE_SIZE;
pub const APP_MAX: u32 = HEADER_ADDR - APP_START;
pub const PAGE_SIZE: u32 = 256;

/// App header, written by the bootloader after the CRC of the written image checks out.
pub const HEADER_MAGIC: u32 = 0x4F33_4321; // "!C3O"

/// Bootloader handoff word at the start of RAM (both images keep it out of their memory.x).
pub const HANDOFF_ADDR: u32 = 0x2000_0000;
pub const HANDOFF_STAY_IN_BOOT: u32 = 0xB007_10AD;

pub mod mode {
    pub const BOOT: u8 = 1;
    pub const APP: u8 = 2;
}

/// Host to device.
pub mod cmd {
    /// -> INFO
    pub const INFO: u8 = 0x01;

    // App only.
    /// [1..10]: 3 LEDs (left, middle, right) x (r, g, b), 0..255. [10]: 1 = hold these
    /// (override the built-in pattern), 0 = back to the built-in pattern.
    pub const LEDS: u8 = 0x02;
    /// [1]: line 0..4, [2..5]: rgb565 colour LE, [4]: len, [5..]: ASCII text. Draws in the host
    /// text area of the built-in screen.
    pub const TEXT: u8 = 0x03;
    /// [1]: 1 = stream RAW reports every 10 ms, 0 = stop.
    pub const STREAM: u8 = 0x04;
    /// [1..3]: first pixel index (u16), [3]: count n <= 30, [4..]: n rgb565 pixels.
    /// Writes into the host framebuffer. Shown only in screen mode 1.
    pub const PIXELS: u8 = 0x05;
    /// [1]: 0 = built-in screen, 1 = host framebuffer. Mode 1 redraws from the framebuffer.
    pub const SCREEN_MODE: u8 = 0x06;
    /// [1]: key 0..2, [2..4]: press threshold, [4..6]: release threshold, in ADC counts away
    /// from the rest value. 0 keeps the current value.
    pub const THRESHOLDS: u8 = 0x07;
    /// [1]: brightness 0..255 of the built-in LED pattern, [2]: screen 1 = on, 0 = off (panel
    /// asleep; the backlight has no switch). Ends the automatic screen-off.
    pub const BRIGHTNESS: u8 = 0x08;
    /// [1]: LED 0..2 (left to right), [2..5]: r, g, b, [5..7]: on time = off time in ms,
    /// [7]: number of blinks (0 = 1). Shown over the built-in pattern (off) until done.
    pub const BLINK: u8 = 0x09;
    /// Debug: [1] port (0 = A, 1 = B, 2 = C), [2] pin, [3] mode nibble (CNF/MODE),
    /// [4] 0 = low, 1 = high, 2 = leave the output register. Replies ACK and [3..7] the
    /// port's input register.
    pub const PIN: u8 = 0x30;
    /// Reset into the bootloader.
    pub const BOOTLOADER: u8 = 0x7F;
    /// Bootloader update, done by the app: [1..5] image length. Erases the 4 KiB sectors
    /// of [0, len). From here until BOOT_COMMIT succeeds a reset leaves no bootloader:
    /// recovery is then WCH's ROM bootloader (BOOT0 pads).
    pub const BOOT_ERASE: u8 = 0x20;
    /// As WRITE, into the bootloader region (offset from 0).
    pub const BOOT_WRITE: u8 = 0x21;
    /// [1..5] length, [5..9] CRC-32: programs the last page and checks the CRC over flash.
    pub const BOOT_COMMIT: u8 = 0x22;

    // Bootloader only.
    /// [1..5]: image length (0 = the whole app region). Erases the header and the 4 KiB
    /// sectors the image needs.
    pub const ERASE: u8 = 0x10;
    /// [1..5]: offset from APP_START (multiple of 32), [5]: len <= 32, [8..8+len]: data.
    /// Offsets must arrive in order; each full 256-byte page is programmed when complete.
    pub const WRITE: u8 = 0x11;
    /// [1..5]: image length, [5..9]: CRC-32 (IEEE) of the image. Programs the last partial
    /// page, checks the CRC over flash and writes the header.
    pub const COMMIT: u8 = 0x12;
    /// Reset and start the app (only if the header is valid).
    pub const RUN: u8 = 0x13;
    /// Same value in the app: reset (and start the app again).
    pub const RESET: u8 = 0x7E;
}

/// Device to host.
pub mod msg {
    /// [1]: mode, [2]: PROTO_VERSION, [3..5]: firmware version, [5..17]: chip UID (12 bytes),
    /// [17..21]: APP_MAX, [21]: app header valid (bootloader only).
    pub const INFO: u8 = 0x81;
    /// [1]: command echoed, [2]: status (0 = ok).
    pub const ACK: u8 = 0x82;
    /// [1]: event kind, [2]: index, [3..5]: i16 value.
    pub const EVENT: u8 = 0x83;
    /// [1..7]: 3 x u16 ADC, [7..13]: 3 x u16 rest values, [13]: digital pins (bit0 PB12,
    /// bit1 PB13, bit2 PB14; 1 = high), [14]: key state bits, [15..19]: u32 ms uptime,
    /// [19..25]: 3 x u16 learned full travel.
    pub const RAW: u8 = 0x84;
}

pub mod event {
    /// index = key 0..2, value = 1 pressed / 0 released.
    pub const KEY: u8 = 1;
    /// index 0, value = detents turned (+ clockwise).
    pub const KNOB: u8 = 2;
    /// index 0, value = 1 pressed / 0 released.
    pub const KNOB_BUTTON: u8 = 3;
}

pub mod status {
    pub const OK: u8 = 0;
    pub const BAD_ARG: u8 = 1;
    pub const FLASH: u8 = 2;
    pub const CRC: u8 = 3;
    pub const ORDER: u8 = 4;
    pub const NO_APP: u8 = 5;
    pub const UNKNOWN: u8 = 0xFF;
}

/// CRC-32/ISO-HDLC (zlib), bitwise. Small and fast enough for 160 KB once per flash.
pub fn crc32_update(mut crc: u32, data: &[u8]) -> u32 {
    crc = !crc;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xEDB8_8320 } else { crc >> 1 };
        }
    }
    !crc
}

pub fn crc32(data: &[u8]) -> u32 {
    crc32_update(0, data)
}

#[inline]
pub fn put_u16(buf: &mut [u8], at: usize, v: u16) {
    buf[at..at + 2].copy_from_slice(&v.to_le_bytes());
}

#[inline]
pub fn put_u32(buf: &mut [u8], at: usize, v: u32) {
    buf[at..at + 4].copy_from_slice(&v.to_le_bytes());
}

#[inline]
pub fn get_u16(buf: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([buf[at], buf[at + 1]])
}

#[inline]
pub fn get_u32(buf: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([buf[at], buf[at + 1], buf[at + 2], buf[at + 3]])
}
