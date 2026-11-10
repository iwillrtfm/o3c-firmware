//! ST7735S 0.96" IPS, 160x80, on SPI1 (PA5 SCK, PA7 MOSI). PA4 = reset, PA6 = data/command,
//! PB0 = chip select. Init sequence, MADCTL 0xA8, inversion and the (1, 26) window offset are
//! the stock firmware's. The backlight has no switch on this board: it is on whenever the
//! keypad has power. PA3 is driven high as the stock firmware does; it changes nothing seen.

use embedded_graphics::pixelcolor::raw::RawU16;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use hw::{gpio, spi1, time};

pub const W: usize = 160;
pub const H: usize = 80;
pub const BYTES: usize = W * H * 2;
const X_OFF: u16 = 1;
const Y_OFF: u16 = 26;

/// Big-endian RGB565, the byte order the panel takes.
pub struct Frame {
    pub buf: [u8; BYTES],
}

impl Frame {
    pub const fn new() -> Self {
        Self { buf: [0; BYTES] }
    }

    pub fn set_raw(&mut self, index: usize, rgb565: u16) {
        if index < W * H {
            self.buf[index * 2..index * 2 + 2].copy_from_slice(&rgb565.to_be_bytes());
        }
    }
}

impl OriginDimensions for Frame {
    fn size(&self) -> Size {
        Size::new(W as u32, H as u32)
    }
}

impl DrawTarget for Frame {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I: IntoIterator<Item = Pixel<Rgb565>>>(&mut self, pixels: I) -> Result<(), Self::Error> {
        for Pixel(p, c) in pixels {
            if p.x >= 0 && p.y >= 0 && (p.x as usize) < W && (p.y as usize) < H {
                let raw: u16 = RawU16::from(c).into_inner();
                self.set_raw(p.y as usize * W + p.x as usize, raw);
            }
        }
        Ok(())
    }

    fn clear(&mut self, color: Rgb565) -> Result<(), Self::Error> {
        let raw: u16 = RawU16::from(color).into_inner();
        let b = raw.to_be_bytes();
        for px in self.buf.chunks_exact_mut(2) {
            px.copy_from_slice(&b);
        }
        Ok(())
    }
}

/// (command, data) from the stock init, in order.
const INIT: &[(u8, &[u8])] = &[
    (0xB1, &[0x01, 0x2D, 0x2E]),
    (0xB2, &[0x01, 0x2D, 0x2E]),
    (0xB3, &[0x01, 0x2D, 0x2E, 0x01, 0x2D, 0x2E]),
    (0xB4, &[0x03]),
    (0xC0, &[0xAB, 0x0B, 0x04]),
    (0xC1, &[0xC5]),
    (0xC2, &[0x0D, 0x00]),
    (0xC3, &[0x8D, 0x6A]),
    (0xC4, &[0x8D, 0xEE]),
    (0xC5, &[0x0F]),
    (0xE0, &[0x04, 0x22, 0x07, 0x0A, 0x2E, 0x30, 0x25, 0x2A, 0x28, 0x26, 0x2E, 0x3A, 0x00, 0x01, 0x03, 0x13]),
    (0xE1, &[0x04, 0x16, 0x06, 0x0D, 0x2D, 0x26, 0x23, 0x27, 0x27, 0x25, 0x2D, 0x3B, 0x00, 0x01, 0x04, 0x13]),
    (0xFC, &[0x80]),
    (0x3A, &[0x05]),
    (0x36, &[0xA8]),
    (0x21, &[]),
    (0x29, &[]),
];

const RST: u32 = 4; // PA4
const DC: u32 = 6; // PA6
const PA3: u32 = 3; // driven high, as the stock firmware does
const CS: u32 = 0; // PB0

fn command(c: u8, data: &[u8]) {
    gpio::low(gpio::B, CS);
    gpio::low(gpio::A, DC);
    spi1::write(&[c]);
    spi1::flush();
    gpio::high(gpio::A, DC);
    if !data.is_empty() {
        spi1::write(data);
        spi1::flush();
    }
    gpio::high(gpio::B, CS);
}

pub fn init() {
    hw::rcc::enable_apb2(hw::rcc::IOPA | hw::rcc::IOPB);
    for p in [RST, DC, PA3] {
        gpio::mode(gpio::A, p, gpio::OUT_PP);
    }
    gpio::high(gpio::B, CS);
    gpio::mode(gpio::B, CS, gpio::OUT_PP);
    gpio::high(gpio::A, PA3);
    spi1::init();
    gpio::low(gpio::A, RST);
    time::delay_ms(30);
    gpio::high(gpio::A, RST);
    time::delay_ms(50);
    command(0x11, &[]); // sleep out
    time::delay_ms(120);
    for (c, d) in INIT {
        command(*c, d);
    }
}

/// Panel on or off. Off is the controller's display-off and sleep: the picture goes, the
/// backlight stays (it has no switch). Frame memory is kept.
pub fn panel(on: bool) {
    if on {
        command(0x11, &[]); // sleep out
        time::delay_ms(5);
        command(0x29, &[]); // display on
    } else {
        command(0x28, &[]); // display off
        command(0x10, &[]); // sleep in
    }
}

/// Starts a frame transfer; `step` sends it in pieces so the main loop keeps polling USB.
pub fn begin() {
    let x0 = X_OFF;
    let x1 = X_OFF + W as u16 - 1;
    let y0 = Y_OFF;
    let y1 = Y_OFF + H as u16 - 1;
    command(0x2A, &[(x0 >> 8) as u8, x0 as u8, (x1 >> 8) as u8, x1 as u8]);
    command(0x2B, &[(y0 >> 8) as u8, y0 as u8, (y1 >> 8) as u8, y1 as u8]);
    gpio::low(gpio::B, CS);
    gpio::low(gpio::A, DC);
    spi1::write(&[0x2C]);
    spi1::flush();
    gpio::high(gpio::A, DC);
}

/// Sends frame bytes from `pos`, at most `max`; returns the new position and ends the
/// transfer at the end of the frame.
pub fn step(frame: &Frame, pos: usize, max: usize) -> usize {
    let end = (pos + max).min(BYTES);
    spi1::write(&frame.buf[pos..end]);
    if end == BYTES {
        spi1::flush();
        gpio::high(gpio::B, CS);
    }
    end
}

/// Whole frame at once (blocking, about 12 ms).
pub fn flush(frame: &Frame) {
    begin();
    step(frame, 0, BYTES);
}
