//! Fatal screen: panics, CPU exceptions and unexpected interrupts. All LEDs on, the reason
//! on a dark red screen, then stop.

use core::fmt::Write;

use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Baseline, Text};

struct Wrap {
    lines: [[u8; 26]; 8],
    line: usize,
    col: usize,
}

impl Write for Wrap {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &c in s.as_bytes() {
            if self.line >= 8 {
                break;
            }
            if c == b'\n' || self.col == 26 {
                self.line += 1;
                self.col = 0;
                if c == b'\n' || self.line >= 8 {
                    continue;
                }
            }
            self.lines[self.line][self.col] = if (0x20..0x7F).contains(&c) { c } else { b'?' };
            self.col += 1;
        }
        Ok(())
    }
}

fn new_wrap() -> Wrap {
    Wrap { lines: [[b' '; 26]; 8], line: 0, col: 0 }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    hw::interrupts_off();
    let mut w = new_wrap();
    let _ = write!(w, "PANIC ");
    if let Some(loc) = info.location() {
        let file = loc.file();
        let short = file.rsplit('/').next().unwrap_or(file);
        let _ = write!(w, "{}:{}\n", short, loc.line());
    }
    let _ = write!(w, "{}", info.message());
    show(&w)
}

/// From rust_trap: an exception or an interrupt nobody handles.
pub fn fatal_trap(mcause: u32) -> ! {
    hw::interrupts_off();
    let (epc, tval, sp): (u32, u32, u32);
    unsafe {
        core::arch::asm!("csrr {0}, mepc", out(reg) epc);
        core::arch::asm!("csrr {0}, mtval", out(reg) tval);
        core::arch::asm!("mv {0}, sp", out(reg) sp);
    }
    let mut w = new_wrap();
    let kind = if mcause & 0x8000_0000 != 0 { "UNHANDLED IRQ" } else { "EXCEPTION" };
    let _ = write!(w, "{kind}\ncause {mcause:08x}\nepc   {epc:08x}\ntval  {tval:08x}\nsp    {sp:08x}");
    show(&w)
}

fn show(w: &Wrap) -> ! {
    crate::leds::stop();
    // All LEDs full on: rows low, columns high.
    hw::gpio::clear_mask(hw::gpio::B, (1 << 2) | (1 << 10) | (1 << 11));
    hw::gpio::set_mask(hw::gpio::C, 1 | (1 << 5) | (1 << 6));
    let frame = crate::frame();
    let _ = frame.clear(Rgb565::new(8, 0, 0));
    for (i, l) in w.lines.iter().enumerate() {
        let text = core::str::from_utf8(l).unwrap_or("");
        let _ = Text::with_baseline(text, Point::new(1, i as i32 * 10), MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE), Baseline::Top)
            .draw(frame);
    }
    hw::gpio::high(hw::gpio::B, 0); // end any half-sent frame
    crate::display::flush(frame);
    crate::display::panel(true);
    loop {}
}
