//! The LEDs: three RGB LEDs in a 3x3 matrix, driven like the stock firmware. Rows PB2, PB10,
//! PB11 (open-drain, low = on) are colour channels, columns PC6, PC5, PC0 (push-pull,
//! high = on) are the LEDs left, middle, right. TIM2 steps through 3 rows x 8 bit planes
//! (binary code modulation), plane k lit for BASE_US << k.

use core::sync::atomic::{AtomicU16, Ordering};

use hw::{gpio, r32, w32};

const ROW_PIN: [u32; 3] = [2, 10, 11]; // GPIOB
const COL_PIN: [u32; 3] = [6, 5, 0]; // GPIOC: left, middle, right
const ALL_ROWS: u32 = (1 << 2) | (1 << 10) | (1 << 11);
const ALL_COLS: u32 = (1 << 0) | (1 << 5) | (1 << 6);

/// Matrix row of red, green, blue for each LED (found by test; the same for all three).
const RGB_ROW: [[usize; 3]; 3] = [[0, 1, 2], [0, 1, 2], [0, 1, 2]];

const TIM2: u32 = 0x4000_0000;
const TIM_CTLR1: u32 = TIM2;
const TIM_DMAINTENR: u32 = TIM2 + 0x0C;
const TIM_INTFR: u32 = TIM2 + 0x10;
const TIM_SWEVGR: u32 = TIM2 + 0x14;
const TIM_CNT: u32 = TIM2 + 0x24;
const TIM_PSC: u32 = TIM2 + 0x28;
const TIM_ATRLR: u32 = TIM2 + 0x2C;
pub const IRQ: u32 = 44;

/// Timer tick 1 us (timer clock 144 MHz / 144). Plane 0 = 2 us; a full frame is 1.5 ms.
const BASE_US: u32 = 2;

/// [row * 8 + plane] = column mask to light in that slot.
static SLOTS: [AtomicU16; 24] = [const { AtomicU16::new(0) }; 24];
static mut SLOT: usize = 0;

/// Set the three LEDs, (r, g, b) 0..255 each, left to right.
pub fn set(rgb: &[[u8; 3]; 3]) {
    let mut levels = [[0u8; 3]; 3]; // [row][col]
    for led in 0..3 {
        for c in 0..3 {
            levels[RGB_ROW[led][c]][led] = rgb[led][c];
        }
    }
    for row in 0..3 {
        for plane in 0..8 {
            let mut cols = 0u16;
            for col in 0..3 {
                if levels[row][col] & (1 << plane) != 0 {
                    cols |= 1 << COL_PIN[col];
                }
            }
            SLOTS[row * 8 + plane].store(cols, Ordering::Relaxed);
        }
    }
}

pub fn start() {
    hw::rcc::enable_apb2(hw::rcc::IOPB | hw::rcc::IOPC);
    for p in ROW_PIN {
        gpio::high(gpio::B, p);
        gpio::mode(gpio::B, p, gpio::OUT_OD);
    }
    for p in COL_PIN {
        gpio::low(gpio::C, p);
        gpio::mode(gpio::C, p, gpio::OUT_PP);
    }
    hw::rcc::enable_apb1(hw::rcc::TIM2);
    w32(TIM_CTLR1, 0);
    w32(TIM_PSC, 143);
    w32(TIM_ATRLR, BASE_US - 1);
    w32(TIM_SWEVGR, 1); // load PSC
    w32(TIM_INTFR, 0);
    w32(TIM_CNT, 0);
    w32(TIM_DMAINTENR, 1); // UIE
    hw::irq_enable(IRQ);
    w32(TIM_CTLR1, 1); // CEN
}

/// Stop the multiplexing (the panic handler turns all LEDs on by hand).
pub fn stop() {
    w32(TIM_CTLR1, 0);
}

pub fn isr() {
    w32(TIM_INTFR, 0);
    let slot = unsafe { SLOT };
    let row = slot >> 3;
    let plane = slot & 7;
    gpio::clear_mask(gpio::C, ALL_COLS);
    gpio::set_mask(gpio::B, ALL_ROWS);
    let cols = SLOTS[slot].load(Ordering::Relaxed) as u32;
    if cols != 0 {
        gpio::clear_mask(gpio::B, 1 << ROW_PIN[row]);
        gpio::set_mask(gpio::C, cols);
    }
    w32(TIM_ATRLR, (BASE_US << plane) - 1);
    unsafe { SLOT = if slot == 23 { 0 } else { slot + 1 } };
    let _ = r32(TIM_INTFR); // make sure the flag clear landed before mret
}
