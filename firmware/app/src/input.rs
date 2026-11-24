//! Hall keys (ADC1: PC3 = IN13 left, PC2 = IN12 middle, PC1 = IN11 right; rest about 2100,
//! a full press about +/-800) and the knob on PB12 (A), PB13 (B), PB14 (switch), pull-up.

pub const KEYS: usize = 3;

/// Bit numbers in the `pins` byte (bit0 = PB12, bit1 = PB13, bit2 = PB14).
pub const KNOB_A: u8 = 0;
pub const KNOB_B: u8 = 1;
pub const KNOB_SW: u8 = 2;
/// Quadrature transitions per detent.
pub const STEPS_PER_DETENT: i8 = 4;

pub const DEFAULT_PRESS: u16 = 400;
pub const DEFAULT_RELEASE: u16 = 250;

#[derive(Clone, Copy)]
pub struct Key {
    pub rest: u32, // x16 fixed point
    pub raw: u16,
    pub travel: u16,
    pub press: u16,
    pub release: u16,
    pub down: bool,
}

impl Key {
    pub const fn new() -> Self {
        Self { rest: 0, raw: 0, travel: 0, press: DEFAULT_PRESS, release: DEFAULT_RELEASE, down: false }
    }

    #[inline(always)]
    pub fn rest(&self) -> u16 {
        (self.rest >> 4) as u16
    }

    /// Unsigned travel distance from resting baseline (supports both magnet polarities).
    #[inline(always)]
    pub fn delta(&self) -> u16 {
        self.raw.abs_diff(self.rest())
    }

    /// Returns Some(new state) on a change.
    #[inline(always)]
    pub fn update(&mut self, raw: u16) -> Option<bool> {
        self.raw = raw;
        let d = self.delta();
        if d > self.travel {
            self.travel = d;
        }
        if !self.down && d >= self.press {
            self.down = true;
            return Some(true);
        }
        if self.down && d <= self.release {
            self.down = false;
            return Some(false);
        }
        None
    }

    /// Follow slow thermal drift of the rest value (~1/256 per ms). Call at 1 kHz when released.
    #[inline(always)]
    pub fn drift(&mut self) {
        let d = self.delta();
        if !self.down && d < self.release / 2 {
            let target = (self.raw as u32) << 4;
            self.rest = self.rest - (self.rest >> 8) + (target >> 8);
        }
    }
}

/// Quadrature decoder; returns detents (+1 / -1) when complete.
pub struct Knob {
    last: u8,
    acc: i8,
}

impl Knob {
    pub const fn new() -> Self {
        Self { last: 0xFF, acc: 0 }
    }

    #[inline(always)]
    pub fn update(&mut self, a: bool, b: bool) -> i8 {
        let s = ((a as u8) << 1) | b as u8;
        if self.last == 0xFF {
            self.last = s;
            return 0;
        }
        // Gray code order 00 -> 01 -> 11 -> 10 -> 00 is counter-clockwise on this knob.
        const T: [i8; 16] = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0];
        let step = T[((self.last << 2) | s) as usize];
        self.last = s;
        self.acc += step;
        if self.acc >= STEPS_PER_DETENT {
            self.acc = 0;
            1
        } else if self.acc <= -STEPS_PER_DETENT {
            self.acc = 0;
            -1
        } else {
            0
        }
    }
}
