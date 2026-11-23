//! Just enough CH32V307 for the O3C: startup, one trap entry, clocks, GPIO, SysTick,
//! ADC1, flash. Register sequences follow WCH's EVT library (system_ch32v30x.c,
//! ch32v30x_flash.c) and the reference manual. No interrupt controller tricks: one trap
//! entry, registers saved in software, no nesting, no hardware register stack (HPE).
//!
//! The binary provides `extern "C" fn rust_main() -> !` and `extern "C" fn rust_trap(mcause: u32)`.
#![no_std]

use core::ptr::{read_volatile as rd, write_volatile as wr};

core::arch::global_asm!(
    r#"
    .section .init, "ax"
    .global _start
_start:
    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop
    la sp, _stack_top
    csrci mstatus, 8
    li t0, 0x1f
    csrw 0xbc0, t0  # CORECFGR as WCH's startup: prefetch and friends
    csrw 0x804, zero  # INTSYSCR: no hardware register stack, no nesting
    la t0, _trap_entry
    csrw mtvec, t0  # direct mode: every trap to _trap_entry
    la a0, _sidata
    la a1, _sdata
    la a2, _edata
1:  bgeu a1, a2, 2f
    lw t0, 0(a0)
    sw t0, 0(a1)
    addi a0, a0, 4
    addi a1, a1, 4
    j 1b
2:  la a0, _sbss
    la a1, _ebss
3:  bgeu a0, a1, 4f
    sw zero, 0(a0)
    addi a0, a0, 4
    j 3b
4:  call rust_main
5:  j 5b

    .section .trap, "ax"
    .align 2
    .global _trap_entry
_trap_entry:
    addi sp, sp, -64
    sw ra, 0(sp)
    sw t0, 4(sp)
    sw t1, 8(sp)
    sw t2, 12(sp)
    sw a0, 16(sp)
    sw a1, 20(sp)
    sw a2, 24(sp)
    sw a3, 28(sp)
    sw a4, 32(sp)
    sw a5, 36(sp)
    sw a6, 40(sp)
    sw a7, 44(sp)
    sw t3, 48(sp)
    sw t4, 52(sp)
    sw t5, 56(sp)
    sw t6, 60(sp)
    csrr a0, mcause
    call rust_trap
    lw ra, 0(sp)
    lw t0, 4(sp)
    lw t1, 8(sp)
    lw t2, 12(sp)
    lw a0, 16(sp)
    lw a1, 20(sp)
    lw a2, 24(sp)
    lw a3, 28(sp)
    lw a4, 32(sp)
    lw a5, 36(sp)
    lw a6, 40(sp)
    lw a7, 44(sp)
    lw t3, 48(sp)
    lw t4, 52(sp)
    lw t5, 56(sp)
    lw t6, 60(sp)
    addi sp, sp, 64
    mret
"#
);

#[inline(always)]
pub fn r32(a: u32) -> u32 {
    unsafe { rd(a as *const u32) }
}
#[inline(always)]
pub fn w32(a: u32, v: u32) {
    unsafe { wr(a as *mut u32, v) }
}
#[inline(always)]
pub fn m32(a: u32, clear: u32, set: u32) {
    w32(a, (r32(a) & !clear) | set)
}

pub fn interrupts_on() {
    unsafe { core::arch::asm!("csrsi mstatus, 8") };
}
pub fn interrupts_off() {
    unsafe { core::arch::asm!("csrci mstatus, 8") };
}

/// Enable an external interrupt in the PFIC.
pub fn irq_enable(irq: u32) {
    w32(0xE000_E100 + 4 * (irq / 32), 1 << (irq % 32));
}

pub fn system_reset() -> ! {
    w32(0xE000_E048, 0xBEEF_0080); // PFIC_CFGR: KEY3 | SYSRESET
    loop {}
}

pub fn uid() -> [u8; 12] {
    let mut out = [0u8; 12];
    for (i, b) in out.iter_mut().enumerate() {
        *b = unsafe { rd((0x1FFF_F7E8 + i as u32) as *const u8) };
    }
    out
}

pub mod rcc {
    use super::*;
    pub const CTLR: u32 = 0x4002_1000;
    pub const CFGR0: u32 = 0x4002_1004;
    pub const AHBPCENR: u32 = 0x4002_1014;
    pub const APB2PCENR: u32 = 0x4002_1018;
    pub const APB1PCENR: u32 = 0x4002_101C;

    /// 144 MHz from the 8 MHz crystal: PLL x18 (PLLMUL 0000 on the V30x D8C). AHB /1
    /// (144 MHz), APB2 /2 (72 MHz), APB1 /2 (72 MHz, timers 144 MHz), ADC 72 / 8 = 9 MHz.
    /// Returns false if the crystal did not start (then it runs on HSI x18 = 144 MHz).
    pub fn init_144mhz() -> bool {
        m32(CTLR, 0, 1); // HSION
        // Reset clock config as WCH's SystemInit.
        w32(CFGR0, r32(CFGR0) & 0xF8FF_0000);
        m32(CTLR, (1 << 16) | (1 << 19) | (1 << 24), 0); // HSEON, CSSON, PLLON off
        m32(CTLR, 1 << 18, 0); // HSEBYP off
        m32(CFGR0, 0x007F_0000, 0); // PLLSRC, PLLXTPRE, PLLMUL, USBPRE
        w32(0x4002_1008, 0x00FF_0000); // clear interrupt flags

        m32(CTLR, 0, 1 << 16); // HSEON
        let mut hse = false;
        for _ in 0..0x8000 {
            if r32(CTLR) & (1 << 17) != 0 {
                hse = true;
                break;
            }
        }
        // AHB /1, APB1 /2, APB2 /2, ADC /8
        m32(CFGR0, (0xF << 4) | (0x7 << 8) | (0x7 << 11) | (0x3 << 14), (0x4 << 8) | (0x4 << 11) | (0x3 << 14));
        if hse {
            m32(CFGR0, 0x003F_0000, 1 << 16); // PLLSRC = HSE, PLLXTPRE /1, PLLMUL x18
        } else {
            m32(0x4002_3800, 0, 1 << 4); // EXTEN_CTR: HSI not halved for the PLL
            m32(CFGR0, 0x003F_0000, 0); // PLLSRC = HSI, x18
        }
        m32(CTLR, 0, 1 << 24); // PLLON
        while r32(CTLR) & (1 << 25) == 0 {}
        m32(CFGR0, 0x3, 0x2); // SW = PLL
        while r32(CFGR0) & 0xC != 0x8 {}
        hse
    }

    pub fn enable_apb2(bits: u32) {
        m32(APB2PCENR, 0, bits);
    }
    pub fn enable_apb1(bits: u32) {
        m32(APB1PCENR, 0, bits);
    }
    pub const IOPA: u32 = 1 << 2;
    pub const IOPB: u32 = 1 << 3;
    pub const IOPC: u32 = 1 << 4;
    pub const ADC1: u32 = 1 << 9;
    pub const SPI1: u32 = 1 << 12;
    pub const TIM2: u32 = 1 << 0;
}

pub mod gpio {
    use super::*;
    pub const A: u32 = 0x4001_0800;
    pub const B: u32 = 0x4001_0C00;
    pub const C: u32 = 0x4001_1000;

    pub const ANALOG: u32 = 0x0;
    pub const INPUT_PULL: u32 = 0x8;
    pub const OUT_PP: u32 = 0x3;
    pub const OUT_OD: u32 = 0x7;
    pub const AF_PP: u32 = 0xB;

    /// Pin configuration nibble (CNF[1:0] MODE[1:0]).
    pub fn mode(port: u32, pin: u32, cfg: u32) {
        let reg = if pin < 8 { port } else { port + 4 };
        let sh = (pin % 8) * 4;
        m32(reg, 0xF << sh, cfg << sh);
    }
    pub fn pull_up(port: u32, pin: u32) {
        mode(port, pin, INPUT_PULL);
        high(port, pin);
    }
    #[inline(always)]
    pub fn high(port: u32, pin: u32) {
        w32(port + 0x10, 1 << pin);
    }
    #[inline(always)]
    pub fn low(port: u32, pin: u32) {
        w32(port + 0x14, 1 << pin);
    }
    #[inline(always)]
    pub fn set_mask(port: u32, mask: u32) {
        w32(port + 0x10, mask);
    }
    #[inline(always)]
    pub fn clear_mask(port: u32, mask: u32) {
        w32(port + 0x14, mask);
    }
    #[inline(always)]
    pub fn read(port: u32) -> u32 {
        r32(port + 0x08)
    }
}

/// SysTick as a free-running 64-bit counter at HCLK (144 MHz). No interrupt.
pub mod time {
    use super::*;
    const CTLR: u32 = 0xE000_F000;
    const CNTL: u32 = 0xE000_F008;
    const CNTH: u32 = 0xE000_F00C;
    pub const HZ: u64 = 144_000_000;

    pub fn init() {
        w32(CTLR, 0);
        w32(CNTL, 0);
        w32(CNTH, 0);
        w32(CTLR, (1 << 5) | (1 << 2) | 1); // INIT, HCLK, STE; up-counting, no reload
    }
    pub fn ticks() -> u64 {
        loop {
            let h = r32(CNTH);
            let l = r32(CNTL);
            if r32(CNTH) == h {
                return ((h as u64) << 32) | l as u64;
            }
        }
    }
    pub fn us() -> u64 {
        ticks() / (HZ / 1_000_000)
    }
    pub fn ms() -> u64 {
        ticks() / (HZ / 1000)
    }
    pub fn delay_us(n: u64) {
        let end = ticks() + n * (HZ / 1_000_000);
        while ticks() < end {}
    }
    pub fn delay_ms(n: u64) {
        delay_us(n * 1000);
    }
}


/// ADC1, single software-started conversions.
pub mod adc1 {
    use super::*;
    const BASE: u32 = 0x4001_2400;
    const STATR: u32 = BASE;
    const CTLR2: u32 = BASE + 0x08;
    const SAMPTR1: u32 = BASE + 0x0C;
    const SAMPTR2: u32 = BASE + 0x10;
    const RSQR1: u32 = BASE + 0x2C;
    const RSQR3: u32 = BASE + 0x34;
    const RDATAR: u32 = BASE + 0x4C;

    pub fn init() {
        rcc::enable_apb2(rcc::ADC1);
        // EXTTRIG with EXTSEL = SWSTART (111), ADON
        w32(CTLR2, (1 << 20) | (0b111 << 17) | 1);
        w32(RSQR1, 0); // one conversion
        m32(CTLR2, 0, 1 << 3); // RSTCAL
        for _ in 0..100_000 {
            if r32(CTLR2) & (1 << 3) == 0 {
                break;
            }
        }
        m32(CTLR2, 0, 1 << 2); // CAL
        for _ in 0..100_000 {
            if r32(CTLR2) & (1 << 2) == 0 {
                break;
            }
        }
    }
    /// Sample time code (0 = 1.5 .. 7 = 239.5 cycles) for a channel.
    pub fn sample_time(chan: u32, code: u32) {
        if chan < 10 {
            m32(SAMPTR2, 7 << (chan * 3), code << (chan * 3));
        } else {
            m32(SAMPTR1, 7 << ((chan - 10) * 3), code << ((chan - 10) * 3));
        }
    }
    /// One conversion; None on timeout.
    pub fn read(chan: u32) -> Option<u16> {
        w32(RSQR3, chan);
        w32(STATR, 0);
        m32(CTLR2, 0, 1 << 22); // SWSTART
        for _ in 0..20_000 {
            if r32(STATR) & 2 != 0 {
                return Some(r32(RDATAR) as u16);
            }
        }
        None
    }
}

/// Flash: 4 KiB sector erase, 256-byte fast page programming. Addresses are CPU
/// addresses (0x0000_0000 + n); the controller wants 0x0800_0000 + n.
pub mod flash {
    use super::*;
    const KEYR: u32 = 0x4002_2004;
    const STATR: u32 = 0x4002_200C;
    const CTLR: u32 = 0x4002_2010;
    const ADDR: u32 = 0x4002_2014;
    const MODEKEYR: u32 = 0x4002_2024;
    const KEY1: u32 = 0x4567_0123;
    const KEY2: u32 = 0xCDEF_89AB;
    const BASE: u32 = 0x0800_0000;

    const PER: u32 = 1 << 1;
    const STRT: u32 = 1 << 6;
    const LOCK: u32 = 1 << 7;
    const FLOCK: u32 = 1 << 15;
    const PAGE_PG: u32 = 1 << 16;
    const PGSTART: u32 = 1 << 21;
    const BSY: u32 = 1 << 0;
    const WR_BSY: u32 = 1 << 1;
    const WRPRTERR: u32 = 1 << 4;
    const EOP: u32 = 1 << 5;

    pub const SECTOR: u32 = 4096;
    pub const PAGE: u32 = 256;

    fn wait() -> bool {
        while r32(STATR) & BSY != 0 {}
        let s = r32(STATR);
        w32(STATR, EOP | WRPRTERR);
        s & WRPRTERR == 0
    }

    fn unlock() {
        if r32(CTLR) & LOCK != 0 {
            w32(KEYR, KEY1);
            w32(KEYR, KEY2);
        }
        if r32(CTLR) & FLOCK != 0 {
            w32(MODEKEYR, KEY1);
            w32(MODEKEYR, KEY2);
        }
    }

    fn lock() {
        m32(CTLR, 0, LOCK | FLOCK);
    }

    /// Erase the 4 KiB sector holding `addr`.
    pub fn erase_sector(addr: u32) -> bool {
        unlock();
        wait();
        m32(CTLR, 0, PER);
        w32(ADDR, BASE | (addr & !(SECTOR - 1)));
        m32(CTLR, 0, STRT);
        let ok = wait();
        m32(CTLR, PER, 0);
        lock();
        ok
    }

    /// Program one 256-byte page at `addr` (page aligned, erased).
    pub fn program_page(addr: u32, data: &[u8; 256]) -> bool {
        unlock();
        wait();
        m32(CTLR, 0, PAGE_PG);
        let mut a = BASE | addr;
        for w in data.chunks_exact(4) {
            w32(a, u32::from_le_bytes([w[0], w[1], w[2], w[3]]));
            while r32(STATR) & WR_BSY != 0 {}
            a += 4;
        }
        m32(CTLR, 0, PGSTART);
        let ok = wait();
        m32(CTLR, PAGE_PG, 0);
        lock();
        ok && unsafe { core::slice::from_raw_parts(addr as *const u8, 256) } == &data[..]
    }

    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    pub enum WriteError {
        /// Not started, out of order, crossing a page, or past the end.
        Order,
        Flash,
    }

    /// Streams an image into erased flash, in order, in pieces that do not cross a page; each
    /// 256-byte page is programmed when it is full. Used by the bootloader (the app) and by
    /// the app (the bootloader).
    pub struct PageWriter {
        base: u32,
        limit: u32,
        page: [u8; PAGE as usize],
        page_off: u32,
        next: u32,
        active: bool,
    }

    impl PageWriter {
        pub const fn new() -> Self {
            Self { base: 0, limit: 0, page: [0xFF; PAGE as usize], page_off: 0, next: 0, active: false }
        }

        /// Start an image at `base`, at most `limit` bytes. The caller has erased the range.
        pub fn start(&mut self, base: u32, limit: u32) {
            *self = Self { base, limit, active: true, ..Self::new() };
        }

        fn program(&mut self) -> Result<(), WriteError> {
            if !program_page(self.base + self.page_off, &self.page) {
                return Err(WriteError::Flash);
            }
            self.page = [0xFF; PAGE as usize];
            self.page_off += PAGE;
            Ok(())
        }

        /// The piece at `off` (from `base`); it must start where the last one ended.
        pub fn write(&mut self, off: u32, data: &[u8]) -> Result<(), WriteError> {
            let end = off + data.len() as u32;
            if !self.active || off != self.next || end > self.limit || end > self.page_off + PAGE {
                return Err(WriteError::Order);
            }
            let at = (off - self.page_off) as usize;
            self.page[at..at + data.len()].copy_from_slice(data);
            self.next = end;
            if end == self.page_off + PAGE {
                self.program()?;
            }
            Ok(())
        }

        /// Program the last partial page. Returns the image as it is in flash, `len` bytes,
        /// read through the 0x0800_0000 mirror (address 0 is a null pointer to Rust).
        pub fn finish(&mut self, len: u32) -> Result<&'static [u8], WriteError> {
            if !self.active || len != self.next || len == 0 {
                return Err(WriteError::Order);
            }
            if self.next > self.page_off {
                self.program()?;
            }
            self.active = false;
            Ok(unsafe { core::slice::from_raw_parts((BASE | self.base) as *const u8, len as usize) })
        }
    }
}

