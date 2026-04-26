const microzig = @import("microzig");
const gcr = microzig.chip.peripherals.GCR;
const lpgcr = microzig.chip.peripherals.LPGCR;

pub const GcrError = error{};

pub const Peripheral = enum {
    adc,
    aes,
    cnn,
    cpu1,
    crc,
    dma,
    gpio0,
    gpio1,
    gpio2,
    i2c0,
    i2c1,
    i2c2,
    i2s0,
    lpcomp,
    owm,
    pt,
    smphr,
    spi0,
    spi1,
    trng,
    tmr0,
    tmr1,
    tmr2,
    tmr3,
    tmr4,
    tmr5,
    uart0,
    uart1,
    uart2,
    uart3,
    wdt0,
    wdt1,
    // extends when more peripherals are added
};

pub fn init_defaults() void {
    // TODO:
    // - verify reset/default clock state from the user guide + MSDK
    // - disable/enable watchdogs when justified
    // - establish a known peripheral clock baseline
}

const ClockState = enum {
    enable,
    disable,
};

fn set_pclk(reg: anytype, comptime field: []const u8, state: ClockState) void {
    switch (state) {
        .enable => reg.modify_one(field, .en),
        .disable => reg.modify_one(field, .dis),
    }
}

fn set_clock(peripheral: Peripheral, state: ClockState) void {
    switch (peripheral) {
        .adc => set_pclk(gcr.PCLKDIS0, "ADC", state),
        .aes => set_pclk(gcr.PCLKDIS1, "AES", state),
        .cnn => set_pclk(gcr.PCLKDIS0, "CNN", state),
        .cpu1 => set_pclk(gcr.PCLKDIS1, "CPU1", state),
        .crc => set_pclk(gcr.PCLKDIS1, "CRC", state),
        .dma => set_pclk(gcr.PCLKDIS0, "DMA", state),
        .gpio0 => set_pclk(gcr.PCLKDIS0, "GPIO0", state),
        .gpio1 => set_pclk(gcr.PCLKDIS0, "GPIO1", state),
        .gpio2 => set_pclk(lpgcr.PCLKDIS, "GPIO2", state),
        .i2c0 => set_pclk(gcr.PCLKDIS0, "I2C0", state),
        .i2c1 => set_pclk(gcr.PCLKDIS0, "I2C1", state),
        .i2c2 => set_pclk(gcr.PCLKDIS1, "I2C2", state),
        .i2s0 => set_pclk(gcr.PCLKDIS1, "I2S", state),
        .lpcomp => set_pclk(lpgcr.PCLKDIS, "LPCOMP", state),
        .owm => set_pclk(gcr.PCLKDIS1, "OWM", state),
        .pt => set_pclk(gcr.PCLKDIS0, "PT", state),
        .smphr => set_pclk(gcr.PCLKDIS1, "SMPHR", state),
        .spi0 => set_pclk(gcr.PCLKDIS1, "SPI0", state),
        .spi1 => set_pclk(gcr.PCLKDIS0, "SPI1", state),
        .tmr0 => set_pclk(gcr.PCLKDIS0, "TMR0", state),
        .tmr1 => set_pclk(gcr.PCLKDIS0, "TMR1", state),
        .tmr2 => set_pclk(gcr.PCLKDIS0, "TMR2", state),
        .tmr3 => set_pclk(gcr.PCLKDIS0, "TMR3", state),
        .tmr4 => set_pclk(lpgcr.PCLKDIS, "TMR4", state),
        .tmr5 => set_pclk(lpgcr.PCLKDIS, "TMR5", state),
        .trng => set_pclk(gcr.PCLKDIS1, "TRNG", state),
        .uart0 => set_pclk(gcr.PCLKDIS0, "UART0", state),
        .uart1 => set_pclk(gcr.PCLKDIS0, "UART1", state),
        .uart2 => set_pclk(gcr.PCLKDIS1, "UART2", state),
        .uart3 => set_pclk(lpgcr.PCLKDIS, "UART3", state),
        .wdt0 => set_pclk(gcr.PCLKDIS1, "WDT0", state),
        .wdt1 => set_pclk(lpgcr.PCLKDIS, "WDT1", state),
    }
}

pub fn enable_clock(peripheral: Peripheral) void {
    set_clock(peripheral, .enable);
}

pub fn disable_clock(peripheral: Peripheral) void {
    set_clock(peripheral, .disable);
}

pub fn reset(peripheral: Peripheral) void {
    switch (peripheral) {
        .adc => gcr.RST0.modify(.{ .ADC = 1 }),
        .aes => gcr.RST1.modify(.{ .AES = 1 }),
        .cnn => gcr.RST0.modify(.{ .CNN = 1 }),
        .cpu1 => gcr.RST1.modify(.{ .CPU1 = 1 }),
        .crc => gcr.RST1.modify(.{ .CRC = 1 }),
        .dma => gcr.RST0.modify(.{ .DMA = 1 }),
        .gpio0 => gcr.RST0.modify(.{ .GPIO0 = 1 }),
        .gpio1 => gcr.RST0.modify(.{ .GPIO1 = 1 }),
        .gpio2 => lpgcr.RST.modify(.{ .GPIO2 = 1 }),
        .i2c0 => gcr.RST0.modify(.{ .I2C0 = 1 }),
        .i2c1 => gcr.RST1.modify(.{ .I2C1 = 1 }),
        .i2c2 => gcr.RST1.modify(.{ .I2C2 = 1 }),
        .i2s0 => gcr.RST1.modify(.{ .I2S = 1 }),
        .lpcomp => lpgcr.RST.modify(.{ .LPCOMP = 1 }),
        .owm => gcr.RST1.modify(.{ .OWM = 1 }),
        .pt => gcr.RST0.modify(.{ .PT = 1 }),
        .smphr => gcr.RST1.modify(.{ .SMPHR = 1 }),
        .spi0 => gcr.RST1.modify(.{ .SPI0 = 1 }),
        .spi1 => gcr.RST0.modify(.{ .SPI1 = 1 }),
        .tmr0 => gcr.RST0.modify(.{ .TMR0 = 1 }),
        .tmr1 => gcr.RST0.modify(.{ .TMR1 = 1 }),
        .tmr2 => gcr.RST0.modify(.{ .TMR2 = 1 }),
        .tmr3 => gcr.RST0.modify(.{ .TMR3 = 1 }),
        .tmr4 => lpgcr.RST.modify(.{ .TMR4 = 1 }),
        .tmr5 => lpgcr.RST.modify(.{ .TMR5 = 1 }),
        .trng => gcr.RST0.modify(.{ .TRNG = 1 }),
        .uart0 => gcr.RST0.modify(.{ .UART0 = 1 }),
        .uart1 => gcr.RST0.modify(.{ .UART1 = 1 }),
        .uart2 => gcr.RST0.modify(.{ .UART2 = 1 }),
        .uart3 => lpgcr.RST.modify(.{ .UART3 = 1 }),
        .wdt0 => gcr.RST0.modify(.{ .WDT0 = 1 }),
        .wdt1 => lpgcr.RST.modify(.{ .WDT1 = 1 }),
    }
}

pub fn unreset(peripheral: Peripheral) void {
    _ = peripheral;
    // TODO: map peripheral -> GCR.PCLKDIS0/PCLKDIS1 bit
}

pub fn enable_and_reset_release(peripheral: Peripheral) void {
    enable_clock(peripheral);
    unreset(peripheral);
}

fn disable_ipo() void {
    gcr.PM.modify(.{ .IPO_PD = 1 });
}

fn enable_ipo() void {
    gcr.PM.modify(.{ .IPO_PD = 0 });
}

/// Resets all peripherals. CPU retains it's state.
/// GPIO, watchdog timers, AoD, RAM retention, and
/// general control registers (GCR), including the
/// clock configuration, are unaffected.
fn reset_peripherals() void {
    gcr.RST0.modify(.{ .PERIPH = 1 });
}

/// Perform a soft reset. Same as peripheral reset
/// except that it also resets the `GPIO` to it POR
/// state.
fn soft_reset() void {
    gcr.RST0.modify(.{ .SOFT = 1 });
}

/// Same as a soft reset, except that it also resets
/// all `GCR`, this resets the clocks to their POR state.
/// CPU state is reset, as well as the watchdog timers.
/// AoD and RAM are unaffected.
fn system_reset() void {
    gcr.RST0.modify(.{ .SYS = 1 });
}
