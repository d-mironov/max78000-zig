const microzig = @import("microzig");
pub const regs = microzig.chip.peripherals.GCR;
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

    // Reset all peripherals
    reset_peripherals();
}

/// Represents the *Clock State* (e.g. for a peripheral)
const ClockState = enum {
    /// Clock Enable
    enable,
    /// Clock Disable
    disable,
};

/// Set the peripherals clock state for a given peripheral
///
/// Parameters:
/// - `reg`: regsiter to modify
/// - `comptime field`: peripheral as a string (e.g. "ADC", "GPIO0", etc.)
/// - `state`: `.enable` to enable clock for peripheral, `.disable` to disable.
///
/// Example:
/// ```zig
/// const hal = @import("microzig").hal;
/// const gcr = hal.gcr;
///
/// // Enable clock for I2C0
/// gcr.set_pclk(regs.PCLKDIS0, "I2C0", .enable);
/// ```
fn set_pclk(reg: anytype, comptime field: []const u8, state: ClockState) void {
    switch (state) {
        .enable => reg.modify_one(field, .en),
        .disable => reg.modify_one(field, .dis),
    }
}

/// Set the peripherals clock state for a given peripheral
///
/// Parameters:
/// - `peripheral`: `Peripheral` to enable the clock for
/// - `state`: `.enable` to enable clock for peripheral, `.disable` to disable.
///
/// Example:
/// ```zig
/// const hal = @import("microzig").hal;
/// const gcr = hal.gcr;
///
/// // Enable clock for I2C0
/// gcr.set_clock(.i2c0, .enable);
/// ```
fn set_clock(peripheral: Peripheral, state: ClockState) void {
    switch (peripheral) {
        .adc => set_pclk(regs.PCLKDIS0, "ADC", state),
        .aes => set_pclk(regs.PCLKDIS1, "AES", state),
        .cnn => set_pclk(regs.PCLKDIS0, "CNN", state),
        .cpu1 => set_pclk(regs.PCLKDIS1, "CPU1", state),
        .crc => set_pclk(regs.PCLKDIS1, "CRC", state),
        .dma => set_pclk(regs.PCLKDIS0, "DMA", state),
        .gpio0 => set_pclk(regs.PCLKDIS0, "GPIO0", state),
        .gpio1 => set_pclk(regs.PCLKDIS0, "GPIO1", state),
        .gpio2 => set_pclk(lpgcr.PCLKDIS, "GPIO2", state),
        .i2c0 => set_pclk(regs.PCLKDIS0, "I2C0", state),
        .i2c1 => set_pclk(regs.PCLKDIS0, "I2C1", state),
        .i2c2 => set_pclk(regs.PCLKDIS1, "I2C2", state),
        .i2s0 => set_pclk(regs.PCLKDIS1, "I2S", state),
        .lpcomp => set_pclk(lpgcr.PCLKDIS, "LPCOMP", state),
        .owm => set_pclk(regs.PCLKDIS1, "OWM", state),
        .pt => set_pclk(regs.PCLKDIS0, "PT", state),
        .smphr => set_pclk(regs.PCLKDIS1, "SMPHR", state),
        .spi0 => set_pclk(regs.PCLKDIS1, "SPI0", state),
        .spi1 => set_pclk(regs.PCLKDIS0, "SPI1", state),
        .tmr0 => set_pclk(regs.PCLKDIS0, "TMR0", state),
        .tmr1 => set_pclk(regs.PCLKDIS0, "TMR1", state),
        .tmr2 => set_pclk(regs.PCLKDIS0, "TMR2", state),
        .tmr3 => set_pclk(regs.PCLKDIS0, "TMR3", state),
        .tmr4 => set_pclk(lpgcr.PCLKDIS, "TMR4", state),
        .tmr5 => set_pclk(lpgcr.PCLKDIS, "TMR5", state),
        .trng => set_pclk(regs.PCLKDIS1, "TRNG", state),
        .uart0 => set_pclk(regs.PCLKDIS0, "UART0", state),
        .uart1 => set_pclk(regs.PCLKDIS0, "UART1", state),
        .uart2 => set_pclk(regs.PCLKDIS1, "UART2", state),
        .uart3 => set_pclk(lpgcr.PCLKDIS, "UART3", state),
        .wdt0 => set_pclk(regs.PCLKDIS1, "WDT0", state),
        .wdt1 => set_pclk(lpgcr.PCLKDIS, "WDT1", state),
    }
}

/// Enable the clock of a `Peripheral`.
///
/// Parameters:
/// - `peripheral`: `Peripheral` to enable.
pub fn enable_clock(peripheral: Peripheral) void {
    set_clock(peripheral, .enable);
}

/// Disable the clock of a `Peripheral`.
///
/// Parameters:
/// - `peripheral`: `Peripheral` to disable.
pub fn disable_clock(peripheral: Peripheral) void {
    set_clock(peripheral, .disable);
}

/// Reset a `Peripheral`.
///
/// Parameters:
/// - `peripheral`: `Peripheral` to reset.
pub fn reset(peripheral: Peripheral) void {
    switch (peripheral) {
        .adc => regs.RST0.modify(.{ .ADC = 1 }),
        .aes => regs.RST1.modify(.{ .AES = 1 }),
        .cnn => regs.RST0.modify(.{ .CNN = 1 }),
        .cpu1 => regs.RST1.modify(.{ .CPU1 = 1 }),
        .crc => regs.RST1.modify(.{ .CRC = 1 }),
        .dma => regs.RST0.modify(.{ .DMA = 1 }),
        .gpio0 => regs.RST0.modify(.{ .GPIO0 = 1 }),
        .gpio1 => regs.RST0.modify(.{ .GPIO1 = 1 }),
        .gpio2 => lpgcr.RST.modify(.{ .GPIO2 = 1 }),
        .i2c0 => regs.RST0.modify(.{ .I2C0 = 1 }),
        .i2c1 => regs.RST1.modify(.{ .I2C1 = 1 }),
        .i2c2 => regs.RST1.modify(.{ .I2C2 = 1 }),
        .i2s0 => regs.RST1.modify(.{ .I2S = 1 }),
        .lpcomp => lpgcr.RST.modify(.{ .LPCOMP = 1 }),
        .owm => regs.RST1.modify(.{ .OWM = 1 }),
        .pt => regs.RST0.modify(.{ .PT = 1 }),
        .smphr => regs.RST1.modify(.{ .SMPHR = 1 }),
        .spi0 => regs.RST1.modify(.{ .SPI0 = 1 }),
        .spi1 => regs.RST0.modify(.{ .SPI1 = 1 }),
        .tmr0 => regs.RST0.modify(.{ .TMR0 = 1 }),
        .tmr1 => regs.RST0.modify(.{ .TMR1 = 1 }),
        .tmr2 => regs.RST0.modify(.{ .TMR2 = 1 }),
        .tmr3 => regs.RST0.modify(.{ .TMR3 = 1 }),
        .tmr4 => lpgcr.RST.modify(.{ .TMR4 = 1 }),
        .tmr5 => lpgcr.RST.modify(.{ .TMR5 = 1 }),
        .trng => regs.RST0.modify(.{ .TRNG = 1 }),
        .uart0 => regs.RST0.modify(.{ .UART0 = 1 }),
        .uart1 => regs.RST0.modify(.{ .UART1 = 1 }),
        .uart2 => regs.RST0.modify(.{ .UART2 = 1 }),
        .uart3 => lpgcr.RST.modify(.{ .UART3 = 1 }),
        .wdt0 => regs.RST0.modify(.{ .WDT0 = 1 }),
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

/// Resets all peripherals. CPU retains it's state.
/// GPIO, watchdog timers, AoD, RAM retention, and
/// general control registers (GCR), including the
/// clock configuration, are unaffected.
pub fn reset_peripherals() void {
    regs.RST0.modify(.{ .PERIPH = 1 });
}

/// Perform a soft reset. Same as peripheral reset
/// except that it also resets the `GPIO` to it POR
/// state.
pub fn soft_reset() void {
    regs.RST0.modify(.{ .SOFT = 1 });
}

/// Same as a soft reset, except that it also resets
/// all `GCR`, this resets the clocks to their POR state.
/// CPU state is reset, as well as the watchdog timers.
/// AoD and RAM are unaffected.
pub fn system_reset() void {
    regs.RST0.modify(.{ .SYS = 1 });
}

/// Represents the oscillator status.
pub const OscStatus = enum {
    /// Oscillator is disabled or otherwise unavailable.
    disabled,
    /// Oscillator is enabled but not yet ready.
    enabled,
    /// Oscillator is enabled and ready.
    ready,
};

pub const Oscillator = enum {
    /// 100MHz Internal Primary Oscillator (IPO)
    ipo,
    /// 60MHz Internal Secondary Oscillator (ISO)
    iso,
    /// 32.768kHz External RTC Oscillator (ERTCO)
    ertco,
    /// 8kHz Internal Nano-Ring Oscillator (INRO)
    inro,
    /// 7.3728MHz Internal Baud Rate Oscillator (IBRO)
    ibro,
};

/// Power down the 100MHz Internal Primary Oscillator (IPO) in
/// low-power mode (LPM).
pub fn lpm_ipo_power_off() void {
    regs.PM.modify(.{ .IPO_PD = 1 });
}

/// Power on 100MHz Internal Primary Oscillator (IPO) in
/// low-power mode (LPM).
pub fn lpm_ipo_power_on() void {
    regs.PM.modify(.{ .IPO_PD = 0 });
}

/// Status of the 100MHz Internal Primary Oscillator (IPO).
///
/// Example:
/// ```zig
/// const hal = @import("microzig").hal;
/// const gcr = hal.gcr;
///
/// while (gcr.ipo_status() != .ready) {}
/// ```
pub fn ipo_status() OscStatus {
    return osc_status(.ipo);
}

/// Power down the 60MHz Internal Secondary Oscillator (ISO) in
/// low-power mode (LPM).
pub fn lpm_iso_power_off() void {
    regs.PM.modify(.{ .ISO_PD = 1 });
}

/// Power on the 60MHz Internal Secondary Oscillator (ISO) in
/// low-power mode (LPM).
pub fn lpm_iso_power_on() void {
    regs.PM.modify(.{ .ISO_PD = 0 });
}

/// Status of the 60MHz Internal Secondary Oscillator (ISO).
///
/// Example:
/// ```zig
/// const hal = @import("microzig").hal;
/// const gcr = hal.gcr;
///
/// while (gcr.iso_status() != .ready) {}
/// ```
pub fn iso_status() OscStatus {
    return osc_status(.iso);
}

/// Status of the 7.3728MHz Internal Baud Rate Oscillator (IBRO).
pub fn ibro_status() OscStatus {
    return osc_status(.ibro);
}

fn osc_enabled_ready_status(enabled: bool, ready: bool) OscStatus {
    if (!enabled) return .disabled;
    if (!ready) return .enabled;
    return .ready;
}

/// Status of the selected oscillator.
///
/// `INRO` does not expose a dedicated enable bit in `CLKCTRL`, so its status is
/// derived from `INRO_RDY` alone: `.ready` when ready, otherwise `.disabled`.
pub fn osc_status(osc: Oscillator) OscStatus {
    const clkctrl = regs.CLKCTRL.read();

    return switch (osc) {
        .ertco => osc_enabled_ready_status(clkctrl.ERTCO_EN == .en, clkctrl.ERTCO_RDY == .ready),
        .ibro => osc_enabled_ready_status(clkctrl.IBRO_EN == .en, clkctrl.IBRO_RDY == .ready),
        .ipo => osc_enabled_ready_status(clkctrl.IPO_EN == .en, clkctrl.IPO_RDY == .ready),
        .iso => osc_enabled_ready_status(clkctrl.ISO_EN == .en, clkctrl.ISO_RDY == .ready),
        .inro => if (clkctrl.INRO_RDY == .ready) .ready else .disabled,
    };
}
