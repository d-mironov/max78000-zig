//! GPIO0/1/2 abstraction for MAX78000.
//!
//! Three layers live in this file:
//! - Layer 1: Port descriptor table (register pointer, clock token, IRQ)
//! - Layer 2: Primitive register operations (atomic SET/CLR, staged AF)
//! - Layer 3: Pin handle (the Pin enum + it's methods)
//!
//! GPIO3 is a different peripheral. WIP.
//!
//! Appendix (from MAX78000 User Guide):
//!
//! All I/O default to GPIO mode during a power-on-reset (POR) event as
//! high impedance inputs except the `SWDIO` and `SWDCLK` pins. After a
//! POR, the *SWD* is enabled by default with `AF1` selected by hardware.
//!
//! Following a POR event, all GPIO, execpt device pins that have the
//! `SWDIO` and `SWDCLK` function, are configured with the following
//! default settings:
//! - GPIO mode enabled
//!   - `GPIOn.EN0.en[pin] = 1`
//!   - `GPIOn.EN1.en[pin] = 0`
//!   - `GPIOn.EN2.en[pin] = 0`
//! - Pullup/pulldown disabled, I/O in Hi-Z mode
//!   - `GPIOn.PADCTRL0.mode[pin] = 0`
//!   - `GPIOn.PADCTRL1.mode[pin]`
//! - Output mode disabled
//!   - `GPIOn.OUTEN.en[pin] = 0`
//! - Interrupt disabled
//!   - `GPIOn.INTEN.en[pin] = 0`

const microzig = @import("microzig");
const peripherals = microzig.chip.peripherals;
const gcr = @import("gcr/gcr.zig");

pub const Port = enum(u2) {
    gpio0,
    gpio1,
    gpio2,
};

/// Pin index inside a port (0..31). Each port has a different valid range,
/// validated in `pins.zig`.
pub const Index = u5;

/// All three peripheral instances share this register layout.
const GpioRegs = @TypeOf(peripherals.GPIO0.*);

const PortDescriptor = struct {
    /// Pointer to the PAC peripheral instance
    regs: *volatile GpioRegs,
    /// Clock-gating token consumed by `gcr`.
    clock: gcr.Peripheral,
    /// IRQn for per-port interrupt vector
    irq: u8,
};

inline fn descriptor(comptime port: Port) PortDescriptor {
    return switch (port) {
        .gpio0 => .{ .regs = peripherals.GPIO0, .clock = .gpio0, .irq = 40 },
        .gpio1 => .{ .regs = peripherals.GPIO1, .clock = .gpio1, .irq = 41 },
        .gpio2 => .{ .regs = peripherals.GPIO2, .clock = .gpio2, .irq = 42 },
    };
}

/// Shared wake IRQ for all GPIOs.
pub const gpio_wake_irq: u8 = 70;

/// Every logically valid pin on the die. Encoded as `(port << 5) | index`,
/// so `port = @intFromEnum(p) >> 5` and `index = @intFromEnum(p) & 0x1F`.
///
/// Bonded-out subset depends on package and is enforced by `pins.zig`.
pub const Pin = enum(u7) {
    // GPIO0: P0.0..P0.30
    P0_0 = 0,
    P0_1,
    P0_2,
    P0_3,
    P0_4,
    P0_5,
    P0_6,
    P0_7,
    P0_8,
    P0_9,
    P0_10,
    P0_11,
    P0_12,
    P0_13,
    P0_14,
    P0_15,
    P0_16,
    P0_17,
    P0_18,
    P0_19,
    P0_20,
    P0_21,
    P0_22,
    P0_23,
    P0_24,
    P0_25,
    P0_26,
    P0_27,
    P0_28,
    P0_29,
    P0_30,

    // GPIO1: P1.0..P1.9
    P1_0 = 32,
    P1_1,
    P1_2,
    P1_3,
    P1_4,
    P1_5,
    P1_6,
    P1_7,
    P1_8,
    P1_9,

    // GPIO2: P2.0..P2.7
    P2_0 = 64,
    P2_1,
    P2_2,
    P2_3,
    P2_4,
    P2_5,
    P2_6,
    P2_7,

    pub inline fn port(self: Pin) Port {
        return @enumFromInt(@intFromEnum(self) >> 5);
    }

    pub inline fn index(self: Pin) Index {
        return @truncate(@intFromEnum(self) & 0x1F);
    }

    pub inline fn mask(self: Pin) u32 {
        return @as(u32, 1) << self.index();
    }
};

pub const Mode = union(enum) {
    input: InputConfig,
    output: OutputConfig,
    alternate: AlternateConfig,
};

pub const InputConfig = struct {};

pub const OutputConfig = struct {};

pub const AlternateConfig = struct {};

pub const AlternateMode = enum(u2) {
    gpio,
    af1,
    af2,
};
