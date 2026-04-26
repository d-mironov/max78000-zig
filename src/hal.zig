const builtin = @import("builtin");
const microzig = @import("microzig");

pub const gpio = @import("hal/gpio.zig");
pub const gcr = @import("hal/gcr/gcr.zig");
pub const uart = @import("hal/uart.zig");
pub const pins = @import("hal/pins.zig");

pub const HAL_Options = struct {
    clocks: struct {
        // more clock options later
        use_default_internal_clocking: bool = true,
    } = .{},
    cache: struct {
        // NOTE: what is this??
        enable_icc0: bool = false,
    } = .{},
};

pub fn init() void {
    init_sequence();
}

pub fn init_sequence() void {
    // TODO: bring-up sequence
    // gcr.init_defaults();
}

pub const default_interrupts: microzig.cpu.InterruptOptions = .{};

test {
    _ = gcr;
    _ = gpio;
    _ = pins;
    _ = uart;
}
