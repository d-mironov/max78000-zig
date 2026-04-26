const std = @import("std");
const gpio = @import("gpio.zig");

pub const Function = enum {
    gpio,
    uart,
    i2c,
    spi,
    // consult pin matrix
};

pub const PinId = struct {
    port: gpio.Port,
    index: u5,

    pub fn to_gpio(self: PinId) gpio.Pin {
        return .{ .port = self.port, .index = .self.index };
    }
};

pub const Configuration = struct {
    name: ?[:0]const u8 = null,
    function: Function = .gpio,
    gpio_config: ?gpio.Config = null,
};

pub const GlobalConfiguration = struct {
    entries: []const struct {
        id: PinId,
        config: Configuration,
    },

    pub fn apply(comptime self: GlobalConfiguration) type {
        // First version simple
        for (self.entries) |entry| {
            if (entry.config.gpio_config) |cfg| {
                entry.id.to_gpio().init(cfg);
            }
            // TODO: apply mux/function selection once defined
        }

        return struct {};
    }
};
