const microzig = @import("microzig");
const gcr = @import("gcr/gcr.zig");

pub const Port = enum {
    gpio0,
    gpio1,
    gpio2,
};

pub const Direction = enum {
    input,
    output,
};

pub const Pull = enum {
    none,
    up,
    down,
};

pub const OutputMode = enum {
    push_pull,
    open_drain,
};

pub const Config = struct {
    direction: Direction,
    pull: Pull = .none,
    output_mode: OutputMode = .push_pull,
};

pub const Pin = struct {
    port: Port,
    index: u5,

    pub fn init(self: Pin, config: Config) void {
        _ = config;
        gcr.enable_and_reset_release(switch (self.port) {
            .gpio0 => .gpoi0,
            .gpio1 => .gpio1,
            .gpio2 => .gpio2,
        });
        // TODO: write registers
    }

    pub fn put(self: Pin, value: bool) void {
        if (value) self.set() else self.clear();
    }

    pub fn set(self: Pin) void {
        _ = self;
        // TODO
    }

    pub fn clear(self: Pin) void {
        _ = self;
        // TODO
    }

    pub fn toggle(self: Pin) void {
        self.put(!self.get());
    }

    pub fn get(self: Pin) bool {
        _ = self;
        // TODO
        return false;
    }
};
