# MAX78000 HAL bring-up plan

Audience: maintainers of this package who want to start implementing a handwritten `microzig.hal` for MAX78000.

This document is intentionally practical. It tells you what is still missing, what files to create, what order to do the work in, and what to read before each step.

---

## 1. Current state

### Already in place

- [verified] A self-contained MicroZig chip package exists in `build.zig`.
- [verified] The package exports:
  - `chips.max78000`
  - `boards.max78000evkit`
  - `boards.max78000fthr`
- [verified] Register generation works from `svd/max78000.svd`.
- [verified] The temporary SVD expander is wired into the build in `tools/expand_max78000_svd.zig`.
- [verified] The package is consumable as a downstream dependency.

### Still missing before HAL work can start cleanly

- [verified] No HAL is attached to the target yet.
- [verified] There is no `src/hal.zig`.
- [verified] There are no handwritten HAL modules under `src/hal/`.
- [verified] Current board files are metadata-only placeholders:
  - `src/boards/max78000evkit.zig`
  - `src/boards/max78000fthr.zig`

### Important MicroZig behavior to keep in mind

- [verified] `microzig.hal` only exists if the target provides `.hal = .{ .root_source_file = ... }`.
  - Read: `/home/mironov/workspace/microzig/build-internals/build.zig`
- [verified] On startup, MicroZig calls `app.init()` if present, otherwise `microzig.hal.init()` if present.
  - Read: `/home/mironov/workspace/microzig/core/src/start.zig`
- [verified] `microzig.board` is optional and board init is **not** auto-called by MicroZig.
  - Read: `/home/mironov/workspace/microzig/core/src/microzig.zig`

Implication: the first HAL milestone should be **attach a chip-level HAL** and keep board code thin.

---

## 2. Recommended first milestone

Do **not** start with a huge HAL. The first milestone should be:

1. HAL wiring in `build.zig`
2. `src/hal.zig`
3. `src/hal/gcr.zig`
4. `src/hal/gpio.zig`
5. `src/hal/pins.zig`
6. `src/hal/uart.zig`
7. one smoke example that imports `microzig.hal`

That is enough to validate:

- HAL injection into MicroZig
- default init flow
- clock/reset gating strategy
- pin configuration API shape
- one real peripheral API

Do **not** start with DMA, flash, I2C, SPI, CNN, or low-power features until the GPIO + pinmux + UART architecture feels right.

---

## 3. Read this first

### MicroZig architecture references

Read these local files before writing code:

- `build.zig` in this repo
- `/home/mironov/workspace/microzig/core/src/microzig.zig`
- `/home/mironov/workspace/microzig/core/src/start.zig`
- `/home/mironov/workspace/microzig/build-internals/build.zig`

Read these as implementation patterns:

- RP2040/RP2350 shared HAL root:
  - `/home/mironov/workspace/microzig/port/raspberrypi/rp2xxx/src/hal.zig`
- RP2040/RP2350 pin configuration model:
  - `/home/mironov/workspace/microzig/port/raspberrypi/rp2xxx/src/hal/pins.zig`
- RP2040/RP2350 UART style:
  - `/home/mironov/workspace/microzig/port/raspberrypi/rp2xxx/src/hal/uart.zig`
- STM32 family HAL aggregator:
  - `/home/mironov/workspace/microzig/port/stmicro/stm32/src/hals/STM32F303.zig`
- STM32 reusable typed pin layer:
  - `/home/mironov/workspace/microzig/port/stmicro/stm32/src/hals/common/pins_v2.zig`
- STM32 board helper style:
  - `/home/mironov/workspace/microzig/port/stmicro/stm32/src/boards/STM32F3DISCOVERY.zig`

### MAX78000 silicon / vendor references

Start with official sources:

- MAX78000 datasheet: <https://www.analog.com/media/en/technical-documentation/data-sheets/max78000.pdf>
- MAX78000 user guide: <https://www.analog.com/media/en/technical-documentation/user-guides/max78000-user-guide.pdf>
- ADI MSDK repo: <https://github.com/analogdevicesinc/msdk>
- MSDK MAX78000 peripheral driver docs: <https://analogdevicesinc.github.io/msdk/Libraries/PeriphDrivers/Documentation/MAX78000/>
- MSDK MAX78000 boards: <https://github.com/analogdevicesinc/msdk/tree/main/Libraries/Boards/MAX78000>
- MAX78000EVKIT docs: <https://www.analog.com/media/en/technical-documentation/data-sheets/max78000evkit.pdf>
- MAX78000FTHR docs: <https://www.analog.com/media/en/technical-documentation/data-sheets/max78000fthr.pdf>

Use the datasheet/user guide for hardware truth. Use MSDK for vendor naming, init order, and BSP conventions.

---

## 4. What to change in this repo first

### 4.1 Attach a HAL to the target

Change `build.zig` so the chip target has a HAL root.

### Suggested shape

```zig
fn max78000Target(
    dep: *std.Build.Dependency,
    svd: std.Build.LazyPath,
    hal_root: std.Build.LazyPath,
) Target {
    return .{
        .dep = dep,
        .preferred_binary_format = .elf,
        .zig_target = .{
            .cpu_arch = .thumb,
            .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m4 },
            .cpu_features_add = std.Target.arm.featureSet(&.{ .vfp4d16sp }),
            .os_tag = .freestanding,
            .abi = .eabihf,
        },
        .chip = .{
            .name = "max78000",
            .url = "https://www.analog.com/en/products/max78000.html",
            .register_definition = .{ .svd = svd },
            .memory_regions = &.{
                .{ .name = "FLASH", .tag = .flash, .offset = 0x10000000, .length = 512 * 1024, .access = .rx },
                .{ .name = "SRAM", .tag = .ram, .offset = 0x20000000, .length = 128 * 1024, .access = .rwx },
            },
        },
        .hal = .{
            .root_source_file = hal_root,
        },
        .linker_script = .{
            .generate = .{ .memory_regions_and_sections = .{
                .rodata_location = .flash,
            } },
        },
    };
}
```

Then call it like this:

```zig
pub fn init(dep: *std.Build.Dependency) Self {
    const b = dep.builder;
    const target = max78000Target(
        dep,
        expandedSvdFromDep(dep),
        b.path("src/hal.zig"),
    );

    // ... existing return
}

pub fn build(b: *std.Build) void {
    const mz_dep = b.dependency("microzig", .{});
    const mb = MicroBuild.init(b, mz_dep) orelse return;

    const target = max78000Target(
        mz_dep,
        expandedSvdFromBuild(b),
        b.path("src/hal.zig"),
    );

    // ... existing example build
}
```

This is the minimum change required to make `microzig.hal` exist.

---

## 5. Suggested file tree to create

```text
src/
  hal.zig
  hal/
    gcr.zig
    gpio.zig
    pins.zig
    uart.zig
    compatibility.zig        # optional; useful if variants appear later
    time.zig                 # later
    i2c.zig                  # later
    spi.zig                  # later
    dma.zig                  # later
    flc.zig                  # later
    icc0.zig                 # later
```

### Why this layout

- `src/hal.zig` is the stable public namespace for `microzig.hal.*`
- `src/hal/*.zig` keeps one subsystem per file
- `src/boards/*.zig` should remain board metadata / aliases, not become the HAL itself

---

## 6. Starter scaffolding

The point of this section is not to give you final code. It gives you the minimum code shape so you can start filling in real register logic.

### 6.1 `src/hal.zig`

```zig
const builtin = @import("builtin");
const microzig = @import("microzig");

pub const gcr = @import("hal/gcr.zig");
pub const gpio = @import("hal/gpio.zig");
pub const pins = @import("hal/pins.zig");
pub const uart = @import("hal/uart.zig");

pub const HAL_Options = struct {
    clocks: struct {
        // Keep this simple at first.
        // Replace with a richer clock policy once GCR is understood.
        use_default_internal_clocking: bool = true,
    } = .{},

    cache: struct {
        enable_icc0: bool = false,
    } = .{},
};

pub fn init() void {
    init_sequence();
}

pub fn init_sequence() void {
    // TODO: replace with real bring-up sequence once verified against the user guide/MSDK.
    gcr.init_defaults();
}

pub const default_interrupts: microzig.cpu.InterruptOptions = .{};

test {
    _ = gcr;
    _ = gpio;
    _ = pins;
    _ = uart;

    if (!builtin.is_test) {
        // Keep room for any future exported intrinsics or linker-visible items.
    }
}
```

Use this file as the top-level public API.

### 6.2 `src/hal/gcr.zig`

Purpose:

- system clock policy
- peripheral clock gating
- peripheral reset/unreset
- eventually low-power helpers if they naturally belong here

```zig
const microzig = @import("microzig");
const regs = microzig.chip.peripherals.GCR;

pub const Peripheral = enum {
    gpio0,
    gpio1,
    gpio2,
    uart0,
    uart1,
    uart2,
    uart3,
    i2c0,
    i2c1,
    i2c2,
    spi0,
    spi1,
    // Extend as needed.
};

pub fn init_defaults() void {
    // TODO:
    // - verify reset/default clock state from the user guide + MSDK
    // - disable/enable watchdogs only when justified
    // - establish a known peripheral clock baseline
}

pub fn enable_clock(peripheral: Peripheral) void {
    _ = peripheral;
    // TODO: map Peripheral -> GCR.PCLKDIS0/PCLKDIS1 bit
}

pub fn disable_clock(peripheral: Peripheral) void {
    _ = peripheral;
    // TODO: map Peripheral -> GCR.PCLKDIS0/PCLKDIS1 bit
}

pub fn reset(peripheral: Peripheral) void {
    _ = peripheral;
    // TODO: map Peripheral -> GCR.RST0/RST1 bit
}

pub fn unreset(peripheral: Peripheral) void {
    _ = peripheral;
    // TODO: map Peripheral -> GCR.RST0/RST1 bit
}

pub fn enable_and_reset_release(peripheral: Peripheral) void {
    enable_clock(peripheral);
    unreset(peripheral);
}
```

This module should become the one authoritative place for peripheral gating and reset sequencing.

### 6.3 `src/hal/gpio.zig`

Purpose:

- direct pin I/O wrapper over `GPIO0`, `GPIO1`, `GPIO2`
- port/pin identification
- high-level pin methods (`set`, `clear`, `toggle`, `read`)

```zig
const microzig = @import("microzig");
const gcr = @import("gcr.zig");

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
            .gpio0 => .gpio0,
            .gpio1 => .gpio1,
            .gpio2 => .gpio2,
        });
        // TODO: write real GPIO registers here.
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
```

Keep GPIO simple and boring. You need a reliable substrate for everything else.

### 6.4 `src/hal/pins.zig`

This module should provide the *ergonomic* API, not the raw register substrate.

Recommended first version:

- explicit pin enum or `Port + index`
- compile-time `GlobalConfiguration`
- optional pin names for board aliases
- pin function selection once the matrix is understood

```zig
const std = @import("std");
const gpio = @import("gpio.zig");

pub const Function = enum {
    gpio,
    uart,
    i2c,
    spi,
    // Split this into more exact functions once the matrix is known.
};

pub const PinId = struct {
    port: gpio.Port,
    index: u5,

    pub fn to_gpio(self: PinId) gpio.Pin {
        return .{ .port = self.port, .index = self.index };
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
        // First version can be simple.
        // Later you can make this return a typed namespace of named pins,
        // similar in spirit to RP2xxx.
        for (self.entries) |entry| {
            if (entry.config.gpio_config) |cfg| {
                entry.id.to_gpio().init(cfg);
            }
            // TODO: apply mux/function selection here once defined.
        }

        return struct {};
    }
};
```

Do not over-design `pins.zig` on day 1. Start with a plain shape and evolve it once you know the MAX78000 pin matrix.

### 6.5 `src/hal/uart.zig`

Purpose:

- instance enum
- config struct
- init/get_or_init
- `tx`, `rx`
- optional `std.Io.Writer` adapter later

```zig
const std = @import("std");
const microzig = @import("microzig");
const gcr = @import("gcr.zig");

pub const Instance = enum {
    uart0,
    uart1,
    uart2,
    uart3,
};

pub const Parity = enum { none, even, odd };
pub const StopBits = enum { one, two };

pub const Config = struct {
    baud_rate: u32 = 115200,
    parity: Parity = .none,
    stop_bits: StopBits = .one,
};

pub const ConfigError = error{
    UnsupportedBaudRate,
    InvalidConfig,
};

pub fn Uart(comptime instance: Instance) type {
    return struct {
        const Self = @This();

        pub fn init(config: Config) ConfigError!Self {
            _ = config;

            gcr.enable_and_reset_release(switch (instance) {
                .uart0 => .uart0,
                .uart1 => .uart1,
                .uart2 => .uart2,
                .uart3 => .uart3,
            });

            // TODO:
            // - locate register block for selected UART instance
            // - apply framing / baud configuration
            // - enable TX/RX
            return Self{};
        }

        pub fn tx_byte(self: Self, byte: u8) void {
            _ = self;
            _ = byte;
            // TODO
        }

        pub fn tx(self: Self, bytes: []const u8) void {
            for (bytes) |b| self.tx_byte(b);
        }

        pub fn rx_byte(self: Self) u8 {
            _ = self;
            // TODO
            return 0;
        }
    };
}

pub fn Writer(comptime instance: Instance) type {
    return struct {
        uart: Uart(instance),

        pub fn writeAll(self: *@This(), bytes: []const u8) void {
            self.uart.tx(bytes);
        }
    };
}
```

The key decision here is the *instance-parameterized type*. That works well in Zig and matches proven MicroZig patterns.

---

## 7. What to put in board files once the HAL exists

Keep `src/boards/*.zig` thin.

Good contents:

- crystal frequency constants if needed
- board LED aliases
- board button aliases
- default UART selection / console pins
- sensor/display attached-device helpers

Avoid putting real peripheral implementation logic in board files.

### Suggested board skeleton

```zig
const microzig = @import("microzig");
const hal = microzig.hal;

pub const led = hal.gpio.Pin{
    .port = .gpio2,
    .index = 0, // TODO: verify against board schematic
};

pub const console_uart = hal.uart.Instance.uart0; // TODO: verify
```

Only add real aliases after checking:

- EVKIT/FTHR docs
- MSDK board support package
- board schematics

---

## 8. Verification plan from day 1

Do not wait until the HAL is large before testing it.

### 8.1 Build-time checks

After HAL wiring:

- `zig build`
- downstream consumer build using this package

This confirms:

- `microzig.hal` exists
- the package still works as a dependency
- the SVD expander + regz + HAL pipeline all compose

### 8.2 Host-testable unit tests

Put pure logic tests in modules where possible:

- baud-rate validation
- enum-to-register mapping helpers
- pin table validation
- clock divider calculations

Follow the RP2xxx UART style here:

- `/home/mironov/workspace/microzig/port/raspberrypi/rp2xxx/src/hal/uart.zig`

### 8.3 Compile-only smoke example

Add one example whose only purpose is to prove the public HAL API compiles.

Suggested future file:

```text
examples/hal-smoke.zig
```

Suggested shape:

```zig
const microzig = @import("microzig");
const hal = microzig.hal;

pub fn main() !void {
    const led = hal.gpio.Pin{ .port = .gpio2, .index = 0 };
    led.init(.{ .direction = .output });

    const uart = try hal.uart.Uart(.uart0).init(.{ .baud_rate = 115200 });
    uart.tx("hello\r\n");

    while (true) {
        led.toggle();
        asm volatile ("wfi");
    }
}
```

Do not promise the pin numbers are correct until verified from board docs.

### 8.4 Real-hardware validation order

1. GPIO output on a known LED
2. GPIO input on a known button
3. UART TX
4. UART RX
5. only then I2C/SPI

This order gives you fast feedback on the most foundational layers.

---

## 9. Suggested bring-up order by PR / work chunk

### Chunk 1
- wire HAL into `build.zig`
- add `src/hal.zig`
- add empty `gcr`, `gpio`, `pins`, `uart` modules
- make `zig build` pass again

### Chunk 2
- implement GCR peripheral gate/reset helpers
- validate against user guide + MSDK

### Chunk 3
- implement raw GPIO pin API
- get one LED blinking

### Chunk 4
- implement `pins.GlobalConfiguration`
- add board aliases for the first board you target

### Chunk 5
- implement UART init + TX
- get serial console output

### Chunk 6
- clean naming and public API
- only then add I2C / SPI / time / DMA

---

## 10. Architectural traps to avoid during bring-up

- [inferred] Do not let board files become the HAL.
- [inferred] Do not start by mirroring the entire vendor C API 1:1.
- [inferred] Do not expose only raw registers; add small typed wrappers immediately.
- [inferred] Do not build a giant global init function that configures every peripheral.
- [inferred] Do not design the whole pinmux system before one GPIO + one UART path works.
- [inferred] Do not depend on undocumented reset/default states; encode them explicitly in `gcr` once verified.

---

## 11. Further reading

### Local MicroZig references

- HAL / board / chip layering:
  - `/home/mironov/workspace/microzig/core/src/microzig.zig`
- startup init order:
  - `/home/mironov/workspace/microzig/core/src/start.zig`
- target fields for `.hal` and `.board`:
  - `/home/mironov/workspace/microzig/build-internals/build.zig`
- shared-family HAL example:
  - `/home/mironov/workspace/microzig/port/raspberrypi/rp2xxx/src/hal.zig`
- strong compile-time pin config example:
  - `/home/mironov/workspace/microzig/port/raspberrypi/rp2xxx/src/hal/pins.zig`
- typed UART example:
  - `/home/mironov/workspace/microzig/port/raspberrypi/rp2xxx/src/hal/uart.zig`
- reusable family/common pin layer example:
  - `/home/mironov/workspace/microzig/port/stmicro/stm32/src/hals/common/pins_v2.zig`

### External references

- MicroZig internals: <https://microzig.tech/docs/internals/>
- MicroZig HAL design discussion: <https://github.com/ZigEmbeddedGroup/microzig/issues/208>
- MicroZig board-target ergonomics discussion: <https://github.com/ZigEmbeddedGroup/microzig/issues/919>
- ADI MSDK: <https://github.com/analogdevicesinc/msdk>
- MAX78000 peripheral docs: <https://analogdevicesinc.github.io/msdk/Libraries/PeriphDrivers/Documentation/MAX78000/>

---

## 12. Bottom line

The package is ready for HAL work once you do one plumbing change: attach `src/hal.zig` to the target.

After that, keep the first milestone small:

- `gcr`
- `gpio`
- `pins`
- `uart`

If those four feel clean, the rest of the HAL will have a stable foundation.
