# MAX78000 HAL design notes

Audience: maintainers designing the long-term architecture of a handwritten MicroZig HAL for MAX78000.

This is not a bring-up checklist. It is a design document: what architecture to choose, what tradeoffs exist, and what Zig patterns are worth using.

---

## 1. Verified MicroZig model

### What MicroZig expects

- [verified] `chip` is generated low-level register access.
- [verified] `hal` is handwritten higher-level chip interaction.
- [verified] `board` is optional metadata / helpers for attached hardware.
- [verified] `microzig.hal` and `microzig.board` only exist when the target provides them.
- [verified] startup calls `app.init()` if present, else `microzig.hal.init()` if present.

Read locally:

- `/home/mironov/workspace/microzig/core/src/microzig.zig`
- `/home/mironov/workspace/microzig/core/src/start.zig`
- `/home/mironov/workspace/microzig/build-internals/build.zig`

### Immediate consequence for MAX78000

- [inferred] The HAL should be attached at the **chip** level.
- [inferred] EVKIT and FTHR should derive from the chip target and inherit that HAL.
- [inferred] Board files should stay thin and describe only board-specific facts.

That matches the general MicroZig model and avoids duplicating HAL code per board.

---

## 2. Architectural patterns worth copying

## 2.1 RP2040 / RP2350 pattern: shared family HAL + thin boards

Observed in:

- `/home/mironov/workspace/microzig/port/raspberrypi/rp2xxx/build.zig`
- `/home/mironov/workspace/microzig/port/raspberrypi/rp2xxx/src/hal.zig`
- `/home/mironov/workspace/microzig/port/raspberrypi/rp2xxx/src/boards/*.zig`

### Characteristics

- one shared `src/hal.zig`
- lots of small subsystem modules under `src/hal/`
- default `init()` plus customizable `init_sequence(...)`
- compile-time `HAL_Options`
- board files provide data like `xosc_freq` and pin aliases
- strong compile-time pin configuration API

### Why it is good

- coherent public API under `microzig.hal.*`
- easy for users to discover
- easy to grow incrementally
- board metadata stays reusable

### Why it can hurt

- pin APIs can become very compile-time heavy
- it is easy to over-engineer the pin matrix too early

### Relevance to MAX78000

- [recommended] Copy this overall shape.
- [recommended] Especially copy:
  - `hal.zig` as aggregator
  - `HAL_Options`
  - default `init()` + explicit `init_sequence(...)`
  - thin boards

---

## 2.2 STM32 pattern: family HALs + reusable common backends

Observed in:

- `/home/mironov/workspace/microzig/port/stmicro/stm32/build.zig`
- `/home/mironov/workspace/microzig/port/stmicro/stm32/src/hals/STM32F303.zig`
- `/home/mironov/workspace/microzig/port/stmicro/stm32/src/hals/common/pins_v2.zig`
- `/home/mironov/workspace/microzig/port/stmicro/stm32/src/hals/common/uart_v3.zig`

### Characteristics

- generated or cataloged chips
- HAL chosen per family / chip family
- shared common helper layers for GPIO/UART/I2C/SPI where hardware blocks are similar
- board files often contain board-specific helpers and aliases

### Why it is good

- scales well across many chips
- encourages reuse of common IP-block logic
- useful once multiple variants share the same UART/GPIO/IP design

### Why it can hurt

- more indirection
- easier to end up with too many abstractions for a single-chip package
- more maintenance burden before the first useful HAL lands

### Relevance to MAX78000

- [recommended] Borrow the *idea* of shared helpers.
- [not recommended yet] Do **not** start with a large common-backend architecture.
- [inferred] You only have one chip family in this repo today, so a simpler family HAL is the better starting point.

---

## 2.3 Minimal direct-register HALs from standalone Zig repos

External references:

- `d-mironov/stm32f4.zig`: <https://github.com/d-mironov/stm32f4.zig>
- `ikubaku/rp2040_zig`: <https://github.com/ikubaku/rp2040_zig>

### Characteristics

- direct handwritten register access
- low abstraction overhead
- often fewer layers between PAC and driver code

### Why it is good

- excellent for learning and first bring-up
- easy to debug
- little hidden machinery

### Why it can hurt

- public API often mirrors hardware too closely
- board ergonomics are usually weaker
- less reusable once the package grows

### Relevance to MAX78000

- [recommended] Use this mindset for the first implementation pass.
- [recommended] Keep the code direct and readable.
- [not recommended] Do not stop at a raw-register-only public API.

---

## 3. Recommended architecture for MAX78000

### Short version

Use a **hybrid**:

- RP2040/RP2350-style overall package structure
- STM32-style typed subsystem modules where helpful
- minimal direct-register implementation internally
- thin board metadata

### Recommended module layout

```text
src/
  hal.zig
  hal/
    gcr.zig
    gpio.zig
    pins.zig
    uart.zig
    i2c.zig
    spi.zig
    dma.zig
    time.zig
    flc.zig
    icc0.zig
    board_support.zig       # optional helper module later
```

### Responsibility split

- `microzig.chip.*`
  - generated register definitions only
- `microzig.hal.gcr`
  - clock source / divider policy
  - peripheral gating
  - peripheral reset / release
- `microzig.hal.gpio`
  - raw pin read/write/configure API
- `microzig.hal.pins`
  - ergonomic pin naming / compile-time config / mux setup
- `microzig.hal.uart`
  - typed UART wrapper
- `microzig.board`
  - pin aliases, board LEDs/buttons, crystal info, onboard devices

This split keeps the substrate (`gpio`, `gcr`) separate from the ergonomic layer (`pins`, `board`).

---

## 4. Major design problems to solve consciously

## 4.1 Pin muxing is usually the hardest part of embedded HAL ergonomics

### Problem

- [verified] RP2xxx has a strong compile-time pin matrix.
- [verified] STM32 common layers are more mode/AF oriented than full-matrix validated.
- [unknown] The full MAX78000 legal pin-function matrix is not yet represented in this repo.

### Recommendation

Start in stages:

### Stage 1
- explicit GPIO pin API
- explicit mux-setting API
- little or no automatic validation

### Stage 2
- partial validation for UART/I2C/SPI well-known routes

### Stage 3
- full compile-time matrix if you extract/verify the vendor pin table

This avoids spending a week designing a perfect API before one LED toggles.

---

## 4.2 Board-specific convenience vs chip-level portability

### Problem

A board-centric API is pleasant for examples, but it can make the HAL less reusable.

### Recommendation

- chip HAL should expose generic hardware building blocks
- board modules should expose aliases built on top of those blocks

Good:

```zig
const led = microzig.board.led;
led.set();
```

Also good:

```zig
const pin = microzig.hal.gpio.Pin{ .port = .gpio2, .index = 0 };
pin.set();
```

Bad:

- putting chip-wide GPIO logic directly in the EVKIT file
- making UART usable only through a board helper

---

## 4.3 Clocking and peripheral gating can leak everywhere if not centralized

### Problem

If every driver enables its own clocks in its own style, the HAL becomes inconsistent and hard to audit.

### Recommendation

Put all peripheral gate/reset logic in `gcr.zig`.

Each peripheral module may call into `gcr`, but it should not duplicate clock-bit knowledge.

That gives you:

- one source of truth for PCLKDIS/RST bit mappings
- easy auditing of bring-up order
- easier future low-power work

---

## 4.4 Generated register names are not a stable ergonomic API

### Problem

`regz`-generated names are useful, but they reflect the SVD. They are not always ideal as the public HAL API.

### Recommendation

- use generated register types internally
- present small, stable HAL wrapper APIs externally
- expose raw chip registers as the escape hatch, not the primary API

Example principle:

```zig
// Good public HAL API
try hal.uart.Uart(.uart0).init(.{ .baud_rate = 115200 });

// Raw escape hatch still available when needed
microzig.chip.peripherals.UART0
```

This lets the HAL remain stable even if SVD naming or `regz` output changes.

---

## 5. Zig language features worth leaning on

## 5.1 `comptime` instance-specialized types

This is one of the best fits for embedded Zig HALs.

Example pattern:

```zig
pub const Instance = enum { uart0, uart1, uart2, uart3 };

pub fn Uart(comptime instance: Instance) type {
    return struct {
        pub fn init(config: Config) !@This() {
            // instance-specific implementation
        }
    };
}
```

Why this is good:

- no runtime instance lookup overhead if you do not need it
- clear code generation
- easy to inline
- easy to attach instance-specific constants or regs

Use this for:

- UART
- I2C
- SPI
- DMA channels if practical

---

## 5.2 Configuration as plain data

Prefer plain config structs over long argument lists.

```zig
pub const Config = struct {
    baud_rate: u32 = 115200,
    parity: Parity = .none,
    stop_bits: StopBits = .one,
};
```

This makes:

- call sites readable
- defaults explicit
- tests easier to write
- future extension easier without API breakage

---

## 5.3 Tagged unions for pin mode / peripheral mode

Tagged unions are often the cleanest way to encode mutually exclusive hardware modes.

```zig
pub const Mode = union(enum) {
    input: InputConfig,
    output: OutputConfig,
    alternate: AlternateConfig,
    analog,
};
```

This is nicer than a single giant struct full of meaningless optional fields.

---

## 5.4 Compile-time validation when the input is compile-time known

Follow the RP2xxx style where appropriate:

- compile error for impossible pin/function pairs in comptime configs
- runtime error for dynamically chosen invalid configs

A useful rule:

- if the user passes a `comptime` board/pin configuration, prefer `@compileError`
- if the user chooses something at runtime, return `error.InvalidConfig`

This gives excellent ergonomics without overusing panics.

---

## 5.5 `std.Io.Writer` / adapter types for peripheral I/O

UART is a good candidate for this.

A small adapter type gives users:

- formatted printing
- logging integration
- buffering if needed

Look at patterns like STM32 common UART writers in:

- `/home/mironov/workspace/microzig/port/stmicro/stm32/src/hals/common/uart_v3.zig`

Use adapters as a thin layer on top of the core UART API; do not let them become the only API.

---

## 5.6 Avoid hidden allocation and hidden global state

Prefer:

- zero-allocation APIs
- explicit handles
- explicit initialization
- compile-time configuration

Avoid:

- hidden heaps
- opaque singleton state unless the hardware is truly singleton and that is the most honest model

Embedded users expect predictability.

---

## 6. Public API design recommendations

## 6.1 Keep the HAL namespace shallow and discoverable

Recommended top level:

```zig
microzig.hal.gcr
microzig.hal.gpio
microzig.hal.pins
microzig.hal.uart
microzig.hal.i2c
microzig.hal.spi
microzig.hal.time
```

Do not bury everything three namespaces deep without a strong reason.

---

## 6.2 Provide both ergonomic and raw layers

Recommended stack:

### Layer 0
- `microzig.chip.*` raw generated PAC

### Layer 1
- `microzig.hal.gcr`, `gpio`, `uart` typed wrappers around PAC

### Layer 2
- `microzig.hal.pins`, board aliases, convenience helpers

This keeps escape hatches available while still making common tasks pleasant.

---

## 6.3 Make init explicit and overridable

Recommended pattern:

```zig
pub const HAL_Options = struct {
    clocks: ClockOptions = .{},
};

pub fn init() void {
    init_sequence(default_clock_config());
}

pub fn init_sequence(cfg: ClockConfig) void {
    // known-safe bring-up sequence
}
```

Why this is good:

- default experience is easy
- advanced users can customize without forking the HAL
- board code can reuse the same sequence

This matches a proven RP2xxx pattern.

---

## 6.4 Keep board modules declarative where possible

Ideal board contents:

- constants
- aliases
- helper constructors
- maybe board-specific composite drivers

Not ideal:

- hidden chip bring-up policy
- chip-level peripheral code
- anything that makes the chip HAL unusable without a specific board

---

## 7. Suggested long-term milestones

### Milestone A: core bring-up
- `hal.zig`
- `gcr.zig`
- `gpio.zig`
- `pins.zig`
- `uart.zig`

### Milestone B: timing and buses
- `time.zig`
- `i2c.zig`
- `spi.zig`

### Milestone C: performance and storage
- `dma.zig`
- `icc0.zig`
- `flc.zig`

### Milestone D: board polish
- EVKIT aliases
- FTHR aliases
- console helpers
- onboard device helpers

### Milestone E: stricter validation
- compile-time pin matrix
- stronger clock validation
- interrupt defaults where helpful

---

## 8. Testing strategy for a good HAL

A good embedded HAL should be tested at three levels.

## 8.1 Pure host tests

Test anything not requiring live MMIO:

- divider math
- baud computation
- pin/function validation tables
- config validation

## 8.2 Compile-only API tests

Test that example code compiles:

- pin config application
- UART creation
- board alias use

## 8.3 On-target smoke tests

At minimum:

- LED blink
- button read
- UART TX
- UART RX loopback

Do not rely only on host tests; MMIO code needs hardware confirmation.

---

## 9. Recommended API style for MAX78000 specifically

If you want a short answer, this is the design I would implement.

### Core rules

1. chip-level HAL, thin boards
2. one `src/hal.zig` root
3. `gcr` owns gating/reset
4. `gpio` owns raw pin I/O
5. `pins` owns ergonomic configuration
6. `uart` owns typed serial API
7. use compile-time validation only where you have trustworthy data
8. keep raw PAC access available through `microzig.chip`

### Example desired user experience

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
    }
}
```

Later, once board aliases are verified:

```zig
const microzig = @import("microzig");

pub fn main() !void {
    const led = microzig.board.led;
    led.init(.{ .direction = .output });

    while (true) {
        led.toggle();
    }
}
```

This gives both portability and ergonomics.

---

## 10. Further reading

### Local references

- MicroZig layering:
  - `/home/mironov/workspace/microzig/core/src/microzig.zig`
- MicroZig startup flow:
  - `/home/mironov/workspace/microzig/core/src/start.zig`
- Target `.hal` / `.board` definitions:
  - `/home/mironov/workspace/microzig/build-internals/build.zig`
- RP2xxx HAL root:
  - `/home/mironov/workspace/microzig/port/raspberrypi/rp2xxx/src/hal.zig`
- RP2xxx pins:
  - `/home/mironov/workspace/microzig/port/raspberrypi/rp2xxx/src/hal/pins.zig`
- RP2xxx UART:
  - `/home/mironov/workspace/microzig/port/raspberrypi/rp2xxx/src/hal/uart.zig`
- STM32 F303 HAL root:
  - `/home/mironov/workspace/microzig/port/stmicro/stm32/src/hals/STM32F303.zig`
- STM32 reusable pins:
  - `/home/mironov/workspace/microzig/port/stmicro/stm32/src/hals/common/pins_v2.zig`
- STM32 reusable UART:
  - `/home/mironov/workspace/microzig/port/stmicro/stm32/src/hals/common/uart_v3.zig`

### External references

- MicroZig internals: <https://microzig.tech/docs/internals/>
- MicroZig HAL design issue: <https://github.com/ZigEmbeddedGroup/microzig/issues/208>
- MicroZig board target discussion: <https://github.com/ZigEmbeddedGroup/microzig/issues/919>
- ZigEmbeddedGroup legacy RP2040 HSP: <https://github.com/ZigEmbeddedGroup/raspberrypi-rp2040>
- ADI MSDK: <https://github.com/analogdevicesinc/msdk>
- MAX78000 peripheral driver docs: <https://analogdevicesinc.github.io/msdk/Libraries/PeriphDrivers/Documentation/MAX78000/>
- MAX78000 datasheet: <https://www.analog.com/media/en/technical-documentation/data-sheets/max78000.pdf>
- MAX78000 user guide: <https://www.analog.com/media/en/technical-documentation/user-guides/max78000-user-guide.pdf>
- MAX78000EVKIT docs: <https://www.analog.com/media/en/technical-documentation/data-sheets/max78000evkit.pdf>
- MAX78000FTHR docs: <https://www.analog.com/media/en/technical-documentation/data-sheets/max78000fthr.pdf>

---

## 11. Decision summary

If you want one architecture decision to lock in now, make it this:

> Build a chip-level MAX78000 HAL with a small, typed public API; keep board files declarative; use the generated PAC internally; and adopt an RP2xxx-style `hal.zig` + subsystem layout.

That direction is the best balance of:

- MicroZig compatibility
- easy bring-up
- good ergonomics
- future extensibility
- low initial complexity
