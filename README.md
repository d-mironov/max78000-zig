# max78000-zig

> Self-contained [MicroZig](https://github.com/ZigEmbeddedGroup/microzig) chip-support package for the **Analog Devices MAX78000** — an ultra-low-power AI microcontroller with a Cortex-M4F application core.

![Zig](https://img.shields.io/badge/Zig-0.15.2-f7a41d?logo=zig&logoColor=white)
![MicroZig](https://img.shields.io/badge/MicroZig-0.15.1-5c6bc0?logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAyNCAyNCI+PC9zdmc+)
![Platform](https://img.shields.io/badge/CPU-Cortex--M4F-blue?logo=arm&logoColor=white)
![Status](https://img.shields.io/badge/HAL-in%20progress-yellow)

---

## Table of Contents

- [What's included](#whats-included)
- [Target summary](#target-summary)
- [Quick start](#quick-start)
- [Using as a dependency](#using-as-a-dependency)
- [Package API](#package-api)
- [HAL modules](#hal-modules)
- [SVD preprocessing](#svd-preprocessing)
- [Repository layout](#repository-layout)
- [Template usage](#template-usage)
- [HAL development notes](#hal-development-notes)
- [Memory map sources](#memory-map-sources)

---

## What's included

| Export | Description |
|---|---|
| `chips.max78000` | Chip-level `microzig.Target` for the Cortex-M4F core |
| `boards.max78000evkit` | Board-level target for the [MAX78000EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max78000evkit.html) |
| `boards.max78000fthr` | Board-level target for the [MAX78000FTHR](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max78000fthr.html) |
| Register definitions | Generated at build time from `svd/max78000.svd` via a custom expander |
| Linker script | Auto-generated for internal flash + SRAM layout |
| Example firmware | `examples/empty.zig` — a minimal buildable starting point |

---

## Target summary

| Property | Value |
|---|---|
| CPU | ARM Cortex-M4F |
| ABI | `eabihf` |
| Flash base | `0x10000000` |
| Flash size | 512 KiB |
| SRAM base | `0x20000000` |
| SRAM size | 128 KiB |

> **Note:** The vendor DTS describes SRAM as four contiguous banks (32 + 32 + 48 + 16 KiB). This package exposes them as a single 128 KiB region for MicroZig.

---

## Quick start

### Prerequisites

- Zig `0.15.2` or later — [download](https://ziglang.org/download/)

### Build the example firmware

```sh
git clone <repo-url>
cd max78000-zig
zig build
```

Produces:

```
zig-out/firmware/empty.elf
```

---

## Using as a dependency

### 1. Add to `build.zig.zon`

```zig
.dependencies = .{
    .microzig = .{
        .url = "https://github.com/ZigEmbeddedGroup/microzig/releases/download/0.15.1/microzig.tar.gz",
        .hash = "microzig-0.15.1-D20YSQYfCACrGl4NnHj6WXgGJUJvoEvLOJKBzeBzt-qX",
    },
    .max78000_zig = .{
        .url = "<package-url>",
        .hash = "<package-hash>",
    },
},
```

### 2. Wire up `build.zig`

```zig
const std = @import("std");
const microzig = @import("microzig");
const max78000_zig = @import("max78000_zig");

const MicroBuild = microzig.MicroBuild(.{});

pub fn build(b: *std.Build) void {
    const mz_dep = b.dependency("microzig", .{});
    const max_dep = b.dependency("max78000_zig", .{});
    const mb = MicroBuild.init(b, mz_dep) orelse return;
    const targets = max78000_zig.init(max_dep);

    const fw = mb.add_firmware(.{
        .name = "app",
        // chip-level target:
        // .target = targets.chips.max78000,
        // or a board-level target:
        .target = targets.boards.max78000evkit,
        .optimize = b.standardOptimizeOption(.{}),
        .root_source_file = b.path("src/main.zig"),
    });

    mb.install_firmware(fw, .{});
}
```

---

## Package API

The package exposes a single entry point:

```zig
pub fn init(dep: *std.Build.Dependency) Self
```

`Self` provides two namespaces:

```
targets.chips.max78000          — bare chip, no board pin mapping
targets.boards.max78000evkit    — chip + EVKIT board definition
targets.boards.max78000fthr     — chip + Feather board definition
```

Each field is a `*const microzig.Target` ready to pass to `mb.add_firmware(...)`.

---

## HAL modules

Early-stage HAL. Available modules are re-exported from `src/hal.zig`:

| Module | File | Status |
|---|---|---|
| `hal.gpio` | `src/hal/gpio.zig` | 🟡 Scaffolded |
| `hal.gcr` | `src/hal/gcr.zig` | 🟡 Scaffolded |
| `hal.uart` | `src/hal/uart.zig` | 🟡 Scaffolded |
| `hal.pins` | `src/hal/pins.zig` | 🟡 Scaffolded |

See [HAL development notes](#hal-development-notes) for the bring-up roadmap.

---

## SVD preprocessing

The raw vendor SVD (`svd/max78000.svd`) is not fed directly to `regz`. Instead, the build runs `tools/expand_max78000_svd.zig` at compile time to produce an expanded SVD in the build cache.

**Why this is necessary:** `regz` handles peripheral-level `derivedFrom` but does not fully resolve register-level and field-level `derivedFrom`. Without expansion, generated register APIs are incomplete — for example, missing derived fields or falling back to plain `u32` registers.

**The expander validates:**

- No remaining register/field `derivedFrom` references after expansion
- Expected MAX78000 sentinel registers/fields are still present
- Peripheral-level `derivedFrom` count is preserved

If any invariant is violated, the build fails loudly. The expanded SVD artifact lives in the build cache and is intentionally not checked in.

---

## Repository layout

```
max78000-zig/
├── build.zig             # Package entry point and standalone build
├── build.zig.zon         # Package manifest (v0.1.0, Zig ≥ 0.15.2)
├── svd/
│   └── max78000.svd      # Raw vendor SVD (unmodified)
├── tools/
│   └── expand_max78000_svd.zig   # Build-time SVD expander
├── src/
│   ├── hal.zig           # HAL root — re-exports all modules
│   ├── hal/
│   │   ├── gpio.zig
│   │   ├── gcr.zig
│   │   ├── uart.zig
│   │   └── pins.zig
│   └── boards/
│       ├── max78000evkit.zig
│       └── max78000fthr.zig
├── examples/
│   └── empty.zig         # Minimal firmware; also validates regz expansion
├── docs/
│   └── TEMPLATE.md       # Guide for reusing this layout for other MCUs
├── hal-bringup.md        # Practical HAL bring-up order and verification plan
└── hal-design.md         # Architecture options and recommended HAL shape
```

---

## Template usage

This repository is structured as a reusable template for out-of-tree MicroZig chip-support packages.

See [`docs/TEMPLATE.md`](docs/TEMPLATE.md) for:

- Required package API (`init(dep)` + `chips`/`boards` targets)
- Recommended file layout
- `build.zig` and `build.zig.zon` checklists
- Distribution checklist

---

## Memory map sources

- Bundled `svd/max78000.svd` — core and peripheral identification
- Zephyr ADI MAX32 DTS data — flash base address and SRAM bank layout
- [MAX78000 product page](https://www.analog.com/en/products/max78000.html)
