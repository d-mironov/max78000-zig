# max78000-zig

Self-contained MicroZig chip definition package for the Analog Devices / Maxim MAX78000.

## What it provides

- `chips.max78000`: chip-level `microzig.Target` for the Cortex-M4F core
- `boards.max78000evkit`: board-level target derived from `chips.max78000`
- `boards.max78000fthr`: board-level target derived from `chips.max78000`
- register generation from the bundled `svd/max78000.svd`
- default linker script generation for internal flash + SRAM
- a minimal standalone firmware example built by `zig build`

## Target summary

- CPU: ARM Cortex-M4F
- ABI: `eabihf`
- Flash: `0x10000000`, 512 KiB
- SRAM: `0x20000000`, 128 KiB

The SRAM is exposed as a single contiguous 128 KiB region for MicroZig, even though vendor DTS data describes it as four contiguous banks.

## SVD preprocessing

The bundled `svd/max78000.svd` is kept as the raw vendor SVD.

During the build, this package runs `tools/expand_max78000_svd.zig` to generate an expanded SVD in the build cache and feeds that generated file to MicroZig/regz.

The expander includes validation checks (no remaining register/field `derivedFrom`, expected MAX78000 sentinel registers/fields still present, peripheral-level `derivedFrom` count preserved) and fails the build if invariants are violated.

This is required because current `regz` support handles peripheral-level `derivedFrom`, but does not fully handle register-level and field-level `derivedFrom`. Without this preprocessing step, generated register APIs can be incomplete (for example, missing derived fields or falling back to plain `u32` registers).

The expanded SVD artifact is generated at build time and is intentionally not checked into the repository.

## Standalone build

```sh
zig build
```

This produces:

- `zig-out/firmware/empty.elf`

## Using it from another MicroZig project

Add both `microzig` and `max78000_zig` to `build.zig.zon`, then:

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
        // Either chip-level target:
        // .target = targets.chips.max78000,
        // or a board-level target:
        .target = targets.boards.max78000evkit,
        .optimize = b.standardOptimizeOption(.{}),
        .root_source_file = b.path("src/main.zig"),
    });

    mb.install_firmware(fw, .{});
}
```

## Template usage for custom chip packages

If you want to use this repository structure as a template for another MCU family,
see `docs/TEMPLATE.md`.

It documents the expected package API (`init(dep)` + `chips`/`boards` targets),
required file layout, and packaging/distribution checklist.

## HAL development notes

For MAX78000 HAL work, see:

- `hal-bringup.md` — practical bring-up order, scaffolding, and verification plan
- `hal-design.md` — architecture options, tradeoffs, and recommended HAL shape

## Sources used for the memory map

- bundled MAX78000 SVD for core/peripheral identification
- Zephyr ADI MAX32 DTS data for flash base and SRAM bank layout
