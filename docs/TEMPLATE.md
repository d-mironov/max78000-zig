# Custom MicroZig Chip Package Template

This repository is structured as a reusable template for out-of-tree MicroZig chip support packages.

## Required package API

Your package `build.zig` should expose:

- `pub fn init(dep: *std.Build.Dependency) Self`
- `Self.chips.<chip_name>: *const microzig.Target`
- optional `Self.boards.<board_name>: *const microzig.Target`

Consumers then do:

```zig
const chip_pkg = @import("your_chip_pkg");
const targets = chip_pkg.init(b.dependency("your_chip_pkg", .{}));
```

## Recommended file layout

```text
build.zig
build.zig.zon
README.md
svd/
  <chip>.svd
tools/
  <optional preprocessors>.zig
src/
  boards/
    <optional board defs>.zig
examples/
  empty.zig
```

## build.zig checklist

1. Import `microzig` and create `const MicroBuild = microzig.MicroBuild(.{});`.
2. Define a `microzig.Target` with:
   - `.dep = dep`
   - `.zig_target`
   - `.chip.name`
   - `.chip.register_definition`
   - `.chip.memory_regions`
3. Return `target.derive(.{})` entries via `Self.chips` (and optionally `Self.boards`).
4. Keep a local `pub fn build(b: *std.Build)` that builds at least one tiny firmware (`examples/empty.zig`).

## build.zig.zon checklist

- include `microzig` in `.dependencies`
- include every path used by `init(dep)` in `.paths`
  - especially `src/` and `tools/` if referenced by `dep.path(...)`

## Distribution checklist

1. Tag and publish this package.
2. In downstream project:
   - add `microzig`
   - add your chip package
   - import your package in `build.zig`
   - call `init(dep)` and use `targets.chips.<chip>` or `targets.boards.<board>`

## Notes for this MAX78000 package

- Raw SVD is at `svd/max78000.svd`.
- Build-time expander is at `tools/expand_max78000_svd.zig`.
- Consumers automatically get expansion because `init(dep)` wires the tool into the build graph.
