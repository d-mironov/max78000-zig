const std = @import("std");
const microzig = @import("microzig");

const MicroBuild = microzig.MicroBuild(.{});
const Target = microzig.Target;

const Self = @This();

chips: struct {
    max78000: *const Target,
},

boards: struct {
    max78000evkit: *const Target,
    max78000fthr: *const Target,
},

fn max78000Target(dep: *std.Build.Dependency, svd: std.Build.LazyPath, hal_root: std.Build.LazyPath) Target {
    return .{
        .dep = dep,
        .preferred_binary_format = .elf,
        .zig_target = .{
            .cpu_arch = .thumb,
            .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m4 },
            // NOTE: check what it is.
            .cpu_features_add = std.Target.arm.featureSet(&.{.vfp4d16sp}),
            .os_tag = .freestanding,
            .abi = .eabihf,
        },
        .chip = .{
            .name = "max78000",
            .url = "https://www.analog.com/en/products/max78000.html",
            .register_definition = .{ .svd = svd },
            .memory_regions = &.{
                // MAX78000 SRAM is exposed as four contiguous banks in the vendor DTS:
                // 32 KiB + 32 KiB + 48 KiB + 16 KiB = 128 KiB total.
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

fn expandedSvdFromDep(dep: *std.Build.Dependency) std.Build.LazyPath {
    const b = dep.builder;

    const expander = b.addExecutable(.{
        .name = "expand-max78000-svd",
        .root_module = b.createModule(.{
            .root_source_file = dep.path("tools/expand_max78000_svd.zig"),
            .target = b.graph.host,
            .optimize = .ReleaseSafe,
        }),
    });

    const run = b.addRunArtifact(expander);
    run.addFileArg(dep.path("svd/max78000.svd"));
    run.addArg("--output-path");
    return run.addOutputFileArg("max78000.expanded.svd");
}

fn expandedSvdFromBuild(b: *std.Build) std.Build.LazyPath {
    const expander = b.addExecutable(.{
        .name = "expand-max78000-svd",
        .root_module = b.createModule(.{
            .root_source_file = b.path("tools/expand_max78000_svd.zig"),
            .target = b.graph.host,
            .optimize = .ReleaseSafe,
        }),
    });

    const run = b.addRunArtifact(expander);
    run.addFileArg(b.path("svd/max78000.svd"));
    run.addArg("--output-path");
    return run.addOutputFileArg("max78000.expanded.svd");
}

pub fn init(dep: *std.Build.Dependency) Self {
    const b = dep.builder;
    const target = max78000Target(
        dep,
        expandedSvdFromDep(dep),
        b.path("src/hal.zig"),
    );

    return .{
        .chips = .{
            .max78000 = target.derive(.{}),
        },
        .boards = .{
            .max78000evkit = target.derive(.{ .board = .{
                .name = "Analog Devices MAX78000EVKIT",
                .url = "https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max78000evkit.html",
                .root_source_file = b.path("src/boards/max78000evkit.zig"),
            } }),
            .max78000fthr = target.derive(.{ .board = .{
                .name = "Analog Devices MAX78000FTHR",
                .url = "https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max78000fthr.html",
                .root_source_file = b.path("src/boards/max78000fthr.zig"),
            } }),
        },
    };
}

pub fn build(b: *std.Build) void {
    const mz_dep = b.dependency("microzig", .{});
    const mb = MicroBuild.init(b, mz_dep) orelse return;
    const optimize = b.standardOptimizeOption(.{});

    const target = max78000Target(mz_dep, expandedSvdFromBuild(b));
    const empty = mb.add_firmware(.{
        .name = "empty",
        .target = target.derive(.{}),
        .optimize = optimize,
        .root_source_file = b.path("examples/empty.zig"),
    });

    // const hello_world = mb.add_firmware(.{
    //     .name = "hello_world",
    //     .target = target.derive(.{}),
    //     .optimize = optimize,
    //     .root_source_file = b.path("examples/hello_world.zig"),
    // });

    mb.install_firmware(empty, .{});
}
