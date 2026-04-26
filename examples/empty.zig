const microzig = @import("microzig");

comptime {
    const chip = microzig.chip;

    const DmaInten = @FieldType(chip.types.peripherals.DMA, "INTEN").underlying_type;
    if (!@hasField(DmaInten, "CH1") or !@hasField(DmaInten, "CH2") or !@hasField(DmaInten, "CH3")) {
        @compileError("regz regression: expected DMA.INTEN.CH1/CH2/CH3 from derivedFrom expansion");
    }

    const Lpwkst1 = @FieldType(chip.types.peripherals.PWRSEQ, "LPWKST1").underlying_type;
    if (!@hasField(Lpwkst1, "WAKEST")) {
        @compileError("regz regression: expected PWRSEQ.LPWKST1.WAKEST from derived register expansion");
    }

    const BuckOutReady = @FieldType(chip.types.peripherals.SIMO, "BUCK_OUT_READY").underlying_type;
    if (!@hasField(BuckOutReady, "BUCKOUTRDYB")) {
        @compileError("regz regression: expected SIMO.BUCK_OUT_READY.BUCKOUTRDYB from derived field expansion");
    }
}

pub fn main() !void {
    while (true) {
        asm volatile ("wfi");
    }
}
