const micro = @import("microzig");
const mmio = micro.mmio;

pub const devices = struct {
    ///  MAX78000 Machine Learning System-on-Chip.
    pub const max78000 = struct {
        pub const properties = struct {
            pub const @"cpu.endian" = "little";
            pub const @"cpu.mpuPresent" = "true";
            pub const @"cpu.revision" = "r2p1";
            pub const @"cpu.name" = "CM4";
            pub const @"cpu.nvicPrioBits" = "3";
            pub const @"cpu.vendorSystickConfig" = "false";
            pub const @"cpu.fpuPresent" = "true";
        };

        pub const VectorTable = extern struct {
            const Handler = micro.interrupt.Handler;
            const unhandled = micro.interrupt.unhandled;

            initial_stack_pointer: u32,
            Reset: Handler,
            NMI: Handler = unhandled,
            HardFault: Handler = unhandled,
            MemManageFault: Handler = unhandled,
            BusFault: Handler = unhandled,
            UsageFault: Handler = unhandled,
            reserved5: [4]u32 = undefined,
            SVCall: Handler = unhandled,
            reserved10: [2]u32 = undefined,
            PendSV: Handler = unhandled,
            SysTick: Handler = unhandled,
            reserved14: [1]u32 = undefined,
            WWDT: Handler = unhandled,
            reserved16: [1]u32 = undefined,
            ///  RTC interrupt.
            RTC: Handler = unhandled,
            ///  TRNG interrupt.
            TRNG: Handler = unhandled,
            TMR: Handler = unhandled,
            ///  TMR1 IRQ
            TMR1: Handler = unhandled,
            ///  TMR2 IRQ
            TMR2: Handler = unhandled,
            ///  TMR3 IRQ
            TMR3: Handler = unhandled,
            ///  TMR4 IRQ
            TMR4: Handler = unhandled,
            ///  TMR5 IRQ
            TMR5: Handler = unhandled,
            reserved25: [2]u32 = undefined,
            ///  I2C0 IRQ
            I2C0: Handler = unhandled,
            reserved28: [6]u32 = undefined,
            ///  ADC IRQ
            ADC: Handler = unhandled,
            reserved35: [2]u32 = undefined,
            ///  Flash Controller interrupt.
            Flash_Controller: Handler = unhandled,
            ///  GPIO0 interrupt.
            GPIO0: Handler = unhandled,
            ///  GPIO1 IRQ
            GPIO1: Handler = unhandled,
            ///  GPIO2 IRQ
            GPIO2: Handler = unhandled,
            reserved41: [1]u32 = undefined,
            DMA0: Handler = unhandled,
            reserved43: [3]u32 = undefined,
            ///  SPI1 IRQ
            SPI1: Handler = unhandled,
            reserved47: [3]u32 = undefined,
            ///  I2C1 IRQ
            I2C1: Handler = unhandled,
            reserved51: [8]u32 = undefined,
            DMA1: Handler = unhandled,
            DMA2: Handler = unhandled,
            DMA3: Handler = unhandled,
            reserved62: [5]u32 = undefined,
            ///  WUT IRQ
            Wakeup_Timer: Handler = unhandled,
            reserved68: [2]u32 = undefined,
            SPI0: Handler = unhandled,
            ///  WDT1 IRQ
            WDT1: Handler = unhandled,
            reserved72: [1]u32 = undefined,
            ///  Pulse Train IRQ
            PT: Handler = unhandled,
            reserved74: [2]u32 = undefined,
            ///  I2C2 IRQ
            I2C2: Handler = unhandled,
            reserved77: [4]u32 = undefined,
            OneWire: Handler = unhandled,
            reserved82: [15]u32 = undefined,
            ///  Dynamic Voltage Scaling Interrupt
            DVS: Handler = unhandled,
            reserved98: [7]u32 = undefined,
            CameraIF: Handler = unhandled,
            reserved106: [7]u32 = undefined,
            ///  I2S IRQ
            I2S: Handler = unhandled,
            reserved114: [3]u32 = undefined,
            ///  Low Power Comparato
            LPCMP: Handler = unhandled,
        };

        pub const peripherals = struct {
            ///  Global Control Registers.
            pub const GCR: *volatile types.peripherals.GCR = @ptrFromInt(0x40000000);
            ///  System Initialization Registers.
            pub const SIR: *volatile types.peripherals.SIR = @ptrFromInt(0x40000400);
            ///  Function Control Register.
            pub const FCR: *volatile types.peripherals.FCR = @ptrFromInt(0x40000800);
            ///  Windowed Watchdog Timer
            pub const WDT: *volatile types.peripherals.WDT = @ptrFromInt(0x40003000);
            ///  Dynamic Voltage Scaling
            pub const DVS: *volatile types.peripherals.DVS = @ptrFromInt(0x40003c00);
            ///  Single Inductor Multiple Output Switching Converter
            pub const SIMO: *volatile types.peripherals.SIMO = @ptrFromInt(0x40004400);
            ///  Trim System Initilazation Registers
            pub const TRIMSIR: *volatile types.peripherals.TRIMSIR = @ptrFromInt(0x40005400);
            ///  Global Control Function Register.
            pub const GCFR: *volatile types.peripherals.GCFR = @ptrFromInt(0x40005800);
            ///  Real Time Clock and Alarm.
            pub const RTC: *volatile types.peripherals.RTC = @ptrFromInt(0x40006000);
            ///  32-bit reloadable timer that can be used for timing and wakeup.
            pub const WUT: *volatile types.peripherals.WUT = @ptrFromInt(0x40006400);
            ///  Power Sequencer / Low Power Control Register.
            pub const PWRSEQ: *volatile types.peripherals.PWRSEQ = @ptrFromInt(0x40006800);
            ///  Misc Control.
            pub const MCR: *volatile types.peripherals.MCR = @ptrFromInt(0x40006c00);
            ///  AES Keys.
            pub const AES: *volatile types.peripherals.AES = @ptrFromInt(0x40007400);
            ///  AES Key Registers.
            pub const AESKEYS: *volatile types.peripherals.AESKEYS = @ptrFromInt(0x40007800);
            ///  Individual I/O for each GPIO
            pub const GPIO0: *volatile types.peripherals.GPIO0 = @ptrFromInt(0x40008000);
            ///  Individual I/O for each GPIO 1
            pub const GPIO1: *volatile types.peripherals.GPIO0 = @ptrFromInt(0x40009000);
            ///  Parallel Camera Interface.
            pub const CAMERAIF: *volatile types.peripherals.CAMERAIF = @ptrFromInt(0x4000e000);
            ///  CRC Registers.
            pub const CRC: *volatile types.peripherals.CRC = @ptrFromInt(0x4000f000);
            ///  Low-Power Configurable Timer
            pub const TMR: *volatile types.peripherals.TMR = @ptrFromInt(0x40010000);
            ///  Low-Power Configurable Timer 1
            pub const TMR1: *volatile types.peripherals.TMR = @ptrFromInt(0x40011000);
            ///  Low-Power Configurable Timer 2
            pub const TMR2: *volatile types.peripherals.TMR = @ptrFromInt(0x40012000);
            ///  Low-Power Configurable Timer 3
            pub const TMR3: *volatile types.peripherals.TMR = @ptrFromInt(0x40013000);
            ///  Inter-Integrated Circuit.
            pub const I2C0: *volatile types.peripherals.I2C0 = @ptrFromInt(0x4001d000);
            ///  Inter-Integrated Circuit. 1
            pub const I2C1: *volatile types.peripherals.I2C0 = @ptrFromInt(0x4001e000);
            ///  Inter-Integrated Circuit. 2
            pub const I2C2: *volatile types.peripherals.I2C0 = @ptrFromInt(0x4001f000);
            ///  DMA Controller Fully programmable, chaining capable DMA channels.
            pub const DMA: *volatile types.peripherals.DMA = @ptrFromInt(0x40028000);
            ///  Flash Memory Control.
            pub const FLC: *volatile types.peripherals.FLC = @ptrFromInt(0x40029000);
            ///  Instruction Cache Controller Registers
            pub const ICC0: *volatile types.peripherals.ICC0 = @ptrFromInt(0x4002a000);
            ///  10-bit Analog to Digital Converter
            pub const ADC: *volatile types.peripherals.ADC = @ptrFromInt(0x40034000);
            ///  Pulse Train Generation
            pub const PTG: *volatile types.peripherals.PTG = @ptrFromInt(0x4003c000);
            ///  Pulse Train
            pub const PT: *volatile types.peripherals.PT = @ptrFromInt(0x4003c020);
            ///  Pulse Train 1
            pub const PT1: *volatile types.peripherals.PT = @ptrFromInt(0x4003c030);
            ///  Pulse Train 2
            pub const PT2: *volatile types.peripherals.PT = @ptrFromInt(0x4003c040);
            ///  Pulse Train 3
            pub const PT3: *volatile types.peripherals.PT = @ptrFromInt(0x4003c050);
            ///  1-Wire Master Interface.
            pub const OWM: *volatile types.peripherals.OWM = @ptrFromInt(0x4003d000);
            ///  The Semaphore peripheral allows multiple cores in a system to cooperate when accessing shred resources. The peripheral contains eight semaphores that can be atomically set and cleared. It is left to the discretion of the software architect to decide how and when the semaphores are used and how they are allocated. Existing hardware does not have to be modified for this type of cooperative sharing, and the use of semaphores is exclusively within the software domain.
            pub const SEMA: *volatile types.peripherals.SEMA = @ptrFromInt(0x4003e000);
            ///  UART Low Power Registers
            pub const UART: *volatile types.peripherals.UART = @ptrFromInt(0x40042000);
            ///  UART Low Power Registers 1
            pub const UART1: *volatile types.peripherals.UART = @ptrFromInt(0x40043000);
            ///  UART Low Power Registers 2
            pub const UART2: *volatile types.peripherals.UART = @ptrFromInt(0x40044000);
            ///  SPI peripheral. 1
            pub const SPI1: *volatile types.peripherals.SPI0 = @ptrFromInt(0x40046000);
            ///  Random Number Generator.
            pub const TRNG: *volatile types.peripherals.TRNG = @ptrFromInt(0x4004d000);
            ///  Inter-IC Sound Interface.
            pub const I2S: *volatile types.peripherals.I2S = @ptrFromInt(0x40060000);
            ///  Low Power Global Control.
            pub const LPGCR: *volatile types.peripherals.LPGCR = @ptrFromInt(0x40080000);
            ///  Individual I/O for each GPIO 2
            pub const GPIO2: *volatile types.peripherals.GPIO0 = @ptrFromInt(0x40080400);
            ///  Windowed Watchdog Timer 1
            pub const WDT1: *volatile types.peripherals.WDT = @ptrFromInt(0x40080800);
            ///  Low-Power Configurable Timer 4
            pub const TMR4: *volatile types.peripherals.TMR = @ptrFromInt(0x40080c00);
            ///  Low-Power Configurable Timer 5
            pub const TMR5: *volatile types.peripherals.TMR = @ptrFromInt(0x40081000);
            ///  UART Low Power Registers 3
            pub const UART3: *volatile types.peripherals.UART = @ptrFromInt(0x40081400);
            ///  Low Power Comparator
            pub const LPCMP: *volatile types.peripherals.LPCMP = @ptrFromInt(0x40088000);
            ///  SPI peripheral.
            pub const SPI0: *volatile types.peripherals.SPI0 = @ptrFromInt(0x400be000);
        };
    };
};

pub const types = struct {
    pub const peripherals = struct {
        ///  10-bit Analog to Digital Converter
        pub const ADC = extern struct {
            ///  ADC Control
            CTRL: mmio.Mmio(packed struct(u32) {
                ///  Start ADC Conversion
                start: u1,
                ///  ADC Power Up
                pwr: u1,
                reserved3: u1,
                ///  ADC Reference Buffer Power Up
                refbuf_pwr: u1,
                ///  ADC Reference Select
                ref_sel: u1,
                reserved8: u3,
                ///  ADC Reference Scale
                ref_scale: u1,
                ///  ADC Scale
                scale: u1,
                reserved11: u1,
                ///  ADC Clock Enable
                clk_en: u1,
                ///  ADC Channel Select
                ch_sel: packed union {
                    raw: u5,
                    value: enum(u5) {
                        AIN0 = 0x0,
                        AIN1 = 0x1,
                        AIN2 = 0x2,
                        AIN3 = 0x3,
                        AIN4 = 0x4,
                        AIN5 = 0x5,
                        AIN6 = 0x6,
                        AIN7 = 0x7,
                        VcoreA = 0x8,
                        VcoreB = 0x9,
                        Vrxout = 0xa,
                        Vtxout = 0xb,
                        VddA = 0xc,
                        ///  VddB/4
                        VddB = 0xd,
                        ///  Vddio/4
                        Vddio = 0xe,
                        ///  Vddioh/4
                        Vddioh = 0xf,
                        ///  VregI/4
                        VregI = 0x10,
                        _,
                    },
                },
                ///  Scales the external inputs, all inputs are scaled the same
                adc_divsel: packed union {
                    raw: u2,
                    value: enum(u2) {
                        DIV1 = 0x0,
                        DIV2 = 0x1,
                        DIV3 = 0x2,
                        DIV4 = 0x3,
                    },
                },
                reserved20: u1,
                ///  ADC Data Alignment Select
                data_align: u1,
                padding: u11,
            }),
            ///  ADC Status
            STATUS: mmio.Mmio(packed struct(u32) {
                ///  ADC Conversion In Progress
                active: u1,
                reserved2: u1,
                ///  AFE Power Up Delay Active
                afe_pwr_up_active: u1,
                ///  ADC Overflow
                overflow: u1,
                padding: u28,
            }),
            ///  ADC Output Data
            DATA: mmio.Mmio(packed struct(u32) {
                ///  ADC Converted Sample Data Output
                adc_data: u16,
                padding: u16,
            }),
            ///  ADC Interrupt Control Register
            INTR: mmio.Mmio(packed struct(u32) {
                ///  ADC Done Interrupt Enable
                done_ie: u1,
                ///  ADC Reference Ready Interrupt Enable
                ref_ready_ie: u1,
                ///  ADC Hi Limit Monitor Interrupt Enable
                hi_limit_ie: u1,
                ///  ADC Lo Limit Monitor Interrupt Enable
                lo_limit_ie: u1,
                ///  ADC Overflow Interrupt Enable
                overflow_ie: u1,
                reserved16: u11,
                ///  ADC Done Interrupt Flag
                done_if: u1,
                ///  ADC Reference Ready Interrupt Flag
                ref_ready_if: u1,
                ///  ADC Hi Limit Monitor Interrupt Flag
                hi_limit_if: u1,
                ///  ADC Lo Limit Monitor Interrupt Flag
                lo_limit_if: u1,
                ///  ADC Overflow Interrupt Flag
                overflow_if: u1,
                reserved22: u1,
                ///  ADC Interrupt Pending Status
                pending: u1,
                padding: u9,
            }),
            ///  ADC Limit
            LIMIT: [4]mmio.Mmio(packed struct(u32) {
                ///  Low Limit Threshold
                ch_lo_limit: u10,
                reserved12: u2,
                ///  High Limit Threshold
                ch_hi_limit: u10,
                reserved24: u2,
                ///  ADC Channel Select
                ch_sel: u5,
                ///  Low Limit Monitoring Enable
                ch_lo_limit_en: u1,
                ///  High Limit Monitoring Enable
                ch_hi_limit_en: u1,
                padding: u1,
            }),
        };

        ///  AES Keys.
        pub const AES = extern struct {
            ///  AES Control Register
            CTRL: mmio.Mmio(packed struct(u32) {
                ///  AES Enable
                EN: u1,
                ///  DMA Request To Read Data Output FIFO
                DMA_RX_EN: u1,
                ///  DMA Request To Write Data Input FIFO
                DMA_TX_EN: u1,
                ///  Start AES Calculation
                START: u1,
                ///  Flush the data input FIFO
                INPUT_FLUSH: u1,
                ///  Flush the data output FIFO
                OUTPUT_FLUSH: u1,
                ///  Encryption Key Size
                KEY_SIZE: packed union {
                    raw: u2,
                    value: enum(u2) {
                        ///  128 Bits.
                        AES128 = 0x0,
                        ///  192 Bits.
                        AES192 = 0x1,
                        ///  256 Bits.
                        AES256 = 0x2,
                        _,
                    },
                },
                ///  Encryption Type Selection
                TYPE: u2,
                padding: u22,
            }),
            ///  AES Status Register
            STATUS: mmio.Mmio(packed struct(u32) {
                ///  AES Busy Status
                BUSY: u1,
                ///  Data input FIFO empty status
                INPUT_EM: u1,
                ///  Data input FIFO full status
                INPUT_FULL: u1,
                ///  Data output FIFO empty status
                OUTPUT_EM: u1,
                ///  Data output FIFO full status
                OUTPUT_FULL: u1,
                padding: u27,
            }),
            ///  AES Interrupt Flag Register
            INTFL: mmio.Mmio(packed struct(u32) {
                ///  AES Done Interrupt
                DONE: u1,
                ///  External AES Key Changed Interrupt
                KEY_CHANGE: u1,
                ///  External AES Key Zero Interrupt
                KEY_ZERO: u1,
                ///  Data Output FIFO Overrun Interrupt
                OV: u1,
                ///  KEY_ONE
                KEY_ONE: u1,
                padding: u27,
            }),
            ///  AES Interrupt Enable Register
            INTEN: mmio.Mmio(packed struct(u32) {
                ///  AES Done Interrupt Enable
                DONE: u1,
                ///  External AES Key Changed Interrupt Enable
                KEY_CHANGE: u1,
                ///  External AES Key Zero Interrupt Enable
                KEY_ZERO: u1,
                ///  Data Output FIFO Overrun Interrupt Enable
                OV: u1,
                ///  KEY_ONE
                KEY_ONE: u1,
                padding: u27,
            }),
            ///  AES Data Register
            FIFO: mmio.Mmio(packed struct(u32) {
                ///  AES FIFO
                DATA: u1,
                padding: u31,
            }),
        };

        ///  AES Key Registers.
        pub const AESKEYS = extern struct {
            ///  AES Key 0.
            KEY0: u32,
            ///  AES Key 1.
            KEY1: u32,
            ///  AES Key 2.
            KEY2: u32,
            ///  AES Key 3.
            KEY3: u32,
            ///  AES Key 4.
            KEY4: u32,
            ///  AES Key 5.
            KEY5: u32,
            ///  AES Key 6.
            KEY6: u32,
            ///  AES Key 7.
            KEY7: u32,
        };

        ///  Parallel Camera Interface.
        pub const CAMERAIF = extern struct {
            ///  Hardware Version.
            VER: mmio.Mmio(packed struct(u32) {
                ///  Minor Version Number.
                minor: u8,
                ///  Major Version Number.
                major: u8,
                padding: u16,
            }),
            ///  FIFO Depth.
            FIFO_SIZE: mmio.Mmio(packed struct(u32) {
                ///  FIFO size.
                fifo_size: u8,
                padding: u24,
            }),
            ///  Control Register.
            CTRL: mmio.Mmio(packed struct(u32) {
                ///  Read Mode.
                READ_MODE: packed union {
                    raw: u2,
                    value: enum(u2) {
                        ///  Camera Interface Disabled.
                        dis = 0x0,
                        ///  Single Image Capture.
                        single_img = 0x1,
                        ///  Continuous Image Capture.
                        continuous = 0x2,
                        _,
                    },
                },
                ///  Data Width.
                DATA_WIDTH: packed union {
                    raw: u2,
                    value: enum(u2) {
                        ///  8 bit.
                        @"8bit" = 0x0,
                        ///  10 bit.
                        @"10bit" = 0x1,
                        ///  12 bit.
                        @"12bit" = 0x2,
                        _,
                    },
                },
                ///  DS Timing Enable.
                DS_TIMING_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Timing from VSYNC and HSYNC.
                        dis = 0x0,
                        ///  Timing embedded in data using SAV and EAV codes.
                        en = 0x1,
                    },
                },
                ///  Data FIFO Threshold.
                FIFO_THRSH: u5,
                reserved16: u6,
                ///  DMA Enable.
                RX_DMA: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  DMA disabled.
                        dis = 0x0,
                        ///  DMA enabled.
                        en = 0x1,
                    },
                },
                ///  DMA Threshold.
                RX_DMA_THRSH: u4,
                reserved30: u9,
                ///  Three-channel mode enable.
                THREE_CH_EN: u1,
                ///  PCIF Control.
                PCIF_SYS: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  PCIF disabled.
                        dis = 0x0,
                        ///  PCIF enabled.
                        en = 0x1,
                    },
                },
            }),
            ///  Interupt Enable Register.
            INT_EN: mmio.Mmio(packed struct(u32) {
                ///  Image Done.
                IMG_DONE: u1,
                ///  FIFO Full.
                FIFO_FULL: u1,
                ///  FIFO Threshold Level Met.
                FIFO_THRESH: u1,
                ///  FIFO Not Empty.
                FIFO_NOT_EMPTY: u1,
                padding: u28,
            }),
            ///  Interupt Flag Register.
            INT_FL: mmio.Mmio(packed struct(u32) {
                ///  Image Done.
                IMG_DONE: u1,
                ///  FIFO Full.
                FIFO_FULL: u1,
                ///  FIFO Threshold Level Met.
                FIFO_THRESH: u1,
                ///  FIFO Not Empty.
                FIFO_NOT_EMPTY: u1,
                padding: u28,
            }),
            ///  DS Timing Code Register.
            DS_TIMING_CODES: mmio.Mmio(packed struct(u32) {
                ///  Start Active Video Code.
                SAV: u8,
                ///  End Active Video Code.
                EAV: u8,
                padding: u16,
            }),
            reserved48: [24]u8,
            ///  FIFO DATA Register.
            FIFO_DATA: mmio.Mmio(packed struct(u32) {
                ///  Data from FIFO to be read by DMA.
                DATA: u32,
            }),
        };

        ///  CRC Registers.
        pub const CRC = extern struct {
            ///  CRC Control
            CTRL: mmio.Mmio(packed struct(u32) {
                ///  CRC Enable
                EN: u1,
                ///  DMA Request Enable
                DMA_EN: u1,
                ///  MSB Select
                MSB: u1,
                ///  Byte Swap CRC Data Input
                BYTE_SWAP_IN: u1,
                ///  Byte Swap CRC Value Output
                BYTE_SWAP_OUT: u1,
                reserved16: u11,
                ///  CRC Busy
                BUSY: u1,
                padding: u15,
            }),
            ///  CRC Data Input
            DATAIN32: mmio.Mmio(packed struct(u32) {
                ///  CRC Data
                DATA: u32,
            }),
            ///  CRC Polynomial
            POLY: mmio.Mmio(packed struct(u32) {
                ///  CRC Polynomial
                POLY: u32,
            }),
            ///  Current CRC Value
            VAL: mmio.Mmio(packed struct(u32) {
                ///  Current CRC Value
                VALUE: u32,
            }),
        };

        ///  DMA Controller Fully programmable, chaining capable DMA channels.
        pub const DMA = extern struct {
            ///  DMA Control Register.
            INTEN: mmio.Mmio(packed struct(u32) {
                ///  Channel 0 Interrupt Enable.
                CH0: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable.
                        dis = 0x0,
                        ///  Enable.
                        en = 0x1,
                    },
                },
                ///  Channel 1 Interrupt Enable.
                CH1: u1,
                ///  Channel 2 Interrupt Enable.
                CH2: u1,
                ///  Channel 3 Interrupt Enable.
                CH3: u1,
                padding: u28,
            }),
            ///  DMA Interrupt Register.
            INTFL: mmio.Mmio(packed struct(u32) {
                ///  Channel Interrupt. To clear an interrupt, all active interrupt bits of the DMA_ST must be cleared. The interrupt bits are set only if their corresponding interrupt enable bits are set in DMA_CN.
                CH0: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No interrupt is pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                CH1: u1,
                CH2: u1,
                CH3: u1,
                padding: u28,
            }),
        };

        ///  Dynamic Voltage Scaling
        pub const DVS = extern struct {
            ///  Control Register
            CTL: mmio.Mmio(packed struct(u32) {
                ///  Enable the DVS monitoring circuit
                MON_ENA: u1,
                ///  Enable the power supply adjustment based on measurements
                ADJ_ENA: u1,
                ///  Power Supply Feedback Disable
                PS_FB_DIS: u1,
                ///  Use the TAP Select for automatic adjustment or monitoring
                CTRL_TAP_ENA: u1,
                ///  Additional delay to monitor lines
                PROP_DLY: u2,
                ///  Measure delay once
                MON_ONESHOT: u1,
                ///  Operate in automatic mode or move directly
                GO_DIRECT: u1,
                ///  Step incrementally to target voltage
                DIRECT_REG: u1,
                ///  Include a delay line priming signal before monitoring
                PRIME_ENA: u1,
                ///  Enable Limit Error Interrupt
                LIMIT_IE: u1,
                ///  Enable Range Error Interrupt
                RANGE_IE: u1,
                ///  Enable Adjustment Error Interrupt
                ADJ_IE: u1,
                ///  Select TAP used for voltage adjustment
                REF_SEL: u4,
                ///  Step size to increment voltage when in automatic mode
                INC_VAL: u3,
                ///  Prevent the application code from adjusting Vcore
                DVS_PS_APB_DIS: u1,
                ///  Any high range signal from a delay line will cause a voltage adjustment
                DVS_HI_RANGE_ANY: u1,
                ///  Enable Voltage Adjustment Timeout Interrupt
                FB_TO_IE: u1,
                ///  Enable Low Voltage Interrupt
                FC_LV_IE: u1,
                ///  Prevent DVS from ack'ing a request to enter a low power mode until in the idle state
                PD_ACK_ENA: u1,
                ///  Causes the DVS to enter the idle state immediately on a request to enter a low power mode
                ADJ_ABORT: u1,
                padding: u6,
            }),
            ///  Status Fields
            STAT: mmio.Mmio(packed struct(u32) {
                ///  State machine state
                DVS_STATE: u4,
                ///  DVS Raising voltage
                ADJ_UP_ENA: u1,
                ///  DVS Lowering voltage
                ADJ_DWN_ENA: u1,
                ///  Adjustment to a Direct Voltage
                ADJ_ACTIVE: u1,
                ///  Tap Enabled and the Tap is withing Hi/Low limits
                CTR_TAP_OK: u1,
                ///  Status of selected center tap delay line detect output
                CTR_TAP_SEL: u1,
                ///  Provides the current combined status of all selected Low Range delay lines
                SLOW_TRIP_DET: u1,
                ///  Provides the current combined status of all selected High Range delay lines
                FAST_TRIP_DET: u1,
                ///  Indicates if the power supply is in range
                PS_IN_RANGE: u1,
                ///  Voltage Count value sent to the power supply
                PS_VCNTR: u7,
                ///  Indicates the monitor delay count is at 0
                MON_DLY_OK: u1,
                ///  Indicates the adjustment delay count is at 0
                ADJ_DLY_OK: u1,
                ///  Power supply voltage counter is at low limit
                LO_LIMIT_DET: u1,
                ///  Power supply voltage counter is at high limit
                HI_LIMIT_DET: u1,
                ///  At least one delay line has been enabled
                VALID_TAP: u1,
                ///  Interrupt flag that indicates a voltage count is at/beyond manufacturer limits
                LIMIT_ERR: u1,
                ///  Interrupt flag that indicates a tap has an invalid value
                RANGE_ERR: u1,
                ///  Interrupt flag that indicates up and down adjustment requested simultaneously
                ADJ_ERR: u1,
                ///  Indicates the ref select register bit is out of range
                REF_SEL_ERR: u1,
                ///  Interrupt flag that indicates a timeout while adjusting the voltage
                FB_TO_ERR: u1,
                ///  Interrupt flag that mirror FB_TO_ERR and is write one clear
                FB_TO_ERR_S: u1,
                ///  Interrupt flag that indicates the power supply voltage requested is below the low threshold
                FC_LV_DET_INT: u1,
                ///  Interrupt flag that mirrors FC_LV_DET_INT
                FC_LV_DET_S: u1,
            }),
            ///  Direct control of target voltage
            DIRECT: mmio.Mmio(packed struct(u32) {
                ///  Sets the target power supply value
                VOLTAGE: u7,
                padding: u25,
            }),
            ///  Monitor Delay
            MON: mmio.Mmio(packed struct(u32) {
                ///  Number of prescaled clocks between delay line samples
                DLY: u24,
                ///  Number of clocks before DVS_MON_DLY is decremented
                PRE: u8,
            }),
            ///  Up Delay Register
            ADJ_UP: mmio.Mmio(packed struct(u32) {
                ///  Number of prescaled clocks between updates of the adjustment delay counter
                DLY: u16,
                ///  Number of clocks before DVS_ADJ_UP_DLY is decremented
                PRE: u8,
                padding: u8,
            }),
            ///  Down Delay Register
            ADJ_DWN: mmio.Mmio(packed struct(u32) {
                ///  Number of prescaled clocks between updates of the adjustment delay counter
                DLY: u16,
                ///  Number of clocks before DVS_ADJ_DWN_DLY is decremented
                PRE: u8,
                padding: u8,
            }),
            ///  Up Delay Register
            THRES_CMP: mmio.Mmio(packed struct(u32) {
                ///  Value used to determine 'low voltage' range
                VCNTR_THRES_CNT: u7,
                reserved8: u1,
                ///  Mask applied to threshold and vcount to determine if the device is in a low voltage range
                VCNTR_THRES_MASK: u7,
                padding: u17,
            }),
            ///  DVS Tap Select Register
            TAP_SEL: [5]mmio.Mmio(packed struct(u32) {
                ///  Select delay line tap for lower bound of auto adjustment
                LO: u5,
                ///  Returns last delay line tap value
                LO_TAP_STAT: u1,
                ///  Returns last delay line tap value
                CTR_TAP_STAT: u1,
                ///  Returns last delay line tap value
                HI_TAP_STAT: u1,
                ///  Selects delay line tap for high point of auto adjustment
                HI: u5,
                reserved16: u3,
                ///  Selects delay line tap for center point of auto adjustment
                CTR: u5,
                reserved24: u3,
                ///  Selects delay line tap for coarse or fixed delay portion of the line
                COARSE: u3,
                reserved29: u2,
                ///  Number of HCLK between delay line launch and sampling
                DET_DLY: u2,
                ///  Set if the delay is active
                DELAY_ACT: u1,
            }),
        };

        ///  Function Control Register.
        pub const FCR = extern struct {
            ///  Function Control 0.
            FCTRL0: mmio.Mmio(packed struct(u32) {
                reserved20: u20,
                ///  I2C0 SDA Pad Deglitcher enable.
                I2C0DGEN0: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Deglitcher disabled.
                        dis = 0x0,
                        ///  Deglitcher enabled.
                        en = 0x1,
                    },
                },
                ///  I2C0 SCL Pad Deglitcher enable.
                I2C0DGEN1: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Deglitcher disabled.
                        dis = 0x0,
                        ///  Deglitcher enabled.
                        en = 0x1,
                    },
                },
                ///  I2C1 SDA Pad Deglitcher enable.
                I2C1DGEN0: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Deglitcher disabled.
                        dis = 0x0,
                        ///  Deglitcher enabled.
                        en = 0x1,
                    },
                },
                ///  I2C1 SCL Pad Deglitcher enable.
                I2C1DGEN1: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Deglitcher disabled.
                        dis = 0x0,
                        ///  Deglitcher enabled.
                        en = 0x1,
                    },
                },
                ///  I2C2 SDA Pad Deglitcher enable.
                I2C2DGEN0: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Deglitcher disabled.
                        dis = 0x0,
                        ///  Deglitcher enabled.
                        en = 0x1,
                    },
                },
                ///  I2C2 SCL Pad Deglitcher enable.
                I2C2DGEN1: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Deglitcher disabled.
                        dis = 0x0,
                        ///  Deglitcher enabled.
                        en = 0x1,
                    },
                },
                padding: u6,
            }),
            ///  Automatic Calibration 0.
            AUTOCAL0: mmio.Mmio(packed struct(u32) {
                ///  Auto-calibration Enable.
                ACEN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disabled.
                        dis = 0x0,
                        ///  Enabled.
                        en = 0x1,
                    },
                },
                ///  Autocalibration Run.
                ACRUN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Not Running.
                        not = 0x0,
                        ///  Running.
                        run = 0x1,
                    },
                },
                ///  Load Trim.
                LDTRM: u1,
                ///  Invert Gain.
                GAININV: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Not Running.
                        not = 0x0,
                        ///  Running.
                        run = 0x1,
                    },
                },
                ///  Atomic mode.
                ATOMIC: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Not Running.
                        not = 0x0,
                        ///  Running.
                        run = 0x1,
                    },
                },
                reserved8: u3,
                ///  MU value.
                MU: u12,
                reserved23: u3,
                ///  HIRC96M Trim Value.
                HIRC96MACTMROUT: u9,
            }),
            ///  Automatic Calibration 1.
            AUTOCAL1: mmio.Mmio(packed struct(u32) {
                ///  Initial Trim Setting.
                INITTRM: u9,
                padding: u23,
            }),
            ///  Automatic Calibration 2
            AUTOCAL2: mmio.Mmio(packed struct(u32) {
                ///  Auto-callibration Done Counter Setting.
                DONECNT: u8,
                ///  Auto-callibration Div Setting.
                ACDIV: u13,
                padding: u11,
            }),
            ///  RISC-V Boot Address.
            URVBOOTADDR: u32,
            ///  RISC-V Control Register.
            URVCTRL: mmio.Mmio(packed struct(u32) {
                ///  RAM2, RAM3 exclusive ownership.
                MEMSEL: u1,
                ///  URV instruction flush enable.
                IFLUSHEN: u1,
                padding: u30,
            }),
        };

        ///  Flash Memory Control.
        pub const FLC = extern struct {
            ///  Flash Write Address.
            ADDR: mmio.Mmio(packed struct(u32) {
                ///  Address for next operation.
                ADDR: u32,
            }),
            ///  Flash Clock Divide. The clock (PLL0) is divided by this value to generate a 1 MHz clock for Flash controller.
            CLKDIV: mmio.Mmio(packed struct(u32) {
                ///  Flash Clock Divide. The clock is divided by this value to generate a 1MHz clock for flash controller.
                CLKDIV: u8,
                padding: u24,
            }),
            ///  Flash Control Register.
            CTRL: mmio.Mmio(packed struct(u32) {
                ///  Write. This bit is automatically cleared after the operation.
                WR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No operation/complete.
                        complete = 0x0,
                        ///  Start operation.
                        start = 0x1,
                    },
                },
                ///  Mass Erase. This bit is automatically cleared after the operation.
                ME: u1,
                ///  Page Erase. This bit is automatically cleared after the operation.
                PGE: u1,
                reserved8: u5,
                ///  Erase Code. The ERASE_CODE must be set up property before erase operation can be initiated. These bits are automatically cleared after the operation is complete.
                ERASE_CODE: packed union {
                    raw: u8,
                    value: enum(u8) {
                        ///  No operation.
                        nop = 0x0,
                        ///  Enable Page Erase.
                        erasePage = 0x55,
                        ///  Enable Mass Erase. The debug port must be enabled.
                        eraseAll = 0xaa,
                        _,
                    },
                },
                reserved24: u8,
                ///  Flash Pending. When Flash operation is in progress (busy), Flash reads and writes will fail. When PEND is set, write to all Flash registers, with exception of the Flash interrupt register, are ignored.
                PEND: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Idle.
                        idle = 0x0,
                        ///  Busy.
                        busy = 0x1,
                    },
                },
                ///  Low Voltage enable.
                LVE: u1,
                reserved28: u2,
                ///  Flash Unlock. The correct unlock code must be written to these four bits before any Flash write or erase operation is allowed.
                UNLOCK: packed union {
                    raw: u4,
                    value: enum(u4) {
                        ///  Flash Unlocked.
                        unlocked = 0x2,
                        ///  Flash Locked.
                        locked = 0x3,
                        _,
                    },
                },
            }),
            reserved36: [24]u8,
            ///  Flash Interrupt Register.
            INTR: mmio.Mmio(packed struct(u32) {
                ///  Flash Done Interrupt. This bit is set to 1 upon Flash write or erase completion.
                DONE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No interrupt is pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  Flash Access Fail. This bit is set when an attempt is made to write the flash while the flash is busy or the flash is locked. This bit can only be set to 1 by hardware.
                AF: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Failure.
                        noError = 0x0,
                        ///  Failure occurs.
                        @"error" = 0x1,
                    },
                },
                reserved8: u6,
                ///  Flash Done Interrupt Enable.
                DONEIE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable.
                        disable = 0x0,
                        ///  Enable.
                        enable = 0x1,
                    },
                },
                AFIE: u1,
                padding: u22,
            }),
            ///  ECC Data Register.
            ECCDATA: mmio.Mmio(packed struct(u32) {
                ///  Error Correction Code Odd Data.
                EVEN: u9,
                reserved16: u7,
                ///  Error Correction Code Even Data.
                ODD: u9,
                padding: u7,
            }),
            reserved48: [4]u8,
            ///  Flash Write Data.
            DATA: [4]mmio.Mmio(packed struct(u32) {
                ///  Data next operation.
                DATA: u32,
            }),
            ///  Access Control Register. Writing the ACTRL register with the following values in the order shown, allows read and write access to the system and user Information block: pflc-actrl = 0x3a7f5ca3; pflc-actrl = 0xa1e34f20; pflc-actrl = 0x9608b2c1. When unlocked, a write of any word will disable access to system and user information block. Readback of this register is always zero.
            ACTRL: mmio.Mmio(packed struct(u32) {
                ///  Access control.
                ACTRL: u32,
            }),
            reserved128: [60]u8,
            ///  WELR0
            WELR0: mmio.Mmio(packed struct(u32) {
                ///  Access control.
                WELR0: u32,
            }),
            reserved136: [4]u8,
            ///  WELR1
            WELR1: mmio.Mmio(packed struct(u32) {
                ///  Access control.
                WELR1: u32,
            }),
            reserved144: [4]u8,
            ///  RLR0
            RLR0: mmio.Mmio(packed struct(u32) {
                ///  Access control.
                RLR0: u32,
            }),
            reserved152: [4]u8,
            ///  RLR1
            RLR1: mmio.Mmio(packed struct(u32) {
                ///  Access control.
                RLR1: u32,
            }),
        };

        ///  Global Control Registers.
        pub const GCR = extern struct {
            ///  System Control.
            SYSCTRL: mmio.Mmio(packed struct(u32) {
                reserved1: u1,
                ///  Boundary Scan TAP enable. When enabled, the JTAG port is conneted to the Boundary Scan TAP instead of the ARM ICE.
                BSTAPEN: u1,
                reserved4: u2,
                ///  Flips the Flash bottom and top halves. (Depending on the total flash size, each half is either 256K or 512K). Initiating a flash page flip will cause a flush of both the data buffer on the DCODE bus and the internal instruction buffer.
                FLASH_PAGE_FLIP: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Physical layout matches logical layout.
                        normal = 0x0,
                        ///  Bottom half mapped to logical top half and vice versa.
                        swapped = 0x1,
                    },
                },
                reserved6: u1,
                ///  Code Cache Flush. This bit is used to flush the code caches and the instruction buffer of the Cortex-M4.
                ICC0_FLUSH: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Normal Code Cache Operation
                        normal = 0x0,
                        ///  Code Caches and CPU instruction buffer are flushed
                        flush = 0x1,
                    },
                },
                reserved12: u5,
                ///  ROM_DONE status. Used to disable SWD interface during system initialization procedure
                ROMDONE: u1,
                ///  Compute ROM Checksum. This bit is self-cleared when calculation is completed. Once set, software clearing this bit is ignored and the bit will remain set until the operation is completed.
                CCHK: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No operation/complete.
                        complete = 0x0,
                        ///  Start operation.
                        start = 0x1,
                    },
                },
                ///  Serial Wire Debug Disable. This bit is used to disable the serial wire debug interface This bit is only writeable if (FMV lock word is not programmed) or if (ICE lock word is not programmed and the ROM_DONE bit is not set).
                SWD_DIS: u1,
                ///  ROM Checksum Result. This bit is only valid when CHKRD=1.
                CHKRES: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  ROM Checksum Correct.
                        pass = 0x0,
                        ///  ROM Checksum Fail.
                        fail = 0x1,
                    },
                },
                ///  Operating Voltage Range.
                OVR: u2,
                padding: u14,
            }),
            ///  Reset.
            RST0: mmio.Mmio(packed struct(u32) {
                ///  DMA Reset.
                DMA: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Reset complete.
                        reset_done = 0x0,
                        ///  Starts Reset or indicates reset in progress.
                        busy = 0x1,
                    },
                },
                ///  Watchdog Timer 0 Reset.
                WDT0: u1,
                ///  GPIO0 Reset. Setting this bit to 1 resets GPIO0 pins to their default states.
                GPIO0: u1,
                ///  GPIO1 Reset. Setting this bit to 1 resets GPIO1 pins to their default states.
                GPIO1: u1,
                reserved5: u1,
                ///  Timer 0 Reset. Setting this bit to 1 resets Timer 0 blocks.
                TMR0: u1,
                ///  Timer 1 Reset. Setting this bit to 1 resets Timer 1 blocks.
                TMR1: u1,
                ///  Timer 2 Reset. Setting this bit to 1 resets Timer 2 blocks.
                TMR2: u1,
                ///  Timer 3 Reset. Setting this bit to 1 resets Timer 3 blocks.
                TMR3: u1,
                reserved11: u2,
                ///  UART 0 Reset. Setting this bit to 1 resets all UART 0 blocks.
                UART0: u1,
                ///  UART 1 Reset. Setting this bit to 1 resets all UART 1 blocks.
                UART1: u1,
                ///  SPI 1 Reset. Setting this bit to 1 resets all SPI 1 blocks.
                SPI1: u1,
                reserved16: u2,
                ///  I2C 0 Reset.
                I2C0: u1,
                ///  Real Time Clock Reset.
                RTC: u1,
                reserved22: u4,
                ///  Semaphore Reset.
                SMPHR: u1,
                reserved24: u1,
                ///  TRNG Reset. This reset is only available during the manufacture testing phase.
                TRNG: u1,
                ///  CNN Reset.
                CNN: u1,
                ///  ADC Reset.
                ADC: u1,
                reserved28: u1,
                ///  UART2 Reset. Setting this bit to 1 resets all UART 2 blocks.
                UART2: u1,
                ///  Soft Reset. Setting this bit to 1 resets everything except the CPU and the watchdog timer.
                SOFT: u1,
                ///  Peripheral Reset. Setting this bit to 1 resets all peripherals. The CPU core, the watchdog timer, and all GPIO pins are unaffected by this reset.
                PERIPH: u1,
                ///  System Reset. Setting this bit to 1 resets the CPU core and all peripherals, including the watchdog timer.
                SYS: u1,
            }),
            ///  Clock Control.
            CLKCTRL: mmio.Mmio(packed struct(u32) {
                reserved6: u6,
                ///  Prescaler Select. This 3 bit field sets the system operating frequency by controlling the prescaler that divides the output of the PLL0.
                SYSCLK_DIV: packed union {
                    raw: u3,
                    value: enum(u3) {
                        ///  Divide by 1.
                        div1 = 0x0,
                        ///  Divide by 2.
                        div2 = 0x1,
                        ///  Divide by 4.
                        div4 = 0x2,
                        ///  Divide by 8.
                        div8 = 0x3,
                        ///  Divide by 16.
                        div16 = 0x4,
                        ///  Divide by 32.
                        div32 = 0x5,
                        ///  Divide by 64.
                        div64 = 0x6,
                        ///  Divide by 128.
                        div128 = 0x7,
                    },
                },
                ///  Clock Source Select. This 3 bit field selects the source for the system clock.
                SYSCLK_SEL: packed union {
                    raw: u3,
                    value: enum(u3) {
                        ///  The internal 60 MHz oscillator is used for the system clock.
                        ISO = 0x0,
                        ///  8 kHz LIRC is used for the system clock.
                        INRO = 0x3,
                        ///  The internal 100 MHz oscillator is used for the system clock.
                        IPO = 0x4,
                        ///  The internal 7.3725 MHz oscillator is used for the system clock.
                        IBRO = 0x5,
                        ///  32 kHz is used for the system clock.
                        ERTCO = 0x6,
                        ///  External clock on GPIO0.30.
                        EXTCLK = 0x7,
                        _,
                    },
                },
                reserved13: u1,
                ///  Clock Ready. This read only bit reflects whether the currently selected system clock source is running.
                SYSCLK_RDY: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Switchover to the new clock source (as selected by CLKSEL) has not yet occurred.
                        busy = 0x0,
                        ///  System clock running from CLKSEL clock source.
                        ready = 0x1,
                    },
                },
                reserved17: u3,
                ///  32 kHz Crystal Oscillator Enable.
                ERTCO_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Is Disabled.
                        dis = 0x0,
                        ///  Is Enabled.
                        en = 0x1,
                    },
                },
                ///  60 MHz High Frequency Internal Reference Clock Enable.
                ISO_EN: u1,
                ///  100 MHz High Frequency Internal Reference Clock Enable.
                IPO_EN: u1,
                ///  7.3725 MHz High Frequency Internal Reference Clock Enable.
                IBRO_EN: u1,
                ///  7.3725 MHz High Frequency Internal Reference Clock Voltage Select. This register bit is used to select the power supply to the IBRO.
                IBRO_VS: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  VCore Supply
                        Vcor = 0x0,
                        ///  Dedicated 1V regulated supply.
                        @"1V" = 0x1,
                    },
                },
                reserved25: u3,
                ///  32 kHz Crystal Oscillator Ready
                ERTCO_RDY: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Is not Ready.
                        not = 0x0,
                        ///  Is Ready.
                        ready = 0x1,
                    },
                },
                ///  60 MHz HIRC Ready.
                ISO_RDY: u1,
                ///  100 MHz HIRC Ready.
                IPO_RDY: u1,
                ///  7.3725 MHz HIRC Ready.
                IBRO_RDY: u1,
                ///  8 kHz Low Frequency Reference Clock Ready.
                INRO_RDY: u1,
                padding: u2,
            }),
            ///  Power Management.
            PM: mmio.Mmio(packed struct(u32) {
                ///  Operating Mode. This two bit field selects the current operating mode for the device. Note that code execution only occurs during ACTIVE mode.
                MODE: packed union {
                    raw: u4,
                    value: enum(u4) {
                        ///  Active Mode.
                        active = 0x0,
                        ///  Cortex-M4 Active, RISC-V Sleep Mode.
                        sleep = 0x1,
                        ///  Standby Mode.
                        standby = 0x2,
                        ///  Backup Mode.
                        backup = 0x4,
                        ///  LPM or CM4 Deep Sleep Mode.
                        lpm = 0x8,
                        ///  UPM.
                        upm = 0x9,
                        ///  Power Down Mode.
                        powerdown = 0xa,
                        _,
                    },
                },
                ///  GPIO Wake Up Enable. This bit enables all GPIO pins as potential wakeup sources. Any GPIO configured for wakeup is capable of causing an exit from IDLE or STANDBY modes when this bit is set.
                GPIO_WE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Wake Up Disable.
                        dis = 0x0,
                        ///  Wake Up Enable.
                        en = 0x1,
                    },
                },
                ///  RTC Alarm Wake Up Enable. This bit enables RTC alarm as wakeup source. If enabled, the desired RTC alarm must be configured via the RTC control registers.
                RTC_WE: u1,
                reserved7: u1,
                ///  WUT Wake Up Enable. This bit enables the Wake-Up Timer as wakeup source.
                WUT_WE: u1,
                reserved9: u1,
                ///  AIN COMP Wake Up Enable. This bit enables AIN COMP as wakeup source.
                AINCOMP_WE: u1,
                reserved15: u5,
                ///  60 MHz power down. This bit selects the 60 MHz clock power state in DEEPSLEEP mode.
                ISO_PD: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Mode is Active.
                        active = 0x0,
                        ///  Powered down in DEEPSLEEP.
                        deepsleep = 0x1,
                    },
                },
                ///  100 MHz power down. This bit selects 100 MHz clock power state in DEEPSLEEP mode.
                IPO_PD: u1,
                ///  7.3725 MHz power down. This bit selects 7.3725 MHz clock power state in DEEPSLEEP mode.
                IBRO_PD: u1,
                padding: u14,
            }),
            reserved24: [8]u8,
            ///  Peripheral Clock Divider.
            PCLKDIV: mmio.Mmio(packed struct(u32) {
                reserved10: u10,
                ///  ADC clock Frequency. These bits define the ADC clock frequency. fADC = fPCLK / (ADCFRQ)
                ADCFRQ: u4,
                ///  CNN Clock Divider.
                CNNCLKDIV: packed union {
                    raw: u3,
                    value: enum(u3) {
                        div2 = 0x0,
                        div4 = 0x1,
                        div8 = 0x2,
                        div16 = 0x3,
                        div1 = 0x4,
                        _,
                    },
                },
                ///  CNN Clock Select.
                CNNCLKSEL: packed union {
                    raw: u1,
                    value: enum(u1) {
                        PCLK = 0x0,
                        ISO = 0x1,
                    },
                },
                padding: u14,
            }),
            reserved36: [8]u8,
            ///  Peripheral Clock Disable.
            PCLKDIS0: mmio.Mmio(packed struct(u32) {
                ///  GPIO0 Clock Disable.
                GPIO0: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  enable it.
                        en = 0x0,
                        ///  disable it.
                        dis = 0x1,
                    },
                },
                ///  GPIO1 Clock Disable.
                GPIO1: u1,
                reserved5: u3,
                ///  DMA Clock Disable.
                DMA: u1,
                ///  SPI 1 Clock Disable.
                SPI1: u1,
                reserved9: u2,
                ///  UART 0 Clock Disable.
                UART0: u1,
                ///  UART 1 Clock Disable.
                UART1: u1,
                reserved13: u2,
                ///  I2C 0 Clock Disable.
                I2C0: u1,
                reserved15: u1,
                ///  Timer 0 Clock Disable.
                TMR0: u1,
                ///  Timer 1 Clock Disable.
                TMR1: u1,
                ///  Timer 2 Clock Disable.
                TMR2: u1,
                ///  Timer 3 Clock Disable.
                TMR3: u1,
                reserved23: u4,
                ///  ADC Clock Disable.
                ADC: u1,
                reserved25: u1,
                ///  CNN Clock Disable.
                CNN: u1,
                reserved28: u2,
                ///  I2C 1 Clock Disable.
                I2C1: u1,
                ///  Pluse Train Clock Disable.
                PT: u1,
                padding: u2,
            }),
            ///  Memory Clock Control Register.
            MEMCTRL: mmio.Mmio(packed struct(u32) {
                ///  Flash Wait State. These bits define the number of wait-state cycles per Flash data read access. Minimum wait state is 2.
                FWS: u3,
                reserved16: u13,
                ///  SYSRAM0 ECC Select.
                SYSRAM0ECC: u1,
                padding: u15,
            }),
            ///  Memory Zeroize Control.
            MEMZ: mmio.Mmio(packed struct(u32) {
                ///  System RAM Block 0 Zeroization.
                RAM0: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No operation/complete.
                        nop = 0x0,
                        ///  Start operation.
                        start = 0x1,
                    },
                },
                ///  System RAM Block 1 Zeroization.
                RAM1: u1,
                ///  System RAM Block 2 Zeroization.
                RAM2: u1,
                ///  System RAM Block 3 Zeroization.
                RAM3: u1,
                ///  System RAM 0 ECC Zeroization.
                SYSRAM0ECC: u1,
                ///  Instruction Cachei 0 Zeroization.
                ICC0: u1,
                ///  Instruction Cachei 1 Zeroization.
                ICC1: u1,
                padding: u25,
            }),
            reserved64: [16]u8,
            ///  System Status Register.
            SYSST: mmio.Mmio(packed struct(u32) {
                ///  ARM ICE Lock Status.
                ICELOCK: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  ICE is unlocked.
                        unlocked = 0x0,
                        ///  ICE is locked.
                        locked = 0x1,
                    },
                },
                padding: u31,
            }),
            ///  Reset 1.
            RST1: mmio.Mmio(packed struct(u32) {
                ///  I2C1 Reset.
                I2C1: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Reset complete.
                        reset_done = 0x0,
                        ///  Starts reset or indicates reset in progress.
                        busy = 0x1,
                    },
                },
                ///  PT Reset.
                PT: u1,
                reserved7: u5,
                ///  OWM Reset.
                OWM: u1,
                reserved9: u1,
                ///  CRC Reset.
                CRC: u1,
                ///  AES Reset.
                AES: u1,
                ///  SPI 0 Reset.
                SPI0: u1,
                reserved16: u4,
                ///  SMPHR Reset.
                SMPHR: u1,
                reserved19: u2,
                ///  I2S Reset.
                I2S: u1,
                ///  I2C2 Reset.
                I2C2: u1,
                reserved24: u3,
                ///  DVS Reset.
                DVS: u1,
                ///  SIMO Reset.
                SIMO: u1,
                reserved31: u5,
                ///  CPU1 Reset.
                CPU1: u1,
            }),
            ///  Peripheral Clock Disable.
            PCLKDIS1: mmio.Mmio(packed struct(u32) {
                reserved1: u1,
                ///  UART2 Clock Disable.
                UART2: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Enable.
                        en = 0x0,
                        ///  Disable.
                        dis = 0x1,
                    },
                },
                ///  TRNG Clock Disable.
                TRNG: u1,
                reserved9: u6,
                ///  SMPHR Clock Disable.
                SMPHR: u1,
                reserved13: u3,
                ///  One-Wire Clock Disable.
                OWM: u1,
                ///  CRC Clock Disable.
                CRC: u1,
                ///  AES Clock Disable.
                AES: u1,
                ///  SPI 0 Clock Disable.
                SPI0: u1,
                reserved18: u1,
                ///  Parallel Camera Interface Clock Disable.
                PCIF: u1,
                reserved23: u4,
                ///  I2S Clock Disable.
                I2S: u1,
                ///  I2C2 Clock Disable.
                I2C2: u1,
                reserved27: u2,
                ///  Watch Dog Timer 0 Clock Disable.
                WDT0: u1,
                reserved31: u3,
                ///  CPU1 Clock Disable.
                CPU1: u1,
            }),
            ///  Event Enable Register.
            EVENTEN: mmio.Mmio(packed struct(u32) {
                ///  Enable DMA event. When this bit is set, a DMA event will cause an RXEV event to wake the CPU from WFE sleep mode.
                DMA: u1,
                ///  Enable RXEV pin event. When this bit is set, a logic high of GPIO1.8 will cause an RXEV event to wake the CPU from WFE sleep mode.
                RX: u1,
                ///  Enable TXEV pin event. When this bit is set, TXEV event from the CPU is output to GPIO1.9.
                TX: u1,
                padding: u29,
            }),
            ///  Revision Register.
            REVISION: mmio.Mmio(packed struct(u32) {
                ///  Manufacturer Chip Revision.
                REVISION: u16,
                padding: u16,
            }),
            ///  System Status Interrupt Enable Register.
            SYSIE: mmio.Mmio(packed struct(u32) {
                ///  ARM ICE Unlock Interrupt Enable.
                ICEUNLOCK: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  disabled.
                        dis = 0x0,
                        ///  enabled.
                        en = 0x1,
                    },
                },
                padding: u31,
            }),
            reserved100: [12]u8,
            ///  ECC Error Register
            ECCERR: mmio.Mmio(packed struct(u32) {
                ///  ECC System RAM0 Error Flag. Write 1 to clear.
                RAM: u1,
                padding: u31,
            }),
            ///  ECC Not Double Error Detect Register
            ECCCED: mmio.Mmio(packed struct(u32) {
                ///  ECC System RAM0 Error Flag. Write 1 to clear.
                RAM: u1,
                padding: u31,
            }),
            ///  ECC IRQ Enable Register
            ECCIE: mmio.Mmio(packed struct(u32) {
                ///  ECC System RAM0 Error Interrup Enable
                RAM: u1,
                padding: u31,
            }),
            ///  ECC Error Address Register
            ECCADDR: mmio.Mmio(packed struct(u32) {
                ///  ECC Error Address.
                ECCERRAD: u32,
            }),
            reserved128: [12]u8,
            ///  General Purpose Register.
            GPR: u32,
        };

        ///  Global Control Function Register.
        pub const GCFR = extern struct {
            ///  Register 0.
            REG0: mmio.Mmio(packed struct(u32) {
                ///  CNNx16_0 Power Domain Enable
                cnnx16_0_pwr_en: u1,
                ///  CNNx16_1 Power Domain Enable
                cnnx16_1_pwr_en: u1,
                ///  CNNx16_2 Power Domain Enable
                cnnx16_2_pwr_en: u1,
                ///  CNNx16_3 Power Domain Enable
                cnnx16_3_pwr_en: u1,
                padding: u28,
            }),
            ///  Register 1.
            REG1: mmio.Mmio(packed struct(u32) {
                ///  CNNx16_0 RAM Power Enable
                cnnx16_0_ram_en: u1,
                ///  CNNx16_1 RAM Power Enable
                cnnx16_1_ram_en: u1,
                ///  CNNx16_2 RAM Power Enable
                cnnx16_2_ram_en: u1,
                ///  CNNx16_3 RAM Power Enable
                cnnx16_3_ram_en: u1,
                padding: u28,
            }),
            ///  Register 2.
            REG2: mmio.Mmio(packed struct(u32) {
                ///  CNNx16_0 Power Domain Isolation
                cnnx16_0_iso: u1,
                ///  CNNx16_1 Power Domain Isolation
                cnnx16_1_iso: u1,
                ///  CNNx16_2 Power Domain Isolation
                cnnx16_2_iso: u1,
                ///  CNNx16_3 Power Domain Isolation
                cnnx16_3_iso: u1,
                padding: u28,
            }),
            ///  Register 3.
            REG3: mmio.Mmio(packed struct(u32) {
                ///  CNNx16_0 Power Domain Reset
                cnnx16_0_rst: u1,
                ///  CNNx16_1 Power Domain Reset
                cnnx16_1_rst: u1,
                ///  CNNx16_2 Power Domain Reset
                cnnx16_2_rst: u1,
                ///  CNNx16_3 Power Domain Reset
                cnnx16_3_rst: u1,
                padding: u28,
            }),
        };

        ///  Individual I/O for each GPIO
        pub const GPIO0 = extern struct {
            ///  GPIO Function Enable Register. Each bit controls the GPIO_EN setting for one GPIO pin on the associated port.
            EN0: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_EN: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  Alternate function enabled.
                        ALTERNATE = 0x0,
                        ///  GPIO function is enabled.
                        GPIO = 0x1,
                        _,
                    },
                },
            }),
            ///  GPIO Set Function Enable Register. Writing a 1 to one or more bits in this register sets the bits in the same positions in GPIO_EN to 1, without affecting other bits in that register.
            EN0_SET: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                ALL: u32,
            }),
            ///  GPIO Clear Function Enable Register. Writing a 1 to one or more bits in this register clears the bits in the same positions in GPIO_EN to 0, without affecting other bits in that register.
            EN0_CLR: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                ALL: u32,
            }),
            ///  GPIO Output Enable Register. Each bit controls the GPIO_OUT_EN setting for one GPIO pin in the associated port.
            OUTEN: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                EN: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  GPIO Output Disable
                        dis = 0x0,
                        ///  GPIO Output Enable
                        en = 0x1,
                        _,
                    },
                },
            }),
            ///  GPIO Output Enable Set Function Enable Register. Writing a 1 to one or more bits in this register sets the bits in the same positions in GPIO_OUT_EN to 1, without affecting other bits in that register.
            OUTEN_SET: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                ALL: u32,
            }),
            ///  GPIO Output Enable Clear Function Enable Register. Writing a 1 to one or more bits in this register clears the bits in the same positions in GPIO_OUT_EN to 0, without affecting other bits in that register.
            OUTEN_CLR: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                ALL: u32,
            }),
            ///  GPIO Output Register. Each bit controls the GPIO_OUT setting for one pin in the associated port. This register can be written either directly, or by using the GPIO_OUT_SET and GPIO_OUT_CLR registers.
            OUT: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_OUT: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  Drive Logic 0 (low) on GPIO output.
                        low = 0x0,
                        ///  Drive logic 1 (high) on GPIO output.
                        high = 0x1,
                        _,
                    },
                },
            }),
            ///  GPIO Output Set. Writing a 1 to one or more bits in this register sets the bits in the same positions in GPIO_OUT to 1, without affecting other bits in that register.
            OUT_SET: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_OUT_SET: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  No Effect.
                        no = 0x0,
                        ///  Set GPIO_OUT bit in this position to '1'
                        set = 0x1,
                        _,
                    },
                },
            }),
            ///  GPIO Output Clear. Writing a 1 to one or more bits in this register clears the bits in the same positions in GPIO_OUT to 0, without affecting other bits in that register.
            OUT_CLR: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_OUT_CLR: u32,
            }),
            ///  GPIO Input Register. Read-only register to read from the logic states of the GPIO pins on this port.
            IN: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_IN: u32,
            }),
            ///  GPIO Interrupt Mode Register. Each bit in this register controls the interrupt mode setting for the associated GPIO pin on this port.
            INTMODE: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_INTMODE: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  Interrupts for this pin are level triggered.
                        level = 0x0,
                        ///  Interrupts for this pin are edge triggered.
                        edge = 0x1,
                        _,
                    },
                },
            }),
            ///  GPIO Interrupt Polarity Register. Each bit in this register controls the interrupt polarity setting for one GPIO pin in the associated port.
            INTPOL: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_INTPOL: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  Interrupts are latched on a falling edge or low level condition for this pin.
                        falling = 0x0,
                        ///  Interrupts are latched on a rising edge or high condition for this pin.
                        rising = 0x1,
                        _,
                    },
                },
            }),
            ///  GPIO Input Enable
            INEN: u32,
            ///  GPIO Interrupt Enable Register. Each bit in this register controls the GPIO interrupt enable for the associated pin on the GPIO port.
            INTEN: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_INTEN: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  Interrupts are disabled for this GPIO pin.
                        dis = 0x0,
                        ///  Interrupts are enabled for this GPIO pin.
                        en = 0x1,
                        _,
                    },
                },
            }),
            ///  GPIO Interrupt Enable Set. Writing a 1 to one or more bits in this register sets the bits in the same positions in GPIO_INT_EN to 1, without affecting other bits in that register.
            INTEN_SET: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_INTEN_SET: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  No effect.
                        no = 0x0,
                        ///  Set GPIO_INT_EN bit in this position to '1'
                        set = 0x1,
                        _,
                    },
                },
            }),
            ///  GPIO Interrupt Enable Clear. Writing a 1 to one or more bits in this register clears the bits in the same positions in GPIO_INT_EN to 0, without affecting other bits in that register.
            INTEN_CLR: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_INTEN_CLR: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  No Effect.
                        no = 0x0,
                        ///  Clear GPIO_INT_EN bit in this position to '0'
                        clear = 0x1,
                        _,
                    },
                },
            }),
            ///  GPIO Interrupt Status Register. Each bit in this register contains the pending interrupt status for the associated GPIO pin in this port.
            INTFL: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_INTFL: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  No Interrupt is pending on this GPIO pin.
                        no = 0x0,
                        ///  An Interrupt is pending on this GPIO pin.
                        pending = 0x1,
                        _,
                    },
                },
            }),
            reserved72: [4]u8,
            ///  GPIO Status Clear. Writing a 1 to one or more bits in this register clears the bits in the same positions in GPIO_INT_STAT to 0, without affecting other bits in that register.
            INTFL_CLR: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                ALL: u32,
            }),
            ///  GPIO Wake Enable Register. Each bit in this register controls the PMU wakeup enable for the associated GPIO pin in this port.
            WKEN: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_WKEN: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  PMU wakeup for this GPIO is disabled.
                        dis = 0x0,
                        ///  PMU wakeup for this GPIO is enabled.
                        en = 0x1,
                        _,
                    },
                },
            }),
            ///  GPIO Wake Enable Set. Writing a 1 to one or more bits in this register sets the bits in the same positions in GPIO_WAKE_EN to 1, without affecting other bits in that register.
            WKEN_SET: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                ALL: u32,
            }),
            ///  GPIO Wake Enable Clear. Writing a 1 to one or more bits in this register clears the bits in the same positions in GPIO_WAKE_EN to 0, without affecting other bits in that register.
            WKEN_CLR: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                ALL: u32,
            }),
            reserved92: [4]u8,
            ///  GPIO Interrupt Dual Edge Mode Register. Each bit in this register selects dual edge mode for the associated GPIO pin in this port.
            DUALEDGE: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_DUALEDGE: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  No Effect.
                        no = 0x0,
                        ///  Dual Edge mode is enabled. If edge-triggered interrupts are enabled on this GPIO pin, then both rising and falling edges will trigger interrupts regardless of the GPIO_INT_POL setting.
                        en = 0x1,
                        _,
                    },
                },
            }),
            ///  GPIO Input Mode Config 1. Each bit in this register enables the weak pull-up for the associated GPIO pin in this port.
            PADCTRL0: mmio.Mmio(packed struct(u32) {
                ///  The two bits in GPIO_PAD_CFG1 and GPIO_PAD_CFG2 for each GPIO pin work together to determine the pad mode when the GPIO is set to input mode.
                GPIO_PADCTRL0: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  High Impedance.
                        impedance = 0x0,
                        ///  Weak pull-up mode.
                        pu = 0x1,
                        ///  weak pull-down mode.
                        pd = 0x2,
                        _,
                    },
                },
            }),
            ///  GPIO Input Mode Config 2. Each bit in this register enables the weak pull-up for the associated GPIO pin in this port.
            PADCTRL1: mmio.Mmio(packed struct(u32) {
                ///  The two bits in GPIO_PAD_CFG1 and GPIO_PAD_CFG2 for each GPIO pin work together to determine the pad mode when the GPIO is set to input mode.
                GPIO_PADCTRL1: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  High Impedance.
                        impedance = 0x0,
                        ///  Weak pull-up mode.
                        pu = 0x1,
                        ///  weak pull-down mode.
                        pd = 0x2,
                        _,
                    },
                },
            }),
            ///  GPIO Alternate Function Enable Register. Each bit in this register selects between primary/secondary functions for the associated GPIO pin in this port.
            EN1: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_EN1: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  Primary function selected.
                        primary = 0x0,
                        ///  Secondary function selected.
                        secondary = 0x1,
                        _,
                    },
                },
            }),
            ///  GPIO Alternate Function Set. Writing a 1 to one or more bits in this register sets the bits in the same positions in GPIO_EN1 to 1, without affecting other bits in that register.
            EN1_SET: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                ALL: u32,
            }),
            ///  GPIO Alternate Function Clear. Writing a 1 to one or more bits in this register clears the bits in the same positions in GPIO_EN1 to 0, without affecting other bits in that register.
            EN1_CLR: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                ALL: u32,
            }),
            ///  GPIO Alternate Function Enable Register. Each bit in this register selects between primary/secondary functions for the associated GPIO pin in this port.
            EN2: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_EN2: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  Primary function selected.
                        primary = 0x0,
                        ///  Secondary function selected.
                        secondary = 0x1,
                        _,
                    },
                },
            }),
            ///  GPIO Alternate Function 2 Set. Writing a 1 to one or more bits in this register sets the bits in the same positions in GPIO_EN2 to 1, without affecting other bits in that register.
            EN2_SET: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                ALL: u32,
            }),
            ///  GPIO Wake Alternate Function Clear. Writing a 1 to one or more bits in this register clears the bits in the same positions in GPIO_EN2 to 0, without affecting other bits in that register.
            EN2_CLR: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                ALL: u32,
            }),
            reserved168: [40]u8,
            ///  GPIO Input Hysteresis Enable.
            HYSEN: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_HYSEN: u32,
            }),
            ///  GPIO Slew Rate Enable Register.
            SRSEL: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_SRSEL: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  Fast Slew Rate selected.
                        FAST = 0x0,
                        ///  Slow Slew Rate selected.
                        SLOW = 0x1,
                        _,
                    },
                },
            }),
            ///  GPIO Drive Strength Register. Each bit in this register selects the drive strength for the associated GPIO pin in this port. Refer to the Datasheet for sink/source current of GPIO pins in each mode.
            DS0: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_DS0: packed union {
                    raw: u32,
                    value: enum(u32) {
                        ///  GPIO port pin is in low-drive mode.
                        ld = 0x0,
                        ///  GPIO port pin is in high-drive mode.
                        hd = 0x1,
                        _,
                    },
                },
            }),
            ///  GPIO Drive Strength 1 Register. Each bit in this register selects the drive strength for the associated GPIO pin in this port. Refer to the Datasheet for sink/source current of GPIO pins in each mode.
            DS1: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                GPIO_DS1: u32,
            }),
            ///  GPIO Pull Select Mode.
            PS: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                ALL: u32,
            }),
            reserved192: [4]u8,
            ///  GPIO Voltage Select.
            VSSEL: mmio.Mmio(packed struct(u32) {
                ///  Mask of all of the pins on the port.
                ALL: u32,
            }),
        };

        ///  32-bit reloadable timer that can be used for timing and wakeup.
        pub const WUT = extern struct {
            ///  Count. This register stores the current timer count.
            CNT: mmio.Mmio(packed struct(u32) {
                ///  Timer Count Value.
                COUNT: u32,
            }),
            ///  Compare. This register stores the compare value, which is used to set the maximum count value to initiate a reload of the timer to 0x0001.
            CMP: mmio.Mmio(packed struct(u32) {
                ///  Timer Compare Value.
                COMPARE: u32,
            }),
            reserved12: [4]u8,
            ///  Clear Interrupt. Writing a value (0 or 1) to a bit in this register clears the associated interrupt.
            INTR: mmio.Mmio(packed struct(u32) {
                ///  Clear Interrupt.
                IRQ_CLR: u1,
                padding: u31,
            }),
            ///  Timer Control Register.
            CTRL: mmio.Mmio(packed struct(u32) {
                ///  Timer Mode.
                TMODE: packed union {
                    raw: u3,
                    value: enum(u3) {
                        ///  One Shot Mode.
                        oneShot = 0x0,
                        ///  Continuous Mode.
                        continuous = 0x1,
                        ///  Counter Mode.
                        counter = 0x2,
                        ///  Capture Mode.
                        capture = 0x4,
                        ///  Compare Mode.
                        compare = 0x5,
                        ///  Gated Mode.
                        gated = 0x6,
                        ///  Capture/Compare Mode.
                        captureCompare = 0x7,
                        _,
                    },
                },
                ///  Prescaler. Set the Timer's prescaler value. The prescaler divides the PCLK input to the timer and sets the Timer's Count Clock, F_CNT_CLK = PCLK(HZ)/prescaler. The Timer's prescaler setting is a 4-bit value with pres3:pres[2:0].
                PRES: packed union {
                    raw: u3,
                    value: enum(u3) {
                        ///  Divide by 1.
                        div1 = 0x0,
                        ///  Divide by 2.
                        div2 = 0x1,
                        ///  Divide by 4.
                        div4 = 0x2,
                        ///  Divide by 8.
                        div8 = 0x3,
                        ///  Divide by 16.
                        div16 = 0x4,
                        ///  Divide by 32.
                        div32 = 0x5,
                        ///  Divide by 64.
                        div64 = 0x6,
                        ///  Divide by 128.
                        div128 = 0x7,
                    },
                },
                ///  Timer input/output polarity bit.
                TPOL: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Active High.
                        activeHi = 0x0,
                        ///  Active Low.
                        activeLo = 0x1,
                    },
                },
                ///  Timer Enable.
                TEN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable.
                        dis = 0x0,
                        ///  Enable.
                        en = 0x1,
                    },
                },
                ///  MSB of prescaler value.
                PRES3: u1,
                padding: u23,
            }),
            ///  Timer Non-Overlapping Compare Register.
            NOLCMP: mmio.Mmio(packed struct(u32) {
                ///  Non-overlapping Low Compare. The 8-bit timer count value of non-overlapping time between falling edge of PWM output 0A and next rising edge of PWM output 0A'.
                NOLLCMP: u8,
                ///  Non-overlapping High Compare. The 8-bit timer count value of non-overlapping time between falling edge of PWM output 0A' and next rising edge of PWM output 0A.
                NOLHCMP: u8,
                padding: u16,
            }),
            ///  Preset register.
            PRESET: mmio.Mmio(packed struct(u32) {
                ///  Preset Value.
                PRESET: u32,
            }),
            ///  Reload register.
            RELOAD: mmio.Mmio(packed struct(u32) {
                ///  Rerload Value.
                RELOAD: u32,
            }),
            ///  Snapshot register.
            SNAPSHOT: mmio.Mmio(packed struct(u32) {
                ///  Snapshot Value.
                SNAPSHOT: u32,
            }),
        };

        ///  SPI peripheral.
        pub const SPI0 = extern struct {
            ///  Register for reading and writing the FIFO.
            FIFO32: mmio.Mmio(packed struct(u32) {
                ///  Read to pull from RX FIFO, write to put into TX FIFO.
                DATA: u32,
            }),
            ///  Register for controlling SPI peripheral.
            CTRL0: mmio.Mmio(packed struct(u32) {
                ///  SPI Enable.
                EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  SPI is disabled.
                        dis = 0x0,
                        ///  SPI is enabled.
                        en = 0x1,
                    },
                },
                ///  Master Mode Enable.
                MST_MODE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  SPI is Slave mode.
                        dis = 0x0,
                        ///  SPI is Master mode.
                        en = 0x1,
                    },
                },
                reserved4: u2,
                ///  Slave Select 0, IO direction, to support Multi-Master mode,Slave Select 0 can be input in Master mode. This bit has no effect in slave mode.
                SS_IO: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Slave select 0 is output.
                        output = 0x0,
                        ///  Slave Select 0 is input, only valid if MMEN=1.
                        input = 0x1,
                    },
                },
                ///  Start Transmit.
                START: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Master Initiates a transaction, this bit is self clearing when transactions are done. If a transaction cimpletes, and the TX FIFO is empty, the Master halts, if a transaction completes, and the TX FIFO is not empty, the Master initiates another transaction.
                        start = 0x1,
                        _,
                    },
                },
                reserved8: u2,
                ///  Start Select Control. Used in Master mode to control the behavior of the Slave Select signal at the end of a transaction.
                SS_CTRL: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  SPI De-asserts Slave Select at the end of a transaction.
                        DEASSERT = 0x0,
                        ///  SPI leaves Slave Select asserted at the end of a transaction.
                        ASSERT = 0x1,
                    },
                },
                reserved16: u7,
                ///  Slave Select, when in Master mode selects which Slave devices are selected. More than one Slave device can be selected.
                SS_ACTIVE: packed union {
                    raw: u4,
                    value: enum(u4) {
                        ///  SS0 is selected.
                        SS0 = 0x1,
                        ///  SS1 is selected.
                        SS1 = 0x2,
                        ///  SS2 is selected.
                        SS2 = 0x4,
                        ///  SS3 is selected.
                        SS3 = 0x8,
                        _,
                    },
                },
                padding: u12,
            }),
            ///  Register for controlling SPI peripheral.
            CTRL1: mmio.Mmio(packed struct(u32) {
                ///  Nubmer of Characters to transmit.
                TX_NUM_CHAR: u16,
                ///  Nubmer of Characters to receive.
                RX_NUM_CHAR: u16,
            }),
            ///  Register for controlling SPI peripheral.
            CTRL2: mmio.Mmio(packed struct(u32) {
                ///  Clock Phase.
                CLKPHA: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Data Sampled on clock rising edge. Use when in SPI Mode 0 and Mode 2
                        Rising_Edge = 0x0,
                        ///  Data Sampled on clock falling edge. Use when in SPI Mode 1 and Mode 3
                        Falling_Edge = 0x1,
                    },
                },
                ///  Clock Polarity.
                CLKPOL: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Normal Clock. Use when in SPI Mode 0 and Mode 1
                        Normal = 0x0,
                        ///  Inverted Clock. Use when in SPI Mode 2 and Mode 3
                        Inverted = 0x1,
                    },
                },
                reserved8: u6,
                ///  Number of Bits per character.
                NUMBITS: packed union {
                    raw: u4,
                    value: enum(u4) {
                        ///  16 bits per character.
                        @"16" = 0x0,
                        ///  1 bits per character.
                        @"1" = 0x1,
                        ///  2 bits per character.
                        @"2" = 0x2,
                        ///  3 bits per character.
                        @"3" = 0x3,
                        ///  4 bits per character.
                        @"4" = 0x4,
                        ///  5 bits per character.
                        @"5" = 0x5,
                        ///  6 bits per character.
                        @"6" = 0x6,
                        ///  7 bits per character.
                        @"7" = 0x7,
                        ///  8 bits per character.
                        @"8" = 0x8,
                        ///  9 bits per character.
                        @"9" = 0x9,
                        ///  10 bits per character.
                        @"10" = 0xa,
                        ///  11 bits per character.
                        @"11" = 0xb,
                        ///  12 bits per character.
                        @"12" = 0xc,
                        ///  13 bits per character.
                        @"13" = 0xd,
                        ///  14 bits per character.
                        @"14" = 0xe,
                        ///  15 bits per character.
                        @"15" = 0xf,
                    },
                },
                ///  SPI Data width.
                DATA_WIDTH: packed union {
                    raw: u2,
                    value: enum(u2) {
                        ///  1 data pin.
                        Mono = 0x0,
                        ///  2 data pins.
                        Dual = 0x1,
                        ///  4 data pins.
                        Quad = 0x2,
                        _,
                    },
                },
                reserved15: u1,
                ///  Three Wire mode. MOSI/MISO pin (s) shared. Only Mono mode suports Four-Wire.
                THREE_WIRE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Use four wire mode (Mono only).
                        dis = 0x0,
                        ///  Use three wire mode.
                        en = 0x1,
                    },
                },
                ///  Slave Select Polarity, each Slave Select can have unique polarity.
                SS_POL: packed union {
                    raw: u8,
                    value: enum(u8) {
                        ///  SS0 active high.
                        SS0_high = 0x1,
                        ///  SS1 active high.
                        SS1_high = 0x2,
                        ///  SS2 active high.
                        SS2_high = 0x4,
                        ///  SS3 active high.
                        SS3_high = 0x8,
                        _,
                    },
                },
                padding: u8,
            }),
            ///  Register for controlling SPI peripheral/Slave Select Timing.
            SSTIME: mmio.Mmio(packed struct(u32) {
                ///  Slave Select Pre delay 1.
                PRE: packed union {
                    raw: u8,
                    value: enum(u8) {
                        ///  256 system clocks between SS active and first serial clock edge.
                        @"256" = 0x0,
                        _,
                    },
                },
                ///  Slave Select Post delay 2.
                POST: packed union {
                    raw: u8,
                    value: enum(u8) {
                        ///  256 system clocks between last serial clock edge and SS inactive.
                        @"256" = 0x0,
                        _,
                    },
                },
                ///  Slave Select Inactive delay.
                INACT: packed union {
                    raw: u8,
                    value: enum(u8) {
                        ///  256 system clocks between transactions.
                        @"256" = 0x0,
                        _,
                    },
                },
                padding: u8,
            }),
            ///  Register for controlling SPI clock rate.
            CLKCTRL: mmio.Mmio(packed struct(u32) {
                ///  Low duty cycle control. In timer mode, reload[7:0].
                LO: packed union {
                    raw: u8,
                    value: enum(u8) {
                        ///  Duty cycle control of serial clock generation is disabled.
                        Dis = 0x0,
                        _,
                    },
                },
                ///  High duty cycle control. In timer mode, reload[15:8].
                HI: packed union {
                    raw: u8,
                    value: enum(u8) {
                        ///  Duty cycle control of serial clock generation is disabled.
                        Dis = 0x0,
                        _,
                    },
                },
                ///  System Clock scale factor. Scales the AMBA clock by 2^SCALE before generating serial clock.
                CLKDIV: u4,
                padding: u12,
            }),
            reserved28: [4]u8,
            ///  Register for controlling DMA.
            DMA: mmio.Mmio(packed struct(u32) {
                ///  Transmit FIFO level that will trigger a DMA request, also level for threshold status. When TX FIFO has fewer than this many bytes, the associated events and conditions are triggered.
                TX_THD_VAL: u5,
                reserved6: u1,
                ///  Transmit FIFO enabled for SPI transactions.
                TX_FIFO_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Transmit FIFO is not enabled.
                        dis = 0x0,
                        ///  Transmit FIFO is enabled.
                        en = 0x1,
                    },
                },
                ///  Clear TX FIFO, clear is accomplished by resetting the read and write pointers. This should be done when FIFO is not being accessed on the SPI side.
                TX_FLUSH: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Clear the Transmit FIFO, clears any pending TX FIFO status.
                        CLEAR = 0x1,
                        _,
                    },
                },
                ///  Count of entries in TX FIFO.
                TX_LVL: u6,
                reserved15: u1,
                ///  TX DMA Enable.
                DMA_TX_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  TX DMA requests are disabled, andy pending DMA requests are cleared.
                        DIS = 0x0,
                        ///  TX DMA requests are enabled.
                        en = 0x1,
                    },
                },
                ///  Receive FIFO level that will trigger a DMA request, also level for threshold status. When RX FIFO has more than this many bytes, the associated events and conditions are triggered.
                RX_THD_VAL: u5,
                reserved22: u1,
                ///  Receive FIFO enabled for SPI transactions.
                RX_FIFO_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Receive FIFO is not enabled.
                        DIS = 0x0,
                        ///  Receive FIFO is enabled.
                        en = 0x1,
                    },
                },
                ///  Clear RX FIFO, clear is accomplished by resetting the read and write pointers. This should be done when FIFO is not being accessed on the SPI side.
                RX_FLUSH: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Clear the Receive FIFO, clears any pending RX FIFO status.
                        CLEAR = 0x1,
                        _,
                    },
                },
                ///  Count of entries in RX FIFO.
                RX_LVL: u6,
                reserved31: u1,
                ///  RX DMA Enable.
                DMA_RX_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  RX DMA requests are disabled, any pending DMA requests are cleared.
                        dis = 0x0,
                        ///  RX DMA requests are enabled.
                        en = 0x1,
                    },
                },
            }),
            ///  Register for reading and clearing interrupt flags. All bits are write 1 to clear.
            INTFL: mmio.Mmio(packed struct(u32) {
                ///  TX FIFO Threshold Crossed.
                TX_THD: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                ///  TX FIFO Empty.
                TX_EM: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                ///  RX FIFO Threshold Crossed.
                RX_THD: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                ///  RX FIFO FULL.
                RX_FULL: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                ///  Slave Select Asserted.
                SSA: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                ///  Slave Select Deasserted.
                SSD: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                reserved8: u2,
                ///  Multi-Master Mode Fault.
                FAULT: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                ///  Slave Abort Detected.
                ABORT: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                reserved11: u1,
                ///  Master Done, set when SPI Master has completed any transactions.
                MST_DONE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                ///  Transmit FIFO Overrun, set when the AMBA side attempts to write data to a full transmit FIFO.
                TX_OV: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                ///  Transmit FIFO Underrun, set when the SPI side attempts to read data from an empty transmit FIFO.
                TX_UN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                ///  Receive FIFO Overrun, set when the SPI side attempts to write to a full receive FIFO.
                RX_OV: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                ///  Receive FIFO Underrun, set when the AMBA side attempts to read data from an empty receive FIFO.
                RX_UN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                padding: u16,
            }),
            ///  Register for enabling interrupts.
            INTEN: mmio.Mmio(packed struct(u32) {
                ///  TX FIFO Threshold interrupt enable.
                TX_THD: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt is disabled.
                        dis = 0x0,
                        ///  Interrupt is enabled.
                        en = 0x1,
                    },
                },
                ///  TX FIFO Empty interrupt enable.
                TX_EM: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt is disabled.
                        dis = 0x0,
                        ///  Interrupt is enabled.
                        en = 0x1,
                    },
                },
                ///  RX FIFO Threshold Crossed interrupt enable.
                RX_THD: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt is disabled.
                        dis = 0x0,
                        ///  Interrupt is enabled.
                        en = 0x1,
                    },
                },
                ///  RX FIFO FULL interrupt enable.
                RX_FULL: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt is disabled.
                        dis = 0x0,
                        ///  Interrupt is enabled.
                        en = 0x1,
                    },
                },
                ///  Slave Select Asserted interrupt enable.
                SSA: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt is disabled.
                        dis = 0x0,
                        ///  Interrupt is enabled.
                        en = 0x1,
                    },
                },
                ///  Slave Select Deasserted interrupt enable.
                SSD: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt is disabled.
                        dis = 0x0,
                        ///  Interrupt is enabled.
                        en = 0x1,
                    },
                },
                reserved8: u2,
                ///  Multi-Master Mode Fault interrupt enable.
                FAULT: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt is disabled.
                        dis = 0x0,
                        ///  Interrupt is enabled.
                        en = 0x1,
                    },
                },
                ///  Slave Abort Detected interrupt enable.
                ABORT: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt is disabled.
                        dis = 0x0,
                        ///  Interrupt is enabled.
                        en = 0x1,
                    },
                },
                reserved11: u1,
                ///  Master Done interrupt enable.
                MST_DONE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt is disabled.
                        dis = 0x0,
                        ///  Interrupt is enabled.
                        en = 0x1,
                    },
                },
                ///  Transmit FIFO Overrun interrupt enable.
                TX_OV: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt is disabled.
                        dis = 0x0,
                        ///  Interrupt is enabled.
                        en = 0x1,
                    },
                },
                ///  Transmit FIFO Underrun interrupt enable.
                TX_UN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt is disabled.
                        dis = 0x0,
                        ///  Interrupt is enabled.
                        en = 0x1,
                    },
                },
                ///  Receive FIFO Overrun interrupt enable.
                RX_OV: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt is disabled.
                        dis = 0x0,
                        ///  Interrupt is enabled.
                        en = 0x1,
                    },
                },
                ///  Receive FIFO Underrun interrupt enable.
                RX_UN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt is disabled.
                        dis = 0x0,
                        ///  Interrupt is enabled.
                        en = 0x1,
                    },
                },
                padding: u16,
            }),
            ///  Register for wake up flags. All bits in this register are write 1 to clear.
            WKFL: mmio.Mmio(packed struct(u32) {
                ///  Wake on TX FIFO Threshold Crossed.
                TX_THD: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                ///  Wake on TX FIFO Empty.
                TX_EM: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                ///  Wake on RX FIFO Threshold Crossed.
                RX_THD: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                ///  Wake on RX FIFO Full.
                RX_FULL: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Flag is set when value read is 1. Write 1 to clear this flag.
                        clear = 0x1,
                        _,
                    },
                },
                padding: u28,
            }),
            ///  Register for wake up enable.
            WKEN: mmio.Mmio(packed struct(u32) {
                ///  Wake on TX FIFO Threshold Crossed Enable.
                TX_THD: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Wakeup source disabled.
                        dis = 0x0,
                        ///  Wakeup source enabled.
                        en = 0x1,
                    },
                },
                ///  Wake on TX FIFO Empty Enable.
                TX_EM: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Wakeup source disabled.
                        dis = 0x0,
                        ///  Wakeup source enabled.
                        en = 0x1,
                    },
                },
                ///  Wake on RX FIFO Threshold Crossed Enable.
                RX_THD: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Wakeup source disabled.
                        dis = 0x0,
                        ///  Wakeup source enabled.
                        en = 0x1,
                    },
                },
                ///  Wake on RX FIFO Full Enable.
                RX_FULL: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Wakeup source disabled.
                        dis = 0x0,
                        ///  Wakeup source enabled.
                        en = 0x1,
                    },
                },
                padding: u28,
            }),
            ///  SPI Status register.
            STAT: mmio.Mmio(packed struct(u32) {
                ///  SPI active status. In Master mode, set when transaction starts, cleared when last bit of last character is acted upon and Slave Select de-assertion would occur. In Slave mode, set when Slave Select is asserted, cleared when Slave Select is de-asserted. Not used in Timer mode.
                BUSY: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  SPI not active.
                        not = 0x0,
                        ///  SPI active.
                        active = 0x1,
                    },
                },
                padding: u31,
            }),
        };

        ///  Inter-Integrated Circuit.
        pub const I2C0 = extern struct {
            ///  Control Register0.
            CTRL: mmio.Mmio(packed struct(u32) {
                ///  I2C Enable.
                EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable I2C.
                        dis = 0x0,
                        ///  enable I2C.
                        en = 0x1,
                    },
                },
                ///  Master Mode Enable.
                MST_MODE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Slave Mode.
                        slave_mode = 0x0,
                        ///  Master Mode.
                        master_mode = 0x1,
                    },
                },
                ///  General Call Address Enable.
                GC_ADDR_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Ignore Gneral Call Address.
                        dis = 0x0,
                        ///  Acknowledge general call address.
                        en = 0x1,
                    },
                },
                ///  Interactive Receive Mode.
                IRXM_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable Interactive Receive Mode.
                        dis = 0x0,
                        ///  Enable Interactive Receive Mode.
                        en = 0x1,
                    },
                },
                ///  Data Acknowledge. This bit defines the acknowledge bit returned by the I2C receiver while IRXM = 1 HW forces ACK to 0 when IRXM = 0.
                IRXM_ACK: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  return ACK (pulling SDA LOW).
                        ack = 0x0,
                        ///  return NACK (leaving SDA HIGH).
                        nack = 0x1,
                    },
                },
                reserved6: u1,
                ///  SCL Output. This bits control SCL output when SWOE =1.
                SCL_OUT: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Drive SCL low.
                        drive_scl_low = 0x0,
                        ///  Release SCL.
                        release_scl = 0x1,
                    },
                },
                ///  SDA Output. This bits control SDA output when SWOE = 1.
                SDA_OUT: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Drive SDA low.
                        drive_sda_low = 0x0,
                        ///  Release SDA.
                        release_sda = 0x1,
                    },
                },
                ///  SCL status. This bit reflects the logic gate of SCL signal.
                SCL: u1,
                ///  SDA status. THis bit reflects the logic gate of SDA signal.
                SDA: u1,
                ///  Software Output Enable.
                BB_MODE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  I2C Outputs SCLO and SDAO disabled.
                        outputs_disable = 0x0,
                        ///  I2C Outputs SCLO and SDAO enabled.
                        outputs_enable = 0x1,
                    },
                },
                ///  Read. This bit reflects the R/W bit of an address match (AMI = 1) or general call match (GCI = 1). This bit is valid 3 cycles after the relevant interrupt bit is set.
                READ: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Write.
                        write = 0x0,
                        ///  Read.
                        read = 0x1,
                    },
                },
                ///  This bit will disable slave clock stretching when set.
                CLKSTR_DIS: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Slave clock stretching enabled.
                        en = 0x0,
                        ///  Slave clock stretching disabled.
                        dis = 0x1,
                    },
                },
                ///  SCL Push-Pull Mode. This bit controls whether SCL is operated in a the I2C standard open-drain mode, or in a non-standard push-pull mode where the Hi-Z output isreplaced with Drive-1. The non-standard mode should only be used when operating as a master and communicating with slaves that are guaranteed to never drive SCL low.
                ONE_MST_MODE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Standard open-drain operation: drive low for 0, Hi-Z for 1
                        dis = 0x0,
                        ///  Non-standard push-pull operation: drive low for 0, drive high for 1
                        en = 0x1,
                    },
                },
                reserved15: u1,
                ///  High speed mode enable
                HS_EN: u1,
                padding: u16,
            }),
            ///  Status Register.
            STATUS: mmio.Mmio(packed struct(u32) {
                ///  Bus Status.
                BUSY: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  I2C Bus Idle.
                        idle = 0x0,
                        ///  I2C Bus Busy.
                        busy = 0x1,
                    },
                },
                ///  RX empty.
                RX_EM: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Not Empty.
                        not_empty = 0x0,
                        ///  Empty.
                        empty = 0x1,
                    },
                },
                ///  RX Full.
                RX_FULL: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Not Full.
                        not_full = 0x0,
                        ///  Full.
                        full = 0x1,
                    },
                },
                ///  TX Empty.
                TX_EM: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Not Empty.
                        not_empty = 0x0,
                        ///  Empty.
                        empty = 0x1,
                    },
                },
                ///  TX Full.
                TX_FULL: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Not Empty.
                        not_empty = 0x0,
                        ///  Empty.
                        empty = 0x1,
                    },
                },
                ///  Clock Mode.
                MST_BUSY: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Device not actively driving SCL clock cycles.
                        not_actively_driving_scl_clock = 0x0,
                        ///  Device operating as master and actively driving SCL clock cycles.
                        actively_driving_scl_clock = 0x1,
                    },
                },
                padding: u26,
            }),
            ///  Interrupt Status Register.
            INTFL0: mmio.Mmio(packed struct(u32) {
                ///  Transfer Done Interrupt.
                DONE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  Interactive Receive Interrupt.
                IRXM: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  Slave General Call Address Match Interrupt.
                GC_ADDR_MATCH: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  Slave Address Match Interrupt.
                ADDR_MATCH: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  Receive Threshold Interrupt. This bit is automaticcaly cleared when RX_FIFO is below the threshold level.
                RX_THD: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No interrupt is pending.
                        inactive = 0x0,
                        ///  An interrupt is pending. RX_FIFO equal or more bytes than the threshold.
                        pending = 0x1,
                    },
                },
                ///  Transmit Threshold Interrupt. This bit is automaticcaly cleared when TX_FIFO is above the threshold level.
                TX_THD: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No interrupt is pending.
                        inactive = 0x0,
                        ///  An interrupt is pending. TX_FIFO has equal or less bytes than the threshold.
                        pending = 0x1,
                    },
                },
                ///  STOP Interrupt.
                STOP: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No interrupt is pending.
                        inactive = 0x0,
                        ///  An interrupt is pending. TX_FIFO has equal or less bytes than the threshold.
                        pending = 0x1,
                    },
                },
                ///  Address Acknowledge Interrupt.
                ADDR_ACK: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  Arbritation error Interrupt.
                ARB_ERR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  timeout Error Interrupt.
                TO_ERR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  Address NACK Error Interrupt.
                ADDR_NACK_ERR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  Data NACK Error Interrupt.
                DATA_ERR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  Do Not Respond Error Interrupt.
                DNR_ERR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  Start Error Interrupt.
                START_ERR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  Stop Error Interrupt.
                STOP_ERR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  Transmit Lock Out Interrupt.
                TX_LOCKOUT: u1,
                ///  Multiple Address Match Interrupt
                MAMI: u6,
                ///  Slave Read Address Match Interrupt
                RD_ADDR_MATCH: u1,
                ///  Slave Write Address Match Interrupt
                WR_ADDR_MATCH: u1,
                padding: u8,
            }),
            ///  Interrupt Enable Register.
            INTEN0: mmio.Mmio(packed struct(u32) {
                ///  Transfer Done Interrupt Enable.
                DONE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt disabled.
                        dis = 0x0,
                        ///  Interrupt enabled when DONE = 1.
                        en = 0x1,
                    },
                },
                ///  Description not available.
                IRXM: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt disabled.
                        dis = 0x0,
                        ///  Interrupt enabled when RX_MODE = 1.
                        en = 0x1,
                    },
                },
                ///  Slave mode general call address match received input enable.
                GC_ADDR_MATCH: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt disabled.
                        dis = 0x0,
                        ///  Interrupt enabled when GEN_CTRL_ADDR = 1.
                        en = 0x1,
                    },
                },
                ///  Slave mode incoming address match interrupt.
                ADDR_MATCH: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt disabled.
                        dis = 0x0,
                        ///  Interrupt enabled when ADDR_MATCH = 1.
                        en = 0x1,
                    },
                },
                ///  RX FIFO Above Treshold Level Interrupt Enable.
                RX_THD: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt disabled.
                        dis = 0x0,
                        ///  Interrupt enabled.
                        en = 0x1,
                    },
                },
                ///  TX FIFO Below Treshold Level Interrupt Enable.
                TX_THD: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt disabled.
                        dis = 0x0,
                        ///  Interrupt enabled.
                        en = 0x1,
                    },
                },
                ///  Stop Interrupt Enable
                STOP: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt disabled.
                        dis = 0x0,
                        ///  Interrupt enabled when STOP = 1.
                        en = 0x1,
                    },
                },
                ///  Received Address ACK from Slave Interrupt.
                ADDR_ACK: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt disabled.
                        dis = 0x0,
                        ///  Interrupt enabled.
                        en = 0x1,
                    },
                },
                ///  Master Mode Arbitration Lost Interrupt.
                ARB_ERR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt disabled.
                        dis = 0x0,
                        ///  Interrupt enabled.
                        en = 0x1,
                    },
                },
                ///  Timeout Error Interrupt Enable.
                TO_ERR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt disabled.
                        dis = 0x0,
                        ///  Interrupt enabled.
                        en = 0x1,
                    },
                },
                ///  Master Mode Address NACK Received Interrupt.
                ADDR_NACK_ERR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt disabled.
                        dis = 0x0,
                        ///  Interrupt enabled.
                        en = 0x1,
                    },
                },
                ///  Master Mode Data NACK Received Interrupt.
                DATA_ERR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt disabled.
                        dis = 0x0,
                        ///  Interrupt enabled.
                        en = 0x1,
                    },
                },
                ///  Slave Mode Do Not Respond Interrupt.
                DNR_ERR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt disabled.
                        dis = 0x0,
                        ///  Interrupt enabled.
                        en = 0x1,
                    },
                },
                ///  Out of Sequence START condition detected interrupt.
                START_ERR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt disabled.
                        dis = 0x0,
                        ///  Interrupt enabled.
                        en = 0x1,
                    },
                },
                ///  Out of Sequence STOP condition detected interrupt.
                STOP_ERR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Interrupt disabled.
                        dis = 0x0,
                        ///  Interrupt enabled.
                        en = 0x1,
                    },
                },
                ///  TX FIFO Locked Out Interrupt.
                TX_LOCKOUT: u1,
                ///  Multiple Address Match Interrupt
                MAMI: u6,
                ///  Slave Read Address Match Interrupt
                RD_ADDR_MATCH: u1,
                ///  Slave Write Address Match Interrupt
                WR_ADDR_MATCH: u1,
                padding: u8,
            }),
            ///  Interrupt Status Register 1.
            INTFL1: mmio.Mmio(packed struct(u32) {
                ///  Receiver Overflow Interrupt. When operating as a slave receiver, this bit is set when you reach the first data bit and the RX FIFO and shift register are both full.
                RX_OV: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  Transmit Underflow Interrupt. When operating as a slave transmitter, this bit is set when you reach the first data bit and the TX FIFO is empty and the master is still asking for more data (i.e the master hasn't sent a NACK yet).
                TX_UN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  START Condition Status Flag.
                START: u1,
                padding: u29,
            }),
            ///  Interrupt Staus Register 1.
            INTEN1: mmio.Mmio(packed struct(u32) {
                ///  Receiver Overflow Interrupt Enable.
                RX_OV: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        dis = 0x0,
                        ///  An interrupt is pending.
                        en = 0x1,
                    },
                },
                ///  Transmit Underflow Interrupt Enable.
                TX_UN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No Interrupt is Pending.
                        dis = 0x0,
                        ///  An interrupt is pending.
                        en = 0x1,
                    },
                },
                ///  START Condition Interrupt Enable.
                START: u1,
                padding: u29,
            }),
            ///  FIFO Configuration Register.
            FIFOLEN: mmio.Mmio(packed struct(u32) {
                ///  Receive FIFO Length.
                RX_DEPTH: u8,
                ///  Transmit FIFO Length.
                TX_DEPTH: u8,
                padding: u16,
            }),
            ///  Receive Control Register 0.
            RXCTRL0: mmio.Mmio(packed struct(u32) {
                ///  Do Not Respond.
                DNR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Always respond to address match.
                        respond = 0x0,
                        ///  Do not respond to address match when RX_FIFO is not empty.
                        not_respond_rx_fifo_empty = 0x1,
                    },
                },
                reserved7: u6,
                ///  Receive FIFO Flush. This bit is automatically cleared to 0 after the operation. Setting this bit to 1 will affect RX_FIFO status.
                FLUSH: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  FIFO not flushed.
                        not_flushed = 0x0,
                        ///  Flush RX_FIFO.
                        flush = 0x1,
                    },
                },
                ///  Receive FIFO Threshold. These bits define the RX_FIFO interrupt threshold.
                THD_LVL: u4,
                padding: u20,
            }),
            ///  Receive Control Register 1.
            RXCTRL1: mmio.Mmio(packed struct(u32) {
                ///  Receive Count Bits. These bits define the number of bytes to be received in a transaction, except for the case RXCNT = 0. RXCNT = 0 means 256 bytes to be received in a transaction.
                CNT: u8,
                ///  Receive FIFO Count. These bits reflect the number of byte in the RX_FIFO. These bits are flushed when I2CEN = 0.
                LVL: u4,
                padding: u20,
            }),
            ///  Transmit Control Register 0.
            TXCTRL0: mmio.Mmio(packed struct(u32) {
                ///  Transmit FIFO Preaload Mode. Setting this bit will allow for high speed application to preload the transmit FIFO prior to Slave Address Match.
                PRELOAD_MODE: u1,
                ///  Transmit FIFO Ready Manual Mode.
                TX_READY_MODE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  HW control of I2CTXRDY enabled.
                        en = 0x0,
                        ///  HW control of I2CTXRDY disabled.
                        dis = 0x1,
                    },
                },
                ///  TX FIFO General Call Address Match Auto Flush Disable.
                GC_ADDR_FLUSH_DIS: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Enabled.
                        en = 0x0,
                        ///  Disabled.
                        dis = 0x1,
                    },
                },
                ///  TX FIFO Slave Address Match Write Auto Flush Disable.
                WR_ADDR_FLUSH_DIS: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Enabled.
                        en = 0x0,
                        ///  Disabled.
                        dis = 0x1,
                    },
                },
                ///  TX FIFO Slave Address Match Read Auto Flush Disable.
                RD_ADDR_FLUSH_DIS: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Enabled.
                        en = 0x0,
                        ///  Disabled.
                        dis = 0x1,
                    },
                },
                ///  TX FIFO received NACK Auto Flush Disable.
                NACK_FLUSH_DIS: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Enabled.
                        en = 0x0,
                        ///  Disabled.
                        dis = 0x1,
                    },
                },
                reserved7: u1,
                ///  Transmit FIFO Flush. This bit is automatically cleared to 0 after the operation.
                FLUSH: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  FIFO not flushed.
                        not_flushed = 0x0,
                        ///  Flush TX_FIFO.
                        flush = 0x1,
                    },
                },
                ///  Transmit FIFO Threshold. These bits define the TX_FIFO interrupt threshold.
                THD_VAL: u4,
                padding: u20,
            }),
            ///  Transmit Control Register 1.
            TXCTRL1: mmio.Mmio(packed struct(u32) {
                ///  Transmit FIFO Preload Ready.
                PRELOAD_RDY: u1,
                reserved8: u7,
                ///  Transmit FIFO Count. These bits reflect the number of bytes in the TX_FIFO.
                LVL: u4,
                padding: u20,
            }),
            ///  Data Register.
            FIFO: mmio.Mmio(packed struct(u32) {
                ///  Data is read from or written to this location. Transmit and receive FIFO are separate but both are addressed at this location.
                DATA: u8,
                padding: u24,
            }),
            ///  Master Control Register.
            MSTCTRL: mmio.Mmio(packed struct(u32) {
                ///  Setting this bit to 1 will start a master transfer.
                START: u1,
                ///  Setting this bit to 1 will generate a repeated START.
                RESTART: u1,
                ///  Setting this bit to 1 will generate a STOP condition.
                STOP: u1,
                reserved7: u4,
                ///  Slave Extend Address Select.
                EX_ADDR_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  7-bit address.
                        @"7_bits_address" = 0x0,
                        ///  10-bit address.
                        @"10_bits_address" = 0x1,
                    },
                },
                padding: u24,
            }),
            ///  Clock Low Register.
            CLKLO: mmio.Mmio(packed struct(u32) {
                ///  Clock low. In master mode, these bits define the SCL low period. In slave mode, these bits define the time SCL will be held low after data is outputted.
                LO: u9,
                padding: u23,
            }),
            ///  Clock high Register.
            CLKHI: mmio.Mmio(packed struct(u32) {
                ///  Clock High. In master mode, these bits define the SCL high period.
                HI: u9,
                padding: u23,
            }),
            ///  Clock high Register.
            HSCLK: mmio.Mmio(packed struct(u32) {
                ///  Clock Low. This field sets the Hs-Mode clock low count. In Slave mode, this is the time SCL is held low after data is output on SDA.
                LO: u8,
                ///  Clock High. This field sets the Hs-Mode clock high count. In Slave mode, this is the time SCL is held high after data is output on SDA
                HI: u8,
                padding: u16,
            }),
            ///  Timeout Register
            TIMEOUT: mmio.Mmio(packed struct(u32) {
                ///  Timeout
                SCL_TO_VAL: u16,
                padding: u16,
            }),
            reserved72: [4]u8,
            ///  DMA Register.
            DMA: mmio.Mmio(packed struct(u32) {
                ///  TX channel enable.
                TX_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable.
                        dis = 0x0,
                        ///  Enable.
                        en = 0x1,
                    },
                },
                ///  RX channel enable.
                RX_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable.
                        dis = 0x0,
                        ///  Enable.
                        en = 0x1,
                    },
                },
                padding: u30,
            }),
            ///  Slave Address Register.
            SLAVE0: u32,
            ///  Slave Address Register.
            SLAVE1: u32,
            ///  Slave Address Register.
            SLAVE2: u32,
            ///  Slave Address Register.
            SLAVE3: u32,
        };

        ///  Windowed Watchdog Timer
        pub const WDT = extern struct {
            ///  Watchdog Timer Control Register.
            CTRL: mmio.Mmio(packed struct(u32) {
                ///  Windowed Watchdog Interrupt Upper Limit. Sets the number of WDTCLK cycles until a windowed watchdog timer interrupt is generated (if enabled) if the CPU does not write the windowed watchdog reset sequence to the WWDT_RST register before the watchdog timer has counted this time period since the last timer reset.
                INT_LATE_VAL: packed union {
                    raw: u4,
                    value: enum(u4) {
                        ///  2**31 clock cycles.
                        wdt2pow31 = 0x0,
                        ///  2**30 clock cycles.
                        wdt2pow30 = 0x1,
                        ///  2**29 clock cycles.
                        wdt2pow29 = 0x2,
                        ///  2**28 clock cycles.
                        wdt2pow28 = 0x3,
                        ///  2^27 clock cycles.
                        wdt2pow27 = 0x4,
                        ///  2**26 clock cycles.
                        wdt2pow26 = 0x5,
                        ///  2**25 clock cycles.
                        wdt2pow25 = 0x6,
                        ///  2**24 clock cycles.
                        wdt2pow24 = 0x7,
                        ///  2**23 clock cycles.
                        wdt2pow23 = 0x8,
                        ///  2**22 clock cycles.
                        wdt2pow22 = 0x9,
                        ///  2**21 clock cycles.
                        wdt2pow21 = 0xa,
                        ///  2**20 clock cycles.
                        wdt2pow20 = 0xb,
                        ///  2**19 clock cycles.
                        wdt2pow19 = 0xc,
                        ///  2**18 clock cycles.
                        wdt2pow18 = 0xd,
                        ///  2**17 clock cycles.
                        wdt2pow17 = 0xe,
                        ///  2**16 clock cycles.
                        wdt2pow16 = 0xf,
                    },
                },
                ///  Windowed Watchdog Reset Upper Limit. Sets the number of WDTCLK cycles until a system reset occurs (if enabled) if the CPU does not write the watchdog reset sequence to the WDT_RST register before the watchdog timer has counted this time period since the last timer reset.
                RST_LATE_VAL: packed union {
                    raw: u4,
                    value: enum(u4) {
                        ///  2**31 clock cycles.
                        wdt2pow31 = 0x0,
                        ///  2**30 clock cycles.
                        wdt2pow30 = 0x1,
                        ///  2**29 clock cycles.
                        wdt2pow29 = 0x2,
                        ///  2**28 clock cycles.
                        wdt2pow28 = 0x3,
                        ///  2^27 clock cycles.
                        wdt2pow27 = 0x4,
                        ///  2**26 clock cycles.
                        wdt2pow26 = 0x5,
                        ///  2**25 clock cycles.
                        wdt2pow25 = 0x6,
                        ///  2**24 clock cycles.
                        wdt2pow24 = 0x7,
                        ///  2**23 clock cycles.
                        wdt2pow23 = 0x8,
                        ///  2**22 clock cycles.
                        wdt2pow22 = 0x9,
                        ///  2**21 clock cycles.
                        wdt2pow21 = 0xa,
                        ///  2**20 clock cycles.
                        wdt2pow20 = 0xb,
                        ///  2**19 clock cycles.
                        wdt2pow19 = 0xc,
                        ///  2**18 clock cycles.
                        wdt2pow18 = 0xd,
                        ///  2**17 clock cycles.
                        wdt2pow17 = 0xe,
                        ///  2**16 clock cycles.
                        wdt2pow16 = 0xf,
                    },
                },
                ///  Windowed Watchdog Timer Enable.
                EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable.
                        dis = 0x0,
                        ///  Enable.
                        en = 0x1,
                    },
                },
                ///  Windowed Watchdog Timer Interrupt Flag Too Late.
                INT_LATE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No interrupt is pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                ///  Windowed Watchdog Timer Interrupt Enable.
                WDT_INT_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable.
                        dis = 0x0,
                        ///  Enable.
                        en = 0x1,
                    },
                },
                ///  Windowed Watchdog Timer Reset Enable.
                WDT_RST_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable.
                        dis = 0x0,
                        ///  Enable.
                        en = 0x1,
                    },
                },
                ///  Windowed Watchdog Timer Interrupt Flag Too Soon.
                INT_EARLY: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No interrupt is pending.
                        inactive = 0x0,
                        ///  An interrupt is pending.
                        pending = 0x1,
                    },
                },
                reserved16: u3,
                ///  Windowed Watchdog Interrupt Lower Limit. Sets the number of WDTCLK cycles that establishes the lower boundary of the watchdog window. A windowed watchdog timer interrupt is generated (if enabled) if the CPU writes the windowed watchdog reset sequence to the WWDT_RST register before the watchdog timer has counted this time period since the last timer reset.
                INT_EARLY_VAL: packed union {
                    raw: u4,
                    value: enum(u4) {
                        ///  2**31 clock cycles.
                        wdt2pow31 = 0x0,
                        ///  2**30 clock cycles.
                        wdt2pow30 = 0x1,
                        ///  2**29 clock cycles.
                        wdt2pow29 = 0x2,
                        ///  2**28 clock cycles.
                        wdt2pow28 = 0x3,
                        ///  2^27 clock cycles.
                        wdt2pow27 = 0x4,
                        ///  2**26 clock cycles.
                        wdt2pow26 = 0x5,
                        ///  2**25 clock cycles.
                        wdt2pow25 = 0x6,
                        ///  2**24 clock cycles.
                        wdt2pow24 = 0x7,
                        ///  2**23 clock cycles.
                        wdt2pow23 = 0x8,
                        ///  2**22 clock cycles.
                        wdt2pow22 = 0x9,
                        ///  2**21 clock cycles.
                        wdt2pow21 = 0xa,
                        ///  2**20 clock cycles.
                        wdt2pow20 = 0xb,
                        ///  2**19 clock cycles.
                        wdt2pow19 = 0xc,
                        ///  2**18 clock cycles.
                        wdt2pow18 = 0xd,
                        ///  2**17 clock cycles.
                        wdt2pow17 = 0xe,
                        ///  2**16 clock cycles.
                        wdt2pow16 = 0xf,
                    },
                },
                ///  Windowed Watchdog Reset Lower Limit. Sets the number of WDTCLK cycles that establishes the lower boundary of the watchdog window. A system reset occurs (if enabled) if the CPU writes the windowed watchdog reset sequence to the WWDT_RST register before the watchdog timer has counted this time period since the last timer reset.
                RST_EARLY_VAL: packed union {
                    raw: u4,
                    value: enum(u4) {
                        ///  2**31 clock cycles.
                        wdt2pow31 = 0x0,
                        ///  2**30 clock cycles.
                        wdt2pow30 = 0x1,
                        ///  2**29 clock cycles.
                        wdt2pow29 = 0x2,
                        ///  2**28 clock cycles.
                        wdt2pow28 = 0x3,
                        ///  2^27 clock cycles.
                        wdt2pow27 = 0x4,
                        ///  2**26 clock cycles.
                        wdt2pow26 = 0x5,
                        ///  2**25 clock cycles.
                        wdt2pow25 = 0x6,
                        ///  2**24 clock cycles.
                        wdt2pow24 = 0x7,
                        ///  2**23 clock cycles.
                        wdt2pow23 = 0x8,
                        ///  2**22 clock cycles.
                        wdt2pow22 = 0x9,
                        ///  2**21 clock cycles.
                        wdt2pow21 = 0xa,
                        ///  2**20 clock cycles.
                        wdt2pow20 = 0xb,
                        ///  2**19 clock cycles.
                        wdt2pow19 = 0xc,
                        ///  2**18 clock cycles.
                        wdt2pow18 = 0xd,
                        ///  2**17 clock cycles.
                        wdt2pow17 = 0xe,
                        ///  2**16 clock cycles.
                        wdt2pow16 = 0xf,
                    },
                },
                reserved27: u3,
                ///  Switch Ready Interrupt Enable. Fires an interrupt when it is safe to swithc the clock.
                CLKRDY_IE: u1,
                ///  Clock Status.
                CLKRDY: u1,
                ///  Enables the Windowed Watchdog Function.
                WIN_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Windowed Mode Disabled (i.e. Compatibility Mode).
                        dis = 0x0,
                        ///  Windowed Mode Enabled.
                        en = 0x1,
                    },
                },
                ///  Windowed Watchdog Timer Reset Flag Too Soon.
                RST_EARLY: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  The event has not occurred.
                        noEvent = 0x0,
                        ///  The event has occurred.
                        occurred = 0x1,
                    },
                },
                ///  Windowed Watchdog Timer Reset Flag Too Late.
                RST_LATE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  The event has not occurred.
                        noEvent = 0x0,
                        ///  The event has occurred.
                        occurred = 0x1,
                    },
                },
            }),
            ///  Windowed Watchdog Timer Reset Register.
            RST: mmio.Mmio(packed struct(u32) {
                ///  Writing the watchdog counter 'reset sequence' to this register resets the watchdog counter. If the watchdog count exceeds INT_PERIOD_UPPER_LIMIT then a watchdog interrupt will occur, if enabled. If the watchdog count exceeds RST_PERIOD_UPPER_LIMIT then a watchdog reset will occur, if enabled.
                RESET: packed union {
                    raw: u8,
                    value: enum(u8) {
                        ///  The first value to be written to reset the WDT.
                        seq0 = 0xa5,
                        ///  The second value to be written to reset the WDT.
                        seq1 = 0x5a,
                        _,
                    },
                },
                padding: u24,
            }),
            ///  Windowed Watchdog Timer Clock Select Register.
            CLKSEL: mmio.Mmio(packed struct(u32) {
                ///  WWDT Clock Selection Register.
                SOURCE: u3,
                padding: u29,
            }),
            ///  Windowed Watchdog Timer Count Register.
            CNT: mmio.Mmio(packed struct(u32) {
                ///  Current Value of the Windowed Watchdog Timer Counter.
                COUNT: u32,
            }),
        };

        ///  Random Number Generator.
        pub const TRNG = extern struct {
            ///  TRNG Control Register.
            CTRL: mmio.Mmio(packed struct(u32) {
                reserved1: u1,
                ///  To enable IRQ generation when a new 32-bit Random number is ready.
                RND_IE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable
                        disable = 0x0,
                        ///  Enable
                        enable = 0x1,
                    },
                },
                reserved3: u1,
                ///  AES Key Generate. When enabled, the key for securing NVSRAM is generated and transferred to the secure key register automatically without user visibility or intervention. This bit is cleared by hardware once the key has been transferred to the secure key register.
                KEYGEN: u1,
                reserved15: u11,
                ///  To wipe the Battery Backed key.
                KEYWIPE: u1,
                padding: u16,
            }),
            ///  Data. The content of this register is valid only when RNG_IS = 1. When TRNG is disabled, read returns 0x0000 0000.
            STATUS: mmio.Mmio(packed struct(u32) {
                ///  32-bit random data is ready to read from TRNG_DATA register. Reading TRNG_DATA when RND_RDY=0 will return all 0's. IRQ is generated when RND_RDY=1 if TRNG_CN.RND_IRQ_EN=1.
                RDY: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  TRNG Busy
                        Busy = 0x0,
                        ///  32 bit random data is ready
                        Ready = 0x1,
                    },
                },
                padding: u31,
            }),
            ///  Data. The content of this register is valid only when RNG_IS = 1. When TRNG is disabled, read returns 0x0000 0000.
            DATA: mmio.Mmio(packed struct(u32) {
                ///  Data. The content of this register is valid only when RNG_IS =1. When TNRG is disabled, read returns 0x0000 0000.
                DATA: u32,
            }),
        };

        ///  Inter-IC Sound Interface.
        pub const I2S = extern struct {
            ///  Global mode channel.
            CTRL0CH0: mmio.Mmio(packed struct(u32) {
                reserved1: u1,
                ///  LSB Transmit Receive First.
                LSB_FIRST: u1,
                ///  PDM Filter.
                PDM_FILT: u1,
                ///  PDM Enable.
                PDM_EN: u1,
                ///  DDR.
                USEDDR: u1,
                ///  Invert PDM.
                PDM_INV: u1,
                ///  SCK Select.
                CH_MODE: u2,
                ///  WS polarity select.
                WS_POL: u1,
                ///  MSB location.
                MSB_LOC: u1,
                ///  Align to MSB or LSB.
                ALIGN: u1,
                ///  External SCK/WS selection.
                EXT_SEL: u1,
                ///  Stereo mode of I2S.
                STEREO: u2,
                ///  Data size when write to FIFO.
                WSIZE: u2,
                ///  TX channel enable.
                TX_EN: u1,
                ///  RX channel enable.
                RX_EN: u1,
                ///  Flushes the TX/RX FIFO buffer.
                FLUSH: u1,
                ///  Write 1 to reset channel.
                RST: u1,
                ///  Bit Field Control.
                FIFO_LSB: u1,
                reserved24: u3,
                ///  depth of receive FIFO for threshold interrupt generation.
                RX_THD_VAL: u8,
            }),
            reserved16: [12]u8,
            ///  Local channel Setup.
            CTRL1CH0: mmio.Mmio(packed struct(u32) {
                ///  I2S word length.
                BITS_WORD: u5,
                reserved8: u3,
                ///  I2S clock enable.
                EN: u1,
                ///  I2S sample size length.
                SMP_SIZE: u5,
                reserved15: u1,
                ///  LSB/MSB Justify.
                ADJUST: u1,
                ///  I2S clock frequency divisor.
                CLKDIV: u16,
            }),
            reserved32: [12]u8,
            ///  Filter.
            FILTCH0: u32,
            reserved48: [12]u8,
            ///  DMA Control.
            DMACH0: mmio.Mmio(packed struct(u32) {
                ///  TX FIFO Level DMA Trigger.
                DMA_TX_THD_VAL: u7,
                ///  TX DMA channel enable.
                DMA_TX_EN: u1,
                ///  RX FIFO Level DMA Trigger.
                DMA_RX_THD_VAL: u7,
                ///  RX DMA channel enable.
                DMA_RX_EN: u1,
                ///  Number of data word in the TX FIFO.
                TX_LVL: u8,
                ///  Number of data word in the RX FIFO.
                RX_LVL: u8,
            }),
            reserved64: [12]u8,
            ///  I2S Fifo.
            FIFOCH0: mmio.Mmio(packed struct(u32) {
                ///  Load/unload location for TX and RX FIFO buffers.
                DATA: u32,
            }),
            reserved80: [12]u8,
            ///  ISR Status.
            INTFL: mmio.Mmio(packed struct(u32) {
                ///  Status for RX FIFO Overrun interrupt.
                RX_OV_CH0: u1,
                ///  Status for interrupt when RX FIFO reaches the number of bytes configured by the RXTHD field.
                RX_THD_CH0: u1,
                ///  Status for interrupt when TX FIFO has only one byte remaining.
                TX_OB_CH0: u1,
                ///  Status for interrupt when TX FIFO is half empty.
                TX_HE_CH0: u1,
                padding: u28,
            }),
            ///  Interrupt Enable.
            INTEN: mmio.Mmio(packed struct(u32) {
                ///  Enable for RX FIFO Overrun interrupt.
                RX_OV_CH0: u1,
                ///  Enable for interrupt when RX FIFO reaches the number of bytes configured by the RXTHD field.
                RX_THD_CH0: u1,
                ///  Enable for interrupt when TX FIFO has only one byte remaining.
                TX_OB_CH0: u1,
                ///  Enable for interrupt when TX FIFO is half empty.
                TX_HE_CH0: u1,
                padding: u28,
            }),
            ///  Ext Control.
            EXTSETUP: mmio.Mmio(packed struct(u32) {
                ///  Word Length for ch_mode.
                EXT_BITS_WORD: u5,
                padding: u27,
            }),
            ///  Wakeup Enable.
            WKEN: u32,
            ///  Wakeup Flags.
            WKFL: u32,
        };

        ///  Instruction Cache Controller Registers
        pub const ICC0 = extern struct {
            ///  Cache ID Register.
            INFO: mmio.Mmio(packed struct(u32) {
                ///  Release Number. Identifies the RTL release version.
                RELNUM: u6,
                ///  Part Number. This field reflects the value of C_ID_PART_NUMBER configuration parameter.
                PARTNUM: u4,
                ///  Cache ID. This field reflects the value of the C_ID_CACHEID configuration parameter.
                ID: u6,
                padding: u16,
            }),
            ///  Memory Configuration Register.
            SZ: mmio.Mmio(packed struct(u32) {
                ///  Cache Size. Indicates total size in Kbytes of cache.
                CCH: u16,
                ///  Main Memory Size. Indicates the total size, in units of 128 Kbytes, of code memory accessible to the cache controller.
                MEM: u16,
            }),
            reserved256: [248]u8,
            ///  Cache Control and Status Register.
            CTRL: mmio.Mmio(packed struct(u32) {
                ///  Cache Enable. Controls whether the cache is bypassed or is in use. Changing the state of this bit will cause the instruction cache to be flushed and its contents invalidated.
                EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Cache Bypassed. Instruction data is stored in the line fill buffer but is not written to main cache memory array.
                        dis = 0x0,
                        ///  Cache Enabled.
                        en = 0x1,
                    },
                },
                reserved16: u15,
                ///  Cache Ready flag. Cleared by hardware when at any time the cache as a whole is invalidated (including a system reset). When this bit is 0, the cache is effectively in bypass mode (instruction fetches will come from main memory or from the line fill buffer). Set by hardware when the invalidate operation is complete and the cache is ready.
                RDY: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Not Ready.
                        notReady = 0x0,
                        ///  Ready.
                        ready = 0x1,
                    },
                },
                padding: u15,
            }),
            reserved1792: [1532]u8,
            ///  Invalidate All Registers.
            INVALIDATE: mmio.Mmio(packed struct(u32) {
                ///  Invalidate.
                INVALID: u32,
            }),
        };

        ///  Low Power Comparator
        pub const LPCMP = extern struct {
            ///  Comparator Control Register.
            CTRL: [3]mmio.Mmio(packed struct(u32) {
                ///  Comparator Enable.
                EN: u1,
                reserved5: u4,
                ///  Polarity Select
                POL: u1,
                ///  IRQ Enable.
                INT_EN: u1,
                reserved14: u7,
                ///  Raw Compartor Input.
                OUT: u1,
                ///  IRQ Flag
                INT_FL: u1,
                padding: u16,
            }),
        };

        ///  Low Power Global Control.
        pub const LPGCR = extern struct {
            reserved8: [8]u8,
            ///  Low Power Reset Register.
            RST: mmio.Mmio(packed struct(u32) {
                ///  Low Power GPIO 2 Reset.
                GPIO2: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Reset complete.
                        reset_done = 0x0,
                        ///  Starts Reset or indicates reset in progress.
                        busy = 0x1,
                    },
                },
                ///  Low Power Watchdog Timer 1 Reset.
                WDT1: u1,
                ///  Low Power Timer 4 Reset.
                TMR4: u1,
                ///  Low Power Timer 5 Reset.
                TMR5: u1,
                ///  Low Power UART 3 Reset.
                UART3: u1,
                reserved6: u1,
                ///  Low Power Comparator Reset.
                LPCOMP: u1,
                padding: u25,
            }),
            ///  Low Power Peripheral Clock Disable Register.
            PCLKDIS: mmio.Mmio(packed struct(u32) {
                ///  Low Power GPIO 2 Clock Disable.
                GPIO2: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  enable it.
                        en = 0x0,
                        ///  disable it.
                        dis = 0x1,
                    },
                },
                ///  Low Power Watchdog 1 Clock Disable.
                WDT1: u1,
                ///  Low Power Timer 4 Clock Disable.
                TMR4: u1,
                ///  Low Power Timer 5 Clock Disable.
                TMR5: u1,
                ///  Low Power UART 3 Clock Disable.
                UART3: u1,
                reserved6: u1,
                ///  Low Power Comparator Clock Disable.
                LPCOMP: u1,
                padding: u25,
            }),
        };

        ///  Misc Control.
        pub const MCR = extern struct {
            ///  ECC Enable Register
            ECCEN: mmio.Mmio(packed struct(u32) {
                ///  ECC System RAM0 Enable.
                RAM0: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  disabled.
                        dis = 0x0,
                        ///  enabled.
                        en = 0x1,
                    },
                },
                padding: u31,
            }),
            ///  IPO Manual Register
            IPO_MTRIM: mmio.Mmio(packed struct(u32) {
                ///  Manual Trim Value.
                MTRIM: u8,
                ///  Trim Range Select.
                TRIM_RANGE: u1,
                padding: u23,
            }),
            ///  Output Enable Register
            OUTEN: mmio.Mmio(packed struct(u32) {
                ///  Square Wave Output Enable.
                SQWOUT_EN: u1,
                ///  Power Down Output Enable.
                PDOWN_OUT_EN: u1,
                padding: u30,
            }),
            ///  Comparator Control Register.
            CMP_CTRL: mmio.Mmio(packed struct(u32) {
                ///  Comparator Enable.
                EN: u1,
                reserved5: u4,
                ///  Polarity Select
                POL: u1,
                ///  IRQ Enable.
                INT_EN: u1,
                reserved14: u7,
                ///  Comparator Output State.
                OUT: u1,
                ///  IRQ Flag
                INT_FL: u1,
                padding: u16,
            }),
            ///  Miscellaneous Control Register.
            CTRL: mmio.Mmio(packed struct(u32) {
                reserved2: u2,
                ///  INRO Enable.
                INRO_EN: u1,
                ///  ERTCO Enable.
                ERTCO_EN: u1,
                reserved8: u4,
                ///  SIMO Clock Scaling Enable.
                SIMO_CLKSCL_EN: u1,
                ///  SIMO System Reset Disable.
                SIMO_RSTD: u1,
                padding: u22,
            }),
            reserved32: [12]u8,
            ///  GPIO3 Pin Control Register.
            GPIO3_CTRL: mmio.Mmio(packed struct(u32) {
                ///  GPIO3 Pin 0 Data Output.
                P30_DO: u1,
                ///  GPIO3 Pin 0 Output Enable.
                P30_OE: u1,
                ///  GPIO3 Pin 0 Pull-up Enable.
                P30_PE: u1,
                ///  GPIO3 Pin 0 Input Status.
                P30_IN: u1,
                ///  GPIO3 Pin 1 Data Output.
                P31_DO: u1,
                ///  GPIO3 Pin 1 Output Enable.
                P31_OE: u1,
                ///  GPIO3 Pin 1 Pull-up Enable.
                P31_PE: u1,
                ///  GPIO3 Pin 1 Input Status.
                P31_IN: u1,
                padding: u24,
            }),
        };

        ///  1-Wire Master Interface.
        pub const OWM = extern struct {
            ///  1-Wire Master Configuration.
            CFG: mmio.Mmio(packed struct(u32) {
                ///  Long Line Mode.
                long_line_mode: u1,
                ///  Force Line During Presence Detect.
                force_pres_det: u1,
                ///  Bit Bang Enable.
                bit_bang_en: u1,
                ///  Provide an extra output control to control an external pullup.
                ext_pullup_mode: u1,
                ///  Enable External Pullup.
                ext_pullup_enable: u1,
                ///  Enable Single Bit TX/RX Mode.
                single_bit_mode: u1,
                ///  Enables overdrive speed for 1-Wire operations.
                overdrive: u1,
                ///  Enable intenral pullup.
                int_pullup_enable: u1,
                padding: u24,
            }),
            ///  1-Wire Master Clock Divisor.
            CLK_DIV_1US: mmio.Mmio(packed struct(u32) {
                ///  Clock Divisor for 1Mhz.
                divisor: u8,
                padding: u24,
            }),
            ///  1-Wire Master Control/Status.
            CTRL_STAT: mmio.Mmio(packed struct(u32) {
                ///  Start OW Reset.
                start_ow_reset: u1,
                ///  SRA Mode.
                sra_mode: u1,
                ///  Bit Bang Output Enable.
                bit_bang_oe: u1,
                ///  OW Input State.
                ow_input: u1,
                ///  Overdrive Spec Mode.
                od_spec_mode: u1,
                reserved7: u2,
                ///  Presence Pulse Detected.
                presence_detect: u1,
                padding: u24,
            }),
            ///  1-Wire Master Data Buffer.
            DATA: mmio.Mmio(packed struct(u32) {
                ///  TX/RX Buffer.
                tx_rx: u8,
                padding: u24,
            }),
            ///  1-Wire Master Interrupt Flags.
            INTFL: mmio.Mmio(packed struct(u32) {
                ///  OW Reset Sequence Completed.
                ow_reset_done: u1,
                ///  TX Data Empty Interrupt Flag.
                tx_data_empty: u1,
                ///  RX Data Ready Interrupt Flag
                rx_data_ready: u1,
                ///  OW Line Short Detected Interrupt Flag.
                line_short: u1,
                ///  OW Line Low Detected Interrupt Flag.
                line_low: u1,
                padding: u27,
            }),
            ///  1-Wire Master Interrupt Enables.
            INTEN: mmio.Mmio(packed struct(u32) {
                ///  OW Reset Sequence Completed.
                ow_reset_done: u1,
                ///  Tx Data Empty Interrupt Enable.
                tx_data_empty: u1,
                ///  Rx Data Ready Interrupt Enable.
                rx_data_ready: u1,
                ///  OW Line Short Detected Interrupt Enable.
                line_short: u1,
                ///  OW Line Low Detected Interrupt Enable.
                line_low: u1,
                padding: u27,
            }),
        };

        ///  Pulse Train
        pub const PT = extern struct {
            ///  Pulse Train Configuration
            RATE_LENGTH: mmio.Mmio(packed struct(u32) {
                ///  Pulse Train Enable and Rate Control. Set to 0 to disable the Pulse Train.
                rate_control: u27,
                ///  Pulse Train Output Mode/Train Length
                mode: packed union {
                    raw: u5,
                    value: enum(u5) {
                        ///  Pulse train, 32 bit pattern.
                        @"32_BIT" = 0x0,
                        ///  Square wave mode.
                        SQUARE_WAVE = 0x1,
                        ///  Pulse train, 2 bit pattern.
                        @"2_BIT" = 0x2,
                        ///  Pulse train, 3 bit pattern.
                        @"3_BIT" = 0x3,
                        ///  Pulse train, 4 bit pattern.
                        @"4_BIT" = 0x4,
                        ///  Pulse train, 5 bit pattern.
                        @"5_BIT" = 0x5,
                        ///  Pulse train, 6 bit pattern.
                        @"6_BIT" = 0x6,
                        ///  Pulse train, 7 bit pattern.
                        @"7_BIT" = 0x7,
                        ///  Pulse train, 8 bit pattern.
                        @"8_BIT" = 0x8,
                        ///  Pulse train, 9 bit pattern.
                        @"9_BIT" = 0x9,
                        ///  Pulse train, 10 bit pattern.
                        @"10_BIT" = 0xa,
                        ///  Pulse train, 11 bit pattern.
                        @"11_BIT" = 0xb,
                        ///  Pulse train, 12 bit pattern.
                        @"12_BIT" = 0xc,
                        ///  Pulse train, 13 bit pattern.
                        @"13_BIT" = 0xd,
                        ///  Pulse train, 14 bit pattern.
                        @"14_BIT" = 0xe,
                        ///  Pulse train, 15 bit pattern.
                        @"15_BIT" = 0xf,
                        ///  Pulse train, 16 bit pattern.
                        @"16_BIT" = 0x10,
                        ///  Pulse train, 17 bit pattern.
                        @"17_BIT" = 0x11,
                        ///  Pulse train, 18 bit pattern.
                        @"18_BIT" = 0x12,
                        ///  Pulse train, 19 bit pattern.
                        @"19_BIT" = 0x13,
                        ///  Pulse train, 20 bit pattern.
                        @"20_BIT" = 0x14,
                        ///  Pulse train, 21 bit pattern.
                        @"21_BIT" = 0x15,
                        ///  Pulse train, 22 bit pattern.
                        @"22_BIT" = 0x16,
                        ///  Pulse train, 23 bit pattern.
                        @"23_BIT" = 0x17,
                        ///  Pulse train, 24 bit pattern.
                        @"24_BIT" = 0x18,
                        ///  Pulse train, 25 bit pattern.
                        @"25_BIT" = 0x19,
                        ///  Pulse train, 26 bit pattern.
                        @"26_BIT" = 0x1a,
                        ///  Pulse train, 27 bit pattern.
                        @"27_BIT" = 0x1b,
                        ///  Pulse train, 28 bit pattern.
                        @"28_BIT" = 0x1c,
                        ///  Pulse train, 29 bit pattern.
                        @"29_BIT" = 0x1d,
                        ///  Pulse train, 30 bit pattern.
                        @"30_BIT" = 0x1e,
                        ///  Pulse train, 31 bit pattern.
                        @"31_BIT" = 0x1f,
                    },
                },
            }),
            ///  Write the repeating bit pattern that is shifted out, LSB first, when configured in Pulse Train mode. See PT_RATE_LENGTH.mode for setting the length.
            TRAIN: u32,
            ///  Pulse Train Loop Count
            LOOP: mmio.Mmio(packed struct(u32) {
                ///  Number of loops for this pulse train to repeat.
                count: u16,
                ///  Delay between loops of the Pulse Train in PT Peripheral Clock cycles
                delay: u12,
                padding: u4,
            }),
            ///  Pulse Train Auto-Restart Configuration.
            RESTART: mmio.Mmio(packed struct(u32) {
                ///  Auto-Restart PT X Select
                pt_x_select: u5,
                reserved7: u2,
                ///  Enable Auto-Restart on PT X Loop Exit
                on_pt_x_loop_exit: u1,
                ///  Auto-Restart PT Y Select
                pt_y_select: u5,
                reserved15: u2,
                ///  Enable Auto-Restart on PT Y Loop Exit
                on_pt_y_loop_exit: u1,
                padding: u16,
            }),
        };

        ///  Low-Power Configurable Timer
        pub const TMR = extern struct {
            ///  Timer Counter Register.
            CNT: mmio.Mmio(packed struct(u32) {
                ///  The current count value for the timer. This field increments as the timer counts.
                COUNT: u32,
            }),
            ///  Timer Compare Register.
            CMP: mmio.Mmio(packed struct(u32) {
                ///  The value in this register is used as the compare value for the timer's count value. The compare field meaning is determined by the specific mode of the timer.
                COMPARE: u32,
            }),
            ///  Timer PWM Register.
            PWM: mmio.Mmio(packed struct(u32) {
                ///  Timer PWM Match: In PWM Mode, this field sets the count value for the first transition period of the PWM cycle. At the end of the cycle where CNT equals PWM, the PWM output transitions to the second period of the PWM cycle. The second PWM period count is stored in the CMP register. The value set for PWM must me less than the value set in CMP for PWM mode operation. Timer Capture Value: In Capture, Compare, and Capture/Compare modes, this field is used to store the CNT value when a Capture, Compare, or Capture/Compare event occurs.
                PWM: u32,
            }),
            ///  Timer Interrupt Status Register.
            INTFL: mmio.Mmio(packed struct(u32) {
                ///  Interrupt Flag for Timer A.
                IRQ_A: u1,
                reserved8: u7,
                ///  Write Done Flag for Timer A indicating the write is complete from APB to CLK_TMR domain.
                WRDONE_A: u1,
                ///  Write Disable to CNT/PWM for Timer A in the non-cascaded dual timer configuration.
                WR_DIS_A: u1,
                reserved16: u6,
                ///  Interrupt Flag for Timer B.
                IRQ_B: u1,
                reserved24: u7,
                ///  Write Done Flag for Timer B indicating the write is complete from APB to CLK_TMR domain.
                WRDONE_B: u1,
                ///  Write Disable to CNT/PWM for Timer B in the non-cascaded dual timer configuration.
                WR_DIS_B: u1,
                padding: u6,
            }),
            ///  Timer Control Register.
            CTRL0: mmio.Mmio(packed struct(u32) {
                ///  Mode Select for Timer A
                MODE_A: packed union {
                    raw: u4,
                    value: enum(u4) {
                        ///  One-Shot Mode
                        ONE_SHOT = 0x0,
                        ///  Continuous Mode
                        CONTINUOUS = 0x1,
                        ///  Counter Mode
                        COUNTER = 0x2,
                        ///  PWM Mode
                        PWM = 0x3,
                        ///  Capture Mode
                        CAPTURE = 0x4,
                        ///  Compare Mode
                        COMPARE = 0x5,
                        ///  Gated Mode
                        GATED = 0x6,
                        ///  Capture/Compare Mode
                        CAPCOMP = 0x7,
                        ///  Dual Edge Capture Mode
                        DUAL_EDGE = 0x8,
                        ///  Inactive Gated Mode
                        IGATED = 0xe,
                        _,
                    },
                },
                ///  Clock Divider Select for Timer A
                CLKDIV_A: packed union {
                    raw: u4,
                    value: enum(u4) {
                        ///  Prescaler Divide-By-1
                        DIV_BY_1 = 0x0,
                        ///  Prescaler Divide-By-2
                        DIV_BY_2 = 0x1,
                        ///  Prescaler Divide-By-4
                        DIV_BY_4 = 0x2,
                        ///  Prescaler Divide-By-8
                        DIV_BY_8 = 0x3,
                        ///  Prescaler Divide-By-16
                        DIV_BY_16 = 0x4,
                        ///  Prescaler Divide-By-32
                        DIV_BY_32 = 0x5,
                        ///  Prescaler Divide-By-64
                        DIV_BY_64 = 0x6,
                        ///  Prescaler Divide-By-128
                        DIV_BY_128 = 0x7,
                        ///  Prescaler Divide-By-256
                        DIV_BY_256 = 0x8,
                        ///  Prescaler Divide-By-512
                        DIV_BY_512 = 0x9,
                        ///  Prescaler Divide-By-1024
                        DIV_BY_1024 = 0xa,
                        ///  Prescaler Divide-By-2048
                        DIV_BY_2048 = 0xb,
                        ///  TBD
                        DIV_BY_4096 = 0xc,
                        _,
                    },
                },
                ///  Timer Polarity for Timer A
                POL_A: u1,
                ///  PWM Synchronization Mode for Timer A
                PWMSYNC_A: u1,
                ///  PWM Phase A (Non-Overlapping High) Polarity for Timer A
                NOLHPOL_A: u1,
                ///  PWM Phase A-Prime (Non-Overlapping Low) Polarity for Timer A
                NOLLPOL_A: u1,
                ///  PWM Phase A-Prime Output Disable for Timer A
                PWMCKBD_A: u1,
                ///  Resets all flip flops in the CLK_TMR domain for Timer A. Self-clears.
                RST_A: u1,
                ///  Write 1 to Enable CLK_TMR for Timer A
                CLKEN_A: u1,
                ///  Enable for Timer A
                EN_A: u1,
                ///  Mode Select for Timer B
                MODE_B: packed union {
                    raw: u4,
                    value: enum(u4) {
                        ///  One-Shot Mode
                        ONE_SHOT = 0x0,
                        ///  Continuous Mode
                        CONTINUOUS = 0x1,
                        ///  Counter Mode
                        COUNTER = 0x2,
                        ///  PWM Mode
                        PWM = 0x3,
                        ///  Capture Mode
                        CAPTURE = 0x4,
                        ///  Compare Mode
                        COMPARE = 0x5,
                        ///  Gated Mode
                        GATED = 0x6,
                        ///  Capture/Compare Mode
                        CAPCOMP = 0x7,
                        ///  Dual Edge Capture Mode
                        DUAL_EDGE = 0x8,
                        ///  Inactive Gated Mode
                        IGATED = 0xe,
                        _,
                    },
                },
                ///  Clock Divider Select for Timer B
                CLKDIV_B: packed union {
                    raw: u4,
                    value: enum(u4) {
                        ///  Prescaler Divide-By-1
                        DIV_BY_1 = 0x0,
                        ///  Prescaler Divide-By-2
                        DIV_BY_2 = 0x1,
                        ///  Prescaler Divide-By-4
                        DIV_BY_4 = 0x2,
                        ///  Prescaler Divide-By-8
                        DIV_BY_8 = 0x3,
                        ///  Prescaler Divide-By-16
                        DIV_BY_16 = 0x4,
                        ///  Prescaler Divide-By-32
                        DIV_BY_32 = 0x5,
                        ///  Prescaler Divide-By-64
                        DIV_BY_64 = 0x6,
                        ///  Prescaler Divide-By-128
                        DIV_BY_128 = 0x7,
                        ///  Prescaler Divide-By-256
                        DIV_BY_256 = 0x8,
                        ///  Prescaler Divide-By-512
                        DIV_BY_512 = 0x9,
                        ///  Prescaler Divide-By-1024
                        DIV_BY_1024 = 0xa,
                        ///  Prescaler Divide-By-2048
                        DIV_BY_2048 = 0xb,
                        ///  TBD
                        DIV_BY_4096 = 0xc,
                        _,
                    },
                },
                ///  Timer Polarity for Timer B
                POL_B: u1,
                ///  PWM Synchronization Mode for Timer B
                PWMSYNC_B: u1,
                ///  PWM Phase A (Non-Overlapping High) Polarity for Timer B
                NOLHPOL_B: u1,
                ///  PWM Phase A-Prime (Non-Overlapping Low) Polarity for Timer B
                NOLLPOL_B: u1,
                ///  PWM Phase A-Prime Output Disable for Timer B
                PWMCKBD_B: u1,
                ///  Resets all flip flops in the CLK_TMR domain for Timer B. Self-clears.
                RST_B: u1,
                ///  Write 1 to Enable CLK_TMR for Timer B
                CLKEN_B: u1,
                ///  Enable for Timer B
                EN_B: u1,
            }),
            ///  Timer Non-Overlapping Compare Register.
            NOLCMP: mmio.Mmio(packed struct(u32) {
                ///  Non-Overlapping Low Compare value for Timer A controls the time between the falling edge of PWM Phase A and the next rising edge of PWM Phase A-Prime.
                LO_A: u8,
                ///  Non-Overlapping High Compare value for Timer A controls the time between the falling edge of PWM Phase A-Prime and the next rising edge of PWM Phase A.
                HI_A: u8,
                ///  Non-Overlapping Low Compare value for Timer B controls the time between the falling edge of PWM Phase A and the next rising edge of PWM Phase A-Prime.
                LO_B: u8,
                ///  Non-Overlapping High Compare value for Timer B controls the time between the falling edge of PWM Phase A-Prime and the next rising edge of PWM Phase A.
                HI_B: u8,
            }),
            ///  Timer Configuration Register.
            CTRL1: mmio.Mmio(packed struct(u32) {
                ///  Timer Clock Select for Timer A
                CLKSEL_A: u2,
                ///  Timer A Enable Status
                CLKEN_A: u1,
                ///  CLK_TMR Ready Flag for Timer A
                CLKRDY_A: u1,
                ///  Event Select for Timer A
                EVENT_SEL_A: u3,
                ///  Negative Edge Trigger for Event for Timer A
                NEGTRIG_A: u1,
                ///  Interrupt Enable for Timer A
                IE_A: u1,
                ///  Capture Event Select for Timer A
                CAPEVENT_SEL_A: u2,
                ///  Software Capture Event for Timer A
                SW_CAPEVENT_A: u1,
                ///  Wake-Up Enable for Timer A
                WE_A: u1,
                ///  OUT_OE_O Enable for Modes 0, 1,and 5 for Timer A
                OUTEN_A: u1,
                ///  PWM_CKB_EN_O Enable for Modes other than Mode 3 for Timer A
                OUTBEN_A: u1,
                reserved16: u1,
                ///  Timer Clock Select for Timer B
                CLKSEL_B: u2,
                ///  Timer B Enable Status
                CLKEN_B: u1,
                ///  CLK_TMR Ready Flag for Timer B
                CLKRDY_B: u1,
                ///  Event Select for Timer B
                EVENT_SEL_B: u3,
                ///  Negative Edge Trigger for Event for Timer B
                NEGTRIG_B: u1,
                ///  Interrupt Enable for Timer B
                IE_B: u1,
                ///  Capture Event Select for Timer B
                CAPEVENT_SEL_B: u2,
                ///  Software Capture Event for Timer B
                SW_CAPEVENT_B: u1,
                ///  Wake-Up Enable for Timer B
                WE_B: u1,
                reserved31: u2,
                ///  Cascade two 16-bit timers into one 32-bit timer. Only available when C_TMR16=0 adn C_DUALTMR16=1.
                CASCADE: u1,
            }),
            ///  Timer Wakeup Status Register.
            WKFL: mmio.Mmio(packed struct(u32) {
                ///  Wake-Up Flag for Timer A
                A: u1,
                reserved16: u15,
                ///  Wake-Up Flag for Timer B
                B: u1,
                padding: u15,
            }),
        };

        ///  Trim System Initilazation Registers
        pub const TRIMSIR = extern struct {
            reserved8: [8]u8,
            ///  RTC Trim System Initialization Register.
            RTC: mmio.Mmio(packed struct(u32) {
                reserved16: u16,
                ///  RTC X1 Trim.
                X1TRIM: u5,
                ///  RTC X2 Trim.
                X2TRIM: u5,
                reserved31: u5,
                ///  Lock.
                LOCK: u1,
            }),
            reserved52: [40]u8,
            ///  SIMO Trim System Initialization Register.
            SIMO: mmio.Mmio(packed struct(u32) {
                ///  SIMO Clock Divide.
                CLKDIV: packed union {
                    raw: u3,
                    value: enum(u3) {
                        DIV1 = 0x0,
                        DIV16 = 0x1,
                        DIV32 = 0x3,
                        DIV64 = 0x5,
                        DIV128 = 0x7,
                        _,
                    },
                },
                padding: u29,
            }),
            reserved60: [4]u8,
            ///  IPO Low Trim System Initialization Register.
            IPOLO: mmio.Mmio(packed struct(u32) {
                ///  IPO Low Limit Trim.
                IPO_LIMITLO: u8,
                padding: u24,
            }),
            ///  Control Trim System Initialization Register.
            CTRL: mmio.Mmio(packed struct(u32) {
                ///  VDDA Low Trim Limit.
                VDDA_LIMITLO: u7,
                reserved8: u1,
                ///  VDDA High Trim Limit.
                VDDA_LIMITHI: u7,
                ///  IPO High Trim Limit.
                IPO_LIMITHI: u9,
                ///  INRO Clock Select.
                INRO_SEL: packed union {
                    raw: u2,
                    value: enum(u2) {
                        @"8KHZ" = 0x0,
                        @"16KHZ" = 0x1,
                        @"30KHZ" = 0x2,
                        _,
                    },
                },
                reserved29: u3,
                ///  INRO Clock Trim.
                INRO_TRIM: u3,
            }),
            ///  RTC Trim System Initialization Register.
            INRO: mmio.Mmio(packed struct(u32) {
                ///  INRO 16KHz Trim.
                TRIM16K: u3,
                ///  INRO 30KHz Trim.
                TRIM30K: u3,
                ///  INRO Low Power Mode Clock Select.
                LPCLKSEL: packed union {
                    raw: u2,
                    value: enum(u2) {
                        @"8KHZ" = 0x0,
                        @"16KHZ" = 0x1,
                        @"30KHZ" = 0x2,
                        _,
                    },
                },
                padding: u24,
            }),
        };

        ///  UART Low Power Registers
        pub const UART = extern struct {
            ///  Control register
            CTRL: mmio.Mmio(packed struct(u32) {
                ///  This field specifies the depth of receive FIFO for interrupt generation (value 0 and > 16 are ignored)
                RX_THD_VAL: u4,
                ///  Parity Enable
                PAR_EN: u1,
                ///  when PAREN=1 selects odd or even parity odd is 1 even is 0
                PAR_EO: u1,
                ///  Selects parity based on 1s or 0s count (when PAREN=1)
                PAR_MD: u1,
                ///  CTS Sampling Disable
                CTS_DIS: u1,
                ///  Flushes the TX FIFO buffer. This bit is automatically cleared by hardware when flush is completed.
                TX_FLUSH: u1,
                ///  Flushes the RX FIFO buffer. This bit is automatically cleared by hardware when flush is completed.
                RX_FLUSH: u1,
                ///  Selects UART character size
                CHAR_SIZE: packed union {
                    raw: u2,
                    value: enum(u2) {
                        ///  5 bits
                        @"5bits" = 0x0,
                        ///  6 bits
                        @"6bits" = 0x1,
                        ///  7 bits
                        @"7bits" = 0x2,
                        ///  8 bits
                        @"8bits" = 0x3,
                    },
                },
                ///  Selects the number of stop bits that will be generated
                STOPBITS: u1,
                ///  Enables/disables hardware flow control
                HFC_EN: u1,
                ///  Hardware Flow Control RTS Mode
                RTSDC: u1,
                ///  Baud clock enable
                BCLKEN: u1,
                ///  To select the UART clock source for the UART engine (except APB registers). Secondary clock (used for baud rate generator) can be asynchronous from APB clock.
                BCLKSRC: packed union {
                    raw: u2,
                    value: enum(u2) {
                        ///  apb clock
                        Peripheral_Clock = 0x0,
                        ///  Clock 1
                        External_Clock = 0x1,
                        ///  Clock 2
                        CLK2 = 0x2,
                        ///  Clock 3
                        CLK3 = 0x3,
                    },
                },
                ///  Data/Parity bit frame error detection enable
                DPFE_EN: u1,
                ///  Baud clock Ready read only bit
                BCLKRDY: u1,
                ///  UART Clock Auto Gating mode
                UCAGM: u1,
                ///  Fractional Division Mode
                FDM: u1,
                ///  RX Dual Edge Sampling Mode
                DESM: u1,
                padding: u9,
            }),
            ///  Status register
            STATUS: mmio.Mmio(packed struct(u32) {
                ///  Read-only flag indicating the UART transmit status
                TX_BUSY: u1,
                ///  Read-only flag indicating the UART receiver status
                RX_BUSY: u1,
                reserved4: u2,
                ///  Read-only flag indicating the RX FIFO state
                RX_EM: u1,
                ///  Read-only flag indicating the RX FIFO state
                RX_FULL: u1,
                ///  Read-only flag indicating the TX FIFO state
                TX_EM: u1,
                ///  Read-only flag indicating the TX FIFO state
                TX_FULL: u1,
                ///  Indicates the number of bytes currently in the RX FIFO (0-RX FIFO_ELTS)
                RX_LVL: u4,
                ///  Indicates the number of bytes currently in the TX FIFO (0-TX FIFO_ELTS)
                TX_LVL: u4,
                padding: u16,
            }),
            ///  Interrupt Enable control register
            INT_EN: mmio.Mmio(packed struct(u32) {
                ///  Enable Interrupt For RX Frame Error
                RX_FERR: u1,
                ///  Enable Interrupt For RX Parity Error
                RX_PAR: u1,
                ///  Enable Interrupt For CTS signal change Error
                CTS_EV: u1,
                ///  Enable Interrupt For RX FIFO Overrun Error
                RX_OV: u1,
                ///  Enable Interrupt For RX FIFO reaches the number of bytes configured by RXTHD
                RX_THD: u1,
                ///  Enable Interrupt For TX FIFO has one byte remaining
                TX_OB: u1,
                ///  Enable Interrupt For TX FIFO has half empty
                TX_HE: u1,
                padding: u25,
            }),
            ///  Interrupt status flags Control register
            INT_FL: mmio.Mmio(packed struct(u32) {
                ///  Flag for RX Frame Error Interrupt.
                RX_FERR: u1,
                ///  Flag for RX Parity Error interrupt
                RX_PAR: u1,
                ///  Flag for CTS signal change interrupt (hardware flow control disabled)
                CTS_EV: u1,
                ///  Flag for RX FIFO Overrun interrupt
                RX_OV: u1,
                ///  Flag for interrupt when RX FIFO reaches the number of bytes configured by the RXTHD field
                RX_THD: u1,
                ///  Flag for interrupt when TX FIFO has one byte remaining
                TX_OB: u1,
                ///  Flag for interrupt when TX FIFO is half empty
                TX_HE: u1,
                padding: u25,
            }),
            ///  Clock Divider register
            CLKDIV: mmio.Mmio(packed struct(u32) {
                ///  Baud rate divisor value
                CLKDIV: u20,
                padding: u12,
            }),
            ///  Over Sampling Rate register
            OSR: mmio.Mmio(packed struct(u32) {
                ///  OSR
                OSR: u3,
                padding: u29,
            }),
            ///  TX FIFO Output Peek register
            TXPEEK: mmio.Mmio(packed struct(u32) {
                ///  Read TX FIFO next data. Reading from this field does not affect the contents of TX FIFO. Note that the parity bit is available from this field.
                DATA: u8,
                padding: u24,
            }),
            ///  Pin register
            PNR: mmio.Mmio(packed struct(u32) {
                ///  Current sampled value of CTS IO
                CTS: u1,
                ///  This bit controls the value to apply on the RTS IO. If set to 1, the RTS IO is set to high level. If set to 0, the RTS IO is set to low level.
                RTS: u1,
                padding: u30,
            }),
            ///  FIFO Read/Write register
            FIFO: mmio.Mmio(packed struct(u32) {
                ///  Load/unload location for TX and RX FIFO buffers.
                DATA: u8,
                ///  Parity error flag for next byte to be read from FIFO.
                RX_PAR: u1,
                padding: u23,
            }),
            reserved48: [12]u8,
            ///  DMA Configuration register
            DMA: mmio.Mmio(packed struct(u32) {
                ///  TX FIFO Level DMA Trigger If the TX FIFO level is less than this value, then the TX FIFO DMA interface will send a signal to system DMA to notify that TX FIFO is ready to receive data from memory.
                TX_THD_VAL: u4,
                ///  TX DMA channel enable
                TX_EN: u1,
                ///  Rx FIFO Level DMA Trigger If the RX FIFO level is greater than this value, then the RX FIFO DMA interface will send a signal to the system DMA to notify that RX FIFO has characters to transfer to memory.
                RX_THD_VAL: u4,
                ///  RX DMA channel enable
                RX_EN: u1,
                padding: u22,
            }),
            ///  Wake up enable Control register
            WKEN: mmio.Mmio(packed struct(u32) {
                ///  Wake-Up Enable for RX FIFO Not Empty
                RX_NE: u1,
                ///  Wake-Up Enable for RX FIFO Full
                RX_FULL: u1,
                ///  Wake-Up Enable for RX FIFO Threshold Met
                RX_THD: u1,
                padding: u29,
            }),
            ///  Wake up Flags register
            WKFL: mmio.Mmio(packed struct(u32) {
                ///  Wake-Up Flag for RX FIFO Not Empty
                RX_NE: u1,
                ///  Wake-Up Flag for RX FIFO Full
                RX_FULL: u1,
                ///  Wake-Up Flag for RX FIFO Threshold Met
                RX_THD: u1,
                padding: u29,
            }),
        };

        ///  Pulse Train Generation
        pub const PTG = extern struct {
            ///  Global Enable/Disable Controls for All Pulse Trains
            ENABLE: mmio.Mmio(packed struct(u32) {
                ///  Enable/Disable control for PT0
                pt0: u1,
                ///  Enable/Disable control for PT1
                pt1: u1,
                ///  Enable/Disable control for PT2
                pt2: u1,
                ///  Enable/Disable control for PT3
                pt3: u1,
                padding: u28,
            }),
            ///  Global Resync (All Pulse Trains) Control
            RESYNC: mmio.Mmio(packed struct(u32) {
                ///  Resync control for PT0
                pt0: u1,
                ///  Resync control for PT1
                pt1: u1,
                ///  Resync control for PT2
                pt2: u1,
                ///  Resync control for PT3
                pt3: u1,
                padding: u28,
            }),
            ///  Pulse Train Interrupt Flags
            INTFL: mmio.Mmio(packed struct(u32) {
                ///  Pulse Train 0 Stopped Interrupt Flag
                pt0: u1,
                ///  Pulse Train 1 Stopped Interrupt Flag
                pt1: u1,
                ///  Pulse Train 2 Stopped Interrupt Flag
                pt2: u1,
                ///  Pulse Train 3 Stopped Interrupt Flag
                pt3: u1,
                padding: u28,
            }),
            ///  Pulse Train Interrupt Enable/Disable
            INTEN: mmio.Mmio(packed struct(u32) {
                ///  Pulse Train 0 Stopped Interrupt Enable/Disable
                pt0: u1,
                ///  Pulse Train 1 Stopped Interrupt Enable/Disable
                pt1: u1,
                ///  Pulse Train 2 Stopped Interrupt Enable/Disable
                pt2: u1,
                ///  Pulse Train 3 Stopped Interrupt Enable/Disable
                pt3: u1,
                padding: u28,
            }),
            ///  Pulse Train Global Safe Enable.
            SAFE_EN: mmio.Mmio(packed struct(u32) {
                PT0: u1,
                PT1: u1,
                PT2: u1,
                PT3: u1,
                padding: u28,
            }),
            ///  Pulse Train Global Safe Disable.
            SAFE_DIS: mmio.Mmio(packed struct(u32) {
                PT0: u1,
                PT1: u1,
                PT2: u1,
                PT3: u1,
                padding: u28,
            }),
        };

        ///  Power Sequencer / Low Power Control Register.
        pub const PWRSEQ = extern struct {
            ///  Low Power Control Register.
            LPCN: mmio.Mmio(packed struct(u32) {
                ///  System RAM retention in BACKUP mode. These two bits are used in conjuction with RREGEN bit.
                RAMRET0: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable Ram Retention.
                        dis = 0x0,
                        ///  Enable System RAM 0 retention.
                        en = 0x1,
                    },
                },
                ///  System RAM retention in BACKUP mode. These two bits are used in conjuction with RREGEN bit.
                RAMRET1: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable Ram Retention.
                        dis = 0x0,
                        ///  Enable System RAM 1 retention.
                        en = 0x1,
                    },
                },
                ///  System RAM retention in BACKUP mode. These two bits are used in conjuction with RREGEN bit.
                RAMRET2: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable Ram Retention.
                        dis = 0x0,
                        ///  Enable System RAM 2 retention.
                        en = 0x1,
                    },
                },
                ///  System RAM retention in BACKUP mode. These two bits are used in conjuction with RREGEN bit.
                RAMRET3: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable Ram Retention.
                        dis = 0x0,
                        ///  Enable System RAM 3 retention.
                        en = 0x1,
                    },
                },
                reserved8: u4,
                ///  Low Power Mode APB Clock Select.
                LPMCLKSEL: u1,
                ///  Low Power Mode Clock Select.
                LPMFAST: u1,
                reserved11: u1,
                ///  Bandgap OFF. This controls the System Bandgap in DeepSleep mode.
                BG_DIS: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Bandgap is always ON.
                        on = 0x0,
                        ///  Bandgap is OFF in DeepSleep mode (default).
                        off = 0x1,
                    },
                },
                reserved31: u19,
                ///  Low Power Wakeup Status Register Clear
                LPWKST_CLR: u1,
            }),
            ///  Low Power I/O Wakeup Status Register 0. This register indicates the low power wakeup status for GPIO0.
            LPWKST0: mmio.Mmio(packed struct(u32) {
                ///  Wakeup IRQ flags (write ones to clear). One or more of these bits will be set when the corresponding dedicated GPIO pin (s) transition (s) from low to high or high to low. If GPIO wakeup source is selected, using PM.GPIOWKEN register, and the corresponding bit is also selected in LPWKEN register, an interrupt will be gnerated to wake up the CPU from a low power mode.
                WAKEST: u1,
                padding: u31,
            }),
            ///  Low Power I/O Wakeup Enable Register 0. This register enables low power wakeup functionality for GPIO0.
            LPWKEN0: mmio.Mmio(packed struct(u32) {
                ///  Enable wakeup. These bits allow wakeup from the corresponding GPIO pin (s) on transition (s) from low to high or high to low when PM.GPIOWKEN is set. Wakeup status is indicated in PPWKST register.
                WAKEEN: u31,
                padding: u1,
            }),
            ///  Low Power I/O Wakeup Status Register 1. This register indicates the low power wakeup status for GPIO1.
            LPWKST1: u32,
            ///  Low Power I/O Wakeup Enable Register 1. This register enables low power wakeup functionality for GPIO1.
            LPWKEN1: u32,
            ///  Low Power I/O Wakeup Status Register 2. This register indicates the low power wakeup status for GPIO2.
            LPWKST2: u32,
            ///  Low Power I/O Wakeup Enable Register 2. This register enables low power wakeup functionality for GPIO2.
            LPWKEN2: u32,
            ///  Low Power I/O Wakeup Status Register 3. This register indicates the low power wakeup status for GPIO3.
            LPWKST3: u32,
            ///  Low Power I/O Wakeup Enable Register 3. This register enables low power wakeup functionality for GPIO3.
            LPWKEN3: u32,
            reserved48: [12]u8,
            ///  Low Power Peripheral Wakeup Status Register.
            LPPWST: mmio.Mmio(packed struct(u32) {
                reserved4: u4,
                ///  Analog Input Comparator Wakeup Flag.
                AINCOMP0: u1,
                reserved16: u11,
                ///  Backup Mode Wakeup Flag.
                BACKUP: u1,
                ///  Reset Detected Wakeup Flag.
                RESET: u1,
                padding: u14,
            }),
            ///  Low Power Peripheral Wakeup Enable Register.
            LPPWEN: mmio.Mmio(packed struct(u32) {
                reserved4: u4,
                ///  AINCOMP0 Wakeup Enable. This bit allows wakeup from the AINCOMP0.
                AINCOMP0: u1,
                reserved8: u3,
                ///  WDT0 Wakeup Enable. This bit allows wakeup from the WDT0.
                WDT0: u1,
                ///  WDT1 Wakeup Enable. This bit allows wakeup from the WDT1.
                WDT1: u1,
                ///  CPU1 Wakeup Enable. This bit allows wakeup from the CPU1.
                CPU1: u1,
                ///  TMR0 Wakeup Enable. This bit allows wakeup from the TMR0.
                TMR0: u1,
                ///  TMR1 Wakeup Enable. This bit allows wakeup from the TMR1.
                TMR1: u1,
                ///  TMR2 Wakeup Enable. This bit allows wakeup from the TMR2.
                TMR2: u1,
                ///  TMR3 Wakeup Enable. This bit allows wakeup from the TMR3.
                TMR3: u1,
                ///  TMR4 Wakeup Enable. This bit allows wakeup from the TMR4.
                TMR4: u1,
                ///  TMR5 Wakeup Enable. This bit allows wakeup from the TMR5.
                TMR5: u1,
                ///  UART0 Wakeup Enable. This bit allows wakeup from the UART0.
                UART0: u1,
                ///  UART1 Wakeup Enable. This bit allows wakeup from the UART1.
                UART1: u1,
                ///  UART2 Wakeup Enable. This bit allows wakeup from the UART2.
                UART2: u1,
                ///  UART3 Wakeup Enable. This bit allows wakeup from the UART3.
                UART3: u1,
                ///  I2C0 Wakeup Enable. This bit allows wakeup from the I2C0.
                I2C0: u1,
                ///  I2C1 Wakeup Enable. This bit allows wakeup from the I2C1.
                I2C1: u1,
                ///  I2C2 Wakeup Enable. This bit allows wakeup from the I2C2.
                I2C2: u1,
                ///  I2S Wakeup Enable. This bit allows wakeup from the I2S.
                I2S: u1,
                ///  SPI1 Wakeup Enable. This bit allows wakeup from the SPI1.
                SPI1: u1,
                ///  LPCMP Wakeup Enable. This bit allows wakeup from the LPCMP.
                LPCMP: u1,
                padding: u5,
            }),
            reserved72: [16]u8,
            ///  General Purpose Register 0
            GP0: u32,
            ///  General Purpose Register 1
            GP1: u32,
        };

        ///  Real Time Clock and Alarm.
        pub const RTC = extern struct {
            ///  RTC Second Counter. This register contains the 32-bit second counter.
            SEC: mmio.Mmio(packed struct(u32) {
                ///  Seconds Counter.
                SEC: u32,
            }),
            ///  RTC Sub-second Counter. This counter increments at 256Hz. RTC_SEC is incremented when this register rolls over from 0xFF to 0x00.
            SSEC: mmio.Mmio(packed struct(u32) {
                ///  Sub-Seconds Counter (12-bit).
                SSEC: u12,
                padding: u20,
            }),
            ///  Time-of-day Alarm.
            TODA: mmio.Mmio(packed struct(u32) {
                ///  Time-of-day Alarm.
                TOD_ALARM: u20,
                padding: u12,
            }),
            ///  RTC sub-second alarm. This register contains the reload value for the sub-second alarm.
            SSECA: mmio.Mmio(packed struct(u32) {
                ///  This register contains the reload value for the sub-second alarm.
                SSEC_ALARM: u32,
            }),
            ///  RTC Control Register.
            CTRL: mmio.Mmio(packed struct(u32) {
                ///  Real Time Clock Enable. This bit enables the Real Time Clock. This bit can only be written when WE=1 and BUSY =0. Change to this bit is effective only after BUSY is cleared from 1 to 0.
                EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable.
                        dis = 0x0,
                        ///  Enable.
                        en = 0x1,
                    },
                },
                ///  Alarm Time-of-Day Interrupt Enable. Change to this bit is effective only after BUSY is cleared from 1 to 0.
                TOD_ALARM_IE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable.
                        dis = 0x0,
                        ///  Enable.
                        en = 0x1,
                    },
                },
                ///  Alarm Sub-second Interrupt Enable. Change to this bit is effective only after BUSY is cleared from 1 to 0.
                SSEC_ALARM_IE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable.
                        dis = 0x0,
                        ///  Enable.
                        en = 0x1,
                    },
                },
                ///  RTC Busy. This bit is set to 1 by hardware when changes to RTC registers required a synchronized version of the register to be in place. This bit is automatically cleared by hardware.
                BUSY: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Idle.
                        idle = 0x0,
                        ///  Busy.
                        busy = 0x1,
                    },
                },
                ///  RTC Ready. This bit is set to 1 by hardware when the RTC count registers update. It can be cleared to 0 by software at any time. It will also be cleared to 0 by hardware just prior to an update of the RTC count register.
                RDY: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Register has not updated.
                        busy = 0x0,
                        ///  Ready.
                        ready = 0x1,
                    },
                },
                ///  RTC Ready Interrupt Enable.
                RDY_IE: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Disable.
                        dis = 0x0,
                        ///  Enable.
                        en = 0x1,
                    },
                },
                ///  Time-of-Day Alarm Interrupt Flag. This alarm is qualified as wake-up source to the processor.
                TOD_ALARM: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Not active
                        inactive = 0x0,
                        ///  Active
                        Pending = 0x1,
                    },
                },
                ///  Sub-second Alarm Interrupt Flag. This alarm is qualified as wake-up source to the processor.
                SSEC_ALARM: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Not active
                        inactive = 0x0,
                        ///  Active
                        Pending = 0x1,
                    },
                },
                ///  Square Wave Output Enable.
                SQW_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Not active
                        inactive = 0x0,
                        ///  Active
                        Pending = 0x1,
                    },
                },
                ///  Frequency Output Selection. When SQE=1, these bits specify the output frequency on the SQW pin.
                SQW_SEL: packed union {
                    raw: u2,
                    value: enum(u2) {
                        ///  1 Hz (Compensated).
                        freq1Hz = 0x0,
                        ///  512 Hz (Compensated).
                        freq512Hz = 0x1,
                        ///  4 KHz.
                        freq4KHz = 0x2,
                        ///  RTC Input Clock / 8.
                        clkDiv8 = 0x3,
                    },
                },
                reserved14: u3,
                ///  Asynchronous Counter Read Enable.
                RD_EN: u1,
                ///  Write Enable. This register bit serves as a protection mechanism against unintentional writes to critical RTC bits.
                WR_EN: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Not active
                        inactive = 0x0,
                        ///  Active
                        Pending = 0x1,
                    },
                },
                padding: u16,
            }),
            ///  RTC Trim Register.
            TRIM: mmio.Mmio(packed struct(u32) {
                ///  RTC Trim. This register contains the 2's complement value that specifies the trim resolution. Each increment or decrement of the bit adds or subtracts 1ppm at each 4KHz clock value, with a maximum correction of +/- 127ppm.
                TRIM: u8,
                ///  VBAT Timer Value. When RTC is running off of VBAT, this field is incremented every 32 seconds.
                VRTC_TMR: u24,
            }),
            ///  RTC Oscillator Control Register.
            OSCCTRL: mmio.Mmio(packed struct(u32) {
                ///  Enables analog deglitch filter.
                FILTER_EN: u1,
                ///  If IBIAS_EN is 1, selects 4x,2x mode.
                IBIAS_SEL: u1,
                ///  Enables high current hysteresis buffer.
                HYST_EN: u1,
                ///  Enables higher 4x,2x current modes.
                IBIAS_EN: u1,
                ///  RTC Crystal Bypass
                BYPASS: u1,
                ///  RTC 32kHz Square Wave Output
                SQW_32K: u1,
                padding: u26,
            }),
        };

        ///  The Semaphore peripheral allows multiple cores in a system to cooperate when accessing shred resources. The peripheral contains eight semaphores that can be atomically set and cleared. It is left to the discretion of the software architect to decide how and when the semaphores are used and how they are allocated. Existing hardware does not have to be modified for this type of cooperative sharing, and the use of semaphores is exclusively within the software domain.
        pub const SEMA = extern struct {
            ///  Read to test and set, returns prior value. Write 0 to clear semaphore.
            SEMAPHORES: [8]mmio.Mmio(packed struct(u32) {
                sema: u1,
                padding: u31,
            }),
            reserved64: [32]u8,
            ///  Semaphore IRQ0 register.
            irq0: mmio.Mmio(packed struct(u32) {
                en: u1,
                reserved16: u15,
                cm4_irq: u1,
                padding: u15,
            }),
            ///  Semaphore Mailbox 0 register.
            mail0: mmio.Mmio(packed struct(u32) {
                data: u32,
            }),
            ///  Semaphore IRQ1 register.
            irq1: mmio.Mmio(packed struct(u32) {
                en: u1,
                reserved16: u15,
                rv32_irq: u1,
                padding: u15,
            }),
            ///  Semaphore Mailbox 1 register.
            mail1: mmio.Mmio(packed struct(u32) {
                data: u32,
            }),
            reserved256: [176]u8,
            ///  Semaphore status bits. 0 indicates the semaphore is free, 1 indicates taken.
            status: mmio.Mmio(packed struct(u32) {
                status0: u1,
                status1: u1,
                status2: u1,
                status3: u1,
                status4: u1,
                status5: u1,
                status6: u1,
                status7: u1,
                padding: u24,
            }),
        };

        ///  Single Inductor Multiple Output Switching Converter
        pub const SIMO = extern struct {
            reserved4: [4]u8,
            ///  Buck Voltage Regulator A Control Register
            VREGO_A: mmio.Mmio(packed struct(u32) {
                ///  Regulator Output Voltage Setting
                VSETA: u7,
                ///  Regulator Output Range Set
                RANGEA: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Low output voltage range
                        low = 0x0,
                        ///  High output voltage range
                        high = 0x1,
                    },
                },
                padding: u24,
            }),
            ///  Buck Voltage Regulator B Control Register
            VREGO_B: mmio.Mmio(packed struct(u32) {
                ///  Regulator Output Voltage Setting
                VSETB: u7,
                ///  Regulator Output Range Set
                RANGEB: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Low output voltage range
                        low = 0x0,
                        ///  High output voltage range
                        high = 0x1,
                    },
                },
                padding: u24,
            }),
            ///  Buck Voltage Regulator C Control Register
            VREGO_C: mmio.Mmio(packed struct(u32) {
                ///  Regulator Output Voltage Setting
                VSETC: u7,
                ///  Regulator Output Range Set
                RANGEC: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Low output voltage range
                        low = 0x0,
                        ///  High output voltage range
                        high = 0x1,
                    },
                },
                padding: u24,
            }),
            ///  Buck Voltage Regulator D Control Register
            VREGO_D: mmio.Mmio(packed struct(u32) {
                ///  Regulator Output Voltage Setting
                VSETD: u7,
                ///  Regulator Output Range Set
                RANGED: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Low output voltage range
                        low = 0x0,
                        ///  High output voltage range
                        high = 0x1,
                    },
                },
                padding: u24,
            }),
            ///  High Side FET Peak Current VREGO_A/VREGO_B Register
            IPKA: mmio.Mmio(packed struct(u32) {
                ///  Voltage Regulator Peak Current Setting
                IPKSETA: u4,
                ///  Voltage Regulator Peak Current Setting
                IPKSETB: u4,
                padding: u24,
            }),
            ///  High Side FET Peak Current VREGO_C/VREGO_D Register
            IPKB: mmio.Mmio(packed struct(u32) {
                ///  Voltage Regulator Peak Current Setting
                IPKSETC: u4,
                ///  Voltage Regulator Peak Current Setting
                IPKSETD: u4,
                padding: u24,
            }),
            ///  Maximum High Side FET Time On Register
            MAXTON: mmio.Mmio(packed struct(u32) {
                ///  Sets the maximum on time for the high side FET, each increment represents 500ns
                TONSET: u4,
                padding: u28,
            }),
            ///  Buck Cycle Count VREGO_A Register
            ILOAD_A: mmio.Mmio(packed struct(u32) {
                ///  Number of buck cycles that occur within the cycle clock
                ILOADA: u8,
                padding: u24,
            }),
            ///  Buck Cycle Count VREGO_B Register
            ILOAD_B: mmio.Mmio(packed struct(u32) {
                ///  Number of buck cycles that occur within the cycle clock
                ILOADB: u8,
                padding: u24,
            }),
            ///  Buck Cycle Count VREGO_C Register
            ILOAD_C: mmio.Mmio(packed struct(u32) {
                ///  Number of buck cycles that occur within the cycle clock
                ILOADC: u8,
                padding: u24,
            }),
            ///  Buck Cycle Count VREGO_D Register
            ILOAD_D: mmio.Mmio(packed struct(u32) {
                ///  Number of buck cycles that occur within the cycle clock
                ILOADD: u8,
                padding: u24,
            }),
            ///  Buck Cycle Count Alert VERGO_A Register
            BUCK_ALERT_THR_A: mmio.Mmio(packed struct(u32) {
                ///  Threshold for ILOADA to generate the BUCK_ALERT
                BUCKTHRA: u8,
                padding: u24,
            }),
            ///  Buck Cycle Count Alert VERGO_B Register
            BUCK_ALERT_THR_B: mmio.Mmio(packed struct(u32) {
                ///  Threshold for ILOADB to generate the BUCK_ALERT
                BUCKTHRB: u8,
                padding: u24,
            }),
            ///  Buck Cycle Count Alert VERGO_C Register
            BUCK_ALERT_THR_C: mmio.Mmio(packed struct(u32) {
                ///  Threshold for ILOADC to generate the BUCK_ALERT
                BUCKTHRC: u8,
                padding: u24,
            }),
            ///  Buck Cycle Count Alert VERGO_D Register
            BUCK_ALERT_THR_D: mmio.Mmio(packed struct(u32) {
                ///  Threshold for ILOADD to generate the BUCK_ALERT
                BUCKTHRD: u8,
                padding: u24,
            }),
            ///  Buck Regulator Output Ready Register
            BUCK_OUT_READY: mmio.Mmio(packed struct(u32) {
                ///  When set, indicates that the output voltage has reached its regulated value
                BUCKOUTRDYA: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Output voltage not in range
                        notrdy = 0x0,
                        ///  Output voltage in range
                        rdy = 0x1,
                    },
                },
                ///  When set, indicates that the output voltage has reached its regulated value
                BUCKOUTRDYB: u1,
                ///  When set, indicates that the output voltage has reached its regulated value
                BUCKOUTRDYC: u1,
                ///  When set, indicates that the output voltage has reached its regulated value
                BUCKOUTRDYD: u1,
                padding: u28,
            }),
            ///  Zero Cross Calibration VERGO_A Register
            ZERO_CROSS_CAL_A: mmio.Mmio(packed struct(u32) {
                ///  Zero Cross Calibrartion Value VREGO_A
                ZXCALA: u4,
                padding: u28,
            }),
            ///  Zero Cross Calibration VERGO_B Register
            ZERO_CROSS_CAL_B: mmio.Mmio(packed struct(u32) {
                ///  Zero Cross Calibrartion Value VREGO_B
                ZXCALB: u4,
                padding: u28,
            }),
            ///  Zero Cross Calibration VERGO_C Register
            ZERO_CROSS_CAL_C: mmio.Mmio(packed struct(u32) {
                ///  Zero Cross Calibrartion Value VREGO_C
                ZXCALC: u4,
                padding: u28,
            }),
            ///  Zero Cross Calibration VERGO_D Register
            ZERO_CROSS_CAL_D: mmio.Mmio(packed struct(u32) {
                ///  Zero Cross Calibrartion Value VREGO_D
                ZXCALD: u4,
                padding: u28,
            }),
        };

        ///  System Initialization Registers.
        pub const SIR = extern struct {
            ///  System Initialization Status Register.
            SISTAT: mmio.Mmio(packed struct(u32) {
                ///  Magic Word Validation. This bit is set by the system initialization block following power-up.
                MAGIC: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  Magic word was not set (OTP has not been initialized properly).
                        magicNotSet = 0x0,
                        ///  Magic word was set (OTP contains valid settings).
                        magicSet = 0x1,
                    },
                },
                ///  CRC Error Status. This bit is set by the system initialization block following power-up.
                CRCERR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        ///  No CRC errors occurred during the read of the OTP memory block.
                        noError = 0x0,
                        ///  A CRC error occurred while reading the OTP. The address of the failure location in the OTP memory is stored in the ERRADDR register.
                        @"error" = 0x1,
                    },
                },
                padding: u30,
            }),
            ///  Read-only field set by the SIB block if a CRC error occurs during the read of the OTP memory. Contains the failing address in OTP memory (when CRCERR equals 1).
            ADDR: mmio.Mmio(packed struct(u32) {
                ERRADDR: u32,
            }),
            reserved256: [248]u8,
            ///  funcstat register.
            FSTAT: mmio.Mmio(packed struct(u32) {
                ///  FPU Function.
                FPU: packed union {
                    raw: u1,
                    value: enum(u1) {
                        no = 0x0,
                        yes = 0x1,
                    },
                },
                reserved2: u1,
                ///  10-bit Sigma Delta ADC.
                ADC: packed union {
                    raw: u1,
                    value: enum(u1) {
                        no = 0x0,
                        yes = 0x1,
                    },
                },
                reserved7: u4,
                ///  SMPHR function.
                SMPHR: packed union {
                    raw: u1,
                    value: enum(u1) {
                        no = 0x0,
                        yes = 0x1,
                    },
                },
                padding: u24,
            }),
            ///  Security function status register.
            SFSTAT: mmio.Mmio(packed struct(u32) {
                ///  TRNG Function.
                TRNG: packed union {
                    raw: u1,
                    value: enum(u1) {
                        no = 0x0,
                        yes = 0x1,
                    },
                },
                reserved2: u1,
                ///  AES Block.
                AES: packed union {
                    raw: u1,
                    value: enum(u1) {
                        no = 0x0,
                        yes = 0x1,
                    },
                },
                padding: u29,
            }),
        };
    };
};
