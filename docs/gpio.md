# GPIO Hardware Bring-Up Notes

## Power-On-Reset Configuration

Following a POR event, all GPIO, except device pins that have the `SWDIO` and 
`SWDCLK` function, are configured with the following default settings:
- GPIO mode enabled
    - `GPIOn.EN0.EN[pin] = 1`
    - `GPIOn.EN1.EN[pin] = 0`
    - `GPIOn.EN2.EN[pin] = 0`
- Pullup/pulldown disabled, I/O in Hi-Z mode
    - `GPIOn.PADCTRL0.MODE[pin] = 0`
    - `GPIOn.PADCTRL1.MODE[pin]`
- Output mode disabled
    - `GPIOn.OUTEN.EN[PIN] = 0`
- Interrupt disabled
    - `GPIOn.INTEN.EN[pin] = 0`


## Serial Wire Debug (SWD) Configuration

Perform the following steps to configure the `SWDIO` and `SWDCLK` device pins 
for SWD mode:
1. Set device pin `P0.28` to `AF1` mode:
    a. `GPIOn.EN0.CONFIG[28] = 0`
    b. `GPIOn.EN1.CONFIG[28] = 0`
    c. `GPIOn.EN2.CONFIG[28] = 0`
2. Set device pin `P0.29` to `AF1` mode:
    a. `GPIOn.EN0.CONFIG[29] = 0`
    b. `GPIOn.EN1.CONFIG[29] = 0`
    c. `GPIOn.EN2.CONFIG[29] = 0`

> ![note]
> To use the SWD pins in GPIO mode, set the desired GPIO pins to SWD AF and 
> disable the SWD (`GCR_SYSCTRL.SWD_DIS = 1`).

## Alternate Function (AF) Configuration

| **Mode**                  | `GPIOn.EN0.CONFIG[pin]` | `GPIOn.EN1.CONFIG[pin]` | `GPIOn.EN2.CONFIG[pin]` |
| :-----------------------: | :---------------------: | :---------------------: | :---------------------: |
| `AF1`                     | 0                       | 0                       | 0                       |
| `AF2`                     | 0                       | 1                       | 0                       |
| I/O (transition to `AF1`) | 1                       | 0                       | 0                       |
| I/O (transition to `AF2`) | 1                       | 1                       | 0                       |


## Input Mode Configuration

`GPIOn.INEN.EN[pin] = 1` to put the pin into input mode. This enables the input 
buffer connected to the GPIO pin.


| **Input Mode**                          | `GPIOn.PADCTRL1.CONFIG[pin]` | `GPIOn.PADCTRL0.CONFIG[pin]` | `GPIOn.PS.PULL_SEL[pin]` | `GPIOn.VSSEL.V_SEL[pin]` |
| :-------------------------------------- | :--------------------------: | :--------------------------: | :----------------------: | :----------------------: |
| High impedance                          | 0                            | 0                            | N/A                      | N/A                      |
| Weak Pullup to $V_{DDIO}$ (1MOhm)       | 0                            | 1                            | 0                        | 0                        |
| Strong Pullup to $V_{DDIO}$ (25KOhm)    | 0                            | 1                            | 1                        | 0                        |
| Weak Pulldown to $V_{DDIOH}$ (1MOhm)    | 1                            | 0                            | 0                        | 1                        |
| Strong Pulldown to $V_{DDIOH}$ (25KOhm) | 1                            | 0                            | 1                        | 1                        |
| Reserved                                | 1                            | 1                            | N/A                      | N/A                      |


- `GPIOn.PADCTRL{0,1}` control the pull mode - High-impedance, Pull-up, Pull-down.
- `GPIOn.PS.PULL_SEL` controls the Pull-up/Pull-down strength.
- `GPIOn.VSSEL.V_SEL` controls the power supply selection.

> So we can select the pull and the pull strength

Input pins can be read via `GPIOn.IN.level[pin]`.

### Output Mode Configuration

Set `GPIOn.OUTEN.EN[pin] = 1` to set the pin to output. Enables output buffer 
for the pin.


| **Output Mode**                      | `GPIOn.DS1.CONFIG[pin]` | `GPIOn.DS0.CONFIG[pin]` | `GPIOn.VSSEL.V_SEL[pin]` |
| :----------------------------------: | :---------------------: | :---------------------: | :---------------------: |
| Drive Strength 0, $V_{DDIO}$ Supply  | 0                       | 0                       | 0                       |
| Drive Strength 1, $V_{DDIO}$ Supply  | 0                       | 1                       | 0                       |
| Drive Strength 2, $V_{DDIO}$ Supply  | 1                       | 0                       | 0                       |
| Drive Strength 3, $V_{DDIO}$ Supply  | 1                       | 1                       | 0                       |
| Drive Strength 0, $V_{DDIOH}$ Supply | 0                       | 0                       | 1                       |
| Drive Strength 1, $V_{DDIOH}$ Supply | 0                       | 1                       | 1                       |
| Drive Strength 2, $V_{DDIOH}$ Supply | 1                       | 0                       | 1                       |
| Drive Strength 3, $V_{DDIOH}$ Supply | 1                       | 1                       | 1                       |

Each GPIO port is assigned a dedicated interrup vector.

> So you can select the Power supply ($V_{DDIO}$ or $V_{DDIOH}$) as well as the 
> drive strength (0 - 3).

Then the output pin can be written to:
```
// HIGH
GPIOn.OUT.LEVEL[pin] = 1
// LOW
GPIOn.OUT.LEVEL[pin] = 0
```

## Input/Output Configuration summary
- Pull-up/Pull-down: `GPIOn.PADCTRL{0,1}.CONFIG[pin]`
- Strength select: `GPIOn.PS.PULL_SEL[pin]`
- Drive Strength: `GPIOn.DS{0,1}.CONFIG[pin]`
- Voltage: `GPIOn.VSSEL.V_SEL[pin]`

## Interrupt Configuration

- **Enable Interrupt**: Set `GPIOn.INTEN.EN[pin] = 1`.
- **Status**:  Read `GPIOn.INTFL.config[pin]`.
- **Dual Edge**: `GPIOn.DUALEDGE.DUALEDGE[pin]`
- **Polarity**: `GPIOn.INTPOL.POL[pin]`
- **Trigger**: `GPIOn.INTMODE.GPIO_INTMODE[pin]`
- **Wakeup**: `GPIOn.WKEN.EN[0]`
