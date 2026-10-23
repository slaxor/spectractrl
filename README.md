# spectractrl

Firmware for an RP2040 board that sits between a USB host and an Acton Research
SpectraPro-275 monochromator setup: the **SpectraDrive** controller (grating and
wavelength) and the **PS-445** (PMT supply and readout). Both speak Forth over
RS-232. spectractrl offers one USB serial console that talks to both, guards
what gets sent, and runs measurement macros that coordinate the two devices.

Written in Rust on [embassy](https://embassy.dev), `no_std`, no heap.

## Features

- One console, two devices: `sd …` goes to the SpectraDrive, `ps …` to the PS-445.
  Each device has its own queue and worker, so a grating change that takes
  minutes does not block the other device.
- Every reply is shown as an event: `[sd:done]`, `[ps:out]`, `[sd:fail]`,
  `[sd:abort]`. Unsolicited output (power-on banners) shows up too, and
  non-printable bytes are shown as an `xxd` dump.
- Per-device whitelist with argument checking: `300.5 GOTO` passes, `300 GOTO`
  and `300.500 GOTO` are refused before they reach the device.
- Hardware unlock: a jumper on GP2 lifts the whitelist and enables a transparent
  link to a device. Nothing typed over USB can do that.
- Macros: integrated scan, multi-read scan, point measurement. All of them can be
  previewed with `dry-run`.
- Line editing with history (up/down) and `.` to repeat the last command.
- The RGB LED shows the colour of the wavelength the SpectraDrive was last sent to.

## Hardware

Developed on a Pimoroni Tiny 2040; a Raspberry Pi Pico works the same, only
without the on-board RGB LED.

| GPIO | Function |
|---|---|
| GP0 / GP1 | UART0 TX / RX → SpectraDrive |
| GP4 / GP5 | UART1 TX / RX → PS-445 |
| GP2 | unlock jumper to GND (internal pull-up, open = locked) |
| GP18 / GP19 / GP20 | RGB LED red / green / blue (Tiny 2040, active low) |

Both UARTs run at 9600 baud, 8N1, through **MAX3232** level shifters powered
from 3V3. Do not use 5 V MAX232 modules: their TTL outputs would drive 5 V into
the RP2040's RX pins.

## Console

The board enumerates as a CDC-ACM device (`16c0:27dd`, product `spectractrl`).
Use the by-id path, since a debug probe usually takes `ttyACM0`:

```sh
tio /dev/serial/by-id/usb-Microprojects_spectractrl_0001-if00
```

| Command | Effect |
|---|---|
| `help` | list of all commands |
| `sd <forth>` / `ps <forth>` | queue a command for the SpectraDrive / PS-445 |
| `status` | local state; also asks idle devices for theirs |
| `allowed sd\|ps` | whitelisted words with their arguments |
| `abort sd\|ps\|scan` | stop waiting for a device, or stop the running macro |
| `dry-run <command>` | show what a device command or macro would send, send nothing |
| `goto sd\|ps` | transparent link to a device, unlock jumper only; `Ctrl-]` leaves |
| up / down, `.` | command history, repeat the last command |

The first word of a line decides what it is: `sd`/`ps` always go to a device,
console commands go by name, everything else is parsed as a macro.

### Macros

```
<grating> <start-nm> <end-nm> <step-nm> <pmt-volts> <integration-ms> SCAN
<grating> <start-nm> <end-nm> <step-nm> <pmt-volts> <reads> MULTI-SCAN
<grating> <nm> <reads> POINT-MEASURE
```

- **SCAN** is the default: `GRATING`, `VOLTS`, `CURRENT`, `PMT-ON`, `ITIME`,
  `INTEGRATE-START`, then per step `GOTO`, a wait of two integration windows and
  one `.READ`. Output is one `nm,reading` row per step.
- **MULTI-SCAN** reads `<reads>` times per step instead of integrating, giving one
  `nm,reading,reading,...` row per step for spread and outliers.
- **POINT-MEASURE** does the same at a single wavelength and leaves the PMT voltage
  as it is.

Every macro ends with `PMT-OFF`, also when a step fails or the scan is aborted.
Commands sent by a macro always go through the whitelist.

```
> dry-run 3 200.0 201.0 0.5 450 800 SCAN
[scan:out] # grating 3, 200.0..201.0 nm, step 0.5 nm, 450 V, 800 ms integration per step
[scan:out] sd> 3 GRATING
[scan:out] ps> 450 VOLTS
...
[scan:done] dry run, 3 steps, nothing sent
```

### Numbers

Device commands take `<int>` (`500`) or `<dec>` with exactly one digit after the
point (`300.5`). Lines sent to a device are limited to 79 characters, because the
devices' Forth input buffer holds 80.

## Safety model

- **Whitelist:** each device has a list of allowed words with argument types.
  Anything else is refused on the controller.
- **Unlock jumper (GP2 to GND):** `sd`/`ps` skip the whitelist and `goto` becomes
  available. The LED blinks red while it is closed. Opening it relocks immediately
  and ends a running `goto`. Macros stay whitelisted regardless.
- **Line length:** lines over 79 characters are refused, jumper or not.

This keeps the USB side from sending arbitrary commands. It does not protect
against reflashing the board.

## Layout

- `lib/`: `no_std` logic without peripheral access, unit-tested on the host:
  command whitelist, macro parsing, history, wavelength colour, `xxd` rows.
- `firmware/`: embassy entry point for the RP2040, wiring the lib to UARTs, USB,
  GPIO and PWM.

## Building

```sh
nix develop      # Rust with thumbv6m-none-eabi, probe-rs, just, llvm
just test        # lib unit tests on the host
just build
just run         # build, flash via probe-rs, show defmt logs
```

Flashing needs an SWD probe, e.g. a second Pico running the
[debugprobe](https://github.com/raspberrypi/debugprobe) firmware. Set the log
level with `DEFMT_LOG=debug just run`; `trace` shows every byte on the UARTs.

## Status

Work in progress. Device behaviour is partly based on observation, not a
manual: replies are taken as finished on a trailing `ok`, as failed on a
trailing `?`, or after 500 ms of silence following any other line. The grating
wavelength ranges in `GRATINGS` are placeholders.

## License

GPLv3, see [LICENSE](./LICENSE).
