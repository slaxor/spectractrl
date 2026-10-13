# spectractrl

Embassy async firmware for the RP2040 (e.g. Raspberry Pi Pico).

## Layout

- `lib/` — reusable, `no_std` async logic. Free of direct peripheral access.
- `firmware/` — thin embassy entrypoint, wires the lib's logic to real hardware.

## Hardware note

The blink example targets `PIN_25`, the onboard LED on a plain Pico/Pico2.
On Pico W/Pico2 W boards the LED is wired to the CYW43 wifi chip instead and
needs a different setup (see embassy's `cyw43` examples).

## Dev shell

```sh
nix develop
```

## Build & flash

```sh
cargo build --release
cargo run --release   # builds, flashes via probe-rs, and shows defmt/RTT logs
```

Needs a debug probe (e.g. a second Pico running the [debugprobe](https://github.com/raspberrypi/debugprobe) firmware, or a Raspberry Pi Debug Probe) wired to the SWD pins.

## License

GPLv3, see [LICENSE](./LICENSE).
