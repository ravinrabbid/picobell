# Picobell - Sender and Receiver for Honeywell "ActivLink" Wireless Doorbells

This project provides RP2040 firmware for sending and receiving packets for [Honeywell "ActivLink"](https://github.com/klohner/honeywell-wireless-doorbell) type doorbells. It uses a CC1101 transceiver as a radio and is based on [embassy-rs](https://github.com/embassy-rs/embassy).

It has so far only been tested with a Friedland Libra+ 868MHz wireless doorbell, but should be compatible with other devices using the Honeywell protocol.

## Features

- Three different firmware binaries to choose from: "transmitter", "receiver" and "transceiver".
- Support for LEDs displaying alarm type, low battery of the sender, and time elapsed since a packet was received.
- Support to trigger external wired doorbells.

## Hardware

The firmwares have been tested with a Seeed Studio XIAO-RP2040, but should also be compatible with other modules like the RaspberryPi Pico.

Further you'll need a CC1101 SPI transceiver module matching the frequency your doorbell uses, senders will also need a push button.

Optionally you can attach some WS2810 type LEDs for optical feedback on sending/receiving and an external wired doorbell for sound.

## Configuration

Configuration is done within the `config.rs` files accompanying the binaries. See documentation within the files for details.

Here you can also configure the pins used for all external peripherals.

## Building

Run

```sh
cargo build --release
```

or one of

```sh
cargo build --release --bin receiver
cargo build --release --bin transmitter
cargo build --release --bin transceiver
```

to build all or single binaries respectively.

Flash using `picotool`

```sh
picotool load -f -v -x target/thumbv6m-none-eabi/release/receiver -t elf
picotool load -f -v -x target/thumbv6m-none-eabi/release/transmitter -t elf
picotool load -f -v -x target/thumbv6m-none-eabi/release/transceiver -t elf
```

You can also install [probe-rs](https://probe.rs) and directly use

```sh
cargo run --release --bin receiver
cargo run --release --bin transmitter
cargo run --release --bin transceiver
```

## Acknowledgements

- [klohner](https://github.com/klohner) for reverse engineering the [Honeywell protocoll](https://github.com/klohner/honeywell-wireless-doorbell)
- [jgromes](https://github.com/jgromes) for deciphering the CC1101 in their [RadioLib](https://github.com/jgromes/RadioLib)
- The [embassy-rs](https://github.com/embassy-rs/embassy) project for their excellent examples
