# dacx578

[![Crates.io](https://img.shields.io/crates/v/dacx578.svg)](https://crates.io/crates/dacx578)
[![Docs.rs](https://docs.rs/dacx578/badge.svg)](https://docs.rs/dacx578)
[![License](https://img.shields.io/crates/l/dacx578.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/rust-1.70+-orange.svg)](https://www.rust-lang.org/)

Unified Rust driver for Texas Instruments **DAC5578**, **DAC6578**, and **DAC7578** digital-to-analog converters.

This crate is built on top of the Rust [embedded-hal](https://github.com/rust-embedded/embedded-hal) and provides a single, resolution-aware API for the entire DACx578 family.

## Supported devices

| Device  | Resolution |
| ------- | ---------- |
| DAC5578 | 8-bit      |
| DAC6578 | 10-bit     |
| DAC7578 | 12-bit     |

## Origin

This crate is **based on the original open-source DAC5578 driver**:

* API documentation: [https://docs.rs/dac5578/](https://docs.rs/dac5578/)
* GitHub repository: [https://github.com/chmanie/dac5578](https://github.com/chmanie/dac5578)
* Crates.io: [https://crates.io/crates/dac5578](https://crates.io/crates/dac5578)

The original project provided a clean and minimal driver for the DAC5578.

This codebase extends that work by generalizing the implementation to support the full **DACx578 family** (DAC5578, DAC6578, DAC7578), adding resolution-aware encoding while preserving the original command structure and API philosophy.

## Features

* Supports all 8 output channels (A–H)
* Resolution-aware value handling (8 / 10 / 12 bits)
* Write, update, write-and-update, and global update commands
* Software reset and I2C general-call support
* `#![no_std]` compatible

## Usage

### Create a driver instance

```rust
use dacx578::{DACx578, Address, DacResolution};
use embedded_hal_mock::i2c::Mock;

let i2c = Mock::new(&[]);
let mut dac = DACx578::new(i2c, Address::PinLow, DacResolution::Bits12);
```

### Write to a channel

```rust
use dacx578::Channel;

// Write a 12-bit value and update channel A
dac.write_and_update(Channel::A, 0x0FFF).unwrap();
```

The provided value is automatically masked and aligned according to the selected DAC resolution.

## I2C Addressing

The I2C address is selected via the **ADDR0** pin:

* `PinLow`   → `0x48`
* `PinHigh`  → `0x4A`
* `PinFloat` → `0x4C`

## Datasheets

* DAC5578: [https://www.ti.com/product/DAC5578](https://www.ti.com/product/DAC5578)
* DAC6578: [https://www.ti.com/product/DAC6578](https://www.ti.com/product/DAC6578)
* DAC7578: [https://www.ti.com/product/DAC7578](https://www.ti.com/product/DAC7578)

## License

MIT OR Apache-2.0
