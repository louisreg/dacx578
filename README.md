# dacx578

[![Crates.io](https://img.shields.io/crates/v/dacx578.svg)](https://crates.io/crates/dacx578)
[![Docs.rs](https://docs.rs/dacx578/badge.svg)](https://docs.rs/dacx578)
[![License](https://img.shields.io/crates/l/dacx578.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/rust-1.70+-orange.svg)](https://www.rust-lang.org/)

Unified Rust driver for Texas Instruments **DAC5578**, **DAC6578**, and **DAC7578**
digital-to-analog converters.

This crate is built on top of the Rust
[embedded-hal](https://github.com/rust-embedded/embedded-hal) and provides a
single, **resolution-aware** and **`no_std` compatible** API for the entire
DACx578 family.

## Supported devices

| Device  | Resolution |
|-------:|-----------:|
| DAC5578 | 8-bit  |
| DAC6578 | 10-bit |
| DAC7578 | 12-bit |

## Origin

This crate is **based on the original open-source DAC5578 driver**:

- API documentation: <https://docs.rs/dac5578/>
- GitHub repository: <https://github.com/chmanie/dac5578>
- Crates.io: <https://crates.io/crates/dac5578>

The original project provided a clean and minimal driver for the DAC5578.

This codebase extends that work by generalizing the implementation to support the
full **DACx578 family**, adding:
- resolution-aware value encoding,
- support for multiple I²C addressing schemes,
- and improved test coverage,

while preserving the original command structure and API philosophy.

> **Note:** This driver has been **tested on real hardware with the DAC6578**.
> The DAC5578 and DAC7578 are expected to behave identically (aside from
> resolution) based on the datasheets and the original DAC5578 driver, but have
> not been fully tested on hardware.

## Features

- Supports all 8 output channels (A–H)
- Resolution-aware value handling (8 / 10 / 12 bits)
- Write, update, write-and-update, and global update commands
- Software reset and I²C general-call support
- Supports single-pin and dual-pin I²C addressing
- `#![no_std]` compatible
- Compatible with `embedded-hal` 1.0

## Usage

### Create a driver instance

#### Single address pin (legacy packages)

```rust
use dacx578::{DACx578, I2cAddress, AddrPin, DacResolution};
use embedded_hal_mock::eh1::i2c::Mock;

let i2c = Mock::new(&[]);

let mut dac = DACx578::new(
    i2c,
    I2cAddress::SinglePin { addr0: AddrPin::Low },
    DacResolution::Bits12,
);
```

#### Single address pin  (TSSOP-16 (PW) Package)

```rust
use dacx578::{DACx578, I2cAddress, AddrPin, DacResolution};
use embedded_hal_mock::eh1::i2c::Mock;

let i2c = Mock::new(&[]);

let mut dac = DACx578::new(
    i2c,
    I2cAddress::DualPin {
        addr1: AddrPin::High,
        addr0: AddrPin::Low,
    },
    DacResolution::Bits10,
);
```

#### Write to a channel

```rust
use dacx578::Channel;

// Write a value and immediately update channel A
dac.write_and_update(Channel::A, 0x0FFF).unwrap();
```

The provided value is automatically masked and aligned according to the selected DAC resolution.

## I²C Addressing

The DACx578 family supports multiple I²C addressing configurations depending on
the package variant. This driver supports **both single-pin and dual-pin address
selection schemes**.

### Single-pin addressing (ADDR0)

Some DACx578 variants expose only a single address selection pin (**ADDR0**).
In this case, the I²C address is determined solely by the state of this pin.

| ADDR0 state | I²C Address |
|------------:|------------:|
| Low         | `0x48` |
| High        | `0x4A` |
| Float       | `0x4C` |

Use this configuration with:

```rust
I2cAddress::SinglePin { addr0 }
```

This mode matches the behavior of the original DAC5578 driver.

### Dual-pin addressing (ADDR1 + ADDR0)

QFN-24 (RGE) package variants expose **two address pins**: **ADDR1** and **ADDR0**.
This allows up to **8 distinct I²C addresses** on the same bus.

| ADDR1 | ADDR0 | I²C Address |
|------:|------:|------------:|
| Low   | Low   | `0x48` |
| Low   | High  | `0x49` |
| High  | Low   | `0x4A` |
| High  | High  | `0x4B` |
| Float | Low   | `0x4C` |
| Float | High  | `0x4D` |
| Low   | Float | `0x4E` |
| High  | Float | `0x4F` |

⚠️ The combination **ADDR1 = Float** and **ADDR0 = Float** is **not supported by
the hardware**. Attempting to use this configuration will cause a panic.

Use this configuration with:

```rust
I2cAddress::DualPin { addr1, addr0 }
```

### Notes

- The base address for all DACx578 devices is `0x48`
- Address bits are derived directly from the pin configuration, following the
  TI datasheet
- The addressing mode does **not** affect command encoding or resolution handling
- Both modes are fully supported by the same `DACx578` driver API


## Datasheets

* DAC5578: [https://www.ti.com/product/DAC5578](https://www.ti.com/product/DAC5578)
* DAC6578: [https://www.ti.com/product/DAC6578](https://www.ti.com/product/DAC6578)
* DAC7578: [https://www.ti.com/product/DAC7578](https://www.ti.com/product/DAC7578)
  
## Possible Improvements

* Provide an **async** version for non-blocking I2C operations
* Support for the reference voltage (`Vref`) to allow writing values directly in volts
* Add an API to read DAC internal registers or status
* Expand testing to cover DAC5578 and DAC7578 for full compatibility

## License

MIT OR Apache-2.0
