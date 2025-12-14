# DACx578 STM32F103 Example

This is an example project demonstrating the use of 8 DAC6578 (10-bit) devices on a custom STM32F103CBTx-based platform. The project uses two I2C buses and showcases a simple sawtooth waveform on all DAC channels.

## Features

- Supports **8 DAC6578 devices** (10-bit resolution) across **2 I2C buses**.
- Uses `embedded_hal_bus` to safely **share the I2C bus** among multiple DACs.
- Generates a **sawtooth waveform** on all DAC channels with a 500 ms interval.
- Compatible with **no_std** Rust environments using `embassy` async executor.

## Hardware Setup

- **Microcontroller:** STM32F103CBTx
- **DACs:** 8 × DAC6578
  - 4 devices on **I2C1**
  - 4 devices on **I2C2**
- I2C bus connections must match the pin assignments in the code:
  - **I2C1:** PB6 (SCL), PB7 (SDA)
  - **I2C2:** PB10 (SCL), PB11 (SDA)
- LEDs for status indication (example uses PB15).

## Software Requirements

- Rust toolchain.
- `embassy-stm32` for async STM32 HAL.
- `dacx578` crate for DAC control.
- `embedded-hal-bus` for shared I2C bus management.

## Usage

1. Initialize the I2C buses and DAC devices.
2. Write a sawtooth waveform to all DAC channels.
3. The DAC output value is incremented every 500 ms and wraps around at 10-bit max (1023).
4. Example logging outputs the DAC value and approximate voltage.

```rust
dac1.write_and_update(Channel::All, value).unwrap();
dac2.write_and_update(Channel::All, value).unwrap();
...
value = value.wrapping_add(1);
if value > 0x03FF {
    value = 0;
}
```

## Notes

1. Each DAC is initialized with a unique I2C address using AddrPin configurations.
2. embedded_hal_bus::util::AtomicCell ensures safe concurrent access to the shared I2C bus.
3. This example uses the async embassy_time::Timer for non-blocking delays.