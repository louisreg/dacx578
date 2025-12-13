//! *Texas Instruments DAC5578 / DAC6578 / DAC7578 Driver for Rust Embedded HAL*
//!
//! This crate provides a unified driver for the TI **DACx578** family:
//! - **DAC5578** (8-bit resolution)
//! - **DAC6578** (10-bit resolution)
//! - **DAC7578** (12-bit resolution)
//!
//! The driver is built on top of the Rust
//! [embedded HAL](https://github.com/rust-embedded/embedded-hal)
//! and allows sending commands to any DACx578 device over I2C.
//!
//! ## Origin of this crate
//! This driver is **based on the original open-source DAC5578 driver**:
//! - API documentation: <https://docs.rs/dac5578/>
//! - GitHub repository: <https://github.com/chmanie/dac5578>
//! - Crates.io: <https://crates.io/crates/dac5578>
//!
//! The original project provided a clean and minimal driver for the DAC5578.
//! This crate extends that work by generalizing the implementation to support
//! the full **DACx578 family**, introducing resolution-aware encoding while
//! preserving the original command structure and API philosophy.
//!
//! # Initialization
//! To initialize the driver, create an instance of [`DACx578`] by providing:
//! - an I2C interface,
//! - the I2C device address (configured using the ADDR0 pin),
//! - the device resolution (8, 10, or 12 bits depending on the DAC model).
//!
//! ```
//! # use embedded_hal_mock::i2c::Mock;
//! # use dacx578::*;
//! # let i2c = Mock::new(&[]);
//! let mut dac = DACx578::new(i2c, Address::PinLow, DacResolution::Bits12);
//! ```
//!
//! # Writing to a Channel
//! To set the DAC output value of channel A:
//!
//! ```
//! # use embedded_hal_mock::i2c::{Mock, Transaction};
//! # use dacx578::*;
//! # let i2c = Mock::new(&[
//! #     Transaction::write(0x48, vec![0x30, 0x0F, 0xF0])
//! # ]);
//! let mut dac = DACx578::new(i2c, Address::PinLow, DacResolution::Bits12);
//!
//! // Write a 12-bit value to channel A and update the output
//! dac.write_and_update(Channel::A, 0x0FF0).unwrap();
//! ```
//!
//! The provided value is automatically masked and aligned to match the selected
//! resolution (8, 10, or 12 bits).
//!
//! # I2C Addressing
//! The I2C address is selected via the **ADDR0** pin:
//! - `PinLow`   → `0x48`
//! - `PinHigh`  → `0x4A`
//! - `PinFloat` → `0x4C`
//!
//! # More information
//! - DAC5578 datasheet: <https://www.ti.com/product/DAC5578>
//! - DAC6578 datasheet: <https://www.ti.com/product/DAC6578>
//! - DAC7578 datasheet: <https://www.ti.com/product/DAC7578>
//!

#![cfg_attr(not(test), no_std)]
#![warn(missing_debug_implementations, missing_docs)]

use core::fmt::Debug;
use embedded_hal::i2c::I2c;

/// Supported DAC resolutions.
///
/// This determines how input values are masked and aligned before being sent
/// to the DAC.
#[derive(Debug, Clone, Copy)]
pub enum DacResolution {
    /// 8-bit resolution (DAC5578)
    Bits8,
    /// 10-bit resolution (DAC6578)
    Bits10,
    /// 12-bit resolution (DAC7578)
    Bits12,
}

/// ADDR0 pin configuration defining the I2C base address.
///
/// The address is selected via the physical state of the ADDR0 pin.
#[derive(Debug)]
#[repr(u8)]
pub enum Address {
    /// ADDR0 pin tied to GND
    PinLow = 0x48,
    /// ADDR0 pin tied to VDD
    PinHigh = 0x4A,
    /// ADDR0 pin left floating
    PinFloat = 0x4C,
}

/// DAC output channel selection.
///
/// The DACx578 family provides 8 output channels.
#[derive(Debug)]
#[repr(u8)]
pub enum Channel {
    /// DAC output channel A
    A = 0,
    /// DAC output channel B
    B = 1,
    /// DAC output channel C
    C = 2,
    /// DAC output channel D
    D = 3,
    /// DAC output channel E
    E = 4,
    /// DAC output channel F
    F = 5,
    /// DAC output channel G
    G = 6,
    /// DAC output channel H
    H = 7,
    /// Target all DAC channels
    All = 0x0F,
}

impl From<u8> for Channel {
    /// Convert a numeric channel index (0–7) into a [`Channel`].
    ///
    /// # Panics
    /// Panics if the index is outside the valid range.
    fn from(index: u8) -> Self {
        match index {
            0 => Channel::A,
            1 => Channel::B,
            2 => Channel::C,
            3 => Channel::D,
            4 => Channel::E,
            5 => Channel::F,
            6 => Channel::G,
            7 => Channel::H,
            _ => panic!("Invalid channel index {}", index),
        }
    }
}

/// DAC command type.
///
/// These values map directly to the DACx578 command format.
#[derive(Debug)]
#[repr(u8)]
pub enum CommandType {
    /// Write to the DAC input register only
    WriteToChannel = 0x00,
    /// Update the DAC output register only
    UpdateChannel = 0x10,
    /// Write to input register and update the output
    WriteAndUpdate = 0x30,
    /// Write to input register and update all outputs (global LDAC)
    WriteAndUpdateAll = 0x20,
}

/// Software reset mode.
#[derive(Debug)]
#[repr(u8)]
pub enum ResetMode {
    /// Power-on reset (default behavior)
    Por = 0b00,
    /// Reset and force high-speed mode
    SetHighSpeed = 0b01,
    /// Reset while preserving high-speed mode
    MaintainHighSpeed = 0b10,
}

/// Universal DACx578 driver.
///
/// This struct wraps an I2C interface and provides high-level access
/// to the DAC5578, DAC6578, and DAC7578 devices.
#[derive(Debug)]
pub struct DACx578<I2C>
where
    I2C: I2c,
{
    i2c: I2C,
    resolution: DacResolution,
    address: u8,
}

impl<I2C> DACx578<I2C>
where
    I2C: I2c,
{
    /// Create a new DACx578 driver instance.
    ///
    /// # Arguments
    /// * `i2c` – Initialized I2C peripheral
    /// * `address` – I2C address selected via ADDR0 pin
    /// * `resolution` – DAC resolution (8, 10, or 12 bits)
    pub fn new(i2c: I2C, address: Address, resolution: DacResolution) -> Self {
        DACx578 {
            i2c,
            resolution,
            address: address as u8,
        }
    }

    /// Write a value to the DAC input register.
    ///
    /// This does **not** update the analog output until an update command
    /// is issued.
    ///
    /// The input value is automatically masked to the selected resolution.
    pub fn write(&mut self, channel: Channel, value: u16) -> Result<(), I2C::Error> {
        let bytes = self.encode(CommandType::WriteAndUpdate, channel, value);
        self.i2c.write(self.address, &bytes)?;
        Ok(())
    }

    /// Update the DAC output register.
    ///
    /// This command transfers the value from the input register
    /// to the analog output.
    pub fn update(&mut self, channel: Channel, value: u16) -> Result<(), I2C::Error> {
        let bytes = self.encode(CommandType::UpdateChannel, channel, value);
        self.i2c.write(self.address, &bytes)?;
        Ok(())
    }

    /// Write to the input register and immediately update the output.
    ///
    /// This is the most commonly used operation.
    pub fn write_and_update(&mut self, channel: Channel, value: u16) -> Result<(), I2C::Error> {
        let bytes = self.encode(CommandType::WriteAndUpdate, channel, value);
        self.i2c.write(self.address, &bytes)?;
        Ok(())
    }

    /// Write to the input register and update all DAC outputs.
    ///
    /// This acts as a software global LDAC.
    pub fn write_and_update_all(&mut self, channel: Channel, value: u16) -> Result<(), I2C::Error> {
        let bytes = self.encode(CommandType::WriteAndUpdateAll, channel, value);
        self.i2c.write(self.address, &bytes)?;
        Ok(())
    }

    /// Perform a software reset of the DAC.
    ///
    /// The reset behavior depends on the selected [`ResetMode`].
    pub fn reset(&mut self, mode: ResetMode) -> Result<(), I2C::Error> {
        let bytes = [0x70, mode as u8, 0];
        self.i2c.write(self.address, &bytes)?;
        Ok(())
    }

    /// Send a general-call wake-up command.
    ///
    /// ⚠️ This command affects **all devices** on the I2C bus
    /// that support the general-call address.
    pub fn wake_up_all(&mut self) -> Result<(), I2C::Error> {
        self.i2c.write(0x00, &[0x06])?;
        Ok(())
    }

    /// Send a general-call reset command.
    ///
    /// ⚠️ This command resets **all devices** on the I2C bus
    /// that support the general-call address.
    pub fn reset_all(&mut self) -> Result<(),  I2C::Error> {
        self.i2c.write(0x00, &[0x09])?;
        Ok(())
    }

    /// Destroy the driver and return the wrapped I2C interface.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Encode a DAC command according to the selected resolution.
    ///
    /// This method handles value masking, bit alignment, and byte formatting
    /// according to the DACx578 datasheet.
    fn encode(&self, cmd: CommandType, channel: Channel, value: u16) -> [u8; 3] {
        let access = channel as u8;

        // Clamp to resolution
        let clamped = match self.resolution {
            DacResolution::Bits8 => value.min(0xFF),
            DacResolution::Bits10 => value.min(0x03FF),
            DacResolution::Bits12 => value.min(0x0FFF),
        };

        // Encode to two bytes
        let v = match self.resolution {
            DacResolution::Bits8  => clamped << 8,
            DacResolution::Bits10 => clamped << 6,
            DacResolution::Bits12 => clamped << 4,
        };

        let msb = (v >> 8) as u8;
        let lsb = (v & 0xFF) as u8;

        [cmd as u8 | access, msb, lsb]
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    #[test]
    fn write_8bit_encoding() {
        let expectations = [
            // 8-bit max value 0xFF → << 8 → MSB=0xFF, LSB=0x00
            I2cTransaction::write(0x48, vec![0x30, 0xFF, 0x00]),
        ];

        let i2c = I2cMock::new(&expectations);
        let mut dac = DACx578::new(i2c, Address::PinLow, DacResolution::Bits8);

        dac.write_and_update(Channel::A, 0xFF).unwrap();
        dac.destroy().done();
    }

    #[test]
    fn write_10bit_encoding() {
        let expectations = [
            // 10-bit max value 0x3FF → << 6 → MSB=0xFF, LSB=0xC0
            I2cTransaction::write(0x4A, vec![0x31, 0xFF, 0xC0]),
        ];

        let i2c = I2cMock::new(&expectations);
        let mut dac = DACx578::new(i2c, Address::PinHigh, DacResolution::Bits10);

        dac.write_and_update(Channel::B, 0x03FF).unwrap();
        dac.destroy().done();
    }

    #[test]
    fn write_12bit_encoding() {
        let expectations = [
            // 12-bit value 0x0FF0 → << 4 → MSB=0xFF, LSB=0x00
            I2cTransaction::write(0x4C, vec![0x32, 0xFF, 0x00]),
        ];

        let i2c = I2cMock::new(&expectations);
        let mut dac = DACx578::new(i2c, Address::PinFloat, DacResolution::Bits12);

        dac.write_and_update(Channel::C, 0x0FF0).unwrap();
        dac.destroy().done();
    }

    #[test]
    fn value_is_clamped_to_resolution() {
        let expectations = [
            // 0x1FF clamped to 8-bit max 0xFF → << 8 → MSB=0xFF, LSB=0x00
            I2cTransaction::write(0x48, vec![0x30, 0xFF, 0x00]),
        ];

        let i2c = I2cMock::new(&expectations);
        let mut dac = DACx578::new(i2c, Address::PinLow, DacResolution::Bits8);

        dac.write_and_update(Channel::A, 0x1FF).unwrap();
        dac.destroy().done();
    }

    #[test]
    fn global_update_all_channels() {
        let expectations = [
            // 12-bit value 0x0800 → << 4 → MSB=0x08, LSB=0x00
            I2cTransaction::write(0x48, vec![0x2F, 0x80, 0x00]),
        ];

        let i2c = I2cMock::new(&expectations);
        let mut dac = DACx578::new(i2c, Address::PinLow, DacResolution::Bits12);

        dac.write_and_update_all(Channel::All, 0x0800).unwrap();
        dac.destroy().done();
    }

    #[test]
    fn software_reset_command() {
        let expectations = [
            I2cTransaction::write(0x48, vec![0x70, 0x01, 0x00]),
        ];

        let i2c = I2cMock::new(&expectations);
        let mut dac = DACx578::new(i2c, Address::PinLow, DacResolution::Bits12);

        dac.reset(ResetMode::SetHighSpeed).unwrap();
        dac.destroy().done();
    }
}
