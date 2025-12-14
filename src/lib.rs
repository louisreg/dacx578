//! *Texas Instruments DAC5578 / DAC6578 / DAC7578 Driver for Rust Embedded HAL*
//!
//! This crate provides a unified driver for the TI **DACx578** family:
//! - **DAC5578** (8-bit resolution)
//! - **DAC6578** (10-bit resolution)
//! - **DAC7578** (12-bit resolution)
//!
//! The driver is built on top of the Rust
//! [embedded-hal](https://github.com/rust-embedded/embedded-hal)
//! and provides a resolution-aware, `no_std` compatible API to control
//! any DACx578 device over I²C.
//!
//! ---
//!
//! ## Origin of this crate
//!
//! This driver is **based on the original open-source DAC5578 driver**:
//! - API documentation: <https://docs.rs/dac5578/>
//! - GitHub repository: <https://github.com/chmanie/dac5578>
//! - Crates.io: <https://crates.io/crates/dac5578>
//!
//! The original project provided a clean and minimal driver for the DAC5578.
//! This crate extends that work by generalizing the implementation to support
//! the full **DACx578 family**, introducing resolution-aware encoding and
//! extended I²C addressing support while preserving the original command
//! structure and API philosophy.
//!
//! ---
//!
//! ## Initialization
//!
//! To initialize the driver, create an instance of [`DACx578`] by providing:
//! - an I²C interface implementing [`embedded_hal::i2c::I2c`],
//! - the I²C address configuration derived from the ADDR pins,
//! - the device resolution (8, 10, or 12 bits depending on the DAC model).
//!
//! ### Example (single address pin)
//!
//! ```
//! use dacx578::{DACx578, I2cAddress, AddrPin, DacResolution};
//! use embedded_hal_mock::eh1::i2c::Mock;
//!
//! let i2c = Mock::new(&[]);
//!
//! let mut dac = DACx578::new(
//!     i2c,
//!     I2cAddress::SinglePin { addr0: AddrPin::Low },
//!     DacResolution::Bits12,
//! );
//! ```
//!
//! ---
//!
//! ## Writing to a Channel
//!
//! To set the DAC output value of channel A:
//!
//! ```
//! use dacx578::{DACx578, I2cAddress, AddrPin, DacResolution, Channel};
//! use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
//!
//! let i2c = Mock::new(&[
//!     Transaction::write(0x48, vec![0x30, 0xFF, 0x00]),
//! ]);
//!
//! let mut dac = DACx578::new(
//!     i2c,
//!     I2cAddress::SinglePin { addr0: AddrPin::Low },
//!     DacResolution::Bits12,
//! );
//!
//! // Write a value and update the output
//! dac.write_and_update(Channel::A, 0x0FF0).unwrap();
//! ```
//!
//! The provided value is automatically **clamped and aligned**
//! according to the selected resolution (8, 10, or 12 bits).
//!
//! ---
//!
//! ## I²C Addressing
//!
//! The DACx578 family exists in multiple package variants with different
//! I²C addressing schemes.
//!
//! ### Single-pin addressing (TSSOP-16 (PW) Package)
//!
//! Some devices expose only **ADDR0**:
//!
//! | ADDR0  | I²C Address |
//! |-------:|------------:|
//! | Low    | `0x48` |
//! | High   | `0x4A` |
//! | Float  | `0x4C` |
//!
//! This is represented using [`I2cAddress::SinglePin`].
//!
//! ### Dual-pin addressing (QFN-24 / RGE package)
//!
//! Devices with **ADDR1 + ADDR0** support up to 8 I²C addresses:
//!
//! | ADDR1  | ADDR0  | Address |
//! |-------:|-------:|--------:|
//! | Low    | Low    | `0x48` |
//! | Low    | High   | `0x49` |
//! | High   | Low    | `0x4A` |
//! | High   | High   | `0x4B` |
//! | Float  | Low    | `0x4C` |
//! | Float  | High   | `0x4D` |
//! | Low    | Float  | `0x4E` |
//! | High   | Float  | `0x4F` |
//!
//! The combination `(Float, Float)` is **not supported** by the hardware
//! and will panic if used.
//!
//! ---
//!
//! ## Device Support Status
//!
//! - The driver has been **tested on real hardware with the DAC6578**.
//! - Support for DAC5578 and DAC7578 is based on:
//!   - the shared command set,
//!   - identical data register formats,
//!   - and the original DAC5578 driver.
//!
//! The DAC7578 is functionally almost identical to the DAC6578,
//! differing mainly in resolution.
//!
//! ---
//!
//! ## More information
//!
//! - DAC5578 datasheet: <https://www.ti.com/product/DAC5578>
//! - DAC6578 datasheet: <https://www.ti.com/product/DAC6578>
//! - DAC7578 datasheet: <https://www.ti.com/product/DAC7578>

#![cfg_attr(not(test), no_std)]
#![warn(missing_debug_implementations, missing_docs)]

use core::fmt::Debug;
use embedded_hal::i2c::I2c;

/// Supported DAC resolutions.
///
/// This determines how input values are masked and aligned before being sent
/// to the DAC, according to the selected device:
///
/// - DAC5578 → 8-bit
/// - DAC6578 → 10-bit
/// - DAC7578 → 12-bit
#[derive(Debug, Clone, Copy)]
pub enum DacResolution {
    /// 8-bit resolution (DAC5578)
    Bits8,
    /// 10-bit resolution (DAC6578)
    Bits10,
    /// 12-bit resolution (DAC7578)
    Bits12,
}

/// Logical state of an address selection pin.
///
/// Some DACx578 devices expose one or two address pins which can be tied
/// to logic low, logic high, or left floating.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddrPin {
    /// Pin tied to GND
    Low,
    /// Pin tied to VDD
    High,
    /// Pin left floating
    Float,
}

/// I2C address configuration for DACx578 devices.
///
/// Depending on the package variant, the I2C address is configured using:
///
/// - a single address pin (ADDR0), or
/// - two address pins (ADDR1 + ADDR0).
///
/// This enum abstracts both variants and computes the correct 7-bit I2C
/// address according to the datasheet.
#[derive(Debug, Clone, Copy)]
pub enum I2cAddress {
    /// Legacy packages using a single address pin (ADDR0 only).
    ///
    /// This corresponds to packages where only ADDR0 is bonded out.
    SinglePin {
        /// State of the ADDR0 pin
        addr0: AddrPin,
    },

    /// QFN-24 (RGE) package using two address pins (ADDR1 and ADDR0).
    ///
    /// This allows up to 8 distinct I2C addresses.
    DualPin {
        /// State of the ADDR1 pin
        addr1: AddrPin,
        /// State of the ADDR0 pin
        addr0: AddrPin,
    },
}

impl I2cAddress {
    /// Compute the 7-bit I2C address corresponding to the pin configuration.
    ///
    /// The base address is `0x48` (`0b1001_000`), with the lower bits
    /// determined by the ADDR pin states as defined in the datasheet.
    ///
    /// # Panics
    ///
    /// Panics if both `ADDR1` and `ADDR0` are set to `Float`, which is
    /// explicitly marked as *not supported* by Texas Instruments.
    pub fn to_u8(self) -> u8 {
        let base = 0x48; // 0b1001_000

        let bits = match self {
            I2cAddress::SinglePin { addr0 } => match addr0 {
                AddrPin::Low   => 0b000,
                AddrPin::High  => 0b010,
                AddrPin::Float => 0b100,
            },

            I2cAddress::DualPin { addr1, addr0 } => {
                use AddrPin::*;

                match (addr1, addr0) {
                    (Low,   Low)   => 0b000,
                    (Low,   High)  => 0b001,
                    (High,  Low)   => 0b010,
                    (High,  High)  => 0b011,
                    (Float, Low)   => 0b100,
                    (Float, High)  => 0b101,
                    (Low,   Float) => 0b110,
                    (High,  Float) => 0b111,
                    (Float, Float) =>
                        panic!("ADDR1=Float and ADDR0=Float is not supported"),
                }
            }
        };

        base | bits
    }
}

/// DAC output channel selection.
///
/// The DACx578 family provides 8 independent output channels.
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
    /// Target all DAC channels simultaneously
    All = 0x0F,
}

impl From<u8> for Channel {
    /// Convert a numeric channel index (0–7) into a [`Channel`].
    ///
    /// # Panics
    ///
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
/// These values map directly to the DACx578 command format as defined
/// in the datasheet.
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
    ///
    /// * `i2c` – Initialized I2C peripheral
    /// * `address` – I2C address configuration derived from ADDR pins
    /// * `resolution` – DAC resolution (8, 10, or 12 bits)
    pub fn new(
        i2c: I2C,
        address: I2cAddress,
        resolution: DacResolution,
    ) -> Self {
        Self {
            i2c,
            address: address.to_u8(),
            resolution,
        }
    }

    /// Write a value to the DAC input register.
    ///
    /// This does **not** update the analog output until an update command
    /// is issued.
    ///
    /// The input value is automatically clamped to the selected resolution.
    pub fn write(&mut self, channel: Channel, value: u16) -> Result<(), I2C::Error> {
        let bytes = self.encode(CommandType::WriteAndUpdate, channel, value);
        self.i2c.write(self.address, &bytes)?;
        Ok(())
    }

    /// Update the DAC output register.
    ///
    /// This transfers the value from the input register to the analog output.
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
    pub fn reset_all(&mut self) -> Result<(), I2C::Error> {
        self.i2c.write(0x00, &[0x09])?;
        Ok(())
    }

    /// Destroy the driver and return the wrapped I2C interface.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Encode a DAC command according to the selected resolution.
    ///
    /// This method performs:
    /// - value clamping
    /// - bit alignment
    /// - command and channel encoding
    ///
    /// The resulting byte sequence matches the DACx578 data input register
    /// format.
    fn encode(&self, cmd: CommandType, channel: Channel, value: u16) -> [u8; 3] {
        let access = channel as u8;

        let clamped = match self.resolution {
            DacResolution::Bits8 => value.min(0xFF),
            DacResolution::Bits10 => value.min(0x03FF),
            DacResolution::Bits12 => value.min(0x0FFF),
        };

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
    use embedded_hal_mock::eh1::i2c::{
        Mock as I2cMock,
        Transaction as I2cTransaction,
    };

    /* ---------------------------------------------------------------------
     * Encoding tests (data alignment & clamping)
     * ------------------------------------------------------------------ */

    #[test]
    fn write_8bit_encoding() {
        let expectations = [
            // 8-bit max 0xFF → << 8 → MSB=0xFF, LSB=0x00
            I2cTransaction::write(0x48, vec![0x30, 0xFF, 0x00]),
        ];

        let addr = I2cAddress::SinglePin {
            addr0: AddrPin::Low,
        };

        let i2c = I2cMock::new(&expectations);
        let mut dac = DACx578::new(i2c, addr, DacResolution::Bits8);

        dac.write_and_update(Channel::A, 0xFF).unwrap();
        dac.destroy().done();
    }

    #[test]
    fn write_10bit_encoding() {
        let expectations = [
            // 10-bit max 0x3FF → << 6 → MSB=0xFF, LSB=0xC0
            I2cTransaction::write(0x4A, vec![0x31, 0xFF, 0xC0]),
        ];

        let addr = I2cAddress::SinglePin {
            addr0: AddrPin::High,
        };

        let i2c = I2cMock::new(&expectations);
        let mut dac = DACx578::new(i2c, addr, DacResolution::Bits10);

        dac.write_and_update(Channel::B, 0x03FF).unwrap();
        dac.destroy().done();
    }

    #[test]
    fn write_12bit_encoding() {
        let expectations = [
            // 12-bit value 0x0FF0 → << 4 → MSB=0xFF, LSB=0x00
            I2cTransaction::write(0x4C, vec![0x32, 0xFF, 0x00]),
        ];

        let addr = I2cAddress::SinglePin {
            addr0: AddrPin::Float,
        };

        let i2c = I2cMock::new(&expectations);
        let mut dac = DACx578::new(i2c, addr, DacResolution::Bits12);

        dac.write_and_update(Channel::C, 0x0FF0).unwrap();
        dac.destroy().done();
    }

    #[test]
    fn value_is_clamped_to_resolution() {
        let expectations = [
            // 0x1FF clamped to 0xFF → << 8 → MSB=0xFF, LSB=0x00
            I2cTransaction::write(0x48, vec![0x30, 0xFF, 0x00]),
        ];

        let addr = I2cAddress::SinglePin {
            addr0: AddrPin::Low,
        };

        let i2c = I2cMock::new(&expectations);
        let mut dac = DACx578::new(i2c, addr, DacResolution::Bits8);

        dac.write_and_update(Channel::A, 0x1FF).unwrap();
        dac.destroy().done();
    }

    #[test]
    fn global_update_all_channels() {
        let expectations = [
            // 12-bit value 0x0800 → << 4 → MSB=0x80, LSB=0x00
            I2cTransaction::write(0x48, vec![0x2F, 0x80, 0x00]),
        ];

        let addr = I2cAddress::SinglePin {
            addr0: AddrPin::Low,
        };

        let i2c = I2cMock::new(&expectations);
        let mut dac = DACx578::new(i2c, addr, DacResolution::Bits12);

        dac.write_and_update_all(Channel::All, 0x0800).unwrap();
        dac.destroy().done();
    }

    #[test]
    fn software_reset_command() {
        let expectations = [
            I2cTransaction::write(0x48, vec![0x70, 0x01, 0x00]),
        ];

        let addr = I2cAddress::SinglePin {
            addr0: AddrPin::Low,
        };

        let i2c = I2cMock::new(&expectations);
        let mut dac = DACx578::new(i2c, addr, DacResolution::Bits12);

        dac.reset(ResetMode::SetHighSpeed).unwrap();
        dac.destroy().done();
    }

    /* ---------------------------------------------------------------------
     * Address mapping tests (NEW – very important)
     * ------------------------------------------------------------------ */

    #[test]
    fn address_single_pin_mapping() {
        assert_eq!(
            I2cAddress::SinglePin { addr0: AddrPin::Low }.to_u8(),
            0x48
        );
        assert_eq!(
            I2cAddress::SinglePin { addr0: AddrPin::High }.to_u8(),
            0x4A
        );
        assert_eq!(
            I2cAddress::SinglePin { addr0: AddrPin::Float }.to_u8(),
            0x4C
        );
    }

    #[test]
    fn address_dual_pin_mapping_qfn24() {
        use AddrPin::*;

        let cases = [
            ((Low,   Low),   0x48),
            ((Low,   High),  0x49),
            ((High,  Low),   0x4A),
            ((High,  High),  0x4B),
            ((Float, Low),   0x4C),
            ((Float, High),  0x4D),
            ((Low,   Float), 0x4E),
            ((High,  Float), 0x4F),
        ];

        for ((addr1, addr0), expected) in cases {
            let addr = I2cAddress::DualPin { addr1, addr0 };
            assert_eq!(addr.to_u8(), expected);
        }
    }

    #[test]
    #[should_panic(expected = "not supported")]
    fn address_dual_pin_float_float_is_rejected() {
        let _ = I2cAddress::DualPin {
            addr1: AddrPin::Float,
            addr0: AddrPin::Float,
        }
        .to_u8();
    }
}
