#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
use fmt::info;
use embassy_stm32::i2c::{Error, I2c};

use dacx578::{DACx578, Address, Channel, DacResolution};



#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
    use embassy_stm32::rcc::*;
    config.rcc.hse = Some(Hse {
        freq: Hertz(16_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll = Some(Pll {
        src: PllSource::HSE,
        prediv: PllPreDiv::DIV2,
        mul: PllMul::MUL9, // 16Mhz / 2 * 9  = 72Mhz
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV1;
    config.rcc.sys = Sysclk::PLL1_P;
    }

    let p = embassy_stm32::init(config);

    let mut i2c1 = I2c::new_blocking(p.I2C1, p.PB6, p.PB7, Default::default());

    // Create DAC driver 
    let mut dac = DACx578::new(i2c1, Address::PinLow, DacResolution::Bits10);

    let mut value: u16 = 0;

    
    // Init the LEDs, from the HAL.
    let mut led_r = Output::new(p.PB15, Level::Low, Speed::Medium);
    led_r.set_high();
    info!("Hello, World!");

    loop {
        // Write value to channel A
        dac.write_and_update(Channel::A, value).unwrap();

        // Increment value for sawtooth effect
        value = value.wrapping_add(1);
        if value > 0x03FF { // 10-bit max
            value = 0;
        }

        let val_in_float = (value as f32) / 1.23 * 0.512;
        info!("DAC Value: {} (~{} mV)", value, val_in_float);

        // Wait 500 ms (async)
        embassy_time::Timer::after(Duration::from_millis(500)).await;
    }
}

ça a l'air de marcher 
--> TODO plusieurs CHANNELS 
    --> meme bus different dac 
    --> sur diffents bus 
    --> besoin d'update address psq 2 pin et pas une seule (voir datasheet)