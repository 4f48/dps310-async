# dps310-async

Universal asynchronous I2C driver in pure Rust for the DPS310 barometric pressure and temperature sensor by Infineon. This is built on top of [embedded-hal-async](https://crates.io/crates/embedded-hal-async) traits to be compatible with a wide range of embedded platforms. This driver is currently in development and lacks many advanced features. It works perfectly with [Adafruit DPS310 breakout](https://www.adafruit.com/product/4494).

## Usage

This example uses the Embassy framework on the RP2350.

```rs
#![no_std]
#![no_main]

use {
    defmt::{debug, error},
    defmt_rtt as _,
    dps310_async::DPS310,
    embassy_executor::Spawner,
    embassy_rp::{bind_interrupts, i2c, peripherals::I2C0},
    embassy_time::{Delay, Timer},
    panic_probe as _,
};

bind_interrupts!(struct Irqs {
   I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let i2c = i2c::I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, Default::default());
    let mut dps310 = DPS310::new(i2c, Delay, 0x77).await.unwrap();

    loop {
        let (prs, tmp) = dps310.pressure_temperature().await.unwrap();
        debug!("{} {}", prs, tmp);
        Timer::after_millis(100).await;
    }
} 
```

## Datasheet
The [Infineon DPS310 datasheet](https://www.mouser.com/datasheet/3/70/1/Infineon-DPS310-DataSheet-v01_02-EN.pdf) contains detailed information about the sensor's features, electrical characteristics, and registers. This crate is implemented based on the datasheet.

## License
dps310-async is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
