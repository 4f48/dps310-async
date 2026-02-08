// Copyright (C) 2026 Olivér Pirger
// SPDX-License-Identifier: GPL-3.0-or-later

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Something went wrong while reading from the I2C bus.
    I2CReadError,

    /// Something went wrong while writing to the I2C bus.
    I2CWriteError,

    /// Timed out waiting.
    TimeOut,

    /// Sensor not found. Check wiring or I2C address.
    NotFound,
}

pub type Result<T> = core::result::Result<T, Error>;
