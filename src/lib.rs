// Copyright (C) 2026 Olivér Pirger
// SPDX-License-Identifier: GPL-3.0-or-later

#![no_std]

mod register;
mod util;

pub mod error;

use crate::{
    error::{Error, Result},
    register::{
        COEF_C0_H, COEF_SRCE, MEAS_CFG, PRODUCT_ID, PRS_B2, PRS_CFG, RESET, TMP_B2, TMP_CFG,
    },
    util::sign_extend32,
};
use embedded_hal_async::{delay::DelayNs, i2c::I2c};

pub struct DPS310<I2C, D> {
    i2c: I2C,
    delay: D,
    address: u8,
    coeffs: CalibrationCoeffs,
    scale_factor: f32,
}

pub struct CalibrationCoeffs {
    c0: i32,
    c1: i32,
    c00: i32,
    c01: i32,
    c10: i32,
    c11: i32,
    c20: i32,
    c21: i32,
    c30: i32,
    tmp_srce: u8,
}

impl Default for CalibrationCoeffs {
    fn default() -> Self {
        Self {
            c0: 0,
            c1: 0,
            c00: 0,
            c01: 0,
            c10: 0,
            c11: 0,
            c20: 0,
            c21: 0,
            c30: 0,
            tmp_srce: 0,
        }
    }
}

impl<I2C, D> DPS310<I2C, D>
where
    I2C: I2c,
    D: DelayNs,
{
    pub async fn new(i2c: I2C, delay: D, address: u8) -> Result<Self> {
        let mut dps310 = Self {
            i2c,
            delay,
            address: address,
            coeffs: Default::default(),
            scale_factor: 524288.0,
        };

        dps310
            .i2c
            .write(dps310.address, &[RESET, 0x89])
            .await
            .map_err(|_| Error::I2CWriteError)?;
        dps310.delay.delay_ms(10).await;

        let mut product_id = [0u8; 1];
        dps310
            .i2c
            .write_read(dps310.address, &[PRODUCT_ID], &mut product_id)
            .await
            .map_err(|_| Error::I2CReadError)?;
        if product_id[0] != 0x10 {
            return Err(Error::NotFound);
        }

        let mut meas_cfg = [0u8; 1];
        for i in 0..50 {
            dps310
                .i2c
                .write_read(dps310.address, &[MEAS_CFG], &mut meas_cfg)
                .await
                .map_err(|_| Error::I2CReadError)?;

            if meas_cfg[0] & (1 << 6) != 0 && meas_cfg[0] & (1 << 7) != 0 {
                break;
            }

            if i == 49 {
                return Err(Error::TimeOut);
            }

            dps310.delay.delay_ms(1).await;
        }

        let mut coeffs = [0u8; 18];
        dps310
            .i2c
            .write_read(dps310.address, &[COEF_C0_H], &mut coeffs)
            .await
            .map_err(|_| Error::I2CReadError)?;

        let c0_raw = ((coeffs[0] as i32) << 4) | ((coeffs[1] as i32) >> 4);
        dps310.coeffs.c0 = sign_extend32(c0_raw, 11);

        let c1_raw = (((coeffs[1] & 0x0F) as i32) << 8) | (coeffs[2] as i32);
        dps310.coeffs.c1 = sign_extend32(c1_raw, 11);

        let c00_raw =
            ((coeffs[3] as i32) << 12) | ((coeffs[4] as i32) << 4) | ((coeffs[5] as i32) >> 4);
        dps310.coeffs.c00 = sign_extend32(c00_raw, 19);

        let c10_raw =
            (((coeffs[5] & 0x0F) as i32) << 16) | ((coeffs[6] as i32) << 8) | (coeffs[7] as i32);
        dps310.coeffs.c10 = sign_extend32(c10_raw, 19);

        let c01_raw = ((coeffs[8] as i32) << 8) | (coeffs[9] as i32);
        dps310.coeffs.c01 = sign_extend32(c01_raw, 15);

        let c11_raw = ((coeffs[10] as i32) << 8) | (coeffs[11] as i32);
        dps310.coeffs.c11 = sign_extend32(c11_raw, 15);

        let c20_raw = ((coeffs[12] as i32) << 8) | (coeffs[13] as i32);
        dps310.coeffs.c20 = sign_extend32(c20_raw, 15);

        let c21_raw = ((coeffs[14] as i32) << 8) | (coeffs[15] as i32);
        dps310.coeffs.c21 = sign_extend32(c21_raw, 15);

        let c30_raw = ((coeffs[16] as i32) << 8) | (coeffs[17] as i32);
        dps310.coeffs.c30 = sign_extend32(c30_raw, 15);

        let mut coef_srce = [0u8; 1];
        dps310
            .i2c
            .write_read(dps310.address, &[COEF_SRCE], &mut coef_srce)
            .await
            .map_err(|_| Error::I2CReadError)?;
        dps310.coeffs.tmp_srce = coef_srce[0] & (1 << 7);

        let tmp_cfg = 0 | dps310.coeffs.tmp_srce;
        dps310
            .i2c
            .write(dps310.address, &[TMP_CFG, tmp_cfg])
            .await
            .map_err(|_| Error::I2CWriteError)?;

        dps310
            .i2c
            .write(dps310.address, &[PRS_CFG, 0])
            .await
            .map_err(|_| Error::I2CWriteError)?;

        Ok(dps310)
    }

    async fn raw_temperature(&mut self) -> Result<i32> {
        self.i2c
            .write(self.address, &[MEAS_CFG, 0x02])
            .await
            .map_err(|_| Error::I2CWriteError)?;

        let mut meas_cfg = [0u8; 1];
        for i in 0..36 {
            self.i2c
                .write_read(self.address, &[MEAS_CFG], &mut meas_cfg)
                .await
                .map_err(|_| Error::I2CReadError)?;

            if meas_cfg[0] & 0x20 != 0 {
                break;
            }

            if i == 35 {
                return Err(Error::TimeOut);
            }

            self.delay.delay_ms(1).await;
        }

        let mut tmp = [0u8; 3];
        self.i2c
            .write_read(self.address, &[TMP_B2], &mut tmp)
            .await
            .map_err(|_| Error::I2CReadError)?;
        Ok(sign_extend32(
            ((tmp[0] as i32) << 16) | ((tmp[1] as i32) << 8) | (tmp[2] as i32),
            23,
        ))
    }

    async fn raw_pressure(&mut self) -> Result<i32> {
        self.i2c
            .write(self.address, &[MEAS_CFG, 0x01])
            .await
            .map_err(|_| Error::I2CWriteError)?;

        let mut meas_cfg = [0u8; 1];
        for i in 0..100 {
            self.i2c
                .write_read(self.address, &[MEAS_CFG], &mut meas_cfg)
                .await
                .map_err(|_| Error::I2CReadError)?;

            if meas_cfg[0] & 0x10 != 0 {
                break;
            }

            if i == 99 {
                return Err(Error::TimeOut);
            }

            self.delay.delay_ms(1).await;
        }

        let mut prs = [0u8; 3];
        self.i2c
            .write_read(self.address, &[PRS_B2], &mut prs)
            .await
            .map_err(|_| Error::I2CReadError)?;
        Ok(sign_extend32(
            ((prs[0] as i32) << 16) | ((prs[1] as i32) << 8) | (prs[2] as i32),
            23,
        ))
    }

    /// Measures temperature in degrees Celsius (°C).
    pub async fn temperature(&mut self) -> Result<f32> {
        let raw = self.raw_temperature().await? as f32;
        let scaled = raw / self.scale_factor;
        Ok((self.coeffs.c0 as f32 * 0.5) + (self.coeffs.c1 as f32 * scaled))
    }

    /// Measures air pressure in hectopascals (hPa).
    pub async fn pressure(&mut self) -> Result<f32> {
        let raw_tmp = self.raw_temperature().await? as f32;
        let raw_prs = self.raw_pressure().await? as f32;

        let scaled_tmp = raw_tmp / self.scale_factor;
        let scaled_prs = raw_prs / self.scale_factor;

        let c00 = self.coeffs.c00 as f32;
        let c10 = self.coeffs.c10 as f32;
        let c20 = self.coeffs.c20 as f32;
        let c30 = self.coeffs.c30 as f32;
        let c01 = self.coeffs.c01 as f32;
        let c11 = self.coeffs.c11 as f32;
        let c21 = self.coeffs.c21 as f32;

        Ok((c00
            + scaled_prs * (c10 + scaled_prs * (c20 + scaled_prs * c30))
            + scaled_tmp * c01
            + scaled_tmp * scaled_prs * (c11 + scaled_prs * c21))
            / 100.0)
    }

    /// Returns both pressure and temperature.
    /// This is more efficient than calling pressure and temperature separately,
    /// because pressure measurement also measures temperature internally.
    pub async fn pressure_temperature(&mut self) -> Result<(f32, f32)> {
        let raw_tmp = self.raw_temperature().await? as f32;
        let raw_prs = self.raw_pressure().await? as f32;

        let scaled_tmp = raw_tmp / self.scale_factor;
        let scaled_prs = raw_prs / self.scale_factor;

        let c00 = self.coeffs.c00 as f32;
        let c10 = self.coeffs.c10 as f32;
        let c20 = self.coeffs.c20 as f32;
        let c30 = self.coeffs.c30 as f32;
        let c01 = self.coeffs.c01 as f32;
        let c11 = self.coeffs.c11 as f32;
        let c21 = self.coeffs.c21 as f32;

        Ok((
            ((c00
                + scaled_prs * (c10 + scaled_prs * (c20 + scaled_prs * c30))
                + scaled_tmp * c01
                + scaled_tmp * scaled_prs * (c11 + scaled_prs * c21))
                / 100.0),
            ((self.coeffs.c0 as f32 * 0.5) + (self.coeffs.c1 as f32 * scaled_tmp)),
        ))
    }
}
