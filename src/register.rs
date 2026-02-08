// Copyright (C) 2026 Olivér Pirger
// SPDX-License-Identifier: GPL-3.0-or-later

#![allow(dead_code)]

pub(crate) const PRS_B2: u8 = 0x00;
pub(crate) const PRS_B1: u8 = 0x01;
pub(crate) const PRS_B0: u8 = 0x02;
pub(crate) const TMP_B2: u8 = 0x03;
pub(crate) const TMP_B1: u8 = 0x04;
pub(crate) const TMP_B0: u8 = 0x05;

pub(crate) const PRS_CFG: u8 = 0x06;
pub(crate) const TMP_CFG: u8 = 0x07;
pub(crate) const MEAS_CFG: u8 = 0x08;
pub(crate) const CFG_REG: u8 = 0x09;

pub(crate) const INT_STS: u8 = 0x0A;
pub(crate) const FIFO_STS: u8 = 0x0B;

pub(crate) const RESET: u8 = 0x0C;
pub(crate) const PRODUCT_ID: u8 = 0x0D;

pub(crate) const COEF_C0_H: u8 = 0x10;
pub(crate) const COEF_C0_L: u8 = 0x11;
pub(crate) const COEF_C1_H: u8 = 0x12;
pub(crate) const COEF_C1_L: u8 = 0x13;
pub(crate) const COEF_C00_H: u8 = 0x14;
pub(crate) const COEF_C00_M: u8 = 0x15;
pub(crate) const COEF_C00_L: u8 = 0x16;
pub(crate) const COEF_C10_H: u8 = 0x17;
pub(crate) const COEF_C10_M: u8 = 0x18;
pub(crate) const COEF_C10_L: u8 = 0x19;
pub(crate) const COEF_C01_H: u8 = 0x1A;
pub(crate) const COEF_C01_L: u8 = 0x1B;
pub(crate) const COEF_C11_H: u8 = 0x1C;
pub(crate) const COEF_C11_L: u8 = 0x1D;
pub(crate) const COEF_C20_H: u8 = 0x1E;
pub(crate) const COEF_C20_L: u8 = 0x1F;
pub(crate) const COEF_C21_H: u8 = 0x20;
pub(crate) const COEF_C21_L: u8 = 0x21;
pub(crate) const COEF_C30_H: u8 = 0x22;
pub(crate) const COEF_C30_L: u8 = 0x23;
pub(crate) const COEF_SRCE: u8 = 0x28;
