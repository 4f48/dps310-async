// Copyright (C) 2026 Olivér Pirger
// SPDX-License-Identifier: GPL-3.0-or-later

pub(crate) fn sign_extend32(value: i32, sign_bit: u8) -> i32 {
    let shift = 31 - sign_bit;
    (value << shift) >> shift
}
