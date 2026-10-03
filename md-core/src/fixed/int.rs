//! 固定小数点版の実装

use core::ops::{Add, Mul, Neg, Sub};

/// i64をi32の範囲に飽和させる
const fn sat_i32(v: i64) -> i32 {
	if v > i32::MAX as i64 {
		i32::MAX
	} else if v < i32::MIN as i64 {
		i32::MIN
	} else {
		v as i32
	}
}

/// f32を2^frac倍して最も近い整数にする。有限でない、またはi64に収まらなければNone
fn scale_f32(v: f32, frac: u32) -> Option<i64> {
	if !v.is_finite() {
		return None;
	}
	let x = v as f64 * (1u64 << frac) as f64;
	let r = if x >= 0.0 { x + 0.5 } else { x - 0.5 };
	// `as i64` は飽和するので、範囲外は先に弾く
	if r >= i64::MAX as f64 || r < i64::MIN as f64 {
		return None;
	}
	Some(r as i64)
}

/// per-unitの信号とゲイン。Q3.28
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Q3_28(i32);

impl Q3_28 {
	const FRAC: u32 = 28;

	pub const ZERO: Self = Q3_28(0);
	pub const ONE: Self = Q3_28(1 << Self::FRAC);
	pub const MAX: Self = Q3_28(i32::MAX);
	pub const MIN: Self = Q3_28(i32::MIN);

	/// 設定時・テスト用。範囲外やNaNはNone
	pub fn checked_from_f32(v: f32) -> Option<Self> {
		let r = scale_f32(v, Self::FRAC)?;
		i32::try_from(r).ok().map(Q3_28)
	}

	pub fn to_f32(self) -> f32 {
		self.0 as f32 / (1u32 << Self::FRAC) as f32
	}

	pub fn min(self, rhs: Self) -> Self {
		Q3_28(self.0.min(rhs.0))
	}

	pub fn max(self, rhs: Self) -> Self {
		Q3_28(self.0.max(rhs.0))
	}

	pub fn abs(self) -> Self {
		Q3_28(self.0.saturating_abs())
	}

	/// 積を丸めずにQ3.60で返す
	pub fn widening_mul(self, rhs: Self) -> Q3_60 {
		// 積はQ6.56。Q3.60に揃える4bit左シフトであふれる場合は飽和させる
		let p = self.0 as i64 * rhs.0 as i64;
		if p > i64::MAX >> 4 {
			Q3_60::MAX
		} else if p < i64::MIN >> 4 {
			Q3_60::MIN
		} else {
			Q3_60(p << 4)
		}
	}

	/// 0除算、または結果が範囲に収まらないときNone
	pub fn checked_div(self, rhs: Self) -> Option<Self> {
		let q = ((self.0 as i64) << Self::FRAC).checked_div(rhs.0 as i64)?;
		i32::try_from(q).ok().map(Q3_28)
	}

	/// 結果が範囲に収まることを呼び出し側が保証する除算。
	/// debugビルドでは違反を検出する。releaseでは検査せず、違反時の結果は不定
	pub fn unchecked_div(self, rhs: Self) -> Self {
		debug_assert!(
			self.checked_div(rhs).is_some(),
			"unchecked_div out of range: {:?} / {:?}", self, rhs
		);
		let q = ((self.0 as i64) << Self::FRAC).checked_div(rhs.0 as i64).unwrap_or(0);
		Q3_28(q as i32)
	}

	pub fn to_q3_60(self) -> Q3_60 {
		Q3_60((self.0 as i64) << 32)
	}

	pub fn to_q16_16(self) -> Q16_16 {
		Q16_16(self.0 >> 12)
	}
}

impl Add for Q3_28 {
	type Output = Self;
	fn add(self, rhs: Self) -> Self {
		Q3_28(self.0.saturating_add(rhs.0))
	}
}

impl Sub for Q3_28 {
	type Output = Self;
	fn sub(self, rhs: Self) -> Self {
		Q3_28(self.0.saturating_sub(rhs.0))
	}
}

impl Neg for Q3_28 {
	type Output = Self;
	fn neg(self) -> Self {
		Q3_28(self.0.saturating_neg())
	}
}

impl Mul for Q3_28 {
	type Output = Self;
	fn mul(self, rhs: Self) -> Self {
		Q3_28(sat_i32((self.0 as i64 * rhs.0 as i64) >> Self::FRAC))
	}
}

/// 積分器用。Q3.60
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Q3_60(i64);

impl Q3_60 {
	const FRAC: u32 = 60;

	pub const ZERO: Self = Q3_60(0);
	pub const MAX: Self = Q3_60(i64::MAX);
	pub const MIN: Self = Q3_60(i64::MIN);

	/// 設定時・テスト用。範囲外やNaNはNone
	pub fn checked_from_f32(v: f32) -> Option<Self> {
		scale_f32(v, Self::FRAC).map(Q3_60)
	}

	pub fn to_f32(self) -> f32 {
		(self.0 as f64 / (1u64 << Self::FRAC) as f64) as f32
	}

	pub fn min(self, rhs: Self) -> Self {
		Q3_60(self.0.min(rhs.0))
	}

	pub fn max(self, rhs: Self) -> Self {
		Q3_60(self.0.max(rhs.0))
	}

	pub fn abs(self) -> Self {
		Q3_60(self.0.saturating_abs())
	}

	/// 下位32bitを切り捨ててQ3.28にする。範囲は同じなので飽和しない
	pub fn to_q3_28(self) -> Q3_28 {
		Q3_28((self.0 >> 32) as i32)
	}
}

impl Add for Q3_60 {
	type Output = Self;
	fn add(self, rhs: Self) -> Self {
		Q3_60(self.0.saturating_add(rhs.0))
	}
}

impl Sub for Q3_60 {
	type Output = Self;
	fn sub(self, rhs: Self) -> Self {
		Q3_60(self.0.saturating_sub(rhs.0))
	}
}

impl Neg for Q3_60 {
	type Output = Self;
	fn neg(self) -> Self {
		Q3_60(self.0.saturating_neg())
	}
}

/// 位置用。Q16.16、1回転 = 1.0
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Q16_16(i32);

impl Q16_16 {
	const FRAC: u32 = 16;

	pub const ZERO: Self = Q16_16(0);
	pub const ONE: Self = Q16_16(1 << Self::FRAC);
	pub const MAX: Self = Q16_16(i32::MAX);
	pub const MIN: Self = Q16_16(i32::MIN);

	/// 設定時・テスト用。範囲外やNaNはNone
	pub fn checked_from_f32(v: f32) -> Option<Self> {
		let r = scale_f32(v, Self::FRAC)?;
		i32::try_from(r).ok().map(Q16_16)
	}

	pub fn to_f32(self) -> f32 {
		self.0 as f32 / (1u32 << Self::FRAC) as f32
	}

	pub fn min(self, rhs: Self) -> Self {
		Q16_16(self.0.min(rhs.0))
	}

	pub fn max(self, rhs: Self) -> Self {
		Q16_16(self.0.max(rhs.0))
	}

	pub fn abs(self) -> Self {
		Q16_16(self.0.saturating_abs())
	}

	/// Q3.28のゲインを掛けてQ3.28にする
	pub fn mul_q3_28(self, k: Q3_28) -> Q3_28 {
		// Q16.16 * Q3.28 = Q19.44 -> 16bit右シフトでQ3.28
		Q3_28(sat_i32((self.0 as i64 * k.0 as i64) >> Self::FRAC))
	}

	pub fn to_q3_28(self) -> Q3_28 {
		Q3_28(sat_i32((self.0 as i64) << 12))
	}
}

impl Add for Q16_16 {
	type Output = Self;
	fn add(self, rhs: Self) -> Self {
		Q16_16(self.0.saturating_add(rhs.0))
	}
}

impl Sub for Q16_16 {
	type Output = Self;
	fn sub(self, rhs: Self) -> Self {
		Q16_16(self.0.saturating_sub(rhs.0))
	}
}

impl Neg for Q16_16 {
	type Output = Self;
	fn neg(self) -> Self {
		Q16_16(self.0.saturating_neg())
	}
}
