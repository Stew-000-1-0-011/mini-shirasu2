//! 浮動小数点版の実装。範囲と飽和の振る舞いは固定小数点版に合わせる

use core::ops::{Add, Mul, Neg, Sub};

/// 最も近い整数に丸める。0.5は0から遠い側。範囲外は飽和
fn round_to_i32(x: f64) -> i32 {
	(if x >= 0.0 { x + 0.5 } else { x - 0.5 }) as i32
}

/// per-unitの信号とゲイン。固定小数点版のQ3.28に相当
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Q3_28(f32);

impl Q3_28 {
	pub const ZERO: Self = Q3_28(0.0);
	pub const ONE: Self = Q3_28(1.0);
	/// 8未満で最大のf32
	pub const MAX: Self = Q3_28(f32::from_bits(0x40FF_FFFF));
	pub const MIN: Self = Q3_28(-8.0);

	fn sat(v: f32) -> Self {
		Q3_28(v.clamp(Self::MIN.0, Self::MAX.0))
	}

	/// NaNはfalse
	fn in_range(v: f32) -> bool {
		v >= Self::MIN.0 && v <= Self::MAX.0
	}

	/// 設定時・テスト用。範囲外やNaNはNone
	pub fn checked_from_f32(v: f32) -> Option<Self> {
		if Self::in_range(v) { Some(Q3_28(v)) } else { None }
	}

	pub fn to_f32(self) -> f32 {
		self.0
	}

	pub fn min(self, rhs: Self) -> Self {
		Q3_28(self.0.min(rhs.0))
	}

	pub fn max(self, rhs: Self) -> Self {
		Q3_28(self.0.max(rhs.0))
	}

	pub fn abs(self) -> Self {
		Self::sat(self.0.abs())
	}

	/// 積を丸めずにQ3_60で返す
	pub fn widening_mul(self, rhs: Self) -> Q3_60 {
		Q3_60::sat(self.0 as f64 * rhs.0 as f64)
	}

	/// 0除算、または結果が範囲に収まらないときNone
	pub fn checked_div(self, rhs: Self) -> Option<Self> {
		if rhs.0 == 0.0 {
			return None;
		}
		let q = self.0 / rhs.0;
		if Self::in_range(q) { Some(Q3_28(q)) } else { None }
	}

	/// 結果が範囲に収まることを呼び出し側が保証する除算。
	/// debugビルドでは違反を検出する。releaseでは検査せず、違反時の結果は不定
	pub fn unchecked_div(self, rhs: Self) -> Self {
		debug_assert!(
			self.checked_div(rhs).is_some(),
			"unchecked_div out of range: {:?} / {:?}", self, rhs
		);
		Q3_28(self.0 / rhs.0)
	}

	pub fn to_q3_60(self) -> Q3_60 {
		Q3_60(self.0 as f64)
	}

	pub fn to_q16_16(self) -> Q16_16 {
		Q16_16(self.0 as f64)
	}

	/// per-unit値に整数を掛け、最も近い整数に丸める。あふれは飽和
	pub fn scale_int(self, n: i32) -> i32 {
		round_to_i32(self.0 as f64 * n as f64)
	}

	/// ゲインに整数を掛ける。あふれは飽和
	pub fn mul_int(self, n: i32) -> Self {
		Self::sat(self.0 * n as f32)
	}
}

impl Add for Q3_28 {
	type Output = Self;
	fn add(self, rhs: Self) -> Self {
		Self::sat(self.0 + rhs.0)
	}
}

impl Sub for Q3_28 {
	type Output = Self;
	fn sub(self, rhs: Self) -> Self {
		Self::sat(self.0 - rhs.0)
	}
}

impl Neg for Q3_28 {
	type Output = Self;
	fn neg(self) -> Self {
		Self::sat(-self.0)
	}
}

impl Mul for Q3_28 {
	type Output = Self;
	fn mul(self, rhs: Self) -> Self {
		Self::sat(self.0 * rhs.0)
	}
}

/// 積分器用。固定小数点版のQ3.60に相当。f32では微小な増分が消えるのでf64で持つ
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Q3_60(f64);

impl Q3_60 {
	pub const ZERO: Self = Q3_60(0.0);
	/// 8未満で最大のf64
	pub const MAX: Self = Q3_60(f64::from_bits(0x401F_FFFF_FFFF_FFFF));
	pub const MIN: Self = Q3_60(-8.0);

	fn sat(v: f64) -> Self {
		Q3_60(v.clamp(Self::MIN.0, Self::MAX.0))
	}

	/// 設定時・テスト用。範囲外やNaNはNone
	pub fn checked_from_f32(v: f32) -> Option<Self> {
		let x = v as f64;
		if x >= Self::MIN.0 && x <= Self::MAX.0 { Some(Q3_60(x)) } else { None }
	}

	pub fn to_f32(self) -> f32 {
		self.0 as f32
	}

	pub fn min(self, rhs: Self) -> Self {
		Q3_60(self.0.min(rhs.0))
	}

	pub fn max(self, rhs: Self) -> Self {
		Q3_60(self.0.max(rhs.0))
	}

	pub fn abs(self) -> Self {
		Self::sat(self.0.abs())
	}

	pub fn to_q3_28(self) -> Q3_28 {
		Q3_28::sat(self.0 as f32)
	}
}

impl Add for Q3_60 {
	type Output = Self;
	fn add(self, rhs: Self) -> Self {
		Self::sat(self.0 + rhs.0)
	}
}

impl Sub for Q3_60 {
	type Output = Self;
	fn sub(self, rhs: Self) -> Self {
		Self::sat(self.0 - rhs.0)
	}
}

impl Neg for Q3_60 {
	type Output = Self;
	fn neg(self) -> Self {
		Self::sat(-self.0)
	}
}

/// 位置用。固定小数点版のQ16.16に相当、1回転 = 1.0。多回転でも分解能を保つためf64で持つ
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Q16_16(f64);

impl Q16_16 {
	pub const ZERO: Self = Q16_16(0.0);
	pub const ONE: Self = Q16_16(1.0);
	/// 32768 - 2^-16
	pub const MAX: Self = Q16_16(32767.9999847412109375);
	pub const MIN: Self = Q16_16(-32768.0);

	fn sat(v: f64) -> Self {
		Q16_16(v.clamp(Self::MIN.0, Self::MAX.0))
	}

	/// 設定時・テスト用。範囲外やNaNはNone
	pub fn checked_from_f32(v: f32) -> Option<Self> {
		let x = v as f64;
		if x >= Self::MIN.0 && x <= Self::MAX.0 { Some(Q16_16(x)) } else { None }
	}

	pub fn to_f32(self) -> f32 {
		self.0 as f32
	}

	pub fn min(self, rhs: Self) -> Self {
		Q16_16(self.0.min(rhs.0))
	}

	pub fn max(self, rhs: Self) -> Self {
		Q16_16(self.0.max(rhs.0))
	}

	pub fn abs(self) -> Self {
		Self::sat(self.0.abs())
	}

	/// Q3_28のゲインを掛けてQ3_28にする
	pub fn mul_q3_28(self, k: Q3_28) -> Q3_28 {
		Q3_28::sat((self.0 * k.0 as f64) as f32)
	}

	pub fn to_q3_28(self) -> Q3_28 {
		Q3_28::sat(self.0 as f32)
	}

	/// 位置に整数を掛け、最も近い整数に丸める。あふれは飽和
	pub fn scale_int(self, n: i32) -> i32 {
		round_to_i32(self.0 * n as f64)
	}

	/// num / den。あふれは飽和。denが0でないことは呼び出し側が保証する。
	/// debugビルドでは違反を検出する。releaseでは0を返す
	pub fn from_ratio(num: i32, den: u32) -> Self {
		debug_assert!(den != 0, "from_ratio: den is zero");
		if den == 0 {
			return Self::ZERO;
		}
		Self::sat(num as f64 / den as f64)
	}
}

impl Add for Q16_16 {
	type Output = Self;
	fn add(self, rhs: Self) -> Self {
		Self::sat(self.0 + rhs.0)
	}
}

impl Sub for Q16_16 {
	type Output = Self;
	fn sub(self, rhs: Self) -> Self {
		Self::sat(self.0 - rhs.0)
	}
}

impl Neg for Q16_16 {
	type Output = Self;
	fn neg(self) -> Self {
		Self::sat(-self.0)
	}
}
