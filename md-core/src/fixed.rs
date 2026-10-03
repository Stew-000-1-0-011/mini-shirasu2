//! 制御計算用の数値型。
//!
//! - `Q3_28`: per-unitの信号とゲイン。範囲は[-8, 8)
//! - `Q3_60`: 積分器。範囲は[-8, 8)で、Q3_28同士の積を丸めずに持てる
//! - `Q16_16`: 位置。1回転 = 1.0、範囲は[-32768, 32768)
//!
//! 既定は固定小数点で、feature `f32` を有効にすると中身が浮動小数点になる。
//! どちらでもAPIと飽和の振る舞いは同じ。
#![allow(non_camel_case_types)]

#[cfg(not(feature = "f32"))]
mod int;
#[cfg(not(feature = "f32"))]
pub use int::{Q3_28, Q3_60, Q16_16};

#[cfg(feature = "f32")]
mod float;
#[cfg(feature = "f32")]
pub use float::{Q3_28, Q3_60, Q16_16};

#[cfg(test)]
mod tests {
	use super::*;

	fn q(v: f32) -> Q3_28 {
		Q3_28::checked_from_f32(v).unwrap()
	}

	fn q60(v: f32) -> Q3_60 {
		Q3_60::checked_from_f32(v).unwrap()
	}

	fn q16(v: f32) -> Q16_16 {
		Q16_16::checked_from_f32(v).unwrap()
	}

	fn assert_close(actual: f32, expected: f32) {
		assert!((actual - expected).abs() < 1e-6, "left: {actual}, right: {expected}");
	}

	// ---- Q3_28 ----

	#[test]
	fn q3_28_roundtrips_f32() {
		assert_close(q(1.5).to_f32(), 1.5);
		assert_close(q(-0.375).to_f32(), -0.375);
		assert_close(Q3_28::ZERO.to_f32(), 0.0);
		assert_close(Q3_28::ONE.to_f32(), 1.0);
	}

	#[test]
	fn q3_28_from_f32_rejects_out_of_range() {
		assert!(Q3_28::checked_from_f32(8.0).is_none());
		assert!(Q3_28::checked_from_f32(-8.5).is_none());
		assert!(Q3_28::checked_from_f32(f32::NAN).is_none());
		assert!(Q3_28::checked_from_f32(f32::INFINITY).is_none());
		// 下限ちょうどは表せる
		assert_eq!(Q3_28::checked_from_f32(-8.0), Some(Q3_28::MIN));
	}

	#[test]
	fn q3_28_add_sub() {
		assert_close((q(1.5) + q(2.25)).to_f32(), 3.75);
		assert_close((q(1.5) - q(2.25)).to_f32(), -0.75);
		assert_close((-q(1.5)).to_f32(), -1.5);
	}

	#[test]
	fn q3_28_add_sub_neg_saturate() {
		assert_eq!(Q3_28::MAX + Q3_28::ONE, Q3_28::MAX);
		assert_eq!(Q3_28::MIN - Q3_28::ONE, Q3_28::MIN);
		assert_eq!(q(5.0) + q(5.0), Q3_28::MAX);
		assert_eq!(q(-5.0) - q(5.0), Q3_28::MIN);
		assert_eq!(-Q3_28::MIN, Q3_28::MAX);
	}

	#[test]
	fn q3_28_mul() {
		assert_close((q(1.5) * q(2.0)).to_f32(), 3.0);
		assert_close((q(-0.5) * q(0.25)).to_f32(), -0.125);
	}

	#[test]
	fn q3_28_mul_saturates() {
		assert_eq!(q(4.0) * q(4.0), Q3_28::MAX);
		assert_eq!(q(-4.0) * q(4.0), Q3_28::MIN);
	}

	#[test]
	fn q3_28_min_max_abs() {
		assert_eq!(q(1.0).min(q(2.0)), q(1.0));
		assert_eq!(q(1.0).max(q(2.0)), q(2.0));
		assert_eq!(q(-1.5).abs(), q(1.5));
		assert_eq!(q(1.5).abs(), q(1.5));
		assert_eq!(Q3_28::MIN.abs(), Q3_28::MAX);
	}

	#[test]
	fn q3_28_compares() {
		assert!(q(1.0) < q(2.0));
		assert!(q(-1.0) < Q3_28::ZERO);
		assert!(Q3_28::MIN < Q3_28::MAX);
	}

	#[test]
	fn q3_28_checked_div() {
		assert_close(q(1.0).checked_div(q(4.0)).unwrap().to_f32(), 0.25);
		assert_close(q(-3.0).checked_div(q(4.0)).unwrap().to_f32(), -0.75);
	}

	#[test]
	fn q3_28_checked_div_rejects_zero_and_overflow() {
		assert!(q(1.0).checked_div(Q3_28::ZERO).is_none());
		// 4 / 0.25 = 16 は範囲外
		assert!(q(4.0).checked_div(q(0.25)).is_none());
		// 1 / 0.125 = 8 もちょうど範囲外
		assert!(q(1.0).checked_div(q(0.125)).is_none());
	}

	#[test]
	fn q3_28_unchecked_div() {
		assert_close(q(3.0).unchecked_div(q(4.0)).to_f32(), 0.75);
	}

	#[test]
	#[cfg(debug_assertions)]
	#[should_panic]
	fn q3_28_unchecked_div_by_zero_panics_in_debug() {
		let _ = q(1.0).unchecked_div(Q3_28::ZERO);
	}

	#[test]
	#[cfg(debug_assertions)]
	#[should_panic]
	fn q3_28_unchecked_div_overflow_panics_in_debug() {
		let _ = q(4.0).unchecked_div(q(0.25));
	}

	#[test]
	fn widening_mul_keeps_product_below_q3_28_resolution() {
		// 1e-4 * 1e-5 = 1e-9 は Q3.28 の分解能(約3.7e-9)より小さい
		let p = q(1e-4).widening_mul(q(1e-5));
		assert!((p.to_f32() - 1e-9).abs() < 1e-11, "left: {}", p.to_f32());
	}

	#[test]
	#[cfg(not(feature = "f32"))]
	fn q3_28_mul_drops_product_below_resolution() {
		// 拡大乗算が必要な理由: 通常の乗算ではこの積が0になる
		assert_eq!(q(1e-4) * q(1e-5), Q3_28::ZERO);
	}

	#[test]
	fn widening_mul_saturates() {
		assert_eq!(q(4.0).widening_mul(q(4.0)), Q3_60::MAX);
		assert_eq!(q(-4.0).widening_mul(q(4.0)), Q3_60::MIN);
	}

	#[test]
	fn q3_28_converts_to_other_formats() {
		assert_close(q(1.25).to_q3_60().to_f32(), 1.25);
		assert_close(q(-1.5).to_q16_16().to_f32(), -1.5);
	}

	// ---- Q3_60 ----

	#[test]
	fn q3_60_roundtrips_f32() {
		assert_close(q60(1.25).to_f32(), 1.25);
		assert_close(Q3_60::ZERO.to_f32(), 0.0);
		assert!(Q3_60::checked_from_f32(8.0).is_none());
		assert!(Q3_60::checked_from_f32(f32::NAN).is_none());
	}

	#[test]
	fn q3_60_add_sub_neg() {
		assert_close((q60(1.5) + q60(2.25)).to_f32(), 3.75);
		assert_close((q60(1.5) - q60(2.25)).to_f32(), -0.75);
		assert_close((-q60(1.5)).to_f32(), -1.5);
	}

	#[test]
	fn q3_60_saturates() {
		assert_eq!(Q3_60::MAX + Q3_60::MAX, Q3_60::MAX);
		assert_eq!(Q3_60::MIN - Q3_60::MAX, Q3_60::MIN);
		assert_eq!(-Q3_60::MIN, Q3_60::MAX);
		assert_eq!(Q3_60::MIN.abs(), Q3_60::MAX);
	}

	#[test]
	fn q3_60_min_max() {
		assert_eq!(q60(1.0).min(q60(2.0)), q60(1.0));
		assert_eq!(q60(1.0).max(q60(2.0)), q60(2.0));
	}

	#[test]
	fn q3_60_converts_to_q3_28() {
		assert_close(q60(-2.75).to_q3_28().to_f32(), -2.75);
		assert_eq!(Q3_60::MAX.to_q3_28(), Q3_28::MAX);
		assert_eq!(Q3_60::MIN.to_q3_28(), Q3_28::MIN);
	}

	// ---- Q16_16 ----

	#[test]
	fn q16_16_roundtrips_f32() {
		assert_close(q16(1000.5).to_f32(), 1000.5);
		assert_close(q16(-0.25).to_f32(), -0.25);
		assert_close(Q16_16::ONE.to_f32(), 1.0);
	}

	#[test]
	fn q16_16_from_f32_rejects_out_of_range() {
		assert!(Q16_16::checked_from_f32(40000.0).is_none());
		assert!(Q16_16::checked_from_f32(32768.0).is_none());
		assert!(Q16_16::checked_from_f32(f32::NAN).is_none());
		assert_eq!(Q16_16::checked_from_f32(-32768.0), Some(Q16_16::MIN));
	}

	#[test]
	fn q16_16_add_sub_neg() {
		assert_close((q16(100.5) + q16(2.25)).to_f32(), 102.75);
		assert_close((q16(100.5) - q16(200.0)).to_f32(), -99.5);
		assert_close((-q16(3.5)).to_f32(), -3.5);
	}

	#[test]
	fn q16_16_saturates() {
		assert_eq!(Q16_16::MAX + Q16_16::ONE, Q16_16::MAX);
		assert_eq!(Q16_16::MIN - Q16_16::ONE, Q16_16::MIN);
		assert_eq!(-Q16_16::MIN, Q16_16::MAX);
		assert_eq!(Q16_16::MIN.abs(), Q16_16::MAX);
	}

	#[test]
	fn q16_16_min_max() {
		assert_eq!(q16(1.0).min(q16(2.0)), q16(1.0));
		assert_eq!(q16(1.0).max(q16(2.0)), q16(2.0));
	}

	#[test]
	fn q16_16_mul_q3_28() {
		// 100回転 * 0.0625 = 6.25。先にQ3_28へ変換すると100は入らない
		assert_close(q16(100.0).mul_q3_28(q(0.0625)).to_f32(), 6.25);
		assert_close(q16(-2.0).mul_q3_28(q(0.5)).to_f32(), -1.0);
	}

	#[test]
	fn q16_16_mul_q3_28_saturates() {
		assert_eq!(q16(1000.0).mul_q3_28(q(1.0)), Q3_28::MAX);
		assert_eq!(q16(-1000.0).mul_q3_28(q(1.0)), Q3_28::MIN);
	}

	#[test]
	fn q16_16_converts_to_q3_28() {
		assert_close(q16(1.5).to_q3_28().to_f32(), 1.5);
		assert_eq!(q16(100.0).to_q3_28(), Q3_28::MAX);
		assert_eq!(q16(-100.0).to_q3_28(), Q3_28::MIN);
	}
}
