//! 制御計算用の数値型。
//!
//! - `Q3_28`: per-unitの信号とゲイン。範囲は[-8, 8)
//! - `Q3_60`: 積分器。範囲は[-8, 8)で、Q3_28同士の積を丸めずに持てる
//! - `Q16_16`: 位置。1回転 = 1.0、範囲は[-32768, 32768)
//! - `Divisor`: 毎周期の割り算を掛け算で済ませるための、除数とその逆数
//!
//! 既定は固定小数点で、feature `f32` を有効にすると中身が浮動小数点になる。
//! どちらでもAPIと飽和の振る舞いは同じ。
#![allow(non_camel_case_types)]

#[cfg(not(feature = "f32"))]
mod int;
#[cfg(not(feature = "f32"))]
pub use int::{Divisor, Q3_28, Q3_60, Q16_16};

#[cfg(feature = "f32")]
mod float;
#[cfg(feature = "f32")]
pub use float::{Divisor, Q3_28, Q3_60, Q16_16};

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
	fn widening_mul_at_exact_boundary() {
		// 2 * 4 = 8 は Q3.60 に入らない(MAXは8未満)。-2 * 4 = -8 はちょうど入る
		assert_eq!(q(2.0).widening_mul(q(4.0)), Q3_60::MAX);
		assert_eq!(q(-2.0).widening_mul(q(4.0)), Q3_60::MIN);
	}

	#[test]
	fn q3_28_checked_div_by_negative_divisor() {
		assert_close(q(3.0).checked_div(q(-4.0)).unwrap().to_f32(), -0.75);
	}

	#[test]
	fn q3_28_converts_to_other_formats() {
		assert_close(q(1.25).to_q3_60().to_f32(), 1.25);
		assert_close(q(-1.5).to_q16_16().to_f32(), -1.5);
	}

	// ---- Divisor ----

	#[test]
	fn divisor_keeps_its_value() {
		assert_eq!(Divisor::new(q(0.125)).value(), q(0.125));
		assert_eq!(Divisor::new(Q3_28::ZERO).value(), Q3_28::ZERO);
	}

	#[test]
	fn div_by_matches_division() {
		assert_close(q(0.25).div_by(Divisor::new(q(0.5))).to_f32(), 0.5);
		assert_close(q(-0.125).div_by(Divisor::new(q(0.5))).to_f32(), -0.25);
		assert_close(Q3_28::ZERO.div_by(Divisor::new(q(0.5))).to_f32(), 0.0);
		// 2進で割り切れない組み合わせ
		assert_close(q(0.1).div_by(Divisor::new(q(0.3))).to_f32(), 1.0 / 3.0);
		assert_close(q(-2.0).div_by(Divisor::new(q(7.0))).to_f32(), -2.0 / 7.0);
	}

	#[test]
	fn div_by_works_when_reciprocal_exceeds_q3_28_range() {
		// 1 / 0.05 = 20 はQ3.28に入らないが、商は(-1, 1)に収まる
		let d = Divisor::new(q(0.05));
		assert_close(q(0.01).div_by(d).to_f32(), 0.2);
		assert_close(q(-0.04).div_by(d).to_f32(), -0.8);
		// 分解能に近い小さな除数
		let d = Divisor::new(q(1e-6));
		assert_close(q(5e-7).div_by(d).to_f32(), q(5e-7).to_f32() / q(1e-6).to_f32());
	}

	#[test]
	fn div_by_is_odd_symmetric() {
		let d = Divisor::new(q(0.3));
		for x in [0.01, 0.1, 0.123456, 0.29] {
			assert_eq!(q(-x).div_by(d), -q(x).div_by(d), "x: {x}");
		}
	}

	#[test]
	#[cfg(debug_assertions)]
	#[should_panic]
	fn div_by_out_of_range_panics_in_debug() {
		let _ = q(0.5).div_by(Divisor::new(q(0.25)));
	}

	// ---- Q3_60 ----

	#[test]
	fn q3_60_roundtrips_f32() {
		assert_close(q60(1.25).to_f32(), 1.25);
		assert_close(Q3_60::ZERO.to_f32(), 0.0);
		assert!(Q3_60::checked_from_f32(8.0).is_none());
		assert_eq!(Q3_60::checked_from_f32(-8.0), Some(Q3_60::MIN));
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

	// ---- 整数との境界 ----

	#[test]
	fn q3_28_scale_int_rounds_to_nearest() {
		assert_eq!(q(0.5).scale_int(1800), 900);
		assert_eq!(q(-0.25).scale_int(1800), -450);
		assert_eq!(Q3_28::ZERO.scale_int(1800), 0);
		assert_eq!(Q3_28::ONE.scale_int(1800), 1800);
		// 0.3は2進で表せないが、丸めれば3になる
		assert_eq!(q(0.3).scale_int(10), 3);
		assert_eq!(q(-0.3).scale_int(10), -3);
		// 2.6 -> 3、-2.6 -> -3
		assert_eq!(q(0.26).scale_int(10), 3);
		assert_eq!(q(-0.26).scale_int(10), -3);
	}

	#[test]
	fn q3_28_scale_int_saturates() {
		assert_eq!(q(4.0).scale_int(i32::MAX), i32::MAX);
		assert_eq!(q(-4.0).scale_int(i32::MAX), i32::MIN);
	}

	#[test]
	fn q3_28_mul_int() {
		assert_close(q(0.125).mul_int(3).to_f32(), 0.375);
		assert_close(q(0.001).mul_int(100).to_f32(), 0.1);
		assert_close(q(0.001).mul_int(-100).to_f32(), -0.1);
		assert_eq!(q(0.5).mul_int(0), Q3_28::ZERO);
	}

	#[test]
	fn q3_28_mul_int_saturates() {
		assert_eq!(Q3_28::ONE.mul_int(100), Q3_28::MAX);
		assert_eq!(Q3_28::ONE.mul_int(-100), Q3_28::MIN);
	}

	#[test]
	fn q16_16_scale_int_rounds_to_nearest() {
		// 65536倍するとQ16.16の生の値になる
		assert_eq!(q16(1.5).scale_int(65536), 98304);
		assert_eq!(q16(-0.25).scale_int(65536), -16384);
		// 7.5 -> 8、-7.5 -> -8
		assert_eq!(q16(2.5).scale_int(3), 8);
		assert_eq!(q16(-2.5).scale_int(3), -8);
	}

	#[test]
	fn q16_16_scale_int_saturates() {
		assert_eq!(q16(30000.0).scale_int(i32::MAX), i32::MAX);
		assert_eq!(q16(-30000.0).scale_int(i32::MAX), i32::MIN);
	}

	#[test]
	fn q16_16_from_ratio() {
		assert_close(Q16_16::from_ratio(1, 4).to_f32(), 0.25);
		assert_close(Q16_16::from_ratio(26624, 8192).to_f32(), 3.25);
		assert_close(Q16_16::from_ratio(-2048, 8192).to_f32(), -0.25);
		assert_eq!(Q16_16::from_ratio(0, 8192), Q16_16::ZERO);
	}

	#[test]
	fn q16_16_from_ratio_saturates() {
		assert_eq!(Q16_16::from_ratio(i32::MAX, 1), Q16_16::MAX);
		assert_eq!(Q16_16::from_ratio(i32::MIN, 1), Q16_16::MIN);
	}

	#[test]
	fn q16_16_from_ratio_roundtrips_raw_value() {
		// 通信ではQ16.16の生の値をやり取りする
		assert_eq!(Q16_16::from_ratio(98304, 65536).scale_int(65536), 98304);
		assert_eq!(Q16_16::from_ratio(-12345, 65536).scale_int(65536), -12345);
	}

	#[test]
	#[cfg(debug_assertions)]
	#[should_panic]
	fn q16_16_from_ratio_zero_denominator_panics_in_debug() {
		let _ = Q16_16::from_ratio(1, 0);
	}
}
