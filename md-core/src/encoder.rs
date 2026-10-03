//! エンコーダの差分カウントから位置と速度を求める

use crate::controller::ConfigError;
use crate::fixed::{Q3_28, Q16_16};

pub struct EncoderParam {
	counts_per_rev: u32,
	reversed: bool,
	w_gain: Q3_28,  // 1周期に1カウント進んだときの速度[pu]
	alpha: Q3_28,  // 速度の1次ローパスの係数
}

impl EncoderParam {
	/// counts_per_rev: 1回転あたりのカウント数(逓倍後)。period: 更新周期[s]。
	/// wbase: 速度の基準値[rad/s]。alpha: 速度の1次ローパスの係数。(0, 1]で、1ならフィルタなし
	pub fn new(
		counts_per_rev: u32,
		reversed: bool,
		period: f32,
		wbase: f32,
		alpha: f32,
	) -> Result<EncoderParam, ConfigError> {
		if counts_per_rev == 0 {
			return Err(ConfigError::NotPositive("encoder_cpr"));
		}
		if !(period > 0.0 && period.is_finite()) {
			return Err(ConfigError::NotPositive("wperiod"));
		}
		if !(wbase > 0.0 && wbase.is_finite()) {
			return Err(ConfigError::NotPositive("wbase"));
		}
		// NaNもここで弾く
		if !(alpha > 0.0) {
			return Err(ConfigError::NotPositive("w_filter_alpha"));
		}
		if alpha > 1.0 {
			return Err(ConfigError::OutOfRange("w_filter_alpha"));
		}

		let w_gain = core::f32::consts::TAU / (counts_per_rev as f32 * period * wbase);
		let w_gain = Q3_28::checked_from_f32(w_gain).ok_or(ConfigError::OutOfRange("encoder_cpr"))?;
		// 丸めで0になると、回っていても速度が0に見える
		if w_gain == Q3_28::ZERO {
			return Err(ConfigError::OutOfRange("encoder_cpr"));
		}

		Ok(EncoderParam {
			counts_per_rev,
			reversed,
			w_gain,
			alpha: Q3_28::checked_from_f32(alpha).ok_or(ConfigError::OutOfRange("w_filter_alpha"))?,
		})
	}
}

pub struct EncoderState {
	count: i32,  // 原点からの累積カウント
	w: Q3_28,
}

impl EncoderState {
	pub fn new() -> EncoderState {
		EncoderState { count: 0, w: Q3_28::ZERO }
	}

	/// 前回からの差分カウントを受け取り、(位置[回転], 速度[pu])を返す。
	/// カウンタのビット幅によるラップは呼び出し側で解いておく
	pub fn update(&mut self, p: &EncoderParam, delta: i32) -> (Q16_16, Q3_28) {
		let delta = if p.reversed { delta.saturating_neg() } else { delta };
		self.count = self.count.saturating_add(delta);
		self.w = self.w + p.alpha * (p.w_gain.mul_int(delta) - self.w);
		(Q16_16::from_ratio(self.count, p.counts_per_rev), self.w)
	}

	/// 現在位置を0にする。速度はそのまま
	pub fn set_origin(&mut self) {
		self.count = 0;
	}
}


#[cfg(test)]
mod tests {
	use super::*;
	use core::f32::consts::TAU;

	fn assert_close(actual: f32, expected: f32) {
		assert!((actual - expected).abs() < 1e-6, "left: {actual}, right: {expected}");
	}

	/// 1000カウント/回転、1ms周期、基準速度10回転/s。1カウント/周期が0.1puになる
	fn param(reversed: bool, alpha: f32) -> EncoderParam {
		EncoderParam::new(1000, reversed, 1e-3, TAU * 10.0, alpha).ok().unwrap()
	}

	#[test]
	fn position_accumulates_counts() {
		let p = param(false, 1.0);
		let mut st = EncoderState::new();
		assert_close(st.update(&p, 250).0.to_f32(), 0.25);
		assert_close(st.update(&p, 750).0.to_f32(), 1.0);
		assert_close(st.update(&p, -1500).0.to_f32(), -0.5);
	}

	#[test]
	fn velocity_is_delta_times_gain_without_filter() {
		let p = param(false, 1.0);
		let mut st = EncoderState::new();
		assert_close(st.update(&p, 5).1.to_f32(), 0.5);
		assert_close(st.update(&p, -2).1.to_f32(), -0.2);
		assert_close(st.update(&p, 0).1.to_f32(), 0.0);
	}

	#[test]
	fn reversed_flips_position_and_velocity() {
		let p = param(true, 1.0);
		let mut st = EncoderState::new();
		let (th, w) = st.update(&p, 5);
		// -0.005はQ16.16で表せないので、分解能(1/65536)の範囲で比べる
		assert!((th.to_f32() + 0.005).abs() < 1.0 / 65536.0, "{}", th.to_f32());
		assert_close(w.to_f32(), -0.5);
	}

	#[test]
	fn velocity_filter_is_first_order() {
		let p = param(false, 0.5);
		let mut st = EncoderState::new();
		// 入力は0.4で一定。0.2 -> 0.3 -> 0.35 と近づく
		assert_close(st.update(&p, 4).1.to_f32(), 0.2);
		assert_close(st.update(&p, 4).1.to_f32(), 0.3);
		assert_close(st.update(&p, 4).1.to_f32(), 0.35);
	}

	#[test]
	fn set_origin_clears_position_and_keeps_velocity() {
		let p = param(false, 0.5);
		let mut st = EncoderState::new();
		// 位置は0.004回転、速度は 0.5 * 0.4 = 0.2
		st.update(&p, 4);
		st.set_origin();
		let (th, w) = st.update(&p, 0);
		assert_close(th.to_f32(), 0.0);
		// 速度は0に飛ばず、フィルタに従って 0.2 -> 0.1 と減る
		assert_close(w.to_f32(), 0.1);
	}

	#[test]
	fn position_saturates_instead_of_wrapping() {
		let p = EncoderParam::new(1, false, 1.0, 1000.0, 1.0).ok().unwrap();
		let mut st = EncoderState::new();
		st.update(&p, i32::MAX);
		assert_eq!(st.update(&p, i32::MAX).0, Q16_16::MAX);

		let mut st = EncoderState::new();
		st.update(&p, i32::MIN);
		assert_eq!(st.update(&p, i32::MIN).0, Q16_16::MIN);
	}

	#[test]
	fn new_rejects_zero_counts_per_rev() {
		assert_eq!(
			EncoderParam::new(0, false, 1e-3, 300.0, 1.0).err(),
			Some(ConfigError::NotPositive("encoder_cpr"))
		);
	}

	#[test]
	fn new_rejects_bad_period_and_base() {
		assert_eq!(
			EncoderParam::new(1000, false, 0.0, 300.0, 1.0).err(),
			Some(ConfigError::NotPositive("wperiod"))
		);
		assert_eq!(
			EncoderParam::new(1000, false, 1e-3, f32::NAN, 1.0).err(),
			Some(ConfigError::NotPositive("wbase"))
		);
	}

	#[test]
	fn new_rejects_alpha_outside_zero_to_one() {
		assert_eq!(
			EncoderParam::new(1000, false, 1e-3, 300.0, 0.0).err(),
			Some(ConfigError::NotPositive("w_filter_alpha"))
		);
		assert_eq!(
			EncoderParam::new(1000, false, 1e-3, 300.0, f32::NAN).err(),
			Some(ConfigError::NotPositive("w_filter_alpha"))
		);
		assert_eq!(
			EncoderParam::new(1000, false, 1e-3, 300.0, 1.5).err(),
			Some(ConfigError::OutOfRange("w_filter_alpha"))
		);
		assert!(EncoderParam::new(1000, false, 1e-3, 300.0, 1.0).is_ok());
	}

	#[test]
	fn new_rejects_velocity_gain_out_of_range() {
		// 2π / (1 * 1e-3 * 1) = 6283 はQ3.28に入らない
		assert_eq!(
			EncoderParam::new(1, false, 1e-3, 1.0, 1.0).err(),
			Some(ConfigError::OutOfRange("encoder_cpr"))
		);
	}

	#[test]
	#[cfg(not(feature = "f32"))]  // 浮動小数点版には分解能がなく、小さな係数も0にならない
	fn new_rejects_velocity_gain_that_quantizes_to_zero() {
		// 2π / (4e9 * 1 * 1) はQ3.28の分解能より小さい。速度が常に0になってしまう
		assert_eq!(
			EncoderParam::new(4_000_000_000, false, 1.0, 1.0, 1.0).err(),
			Some(ConfigError::OutOfRange("encoder_cpr"))
		);
	}
}
