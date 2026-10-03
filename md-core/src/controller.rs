// TODO: 上限による制限を上限と下限による制限に変更

use crate::fixed::{Q3_28, Q3_60, Q16_16};
use crate::scalar::Scalar;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Saturated {
	NotSaturated,
	Overflow,
	Underflow,
}

/// 測定値。すべてper-unit
pub struct Measurement {
	pub vdc: Q3_28,
	pub vdc_inv: Q3_28,
	pub i: Q3_28,
	pub w: Q3_28,
	pub th: Q16_16,  // 1回転 = 1.0
}
// TODO: バリデート
pub struct Config {
	pub vdcmax: Scalar,  // Vdcの最大値
	pub ke: Scalar,  // 逆起電力

	pub cperiod: Scalar,  // 電流制御周期
	pub ckp: Scalar,  // 電流P制御
	pub cki: Scalar,  // 電流I制御, 実際には同時に掛けられるTも入れる
	pub dead_duty: Scalar,  // デッドタイムによる誤差デューティー比
	pub i_threshold_inv: Scalar,  // 電流値が[-i_threshold, i_threshold]の間は符号を[-1, 1]に
	pub kb: Scalar,  // アンチワインドアップ, kb = ki / kp。kiにTが入ってるので、これもT倍になる
	pub duty_max: Scalar,  // シャント抵抗に電流を流したり、ブートストラップするための上限(vmaxとminをとられる)
	pub vmax: Scalar,  // 出力電圧上限(duty_maxとminをとられる)
	pub imax: Scalar,  // 目標電流上限(出力が必ずしも超えないとは限らないことに注意！)

	pub wkp: Scalar,  // 速度P制御
	pub wki: Scalar,  // 速度I制御。実際は同時に掛けられるTも入れる
	pub wb: Scalar,  // 速度P項の目標値への重み
	pub wmax: Scalar,  // 目標速度上限

	pub pkp: Scalar,  // 位置P制御
	pub pmax: Scalar,  // 目標位置上限
}

/// 積分器。積を丸めずにQ3.60で積算し、Q3.28の分解能より小さい増分が消えないようにする
struct Integrator(Q3_60);
impl Integrator {
	fn new() -> Integrator {
		Integrator(Q3_60::ZERO)
	}

	/// self += k * e
	fn add_product(&mut self, k: Q3_28, e: Q3_28) {
		self.0 = self.0 + k.widening_mul(e);
	}

	fn value(&self) -> Q3_28 {
		self.0.to_q3_28()
	}

	fn reset(&mut self) {
		self.0 = Q3_60::ZERO;
	}
}

/// デッドタイム補償用の符号。0付近で暴れないよう、|i| < th の間は i/th で線形に鈍らせる
fn soft_sign(i: Q3_28, th: Q3_28) -> Q3_28 {
	if i >= th {
		Q3_28::ONE
	} else if i <= -th {
		-Q3_28::ONE
	} else {
		// |i| < th なので結果は(-1, 1)に収まる
		i.unchecked_div(th)
	}
}

pub struct CurrentParam {
	kp: Q3_28,
	ki: Q3_28,  // 周期Tを掛けた値
	kb: Q3_28,  // アンチワインドアップ。ki(T込み) / kp
	ke: Q3_28,
	dead_duty: Q3_28,
	i_threshold: Q3_28,  // 電流値が[-i_threshold, i_threshold]の間は符号を[-1, 1]に
	duty_max: Q3_28,
	vmax: Q3_28,
	imax: Q3_28,
}

pub struct CurrentState {
	i_sum: Integrator,  // 積分
}
impl CurrentState {
	pub fn new() -> CurrentState {
		CurrentState { i_sum: Integrator::new() }
	}

	/// 電流 -> デューティ比。PI -> 逆起電力補償 -> 電圧制限 + デューティ上限 -> デューティ換算 + デッドタイム補償
	pub fn update(&mut self, p: &CurrentParam, u: Q3_28, m: &Measurement) -> (Q3_28, Saturated) {
		// 目標電流を制限する
		let u_clamped = u.min(p.imax).max(-p.imax);
		let e = u_clamped - m.i;

		// PI
		let v_pi = p.kp * e + self.i_sum.value();
		// 逆起電力補償
		let v1 = v_pi + p.ke * m.w;
		// 最大電圧制限 & デューティ上限
		let vlim = p.vmax.min(p.duty_max * m.vdc);
		let v2 = v1.min(vlim).max(-vlim);

		// アンチワインドアップを入れた積算
		self.i_sum.add_product(p.ki, e);
		self.i_sum.add_product(p.kb, v2 - v1);

		// デューティ換算してデッドタイム補償(のデューティー比)を足す
		let duty1 = v2 * m.vdc_inv + p.dead_duty * soft_sign(m.i, p.i_threshold);

		let duty2 = duty1.min(Q3_28::ONE).max(-Q3_28::ONE);

		let saturate = if u > p.imax || v1 > v2 {
			Saturated::Overflow
		} else if u < -p.imax || v1 < v2 {
			Saturated::Underflow
		} else {
			Saturated::NotSaturated
		};

		(duty2, saturate)
	}

	pub fn reset(&mut self) {
		self.i_sum.reset();
	}
}

pub struct VelocityParam {
	kp: Q3_28,  // P制御
	ki: Q3_28,  // I制御。周期Tを掛けた値
	b: Q3_28,  // P項の目標値への重み
	wmax: Q3_28,  // 目標速度上限
}

pub struct VelocityState {
	w_sum: Integrator,
}
impl VelocityState {
	pub fn new() -> VelocityState {
		VelocityState { w_sum: Integrator::new() }
	}

	/// 速度 -> トルク。2自由度PI: Kp(b*u - w) + Ki∫(u - w)。uは±wmaxに制限する。
	pub fn update(
		&mut self,
		p: &VelocityParam,
		u: Q3_28,
		m: &Measurement,
		last_saturated: Saturated
	) -> Q3_28 {
		// 目標速度制限
		let u_clamped = u.min(p.wmax).max(-p.wmax);

		let w = p.kp * (p.b * u_clamped - m.w) + self.w_sum.value();

		// 条件付き積分
		let e = u_clamped - m.w;
		let is_flow_plus = e > Q3_28::ZERO;
		match (last_saturated, is_flow_plus) {
			(Saturated::Overflow, true) | (Saturated::Underflow, false) => {},
			_ => {
				self.w_sum.add_product(p.ki, e);
			},
		}

		w
	}

	pub fn reset(&mut self) {
		self.w_sum.reset();
	}
}

pub struct PositionParam {
	kp: Q3_28,
	pmax: Q16_16,
}

pub struct PositionState {}
impl PositionState {
	pub fn new() -> PositionState {
		PositionState {}
	}

	/// 位置 -> 速度。対処できる外乱がないのでP制御。uは±pmaxに制限する。
	pub fn update(&self, p: &PositionParam, u: Q16_16, m: &Measurement) -> Q3_28 {
		// 目標位置制限
		let u_clamped = u.min(p.pmax).max(-p.pmax);

		// 誤差が8回転を超えても速度指令が弱まらないよう、ゲインを掛けてからQ3.28にする
		(u_clamped - m.th).mul_q3_28(p.kp)
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	fn q(v: f32) -> Q3_28 {
		Q3_28::checked_from_f32(v).unwrap()
	}

	fn q16(v: f32) -> Q16_16 {
		Q16_16::checked_from_f32(v).unwrap()
	}

	/// 計算過程で丸めが入る値の比較用
	fn assert_close(actual: Q3_28, expected: f32) {
		let a = actual.to_f32();
		assert!((a - expected).abs() < 1e-6, "left: {a}, right: {expected}");
	}

	/// i, w, th 以外は母線0.5pu固定の測定値
	fn meas(i: f32, w: f32, th: f32) -> Measurement {
		Measurement { vdc: q(0.5), vdc_inv: q(2.0), i: q(i), w: q(w), th: q16(th) }
	}

	/// 補償を無効にした素のパラメータ。duty_maxは1、電流制限は効かせない
	fn bare_current_param(kp: f32, ki: f32, kb: f32, vmax: f32) -> CurrentParam {
		CurrentParam {
			kp: q(kp),
			ki: q(ki),
			kb: q(kb),
			ke: Q3_28::ZERO,
			dead_duty: Q3_28::ZERO,
			i_threshold: q(1.0),
			duty_max: q(1.0),
			vmax: q(vmax),
			imax: q(7.0),
		}
	}

	/// kiは周期Tを含んだ値
	fn vparam(kp: f32, ki: f32, b: f32, wmax: f32) -> VelocityParam {
		VelocityParam { kp: q(kp), ki: q(ki), b: q(b), wmax: q(wmax) }
	}

	// ---- Integrator ----

	#[test]
	fn integrator_accumulates_products() {
		let mut it = Integrator::new();
		it.add_product(q(0.5), q(0.25));
		it.add_product(q(0.5), q(0.25));
		assert_close(it.value(), 0.25);
	}

	#[test]
	fn integrator_keeps_increments_below_q3_28_resolution() {
		// 1回の増分は 1e-4 * 1e-5 = 1e-9 で、Q3.28 の分解能(約3.7e-9)より小さい
		let mut it = Integrator::new();
		for _ in 0..1000 {
			it.add_product(q(1e-4), q(1e-5));
		}
		let v = it.value().to_f32();
		assert!((v - 1e-6).abs() < 1e-8, "left: {v}");
	}

	#[test]
	fn integrator_saturates_instead_of_wrapping() {
		let mut it = Integrator::new();
		for _ in 0..4 {
			it.add_product(q(4.0), q(4.0));
		}
		assert_eq!(it.value(), Q3_28::MAX);

		let mut it = Integrator::new();
		for _ in 0..4 {
			it.add_product(q(-4.0), q(4.0));
		}
		assert_eq!(it.value(), Q3_28::MIN);
	}

	#[test]
	fn integrator_reset_clears() {
		let mut it = Integrator::new();
		it.add_product(q(0.5), q(0.5));
		it.reset();
		assert_eq!(it.value(), Q3_28::ZERO);
	}

	// ---- soft_sign ----

	#[test]
	fn soft_sign_is_sign_outside_threshold() {
		assert_eq!(soft_sign(q(0.5), q(0.125)), Q3_28::ONE);
		assert_eq!(soft_sign(q(-0.5), q(0.125)), -Q3_28::ONE);
		// しきい値ちょうども±1
		assert_eq!(soft_sign(q(0.125), q(0.125)), Q3_28::ONE);
		assert_eq!(soft_sign(q(-0.125), q(0.125)), -Q3_28::ONE);
	}

	#[test]
	fn soft_sign_is_linear_inside_threshold() {
		assert_close(soft_sign(q(0.25), q(0.5)), 0.5);
		assert_close(soft_sign(q(-0.125), q(0.5)), -0.25);
		assert_close(soft_sign(Q3_28::ZERO, q(0.5)), 0.0);
	}

	#[test]
	fn soft_sign_with_zero_threshold_does_not_divide() {
		// しきい値がpu変換の丸めで0になっても0除算しない
		assert_eq!(soft_sign(q(0.5), Q3_28::ZERO), Q3_28::ONE);
		assert_eq!(soft_sign(q(-0.5), Q3_28::ZERO), -Q3_28::ONE);
		assert_eq!(soft_sign(Q3_28::ZERO, Q3_28::ZERO), Q3_28::ONE);
	}

	// ---- 電流制御 ----

	#[test]
	fn current_update_converts_voltage_to_duty() {
		let p = bare_current_param(0.5, 0.0, 0.0, 7.0);
		let mut st = CurrentState::new();
		// e = 0.75 - 0.25 = 0.5, v = 0.5*0.5 = 0.25, duty = 0.25/0.5 = 0.5
		let (duty, _) = st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0));
		assert_close(duty, 0.5);
	}

	#[test]
	fn current_update_limits_voltage_by_vmax() {
		let p = bare_current_param(0.5, 0.0, 0.0, 0.125);
		let mut st = CurrentState::new();
		// v = 0.25 だが vmax=0.125 で頭打ち -> duty = 0.125/0.5 = 0.25
		let (duty, _) = st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0));
		assert_close(duty, 0.25);
	}

	#[test]
	fn current_voltage_limit_follows_measured_bus_voltage() {
		// duty_max=0.5。母線が0.5puなら0.25pu、0.25puに下がれば0.125puが上限になる
		let mut p = bare_current_param(4.0, 0.0, 0.0, 7.0);
		p.duty_max = q(0.5);

		let mut st = CurrentState::new();
		let m_high = Measurement { vdc: q(0.5), vdc_inv: q(2.0), i: q(0.0), w: q(0.0), th: q16(0.0) };
		assert_close(st.update(&p, q(0.5), &m_high).0, 0.5);

		let mut st = CurrentState::new();
		let m_low = Measurement { vdc: q(0.25), vdc_inv: q(4.0), i: q(0.0), w: q(0.0), th: q16(0.0) };
		// 母線が下がってもデューティは duty_max で頭打ちのまま
		assert_close(st.update(&p, q(0.5), &m_low).0, 0.5);
	}

	#[test]
	fn current_update_compensates_back_emf() {
		let mut p = bare_current_param(0.5, 0.0, 0.0, 7.0);
		p.ke = q(0.25);
		let mut st = CurrentState::new();
		// v = 0.5*0.5 + 0.25*0.5 = 0.375 -> duty = 0.75
		assert_close(st.update(&p, q(0.75), &meas(0.25, 0.5, 0.0)).0, 0.75);
	}

	#[test]
	fn current_update_adds_dead_time_duty_after_conversion() {
		let mut p = bare_current_param(0.5, 0.0, 0.0, 7.0);
		p.dead_duty = q(0.0625);
		p.i_threshold = q(0.125);
		let mut st = CurrentState::new();
		// duty = 0.5 + 0.0625*soft_sign(0.25) = 0.5625
		assert_close(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).0, 0.5625);
	}

	#[test]
	fn current_dead_time_duty_is_softened_near_zero_current() {
		let mut p = bare_current_param(0.0, 0.0, 0.0, 7.0);
		p.dead_duty = q(0.125);
		p.i_threshold = q(0.5);
		let mut st = CurrentState::new();
		// i=0.25 はしきい値0.5の内側なので 0.125*0.5 = 0.0625
		assert_close(st.update(&p, q(0.25), &meas(0.25, 0.0, 0.0)).0, 0.0625);
		assert_close(st.update(&p, q(-0.25), &meas(-0.25, 0.0, 0.0)).0, -0.0625);
	}

	#[test]
	fn current_duty_is_clamped_to_plus_minus_one() {
		let mut p = bare_current_param(2.0, 0.0, 0.0, 7.0);
		p.dead_duty = q(0.0625);
		p.i_threshold = q(0.125);
		let mut st = CurrentState::new();
		// v = 2*0.5 = 1.0 -> 母線0.5puで頭打ち -> duty 1.0 + 0.0625 -> 1.0
		assert_close(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).0, 1.0);

		let mut st = CurrentState::new();
		// 逆向きも同様に -1.0 - 0.0625 -> -1.0
		assert_close(st.update(&p, q(-0.75), &meas(-0.25, 0.0, 0.0)).0, -1.0);
	}

	#[test]
	fn current_update_clamps_target_current() {
		let mut p = bare_current_param(0.5, 0.0, 0.0, 7.0);
		p.imax = q(0.5);
		let mut st = CurrentState::new();
		// u=2 は imax=0.5 に制限される -> e = 0.25, v = 0.125 -> duty 0.25
		assert_close(st.update(&p, q(2.0), &meas(0.25, 0.0, 0.0)).0, 0.25);

		let mut st = CurrentState::new();
		// e = -0.5-0.25 = -0.75, v = -0.375 -> duty -0.75
		assert_close(st.update(&p, q(-2.0), &meas(0.25, 0.0, 0.0)).0, -0.75);
	}

	#[test]
	fn current_clamped_target_also_applies_to_integral() {
		let mut p = bare_current_param(0.0, 1.0, 0.0, 7.0);
		p.imax = q(0.5);
		let mut st = CurrentState::new();
		st.update(&p, q(2.0), &meas(0.25, 0.0, 0.0));
		// 制限後のu=0.5を使うので i_sum = 1*(0.5-0.25) = 0.25
		assert_close(st.i_sum.value(), 0.25);
	}

	#[test]
	fn current_update_integrates_error() {
		let p = bare_current_param(0.0, 0.5, 0.0, 7.0);
		let mut st = CurrentState::new();
		// kp=0なので1回目の出力は0、積分だけが溜まる
		assert_close(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).0, 0.0);
		// i_sum = 0.5*0.5 = 0.25 -> 2回目は duty = 0.25/0.5 = 0.5
		assert_close(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).0, 0.5);
	}

	#[test]
	fn current_update_unwinds_integral_when_saturated() {
		let p = bare_current_param(0.5, 0.25, 0.5, 0.125);
		let mut st = CurrentState::new();
		st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0));
		// i_sum = ki*e + kb*(v_limited - v_raw) = 0.25*0.5 + 0.5*(0.125-0.25) = 0.0625
		assert_close(st.i_sum.value(), 0.0625);
	}

	#[test]
	fn current_reset_clears_integral() {
		let p = bare_current_param(0.0, 0.5, 0.0, 7.0);
		let mut st = CurrentState::new();
		st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0));
		st.reset();
		assert_close(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).0, 0.0);
	}

	#[test]
	fn current_reports_not_saturated_within_limits() {
		let p = bare_current_param(0.5, 0.0, 0.0, 7.0);
		let mut st = CurrentState::new();
		assert_eq!(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).1, Saturated::NotSaturated);
	}

	#[test]
	fn current_reports_overflow_when_voltage_clipped_high() {
		let p = bare_current_param(0.5, 0.0, 0.0, 0.125);
		let mut st = CurrentState::new();
		// v = 0.25 -> 0.125で頭打ち
		assert_eq!(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).1, Saturated::Overflow);
	}

	#[test]
	fn current_reports_underflow_when_voltage_clipped_low() {
		let p = bare_current_param(0.5, 0.0, 0.0, 0.125);
		let mut st = CurrentState::new();
		// e = -0.75 - 0.25 = -1, v = -0.5 -> -0.125で頭打ち
		assert_eq!(st.update(&p, q(-0.75), &meas(0.25, 0.0, 0.0)).1, Saturated::Underflow);
	}

	#[test]
	fn current_reports_saturation_when_target_current_clamped() {
		let mut p = bare_current_param(0.5, 0.0, 0.0, 7.0);
		p.imax = q(0.5);
		let mut st = CurrentState::new();
		assert_eq!(st.update(&p, q(2.0), &meas(0.25, 0.0, 0.0)).1, Saturated::Overflow);

		let mut st = CurrentState::new();
		assert_eq!(st.update(&p, q(-2.0), &meas(0.25, 0.0, 0.0)).1, Saturated::Underflow);

		let mut st = CurrentState::new();
		assert_eq!(st.update(&p, q(0.375), &meas(0.25, 0.0, 0.0)).1, Saturated::NotSaturated);
	}

	// ---- 速度制御 ----

	#[test]
	fn velocity_update_weights_target_in_p_term() {
		// b=1 なら普通のP制御: 0.5*(0.75 - 0.25) = 0.25
		let p = vparam(0.5, 0.0, 1.0, 7.0);
		let mut st = VelocityState::new();
		assert_close(st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 0.25);

		// b=0.5 なら目標値の寄与が半分: 0.5*(0.375 - 0.25) = 0.0625
		let p = vparam(0.5, 0.0, 0.5, 7.0);
		let mut st = VelocityState::new();
		assert_close(st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 0.0625);
	}

	#[test]
	fn velocity_update_integrates_unweighted_error_scaled_by_ki() {
		// 積分項には重みをかけない(定常偏差を残さないため)
		let p = vparam(0.0, 0.5, 0.5, 7.0);
		let mut st = VelocityState::new();
		assert_close(st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 0.0);
		// w_sum = 0.5*(0.75-0.25) = 0.25
		assert_close(st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 0.25);
	}

	#[test]
	fn velocity_update_clamps_target_speed() {
		let p = vparam(0.5, 0.0, 1.0, 0.5);
		let mut st = VelocityState::new();
		// u=2 は wmax=0.5 に制限される -> 0.5*(0.5-0.25) = 0.125
		assert_close(st.update(&p, q(2.0), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 0.125);

		let mut st = VelocityState::new();
		// 0.5*(-0.5-0.25) = -0.375
		assert_close(st.update(&p, q(-2.0), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), -0.375);
	}

	#[test]
	fn velocity_clamped_target_also_applies_to_integral() {
		let p = vparam(0.0, 1.0, 1.0, 0.5);
		let mut st = VelocityState::new();
		st.update(&p, q(2.0), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated);
		// 制限後のu=0.5を使うので w_sum = 1*(0.5-0.25) = 0.25
		assert_close(st.update(&p, q(2.0), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 0.25);
	}

	#[test]
	fn velocity_reset_clears_integral() {
		let p = vparam(0.0, 1.0, 1.0, 7.0);
		let mut st = VelocityState::new();
		st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated);
		st.reset();
		assert_close(st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 0.0);
	}

	#[test]
	fn velocity_freezes_integral_while_inner_overflow() {
		let p = vparam(1.0, 1.0, 1.0, 7.0);
		let mut st = VelocityState::new();
		// e = 0.5 > 0。上側に飽和中に正方向へ積分を進めると悪化するので止める
		st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::Overflow);
		// 積分が溜まっていなければ2回目もP項だけ: 0.5
		assert_close(st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::Overflow), 0.5);
	}

	#[test]
	fn velocity_freezes_integral_while_inner_underflow() {
		let p = vparam(1.0, 1.0, 1.0, 7.0);
		let mut st = VelocityState::new();
		// e = -1 < 0。下側に飽和中に負方向へ積分を進めると悪化するので止める
		st.update(&p, q(-0.75), &meas(0.0, 0.25, 0.0), Saturated::Underflow);
		assert_close(st.update(&p, q(-0.75), &meas(0.0, 0.25, 0.0), Saturated::Underflow), -1.0);
	}

	#[test]
	fn velocity_keeps_integrating_when_error_escapes_saturation() {
		let p = vparam(1.0, 1.0, 1.0, 7.0);
		let mut st = VelocityState::new();
		// まず正方向に積分を溜める(飽和なし)。w_sum = 0.5
		st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated);
		// 上側に飽和中でも誤差が負なら積分を戻せる必要がある
		// 出力 = (-0.25-0.25) + 0.5 = 0
		assert_close(st.update(&p, q(-0.25), &meas(0.0, 0.25, 0.0), Saturated::Overflow), 0.0);
		// w_sum = 0.5 + (-0.5) = 0 なので、次は P項だけ
		assert_close(st.update(&p, q(-0.25), &meas(0.0, 0.25, 0.0), Saturated::Overflow), -0.5);
	}

	#[test]
	fn velocity_integral_freeze_needs_saturation() {
		// 飽和していなければ、誤差が出力と同符号でも普通に積分する
		let p = vparam(1.0, 1.0, 1.0, 7.0);
		let mut st = VelocityState::new();
		st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated);
		// w_sum = 0.5 -> 0.5 + 0.5 = 1.0
		assert_close(st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 1.0);
	}

	// ---- 位置制御 ----

	#[test]
	fn position_update_is_proportional() {
		let p = PositionParam { kp: q(0.5), pmax: q16(100.0) };
		let st = PositionState::new();
		// 0.5*(2 - 0.5) = 0.75
		assert_close(st.update(&p, q16(2.0), &meas(0.0, 0.0, 0.5)), 0.75);
		assert_close(st.update(&p, q16(0.0), &meas(0.0, 0.0, 0.5)), -0.25);
	}

	#[test]
	fn position_update_clamps_target_position() {
		let p = PositionParam { kp: q(0.5), pmax: q16(1.0) };
		let st = PositionState::new();
		// u=5 は pmax=1 に制限される -> 0.5*(1 - 0.5) = 0.25
		assert_close(st.update(&p, q16(5.0), &meas(0.0, 0.0, 0.5)), 0.25);
		// u=-5 は -1 に制限される -> 0.5*(-1 - 0.5) = -0.75
		assert_close(st.update(&p, q16(-5.0), &meas(0.0, 0.0, 0.5)), -0.75);
	}

	#[test]
	fn position_error_beyond_eight_turns_is_not_clipped_before_gain() {
		// 誤差100回転はQ3_28に入らないが、ゲインを掛けた結果6.25は入る
		let p = PositionParam { kp: q(0.0625), pmax: q16(1000.0) };
		let st = PositionState::new();
		assert_close(st.update(&p, q16(100.0), &meas(0.0, 0.0, 0.0)), 6.25);
	}

	#[test]
	fn position_output_saturates_for_huge_error() {
		let p = PositionParam { kp: q(1.0), pmax: q16(1000.0) };
		let st = PositionState::new();
		assert_eq!(st.update(&p, q16(1000.0), &meas(0.0, 0.0, 0.0)), Q3_28::MAX);
		assert_eq!(st.update(&p, q16(-1000.0), &meas(0.0, 0.0, 0.0)), Q3_28::MIN);
	}
}
