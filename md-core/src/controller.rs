// TODO: 上限による制限を上限と下限による制限に変更

use crate::scalar::Scalar;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Saturated {
	NotSaturated,
	Overflow,
	Underflow,
}

// TODO: vdcが定格を下回った場合にvdcやvdc_invの更新をしないようにする
pub struct Measurement {
	pub vdc: Scalar,
	pub vdc_inv: Scalar,
	pub i: Scalar,
	pub w: Scalar,
	pub th: Scalar,
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

pub struct CurrentParam {
	kp: Scalar,
	ki: Scalar,
	ke: Scalar,
	dead_duty: Scalar,
	i_threshold_inv: Scalar,
	kb: Scalar,
	duty_max: Scalar,
	vmax: Scalar,
	imax: Scalar,
}
impl CurrentParam {
	pub fn new(c: &Config) -> CurrentParam {
		CurrentParam {
			kp: c.ckp,
			ki: c.cki,
			ke: c.ke,
			dead_duty: c.dead_duty,
			i_threshold_inv: c.i_threshold_inv,
			kb: c.kb,
			duty_max: c.duty_max,
			vmax: c.vmax,
			imax: c.imax,
		}
	}
}

pub struct CurrentState {
	i_sum: Scalar,  // 積分
}
impl CurrentState {
	pub fn new() -> CurrentState {
		CurrentState{i_sum: Scalar::zero()}
	}

	/// 電流 -> デューティ比。PI -> 逆起電力補償 -> 電圧制限 + デューティ上限 -> デューティ換算 + デッドタイム補償
	pub fn update(&mut self, p: &CurrentParam, u: Scalar, m: &Measurement) -> (Scalar, Saturated) {
		// 目標電流を制限する
		let u_clamped = u.min(p.imax).max(-p.imax);
		let e = u_clamped - m.i;

		// PI
		let v_pi = p.kp * e + self.i_sum;
		// 逆起電力補償
		let v1 = v_pi + p.ke * m.w;
		// 最大電圧制限 & デューティ上限
		let vlim = p.vmax.min(p.duty_max * m.vdc);
		let v2 = v1.min(vlim).max(-vlim);

		// アンチワインドアップを入れた積算
		self.i_sum += p.ki * e + p.kb * (v2 - v1);

		// デューティ換算してデッドタイム補償(のデューティー比)を足す
		let duty1 = v2 * m.vdc_inv + p.dead_duty * m.i.sat(p.i_threshold_inv);

		let duty2 = duty1.min(Scalar::one()).max(-Scalar::one());

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
		self.i_sum = Scalar::zero();
	}
}

pub struct VelocityParam {
	kp: Scalar,  // P制御
	ki: Scalar,  // I制御。実際は同時に掛けられるTも入れる
	b: Scalar,  // P項の目標値への重み
	wmax: Scalar,  // 目標速度上限
}
impl VelocityParam {
	pub fn new(c: &Config) -> VelocityParam {
		VelocityParam { kp: c.wkp, ki: c.wki, b: c.wb, wmax: c.wmax }
	}
}

pub struct VelocityState {
	w_sum: Scalar,
}
impl VelocityState {
	pub fn new() -> VelocityState {
		VelocityState { w_sum: Scalar::zero() }
	}

	/// 速度 -> トルク。2自由度PI: Kp(b*u - w) + Ki∫(u - w)。uは±wmaxに制限する。
	pub fn update(
		&mut self,
		p: &VelocityParam,
		u: Scalar,
		m: &Measurement,
		last_saturated: Saturated
	) -> Scalar {
		// 目標速度制限
		let u_clamped = u.min(p.wmax).max(-p.wmax);

		let w = p.kp * (p.b * u_clamped - m.w) + self.w_sum;

		// 条件付き積分
		let e = u_clamped - m.w;
		let is_flow_plus = e > Scalar::zero();
		match (last_saturated, is_flow_plus) {
			(Saturated::Overflow, true) | (Saturated::Underflow, false) => {},
			_ => {
				self.w_sum += p.ki * e;
			},
		}

		w
	}

	pub fn reset(&mut self) {
		self.w_sum = Scalar::zero();
	}
}

pub struct PositionParam {
	kp: Scalar,
	pmax: Scalar,
}
impl PositionParam {
	pub fn new(c: &Config) -> PositionParam {
		PositionParam { kp: c.pkp, pmax: c.pmax }
	}
}

pub struct PositionState {}
impl PositionState {
	pub fn new() -> PositionState {
		PositionState {}
	}

	/// 位置 -> 速度。対処できる外乱がないのでP制御。uは±pmaxに制限する。
	pub fn update(&self, p: &PositionParam, u: Scalar, m: &Measurement) -> Scalar {
		// 目標位置制限
		let u_clamped = u.min(p.pmax).max(-p.pmax);

		p.kp * (u_clamped - m.th)
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	fn s(v: f32) -> Scalar {
		Scalar::from(v)
	}

	/// 計算過程で丸めが入る値の比較用
	fn assert_close(actual: Scalar, expected: f32) {
		let a = f32::from(actual);
		assert!((a - expected).abs() < 1e-6, "left: {a}, right: {expected}");
	}

	/// i, w, th 以外は電圧12V固定の測定値
	fn meas(i: f32, w: f32, th: f32) -> Measurement {
		Measurement { vdc: s(12.0), vdc_inv: s(1.0 / 12.0), i: s(i), w: s(w), th: s(th) }
	}

	/// 補償を無効にした素のパラメータ。duty_maxは1、電流制限は効かせない
	fn bare_current_param(kp: f32, ki: f32, kb: f32, vmax: f32) -> CurrentParam {
		CurrentParam {
			kp: s(kp),
			ki: s(ki),
			ke: s(0.0),
			dead_duty: s(0.0),
			i_threshold_inv: s(1.0),
			kb: s(kb),
			duty_max: s(1.0),
			vmax: s(vmax),
			imax: s(1e9),
		}
	}

	/// 各フィールドが区別できるよう、すべて異なる値を入れたConfig
	fn distinct_config() -> Config {
		Config {
			vdcmax: s(24.0),
			ke: s(0.7),
			cperiod: s(0.0001),
			ckp: s(10.0),
			cki: s(0.6),
			dead_duty: s(0.02),
			i_threshold_inv: s(0.25),
			kb: s(0.06),
			duty_max: s(0.95),
			vmax: s(20.0),
			imax: s(8.0),
			wkp: s(3.0),
			wki: s(0.4),
			wb: s(0.8),
			wmax: s(50.0),
			pkp: s(5.0),
			pmax: s(6.0),
		}
	}

	#[test]
	fn current_param_new_copies_from_config() {
		let p = CurrentParam::new(&distinct_config());
		assert_eq!(p.kp, s(10.0));
		assert_eq!(p.ki, s(0.6));
		assert_eq!(p.ke, s(0.7));
		assert_eq!(p.dead_duty, s(0.02));
		assert_eq!(p.i_threshold_inv, s(0.25));
		assert_eq!(p.kb, s(0.06));
		assert_eq!(p.duty_max, s(0.95));
		assert_eq!(p.vmax, s(20.0));
		assert_eq!(p.imax, s(8.0));
	}

	#[test]
	fn velocity_param_new_copies_from_config() {
		let p = VelocityParam::new(&distinct_config());
		assert_eq!(p.kp, s(3.0));
		assert_eq!(p.ki, s(0.4));
		assert_eq!(p.b, s(0.8));
		assert_eq!(p.wmax, s(50.0));
	}

	#[test]
	fn position_param_new_copies_from_config() {
		let p = PositionParam::new(&distinct_config());
		assert_eq!(p.kp, s(5.0));
		assert_eq!(p.pmax, s(6.0));
	}

	/// kiは周期Tを含んだ値
	fn vparam(kp: f32, ki: f32, b: f32, wmax: f32) -> VelocityParam {
		VelocityParam { kp: s(kp), ki: s(ki), b: s(b), wmax: s(wmax) }
	}

	#[test]
	fn current_update_converts_voltage_to_duty() {
		let p = bare_current_param(2.0, 0.0, 0.0, 100.0);
		let mut st = CurrentState::new();
		// e = 3 - 1 = 2, v = 2*2 = 4V, duty = 4/12
		let (duty, _) = st.update(&p, s(3.0), &meas(1.0, 0.0, 0.0));
		assert_close(duty, 4.0 / 12.0);
	}

	#[test]
	fn current_update_limits_voltage_by_vmax() {
		let p = bare_current_param(2.0, 0.0, 0.0, 3.0);
		let mut st = CurrentState::new();
		// v = 4V だが vmax=3V で頭打ち -> duty = 3/12
		let (duty, _) = st.update(&p, s(3.0), &meas(1.0, 0.0, 0.0));
		assert_close(duty, 3.0 / 12.0);
	}

	#[test]
	fn current_voltage_limit_follows_measured_bus_voltage() {
		// duty_max=0.5。母線が12Vなら6V、6Vに下がれば3Vが上限になる
		let mut p = bare_current_param(100.0, 0.0, 0.0, 100.0);
		p.duty_max = s(0.5);

		let mut st = CurrentState::new();
		let m12 = Measurement { vdc: s(12.0), vdc_inv: s(1.0 / 12.0), i: s(0.0), w: s(0.0), th: s(0.0) };
		assert_close(st.update(&p, s(1.0), &m12).0, 0.5);

		let mut st = CurrentState::new();
		let m6 = Measurement { vdc: s(6.0), vdc_inv: s(1.0 / 6.0), i: s(0.0), w: s(0.0), th: s(0.0) };
		// 母線が下がってもデューティは duty_max で頭打ちのまま
		assert_close(st.update(&p, s(1.0), &m6).0, 0.5);
	}

	#[test]
	fn current_update_compensates_back_emf() {
		let mut p = bare_current_param(2.0, 0.0, 0.0, 100.0);
		p.ke = s(0.5);
		let mut st = CurrentState::new();
		// v = 2*2 + 0.5*10 = 9V
		assert_close(st.update(&p, s(3.0), &meas(1.0, 10.0, 0.0)).0, 9.0 / 12.0);
	}

	#[test]
	fn current_update_adds_dead_time_duty_after_conversion() {
		let mut p = bare_current_param(2.0, 0.0, 0.0, 100.0);
		p.dead_duty = s(0.05);
		let mut st = CurrentState::new();
		// duty = 4/12 + 0.05*sat(1) = 4/12 + 0.05
		assert_close(st.update(&p, s(3.0), &meas(1.0, 0.0, 0.0)).0, 4.0 / 12.0 + 0.05);
	}

	#[test]
	fn current_dead_time_duty_is_softened_near_zero_current() {
		let mut p = bare_current_param(0.0, 0.0, 0.0, 100.0);
		p.dead_duty = s(0.1);
		p.i_threshold_inv = s(1.0);  // しきい値1A
		let mut st = CurrentState::new();
		// i=0.5A はしきい値内なので 0.1*0.5 = 0.05
		assert_close(st.update(&p, s(0.5), &meas(0.5, 0.0, 0.0)).0, 0.05);
	}

	#[test]
	fn current_duty_is_clamped_to_plus_minus_one() {
		let mut p = bare_current_param(2.0, 0.0, 0.0, 100.0);
		p.dead_duty = s(0.05);
		let mut st = CurrentState::new();
		// e = 7-1 = 6, v = 12V -> duty 1.0 + 0.05 -> 1.0
		assert_close(st.update(&p, s(7.0), &meas(1.0, 0.0, 0.0)).0, 1.0);

		let mut st = CurrentState::new();
		// e = -7+1 = -6, v = -12V -> duty -1.0 - 0.05 -> -1.0
		assert_close(st.update(&p, s(-7.0), &meas(-1.0, 0.0, 0.0)).0, -1.0);
	}

	#[test]
	fn current_update_clamps_target_current() {
		let mut p = bare_current_param(2.0, 0.0, 0.0, 100.0);
		p.imax = s(2.0);
		let mut st = CurrentState::new();
		// u=5 は imax=2 に制限される -> e = 2-1 = 1, v = 2V
		assert_close(st.update(&p, s(5.0), &meas(1.0, 0.0, 0.0)).0, 2.0 / 12.0);

		let mut st = CurrentState::new();
		// e = -2-1 = -3, v = -6V
		assert_close(st.update(&p, s(-5.0), &meas(1.0, 0.0, 0.0)).0, -6.0 / 12.0);
	}

	#[test]
	fn current_clamped_target_also_applies_to_integral() {
		let mut p = bare_current_param(0.0, 1.0, 0.0, 100.0);
		p.imax = s(2.0);
		let mut st = CurrentState::new();
		st.update(&p, s(5.0), &meas(1.0, 0.0, 0.0));
		// 制限後のu=2を使うので i_sum = 1*(2-1) = 1
		assert_eq!(st.i_sum, s(1.0));
	}

	#[test]
	fn current_update_integrates_error() {
		let p = bare_current_param(0.0, 0.5, 0.0, 100.0);
		let mut st = CurrentState::new();
		// kp=0なので1回目の出力は0、積分だけが溜まる
		assert_close(st.update(&p, s(3.0), &meas(1.0, 0.0, 0.0)).0, 0.0);
		// i_sum = 0.5*2 = 1.0 -> 2回目は 1.0V
		assert_close(st.update(&p, s(3.0), &meas(1.0, 0.0, 0.0)).0, 1.0 / 12.0);
	}

	#[test]
	fn current_update_unwinds_integral_when_saturated() {
		let p = bare_current_param(2.0, 1.0, 0.5, 3.0);
		let mut st = CurrentState::new();
		st.update(&p, s(3.0), &meas(1.0, 0.0, 0.0));
		// i_sum = ki*e + kb*(v_limited - v_raw) = 1*2 + 0.5*(3-4) = 1.5
		assert_eq!(st.i_sum, s(1.5));
	}

	#[test]
	fn current_reset_clears_integral() {
		let p = bare_current_param(0.0, 0.5, 0.0, 100.0);
		let mut st = CurrentState::new();
		st.update(&p, s(3.0), &meas(1.0, 0.0, 0.0));
		st.reset();
		assert_close(st.update(&p, s(3.0), &meas(1.0, 0.0, 0.0)).0, 0.0);
	}

	#[test]
	fn current_reports_not_saturated_within_limits() {
		let p = bare_current_param(2.0, 0.0, 0.0, 100.0);
		let mut st = CurrentState::new();
		assert_eq!(st.update(&p, s(3.0), &meas(1.0, 0.0, 0.0)).1, Saturated::NotSaturated);
	}

	#[test]
	fn current_reports_overflow_when_voltage_clipped_high() {
		let p = bare_current_param(2.0, 0.0, 0.0, 3.0);
		let mut st = CurrentState::new();
		// v = 4V -> 3Vで頭打ち
		assert_eq!(st.update(&p, s(3.0), &meas(1.0, 0.0, 0.0)).1, Saturated::Overflow);
	}

	#[test]
	fn current_reports_underflow_when_voltage_clipped_low() {
		let p = bare_current_param(2.0, 0.0, 0.0, 3.0);
		let mut st = CurrentState::new();
		// e = -3 - 1 = -4, v = -8V -> -3Vで頭打ち
		assert_eq!(st.update(&p, s(-3.0), &meas(1.0, 0.0, 0.0)).1, Saturated::Underflow);
	}

	#[test]
	fn current_reports_saturation_when_target_current_clamped() {
		let mut p = bare_current_param(2.0, 0.0, 0.0, 100.0);
		p.imax = s(2.0);
		let mut st = CurrentState::new();
		assert_eq!(st.update(&p, s(5.0), &meas(1.0, 0.0, 0.0)).1, Saturated::Overflow);

		let mut st = CurrentState::new();
		assert_eq!(st.update(&p, s(-5.0), &meas(1.0, 0.0, 0.0)).1, Saturated::Underflow);

		let mut st = CurrentState::new();
		assert_eq!(st.update(&p, s(1.5), &meas(1.0, 0.0, 0.0)).1, Saturated::NotSaturated);
	}

	#[test]
	fn velocity_update_weights_target_in_p_term() {
		// b=1 なら普通のP制御
		let p = vparam(2.0, 0.0, 1.0, 100.0);
		let mut st = VelocityState::new();
		assert_eq!(st.update(&p, s(3.0), &meas(0.0, 1.0, 0.0), Saturated::NotSaturated), s(4.0));

		// b=0.5 なら目標値の寄与が半分
		let p = vparam(2.0, 0.0, 0.5, 100.0);
		let mut st = VelocityState::new();
		// 2*(0.5*3 - 1) = 1.0
		assert_eq!(st.update(&p, s(3.0), &meas(0.0, 1.0, 0.0), Saturated::NotSaturated), s(1.0));
	}

	#[test]
	fn velocity_update_integrates_unweighted_error_scaled_by_ki() {
		// 積分項には重みをかけない(定常偏差を残さないため)
		let p = vparam(0.0, 0.5, 0.5, 100.0);
		let mut st = VelocityState::new();
		assert_eq!(st.update(&p, s(3.0), &meas(0.0, 1.0, 0.0), Saturated::NotSaturated), s(0.0));
		// w_sum = 0.5*(3-1) = 1.0
		assert_eq!(st.update(&p, s(3.0), &meas(0.0, 1.0, 0.0), Saturated::NotSaturated), s(1.0));
	}

	#[test]
	fn velocity_update_clamps_target_speed() {
		let p = vparam(2.0, 0.0, 1.0, 2.0);
		let mut st = VelocityState::new();
		// u=5 は wmax=2 に制限される -> 2*(2-1) = 2
		assert_eq!(st.update(&p, s(5.0), &meas(0.0, 1.0, 0.0), Saturated::NotSaturated), s(2.0));

		let mut st = VelocityState::new();
		// 2*(-2-1) = -6
		assert_eq!(st.update(&p, s(-5.0), &meas(0.0, 1.0, 0.0), Saturated::NotSaturated), s(-6.0));
	}

	#[test]
	fn velocity_clamped_target_also_applies_to_integral() {
		let p = vparam(0.0, 1.0, 1.0, 2.0);
		let mut st = VelocityState::new();
		st.update(&p, s(5.0), &meas(0.0, 1.0, 0.0), Saturated::NotSaturated);
		// 制限後のu=2を使うので w_sum = 1*(2-1) = 1
		assert_eq!(st.update(&p, s(5.0), &meas(0.0, 1.0, 0.0), Saturated::NotSaturated), s(1.0));
	}

	#[test]
	fn velocity_reset_clears_integral() {
		let p = vparam(0.0, 1.0, 1.0, 100.0);
		let mut st = VelocityState::new();
		st.update(&p, s(3.0), &meas(0.0, 1.0, 0.0), Saturated::NotSaturated);
		st.reset();
		assert_eq!(st.update(&p, s(3.0), &meas(0.0, 1.0, 0.0), Saturated::NotSaturated), s(0.0));
	}

	#[test]
	fn velocity_freezes_integral_while_inner_overflow() {
		let p = vparam(1.0, 1.0, 1.0, 100.0);
		let mut st = VelocityState::new();
		// e = 3-1 = 2 > 0。上側に飽和中に正方向へ積分を進めると悪化するので止める
		st.update(&p, s(3.0), &meas(0.0, 1.0, 0.0), Saturated::Overflow);
		// 積分が溜まっていなければ2回目もP項だけ: 1*(3-1) = 2
		assert_eq!(st.update(&p, s(3.0), &meas(0.0, 1.0, 0.0), Saturated::Overflow), s(2.0));
	}

	#[test]
	fn velocity_freezes_integral_while_inner_underflow() {
		let p = vparam(1.0, 1.0, 1.0, 100.0);
		let mut st = VelocityState::new();
		// e = -3-1 = -4 < 0。下側に飽和中に負方向へ積分を進めると悪化するので止める
		st.update(&p, s(-3.0), &meas(0.0, 1.0, 0.0), Saturated::Underflow);
		assert_eq!(st.update(&p, s(-3.0), &meas(0.0, 1.0, 0.0), Saturated::Underflow), s(-4.0));
	}

	#[test]
	fn velocity_keeps_integrating_when_error_escapes_saturation() {
		let p = vparam(1.0, 1.0, 1.0, 100.0);
		let mut st = VelocityState::new();
		// まず正方向に積分を溜める(飽和なし)
		st.update(&p, s(3.0), &meas(0.0, 1.0, 0.0), Saturated::NotSaturated);
		// w_sum = 2。上側に飽和中でも誤差が負なら積分を戻せる必要がある
		// 出力 = 1*(-1-1) + 2 = 0
		assert_eq!(st.update(&p, s(-1.0), &meas(0.0, 1.0, 0.0), Saturated::Overflow), s(0.0));
		// w_sum = 2 + (-2) = 0 なので、次は P項だけ
		assert_eq!(st.update(&p, s(-1.0), &meas(0.0, 1.0, 0.0), Saturated::Overflow), s(-2.0));
	}

	#[test]
	fn velocity_integral_freeze_needs_saturation() {
		// 飽和していなければ、誤差が出力と同符号でも普通に積分する
		let p = vparam(1.0, 1.0, 1.0, 100.0);
		let mut st = VelocityState::new();
		st.update(&p, s(3.0), &meas(0.0, 1.0, 0.0), Saturated::NotSaturated);
		// w_sum = 2 -> 1*(3-1) + 2 = 4
		assert_eq!(st.update(&p, s(3.0), &meas(0.0, 1.0, 0.0), Saturated::NotSaturated), s(4.0));
	}

	#[test]
	fn position_update_is_proportional() {
		let p = PositionParam { kp: s(3.0), pmax: s(100.0) };
		let st = PositionState::new();
		// 3*(2 - 0.5) = 4.5
		assert_eq!(st.update(&p, s(2.0), &meas(0.0, 0.0, 0.5)), s(4.5));
		assert_eq!(st.update(&p, s(0.0), &meas(0.0, 0.0, 0.5)), s(-1.5));
	}

	#[test]
	fn position_update_clamps_target_position() {
		let p = PositionParam { kp: s(3.0), pmax: s(2.0) };
		let st = PositionState::new();
		// u=5 は pmax=2 に制限される -> 3*(2 - 0.5) = 4.5
		assert_eq!(st.update(&p, s(5.0), &meas(0.0, 0.0, 0.5)), s(4.5));
		// u=-5 は -2 に制限される -> 3*(-2 - 0.5) = -7.5
		assert_eq!(st.update(&p, s(-5.0), &meas(0.0, 0.0, 0.5)), s(-7.5));
	}
}

