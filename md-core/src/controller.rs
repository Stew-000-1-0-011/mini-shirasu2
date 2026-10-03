// TODO: 上限による制限を上限と下限による制限に変更

use crate::fixed::{Divisor, Q3_28, Q3_60, Q16_16};

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

impl Measurement {
	/// 母線電圧を更新する。逆数がQ3.28に収まらない(vdcが1/8以下 = 母線電圧がvdcmax/8以下)
	/// ときは何も更新せずfalseを返す
	pub fn update_vdc(&mut self, vdc: Q3_28) -> bool {
		if vdc <= Q3_28::ZERO {
			return false;
		}
		match Q3_28::ONE.checked_div(vdc) {
			Some(vdc_inv) => {
				self.vdc = vdc;
				self.vdc_inv = vdc_inv;
				true
			},
			None => false,
		}
	}
}

/// 設定値。物理単位で持ち、各Param::newでper-unitに変換する
pub struct Config {
	pub vdcmax: f32,  // [V] Vdcの最大値。電圧の基準値を兼ねる。母線電圧がこの1/8以下のときは測定値を更新しない
	pub ibase: f32,  // [A] 電流の基準値(測定フルスケール)
	pub wbase: f32,  // [rad/s] 速度の基準値(最高速度程度)
	pub ke: f32,  // [V/(rad/s)] 逆起電力

	pub cperiod: f32,  // [s] 電流制御周期
	pub ckp: f32,  // [V/A] 電流P制御
	pub cki: f32,  // [V/(A*s)] 電流I制御
	pub dead_duty: f32,  // デッドタイムによる誤差デューティー比
	pub i_threshold: f32,  // [A] 電流値が[-i_threshold, i_threshold]の間は符号を[-1, 1]に
	pub duty_max: f32,  // シャント抵抗に電流を流したり、ブートストラップするための上限(vmaxとminをとられる)
	pub vmax: f32,  // [V] 出力電圧上限(duty_maxとminをとられる)
	pub imax: f32,  // [A] 目標電流上限(出力が必ずしも超えないとは限らないことに注意！)

	pub wperiod: f32,  // [s] 速度制御周期
	pub wkp: f32,  // [A/(rad/s)] 速度P制御
	pub wki: f32,  // [A/(rad/s*s)] 速度I制御
	pub wb: f32,  // 速度P項の目標値への重み
	pub wmax: f32,  // [rad/s] 目標速度上限

	pub pkp: f32,  // [1/s] 位置P制御
	pub pmax: f32,  // [回転] 目標位置上限
}

/// Configからの変換に失敗した理由。文字列はConfigのフィールド名
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ConfigError {
	/// 正でなければならない値が0以下、NaN、無限大
	NotPositive(&'static str),
	/// 0以上でなければならない値が負
	Negative(&'static str),
	/// per-unit変換後の値が型の範囲に収まらない、またはNaN
	OutOfRange(&'static str),
}

/// 正の有限値ならそのまま返す
fn positive(name: &'static str, v: f32) -> Result<f32, ConfigError> {
	if v > 0.0 && v.is_finite() { Ok(v) } else { Err(ConfigError::NotPositive(name)) }
}

/// 0以上ならそのまま返す。NaNは後段のto_qでOutOfRangeになるのでここでは通す
fn non_negative(name: &'static str, v: f32) -> Result<f32, ConfigError> {
	if v < 0.0 { Err(ConfigError::Negative(name)) } else { Ok(v) }
}

fn to_q(name: &'static str, v: f32) -> Result<Q3_28, ConfigError> {
	Q3_28::checked_from_f32(v).ok_or(ConfigError::OutOfRange(name))
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
fn soft_sign(i: Q3_28, th: Divisor) -> Q3_28 {
	if i >= th.value() {
		Q3_28::ONE
	} else if i <= -th.value() {
		-Q3_28::ONE
	} else {
		// |i| < th なので結果は(-1, 1)に収まる。毎周期通るので、割り算ではなく逆数との掛け算にする
		i.div_by(th)
	}
}

pub struct CurrentParam {
	kp: Q3_28,
	ki: Q3_28,  // 周期Tを掛けた値
	kb: Q3_28,  // アンチワインドアップ。ki(T込み) / kp
	ke: Q3_28,
	dead_duty: Q3_28,
	i_threshold: Divisor,  // 電流値が[-i_threshold, i_threshold]の間は符号を[-1, 1]に
	duty_max: Q3_28,
	vmax: Q3_28,
	imax: Q3_28,
}

impl CurrentParam {
	pub fn new(c: &Config) -> Result<CurrentParam, ConfigError> {
		let vb = positive("vdcmax", c.vdcmax)?;
		let ib = positive("ibase", c.ibase)?;
		let ob = positive("wbase", c.wbase)?;
		let t = positive("cperiod", c.cperiod)?;
		let ckp = positive("ckp", c.ckp)?;
		let ith = positive("i_threshold", c.i_threshold)?;
		let cki = non_negative("cki", c.cki)?;
		let dead_duty = non_negative("dead_duty", c.dead_duty)?;
		let duty_max = non_negative("duty_max", c.duty_max)?;
		let vmax = non_negative("vmax", c.vmax)?;
		let imax = non_negative("imax", c.imax)?;
		if duty_max > 1.0 {
			return Err(ConfigError::OutOfRange("duty_max"));
		}

		// 飽和中の積分は i_sum' = (1-kb)*i_sum + kb*(v2-ff) となり、kb > 2 で発散して±duty_maxで振動する。
		// kb = 1 なら飽和した周期のうちに積分値を戻せるので、1を上限にする
		let kb = (cki * t / ckp).min(1.0);
		let i_threshold = to_q("i_threshold", ith / ib)?;
		// pu変換の丸めで0になると、電流0でもデッドタイム補償が掛かってしまう
		if i_threshold == Q3_28::ZERO {
			return Err(ConfigError::NotPositive("i_threshold"));
		}

		Ok(CurrentParam {
			kp: to_q("ckp", ckp * ib / vb)?,
			ki: to_q("cki", cki * t * ib / vb)?,
			kb: to_q("kb", kb)?,
			ke: to_q("ke", c.ke * ob / vb)?,
			dead_duty: to_q("dead_duty", dead_duty)?,
			i_threshold: Divisor::new(i_threshold),
			duty_max: to_q("duty_max", duty_max)?,
			vmax: to_q("vmax", vmax / vb)?,
			imax: to_q("imax", imax / ib)?,
		})
	}
}

pub struct CurrentState {
	i_sum: Integrator,  // 積分
}
impl CurrentState {
	pub fn new() -> CurrentState {
		CurrentState { i_sum: Integrator::new() }
	}

	/// 電流 -> デューティ比。PI -> 逆起電力補償 -> デッドタイム補償 -> 電圧制限 + デューティ上限 -> デューティ換算
	pub fn update(&mut self, p: &CurrentParam, u: Q3_28, m: &Measurement) -> (Q3_28, Saturated) {
		// 目標電流を制限する
		let u_clamped = u.min(p.imax).max(-p.imax);
		let e = u_clamped - m.i;

		// PI
		let v_pi = p.kp * e + self.i_sum.value();
		// 逆起電力補償
		let v1 = v_pi + p.ke * m.w;
		// デッドタイム補償(デューティー比の誤差)を電圧に直して足す
		let v2 = v1 + p.dead_duty * soft_sign(m.i, p.i_threshold) * m.vdc;

		// 最大電圧制限 & デューティ上限
		let vlim = p.vmax.min(p.duty_max * m.vdc);
		let v3 = v2.min(vlim).max(-vlim);

		// アンチワインドアップを入れた積算。戻すのは制限で削られた分だけ
		self.i_sum.add_product(p.ki, e);
		self.i_sum.add_product(p.kb, v3 - v2);

		let duty2 = (v3 * m.vdc_inv).min(Q3_28::ONE).max(-Q3_28::ONE);

		let saturate = if u > p.imax || v2 > v3 {
			Saturated::Overflow
		} else if u < -p.imax || v2 < v3 {
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

impl VelocityParam {
	pub fn new(c: &Config) -> Result<VelocityParam, ConfigError> {
		let ib = positive("ibase", c.ibase)?;
		let ob = positive("wbase", c.wbase)?;
		let t = positive("wperiod", c.wperiod)?;
		let wkp = non_negative("wkp", c.wkp)?;
		let wki = non_negative("wki", c.wki)?;
		let wmax = non_negative("wmax", c.wmax)?;

		Ok(VelocityParam {
			kp: to_q("wkp", wkp * ob / ib)?,
			ki: to_q("wki", wki * t * ob / ib)?,
			b: to_q("wb", c.wb)?,
			wmax: to_q("wmax", wmax / ob)?,
		})
	}
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

impl PositionParam {
	pub fn new(c: &Config) -> Result<PositionParam, ConfigError> {
		let ob = positive("wbase", c.wbase)?;
		let pkp = non_negative("pkp", c.pkp)?;
		let pmax = non_negative("pmax", c.pmax)?;

		Ok(PositionParam {
			// 位置は回転単位なので、rad/sへ直す2πが入る
			kp: to_q("pkp", pkp * core::f32::consts::TAU / ob)?,
			pmax: Q16_16::checked_from_f32(pmax).ok_or(ConfigError::OutOfRange("pmax"))?,
		})
	}
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
			i_threshold: Divisor::new(q(1.0)),
			duty_max: q(1.0),
			vmax: q(vmax),
			imax: q(7.0),
		}
	}

	/// kiは周期Tを含んだ値
	fn vparam(kp: f32, ki: f32, b: f32, wmax: f32) -> VelocityParam {
		VelocityParam { kp: q(kp), ki: q(ki), b: q(b), wmax: q(wmax) }
	}

	/// 物理単位の設定例。基準値は V_b=24V, I_b=10A, ω_b=300rad/s
	fn config() -> Config {
		Config {
			vdcmax: 24.0,
			ibase: 10.0,
			wbase: 300.0,
			ke: 0.02,
			cperiod: 5e-5,
			ckp: 6.0,
			cki: 1000.0,
			dead_duty: 0.02,
			i_threshold: 0.5,
			duty_max: 0.95,
			vmax: 20.0,
			imax: 8.0,
			wperiod: 1e-3,
			wkp: 0.05,
			wki: 0.5,
			wb: 0.8,
			wmax: 250.0,
			pkp: 20.0,
			pmax: 100.0,
		}
	}

	// ---- Param::new ----

	#[test]
	fn current_param_new_converts_to_per_unit() {
		let p = CurrentParam::new(&config()).unwrap();
		assert_close(p.kp, 2.5);                 // 6 * 10 / 24
		assert_close(p.ki, 0.020833334);         // 1000 * 5e-5 * 10 / 24
		assert_close(p.kb, 0.008333334);         // 1000 * 5e-5 / 6
		assert_close(p.ke, 0.25);                // 0.02 * 300 / 24
		assert_close(p.dead_duty, 0.02);
		assert_close(p.i_threshold.value(), 0.05);  // 0.5 / 10
		assert_close(p.duty_max, 0.95);
		assert_close(p.vmax, 0.8333333);         // 20 / 24
		assert_close(p.imax, 0.8);               // 8 / 10
	}

	#[test]
	fn velocity_param_new_converts_to_per_unit() {
		let p = VelocityParam::new(&config()).unwrap();
		assert_close(p.kp, 1.5);                 // 0.05 * 300 / 10
		assert_close(p.ki, 0.015);               // 0.5 * 1e-3 * 300 / 10
		assert_close(p.b, 0.8);
		assert_close(p.wmax, 0.8333333);         // 250 / 300
	}

	#[test]
	fn position_param_new_converts_to_per_unit() {
		let p = PositionParam::new(&config()).unwrap();
		assert_close(p.kp, 0.41887903);          // 20 * 2π / 300
		assert!((p.pmax.to_f32() - 100.0).abs() < 1e-3);  // 回転のまま
	}

	#[test]
	fn param_new_rejects_non_positive_values() {
		let mut c = config();
		c.ckp = 0.0;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::NotPositive("ckp")));

		let mut c = config();
		c.i_threshold = 0.0;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::NotPositive("i_threshold")));

		let mut c = config();
		c.cperiod = -1e-4;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::NotPositive("cperiod")));

		let mut c = config();
		c.vdcmax = 0.0;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::NotPositive("vdcmax")));

		let mut c = config();
		c.ibase = 0.0;
		assert_eq!(VelocityParam::new(&c).err(), Some(ConfigError::NotPositive("ibase")));

		let mut c = config();
		c.wperiod = 0.0;
		assert_eq!(VelocityParam::new(&c).err(), Some(ConfigError::NotPositive("wperiod")));

		let mut c = config();
		c.wbase = 0.0;
		assert_eq!(PositionParam::new(&c).err(), Some(ConfigError::NotPositive("wbase")));
	}

	#[test]
	fn param_new_rejects_values_out_of_range() {
		// 100 * 10 / 24 = 41.7 は Q3.28 に入らない
		let mut c = config();
		c.ckp = 100.0;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::OutOfRange("ckp")));

		let mut c = config();
		c.wkp = 1.0;  // 1 * 300 / 10 = 30
		assert_eq!(VelocityParam::new(&c).err(), Some(ConfigError::OutOfRange("wkp")));

		let mut c = config();
		c.pmax = 1e6;
		assert_eq!(PositionParam::new(&c).err(), Some(ConfigError::OutOfRange("pmax")));
	}

	#[test]
	fn param_new_rejects_nan_and_infinity() {
		let mut c = config();
		c.ke = f32::NAN;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::OutOfRange("ke")));

		let mut c = config();
		c.vdcmax = f32::NAN;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::NotPositive("vdcmax")));

		let mut c = config();
		c.wki = f32::INFINITY;
		assert_eq!(VelocityParam::new(&c).err(), Some(ConfigError::OutOfRange("wki")));

		let mut c = config();
		c.pkp = f32::NAN;
		assert_eq!(PositionParam::new(&c).err(), Some(ConfigError::OutOfRange("pkp")));
	}

	#[test]
	fn param_new_rejects_negative_limits() {
		let mut c = config();
		c.imax = -1.0;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::Negative("imax")));

		let mut c = config();
		c.vmax = -1.0;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::Negative("vmax")));

		let mut c = config();
		c.duty_max = -0.5;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::Negative("duty_max")));

		let mut c = config();
		c.dead_duty = -0.02;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::Negative("dead_duty")));

		let mut c = config();
		c.wmax = -1.0;
		assert_eq!(VelocityParam::new(&c).err(), Some(ConfigError::Negative("wmax")));

		let mut c = config();
		c.pmax = -1.0;
		assert_eq!(PositionParam::new(&c).err(), Some(ConfigError::Negative("pmax")));
	}

	#[test]
	fn param_new_rejects_negative_gains() {
		let mut c = config();
		c.cki = -1.0;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::Negative("cki")));

		let mut c = config();
		c.wkp = -0.05;
		assert_eq!(VelocityParam::new(&c).err(), Some(ConfigError::Negative("wkp")));

		let mut c = config();
		c.wki = -0.5;
		assert_eq!(VelocityParam::new(&c).err(), Some(ConfigError::Negative("wki")));

		let mut c = config();
		c.pkp = -1.0;
		assert_eq!(PositionParam::new(&c).err(), Some(ConfigError::Negative("pkp")));
	}

	#[test]
	fn param_new_rejects_duty_max_above_one() {
		let mut c = config();
		c.duty_max = 1.5;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::OutOfRange("duty_max")));

		c.duty_max = 1.0;
		assert!(CurrentParam::new(&c).is_ok());
	}

	#[test]
	fn param_new_accepts_zero_gains_and_limits() {
		let mut c = config();
		c.cki = 0.0;
		c.imax = 0.0;
		assert!(CurrentParam::new(&c).is_ok());

		let mut c = config();
		c.wki = 0.0;
		assert!(VelocityParam::new(&c).is_ok());
	}

	#[test]
	fn param_new_allows_negative_ke_and_wb() {
		let mut c = config();
		c.ke = -0.02;
		let p = CurrentParam::new(&c).unwrap();
		assert_close(p.ke, -0.25);

		let mut c = config();
		c.wb = -0.5;
		assert!(VelocityParam::new(&c).is_ok());
	}

	#[test]
	fn current_param_new_clamps_kb_to_one() {
		// kb = 50000 * 5e-5 / 0.5 = 5 だが、kb > 2 では飽和中の積分が発散するので1に抑える
		let mut c = config();
		c.ckp = 0.5;
		c.cki = 50000.0;
		let p = CurrentParam::new(&c).unwrap();
		assert_close(p.kb, 1.0);
	}

	#[test]
	#[cfg(not(feature = "f32"))]  // 浮動小数点版には分解能がなく、1e-13も0にならない
	fn current_param_new_rejects_i_threshold_that_quantizes_to_zero() {
		// 1e-12 / 10 はQ3.28の分解能(約3.7e-9)より小さく、変換すると0になる
		let mut c = config();
		c.i_threshold = 1e-12;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::NotPositive("i_threshold")));
	}

	#[test]
	fn current_param_new_rejects_infinite_vdcmax() {
		let mut c = config();
		c.vdcmax = f32::INFINITY;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::NotPositive("vdcmax")));
	}

	#[test]
	fn current_loop_recovers_from_saturation_with_low_inductance_tuning() {
		// 低インダクタンスのモーター想定(R=5Ω, L=50µH, T=50µs)でkb = T*R/L = 5 になるチューニング
		let mut c = config();
		c.vdcmax = 24.0;
		c.ibase = 10.0;
		c.cperiod = 5e-5;
		c.ckp = 0.5;
		c.cki = 50000.0;
		c.dead_duty = 0.0;
		c.ke = 0.0;
		let p = CurrentParam::new(&c).unwrap();
		let mut st = CurrentState::new();

		// 純抵抗R=5Ω、母線0.5pu(12V)固定。次周期の電流[pu] = duty * 0.5 * 24 / (5 * 10)
		let mut i = 0.0f32;
		let mut last = [(0.0f32, Saturated::Overflow); 20];
		for n in 0..203 {
			// 最初の3周期は飽和する大きな目標、その後は小さな目標
			let target = if n < 3 { 0.8 } else { 0.05 };
			let (duty, sat) = st.update(&p, q(target), &meas(i, 0.0, 0.0));
			let duty = duty.to_f32();
			i = duty * 0.24;
			if n >= 183 {
				last[n - 183] = (duty, sat);
			}
		}

		assert!(last.iter().all(|&(_, s)| s == Saturated::NotSaturated), "{last:?}");
		let max = last.iter().map(|&(d, _)| d).fold(f32::MIN, f32::max);
		let min = last.iter().map(|&(d, _)| d).fold(f32::MAX, f32::min);
		assert!(max - min < 1e-3, "min: {min}, max: {max}");
		// 定常状態は 0.05 / 0.24
		assert!((max - 0.05 / 0.24).abs() < 1e-2, "max: {max}");
	}

	// ---- 母線電圧 ----

	#[test]
	fn update_vdc_sets_voltage_and_inverse() {
		let mut m = meas(0.0, 0.0, 0.0);
		assert!(m.update_vdc(q(0.75)));
		assert_close(m.vdc, 0.75);
		assert_close(m.vdc_inv, 1.3333334);
	}

	#[test]
	fn update_vdc_skips_when_inverse_does_not_fit() {
		// 逆数が8未満に収まるのは vdc > 1/8 (母線電圧がvdcmax/8より大きい)ときだけ。
		// 収まらなければ前回の値(0.5, 2.0)を保つ
		for vdc in [0.125, 0.0625, 0.0, -0.5] {
			let mut m = meas(0.0, 0.0, 0.0);
			assert!(!m.update_vdc(q(vdc)), "vdc: {vdc}");
			assert_close(m.vdc, 0.5);
			assert_close(m.vdc_inv, 2.0);
		}
	}

	#[test]
	fn update_vdc_just_above_one_eighth_stays_in_range() {
		let mut m = meas(0.0, 0.0, 0.0);
		assert!(m.update_vdc(q(0.13)));
		assert!(m.vdc_inv < Q3_28::MAX);
		// 8に近い値なので、丸めの分だけ許容誤差を広げる
		assert!((m.vdc_inv.to_f32() - 1.0 / 0.13).abs() < 1e-4);
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
		assert_eq!(soft_sign(q(0.5), Divisor::new(q(0.125))), Q3_28::ONE);
		assert_eq!(soft_sign(q(-0.5), Divisor::new(q(0.125))), -Q3_28::ONE);
		// しきい値ちょうども±1
		assert_eq!(soft_sign(q(0.125), Divisor::new(q(0.125))), Q3_28::ONE);
		assert_eq!(soft_sign(q(-0.125), Divisor::new(q(0.125))), -Q3_28::ONE);
	}

	#[test]
	fn soft_sign_is_linear_inside_threshold() {
		assert_close(soft_sign(q(0.25), Divisor::new(q(0.5))), 0.5);
		assert_close(soft_sign(q(-0.125), Divisor::new(q(0.5))), -0.25);
		assert_close(soft_sign(Q3_28::ZERO, Divisor::new(q(0.5))), 0.0);
	}

	#[test]
	fn soft_sign_with_zero_threshold_does_not_divide() {
		// しきい値がpu変換の丸めで0になっても0除算しない
		assert_eq!(soft_sign(q(0.5), Divisor::new(Q3_28::ZERO)), Q3_28::ONE);
		assert_eq!(soft_sign(q(-0.5), Divisor::new(Q3_28::ZERO)), -Q3_28::ONE);
		assert_eq!(soft_sign(Q3_28::ZERO, Divisor::new(Q3_28::ZERO)), Q3_28::ONE);
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
	fn current_update_adds_dead_time_compensation_as_voltage() {
		let mut p = bare_current_param(0.5, 0.0, 0.0, 7.0);
		p.dead_duty = q(0.0625);
		p.i_threshold = Divisor::new(q(0.125));
		let mut st = CurrentState::new();
		// v = 0.25 + 0.0625*soft_sign(0.25)*0.5 = 0.28125 -> duty = 0.5625
		assert_close(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).0, 0.5625);
	}

	#[test]
	fn current_dead_time_duty_is_softened_near_zero_current() {
		let mut p = bare_current_param(0.0, 0.0, 0.0, 7.0);
		p.dead_duty = q(0.125);
		p.i_threshold = Divisor::new(q(0.5));
		let mut st = CurrentState::new();
		// i=0.25 はしきい値0.5の内側なので 0.125*0.5 = 0.0625
		assert_close(st.update(&p, q(0.25), &meas(0.25, 0.0, 0.0)).0, 0.0625);
		assert_close(st.update(&p, q(-0.25), &meas(-0.25, 0.0, 0.0)).0, -0.0625);
	}

	#[test]
	fn current_duty_with_dead_time_never_exceeds_duty_max() {
		let mut p = bare_current_param(2.0, 0.0, 0.0, 7.0);
		p.duty_max = q(0.75);
		p.dead_duty = q(0.0625);
		p.i_threshold = Divisor::new(q(0.125));
		let mut st = CurrentState::new();
		// v = 2*0.5 = 1.0 に補償を足しても、0.75*0.5 = 0.375 で頭打ち -> duty 0.75
		assert_close(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).0, 0.75);

		let mut st = CurrentState::new();
		assert_close(st.update(&p, q(-0.75), &meas(-0.25, 0.0, 0.0)).0, -0.75);
	}

	#[test]
	fn current_dead_time_compensation_alone_is_not_saturation() {
		let mut p = bare_current_param(0.5, 0.0, 0.0, 7.0);
		p.dead_duty = q(0.0625);
		p.i_threshold = Divisor::new(q(0.125));
		let mut st = CurrentState::new();
		// 補償を足しても制限に届かなければ飽和ではない(電流の向きによらず)
		assert_eq!(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).1, Saturated::NotSaturated);
		assert_eq!(st.update(&p, q(-0.75), &meas(-0.25, 0.0, 0.0)).1, Saturated::NotSaturated);
	}

	#[test]
	fn current_reports_saturation_when_dead_time_compensation_hits_limit() {
		// PIの出力0.25は制限0.26の内側だが、補償0.03125を足すと超える
		let mut p = bare_current_param(0.5, 0.0, 0.0, 0.26);
		p.dead_duty = q(0.0625);
		p.i_threshold = Divisor::new(q(0.125));
		let mut st = CurrentState::new();
		let (duty, saturated) = st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0));
		assert_close(duty, 0.52);
		assert_eq!(saturated, Saturated::Overflow);
	}

	#[test]
	fn current_dead_time_compensation_does_not_leak_into_integral() {
		let mut p = bare_current_param(0.0, 0.0, 0.5, 7.0);
		p.dead_duty = q(0.0625);
		p.i_threshold = Divisor::new(q(0.125));
		let mut st = CurrentState::new();
		// 誤差0・飽和なし。アンチワインドアップは制限で削られた分だけを戻すので、積分は動かない
		st.update(&p, q(0.25), &meas(0.25, 0.0, 0.0));
		assert_eq!(st.i_sum.value(), Q3_28::ZERO);
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
