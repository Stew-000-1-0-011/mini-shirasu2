//! 設定値。既定値は持たず、SetParamで1つずつ設定する

use md_core::controller::{Config, ConfigError, CurrentParam, PositionParam, VelocityParam};
use md_core::encoder::EncoderParam;
use md_core::fixed::{Q3_28, Q16_16};

use crate::sense::SenseParam;

/// TIM1の周期。72MHz、センターアラインで 72e6 / (2 * 1800) = 20kHz
pub const PWM_ARR: u16 = 1800;
/// 電流制御の周期[s]。PWMの1周期
pub const CURRENT_PERIOD: f32 = 50e-6;
/// 速度・位置制御は電流制御の何回に1回か
pub const OUTER_DIV: u32 = 20;
/// 速度・位置制御の周期[s]
pub const OUTER_PERIOD: f32 = 1e-3;

// ストリームごとのCAN ID(標準ID)。小さいほど優先度が高い。複数台つなぐときは基板ごとに変える
pub const CAN_ID_TARGET: u16 = 0x100;
pub const CAN_ID_STATUS: u16 = 0x101;
pub const CAN_ID_COMMAND: u16 = 0x200;
pub const CAN_ID_RESPONSE: u16 = 0x201;

/// SetParamの設定ID
pub mod id {
    pub const VDCMAX: u8 = 0x00;
    pub const IBASE: u8 = 0x01;
    pub const WBASE: u8 = 0x02;
    pub const KE: u8 = 0x03;
    pub const CKP: u8 = 0x04;
    pub const CKI: u8 = 0x05;
    pub const DEAD_DUTY: u8 = 0x06;
    pub const I_THRESHOLD: u8 = 0x07;
    pub const DUTY_MAX: u8 = 0x08;
    pub const VMAX: u8 = 0x09;
    pub const IMAX: u8 = 0x0A;
    pub const WKP: u8 = 0x0B;
    pub const WKI: u8 = 0x0C;
    pub const WB: u8 = 0x0D;
    pub const WMAX: u8 = 0x0E;
    pub const PKP: u8 = 0x0F;
    pub const PMAX: u8 = 0x10;
    pub const ACCEL_TO_CURRENT: u8 = 0x20;
    pub const ENCODER_CPR: u8 = 0x21;
    pub const W_FILTER_ALPHA: u8 = 0x22;
    pub const ENCODER_REVERSED: u8 = 0x23;
    pub const STATUS_PERIOD_MS: u8 = 0x24;
}

/// 設定値の総数
const COUNT: usize = 22;
/// 有効化に必要な設定値の数。0x00..=0x10 と 0x20..=0x23。STATUS_PERIOD_MSは含まない
const REQUIRED: usize = 21;
/// 0x20番台が始まる添字
const FIRM_BASE: usize = 0x11;

fn index(id: u8) -> Option<usize> {
    match id {
        0x00..=0x10 => Some(id as usize),
        0x20..=0x24 => Some(FIRM_BASE + (id - 0x20) as usize),
        _ => None,
    }
}

fn id_at(index: usize) -> u8 {
    if index < FIRM_BASE { index as u8 } else { 0x20 + (index - FIRM_BASE) as u8 }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum SetError {
    UnknownId,
    /// NaNか無限大
    NotFinite,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum BuildError {
    /// まだ設定されていない値がある。最も小さいID
    Unset(u8),
    /// 検証に通らない値がある。原因のID
    Invalid(u8),
}

impl BuildError {
    pub fn id(self) -> u8 {
        match self {
            BuildError::Unset(id) | BuildError::Invalid(id) => id,
        }
    }
}

/// 物理単位とper-unitの換算
#[derive(Clone, Copy)]
pub struct Scale {
    ibase: f32,
    wbase: f32,
    vdcmax: f32,
    accel_to_current: f32,
}

impl Scale {
    /// 電流[A] -> pu。範囲外やNaNはNone
    pub fn current_pu(&self, a: f32) -> Option<Q3_28> {
        Q3_28::checked_from_f32(a / self.ibase)
    }

    /// 速度[rad/s] -> pu
    pub fn velocity_pu(&self, w: f32) -> Option<Q3_28> {
        Q3_28::checked_from_f32(w / self.wbase)
    }

    /// 加速度FF[rad/s²] -> トルクFF[pu]。per-unitではトルクと電流が同じ値になる
    pub fn accel_ff_pu(&self, alpha: f32) -> Option<Q3_28> {
        Q3_28::checked_from_f32(alpha * self.accel_to_current / self.ibase)
    }

    pub fn current_a(&self, i: Q3_28) -> f32 {
        i.to_f32() * self.ibase
    }

    pub fn velocity_rad_s(&self, w: Q3_28) -> f32 {
        w.to_f32() * self.wbase
    }

    pub fn voltage_v(&self, v: Q3_28) -> f32 {
        v.to_f32() * self.vdcmax
    }
}

/// 通信上の位置(回転単位のQ16.16)を位置にする
pub fn position_from_wire(raw: i32) -> Q16_16 {
    Q16_16::from_ratio(raw, 65536)
}

/// 位置を通信上の表現(回転単位のQ16.16)にする
pub fn position_to_wire(th: Q16_16) -> i32 {
    th.scale_int(65536)
}

/// 電流制御の割り込みが使うもの
pub struct InnerParams {
    pub current: CurrentParam,
    pub sense: SenseParam,
}

/// 速度・位置制御と通信が使うもの
pub struct OuterParams {
    pub velocity: VelocityParam,
    pub position: PositionParam,
    pub encoder: EncoderParam,
    pub sense: SenseParam,
    pub scale: Scale,
}

pub struct Params {
    pub inner: InnerParams,
    pub outer: OuterParams,
}

/// ConfigErrorのフィールド名を設定IDにする
fn id_of(e: ConfigError) -> u8 {
    let name = match e {
        ConfigError::NotPositive(n) | ConfigError::Negative(n) | ConfigError::OutOfRange(n) => n,
    };
    match name {
        "vdcmax" => id::VDCMAX,
        "ibase" => id::IBASE,
        "wbase" => id::WBASE,
        "ke" => id::KE,
        "ckp" => id::CKP,
        // kb は cki と ckp から決まる
        "cki" | "kb" => id::CKI,
        "dead_duty" => id::DEAD_DUTY,
        "i_threshold" => id::I_THRESHOLD,
        "duty_max" => id::DUTY_MAX,
        "vmax" => id::VMAX,
        "imax" => id::IMAX,
        "wkp" => id::WKP,
        "wki" => id::WKI,
        "wb" => id::WB,
        "wmax" => id::WMAX,
        "pkp" => id::PKP,
        "pmax" => id::PMAX,
        "encoder_cpr" => id::ENCODER_CPR,
        "w_filter_alpha" => id::W_FILTER_ALPHA,
        // 周期はファームで固定しているので、ここには来ない
        _ => 0xFF,
    }
}

pub struct Settings {
    values: [Option<f32>; COUNT],
}

impl Settings {
    pub const fn new() -> Settings {
        Settings { values: [None; COUNT] }
    }

    pub fn set(&mut self, id: u8, value: f32) -> Result<(), SetError> {
        let i = index(id).ok_or(SetError::UnknownId)?;
        if !value.is_finite() {
            return Err(SetError::NotFinite);
        }
        self.values[i] = Some(value);
        Ok(())
    }

    pub fn get(&self, id: u8) -> Option<f32> {
        self.values[index(id)?]
    }

    /// 有効化に必要な設定値のうち、未設定で最も小さいID
    pub fn first_unset(&self) -> Option<u8> {
        (0..REQUIRED).find(|&i| self.values[i].is_none()).map(id_at)
    }

    /// 状態を送る周期[ms]。未設定や1未満なら0(送らない)
    pub fn status_period_ms(&self) -> u32 {
        match self.get(id::STATUS_PERIOD_MS) {
            Some(v) if v >= 1.0 => v as u32,
            _ => 0,
        }
    }

    /// 制御用のパラメータを作る。全項目が設定済みで、検証に通ったときだけ成功する
    pub fn build(&self) -> Result<Params, BuildError> {
        if let Some(id) = self.first_unset() {
            return Err(BuildError::Unset(id));
        }
        // first_unsetがNoneなので、必要な値はすべてSome
        let v = |id: u8| self.get(id).unwrap_or(0.0);
        let invalid = |e: ConfigError| BuildError::Invalid(id_of(e));

        let c = Config {
            vdcmax: v(id::VDCMAX),
            ibase: v(id::IBASE),
            wbase: v(id::WBASE),
            ke: v(id::KE),
            cperiod: CURRENT_PERIOD,
            ckp: v(id::CKP),
            cki: v(id::CKI),
            dead_duty: v(id::DEAD_DUTY),
            i_threshold: v(id::I_THRESHOLD),
            duty_max: v(id::DUTY_MAX),
            vmax: v(id::VMAX),
            imax: v(id::IMAX),
            wperiod: OUTER_PERIOD,
            wkp: v(id::WKP),
            wki: v(id::WKI),
            wb: v(id::WB),
            wmax: v(id::WMAX),
            pkp: v(id::PKP),
            pmax: v(id::PMAX),
        };

        let cpr = v(id::ENCODER_CPR);
        // 1未満(負やほぼ0)は、整数にすると0になるか意味を持たない
        if !(cpr >= 1.0) {
            return Err(BuildError::Invalid(id::ENCODER_CPR));
        }

        let sense = SenseParam::new(c.ibase, c.vdcmax).map_err(invalid)?;
        Ok(Params {
            inner: InnerParams { current: CurrentParam::new(&c).map_err(invalid)?, sense },
            outer: OuterParams {
                velocity: VelocityParam::new(&c).map_err(invalid)?,
                position: PositionParam::new(&c).map_err(invalid)?,
                encoder: EncoderParam::new(
                    cpr as u32,
                    v(id::ENCODER_REVERSED) != 0.0,
                    OUTER_PERIOD,
                    c.wbase,
                    v(id::W_FILTER_ALPHA),
                )
                .map_err(invalid)?,
                sense,
                scale: Scale {
                    ibase: c.ibase,
                    wbase: c.wbase,
                    vdcmax: c.vdcmax,
                    accel_to_current: v(id::ACCEL_TO_CURRENT),
                },
            },
        })
    }
}


/// ほかのモジュールのテストからも使う、全項目を設定済みのSettings
#[cfg(test)]
pub(crate) mod testutil {
    use super::*;

    /// 基準値は V_b=24V, I_b=10A, ω_b=300rad/s。md-coreのテストと同じ値
    pub(crate) fn full_settings() -> Settings {
        let mut s = Settings::new();
        for (id, value) in [
            (id::VDCMAX, 24.0),
            (id::IBASE, 10.0),
            (id::WBASE, 300.0),
            (id::KE, 0.02),
            (id::CKP, 6.0),
            (id::CKI, 1000.0),
            (id::DEAD_DUTY, 0.02),
            (id::I_THRESHOLD, 0.5),
            (id::DUTY_MAX, 0.95),
            (id::VMAX, 20.0),
            (id::IMAX, 8.0),
            (id::WKP, 0.05),
            (id::WKI, 0.5),
            (id::WB, 0.8),
            (id::WMAX, 250.0),
            (id::PKP, 20.0),
            (id::PMAX, 100.0),
            (id::ACCEL_TO_CURRENT, 0.01),
            (id::ENCODER_CPR, 8192.0),
            (id::W_FILTER_ALPHA, 0.2),
            (id::ENCODER_REVERSED, 0.0),
        ] {
            s.set(id, value).unwrap();
        }
        s
    }
}

#[cfg(test)]
mod tests {
    use super::testutil::full_settings;
    use super::*;

    fn assert_close(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < 1e-5, "left: {actual}, right: {expected}");
    }

    #[test]
    fn control_periods_are_consistent() {
        assert_close(CURRENT_PERIOD * OUTER_DIV as f32, OUTER_PERIOD);
        // 72MHz、センターアラインで20kHz
        assert_eq!(PWM_ARR as u32 * 2 * 20_000, 72_000_000);
    }

    #[test]
    fn nothing_is_set_at_start() {
        let s = Settings::new();
        assert_eq!(s.get(id::VDCMAX), None);
        assert_eq!(s.first_unset(), Some(id::VDCMAX));
        assert_eq!(s.build().err(), Some(BuildError::Unset(id::VDCMAX)));
    }

    #[test]
    fn set_stores_value() {
        let mut s = Settings::new();
        assert_eq!(s.set(id::CKP, 6.0), Ok(()));
        assert_eq!(s.get(id::CKP), Some(6.0));
        assert_eq!(s.set(id::STATUS_PERIOD_MS, 10.0), Ok(()));
        assert_eq!(s.get(id::STATUS_PERIOD_MS), Some(10.0));
    }

    #[test]
    fn set_rejects_unknown_id() {
        let mut s = Settings::new();
        assert_eq!(s.set(0x11, 1.0), Err(SetError::UnknownId));
        assert_eq!(s.set(0x1F, 1.0), Err(SetError::UnknownId));
        assert_eq!(s.set(0x25, 1.0), Err(SetError::UnknownId));
        assert_eq!(s.set(0xFF, 1.0), Err(SetError::UnknownId));
        assert_eq!(s.get(0x11), None);
    }

    #[test]
    fn set_rejects_nan_and_infinity_without_storing() {
        let mut s = Settings::new();
        s.set(id::CKP, 6.0).unwrap();
        assert_eq!(s.set(id::CKP, f32::NAN), Err(SetError::NotFinite));
        assert_eq!(s.set(id::CKP, f32::INFINITY), Err(SetError::NotFinite));
        assert_eq!(s.set(id::CKP, f32::NEG_INFINITY), Err(SetError::NotFinite));
        assert_eq!(s.get(id::CKP), Some(6.0));
    }

    #[test]
    fn first_unset_reports_smallest_missing_id() {
        let mut s = Settings::new();
        for i in 0x00..=0x10u8 {
            s.set(i, 1.0).unwrap();
        }
        assert_eq!(s.first_unset(), Some(id::ACCEL_TO_CURRENT));
        s.set(id::ACCEL_TO_CURRENT, 0.0).unwrap();
        s.set(id::ENCODER_CPR, 8192.0).unwrap();
        s.set(id::ENCODER_REVERSED, 0.0).unwrap();
        assert_eq!(s.first_unset(), Some(id::W_FILTER_ALPHA));
    }

    #[test]
    fn status_period_is_not_required() {
        let s = full_settings();
        assert_eq!(s.get(id::STATUS_PERIOD_MS), None);
        assert_eq!(s.first_unset(), None);
        assert!(s.build().is_ok());
    }

    #[test]
    fn status_period_defaults_to_stopped() {
        let mut s = Settings::new();
        assert_eq!(s.status_period_ms(), 0);
        s.set(id::STATUS_PERIOD_MS, 10.0).unwrap();
        assert_eq!(s.status_period_ms(), 10);
        s.set(id::STATUS_PERIOD_MS, 2.9).unwrap();
        assert_eq!(s.status_period_ms(), 2);
        s.set(id::STATUS_PERIOD_MS, 0.5).unwrap();
        assert_eq!(s.status_period_ms(), 0);
        s.set(id::STATUS_PERIOD_MS, -5.0).unwrap();
        assert_eq!(s.status_period_ms(), 0);
    }

    #[test]
    fn build_reports_invalid_field() {
        let mut s = full_settings();
        s.set(id::CKP, 0.0).unwrap();
        assert_eq!(s.build().err(), Some(BuildError::Invalid(id::CKP)));

        // 1 * 300 / 10 = 30 はQ3.28に入らない
        let mut s = full_settings();
        s.set(id::WKP, 1.0).unwrap();
        assert_eq!(s.build().err(), Some(BuildError::Invalid(id::WKP)));

        let mut s = full_settings();
        s.set(id::PMAX, -1.0).unwrap();
        assert_eq!(s.build().err(), Some(BuildError::Invalid(id::PMAX)));

        let mut s = full_settings();
        s.set(id::IBASE, 0.0).unwrap();
        assert_eq!(s.build().err(), Some(BuildError::Invalid(id::IBASE)));
    }

    #[test]
    fn build_reports_invalid_encoder_settings() {
        let mut s = full_settings();
        s.set(id::ENCODER_CPR, 0.5).unwrap();
        assert_eq!(s.build().err(), Some(BuildError::Invalid(id::ENCODER_CPR)));

        let mut s = full_settings();
        s.set(id::ENCODER_CPR, -8192.0).unwrap();
        assert_eq!(s.build().err(), Some(BuildError::Invalid(id::ENCODER_CPR)));

        let mut s = full_settings();
        s.set(id::W_FILTER_ALPHA, 2.0).unwrap();
        assert_eq!(s.build().err(), Some(BuildError::Invalid(id::W_FILTER_ALPHA)));
    }

    #[test]
    fn build_error_exposes_id() {
        assert_eq!(BuildError::Unset(0x03).id(), 0x03);
        assert_eq!(BuildError::Invalid(0x22).id(), 0x22);
    }

    #[test]
    fn scale_converts_physical_units_to_per_unit() {
        let scale = full_settings().build().ok().unwrap().outer.scale;
        assert_close(scale.current_pu(5.0).unwrap().to_f32(), 0.5);
        assert_close(scale.velocity_pu(-150.0).unwrap().to_f32(), -0.5);
        // 100 rad/s² * 0.01 A/(rad/s²) = 1A = 0.1pu
        assert_close(scale.accel_ff_pu(100.0).unwrap().to_f32(), 0.1);
    }

    #[test]
    fn scale_rejects_values_out_of_range() {
        let scale = full_settings().build().ok().unwrap().outer.scale;
        assert_eq!(scale.current_pu(1000.0), None);
        assert_eq!(scale.velocity_pu(f32::NAN), None);
        assert_eq!(scale.accel_ff_pu(f32::INFINITY), None);
    }

    #[test]
    fn scale_converts_per_unit_back_to_physical_units() {
        let scale = full_settings().build().ok().unwrap().outer.scale;
        let half = Q3_28::checked_from_f32(0.5).unwrap();
        assert_close(scale.current_a(half), 5.0);
        assert_close(scale.velocity_rad_s(half), 150.0);
        assert_close(scale.voltage_v(half), 12.0);
    }

    #[test]
    fn position_wire_format_is_q16_16() {
        assert_close(position_from_wire(98304).to_f32(), 1.5);
        assert_eq!(position_to_wire(position_from_wire(98304)), 98304);
        assert_eq!(position_to_wire(position_from_wire(-1)), -1);
        assert_eq!(position_to_wire(position_from_wire(i32::MIN)), i32::MIN);
    }
}
