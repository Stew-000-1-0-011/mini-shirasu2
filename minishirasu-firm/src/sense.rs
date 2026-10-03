//! ADCの生値とエンコーダのカウントを、制御器が使う値にする

use md_core::controller::ConfigError;
use md_core::fixed::Q3_28;

/// ADCの基準電圧[V]
const VREF: f32 = 3.3;
/// 12bit ADCの分解能
const ADC_STEPS: f32 = 4096.0;
/// シャント抵抗[Ω]
const SHUNT_OHM: f32 = 0.002;
/// 電流センスアンプ(INA2181A1)のゲイン
const AMP_GAIN: f32 = 20.0;
/// 母線電圧の分圧比の逆数。(10k + 1k) / 1k
const VSENSE_RATIO: f32 = 11.0;

#[derive(Clone, Copy)]
pub struct SenseParam {
    i_gain: Q3_28, // 1LSBあたりの電流[pu]
    v_gain: Q3_28, // 1LSBあたりの母線電圧[pu]
}

fn gain(name: &'static str, lsb: f32, base: f32) -> Result<Q3_28, ConfigError> {
    if !(base > 0.0 && base.is_finite()) {
        return Err(ConfigError::NotPositive(name));
    }
    Q3_28::checked_from_f32(lsb / base).ok_or(ConfigError::OutOfRange(name))
}

impl SenseParam {
    /// ibase: 電流の基準値[A]。vdcmax: 電圧の基準値[V]
    pub fn new(ibase: f32, vdcmax: f32) -> Result<SenseParam, ConfigError> {
        Ok(SenseParam {
            i_gain: gain("ibase", VREF / ADC_STEPS / (SHUNT_OHM * AMP_GAIN), ibase)?,
            v_gain: gain("vdcmax", VREF / ADC_STEPS * VSENSE_RATIO, vdcmax)?,
        })
    }

    /// モーター電流[pu]。シャントにはON期間だけ同じ向きの電流が流れるので、
    /// その周期に駆動していた向き(sign: 1, -1, 0)で符号を復元する
    pub fn current(&self, raw: u16, offset: u16, sign: i32) -> Q3_28 {
        self.i_gain.mul_int((raw as i32 - offset as i32) * sign)
    }

    /// 母線電圧[pu]
    pub fn bus_voltage(&self, raw: u16) -> Q3_28 {
        self.v_gain.mul_int(raw as i32)
    }
}

/// 16bitカウンタの前回値と今回値から、符号付きの差分を求める。
/// 1周期に±32767カウントを超えて進まないことが前提
pub fn count_delta(prev: u16, now: u16) -> i32 {
    now.wrapping_sub(prev) as i16 as i32
}


#[cfg(test)]
mod tests {
    use super::*;

    fn param() -> SenseParam {
        SenseParam::new(20.0, 36.0).unwrap()
    }

    fn assert_near(actual: Q3_28, expected: f32) {
        let a = actual.to_f32();
        assert!((a - expected).abs() < 1e-4, "left: {a}, right: {expected}");
    }

    #[test]
    fn current_scales_offset_corrected_raw_value() {
        // 1LSB = 3.3 / 4096 / (0.002 * 20) = 0.020142 A。2000LSBで40.28A = 2.0142pu
        assert_near(param().current(2146, 146, 1), 2.0141602);
    }

    #[test]
    fn current_sign_follows_duty_direction() {
        assert_near(param().current(2146, 146, -1), -2.0141602);
    }

    #[test]
    fn current_is_zero_when_no_side_is_driven() {
        assert_eq!(param().current(2146, 146, 0), Q3_28::ZERO);
    }

    #[test]
    fn current_below_offset_is_negative() {
        // 回生でシャントに逆向きの電流が流れると、オフセットより下がる
        assert_near(param().current(46, 146, 1), -0.10070801);
    }

    #[test]
    fn bus_voltage_scales_divider() {
        // 2048LSB = 1.65V、11倍して18.15V。基準36Vで0.50417pu
        assert_near(param().bus_voltage(2048), 0.50416666);
        assert_eq!(param().bus_voltage(0), Q3_28::ZERO);
    }

    #[test]
    fn new_rejects_non_positive_bases() {
        assert_eq!(SenseParam::new(0.0, 36.0).err(), Some(ConfigError::NotPositive("ibase")));
        assert_eq!(SenseParam::new(20.0, f32::NAN).err(), Some(ConfigError::NotPositive("vdcmax")));
        assert_eq!(SenseParam::new(f32::INFINITY, 36.0).err(), Some(ConfigError::NotPositive("ibase")));
    }

    #[test]
    fn count_delta_is_signed_difference() {
        assert_eq!(count_delta(100, 130), 30);
        assert_eq!(count_delta(130, 100), -30);
        assert_eq!(count_delta(5, 5), 0);
    }

    #[test]
    fn count_delta_unwraps_16bit_counter() {
        // 0xFFFFから0へ、0から0xFFFFへまたいでも連続した差分になる
        assert_eq!(count_delta(0xFFF0, 0x0010), 32);
        assert_eq!(count_delta(0x0010, 0xFFF0), -32);
    }
}
