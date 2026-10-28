//! 符号付きデューティを、DRV8701(PH/EN方式)用のタイマのコンペア値にする

use md_core::fixed::Q3_28;

/// TIM1のコンペア値。enはEN(PA8, CH1)、phはPH(PA9, CH2)
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Compare {
    /// ENのPWM。デューティの大きさ
    pub en: u16,
    /// PHのレベル。0なら常にLow、ARR+1なら常にHigh。
    /// CCRはプリロードなので、向きの切り替えもENと同じ山で反映され、ONパルスの途中では変わらない
    pub ph: u16,
}

impl Compare {
    /// 駆動している向き。PH Highなら1、PH Lowなら-1、ENが0なら0
    pub fn sign(&self) -> i32 {
        if self.en == 0 {
            0
        } else if self.ph > 0 {
            1
        } else {
            -1
        }
    }

    /// 向きつきのENのカウント。記録用
    pub fn signed(&self) -> i32 {
        self.sign() * self.en as i32
    }
}

/// 符号付きデューティ(-1..=1)をコンペア値にする。
/// 正ならPHをHigh、負ならLowにして、ENを|duty|でPWMする。|duty| > 1 は arr で頭打ち。
/// ENがLowの間、DRV8701は両ローサイドをONにする(ブレーキ)
pub fn duty_to_compare(duty: Q3_28, arr: u16) -> Compare {
    let n = duty.scale_int(arr as i32);
    let en = n.unsigned_abs().min(arr as u32) as u16;
    let ph = if n > 0 { arr + 1 } else { 0 };
    Compare { en, ph }
}


#[cfg(test)]
mod tests {
    use super::*;

    fn q(v: f32) -> Q3_28 {
        Q3_28::checked_from_f32(v).unwrap()
    }

    #[test]
    fn positive_duty_pwms_en_with_ph_high() {
        // PHはARR+1で常にHigh
        assert_eq!(duty_to_compare(q(0.5), 1800), Compare { en: 900, ph: 1801 });
    }

    #[test]
    fn negative_duty_pwms_en_with_ph_low() {
        assert_eq!(duty_to_compare(q(-0.25), 1800), Compare { en: 450, ph: 0 });
    }

    #[test]
    fn zero_duty_keeps_en_low() {
        // ENがLowの間はブレーキ(両ローサイドON)。PHはどちらでもよいのでLowにする
        assert_eq!(duty_to_compare(Q3_28::ZERO, 1800), Compare { en: 0, ph: 0 });
    }

    #[test]
    fn duty_beyond_one_is_clamped_to_period() {
        assert_eq!(duty_to_compare(q(2.0), 1800), Compare { en: 1800, ph: 1801 });
        assert_eq!(duty_to_compare(Q3_28::MIN, 1800), Compare { en: 1800, ph: 0 });
    }

    #[test]
    fn duty_too_small_to_represent_becomes_zero() {
        // 1カウントの半分に満たないデューティは0に丸まる。向きも残さない
        assert_eq!(duty_to_compare(q(0.0001), 1800), Compare { en: 0, ph: 0 });
        assert_eq!(duty_to_compare(q(-0.0001), 1800), Compare { en: 0, ph: 0 });
    }

    #[test]
    fn sign_follows_driven_direction() {
        assert_eq!(Compare { en: 900, ph: 1801 }.sign(), 1);
        assert_eq!(Compare { en: 450, ph: 0 }.sign(), -1);
        assert_eq!(Compare { en: 0, ph: 0 }.sign(), 0);
        assert_eq!(Compare { en: 0, ph: 1801 }.sign(), 0);
    }

    #[test]
    fn signed_counts_carry_direction() {
        assert_eq!(Compare { en: 900, ph: 1801 }.signed(), 900);
        assert_eq!(Compare { en: 450, ph: 0 }.signed(), -450);
        assert_eq!(Compare { en: 0, ph: 0 }.signed(), 0);
    }
}
