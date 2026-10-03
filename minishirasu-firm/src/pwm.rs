//! 符号付きデューティをタイマのコンペア値にする

use md_core::fixed::Q3_28;

/// 符号付きデューティ(-1..=1)を、(PWMA側, PWMB側)のコンペア値にする。
/// 片側だけをPWMし、反対側は0(Low固定)にする。|duty| > 1 は arr で頭打ち
pub fn duty_to_compare(duty: Q3_28, arr: u16) -> (u16, u16) {
    let n = duty.scale_int(arr as i32);
    let magnitude = n.unsigned_abs().min(arr as u32) as u16;
    if n >= 0 { (magnitude, 0) } else { (0, magnitude) }
}

/// コンペア値の組が駆動している向き。PWMA側なら1、PWMB側なら-1、どちらも0なら0
pub fn compare_sign(compare: (u16, u16)) -> i32 {
    if compare.0 > 0 {
        1
    } else if compare.1 > 0 {
        -1
    } else {
        0
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    fn q(v: f32) -> Q3_28 {
        Q3_28::checked_from_f32(v).unwrap()
    }

    #[test]
    fn positive_duty_drives_a_side() {
        assert_eq!(duty_to_compare(q(0.5), 1800), (900, 0));
    }

    #[test]
    fn negative_duty_drives_b_side() {
        assert_eq!(duty_to_compare(q(-0.25), 1800), (0, 450));
    }

    #[test]
    fn zero_duty_keeps_both_low() {
        assert_eq!(duty_to_compare(Q3_28::ZERO, 1800), (0, 0));
    }

    #[test]
    fn duty_beyond_one_is_clamped_to_period() {
        assert_eq!(duty_to_compare(q(2.0), 1800), (1800, 0));
        assert_eq!(duty_to_compare(Q3_28::MIN, 1800), (0, 1800));
    }

    #[test]
    fn duty_too_small_to_represent_becomes_zero() {
        // 1カウントの半分に満たないデューティは0に丸まる
        assert_eq!(duty_to_compare(q(0.0001), 1800), (0, 0));
    }

    #[test]
    fn compare_sign_follows_driven_side() {
        assert_eq!(compare_sign((900, 0)), 1);
        assert_eq!(compare_sign((0, 450)), -1);
        assert_eq!(compare_sign((0, 0)), 0);
    }
}
