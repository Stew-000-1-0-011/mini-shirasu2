//! 目標値と、位置 -> 速度 -> 電流目標の合成

use md_core::controller::{Measurement, PositionState, Saturated, VelocityState};
use md_core::fixed::{Q3_28, Q16_16};

use crate::config::{self, OuterParams, Scale};
use crate::protocol::Message;
use crate::state::Mode;

/// 目標値。どの量かが制御モードに対応する
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Target {
    Current(Q3_28),
    Velocity(Q3_28),
    Position(Q16_16),
    /// 電流制御を通さずにPWMへ出すデューティ
    Duty(Q3_28),
}

/// すべてper-unit
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Setpoint {
    pub target: Target,
    /// 速度FF。位置モードで位置Pの出力に足す
    pub w_ff: Q3_28,
    /// トルクFF。速度・位置モードで速度PIの出力に足す
    pub i_ff: Q3_28,
}

impl Setpoint {
    /// モードに入った直後の目標値。電流0、速度0、または現在位置を保つ
    pub fn hold(mode: Mode, th: Q16_16) -> Setpoint {
        let target = match mode {
            Mode::Velocity => Target::Velocity(Q3_28::ZERO),
            Mode::Position => Target::Position(th),
            Mode::Duty => Target::Duty(Q3_28::ZERO),
            _ => Target::Current(Q3_28::ZERO),
        };
        Setpoint { target, w_ff: Q3_28::ZERO, i_ff: Q3_28::ZERO }
    }
}

/// 目標値のメッセージをper-unitにする。
/// 現在のモードと合わない、値が範囲外、目標値のメッセージでない、のどれかならNone
pub fn setpoint_from_message(message: &Message, mode: Mode, scale: &Scale) -> Option<Setpoint> {
    match (*message, mode) {
        (Message::TargetCurrent { current }, Mode::Current) => Some(Setpoint {
            target: Target::Current(scale.current_pu(current)?),
            w_ff: Q3_28::ZERO,
            i_ff: Q3_28::ZERO,
        }),
        (Message::TargetVelocity { velocity, accel_ff }, Mode::Velocity) => Some(Setpoint {
            target: Target::Velocity(scale.velocity_pu(velocity)?),
            w_ff: Q3_28::ZERO,
            i_ff: scale.accel_ff_pu(accel_ff)?,
        }),
        (Message::TargetPosition { position, velocity_ff, accel_ff }, Mode::Position) => Some(Setpoint {
            target: Target::Position(config::position_from_wire(position)),
            w_ff: scale.velocity_pu(velocity_ff)?,
            i_ff: scale.accel_ff_pu(accel_ff)?,
        }),
        (Message::TargetDuty { duty }, Mode::Duty) => {
            let duty = Q3_28::checked_from_f32(duty)?;
            if duty > Q3_28::ONE || duty < -Q3_28::ONE {
                return None;
            }
            Some(Setpoint { target: Target::Duty(duty), w_ff: Q3_28::ZERO, i_ff: Q3_28::ZERO })
        }
        _ => None,
    }
}

/// 外側ループから電流ループへの指令
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Drive {
    /// 電流目標[pu]。電流制御をする
    Current(Q3_28),
    /// デューティ。電流制御をせずにそのままPWMへ出す
    Duty(Q3_28),
}

impl Drive {
    /// モードに入った直後の指令。外側ループが最初の目標値を出すまでのあいだ使う
    pub fn hold(mode: Mode) -> Drive {
        match mode {
            Mode::Duty => Drive::Duty(Q3_28::ZERO),
            _ => Drive::Current(Q3_28::ZERO),
        }
    }
}

/// 位置P -> (+速度FF) -> 速度PI -> (+トルクFF) -> 電流目標
pub struct Cascade {
    velocity: VelocityState,
    position: PositionState,
}

impl Cascade {
    pub fn new() -> Cascade {
        Cascade { velocity: VelocityState::new(), position: PositionState::new() }
    }

    pub fn reset(&mut self) {
        self.velocity.reset();
    }

    /// 電流ループへの指令を返す。デューティ目標はそのまま、それ以外は電流目標にする
    pub fn drive(&mut self, p: &OuterParams, sp: &Setpoint, m: &Measurement, saturated: Saturated) -> Drive {
        match sp.target {
            Target::Duty(d) => Drive::Duty(d),
            _ => Drive::Current(self.update(p, sp, m, saturated)),
        }
    }

    /// 電流目標[pu]を返す。saturatedは電流ループが前回報告した飽和。デューティ目標では0
    pub fn update(&mut self, p: &OuterParams, sp: &Setpoint, m: &Measurement, saturated: Saturated) -> Q3_28 {
        match sp.target {
            Target::Current(i) => i,
            Target::Duty(_) => Q3_28::ZERO,
            Target::Velocity(w) => self.velocity.update(&p.velocity, w, m, saturated) + sp.i_ff,
            Target::Position(th) => {
                let w = self.position.update(&p.position, th, m) + sp.w_ff;
                self.velocity.update(&p.velocity, w, m, saturated) + sp.i_ff
            }
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::testutil::full_settings;

    fn q(v: f32) -> Q3_28 {
        Q3_28::checked_from_f32(v).unwrap()
    }

    fn q16(v: f32) -> Q16_16 {
        Q16_16::checked_from_f32(v).unwrap()
    }

    fn assert_close(actual: Q3_28, expected: f32) {
        let a = actual.to_f32();
        assert!((a - expected).abs() < 1e-5, "left: {a}, right: {expected}");
    }

    /// 速度PIは kp=1.5, ki=0.015, b=0.8。位置Pは kp=0.41887903
    fn outer() -> OuterParams {
        full_settings().build().ok().unwrap().outer
    }

    fn meas(w: f32, th: f32) -> Measurement {
        Measurement { vdc: q(0.5), vdc_inv: q(2.0), i: Q3_28::ZERO, w: q(w), th: q16(th) }
    }

    fn setpoint(target: Target, w_ff: f32, i_ff: f32) -> Setpoint {
        Setpoint { target, w_ff: q(w_ff), i_ff: q(i_ff) }
    }

    // ---- 目標値 ----

    #[test]
    fn hold_keeps_the_motor_where_it_is() {
        let th = q16(12.5);
        let zero = Q3_28::ZERO;
        assert_eq!(Setpoint::hold(Mode::Current, th), Setpoint { target: Target::Current(zero), w_ff: zero, i_ff: zero });
        assert_eq!(Setpoint::hold(Mode::Velocity, th), Setpoint { target: Target::Velocity(zero), w_ff: zero, i_ff: zero });
        assert_eq!(Setpoint::hold(Mode::Position, th), Setpoint { target: Target::Position(th), w_ff: zero, i_ff: zero });
    }

    #[test]
    fn target_messages_convert_to_per_unit() {
        let scale = outer().scale;

        let sp = setpoint_from_message(&Message::TargetCurrent { current: 5.0 }, Mode::Current, &scale).unwrap();
        assert_eq!(sp.target, Target::Current(q(0.5)));
        assert_eq!((sp.w_ff, sp.i_ff), (Q3_28::ZERO, Q3_28::ZERO));

        // 150 rad/s = 0.5pu。加速度200 rad/s² * 0.01 = 2A = 0.2pu
        let m = Message::TargetVelocity { velocity: 150.0, accel_ff: 200.0 };
        let sp = setpoint_from_message(&m, Mode::Velocity, &scale).unwrap();
        assert_eq!(sp.target, Target::Velocity(q(0.5)));
        assert_eq!(sp.w_ff, Q3_28::ZERO);
        assert_close(sp.i_ff, 0.2);

        // 98304 = 1.5回転。速度FF 30 rad/s = 0.1pu
        let m = Message::TargetPosition { position: 98304, velocity_ff: 30.0, accel_ff: -100.0 };
        let sp = setpoint_from_message(&m, Mode::Position, &scale).unwrap();
        assert_eq!(sp.target, Target::Position(q16(1.5)));
        assert_close(sp.w_ff, 0.1);
        assert_close(sp.i_ff, -0.1);
    }

    #[test]
    fn target_for_another_mode_is_ignored() {
        let scale = outer().scale;
        let current = Message::TargetCurrent { current: 1.0 };
        let velocity = Message::TargetVelocity { velocity: 1.0, accel_ff: 0.0 };
        let position = Message::TargetPosition { position: 0, velocity_ff: 0.0, accel_ff: 0.0 };

        assert_eq!(setpoint_from_message(&velocity, Mode::Current, &scale), None);
        assert_eq!(setpoint_from_message(&position, Mode::Velocity, &scale), None);
        assert_eq!(setpoint_from_message(&current, Mode::Position, &scale), None);
        // 出力無効中や異常中は、どの目標値も受け取らない
        for m in [current, velocity, position] {
            assert_eq!(setpoint_from_message(&m, Mode::Disabled, &scale), None);
            assert_eq!(setpoint_from_message(&m, Mode::Fault, &scale), None);
        }
    }

    #[test]
    fn target_out_of_range_is_ignored() {
        let scale = outer().scale;
        // 1000A = 100pu はQ3.28に入らない
        assert_eq!(setpoint_from_message(&Message::TargetCurrent { current: 1000.0 }, Mode::Current, &scale), None);
        assert_eq!(setpoint_from_message(&Message::TargetCurrent { current: f32::NAN }, Mode::Current, &scale), None);
        // FFだけが範囲外でも、目標値ごと捨てる
        let m = Message::TargetVelocity { velocity: 1.0, accel_ff: f32::INFINITY };
        assert_eq!(setpoint_from_message(&m, Mode::Velocity, &scale), None);
        let m = Message::TargetPosition { position: 0, velocity_ff: 1e9, accel_ff: 0.0 };
        assert_eq!(setpoint_from_message(&m, Mode::Position, &scale), None);
    }

    #[test]
    fn non_target_message_is_ignored() {
        let scale = outer().scale;
        assert_eq!(setpoint_from_message(&Message::SetMode { mode: 1 }, Mode::Current, &scale), None);
    }

    #[test]
    fn hold_starts_duty_mode_at_zero() {
        let zero = Q3_28::ZERO;
        assert_eq!(Setpoint::hold(Mode::Duty, q16(3.0)), Setpoint { target: Target::Duty(zero), w_ff: zero, i_ff: zero });
    }

    #[test]
    fn duty_target_is_taken_as_is_in_duty_mode() {
        let scale = outer().scale;
        let sp = setpoint_from_message(&Message::TargetDuty { duty: -0.5 }, Mode::Duty, &scale).unwrap();
        assert_eq!(sp, Setpoint { target: Target::Duty(q(-0.5)), w_ff: Q3_28::ZERO, i_ff: Q3_28::ZERO });
        let sp = setpoint_from_message(&Message::TargetDuty { duty: 1.0 }, Mode::Duty, &scale).unwrap();
        assert_eq!(sp.target, Target::Duty(Q3_28::ONE));
    }

    #[test]
    fn duty_target_out_of_range_is_ignored() {
        let scale = outer().scale;
        for duty in [1.01, -1.01, 5.0, f32::NAN, f32::INFINITY] {
            assert_eq!(setpoint_from_message(&Message::TargetDuty { duty }, Mode::Duty, &scale), None, "{duty}");
        }
    }

    #[test]
    fn duty_target_and_duty_mode_do_not_mix_with_others() {
        let scale = outer().scale;
        let duty = Message::TargetDuty { duty: 0.1 };
        for mode in [Mode::Disabled, Mode::Current, Mode::Velocity, Mode::Position, Mode::Fault] {
            assert_eq!(setpoint_from_message(&duty, mode, &scale), None);
        }
        let current = Message::TargetCurrent { current: 1.0 };
        assert_eq!(setpoint_from_message(&current, Mode::Duty, &scale), None);
    }

    #[test]
    fn drive_hold_matches_mode() {
        assert_eq!(Drive::hold(Mode::Duty), Drive::Duty(Q3_28::ZERO));
        assert_eq!(Drive::hold(Mode::Current), Drive::Current(Q3_28::ZERO));
        assert_eq!(Drive::hold(Mode::Position), Drive::Current(Q3_28::ZERO));
    }

    // ---- カスケード ----

    #[test]
    fn drive_passes_duty_through_and_wraps_current_target() {
        let p = outer();
        let mut c = Cascade::new();
        let sp = setpoint(Target::Duty(q(-0.4)), 0.0, 0.0);
        assert_eq!(c.drive(&p, &sp, &meas(0.25, 0.0), Saturated::NotSaturated), Drive::Duty(q(-0.4)));
        let sp = setpoint(Target::Current(q(0.3)), 0.0, 0.0);
        assert_eq!(c.drive(&p, &sp, &meas(0.25, 0.0), Saturated::NotSaturated), Drive::Current(q(0.3)));
    }

    #[test]
    fn current_mode_passes_target_through() {
        let p = outer();
        let mut c = Cascade::new();
        let sp = setpoint(Target::Current(q(0.3)), 0.0, 0.0);
        assert_close(c.update(&p, &sp, &meas(0.25, 0.0), Saturated::NotSaturated), 0.3);
    }

    #[test]
    fn velocity_mode_adds_torque_feedforward() {
        let p = outer();
        let mut c = Cascade::new();
        let sp = setpoint(Target::Velocity(q(0.5)), 0.0, 0.1);
        // 1.5 * (0.8 * 0.5 - 0.25) + 0.1 = 0.325
        assert_close(c.update(&p, &sp, &meas(0.25, 0.0), Saturated::NotSaturated), 0.325);
    }

    #[test]
    fn velocity_mode_integrates_error() {
        let p = outer();
        let mut c = Cascade::new();
        let sp = setpoint(Target::Velocity(q(0.5)), 0.0, 0.0);
        assert_close(c.update(&p, &sp, &meas(0.25, 0.0), Saturated::NotSaturated), 0.225);
        // 積分が 0.015 * 0.25 = 0.00375 だけ溜まる
        assert_close(c.update(&p, &sp, &meas(0.25, 0.0), Saturated::NotSaturated), 0.22875);
    }

    #[test]
    fn saturation_freezes_velocity_integral() {
        let p = outer();
        let mut c = Cascade::new();
        let sp = setpoint(Target::Velocity(q(0.5)), 0.0, 0.0);
        // 電流ループが上側に飽和していて誤差が正なら、積分を進めない
        assert_close(c.update(&p, &sp, &meas(0.25, 0.0), Saturated::Overflow), 0.225);
        assert_close(c.update(&p, &sp, &meas(0.25, 0.0), Saturated::Overflow), 0.225);
    }

    #[test]
    fn position_mode_adds_velocity_and_torque_feedforward() {
        let p = outer();
        let mut c = Cascade::new();
        let sp = setpoint(Target::Position(q16(1.0)), 0.1, 0.05);
        // 速度目標 = 0.41887903 * (1.0 - 0.5) + 0.1 = 0.3094395
        // 電流目標 = 1.5 * (0.8 * 0.3094395 - 0) + 0.05 = 0.4213274
        assert_close(c.update(&p, &sp, &meas(0.0, 0.5), Saturated::NotSaturated), 0.4213274);
    }

    #[test]
    fn reset_clears_velocity_integral() {
        let p = outer();
        let mut c = Cascade::new();
        let sp = setpoint(Target::Velocity(q(0.5)), 0.0, 0.0);
        c.update(&p, &sp, &meas(0.25, 0.0), Saturated::NotSaturated);
        c.reset();
        assert_close(c.update(&p, &sp, &meas(0.25, 0.0), Saturated::NotSaturated), 0.225);
    }

    #[test]
    fn feedforward_sum_saturates() {
        let p = outer();
        let mut c = Cascade::new();
        let sp = Setpoint { target: Target::Current(Q3_28::ZERO), w_ff: Q3_28::ZERO, i_ff: Q3_28::MAX };
        // 電流モードではFFを使わない
        assert_close(c.update(&p, &sp, &meas(0.0, 0.0), Saturated::NotSaturated), 0.0);

        let sp = Setpoint { target: Target::Velocity(q(0.5)), w_ff: Q3_28::ZERO, i_ff: Q3_28::MAX };
        // 0.6 + ほぼ8 はラップせず上限に張り付く
        assert_eq!(c.update(&p, &sp, &meas(0.0, 0.0), Saturated::NotSaturated), Q3_28::MAX);
    }
}
