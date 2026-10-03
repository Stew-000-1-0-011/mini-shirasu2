//! CANなしの実機試験(feature = "bench")。試験の内容はこのファイルの定数を書き換えて決める。
//! 設定値・モード・目標値を、CANで受けたときと同じ入口に流し込む

use crate::config::id;
use crate::protocol::Message;

/// 試験の内容
pub struct Plan {
    /// SetParamで設定する値。IDと値
    pub settings: &'static [(u8, f32)],
    /// 0: 出力無効のまま(センサ値を見るだけ), 1: 電流, 2: 速度, 3: 位置
    pub mode: u8,
    /// モードに入ったあとに与える目標値。modeと合わないものは捨てられる
    pub target: Message,
    /// 目標値を与えてから出力を止めるまでの時間[ms]
    pub run_ms: u32,
}

/// 実行する試験。モーターに合わせて書き換える
pub const PLAN: Plan = Plan {
    settings: &[
        (id::VDCMAX, 24.0),
        (id::IBASE, 10.0),
        (id::WBASE, 300.0),
        (id::KE, 0.02),
        (id::CKP, 6.0),
        (id::CKI, 1000.0),
        (id::DEAD_DUTY, 0.02),
        (id::I_THRESHOLD, 0.5),
        (id::DUTY_MAX, 0.95),
        // モーター未接続でPWMを見る段階。飽和したときのデューティが 6 / 母線電圧 になる
        (id::VMAX, 6.0),
        (id::IMAX, 8.0),
        (id::WKP, 0.05),
        (id::WKI, 0.5),
        (id::WB, 0.8),
        (id::WMAX, 250.0),
        (id::PKP, 20.0),
        (id::PMAX, 100.0),
        (id::ACCEL_TO_CURRENT, 0.01),
        // 駆動軸1回転あたり。8192で駆動軸1回転が約2.7revと読めたので 8192 * 2.7(粗い実測)
        (id::ENCODER_CPR, 22118.0),
        (id::W_FILTER_ALPHA, 0.2),
        (id::ENCODER_REVERSED, 0.0),
    ],
    mode: 1,
    target: Message::TargetCurrent { current: 0.5 },
    run_ms: 5000,
};

/// Benchタスクを起こす周期[ms]。1回につきコマンドを1つ進める
pub const TICK_MS: u32 = 10;
/// 状態をログに出す周期[ms]
pub const LOG_PERIOD_MS: u32 = 100;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Action {
    /// コマンドとして処理する
    Command(Message),
    /// 目標値として与える
    Target(Message),
    Nothing,
}

/// 試験の手順。設定 -> モード -> 目標値 -> (run_ms待つ) -> 出力停止 の順に進む
pub struct Script {
    step: usize,
    /// 目標値を与えた時刻[ms]。止めたあとはNone
    started_ms: Option<u32>,
}

impl Script {
    pub const fn new() -> Script {
        Script { step: 0, started_ms: None }
    }

    /// 次にやること。TICK_MSごとに呼ぶ。nowは起動からの時間[ms]
    pub fn next(&mut self, plan: &Plan, now: u32) -> Action {
        let n = plan.settings.len();
        let step = self.step;
        if step < n {
            self.step += 1;
            let (id, value) = plan.settings[step];
            return Action::Command(Message::SetParam { id, value });
        }
        // 出力無効のままなら、設定だけして終わる
        if plan.mode == 0 {
            return Action::Nothing;
        }
        if step == n {
            self.step += 1;
            return Action::Command(Message::SetMode { mode: plan.mode });
        }
        if step == n + 1 {
            self.step += 1;
            self.started_ms = Some(now);
            return Action::Target(plan.target);
        }
        match self.started_ms {
            Some(started) if now.wrapping_sub(started) >= plan.run_ms => {
                self.started_ms = None;
                Action::Command(Message::SetMode { mode: 0 })
            }
            _ => Action::Nothing,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Settings;

    const SETTINGS: &[(u8, f32)] = &[(id::VDCMAX, 24.0), (id::IBASE, 10.0)];

    fn plan(mode: u8) -> Plan {
        Plan { settings: SETTINGS, mode, target: Message::TargetCurrent { current: 0.5 }, run_ms: 100 }
    }

    #[test]
    fn default_plan_settings_pass_validation() {
        let mut s = Settings::new();
        for &(id, value) in PLAN.settings {
            assert!(s.set(id, value).is_ok(), "id {id:#x}");
        }
        assert!(s.build().is_ok());
    }

    #[test]
    fn settings_are_sent_first_in_order() {
        let p = plan(1);
        let mut s = Script::new();
        assert_eq!(s.next(&p, 10), Action::Command(Message::SetParam { id: id::VDCMAX, value: 24.0 }));
        assert_eq!(s.next(&p, 20), Action::Command(Message::SetParam { id: id::IBASE, value: 10.0 }));
    }

    #[test]
    fn mode_zero_only_sends_settings() {
        let p = plan(0);
        let mut s = Script::new();
        s.next(&p, 10);
        s.next(&p, 20);
        for now in [30, 40, 1000, 100_000] {
            assert_eq!(s.next(&p, now), Action::Nothing);
        }
    }

    #[test]
    fn mode_then_target_then_stop_after_run_time() {
        let p = plan(1);
        let mut s = Script::new();
        s.next(&p, 10);
        s.next(&p, 20);
        assert_eq!(s.next(&p, 30), Action::Command(Message::SetMode { mode: 1 }));
        assert_eq!(s.next(&p, 40), Action::Target(Message::TargetCurrent { current: 0.5 }));
        // 目標値を与えた40msから100ms
        assert_eq!(s.next(&p, 50), Action::Nothing);
        assert_eq!(s.next(&p, 130), Action::Nothing);
        assert_eq!(s.next(&p, 140), Action::Command(Message::SetMode { mode: 0 }));
    }

    #[test]
    fn stop_is_sent_only_once() {
        let p = plan(1);
        let mut s = Script::new();
        for now in [10, 20, 30, 40, 140] {
            s.next(&p, now);
        }
        for now in [150, 160, 10_000] {
            assert_eq!(s.next(&p, now), Action::Nothing);
        }
    }
}
