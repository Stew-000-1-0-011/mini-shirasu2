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
        // 位置と速度は駆動軸(ホイール)基準。モーターはRZ-735VA-8519、減速比 52/12 * 52/19 = 11.86
        (id::WBASE, 300.0),
        // モーター単体で約0.0082 V/(rad/s)(18Vで無負荷20400rpmから)。駆動軸基準では減速比を掛ける。
        // 大きすぎると逆起電力を補償しすぎるので、少し小さめにしてある
        (id::KE, 0.09),
        // このモーターは電気的時定数(L/R)がPWMの1周期ほどしかなく、電流が1周期で電圧に追従する。
        // 比例ゲインが巻線抵抗(約0.18Ω)に近いと1周期おきに振動する(0.3で振動した)ので、
        // 比例は小さくして、積分で追従させる
        (id::CKP, 0.02),
        (id::CKI, 600.0),
        // デッドタイムを測るまでは補償しない
        (id::DEAD_DUTY, 0.0),
        (id::I_THRESHOLD, 0.5),
        (id::DUTY_MAX, 0.95),
        // 電流の読みが外れても電流が上がりすぎないよう、電圧で抑える。1Vでは回らなかった。
        // 3Vなら、巻線抵抗が約0.2Ωとして軸が止まっていても15A程度まで
        (id::VMAX, 3.0),
        (id::IMAX, 8.0),
        (id::WKP, 0.05),
        (id::WKI, 0.5),
        (id::WB, 0.8),
        (id::WMAX, 250.0),
        (id::PKP, 20.0),
        (id::PMAX, 100.0),
        (id::ACCEL_TO_CURRENT, 0.01),
        // 駆動軸1回転あたり。エンコーダ(AMT102-V、2048PPRで4逓倍後8192)は中間軸にあり、
        // 駆動軸との間が 52/19。8192 * 52 / 19 = 22419.4
        (id::ENCODER_CPR, 22419.0),
        (id::W_FILTER_ALPHA, 0.2),
        // 正の電流で速度が負に出たので、エンコーダの向きを反転する
        (id::ENCODER_REVERSED, 1.0),
    ],
    mode: 1,
    target: Message::TargetCurrent { current: 5.0 },
    run_ms: 2000,
};

/// Benchタスクを起こす周期[ms]。1回につきコマンドを1つ進める
pub const TICK_MS: u32 = 10;
/// 状態をログに出す周期[ms]
pub const LOG_PERIOD_MS: u32 = 100;

/// 記録する点数。電流制御の毎周期(50us)に1点なので、400点で20ms
pub const CAPTURE_LEN: usize = 400;

/// 電流制御の1周期ぶんの記録
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Sample {
    /// 電流のADC値。オフセットを引いて、向きの符号を付けたもの
    pub isense: i16,
    /// 書いたコンペア値。PWMA側が正、PWMB側が負
    pub compare: i16,
}

/// 目標値を与えた直後の電流の波形を記録する。いっぱいになったら止まる
pub struct Capture {
    buf: [Sample; CAPTURE_LEN],
    len: usize,
    armed: bool,
}

impl Capture {
    pub const fn new() -> Capture {
        Capture { buf: [Sample { isense: 0, compare: 0 }; CAPTURE_LEN], len: 0, armed: false }
    }

    /// 記録を最初からやり直す
    pub fn arm(&mut self) {
        self.len = 0;
        self.armed = true;
    }

    /// armしてあり、空きがあれば記録する
    pub fn push(&mut self, sample: Sample) {
        if self.armed && self.len < CAPTURE_LEN {
            self.buf[self.len] = sample;
            self.len += 1;
        }
    }

    /// start番目からoutに写し、写した数を返す
    pub fn read(&self, start: usize, out: &mut [Sample]) -> usize {
        let end = self.len.min(start.saturating_add(out.len()));
        let n = end.saturating_sub(start);
        out[..n].copy_from_slice(&self.buf[start.min(end)..end]);
        n
    }
}

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

    fn sample(n: i16) -> Sample {
        Sample { isense: n, compare: -n }
    }

    #[test]
    fn capture_ignores_samples_until_armed() {
        let mut c = Capture::new();
        c.push(sample(1));
        let mut out = [sample(0); 4];
        assert_eq!(c.read(0, &mut out), 0);
    }

    #[test]
    fn capture_reads_back_in_chunks() {
        let mut c = Capture::new();
        c.arm();
        for n in 0..6 {
            c.push(sample(n));
        }
        let mut out = [sample(0); 4];
        assert_eq!(c.read(0, &mut out), 4);
        assert_eq!(out, [sample(0), sample(1), sample(2), sample(3)]);
        assert_eq!(c.read(4, &mut out), 2);
        assert_eq!(out[..2], [sample(4), sample(5)]);
        assert_eq!(c.read(6, &mut out), 0);
        assert_eq!(c.read(100, &mut out), 0);
    }

    #[test]
    fn capture_stops_when_full() {
        let mut c = Capture::new();
        c.arm();
        for n in 0..CAPTURE_LEN as i16 + 10 {
            c.push(sample(n));
        }
        let mut out = [sample(0); 4];
        assert_eq!(c.read(CAPTURE_LEN - 2, &mut out), 2);
        assert_eq!(out[..2], [sample(CAPTURE_LEN as i16 - 2), sample(CAPTURE_LEN as i16 - 1)]);
    }

    #[test]
    fn arming_again_discards_old_samples() {
        let mut c = Capture::new();
        c.arm();
        c.push(sample(1));
        c.push(sample(2));
        c.arm();
        c.push(sample(7));
        let mut out = [sample(0); 4];
        assert_eq!(c.read(0, &mut out), 1);
        assert_eq!(out[0], sample(7));
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
