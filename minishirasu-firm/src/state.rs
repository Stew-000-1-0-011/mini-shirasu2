//! 状態機械。コマンドを受けたときに何をするかを決める(実行はしない)

use crate::protocol::Message;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Mode {
    /// 出力無効。起動直後はこれ
    Disabled,
    Current,
    Velocity,
    Position,
    /// 異常でラッチ。ResetFaultが来るまで抜けない
    Fault,
}

impl Mode {
    /// 通信上の値
    pub fn code(self) -> u8 {
        match self {
            Mode::Disabled => 0,
            Mode::Current => 1,
            Mode::Velocity => 2,
            Mode::Position => 3,
            Mode::Fault => 4,
        }
    }

    /// 出力を出して制御しているか
    pub fn is_active(self) -> bool {
        matches!(self, Mode::Current | Mode::Velocity | Mode::Position)
    }
}

/// Nackの理由
pub mod nack {
    /// CRC・長さ・種別が不正
    pub const FRAME: u8 = 1;
    /// 出力有効中は受け付けない
    pub const ACTIVE: u8 = 2;
    /// 設定値が未設定、または検証に通らない
    pub const CONFIG: u8 = 3;
    /// 異常でラッチ中
    pub const FAULT: u8 = 4;
    /// 値が不正(未知のモード・未知の設定ID・NaNなど)
    pub const VALUE: u8 = 5;
    /// NFAULTが解除されない
    pub const NFAULT: u8 = 6;
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Reject {
    pub reason: u8,
    /// reasonがCONFIGかVALUEのときの設定ID。それ以外は0
    pub param: u8,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Action {
    Nothing,
    Disable,
    /// 制御モードに入る。積分器と目標値を初期化して出力を有効にする
    Enter(Mode),
    SetParam { id: u8, value: f32 },
    ResetFault,
    SetOrigin,
}

fn reject(reason: u8) -> Result<Action, Reject> {
    Err(Reject { reason, param: 0 })
}

/// Disabledのときだけ受け付けるコマンド
fn only_disabled(mode: Mode, action: Action) -> Result<Action, Reject> {
    match mode {
        Mode::Disabled => Ok(action),
        Mode::Fault => reject(nack::FAULT),
        _ => reject(nack::ACTIVE),
    }
}

/// コマンドに対して何をするかを決める。
/// config_error: 制御用のパラメータが無効なら、その原因の設定ID
pub fn decide(mode: Mode, message: &Message, config_error: Option<u8>) -> Result<Action, Reject> {
    match *message {
        Message::SetMode { mode: requested } => {
            if mode == Mode::Fault {
                return reject(nack::FAULT);
            }
            let target = match requested {
                0 => return Ok(Action::Disable),
                1 => Mode::Current,
                2 => Mode::Velocity,
                3 => Mode::Position,
                _ => return reject(nack::VALUE),
            };
            match config_error {
                Some(param) => Err(Reject { reason: nack::CONFIG, param }),
                None => Ok(Action::Enter(target)),
            }
        }
        Message::SetParam { id, value } => only_disabled(mode, Action::SetParam { id, value }),
        Message::SetOrigin => only_disabled(mode, Action::SetOrigin),
        Message::ResetFault => Ok(if mode == Mode::Fault { Action::ResetFault } else { Action::Nothing }),
        _ => reject(nack::FRAME),
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [Mode; 5] = [Mode::Disabled, Mode::Current, Mode::Velocity, Mode::Position, Mode::Fault];

    fn reject(reason: u8, param: u8) -> Result<Action, Reject> {
        Err(Reject { reason, param })
    }

    #[test]
    fn mode_codes_match_spec() {
        let codes: Vec<u8> = ALL.iter().map(|m| m.code()).collect();
        assert_eq!(codes, vec![0, 1, 2, 3, 4]);
    }

    #[test]
    fn only_control_modes_are_active() {
        let active: Vec<bool> = ALL.iter().map(|m| m.is_active()).collect();
        assert_eq!(active, vec![false, true, true, true, false]);
    }

    #[test]
    fn set_mode_enters_control_mode_when_config_is_ready() {
        assert_eq!(decide(Mode::Disabled, &Message::SetMode { mode: 1 }, None), Ok(Action::Enter(Mode::Current)));
        assert_eq!(decide(Mode::Disabled, &Message::SetMode { mode: 2 }, None), Ok(Action::Enter(Mode::Velocity)));
        assert_eq!(decide(Mode::Disabled, &Message::SetMode { mode: 3 }, None), Ok(Action::Enter(Mode::Position)));
    }

    #[test]
    fn set_mode_switches_between_control_modes() {
        assert_eq!(decide(Mode::Current, &Message::SetMode { mode: 3 }, None), Ok(Action::Enter(Mode::Position)));
        assert_eq!(decide(Mode::Position, &Message::SetMode { mode: 3 }, None), Ok(Action::Enter(Mode::Position)));
    }

    #[test]
    fn set_mode_zero_disables() {
        assert_eq!(decide(Mode::Velocity, &Message::SetMode { mode: 0 }, None), Ok(Action::Disable));
        // パラメータが無効でも、止めることはできる
        assert_eq!(decide(Mode::Disabled, &Message::SetMode { mode: 0 }, Some(0x04)), Ok(Action::Disable));
    }

    #[test]
    fn set_mode_needs_valid_config() {
        assert_eq!(decide(Mode::Disabled, &Message::SetMode { mode: 1 }, Some(0x04)), reject(nack::CONFIG, 0x04));
    }

    #[test]
    fn set_mode_rejects_unknown_mode() {
        // 4(異常)は要求できない
        assert_eq!(decide(Mode::Disabled, &Message::SetMode { mode: 4 }, None), reject(nack::VALUE, 0));
        assert_eq!(decide(Mode::Disabled, &Message::SetMode { mode: 0xFF }, None), reject(nack::VALUE, 0));
    }

    #[test]
    fn set_mode_is_refused_while_faulted() {
        for mode in 0..=3 {
            assert_eq!(decide(Mode::Fault, &Message::SetMode { mode }, None), reject(nack::FAULT, 0));
        }
    }

    #[test]
    fn set_param_only_while_disabled() {
        let m = Message::SetParam { id: 0x04, value: 6.0 };
        assert_eq!(decide(Mode::Disabled, &m, Some(0x00)), Ok(Action::SetParam { id: 0x04, value: 6.0 }));
        assert_eq!(decide(Mode::Current, &m, None), reject(nack::ACTIVE, 0));
        assert_eq!(decide(Mode::Velocity, &m, None), reject(nack::ACTIVE, 0));
        assert_eq!(decide(Mode::Position, &m, None), reject(nack::ACTIVE, 0));
        assert_eq!(decide(Mode::Fault, &m, None), reject(nack::FAULT, 0));
    }

    #[test]
    fn set_origin_only_while_disabled() {
        assert_eq!(decide(Mode::Disabled, &Message::SetOrigin, Some(0x00)), Ok(Action::SetOrigin));
        assert_eq!(decide(Mode::Position, &Message::SetOrigin, None), reject(nack::ACTIVE, 0));
        assert_eq!(decide(Mode::Fault, &Message::SetOrigin, None), reject(nack::FAULT, 0));
    }

    #[test]
    fn reset_fault_acts_only_in_fault() {
        assert_eq!(decide(Mode::Fault, &Message::ResetFault, Some(0x00)), Ok(Action::ResetFault));
        // 異常でなければ何もせず受理する
        assert_eq!(decide(Mode::Disabled, &Message::ResetFault, None), Ok(Action::Nothing));
        assert_eq!(decide(Mode::Velocity, &Message::ResetFault, None), Ok(Action::Nothing));
    }

    #[test]
    fn non_command_messages_are_refused() {
        for m in [
            Message::TargetCurrent { current: 1.0 },
            Message::Ack { command: 0x10 },
            Message::FaultNotice,
        ] {
            assert_eq!(decide(Mode::Disabled, &m, None), reject(nack::FRAME, 0));
        }
    }
}
