//! タスクの間で渡す型

use md_core::controller::Saturated;
use md_core::fixed::{Q3_28, Q16_16};
use minishirasu_firm::cascade::Setpoint;
use minishirasu_firm::protocol::Message;

/// 電流ループと外側ループの受け渡し。すべてper-unit
pub struct Link {
    // 外側ループ・Command -> 電流ループ
    pub i_target: Q3_28,
    /// 電流制御をして出力を出すか
    pub enabled: bool,
    /// 制御モードに入るたびに増える。変わったら各ループが積分器をリセットする
    pub epoch: u32,
    pub w: Q3_28,
    pub vdc: Q3_28,
    pub vdc_inv: Q3_28,
    // 電流ループ -> 外側ループ
    pub i: Q3_28,
    pub saturated: Saturated,
    /// CCRの書き込みが山に間に合わなかった回数
    pub late: u32,
}

impl Link {
    pub fn new() -> Link {
        Link {
            i_target: Q3_28::ZERO,
            enabled: false,
            epoch: 0,
            w: Q3_28::ZERO,
            // 母線電圧を測るまでの仮の値
            vdc: Q3_28::ONE,
            vdc_inv: Q3_28::ONE,
            i: Q3_28::ZERO,
            saturated: Saturated::NotSaturated,
            late: 0,
        }
    }
}

/// 通信側 -> 外側ループ
pub struct OuterCmd {
    pub setpoint: Setpoint,
    /// 次の周期で現在位置を0にする
    pub set_origin: bool,
    /// 状態を送る周期[ms]。0なら送らない
    pub status_period_ms: u32,
}

/// 状態フィードバック用のスナップショット。すべてper-unit
#[derive(Clone, Copy)]
pub struct Telemetry {
    pub i: Q3_28,
    pub w: Q3_28,
    pub th: Q16_16,
    pub vdc: Q3_28,
    pub temp: u16,
    /// bit0: 上側に飽和、bit1: 下側に飽和、bit2: 母線電圧が低く更新できていない
    pub flags: u8,
}

/// 電流ループ -> 外側ループ。20周期に1回のサンプル
/// (spawnの入力はRTICXのキューの都合でCloneが要る)
#[derive(Clone)]
pub struct OuterInput {
    pub encoder: u16,
    pub vsense: u16,
    pub temp: u16,
}

#[derive(Clone)]
pub enum CommandInput {
    Message(Message),
    /// コマンドストリームで復号できないものを受けた
    BadFrame,
}

#[derive(Clone)]
pub enum ReportKind {
    Status,
    Fault,
}
