//! CAN。ストリームごとにIDを1つ割り当て、フレームのデータをストリームのバイト列として運ぶ

use bxcan::filter::{BankConfig, ListEntry16};
use bxcan::{Data, Fifo, Frame, Id, Interrupts, Rx0, StandardId, Tx};
use minishirasu_firm::config::{CAN_ID_COMMAND, CAN_ID_RESPONSE, CAN_ID_STATUS, CAN_ID_TARGET};
use minishirasu_firm::txbuf::{TxQueues, TxStream};
use stm32f1::stm32f103 as pac;

/// 1Mbps。APB1=36MHz、プリスケーラ2で1ビット18tq(SJW=1, BS1=15, BS2=2)。サンプル点88.9%
const BTR_1MBPS: u32 = 0x001e_0001;

/// CAN1の所有権の印
pub struct Can1;

// SAFETY: Can1はinitで1つだけ作り、bxcanに渡す。ほかの場所でCAN1のレジスタに触らない
unsafe impl bxcan::Instance for Can1 {
    const REGISTERS: *mut bxcan::RegisterBlock = 0x4000_6400 as *mut _;
}

// SAFETY: F103のCAN1はフィルタバンクを14個持ち、ほかのインスタンスと共有しない
unsafe impl bxcan::FilterOwner for Can1 {
    const NUM_FILTER_BANKS: u8 = 14;
}

fn standard(id: u16) -> StandardId {
    // 設定の定数が11bitに収まっていなければ、起動時にここで止まる
    StandardId::new(id).unwrap()
}

/// クロックとピンは board::init で設定済みであること。バスと同期するまでブロックする
pub fn init() -> (Tx<Can1>, Rx0<Can1>) {
    let mut can = bxcan::Can::builder(Can1).set_bit_timing(BTR_1MBPS).leave_disabled();

    // 受信する2つのIDだけを通す
    can.modify_filters().enable_bank(
        0,
        Fifo::Fifo0,
        BankConfig::List16([
            ListEntry16::data_frames_with_id(standard(CAN_ID_TARGET)),
            ListEntry16::data_frames_with_id(standard(CAN_ID_COMMAND)),
            ListEntry16::data_frames_with_id(standard(CAN_ID_TARGET)),
            ListEntry16::data_frames_with_id(standard(CAN_ID_COMMAND)),
        ]),
    );
    can.enable_interrupts(Interrupts::TRANSMIT_MAILBOX_EMPTY | Interrupts::FIFO0_MESSAGE_PENDING);
    can.modify_config().enable();

    let (tx, rx0, _rx1) = can.split();
    (tx, rx0)
}

#[derive(Clone, Copy, PartialEq)]
pub enum RxStream {
    Target,
    Command,
}

/// 受信フレームがどのストリームのものか
pub fn classify(frame: &Frame) -> Option<RxStream> {
    let Id::Standard(id) = frame.id() else {
        return None;
    };
    match id.as_raw() {
        CAN_ID_TARGET => Some(RxStream::Target),
        CAN_ID_COMMAND => Some(RxStream::Command),
        _ => None,
    }
}

/// 送信バッファから次の1フレームを送る。CAN TX割り込みから呼ぶ。
/// ストリーム内のバイト順を保つため、前のフレームを送り終えてから次を入れる
pub fn pump(tx: &mut Tx<Can1>, queues: &mut TxQueues) {
    tx.clear_interrupt_flags();
    if !tx.is_idle() {
        return;
    }
    let mut chunk = [0u8; 8];
    // IDが小さいストリームを先に送る
    if let Some((stream, n)) = queues.pop_chunk(CAN_ID_STATUS < CAN_ID_RESPONSE, &mut chunk) {
        let id = standard(match stream {
            TxStream::Status => CAN_ID_STATUS,
            TxStream::Response => CAN_ID_RESPONSE,
        });
        if let Some(data) = Data::new(&chunk[..n]) {
            // アイドルを確かめてあるので、必ず空きメールボックスに入る
            let _ = tx.transmit(&Frame::new_data(id, data));
        }
    }
}

/// 送信バッファに積んだあとに呼ぶ。CAN TX割り込みを起こしてpumpを走らせる
pub fn kick() {
    cortex_m::peripheral::NVIC::pend(pac::Interrupt::USB_HP_CAN_TX);
}
