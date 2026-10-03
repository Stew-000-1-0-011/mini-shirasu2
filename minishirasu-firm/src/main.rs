#![cfg_attr(target_os = "none", no_std, no_main)]

// ホストでは何もしない。ワークスペース全体を `cargo build` できるようにするためだけにある
#[cfg(not(target_os = "none"))]
fn main() {}

#[cfg(target_os = "none")]
use defmt_rtt as _; // global logger
#[cfg(target_os = "none")]
use panic_probe as _;

#[cfg(target_os = "none")]
mod board;
#[cfg(target_os = "none")]
mod can;
#[cfg(target_os = "none")]
mod shared;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[cfg(target_os = "none")]
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// 起動時にisenseのオフセットを平均するサンプル数(PWM周期単位)
#[cfg(target_os = "none")]
const OFFSET_SAMPLES: u32 = 1024;

// ソフトウェアタスクのディスパッチャ。宣言順に優先度の低い方から割り当てられる(SPI1=1, SPI2=3)
#[cfg(target_os = "none")]
#[rticx_cortex_m::app(device = stm32f1::stm32f103, dispatchers = [SPI1, SPI2])]
mod app {
    use super::shared::{CommandInput, Link, OuterCmd, OuterInput, ReportKind, Telemetry};
    use super::{OFFSET_SAMPLES, board, can};
    use md_core::controller::{CurrentState, Measurement, Saturated};
    use md_core::encoder::EncoderState;
    use md_core::fixed::{Q3_28, Q16_16};
    use minishirasu_firm::bench::{self, Script};
    use minishirasu_firm::cascade::{self, Cascade, Setpoint};
    use minishirasu_firm::config::{self, BuildError, InnerParams, OuterParams, Settings};
    use minishirasu_firm::protocol::{MAX_ENCODED, Message, Status, StreamParser};
    use minishirasu_firm::state::{self, Action, Mode, Reject, nack};
    use minishirasu_firm::txbuf::{TxQueues, TxStream};
    use minishirasu_firm::{pwm, sense};
    use stm32f1::stm32f103 as pac;

    #[shared]
    struct Shared {
        link: Link,
        outer_cmd: OuterCmd,
        mode: Mode,
        /// 電流ループ用のパラメータ。設定が揃って検証に通るまでNone
        inner: Option<InnerParams>,
        /// 外側ループと通信用のパラメータ。innerと同時にSomeになる
        outer: Option<OuterParams>,
        telemetry: Telemetry,
        tx: TxQueues,
    }

    #[init]
    fn init() -> (Shared, TaskInits) {
        let p = pac::Peripherals::take().unwrap();
        board::init(&p);
        let phase_ok = board::check_update_phase();
        if !phase_ok {
            defmt::error!("update event is not at the peak: flip RCR_BEFORE_START in board.rs");
        }
        // バスと同期するまで戻らないので、ここで止まったことがログでわかるようにする
        defmt::info!("waiting for CAN bus");
        let (tx, rx) = can::init();
        defmt::info!("minishirasu-firm started");

        (
            Shared {
                link: Link::new(),
                outer_cmd: OuterCmd {
                    setpoint: Setpoint::hold(Mode::Disabled, Q16_16::ZERO),
                    set_origin: false,
                    status_period_ms: 0,
                },
                mode: Mode::Disabled,
                inner: None,
                outer: None,
                telemetry: Telemetry {
                    i: Q3_28::ZERO,
                    w: Q3_28::ZERO,
                    th: Q16_16::ZERO,
                    vdc: Q3_28::ZERO,
                    temp: 0,
                    flags: 0,
                },
                tx: TxQueues::new(),
            },
            TaskInits {
                idle: Idle,
                current_loop: CurrentLoop {
                    state: CurrentState::new(),
                    epoch: 0,
                    offset_sum: 0,
                    offset_count: 0,
                    offset: 0,
                    timing_ok: phase_ok,
                    applied_sign: 0,
                    queued_sign: 0,
                    tick: 0,
                    late: 0,
                },
                outer_loop: OuterLoop {
                    encoder: EncoderState::new(),
                    cascade: Cascade::new(),
                    meas: Measurement {
                        vdc: Q3_28::ONE,
                        vdc_inv: Q3_28::ONE,
                        i: Q3_28::ZERO,
                        w: Q3_28::ZERO,
                        th: Q16_16::ZERO,
                    },
                    last_count: None,
                    epoch: 0,
                    elapsed_ms: 0,
                    bench_ms: 0,
                },
                bench: Bench { script: Script::new(), logged_ms: 0 },
                can_rx: CanRx { rx, target: StreamParser::new(), command: StreamParser::new() },
                can_tx: CanTx { tx },
                command: Command { settings: Settings::new(), config_error: Some(config::id::VDCMAX) },
                report: Report { late: 0 },
            },
        )
    }

    #[idle]
    struct Idle;

    impl RticIdleTask for Idle {
        fn exec(&mut self) -> ! {
            loop {
                cortex_m::asm::wfi();
            }
        }
    }

    /// ADC変換完了割り込み(20kHz、ONパルスの中央でトリガ)。電流制御
    #[task(binds = ADC1_2, priority = 4, shared = [link, mode, inner])]
    struct CurrentLoop {
        state: CurrentState,
        epoch: u32,
        offset_sum: u32,
        offset_count: u32,
        /// isenseのオフセット(ADC生値)
        offset: u16,
        /// 更新イベントが山、ADCトリガが谷で出ているか
        timing_ok: bool,
        /// 今サンプルした周期に実際に出ていたデューティの向き
        applied_sign: i32,
        /// 最後に書いたコンペア値の向き
        queued_sign: i32,
        tick: u32,
        late: u32,
    }

    impl RticTask for CurrentLoop {
        fn exec(&mut self) {
            // 山の更新フラグを下ろしておき、コンペア値を書いたあとで山を越えていないかを見る
            board::clear_update_flag();
            let adc = board::read_adc();
            // 谷でトリガされていれば、変換(約11us)が終わった今はアップカウント中。
            // ダウンカウント中なら、トリガの設定が違うか、割り込みが14us以上遅れて山を越えたか
            let after_peak = board::counting_down();

            // 起動直後はオフセットを測る。出力は無効(EN=Low)なのでシャントの電流は0
            if self.offset_count < OFFSET_SAMPLES {
                // この間は割り込みを長く禁止するものがないので、設定の間違いとみなす
                if after_peak {
                    self.timing_ok = false;
                }
                self.offset_sum += adc.isense as u32;
                self.offset_count += 1;
                if self.offset_count == OFFSET_SAMPLES {
                    self.offset = (self.offset_sum / OFFSET_SAMPLES) as u16;
                    defmt::info!("isense offset={}", self.offset);
                    if !self.timing_ok {
                        defmt::error!("ADC trigger or update event timing is wrong: output stays disabled");
                    }
                }
                return;
            }

            let mut s = self.shared();
            let (i_target, mut enabled, epoch, w, vdc, vdc_inv) =
                s.link.lock(|l| (l.i_target, l.enabled, l.epoch, l.w, l.vdc, l.vdc_inv));

            let nfault = board::nfault_asserted();
            if nfault || !self.timing_ok {
                let entered = s.mode.lock(|m| {
                    let entered = *m != Mode::Fault;
                    *m = Mode::Fault;
                    entered
                });
                if entered {
                    board::output_disable();
                    s.link.lock(|l| l.enabled = false);
                    let _ = Report::spawn(ReportKind::Fault);
                    defmt::error!("fault: output disabled (nfault={} timing_ok={})", nfault, self.timing_ok);
                }
                enabled = false;
            }

            if epoch != self.epoch {
                self.state.reset();
                self.epoch = epoch;
            }

            let (duty, saturated, i) = if enabled {
                let (state, sign, offset) = (&mut self.state, self.applied_sign, self.offset);
                s.inner.lock(|p| match p {
                    Some(p) => {
                        let i = p.sense.current(adc.isense, offset, sign);
                        let m = Measurement { vdc, vdc_inv, i, w, th: Q16_16::ZERO };
                        let (duty, saturated) = state.update(&p.current, i_target, &m);
                        (duty, saturated, i)
                    }
                    None => (Q3_28::ZERO, Saturated::NotSaturated, Q3_28::ZERO),
                })
            } else {
                (Q3_28::ZERO, Saturated::NotSaturated, Q3_28::ZERO)
            };

            let compare = pwm::duty_to_compare(duty, config::PWM_ARR);
            board::set_compare(compare.0, compare.1);
            let sign = pwm::compare_sign(compare);
            if after_peak || board::update_flag() {
                // 書く前に山を越えてしまった。次の周期に出るのは1つ前に書いた値。
                // 低い優先度のタスクが割り込みを長く禁止する(ログの出力など)と起きる
                self.applied_sign = self.queued_sign;
                self.late = self.late.wrapping_add(1);
            } else {
                self.applied_sign = sign;
            }
            self.queued_sign = sign;

            let late = self.late;
            s.link.lock(|l| {
                l.i = i;
                l.saturated = saturated;
                l.late = late;
            });

            self.tick += 1;
            if self.tick >= config::OUTER_DIV {
                self.tick = 0;
                // 外側ループが前回ぶんを処理しきれていなければ、今回ぶんは捨てる
                let _ = OuterLoop::spawn(OuterInput {
                    encoder: board::encoder_count(),
                    vsense: adc.vsense,
                    temp: adc.temp,
                });
            }
        }
    }

    /// 速度・位置制御(1kHz)。CurrentLoopが20周期に1回起動する
    #[sw_task(priority = 3, shared = [link, outer_cmd, outer, telemetry])]
    struct OuterLoop {
        encoder: EncoderState,
        cascade: Cascade,
        meas: Measurement,
        /// 前回のエンコーダのカウント。最初の1回はNone
        last_count: Option<u16>,
        epoch: u32,
        elapsed_ms: u32,
        /// 起動からの時間[ms]。feature = "bench" のときだけ進める
        bench_ms: u32,
    }

    impl RticSwTask for OuterLoop {
        type SpawnInput = OuterInput;

        fn exec(&mut self, input: OuterInput) {
            let mut s = self.shared();

            if cfg!(feature = "bench") {
                self.bench_ms = self.bench_ms.wrapping_add(1);
                if self.bench_ms % bench::TICK_MS == 0 {
                    let _ = Bench::spawn(self.bench_ms);
                }
            }

            let delta = match self.last_count {
                Some(last) => sense::count_delta(last, input.encoder),
                None => 0,
            };
            self.last_count = Some(input.encoder);

            let (setpoint, set_origin, period_ms) = s.outer_cmd.lock(|c| {
                let set_origin = c.set_origin;
                c.set_origin = false;
                (c.setpoint, set_origin, c.status_period_ms)
            });
            if set_origin {
                self.encoder.set_origin();
            }

            let (i, saturated, enabled, epoch) = s.link.lock(|l| (l.i, l.saturated, l.enabled, l.epoch));
            if epoch != self.epoch {
                self.cascade.reset();
                self.epoch = epoch;
            }

            let (encoder, cascade, meas) = (&mut self.encoder, &mut self.cascade, &mut self.meas);
            // 基準値がないとper-unitにできないので、パラメータが揃うまでは何も更新しない
            let updated = s.outer.lock(|p| {
                let p = p.as_ref()?;
                let (th, w) = encoder.update(&p.encoder, delta);
                meas.i = i;
                meas.w = w;
                meas.th = th;
                // 母線電圧が低すぎて逆数が収まらないときは、前回の値を使い続ける
                let vdc_ok = meas.update_vdc(p.sense.bus_voltage(input.vsense));
                let i_target = if enabled { cascade.update(p, &setpoint, meas, saturated) } else { Q3_28::ZERO };

                let mut flags = match saturated {
                    Saturated::Overflow => 0b001,
                    Saturated::Underflow => 0b010,
                    Saturated::NotSaturated => 0,
                };
                if !vdc_ok {
                    flags |= 0b100;
                }
                Some((i_target, flags))
            });
            let Some((i_target, flags)) = updated else {
                return;
            };

            let (w, th, vdc, vdc_inv) = (self.meas.w, self.meas.th, self.meas.vdc, self.meas.vdc_inv);
            s.link.lock(|l| {
                l.i_target = i_target;
                l.w = w;
                l.vdc = vdc;
                l.vdc_inv = vdc_inv;
            });
            s.telemetry.lock(|t| *t = Telemetry { i, w, th, vdc, temp: input.temp, flags });

            if period_ms > 0 {
                self.elapsed_ms += 1;
                if self.elapsed_ms >= period_ms {
                    self.elapsed_ms = 0;
                    let _ = Report::spawn(ReportKind::Status);
                }
            }
        }
    }

    /// CAN受信。フレームをストリームに振り分けてパースする
    #[task(binds = USB_LP_CAN_RX0, priority = 2, shared = [outer_cmd, mode, outer])]
    struct CanRx {
        rx: bxcan::Rx0<can::Can1>,
        target: StreamParser,
        command: StreamParser,
    }

    impl RticTask for CanRx {
        fn exec(&mut self) {
            let mut s = self.shared();
            while let Ok(frame) = self.rx.receive() {
                let (Some(stream), Some(data)) = (can::classify(&frame), frame.data()) else {
                    continue;
                };
                for &byte in data.iter() {
                    match stream {
                        can::RxStream::Target => {
                            // 目標値ストリームの壊れたメッセージは黙って捨てる
                            let Some(Ok(message)) = self.target.push(byte) else {
                                continue;
                            };
                            let mode = s.mode.lock(|m| *m);
                            let scale = s.outer.lock(|p| p.as_ref().map(|p| p.scale));
                            let setpoint =
                                scale.and_then(|scale| cascade::setpoint_from_message(&message, mode, &scale));
                            if let Some(setpoint) = setpoint {
                                s.outer_cmd.lock(|c| c.setpoint = setpoint);
                            }
                        }
                        can::RxStream::Command => match self.command.push(byte) {
                            // キューがいっぱいなら捨てる。上位は応答が来ないことで気づける
                            Some(Ok(message)) => {
                                let _ = Command::spawn(CommandInput::Message(message));
                            }
                            Some(Err(_)) => {
                                let _ = Command::spawn(CommandInput::BadFrame);
                            }
                            None => {}
                        },
                    }
                }
            }
        }
    }

    /// CAN送信。メールボックスが空くたびに、送信バッファから次の1フレームを送る
    #[task(binds = USB_HP_CAN_TX, priority = 2, shared = [tx])]
    struct CanTx {
        tx: bxcan::Tx<can::Can1>,
    }

    impl RticTask for CanTx {
        fn exec(&mut self) {
            let mut s = self.shared();
            let tx = &mut self.tx;
            s.tx.lock(|q| can::pump(tx, q));
        }
    }

    /// コマンドの処理。f32の計算(パラメータの検証)はここでやる
    #[sw_task(priority = 1, capacity = 4, shared = [link, outer_cmd, mode, inner, outer, telemetry, tx])]
    struct Command {
        settings: Settings,
        /// 制御用のパラメータが無効なら、その原因の設定ID
        config_error: Option<u8>,
    }

    impl RticSwTask for Command {
        type SpawnInput = CommandInput;

        fn exec(&mut self, input: CommandInput) {
            let mut s = self.shared();

            let (command, result) = match input {
                CommandInput::BadFrame => (0, Err(Reject { reason: nack::FRAME, param: 0 })),
                CommandInput::Message(message) => {
                    let mode = s.mode.lock(|m| *m);
                    let result = match state::decide(mode, &message, self.config_error) {
                        Err(reject) => Err(reject),
                        Ok(Action::Nothing) => Ok(()),
                        Ok(Action::Disable) => {
                            board::output_disable();
                            s.link.lock(|l| l.enabled = false);
                            // 途中で異常に入っていたら、Faultのままにする
                            s.mode.lock(|m| {
                                if *m != Mode::Fault {
                                    *m = Mode::Disabled;
                                }
                            });
                            Ok(())
                        }
                        Ok(Action::Enter(new_mode)) => {
                            let th = s.telemetry.lock(|t| t.th);
                            s.outer_cmd.lock(|c| c.setpoint = Setpoint::hold(new_mode, th));
                            // 異常の検出(CurrentLoop)と競合しないよう、modeをロックしたまま出力を有効にする
                            let link = &mut s.link;
                            let entered = s.mode.lock(|m| {
                                if *m == Mode::Fault {
                                    return false;
                                }
                                *m = new_mode;
                                link.lock(|l| {
                                    l.epoch = l.epoch.wrapping_add(1);
                                    l.i_target = Q3_28::ZERO;
                                    l.enabled = true;
                                });
                                board::output_enable();
                                true
                            });
                            if entered { Ok(()) } else { Err(Reject { reason: nack::FAULT, param: 0 }) }
                        }
                        Ok(Action::SetParam { id, value }) => match self.settings.set(id, value) {
                            Err(_) => Err(Reject { reason: nack::VALUE, param: id }),
                            Ok(()) => {
                                let period_ms = self.settings.status_period_ms();
                                s.outer_cmd.lock(|c| c.status_period_ms = period_ms);
                                // 出力無効中なので、パラメータを差し替えても制御には影響しない
                                match self.settings.build() {
                                    Ok(params) => {
                                        s.inner.lock(|p| *p = Some(params.inner));
                                        s.outer.lock(|p| *p = Some(params.outer));
                                        self.config_error = None;
                                        Ok(())
                                    }
                                    Err(e) => {
                                        s.inner.lock(|p| *p = None);
                                        s.outer.lock(|p| *p = None);
                                        self.config_error = Some(e.id());
                                        match e {
                                            // まだ設定の途中。保存だけして受理する
                                            BuildError::Unset(_) => Ok(()),
                                            BuildError::Invalid(id) => {
                                                Err(Reject { reason: nack::CONFIG, param: id })
                                            }
                                        }
                                    }
                                }
                            }
                        },
                        Ok(Action::ResetFault) => {
                            board::driver_restart();
                            if board::nfault_asserted() {
                                Err(Reject { reason: nack::NFAULT, param: 0 })
                            } else {
                                s.mode.lock(|m| *m = Mode::Disabled);
                                Ok(())
                            }
                        }
                        Ok(Action::SetOrigin) => {
                            s.outer_cmd.lock(|c| c.set_origin = true);
                            Ok(())
                        }
                    };
                    (message.kind(), result)
                }
            };

            let response = match result {
                Ok(()) => Message::Ack { command },
                Err(reject) => {
                    defmt::warn!(
                        "nack: command={=u8:#x} reason={} param={=u8:#x}",
                        command,
                        reject.reason,
                        reject.param
                    );
                    Message::Nack { command, reason: reject.reason, param: reject.param }
                }
            };
            let mut buf = [0u8; MAX_ENCODED];
            let n = response.encode(&mut buf);
            // 入りきらなければ捨てる
            s.tx.lock(|q| q.push(TxStream::Response, &buf[..n]));
            can::kick();
        }
    }

    /// CANなしの実機試験(feature = "bench")。bench::PLANの手順を、CANで受けたときと同じ入口に流し込み、
    /// 状態をログに出す。featureがなければ起動されない
    #[sw_task(priority = 1, shared = [link, outer_cmd, mode, outer, telemetry])]
    struct Bench {
        script: Script,
        /// 最後に状態をログに出した時刻[ms]
        logged_ms: u32,
    }

    impl RticSwTask for Bench {
        type SpawnInput = u32;

        fn exec(&mut self, now: u32) {
            let mut s = self.shared();

            match self.script.next(&bench::PLAN, now) {
                bench::Action::Command(message) => {
                    if let Message::SetMode { mode } = message {
                        defmt::info!("bench: SetMode({})", mode);
                    }
                    let _ = Command::spawn(CommandInput::Message(message));
                }
                bench::Action::Target(message) => {
                    let mode = s.mode.lock(|m| *m);
                    let scale = s.outer.lock(|p| p.as_ref().map(|p| p.scale));
                    match scale.and_then(|scale| cascade::setpoint_from_message(&message, mode, &scale)) {
                        Some(setpoint) => {
                            s.outer_cmd.lock(|c| c.setpoint = setpoint);
                            defmt::info!("bench: target set");
                        }
                        None => defmt::warn!("bench: target rejected (mode={})", mode.code()),
                    }
                }
                bench::Action::Nothing => {}
            }

            if now.wrapping_sub(self.logged_ms) < bench::LOG_PERIOD_MS {
                return;
            }
            self.logged_ms = now;
            // 設定が揃うまでは物理単位に直せない
            let Some(scale) = s.outer.lock(|p| p.as_ref().map(|p| p.scale)) else {
                return;
            };
            let t = s.telemetry.lock(|t| *t);
            let mode = s.mode.lock(|m| *m);
            let late = s.link.lock(|l| l.late);
            defmt::info!(
                "bench: mode={} i={}A w={}rad/s th={}rev vdc={}V temp={} flags={} late={}",
                mode.code(),
                scale.current_a(t.i),
                scale.velocity_rad_s(t.w),
                t.th.to_f32(),
                scale.voltage_v(t.vdc),
                t.temp,
                t.flags,
                late
            );
        }
    }

    /// 状態フィードバックと異常通知の送信
    #[sw_task(priority = 1, capacity = 2, shared = [link, mode, outer, telemetry, tx])]
    struct Report {
        /// 前回見た、CCRの書き込みが山に間に合わなかった回数
        late: u32,
    }

    impl RticSwTask for Report {
        type SpawnInput = ReportKind;

        fn exec(&mut self, kind: ReportKind) {
            let mut s = self.shared();

            let (stream, message) = match kind {
                ReportKind::Fault => (TxStream::Response, Message::FaultNotice),
                ReportKind::Status => {
                    let late = s.link.lock(|l| l.late);
                    if late != self.late {
                        defmt::warn!("current loop missed the peak {} times", late.wrapping_sub(self.late));
                        self.late = late;
                    }

                    let Some(scale) = s.outer.lock(|p| p.as_ref().map(|p| p.scale)) else {
                        return;
                    };
                    let t = s.telemetry.lock(|t| *t);
                    let mode = s.mode.lock(|m| *m);
                    let status = Status {
                        mode: mode.code(),
                        flags: t.flags,
                        current: scale.current_a(t.i),
                        velocity: scale.velocity_rad_s(t.w),
                        position: config::position_to_wire(t.th),
                        vdc: scale.voltage_v(t.vdc),
                        temp: t.temp,
                    };
                    (TxStream::Status, Message::Status(status))
                }
            };

            let mut buf = [0u8; MAX_ENCODED];
            let n = message.encode(&mut buf);
            // 入りきらなければ捨てる
            s.tx.lock(|q| q.push(stream, &buf[..n]));
            can::kick();
        }
    }
}
