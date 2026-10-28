//! レジスタ操作。2025-12-02版の基板(ドライバはDRV8701、水晶は8MHz)のピン配置:
//! - PA8: EN(TIM1 CH1、PWM)、PA9: PH(TIM1 CH2、向き)
//! - PB11: NSLEEP(!EMS)、PB0: NFAULT、PB2: SNSOUT(使っていない)
//! - PA3: ISENSEB(INA2181、ADC ch3)、PA4: ISENSEA(DRV8701のSO、使っていない)
//! - PB1: VSENSE(ADC ch9)、PA7: TEMP(ADC ch7)
//! - PA15/PB3: エンコーダ(TIM2の部分リマップ)
//! - PB8/PB9: CAN

use minishirasu_firm::config::PWM_ARR;
use minishirasu_firm::pwm::Compare;
use stm32f1::stm32f103 as pac;

const SYSCLK_HZ: u32 = 72_000_000;

/// trueなら、RCRをカウンタの起動前に書く。
/// 実機ではfalse(起動後に書く)で更新イベント(CCRの反映)が山(CNT=ARR)で出る。
/// trueだと、UGでリピティションカウンタに1がロードされ、更新イベントは谷で出る。
/// 起動時の確認で「update event is not at the peak」と出たら反対にして試す
const RCR_BEFORE_START: bool = false;

/// ADCトリガ用のチャネル4のコンペア値。OC4REFは CNT < CCR4 の間Highなので、
/// ダウンカウントで谷の手前 (CCR4 - 1) カウントの時点で立ち上がる。
/// ISENSEBのサンプル時間7.5サイクル(12MHzで0.625us = 45カウント)の半分だけ手前にして、
/// サンプル窓の中央を谷(ONパルスの中央)に合わせる
const ADC_TRIGGER_CCR: u16 = 23;

const CH_ISENSEB: u8 = 3; // PA3
const CH_TEMP: u8 = 7; // PA7
const CH_VSENSE: u8 = 9; // PB1

/// DRV8701がNSLEEPを上げてから動けるようになるまでの待ち[ms]。
/// データシートのtWAKEより長くとる。実機で足りなければ延ばす
const WAKE_MS: u32 = 2;

// SAFETY(以下のアクセサ共通): init完了後、これらのレジスタは次の決まりで触る。
// - GPIOB: BSRRへの書き込み(それ自体がアトミック)と、IDR・ODRの読み出しだけ
// - TIM1: CCR1/CCR2/SRはCurrentLoopだけが書く
// - ADC1: CurrentLoopだけが読む
// - TIM2: CNTを読むだけ
fn gpiob() -> &'static pac::gpiob::RegisterBlock {
    unsafe { &*pac::GPIOB::ptr() }
}

fn tim1() -> &'static pac::tim1::RegisterBlock {
    unsafe { &*pac::TIM1::ptr() }
}

fn tim2() -> &'static pac::tim2::RegisterBlock {
    unsafe { &*pac::TIM2::ptr() }
}

fn adc1() -> &'static pac::adc1::RegisterBlock {
    unsafe { &*pac::ADC1::ptr() }
}

fn delay_ms(ms: u32) {
    cortex_m::asm::delay(SYSCLK_HZ / 1000 * ms);
}

/// 全ペリフェラルを設定する。ドライバはスリープ(出力無効)のまま
pub fn init(p: &pac::Peripherals) {
    clock_init(p);
    gpio_init(p);
    adc_init(p);
    tim2_init(p);
    tim1_init(p);
}

/// HSE(8MHz) -> PLL x9 = 72MHz, APB1 = 36MHz, FLASH wait 2
/// (2025-12-02版の基板で水晶が12MHzから8MHzに変わった。12MHzの基板ではx6にする)
fn clock_init(p: &pac::Peripherals) {
    p.RCC.cr.modify(|_, w| w.hseon().set_bit());
    while p.RCC.cr.read().hserdy().bit_is_clear() {}

    p.FLASH.acr.modify(|_, w| unsafe { w.prftbe().set_bit().latency().bits(2) });

    // APB2(ADC元クロック)は分周無し(72MHz)なので、ADCPRE=/6で12MHzにする(ADCは最大14MHz)
    p.RCC
        .cfgr
        .modify(|_, w| w.pllsrc().hse_div_prediv().pllxtpre().div1().pllmul().mul9().ppre1().div2().adcpre().div6());
    p.RCC.cr.modify(|_, w| w.pllon().set_bit());
    while p.RCC.cr.read().pllrdy().bit_is_clear() {}

    p.RCC.cfgr.modify(|_, w| w.sw().pll());
    while !p.RCC.cfgr.read().sws().is_pll() {}
}

/// PA8/PA9: TIM1 AF push-pull。PA3(ISENSEB), PA4(ISENSEA), PA7(TEMP), PB1(VSENSE): アナログ入力
/// PB11(NSLEEP): 出力(Low), PB0(NFAULT): 入力プルアップ
/// PA15/PB3(エンコーダ): 入力フローティング(リセット値のまま)
/// PB8(CAN RX): 入力プルアップ, PB9(CAN TX): AF push-pull
fn gpio_init(p: &pac::Peripherals) {
    p.RCC.apb2enr.modify(|_, w| {
        w.iopaen().set_bit().iopben().set_bit().tim1en().set_bit().afioen().set_bit().adc1en().set_bit()
    });
    p.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit().canen().set_bit());
    // DMAは使わないが、クロックを入れておく。F1ではAHBのマスタが動いていないと、
    // idleのwfi中にデバッガからメモリを読めず、RTTのログが届かない
    p.RCC.ahbenr.modify(|_, w| w.dma1en().set_bit());

    // GPIOA CRL: PA3, PA4, PA7 = 0b0000(アナログ入力)。
    // PA4はDRV8701のSOが1k越しにつながっているので、出力にしない
    p.GPIOA.crl.modify(|r, w| unsafe { w.bits(r.bits() & !((0xF << 12) | (0xF << 16) | (0xF << 28))) });
    // GPIOA CRH: PA8, PA9 = 0b1011(AF出力PP 50MHz)
    p.GPIOA.crh.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !((0xF << 0) | (0xF << 4));
        v |= (0b1011 << 0) | (0b1011 << 4);
        w.bits(v)
    });

    // NSLEEPは先にLow(スリープ)、NFAULTとCAN RXはプルアップ
    p.GPIOB.bsrr.write(|w| w.br11().set_bit().bs0().set_bit().bs8().set_bit());
    // GPIOB CRL: PB0 = 0b1000(入力プル), PB1 = 0b0000(アナログ入力)
    p.GPIOB.crl.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !((0xF << 0) | (0xF << 4));
        v |= 0b1000 << 0;
        w.bits(v)
    });
    // GPIOB CRH: PB8 = 0b1000(入力プル), PB9 = 0b1011(AF出力PP 50MHz), PB11 = 0b0010(出力PP 2MHz)
    p.GPIOB.crh.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !((0xF << 0) | (0xF << 4) | (0xF << 12));
        v |= (0b1000 << 0) | (0b1011 << 4) | (0b0010 << 12);
        w.bits(v)
    });

    // CANをPB8/PB9へ(CAN_REMAP = 10)。TIM2のCH1/CH2をPA15/PB3へ(TIM2_REMAP = 01)。
    // PA15とPB3はJTAGのピンなので、SWJ_CFG = 010でJTAGを切ってSWDだけ残す。
    // SWJ_CFGは書き込み専用で読み値が不定なので、modifyでも毎回書く。
    // 書かないと読んだゴミが書き戻され、デバッガがつながらなくなることがある
    p.AFIO
        .mapr
        .modify(|_, w| unsafe { w.can_remap().bits(0b10).tim2_remap().bits(0b01).swj_cfg().bits(0b010) });
}

/// ADC1: キャリブレーション後、TIM1_TRGOトリガのインジェクテッド変換(ISENSEB, VSENSE, TEMP)を設定
fn adc_init(p: &pac::Peripherals) {
    let adc = &p.ADC1;

    // 電源起動(ADON)、t_STAB待ち
    adc.cr2.modify(|_, w| w.adon().set_bit());
    cortex_m::asm::delay(1000);

    // キャリブレーションレジスタのリセット
    adc.cr2.modify(|_, w| w.rstcal().set_bit());
    while adc.cr2.read().rstcal().bit_is_set() {}

    // キャリブレーション実行
    adc.cr2.modify(|_, w| w.cal().set_bit());
    while adc.cr2.read().cal().bit_is_set() {}

    // サンプリング時間(ADCCLK=12MHz)。ISENSEBはONパルスが短くても測れるよう最短にする
    // 3チャネルとも7.5cyc(0b001)。変換は合計 3 * (7.5 + 12.5) = 60cyc = 5us。
    // 電流ループは変換が終わってから山までにコンペア値を書く必要があり、計算に約14usかかる。
    // 変換が長いと間に合わない(VSENSE 28.5cyc、TEMP 55.5cycの合計10.75usでは足りなかった)。
    // VSENSEとTEMPはピンにコンデンサ(100n, 1u)があるので、短いサンプル時間でも読める
    adc.smpr2.modify(|_, w| w.smp3().bits(0b001).smp7().bits(0b001).smp9().bits(0b001));

    // インジェクテッド3変換。JL=2のときJSQ2,JSQ3,JSQ4の順に変換され、JDR1..3に入る
    adc.jsqr.write(|w| unsafe {
        w.jl().bits(2).jsq2().bits(CH_ISENSEB).jsq3().bits(CH_VSENSE).jsq4().bits(CH_TEMP)
    });

    // 複数チャンネルなのでSCAN有効。変換完了(JEOC)で割り込む
    adc.cr1.modify(|_, w| w.scan().set_bit().jeocie().set_bit());
    // インジェクテッドの外部トリガ = TIM1_TRGO(JEXTSEL=000)
    adc.cr2.modify(|_, w| w.jextsel().bits(0b000).jexttrig().set_bit());
}

/// TIM2: エンコーダモード3(TI1とTI2の両エッジで数える、4逓倍)
fn tim2_init(p: &pac::Peripherals) {
    let t = &p.TIM2;
    t.arr.write(|w| unsafe { w.bits(0xFFFF) });
    // CCMR1: CC1S=01(TI1), IC1F=0011, CC2S=01(TI2), IC2F=0011。フィルタは8サンプル
    t.ccmr1_input().write(|w| unsafe { w.bits(0x3131) });
    // SMCR: SMS=011(エンコーダモード3)
    t.smcr.write(|w| unsafe { w.bits(0x0003) });
    t.cr1.modify(|_, w| w.cen().set_bit());
}

/// TIM1: センターアラインモード1, PWM mode1 (CH1, CH2), ARR=1800 (20kHz)
/// CCRの反映(更新イベント)は山、ADCトリガ(CH4のOC4REFをTRGOに出す)は谷の直前
fn tim1_init(p: &pac::Peripherals) {
    let t = &p.TIM1;
    t.psc.write(|w| w.psc().bits(0));
    t.arr.write(|w| w.arr().bits(PWM_ARR));
    t.ccr1().write(|w| w.ccr().bits(0));
    t.ccr2().write(|w| w.ccr().bits(0));
    t.ccr4().write(|w| w.ccr().bits(ADC_TRIGGER_CCR));

    t.ccmr1_output().modify(|_, w| w.oc1m().pwm_mode1().oc1pe().set_bit().oc2m().pwm_mode1().oc2pe().set_bit());
    // CCMR2: OC4M=110(PWM mode1)。CH4は出力ピンに出さず、OC4REFだけを使う
    t.ccmr2_output().write(|w| unsafe { w.bits(0x6000) });
    t.ccer.modify(|_, w| w.cc1e().set_bit().cc2e().set_bit());
    // CR2: MMS=111(OC4REFをTRGOに出す)
    t.cr2.write(|w| unsafe { w.bits(0x0070) });
    t.cr1.modify(|_, w| w.cms().center_aligned1().arpe().set_bit());
    // 高度制御タイマはMOEを立てないと出力されない
    t.bdtr.modify(|_, w| w.moe().set_bit());

    // RM0008 Repetition counter:
    // 「RCRが奇数のとき、カウンタの起動前にRCRを書くと更新イベントはオーバーフロー(山)で、
    //   起動後に書くとアンダーフロー(谷)で出る」
    if RCR_BEFORE_START {
        t.rcr.write(|w| unsafe { w.rep().bits(1) });
        // UGでプリスケーラとRCRをロードする
        t.egr.write(|w| w.ug().set_bit());
        t.sr.modify(|_, w| w.uif().clear_bit());
        t.cr1.modify(|_, w| w.cen().set_bit());
    } else {
        t.rcr.write(|w| unsafe { w.rep().bits(0) });
        t.egr.write(|w| w.ug().set_bit());
        t.sr.modify(|_, w| w.uif().clear_bit());
        t.cr1.modify(|_, w| w.cen().set_bit());
        t.rcr.write(|w| unsafe { w.rep().bits(1) });
    }
}

/// 更新イベントが山で出ているかを確かめる。割り込みを有効にする前(init中)に呼ぶ。
/// 山(オーバーフロー)の直後はダウンカウント中になる
pub fn check_update_phase() -> bool {
    let mut ok = true;
    for _ in 0..8 {
        clear_update_flag();
        while !update_flag() {}
        ok &= counting_down();
    }
    ok
}

pub struct AdcSample {
    pub isense: u16,
    pub vsense: u16,
    pub temp: u16,
}

/// インジェクテッド変換の結果を読み、変換完了フラグを下ろす
pub fn read_adc() -> AdcSample {
    let adc = adc1();
    adc.sr.modify(|_, w| w.jeoc().clear_bit());
    AdcSample {
        isense: adc.jdr1().read().jdata().bits(),
        vsense: adc.jdr2().read().jdata().bits(),
        temp: adc.jdr3().read().jdata().bits(),
    }
}

pub fn clear_update_flag() {
    tim1().sr.modify(|_, w| w.uif().clear_bit());
}

/// 最後にclear_update_flagを呼んでから、更新イベント(山)があったか
pub fn update_flag() -> bool {
    tim1().sr.read().uif().bit_is_set()
}

/// TIM1がダウンカウント中(山から谷へ向かっている)か
pub fn counting_down() -> bool {
    tim1().cr1.read().dir().bit_is_set()
}

/// 谷からの経過[カウント]。0が谷、ARRが山、2*ARRが次の谷。1カウントは1/72us。
/// CNTと向きを別々に読むので、山や谷のごく近くでは不正確
pub fn phase() -> u16 {
    let cnt = tim1().cnt.read().bits() as u16;
    if counting_down() { (2 * PWM_ARR).saturating_sub(cnt) } else { cnt }
}

/// コンペア値を書く。プリロードなので、次の更新イベント(山)で反映される
pub fn set_compare(c: Compare) {
    // PA8(EN)=CH1, PA9(PH)=CH2
    tim1().ccr1().write(|w| w.ccr().bits(c.en));
    tim1().ccr2().write(|w| w.ccr().bits(c.ph));
}

pub fn encoder_count() -> u16 {
    tim2().cnt.read().bits() as u16
}

/// ドライバを起こし、動けるようになるまで待つ(約2msブロックする)。もう起きていれば待たない。
/// 割り込みを止めたまま待たないよう、output_enableとは分けて、ロックの外で呼ぶ
pub fn driver_wake() {
    if gpiob().odr.read().odr11().bit_is_set() {
        return;
    }
    gpiob().bsrr.write(|w| w.bs11().set_bit());
    delay_ms(WAKE_MS);
}

/// 出力を有効にする。DRV8701にはENがないので、NSLEEPをHighにする。
/// 先にdriver_wakeで起こしておくこと。ここでは待たない
pub fn output_enable() {
    gpiob().bsrr.write(|w| w.bs11().set_bit());
}

/// NSLEEPを下げてスリープにする。ゲートがすべてOFFになり、モーターはフリーになる
pub fn output_disable() {
    gpiob().bsrr.write(|w| w.br11().set_bit());
}

/// NFAULT(PB0)はLowで異常
pub fn nfault_asserted() -> bool {
    gpiob().idr.read().idr0().bit_is_clear()
}

/// ドライバの異常を解く。DRV8701の異常はスリープで解除されるので、スリープのまま1ms待つ。
/// 次に有効にするときにdriver_wakeで起こす
pub fn driver_restart() {
    output_disable();
    delay_ms(1);
}
