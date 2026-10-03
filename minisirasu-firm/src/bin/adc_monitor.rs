#![no_main]
#![no_std]

//! PWMのデューティを正弦波で振りながら、センターアラインPWMの中心(CNT=0, 出力Highパルスの中央)で
//! ADCをインジェクテッド変換し、その値を間引いてprintする動作確認用バイナリ。

use md_core::{pwm::duty_to_compare, scalar::Scalar};
use minisirasu_firm as _; // memory layout + panic handler

const SYSCLK_HZ: u32 = 72_000_000;
const PWM_HZ: u32 = 20_000;
/// センターアラインなのでPWM周期 = 2 * ARR / SYSCLK
const PWM_ARR: u16 = (SYSCLK_HZ / (2 * PWM_HZ)) as u16;

/// 正弦波デューティの振幅(-1..=1)と周波数
const SINE_AMPLITUDE: f32 = 0.1;
const SINE_HZ: f32 = 1.0;
/// Some(d) なら正弦波の代わりに固定デューティ d を出す(テスター測定用)。None で正弦波
const FIXED_DUTY: Option<f32> = Some(0.2);
/// printの間引き(PWM周期単位)。2000周期 = 100ms
const PRINT_DECIMATION: u32 = 2000;
/// 起動時にisenseオフセットを平均するサンプル数(PWM周期単位)
const OFFSET_SAMPLES: u32 = 1024;

const CH_TEMP: u8 = 6; // PA6
const CH_VSENSE: u8 = 8; // PB0
const CH_ISENSEB: u8 = 9; // PB1

const PI: f32 = core::f32::consts::PI;

#[cortex_m_rt::entry]
fn main() -> ! {
    use stm32f1::stm32f103 as pac;

    let p = pac::Peripherals::take().unwrap();

    clock_init(&p);
    gpio_init(&p);
    adc_init(&p);
    tim1_init(&p);

    // ドライバ(内蔵アンプ)を起こしてから、duty=0(電流0)の状態でisenseのオフセットを平均して求める
    driver_enable(&p);
    let mut sum: u32 = 0;
    for _ in 0..OFFSET_SAMPLES {
        while p.ADC1.sr.read().jeoc().bit_is_clear() {}
        p.ADC1.sr.modify(|_, w| w.jeoc().clear_bit());
        sum += p.ADC1.jdr1().read().jdata().bits() as u32;
    }
    let isense_offset = (sum / OFFSET_SAMPLES) as i32;
    defmt::info!("isense offset={}", isense_offset);

    let phase_step = 2.0 * PI * SINE_HZ / PWM_HZ as f32;
    let mut phase: f32 = 0.0;
    let mut duty: f32 = 0.0;
    let mut count: u32 = 0;
    // 診断用: High中心(CNT=0)とLow中心(CNT=ARR)それぞれのisenseの合計と最大
    let (mut sum_hi, mut sum_lo, mut max_hi, mut max_lo) = (0i32, 0i32, i32::MIN, i32::MIN);
    let (mut n_hi, mut n_lo) = (0i32, 0i32);
    // 処理が次の変換完了に間に合わなかった回数(これが多いとhi/loの判定が当てにならない)
    let mut overruns: u32 = 0;

    loop {
        // PWM中心(TIM1 TRGO)でトリガされたインジェクテッド変換の完了待ち
        // 待つ前から JEOC が立っている = 前回の処理が遅れて変換を取りこぼしかけている
        if p.ADC1.sr.read().jeoc().bit_is_set() {
            overruns += 1;
        }
        while p.ADC1.sr.read().jeoc().bit_is_clear() {}
        p.ADC1.sr.modify(|_, w| w.jeoc().clear_bit());

        let isense = p.ADC1.jdr1().read().jdata().bits() as i32 - isense_offset;
        // 変換(~12us)は周期の半分(25us)以内に終わるので、DIRでどちらの中心で取ったか分かる
        // アンダーフロー(CNT=0, High中心)の後はアップカウント(DIR=0)
        let at_high_center = p.TIM1.cr1.read().dir().bit_is_clear();
        if !at_high_center {
            sum_lo += isense;
            max_lo = max_lo.max(isense);
            n_lo += 1;
            continue;
        }
        sum_hi += isense;
        max_hi = max_hi.max(isense);
        n_hi += 1;
        let vsense = p.ADC1.jdr2().read().jdata().bits();
        let temp = p.ADC1.jdr3().read().jdata().bits();

        // NFAULT(PA3) は Low で異常
        if p.GPIOA.idr.read().idr3().bit_is_clear() {
            p.GPIOA.bsrr.write(|w| w.br4().set_bit()); // EN 無効
            p.TIM1.ccr1().write(|w| w.ccr().bits(0));
            p.TIM1.ccr2().write(|w| w.ccr().bits(0));
            defmt::error!("NFAULT asserted: output disabled");
            loop {
                cortex_m::asm::wfi();
            }
        }

        // 次周期のデューティ(CCRはプリロード有効なので次の更新イベントで反映)
        phase += phase_step;
        if phase > PI {
            phase -= 2.0 * PI;
        }
        let prev_duty = duty;
        duty = FIXED_DUTY.unwrap_or_else(|| SINE_AMPLITUDE * sin_approx(phase));
        let (a, b) = duty_to_compare(Scalar::from(duty), PWM_ARR);
        // PA9(PWMA)=CH2, PA8(PWMB)=CH1
        p.TIM1.ccr1().write(|w| w.ccr().bits(b));
        p.TIM1.ccr2().write(|w| w.ccr().bits(a));

        count += 1;
        if count >= PRINT_DECIMATION {
            defmt::info!(
                "duty={} isense hi(avg/max/n)={}/{}/{} lo(avg/max/n)={}/{}/{} overruns={} vsense={} temp={}",
                prev_duty,
                sum_hi / n_hi.max(1),
                max_hi,
                n_hi,
                sum_lo / n_lo.max(1),
                max_lo,
                n_lo,
                overruns,
                vsense,
                temp
            );
            count = 0;
            overruns = 0;
            (sum_hi, sum_lo, max_hi, max_lo) = (0, 0, i32::MIN, i32::MIN);
            (n_hi, n_lo) = (0, 0);
        }
    }
}

/// x∈[-π, π] の sin 近似(放物線近似 + 1段補正, 最大誤差 ~0.001)。libm不要。
fn sin_approx(x: f32) -> f32 {
    const B: f32 = 4.0 / PI;
    const C: f32 = -4.0 / (PI * PI);
    const P: f32 = 0.225;
    let y = B * x + C * x * abs(x);
    P * (y * abs(y) - y) + y
}

fn abs(x: f32) -> f32 {
    if x < 0.0 { -x } else { x }
}

/// HSE(12MHz) -> PLL x6 = 72MHz, APB1 = 36MHz, FLASH wait 2
fn clock_init(p: &stm32f1::stm32f103::Peripherals) {
    p.RCC.cr.modify(|_, w| w.hseon().set_bit());
    while p.RCC.cr.read().hserdy().bit_is_clear() {}

    p.FLASH.acr.modify(|_, w| unsafe { w.prftbe().set_bit().latency().bits(2) });

    // APB2(ADC元クロック)は分周無し(72MHz)なので、ADCPRE=/6で12MHzにする(ADCは最大14MHz)
    p.RCC
        .cfgr
        .modify(|_, w| w.pllsrc().hse_div_prediv().pllxtpre().div1().pllmul().mul6().ppre1().div2().adcpre().div6());
    p.RCC.cr.modify(|_, w| w.pllon().set_bit());
    while p.RCC.cr.read().pllrdy().bit_is_clear() {}

    p.RCC.cfgr.modify(|_, w| w.sw().pll());
    while !p.RCC.cfgr.read().sws().is_pll() {}
}

/// PA8/PA9: TIM1 AF push-pull, PA2(NSLEEP)/PA4(EN): 出力(Low), PA3(NFAULT): 入力プルアップ
/// PA6(TEMP), PB0(VSENSE), PB1(ISENSEB): アナログ入力
fn gpio_init(p: &stm32f1::stm32f103::Peripherals) {
    p.RCC.apb2enr.modify(|_, w| {
        w.iopaen().set_bit().iopben().set_bit().tim1en().set_bit().afioen().set_bit().adc1en().set_bit()
    });

    // 出力は先にLowにしておく、NFAULTはプルアップ
    p.GPIOA.bsrr.write(|w| w.br2().set_bit().br4().set_bit().bs3().set_bit());

    // CRL: PA2 = 0b0010(出力PP 2MHz), PA3 = 0b1000(入力プル), PA4 = 0b0010, PA6 = 0b0000(アナログ)
    p.GPIOA.crl.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !((0xF << 8) | (0xF << 12) | (0xF << 16) | (0xF << 24));
        v |= (0b0010 << 8) | (0b1000 << 12) | (0b0010 << 16);
        w.bits(v)
    });
    // CRH: PA8, PA9 = 0b1011(AF出力PP 50MHz)
    p.GPIOA.crh.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !((0xF << 0) | (0xF << 4));
        v |= (0b1011 << 0) | (0b1011 << 4);
        w.bits(v)
    });

    // GPIOB CRL: PB0, PB1 = CNF00 MODE00(アナログ入力)
    p.GPIOB.crl.modify(|r, w| unsafe { w.bits(r.bits() & !((0xF << 0) | (0xF << 4))) });
}

/// ADC1: キャリブレーション後、TIM1_TRGOトリガのインジェクテッド変換(ISENSEB, VSENSE, TEMP)を設定
fn adc_init(p: &stm32f1::stm32f103::Peripherals) {
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

    // サンプリング時間: 3ch合計がPWM周期(50us)に収まるように選ぶ(ADCCLK=12MHz)
    // ISENSEB/VSENSE: 28.5cyc(0b011) -> 41cyc=3.4us, TEMP: 55.5cyc(0b101) -> 68cyc=5.7us
    adc.smpr2.modify(|_, w| w.smp6().bits(0b101).smp8().bits(0b011).smp9().bits(0b011));

    // インジェクテッド3変換。JL=2のときJSQ2,JSQ3,JSQ4の順に変換され、JDR1..3に入る
    adc.jsqr.write(|w| unsafe {
        w.jl().bits(2).jsq2().bits(CH_ISENSEB).jsq3().bits(CH_VSENSE).jsq4().bits(CH_TEMP)
    });

    // 複数チャンネルなのでSCAN有効
    adc.cr1.modify(|_, w| w.scan().set_bit());
    // インジェクテッドの外部トリガ = TIM1_TRGO(JEXTSEL=000)
    adc.cr2.modify(|_, w| w.jextsel().bits(0b000).jexttrig().set_bit());
}

/// TIM1: センターアラインモード1, PWM mode1 (CH1, CH2), ARR=1800 (20kHz)
/// 更新イベントをTRGOでADCへ出す(RCRは下記参照)
fn tim1_init(p: &stm32f1::stm32f103::Peripherals) {
    let t = &p.TIM1;
    t.psc.write(|w| w.psc().bits(0));
    t.arr.write(|w| w.arr().bits(PWM_ARR));
    t.ccr1().write(|w| w.ccr().bits(0));
    t.ccr2().write(|w| w.ccr().bits(0));
    // 診断中: RCR=0で山(CNT=ARR)と谷(CNT=0)の両方で更新イベント -> 両方の中心でADCを取る
    // 本番では RCR=1 で1周期に1回(谷=Highパルス中央)にする
    t.rcr.write(|w| unsafe { w.rep().bits(0) });

    t.ccmr1_output().modify(|_, w| w.oc1m().pwm_mode1().oc1pe().set_bit().oc2m().pwm_mode1().oc2pe().set_bit());
    t.ccer.modify(|_, w| w.cc1e().set_bit().cc2e().set_bit());
    // TRGO = 更新イベント
    t.cr2.modify(|_, w| w.mms().update());
    t.cr1.modify(|_, w| w.cms().center_aligned1().arpe().set_bit());
    // 高度制御タイマはMOEを立てないと出力されない
    t.bdtr.modify(|_, w| w.moe().set_bit());
    // UGでRCRをロード。UGによる更新イベントでTRGOが1回出るので、その変換結果は捨てる
    t.egr.write(|w| w.ug().set_bit());
    cortex_m::asm::delay(SYSCLK_HZ / 10_000); // 変換完了待ち(~100us)
    p.ADC1.sr.modify(|_, w| w.jeoc().clear_bit());
    t.cr1.modify(|_, w| w.cen().set_bit());
}

/// NSLEEP を上げてドライバを起こし、起動待ち後に EN(!EMS, High=有効) を上げる
fn driver_enable(p: &stm32f1::stm32f103::Peripherals) {
    p.GPIOA.bsrr.write(|w| w.bs2().set_bit());
    cortex_m::asm::delay(SYSCLK_HZ / 200); // 5ms
    p.GPIOA.bsrr.write(|w| w.bs4().set_bit());
}
