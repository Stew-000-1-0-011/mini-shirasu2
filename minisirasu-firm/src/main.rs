#![no_main]
#![no_std]

use minisirasu_firm as _; // memory layout + panic handler

/// オープンループ動作テスト用の固定デューティ(符号付き, -1..=1)。書き換えて使う。
const OPEN_LOOP_DUTY: f32 = 0.05;

const SYSCLK_HZ: u32 = 72_000_000;
const PWM_HZ: u32 = 20_000;
/// センターアラインなのでPWM周期 = 2 * ARR / SYSCLK
const PWM_ARR: u16 = (SYSCLK_HZ / (2 * PWM_HZ)) as u16;

#[rticx_cortex_m::app(device = stm32f1::stm32f103)]
mod app {
    use super::*;
    use md_core::{pwm::duty_to_compare, scalar::Scalar};
    use stm32f1::stm32f103 as pac;

    #[shared]
    struct Shared {}

    #[init]
    fn init() -> (Shared, TaskInits) {
        let p = pac::Peripherals::take().unwrap();

        clock_init(&p);
        gpio_init(&p);
        tim1_init(&p);
        driver_enable(&p);

        let (a, b) = duty_to_compare(Scalar::from(OPEN_LOOP_DUTY), PWM_ARR);
        // PA9(PWMA)=CH2, PA8(PWMB)=CH1
        p.TIM1.ccr1().write(|w| w.ccr().bits(b));
        p.TIM1.ccr2().write(|w| w.ccr().bits(a));
        defmt::info!("open loop: duty={} ccrA={} ccrB={} arr={}", OPEN_LOOP_DUTY, a, b, PWM_ARR);

        (Shared {}, TaskInits { idle: Idle, control_loop: ControlLoop, can_rx: CanRx })
    }

    /// HSE(12MHz) -> PLL x6 = 72MHz, APB1 = 36MHz, FLASH wait 2
    fn clock_init(p: &pac::Peripherals) {
        p.RCC.cr.modify(|_, w| w.hseon().set_bit());
        while p.RCC.cr.read().hserdy().bit_is_clear() {}

        p.FLASH.acr.modify(|_, w| unsafe { w.prftbe().set_bit().latency().bits(2) });

        // ADCは最大14MHz。APB2は72MHzなので /6 で12MHzにする
        p.RCC.cfgr.modify(|_, w| {
            w.pllsrc().hse_div_prediv().pllxtpre().div1().pllmul().mul6().ppre1().div2().adcpre().div6()
        });
        p.RCC.cr.modify(|_, w| w.pllon().set_bit());
        while p.RCC.cr.read().pllrdy().bit_is_clear() {}

        p.RCC.cfgr.modify(|_, w| w.sw().pll());
        while !p.RCC.cfgr.read().sws().is_pll() {}
    }

    /// PA8/PA9: TIM1 AF push-pull, PA2(NSLEEP)/PA4(EN): 出力(Low), PA3(NFAULT): 入力プルアップ
    fn gpio_init(p: &pac::Peripherals) {
        p.RCC.apb2enr.modify(|_, w| {
            w.iopaen().set_bit().iopben().set_bit().tim1en().set_bit().afioen().set_bit().adc1en().set_bit()
        });

        // 出力は先にLowにしておく(リセット値は0だが明示する)
        p.GPIOA.bsrr.write(|w| w.br2().set_bit().br4().set_bit());
        // NFAULT プルアップ
        p.GPIOA.bsrr.write(|w| w.bs3().set_bit());

        // CRL: PA2 = 0b0010(出力PP 2MHz), PA3 = 0b1000(入力プル), PA4 = 0b0010
        p.GPIOA.crl.modify(|r, w| unsafe {
            let mut v = r.bits();
            v &= !((0xF << 8) | (0xF << 12) | (0xF << 16));
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

        // PA6(TEMP) をアナログ入力(CNF=00, MODE=00)に
        p.GPIOA.crl.modify(|r, w| unsafe { w.bits(r.bits() & !(0xF << 24)) });
        // PB0(VSENSE), PB1(ISENSEB) をアナログ入力に
        p.GPIOB.crl.modify(|r, w| unsafe { w.bits(r.bits() & !((0xF << 0) | (0xF << 4))) });
    }

    /// TIM1: センターアラインモード1, PWM mode1 (CH1, CH2), ARR=1800 (20kHz), コンペア0で開始
    fn tim1_init(p: &pac::Peripherals) {
        let t = &p.TIM1;
        t.psc.write(|w| w.psc().bits(0));
        t.arr.write(|w| w.arr().bits(PWM_ARR));
        t.ccr1().write(|w| w.ccr().bits(0));
        t.ccr2().write(|w| w.ccr().bits(0));

        t.ccmr1_output().modify(|_, w| w.oc1m().pwm_mode1().oc1pe().set_bit().oc2m().pwm_mode1().oc2pe().set_bit());
        t.ccer.modify(|_, w| w.cc1e().set_bit().cc2e().set_bit());
        t.cr1.modify(|_, w| w.cms().center_aligned1().arpe().set_bit());
        // 高度制御タイマはMOEを立てないと出力されない
        t.bdtr.modify(|_, w| w.moe().set_bit());
        t.egr.write(|w| w.ug().set_bit());
        t.cr1.modify(|_, w| w.cen().set_bit());
    }

    /// NSLEEP を上げてドライバを起こし、起動待ち後に EN(!EMS, High=有効) を上げる
    fn driver_enable(p: &pac::Peripherals) {
        p.GPIOA.bsrr.write(|w| w.bs2().set_bit());
        cortex_m::asm::delay(SYSCLK_HZ / 200); // 5ms
        p.GPIOA.bsrr.write(|w| w.bs4().set_bit());
    }

    #[idle]
    struct Idle;

    impl RticIdleTask for Idle {
        fn exec(&mut self) -> ! {
            loop {
                // SAFETY: init 完了後、GPIOA(読み出し)とTIM1/GPIOA(異常時の停止)をidleだけが触る
                let p = unsafe { pac::Peripherals::steal() };
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
                cortex_m::asm::wfi();
            }
        }
    }

    /// ADC変換完了割り込み(PWM中央でトリガ) -> 電流/速度/位置制御ループ本体 (未実装)
    #[task(binds = ADC1_2, priority = 3, shared = [])]
    struct ControlLoop;

    impl RticTask for ControlLoop {
        fn exec(&mut self) {
            todo!()
        }
    }

    /// CAN受信割り込み -> 目標値・制御モード更新 (未実装)
    #[task(binds = USB_LP_CAN_RX0, priority = 1, shared = [])]
    struct CanRx;

    impl RticTask for CanRx {
        fn exec(&mut self) {
            todo!()
        }
    }
}
