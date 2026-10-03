//! 電流ループが使う計算の実行時間を、実機のサイクルカウンタで測る(feature = "bench")。
//! 割り込みを許可する前に呼ぶこと。結果はCPUサイクル数で、72サイクルが1us

use core::hint::black_box;

use cortex_m::peripheral::DWT;
use md_core::controller::{CurrentState, Measurement};
use md_core::fixed::{Q3_28, Q16_16};
use minishirasu_firm::bench;
use minishirasu_firm::config::{PWM_ARR, Settings};
use minishirasu_firm::pwm;

/// 1回あたりのサイクル数。ループの分を含むので、emptyの値を引いて読む
fn cycles(mut f: impl FnMut()) -> u32 {
    const N: u32 = 32;
    let start = DWT::cycle_count();
    for _ in 0..N {
        f();
    }
    DWT::cycle_count().wrapping_sub(start) / N
}

fn q(v: f32) -> Q3_28 {
    Q3_28::checked_from_f32(v).unwrap_or(Q3_28::ZERO)
}

/// bench::PLANの設定値で、電流ループの各計算を測ってログに出す
pub fn run() {
    let mut settings = Settings::new();
    for &(id, value) in bench::PLAN.settings {
        let _ = settings.set(id, value);
    }
    let Ok(params) = settings.build() else {
        defmt::warn!("profile: settings are invalid");
        return;
    };
    let p = params.inner;

    // SAFETY: サイクルカウンタを有効にするだけ。DCBとDWTはほかで使っていない
    let mut core = unsafe { cortex_m::Peripherals::steal() };
    core.DCB.enable_trace();
    core.DWT.enable_cycle_counter();

    // f32からの変換はソフトウェアの浮動小数点演算で重い。測る区間に入れないよう、先に済ませる
    let meas = |i: Q3_28| Measurement { vdc: q(0.96), vdc_inv: q(1.0 / 0.96), i, w: q(0.03), th: Q16_16::ZERO };
    let (u_small, u_mid, u_over, duty) = (q(0.02), q(0.3), q(2.0), q(0.05));
    // ADCで5LSBぶんの電流(約0.1A)。実際の割り込みと同じ換算を通した値
    let i_small = p.sense.current(15, 10, 1);

    let empty = cycles(|| {
        black_box(0u32);
    });

    // i = 0。出力が0のときや、読めていないときはこれになる
    let mut state = CurrentState::new();
    let m = meas(Q3_28::ZERO);
    let update_zero = cycles(|| {
        black_box(state.update(black_box(&p.current), black_box(u_small), black_box(&m)));
    });

    // 0 < |i| < i_threshold。デッドタイム補償の符号を割り算で求める経路
    let mut state = CurrentState::new();
    let m = meas(i_small);
    let update_low = cycles(|| {
        black_box(state.update(black_box(&p.current), black_box(u_small), black_box(&m)));
    });

    // |i| >= i_threshold。割り算をしない経路
    let mut state = CurrentState::new();
    let m = meas(u_mid);
    let update_high = cycles(|| {
        black_box(state.update(black_box(&p.current), black_box(u_mid), black_box(&m)));
    });

    // 目標値がimaxを超え、電圧も上限に張り付く。iはしきい値以上
    let mut state = CurrentState::new();
    let m = meas(u_mid);
    let update_sat = cycles(|| {
        black_box(state.update(black_box(&p.current), black_box(u_over), black_box(&m)));
    });

    let sense = cycles(|| {
        black_box(p.sense.current(black_box(250), black_box(10), black_box(1)));
    });

    let compare = cycles(|| {
        black_box(pwm::duty_to_compare(black_box(duty), black_box(PWM_ARR)));
    });

    defmt::info!(
        "profile[cycles]: empty={} update(i=0)={} update(low i)={} update(high i)={} update(saturated)={} sense={} compare={}",
        empty,
        update_zero,
        update_low,
        update_high,
        update_sat,
        sense,
        compare
    );
}
