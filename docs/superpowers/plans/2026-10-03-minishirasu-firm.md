# minishirasu-firm Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** mini-shirasu 基板（STM32F103C8T6）で、CAN から受けた目標値に従ってブラシ付き DC モーターを電流・速度・位置制御するファームを作る。

**Architecture:** md-core には「エンコーダの差分 → 位置・速度」と「固定小数点型と整数の境界の演算」だけを足す。それ以外は minishirasu-firm に置く。minishirasu-firm はレジスタに触らないライブラリ部（ホストでテスト）と、RTICX・レジスタ・bxcan を扱うバイナリ部（thumbv7m 専用）に分ける。

**Tech Stack:** Rust 2024、`md-core`、`rticx-cortex-m` 0.2（`swtasks`）、`stm32f1` 0.15 PAC（レジスタ直接操作）、`bxcan` 0.8、`cobs` 0.5、`crc` 3、`defmt`。

**Spec:** `docs/superpowers/specs/2026-10-03-minishirasu-firm-design.md`

## Global Constraints

- `docs/制御.md` は編集しない。
- md-core は制御の計算と数値型だけを持つ。追加は `encoder` モジュールと境界の演算4つ（`Q3_28::scale_int`、`Q3_28::mul_int`、`Q16_16::scale_int`、`Q16_16::from_ratio`）だけ。既存の `Config` と制御器は変えない。
- md-core の数値型は固定小数点版（`fixed/int.rs`）と浮動小数点版（`fixed/float.rs`）に同じ API を持たせる。テストは既定と `--features f32` の両方で通す。
- md-core のソースはタブでインデントする。minishirasu-firm はスペース4つ。
- 設定値に既定値は持たせない。`config.rs` のコンパイル時定数は CAN ID と制御周期（50µs、1ms）、`PWM_ARR` だけ。
- レジスタは `stm32f1` PAC を直接操作する。`stm32f1xx-hal` は使わない。
- 20kHz の割り込み（`CurrentLoop`）の中で f32 / f64 の演算をしない。
- ハードウェア系の依存は `[target.'cfg(target_os = "none")'.dependencies]` に置く。
- ホストのテストはリポジトリのルートで実行する。thumbv7m のビルドは `minishirasu-firm/` をカレントにして実行する（`.cargo/config.toml` がそこにあるため）。
- 実機でモーターを回す確認はユーザーが行う。実装者は書き込みも実行もしない。

## Review Focus

仕様が暗に求めているが、放っておくとテストされない入力。各項目のテストは、所有するタスクに入れてある。

1. **CAN フレームが1つ欠けた、または途中から受信した**: 次の 0x00 で同期を取り戻し、壊れたメッセージ1つだけを捨てる（Task 5）。
2. **`SetParam` に NaN や無限大が来た**: 保存せずに拒否する（Task 6）。
3. **エンコーダの 16bit カウンタがサンプルの間にラップした**: 差分が正しい符号付きの値になる（Task 4）。
4. **現在のモードと合わない目標値、範囲外の目標値が来た**: 目標値を書き換えない（Task 8）。
5. **CAN バスに相手がいなくて送信バッファがあふれた**: メッセージを丸ごと捨て、途中で切れたメッセージを流さない（Task 5）。

## File Structure

| ファイル | 役割 | タスク |
|---|---|---|
| `md-core/src/fixed/int.rs`、`float.rs`、`fixed.rs` | 境界の演算と、そのテスト | 2 |
| `md-core/src/encoder.rs` | 差分カウント → 位置・速度 | 3 |
| `minishirasu-firm/Cargo.toml` | 依存の整理 | 1 |
| `minishirasu-firm/src/lib.rs` | ライブラリ部のモジュール宣言 | 1 |
| `minishirasu-firm/src/pwm.rs` | デューティ → コンペア値 | 4 |
| `minishirasu-firm/src/sense.rs` | ADC 生値 → per-unit、カウントの差分 | 4 |
| `minishirasu-firm/src/protocol.rs` | メッセージ、符号化・復号、ストリームパーサ | 5 |
| `minishirasu-firm/src/txbuf.rs` | 送信リングバッファ | 5 |
| `minishirasu-firm/src/config.rs` | 設定値、ID、定数、物理単位との換算 | 6 |
| `minishirasu-firm/src/state.rs` | 状態機械とコマンドの判定 | 7 |
| `minishirasu-firm/src/cascade.rs` | 目標値と外側ループの合成 | 8 |
| `minishirasu-firm/src/board.rs` | レジスタ操作（バイナリ部） | 9 |
| `minishirasu-firm/src/can.rs` | bxcan（バイナリ部） | 9 |
| `minishirasu-firm/src/shared.rs` | タスク間で渡す型（バイナリ部） | 10 |
| `minishirasu-firm/src/main.rs` | RTICX の `app` | 1、10 |
| `minishirasu-firm/spec.md`、`docs/数値表現.md` | 文書 | 2、11 |

---

### Task 1: クレートの土台と RTICX の骨組み

現状、ルートの `Cargo.toml` は `minishirasu-firm` をメンバーにしているが、`src/` が空なのでワークスペース全体が読み込めない（`cargo test -p md-core` も失敗する）。まずここを直す。あわせて、RTICX のハードウェアタスク・ソフトウェアタスク・共有リソースの書き方が実際にビルドできることを、最小の骨組みで確かめる。

**Files:**
- Modify: `minishirasu-firm/Cargo.toml`（全体を置き換え）
- Create: `minishirasu-firm/src/lib.rs`
- Create: `minishirasu-firm/src/main.rs`
- Delete: `minishirasu-firm/tests/`（空ディレクトリ）

**Interfaces:**
- Consumes: なし
- Produces: クレート `minishirasu_firm`（ライブラリ）。RTICX の書き方（Task 10 がそのまま使う）:
  - `#[rticx_cortex_m::app(device = stm32f1::stm32f103, dispatchers = [SPI1, SPI2])]`
  - ハードウェアタスク: `#[task(binds = IRQ, priority = N, shared = [a, b])] struct T { 状態 }` と `impl RticTask for T { fn exec(&mut self) }`
  - ソフトウェアタスク: `#[sw_task(priority = N, capacity = M, shared = [a])] struct T { 状態 }` と `impl RticSwTask for T { type SpawnInput = X; fn exec(&mut self, input: X) }`。起動は `T::spawn(x)`（戻り値 `Result<(), X>`）
  - 共有リソース: `let mut s = self.shared(); s.a.lock(|a| ...)`
  - `init` は `(Shared, TaskInits)` を返す。`TaskInits` のフィールド名はタスク型名のスネークケース

- [ ] **Step 1: Cargo.toml を置き換える**

`minishirasu-firm/Cargo.toml`:

```toml
[package]
authors = ["Stew-000-1-0-011"]
name = "minishirasu-firm"
edition = "2024"
version = "0.1.0"

[[bin]]
name = "minishirasu-firm"
path = "src/main.rs"
test = false

[dependencies]
md-core = { path = "../md-core" }
cobs = { version = "0.5", default-features = false }
crc = "3"

# レジスタに触る部分だけが使う。ホストでのテスト時にはビルドしない
[target.'cfg(target_os = "none")'.dependencies]
cortex-m = { version = "0.7", features = ["critical-section-single-core"] }
cortex-m-rt = "0.7"
defmt = "1.0"
defmt-rtt = "1.0"
panic-probe = { version = "1.0", features = ["print-defmt"] }
stm32f1 = { version = "0.15", features = ["stm32f103", "rt"] }
bxcan = "0.8"
rticx-cortex-m = { version = "0.2", features = ["swtasks"] }

# [profile] はワークスペースルートの Cargo.toml にある
```

- [ ] **Step 2: 空の tests ディレクトリを消す**

```powershell
Remove-Item minishirasu-firm/tests -Recurse -Force -Confirm:$false
```

- [ ] **Step 3: lib.rs を作る**

`minishirasu-firm/src/lib.rs`:

```rust
//! minishirasu-firm のうち、レジスタに触らない部分。ホストでテストする
#![cfg_attr(not(test), no_std)]
```

- [ ] **Step 4: RTICX の骨組みを main.rs に書く**

`minishirasu-firm/src/main.rs`:

```rust
#![cfg_attr(target_os = "none", no_std, no_main)]

// ホストでは何もしない。ワークスペース全体を `cargo build` できるようにするためだけにある
#[cfg(not(target_os = "none"))]
fn main() {}

#[cfg(target_os = "none")]
use defmt_rtt as _; // global logger
#[cfg(target_os = "none")]
use panic_probe as _;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[cfg(target_os = "none")]
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

#[cfg(target_os = "none")]
#[rticx_cortex_m::app(device = stm32f1::stm32f103, dispatchers = [SPI1, SPI2])]
mod app {
    #[shared]
    struct Shared {
        count: u32,
    }

    #[init]
    fn init() -> (Shared, TaskInits) {
        (
            Shared { count: 0 },
            TaskInits { idle: Idle, current_loop: CurrentLoop { tick: 0 }, outer_loop: OuterLoop { seen: 0 }, command: Command },
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

    #[task(binds = ADC1_2, priority = 4, shared = [count])]
    struct CurrentLoop {
        tick: u32,
    }

    impl RticTask for CurrentLoop {
        fn exec(&mut self) {
            let mut s = self.shared();
            s.count.lock(|c| *c += 1);
            self.tick += 1;
            if self.tick >= 20 {
                self.tick = 0;
                let _ = OuterLoop::spawn(self.tick as u16);
            }
        }
    }

    #[sw_task(priority = 3, shared = [count])]
    struct OuterLoop {
        seen: u32,
    }

    impl RticSwTask for OuterLoop {
        type SpawnInput = u16;

        fn exec(&mut self, input: u16) {
            let mut s = self.shared();
            self.seen = s.count.lock(|c| *c) + input as u32;
            let _ = Command::spawn(());
        }
    }

    #[sw_task(priority = 1, capacity = 4)]
    struct Command;

    impl RticSwTask for Command {
        type SpawnInput = ();

        fn exec(&mut self, _input: ()) {
            defmt::info!("command");
        }
    }
}
```

- [ ] **Step 5: ホストでワークスペースが読み込めることを確かめる**

Run（ルートで）: `cargo test -p md-core`
Expected: 既存のテストがすべて PASS。

Run（ルートで）: `cargo test -p minishirasu-firm --lib`
Expected: `running 0 tests`、`test result: ok`。

- [ ] **Step 6: thumbv7m でビルドできることを確かめる**

Run:

```powershell
Push-Location minishirasu-firm; cargo build --release; Pop-Location
```

Expected: `Finished`。

マクロがこの書き方を受け付けずにエラーになった場合は、エラーメッセージと次のソースを読んで書き方を直す。直した書き方は Task 10 でも同じように使うので、このタスクの報告に書き残す。タスクの名前、優先度、割り込みの割り当ては変えない。

- `~/.cargo/registry/src/index.crates.io-*/rticx-core-0.2.0/tests/common/mod.rs`（ハードウェアタスクと共有リソースの例）
- `~/.cargo/registry/src/index.crates.io-*/rticx-sw-pass-0.2.0/README.md`（ソフトウェアタスクの例）
- `~/.cargo/registry/src/index.crates.io-*/rticx-sw-pass-0.2.0/src/common/codegen.rs`（`spawn` の生成コード）

- [ ] **Step 7: コミット**

ディレクトリ名の変更（`minisirasu-firm` → `minishirasu-firm`）もこのコミットに入る。

```powershell
git add -A minisirasu-firm minishirasu-firm CLAUDE.md Cargo.toml Cargo.lock
git commit -m "Rename to minishirasu-firm and add RTICX skeleton"
```

---

### Task 2: md-core に整数との境界の演算を足す

**Files:**
- Modify: `md-core/src/fixed/int.rs`
- Modify: `md-core/src/fixed/float.rs`
- Modify: `md-core/src/fixed.rs`（テスト）
- Modify: `docs/数値表現.md`

**Interfaces:**
- Consumes: なし
- Produces:
  - `Q3_28::scale_int(self, n: i32) -> i32`: per-unit 値 × 整数を最も近い整数に丸める（0.5 は 0 から遠い側）。あふれは飽和。
  - `Q3_28::mul_int(self, n: i32) -> Q3_28`: ゲイン × 整数。あふれは飽和。
  - `Q16_16::scale_int(self, n: i32) -> i32`: 位置 × 整数を最も近い整数に丸める。あふれは飽和。
  - `Q16_16::from_ratio(num: i32, den: u32) -> Q16_16`: `num / den`。あふれは飽和。`den == 0` は debug ビルドでパニック、release では `ZERO`。

- [ ] **Step 1: 失敗するテストを書く**

`md-core/src/fixed.rs` の `mod tests` の末尾（最後の `}` の直前）に足す:

```rust

	// ---- 整数との境界 ----

	#[test]
	fn q3_28_scale_int_rounds_to_nearest() {
		assert_eq!(q(0.5).scale_int(1800), 900);
		assert_eq!(q(-0.25).scale_int(1800), -450);
		assert_eq!(Q3_28::ZERO.scale_int(1800), 0);
		assert_eq!(Q3_28::ONE.scale_int(1800), 1800);
		// 0.3は2進で表せないが、丸めれば3になる
		assert_eq!(q(0.3).scale_int(10), 3);
		assert_eq!(q(-0.3).scale_int(10), -3);
		// 2.6 -> 3、-2.6 -> -3
		assert_eq!(q(0.26).scale_int(10), 3);
		assert_eq!(q(-0.26).scale_int(10), -3);
	}

	#[test]
	fn q3_28_scale_int_saturates() {
		assert_eq!(q(4.0).scale_int(i32::MAX), i32::MAX);
		assert_eq!(q(-4.0).scale_int(i32::MAX), i32::MIN);
	}

	#[test]
	fn q3_28_mul_int() {
		assert_close(q(0.125).mul_int(3).to_f32(), 0.375);
		assert_close(q(0.001).mul_int(100).to_f32(), 0.1);
		assert_close(q(0.001).mul_int(-100).to_f32(), -0.1);
		assert_eq!(q(0.5).mul_int(0), Q3_28::ZERO);
	}

	#[test]
	fn q3_28_mul_int_saturates() {
		assert_eq!(Q3_28::ONE.mul_int(100), Q3_28::MAX);
		assert_eq!(Q3_28::ONE.mul_int(-100), Q3_28::MIN);
	}

	#[test]
	fn q16_16_scale_int_rounds_to_nearest() {
		// 65536倍するとQ16.16の生の値になる
		assert_eq!(q16(1.5).scale_int(65536), 98304);
		assert_eq!(q16(-0.25).scale_int(65536), -16384);
		// 7.5 -> 8、-7.5 -> -8
		assert_eq!(q16(2.5).scale_int(3), 8);
		assert_eq!(q16(-2.5).scale_int(3), -8);
	}

	#[test]
	fn q16_16_scale_int_saturates() {
		assert_eq!(q16(30000.0).scale_int(i32::MAX), i32::MAX);
		assert_eq!(q16(-30000.0).scale_int(i32::MAX), i32::MIN);
	}

	#[test]
	fn q16_16_from_ratio() {
		assert_close(Q16_16::from_ratio(1, 4).to_f32(), 0.25);
		assert_close(Q16_16::from_ratio(26624, 8192).to_f32(), 3.25);
		assert_close(Q16_16::from_ratio(-2048, 8192).to_f32(), -0.25);
		assert_eq!(Q16_16::from_ratio(0, 8192), Q16_16::ZERO);
	}

	#[test]
	fn q16_16_from_ratio_saturates() {
		assert_eq!(Q16_16::from_ratio(i32::MAX, 1), Q16_16::MAX);
		assert_eq!(Q16_16::from_ratio(i32::MIN, 1), Q16_16::MIN);
	}

	#[test]
	fn q16_16_from_ratio_roundtrips_raw_value() {
		// 通信ではQ16.16の生の値をやり取りする
		assert_eq!(Q16_16::from_ratio(98304, 65536).scale_int(65536), 98304);
		assert_eq!(Q16_16::from_ratio(-12345, 65536).scale_int(65536), -12345);
	}

	#[test]
	#[cfg(debug_assertions)]
	#[should_panic]
	fn q16_16_from_ratio_zero_denominator_panics_in_debug() {
		let _ = Q16_16::from_ratio(1, 0);
	}
```

- [ ] **Step 2: 失敗を確認する**

Run: `cargo test -p md-core fixed`
Expected: コンパイルエラー（`no method named scale_int` など）。

- [ ] **Step 3: 固定小数点版を実装する**

`md-core/src/fixed/int.rs` の `scale_f32` 関数の直後に足す:

```rust

/// 2^frac を1.0とする値を、最も近い整数に丸める。0.5は0から遠い側
const fn round_shift(p: i64, frac: u32) -> i64 {
	let half = 1i64 << (frac - 1);
	if p >= 0 { (p + half) >> frac } else { -((-p + half) >> frac) }
}
```

`impl Q3_28` の `to_q16_16` の直後に足す:

```rust

	/// per-unit値に整数を掛け、最も近い整数に丸める。あふれは飽和
	pub fn scale_int(self, n: i32) -> i32 {
		sat_i32(round_shift(self.0 as i64 * n as i64, Self::FRAC))
	}

	/// ゲインに整数を掛ける。あふれは飽和
	pub fn mul_int(self, n: i32) -> Self {
		Q3_28(sat_i32(self.0 as i64 * n as i64))
	}
```

`impl Q16_16` の `to_q3_28` の直後に足す:

```rust

	/// 位置に整数を掛け、最も近い整数に丸める。あふれは飽和
	pub fn scale_int(self, n: i32) -> i32 {
		sat_i32(round_shift(self.0 as i64 * n as i64, Self::FRAC))
	}

	/// num / den。あふれは飽和。denが0でないことは呼び出し側が保証する。
	/// debugビルドでは違反を検出する。releaseでは0を返す
	pub fn from_ratio(num: i32, den: u32) -> Self {
		debug_assert!(den != 0, "from_ratio: den is zero");
		if den == 0 {
			return Self::ZERO;
		}
		Q16_16(sat_i32(((num as i64) << Self::FRAC) / den as i64))
	}
```

- [ ] **Step 4: 浮動小数点版を実装する**

`md-core/src/fixed/float.rs` の `use core::ops::...;` の直後に足す:

```rust

/// 最も近い整数に丸める。0.5は0から遠い側。範囲外は飽和
fn round_to_i32(x: f64) -> i32 {
	(if x >= 0.0 { x + 0.5 } else { x - 0.5 }) as i32
}
```

`impl Q3_28` の `to_q16_16` の直後に足す:

```rust

	/// per-unit値に整数を掛け、最も近い整数に丸める。あふれは飽和
	pub fn scale_int(self, n: i32) -> i32 {
		round_to_i32(self.0 as f64 * n as f64)
	}

	/// ゲインに整数を掛ける。あふれは飽和
	pub fn mul_int(self, n: i32) -> Self {
		Self::sat(self.0 * n as f32)
	}
```

`impl Q16_16` の `to_q3_28` の直後に足す:

```rust

	/// 位置に整数を掛け、最も近い整数に丸める。あふれは飽和
	pub fn scale_int(self, n: i32) -> i32 {
		round_to_i32(self.0 * n as f64)
	}

	/// num / den。あふれは飽和。denが0でないことは呼び出し側が保証する。
	/// debugビルドでは違反を検出する。releaseでは0を返す
	pub fn from_ratio(num: i32, den: u32) -> Self {
		debug_assert!(den != 0, "from_ratio: den is zero");
		if den == 0 {
			return Self::ZERO;
		}
		Self::sat(num as f64 / den as f64)
	}
```

- [ ] **Step 5: 両方の実装でテストを通す**

Run: `cargo test -p md-core`
Expected: すべて PASS。

Run: `cargo test -p md-core --features f32`
Expected: すべて PASS。

- [ ] **Step 6: docs/数値表現.md に追記する**

「## 演算の約束」の箇条書きの最後（「型をまたぐ変換は…」の行）の直後に足す:

```markdown
- 整数との境界は次の4つだけ。ADC の生値やエンコーダのカウント、タイマのコンペア値とのやり取りに使う。制御周期の中で浮動小数点を使わずに済ませるためのもの。
  - `Q3_28::scale_int(n)`：per-unit 値 × 整数を、最も近い整数に丸める（デューティ × タイマ周期 → コンペア値）。
  - `Q3_28::mul_int(n)`：ゲイン × 整数（ADC 生値 × 換算係数 → 電流・電圧）。
  - `Q16_16::scale_int(n)`：位置 × 整数を、最も近い整数に丸める（`n = 65536` で Q16.16 の生の値になる）。
  - `Q16_16::from_ratio(num, den)`：整数の比（累積カウント ÷ 1 回転あたりのカウント → 位置）。
```

- [ ] **Step 7: コミット**

```powershell
git add md-core/src/fixed.rs md-core/src/fixed/int.rs md-core/src/fixed/float.rs docs/数値表現.md
git commit -m "Add integer boundary operations to fixed-point types"
```

---

### Task 3: md-core に encoder モジュールを足す

**Files:**
- Create: `md-core/src/encoder.rs`
- Modify: `md-core/src/lib.rs`

**Interfaces:**
- Consumes: Task 2 の `Q3_28::mul_int`、`Q16_16::from_ratio`。既存の `md_core::controller::ConfigError`。
- Produces:
  - `EncoderParam::new(counts_per_rev: u32, reversed: bool, period: f32, wbase: f32, alpha: f32) -> Result<EncoderParam, ConfigError>`
    - エラーのフィールド名: `"encoder_cpr"`、`"wperiod"`、`"wbase"`、`"w_filter_alpha"`
  - `EncoderState::new() -> EncoderState`
  - `EncoderState::update(&mut self, p: &EncoderParam, delta: i32) -> (Q16_16, Q3_28)`（位置 [回転]、速度 [pu]）
  - `EncoderState::set_origin(&mut self)`

- [ ] **Step 1: 失敗するテストを書く**

`md-core/src/lib.rs` を次の内容にする:

```rust
#![no_std]

pub mod controller;
pub mod encoder;
pub mod fixed;
```

`md-core/src/encoder.rs` を作る（まずテストだけ）:

```rust
//! エンコーダの差分カウントから位置と速度を求める

#[cfg(test)]
mod tests {
	use super::*;
	use core::f32::consts::TAU;

	fn assert_close(actual: f32, expected: f32) {
		assert!((actual - expected).abs() < 1e-6, "left: {actual}, right: {expected}");
	}

	/// 1000カウント/回転、1ms周期、基準速度10回転/s。1カウント/周期が0.1puになる
	fn param(reversed: bool, alpha: f32) -> EncoderParam {
		EncoderParam::new(1000, reversed, 1e-3, TAU * 10.0, alpha).ok().unwrap()
	}

	#[test]
	fn position_accumulates_counts() {
		let p = param(false, 1.0);
		let mut st = EncoderState::new();
		assert_close(st.update(&p, 250).0.to_f32(), 0.25);
		assert_close(st.update(&p, 750).0.to_f32(), 1.0);
		assert_close(st.update(&p, -1500).0.to_f32(), -0.5);
	}

	#[test]
	fn velocity_is_delta_times_gain_without_filter() {
		let p = param(false, 1.0);
		let mut st = EncoderState::new();
		assert_close(st.update(&p, 5).1.to_f32(), 0.5);
		assert_close(st.update(&p, -2).1.to_f32(), -0.2);
		assert_close(st.update(&p, 0).1.to_f32(), 0.0);
	}

	#[test]
	fn reversed_flips_position_and_velocity() {
		let p = param(true, 1.0);
		let mut st = EncoderState::new();
		let (th, w) = st.update(&p, 5);
		assert_close(th.to_f32(), -0.005);
		assert_close(w.to_f32(), -0.5);
	}

	#[test]
	fn velocity_filter_is_first_order() {
		let p = param(false, 0.5);
		let mut st = EncoderState::new();
		// 入力は0.4で一定。0.2 -> 0.3 -> 0.35 と近づく
		assert_close(st.update(&p, 4).1.to_f32(), 0.2);
		assert_close(st.update(&p, 4).1.to_f32(), 0.3);
		assert_close(st.update(&p, 4).1.to_f32(), 0.35);
	}

	#[test]
	fn set_origin_clears_position_and_keeps_velocity() {
		let p = param(false, 0.5);
		let mut st = EncoderState::new();
		// 位置は0.004回転、速度は 0.5 * 0.4 = 0.2
		st.update(&p, 4);
		st.set_origin();
		let (th, w) = st.update(&p, 0);
		assert_close(th.to_f32(), 0.0);
		// 速度は0に飛ばず、フィルタに従って 0.2 -> 0.1 と減る
		assert_close(w.to_f32(), 0.1);
	}

	#[test]
	fn position_saturates_instead_of_wrapping() {
		let p = EncoderParam::new(1, false, 1.0, 1000.0, 1.0).ok().unwrap();
		let mut st = EncoderState::new();
		st.update(&p, i32::MAX);
		assert_eq!(st.update(&p, i32::MAX).0, Q16_16::MAX);

		let mut st = EncoderState::new();
		st.update(&p, i32::MIN);
		assert_eq!(st.update(&p, i32::MIN).0, Q16_16::MIN);
	}

	#[test]
	fn new_rejects_zero_counts_per_rev() {
		assert_eq!(
			EncoderParam::new(0, false, 1e-3, 300.0, 1.0).err(),
			Some(ConfigError::NotPositive("encoder_cpr"))
		);
	}

	#[test]
	fn new_rejects_bad_period_and_base() {
		assert_eq!(
			EncoderParam::new(1000, false, 0.0, 300.0, 1.0).err(),
			Some(ConfigError::NotPositive("wperiod"))
		);
		assert_eq!(
			EncoderParam::new(1000, false, 1e-3, f32::NAN, 1.0).err(),
			Some(ConfigError::NotPositive("wbase"))
		);
	}

	#[test]
	fn new_rejects_alpha_outside_zero_to_one() {
		assert_eq!(
			EncoderParam::new(1000, false, 1e-3, 300.0, 0.0).err(),
			Some(ConfigError::NotPositive("w_filter_alpha"))
		);
		assert_eq!(
			EncoderParam::new(1000, false, 1e-3, 300.0, f32::NAN).err(),
			Some(ConfigError::NotPositive("w_filter_alpha"))
		);
		assert_eq!(
			EncoderParam::new(1000, false, 1e-3, 300.0, 1.5).err(),
			Some(ConfigError::OutOfRange("w_filter_alpha"))
		);
		assert!(EncoderParam::new(1000, false, 1e-3, 300.0, 1.0).is_ok());
	}

	#[test]
	fn new_rejects_velocity_gain_out_of_range() {
		// 2π / (1 * 1e-3 * 1) = 6283 はQ3.28に入らない
		assert_eq!(
			EncoderParam::new(1, false, 1e-3, 1.0, 1.0).err(),
			Some(ConfigError::OutOfRange("encoder_cpr"))
		);
	}

	#[test]
	#[cfg(not(feature = "f32"))]  // 浮動小数点版には分解能がなく、小さな係数も0にならない
	fn new_rejects_velocity_gain_that_quantizes_to_zero() {
		// 2π / (4e9 * 1 * 1) はQ3.28の分解能より小さい。速度が常に0になってしまう
		assert_eq!(
			EncoderParam::new(4_000_000_000, false, 1.0, 1.0, 1.0).err(),
			Some(ConfigError::OutOfRange("encoder_cpr"))
		);
	}
}
```

- [ ] **Step 2: 失敗を確認する**

Run: `cargo test -p md-core encoder`
Expected: コンパイルエラー（`cannot find type EncoderParam` など）。

- [ ] **Step 3: 実装する**

`md-core/src/encoder.rs` の先頭のコメント行の直後（`#[cfg(test)]` の前）に足す:

```rust

use crate::controller::ConfigError;
use crate::fixed::{Q3_28, Q16_16};

pub struct EncoderParam {
	counts_per_rev: u32,
	reversed: bool,
	w_gain: Q3_28,  // 1周期に1カウント進んだときの速度[pu]
	alpha: Q3_28,  // 速度の1次ローパスの係数
}

impl EncoderParam {
	/// counts_per_rev: 1回転あたりのカウント数(逓倍後)。period: 更新周期[s]。
	/// wbase: 速度の基準値[rad/s]。alpha: 速度の1次ローパスの係数。(0, 1]で、1ならフィルタなし
	pub fn new(
		counts_per_rev: u32,
		reversed: bool,
		period: f32,
		wbase: f32,
		alpha: f32,
	) -> Result<EncoderParam, ConfigError> {
		if counts_per_rev == 0 {
			return Err(ConfigError::NotPositive("encoder_cpr"));
		}
		if !(period > 0.0 && period.is_finite()) {
			return Err(ConfigError::NotPositive("wperiod"));
		}
		if !(wbase > 0.0 && wbase.is_finite()) {
			return Err(ConfigError::NotPositive("wbase"));
		}
		// NaNもここで弾く
		if !(alpha > 0.0) {
			return Err(ConfigError::NotPositive("w_filter_alpha"));
		}
		if alpha > 1.0 {
			return Err(ConfigError::OutOfRange("w_filter_alpha"));
		}

		let w_gain = core::f32::consts::TAU / (counts_per_rev as f32 * period * wbase);
		let w_gain = Q3_28::checked_from_f32(w_gain).ok_or(ConfigError::OutOfRange("encoder_cpr"))?;
		// 丸めで0になると、回っていても速度が0に見える
		if w_gain == Q3_28::ZERO {
			return Err(ConfigError::OutOfRange("encoder_cpr"));
		}

		Ok(EncoderParam {
			counts_per_rev,
			reversed,
			w_gain,
			alpha: Q3_28::checked_from_f32(alpha).ok_or(ConfigError::OutOfRange("w_filter_alpha"))?,
		})
	}
}

pub struct EncoderState {
	count: i32,  // 原点からの累積カウント
	w: Q3_28,
}

impl EncoderState {
	pub fn new() -> EncoderState {
		EncoderState { count: 0, w: Q3_28::ZERO }
	}

	/// 前回からの差分カウントを受け取り、(位置[回転], 速度[pu])を返す。
	/// カウンタのビット幅によるラップは呼び出し側で解いておく
	pub fn update(&mut self, p: &EncoderParam, delta: i32) -> (Q16_16, Q3_28) {
		let delta = if p.reversed { delta.saturating_neg() } else { delta };
		self.count = self.count.saturating_add(delta);
		self.w = self.w + p.alpha * (p.w_gain.mul_int(delta) - self.w);
		(Q16_16::from_ratio(self.count, p.counts_per_rev), self.w)
	}

	/// 現在位置を0にする。速度はそのまま
	pub fn set_origin(&mut self) {
		self.count = 0;
	}
}
```

- [ ] **Step 4: 両方の実装でテストを通す**

Run: `cargo test -p md-core`
Expected: すべて PASS。

Run: `cargo test -p md-core --features f32`
Expected: すべて PASS。

- [ ] **Step 5: コミット**

```powershell
git add md-core/src/lib.rs md-core/src/encoder.rs
git commit -m "Add encoder position and velocity estimation"
```

---

### Task 4: pwm と sense

**Files:**
- Create: `minishirasu-firm/src/pwm.rs`
- Create: `minishirasu-firm/src/sense.rs`
- Modify: `minishirasu-firm/src/lib.rs`

**Interfaces:**
- Consumes: `Q3_28::scale_int`、`Q3_28::mul_int`（Task 2）、`md_core::controller::ConfigError`。
- Produces:
  - `pwm::duty_to_compare(duty: Q3_28, arr: u16) -> (u16, u16)`: (PWMA 側、PWMB 側)
  - `pwm::compare_sign(compare: (u16, u16)) -> i32`: +1 / −1 / 0
  - `sense::SenseParam`（`Clone, Copy`）、`SenseParam::new(ibase: f32, vdcmax: f32) -> Result<SenseParam, ConfigError>`
  - `SenseParam::current(&self, raw: u16, offset: u16, sign: i32) -> Q3_28`
  - `SenseParam::bus_voltage(&self, raw: u16) -> Q3_28`
  - `sense::count_delta(prev: u16, now: u16) -> i32`

- [ ] **Step 1: lib.rs にモジュールを足す**

`minishirasu-firm/src/lib.rs`:

```rust
//! minishirasu-firm のうち、レジスタに触らない部分。ホストでテストする
#![cfg_attr(not(test), no_std)]

pub mod pwm;
pub mod sense;
```

- [ ] **Step 2: pwm の失敗するテストを書く**

`minishirasu-firm/src/pwm.rs`:

```rust
//! 符号付きデューティをタイマのコンペア値にする

#[cfg(test)]
mod tests {
    use super::*;

    fn q(v: f32) -> Q3_28 {
        Q3_28::checked_from_f32(v).unwrap()
    }

    #[test]
    fn positive_duty_drives_a_side() {
        assert_eq!(duty_to_compare(q(0.5), 1800), (900, 0));
    }

    #[test]
    fn negative_duty_drives_b_side() {
        assert_eq!(duty_to_compare(q(-0.25), 1800), (0, 450));
    }

    #[test]
    fn zero_duty_keeps_both_low() {
        assert_eq!(duty_to_compare(Q3_28::ZERO, 1800), (0, 0));
    }

    #[test]
    fn duty_beyond_one_is_clamped_to_period() {
        assert_eq!(duty_to_compare(q(2.0), 1800), (1800, 0));
        assert_eq!(duty_to_compare(Q3_28::MIN, 1800), (0, 1800));
    }

    #[test]
    fn duty_too_small_to_represent_becomes_zero() {
        // 1カウントの半分に満たないデューティは0に丸まる
        assert_eq!(duty_to_compare(q(0.0001), 1800), (0, 0));
    }

    #[test]
    fn compare_sign_follows_driven_side() {
        assert_eq!(compare_sign((900, 0)), 1);
        assert_eq!(compare_sign((0, 450)), -1);
        assert_eq!(compare_sign((0, 0)), 0);
    }
}
```

- [ ] **Step 3: sense の失敗するテストを書く**

`minishirasu-firm/src/sense.rs`:

```rust
//! ADCの生値とエンコーダのカウントを、制御器が使う値にする

#[cfg(test)]
mod tests {
    use super::*;

    fn param() -> SenseParam {
        SenseParam::new(20.0, 36.0).unwrap()
    }

    fn assert_near(actual: Q3_28, expected: f32) {
        let a = actual.to_f32();
        assert!((a - expected).abs() < 1e-4, "left: {a}, right: {expected}");
    }

    #[test]
    fn current_scales_offset_corrected_raw_value() {
        // 1LSB = 3.3 / 4096 / (0.002 * 20) = 0.020142 A。2000LSBで40.28A = 2.0142pu
        assert_near(param().current(2146, 146, 1), 2.0141602);
    }

    #[test]
    fn current_sign_follows_duty_direction() {
        assert_near(param().current(2146, 146, -1), -2.0141602);
    }

    #[test]
    fn current_is_zero_when_no_side_is_driven() {
        assert_eq!(param().current(2146, 146, 0), Q3_28::ZERO);
    }

    #[test]
    fn current_below_offset_is_negative() {
        // 回生でシャントに逆向きの電流が流れると、オフセットより下がる
        assert_near(param().current(46, 146, 1), -0.10070801);
    }

    #[test]
    fn bus_voltage_scales_divider() {
        // 2048LSB = 1.65V、11倍して18.15V。基準36Vで0.50417pu
        assert_near(param().bus_voltage(2048), 0.50416666);
        assert_eq!(param().bus_voltage(0), Q3_28::ZERO);
    }

    #[test]
    fn new_rejects_non_positive_bases() {
        assert_eq!(SenseParam::new(0.0, 36.0).err(), Some(ConfigError::NotPositive("ibase")));
        assert_eq!(SenseParam::new(20.0, f32::NAN).err(), Some(ConfigError::NotPositive("vdcmax")));
        assert_eq!(SenseParam::new(f32::INFINITY, 36.0).err(), Some(ConfigError::NotPositive("ibase")));
    }

    #[test]
    fn count_delta_is_signed_difference() {
        assert_eq!(count_delta(100, 130), 30);
        assert_eq!(count_delta(130, 100), -30);
        assert_eq!(count_delta(5, 5), 0);
    }

    #[test]
    fn count_delta_unwraps_16bit_counter() {
        // 0xFFFFから0へ、0から0xFFFFへまたいでも連続した差分になる
        assert_eq!(count_delta(0xFFF0, 0x0010), 32);
        assert_eq!(count_delta(0x0010, 0xFFF0), -32);
    }
}
```

- [ ] **Step 4: 失敗を確認する**

Run: `cargo test -p minishirasu-firm --lib`
Expected: コンパイルエラー（`cannot find function duty_to_compare` など）。

- [ ] **Step 5: pwm を実装する**

`minishirasu-firm/src/pwm.rs` の先頭のコメント行の直後に足す:

```rust

use md_core::fixed::Q3_28;

/// 符号付きデューティ(-1..=1)を、(PWMA側, PWMB側)のコンペア値にする。
/// 片側だけをPWMし、反対側は0(Low固定)にする。|duty| > 1 は arr で頭打ち
pub fn duty_to_compare(duty: Q3_28, arr: u16) -> (u16, u16) {
    let n = duty.scale_int(arr as i32);
    let magnitude = n.unsigned_abs().min(arr as u32) as u16;
    if n >= 0 { (magnitude, 0) } else { (0, magnitude) }
}

/// コンペア値の組が駆動している向き。PWMA側なら1、PWMB側なら-1、どちらも0なら0
pub fn compare_sign(compare: (u16, u16)) -> i32 {
    if compare.0 > 0 {
        1
    } else if compare.1 > 0 {
        -1
    } else {
        0
    }
}
```

- [ ] **Step 6: sense を実装する**

`minishirasu-firm/src/sense.rs` の先頭のコメント行の直後に足す:

```rust

use md_core::controller::ConfigError;
use md_core::fixed::Q3_28;

/// ADCの基準電圧[V]
const VREF: f32 = 3.3;
/// 12bit ADCの分解能
const ADC_STEPS: f32 = 4096.0;
/// シャント抵抗[Ω]
const SHUNT_OHM: f32 = 0.002;
/// 電流センスアンプ(INA2181A1)のゲイン
const AMP_GAIN: f32 = 20.0;
/// 母線電圧の分圧比の逆数。(10k + 1k) / 1k
const VSENSE_RATIO: f32 = 11.0;

#[derive(Clone, Copy)]
pub struct SenseParam {
    i_gain: Q3_28, // 1LSBあたりの電流[pu]
    v_gain: Q3_28, // 1LSBあたりの母線電圧[pu]
}

fn gain(name: &'static str, lsb: f32, base: f32) -> Result<Q3_28, ConfigError> {
    if !(base > 0.0 && base.is_finite()) {
        return Err(ConfigError::NotPositive(name));
    }
    Q3_28::checked_from_f32(lsb / base).ok_or(ConfigError::OutOfRange(name))
}

impl SenseParam {
    /// ibase: 電流の基準値[A]。vdcmax: 電圧の基準値[V]
    pub fn new(ibase: f32, vdcmax: f32) -> Result<SenseParam, ConfigError> {
        Ok(SenseParam {
            i_gain: gain("ibase", VREF / ADC_STEPS / (SHUNT_OHM * AMP_GAIN), ibase)?,
            v_gain: gain("vdcmax", VREF / ADC_STEPS * VSENSE_RATIO, vdcmax)?,
        })
    }

    /// モーター電流[pu]。シャントにはON期間だけ同じ向きの電流が流れるので、
    /// その周期に駆動していた向き(sign: 1, -1, 0)で符号を復元する
    pub fn current(&self, raw: u16, offset: u16, sign: i32) -> Q3_28 {
        self.i_gain.mul_int((raw as i32 - offset as i32) * sign)
    }

    /// 母線電圧[pu]
    pub fn bus_voltage(&self, raw: u16) -> Q3_28 {
        self.v_gain.mul_int(raw as i32)
    }
}

/// 16bitカウンタの前回値と今回値から、符号付きの差分を求める。
/// 1周期に±32767カウントを超えて進まないことが前提
pub fn count_delta(prev: u16, now: u16) -> i32 {
    now.wrapping_sub(prev) as i16 as i32
}
```

- [ ] **Step 7: テストを通す**

Run: `cargo test -p minishirasu-firm --lib`
Expected: すべて PASS。

- [ ] **Step 8: コミット**

```powershell
git add minishirasu-firm/src/lib.rs minishirasu-firm/src/pwm.rs minishirasu-firm/src/sense.rs
git commit -m "Add PWM compare and ADC scaling"
```

---

### Task 5: protocol と txbuf

**Files:**
- Create: `minishirasu-firm/src/protocol.rs`
- Create: `minishirasu-firm/src/txbuf.rs`
- Modify: `minishirasu-firm/src/lib.rs`

**Interfaces:**
- Consumes: `cobs`、`crc`。
- Produces:
  - `protocol::MAX_ENCODED: usize = 32`
  - `protocol::kind::{TARGET_CURRENT, TARGET_VELOCITY, TARGET_POSITION, SET_MODE, SET_PARAM, RESET_FAULT, SET_ORIGIN, STATUS, ACK, NACK, FAULT_NOTICE}: u8`
  - `protocol::Status { mode: u8, flags: u8, current: f32, velocity: f32, position: i32, vdc: f32, temp: u16 }`（`Clone, Copy, PartialEq, Debug`）
  - `protocol::Message`（`Clone, Copy, PartialEq, Debug`）: `TargetCurrent { current: f32 }`、`TargetVelocity { velocity: f32, accel_ff: f32 }`、`TargetPosition { position: i32, velocity_ff: f32, accel_ff: f32 }`、`SetMode { mode: u8 }`、`SetParam { id: u8, value: f32 }`、`ResetFault`、`SetOrigin`、`Status(Status)`、`Ack { command: u8 }`、`Nack { command: u8, reason: u8, param: u8 }`、`FaultNotice`
  - `Message::kind(&self) -> u8`
  - `Message::encode(&self, out: &mut [u8; MAX_ENCODED]) -> usize`（末尾の 0x00 を含む長さ）
  - `Message::decode_body(body: &[u8]) -> Result<Message, DecodeError>`
  - `protocol::DecodeError`: `Cobs`、`TooShort`、`Crc`、`UnknownKind(u8)`、`Length(u8)`、`Overflow`
  - `protocol::StreamParser::new()`（`const fn`）、`StreamParser::push(&mut self, byte: u8) -> Option<Result<Message, DecodeError>>`
  - `txbuf::TxStream { Status, Response }`（`Clone, Copy, PartialEq, Debug`）
  - `txbuf::TxQueues::new()`（`const fn`）、`TxQueues::push(&mut self, stream: TxStream, message: &[u8]) -> bool`、`TxQueues::pop_chunk(&mut self, status_first: bool, out: &mut [u8; 8]) -> Option<(TxStream, usize)>`

- [ ] **Step 1: lib.rs にモジュールを足す**

`minishirasu-firm/src/lib.rs`:

```rust
//! minishirasu-firm のうち、レジスタに触らない部分。ホストでテストする
#![cfg_attr(not(test), no_std)]

pub mod protocol;
pub mod pwm;
pub mod sense;
pub mod txbuf;
```

- [ ] **Step 2: protocol の失敗するテストを書く**

`minishirasu-firm/src/protocol.rs`:

```rust
//! バイトストリーム上のメッセージ。
//! フレーミングは COBS( 種別 | ペイロード | CRC-8 ) のあとに区切りの 0x00。数値はリトルエンディアン

#[cfg(test)]
mod tests {
    use super::*;

    fn encode(m: &Message) -> Vec<u8> {
        let mut out = [0u8; MAX_ENCODED];
        let n = m.encode(&mut out);
        out[..n].to_vec()
    }

    fn feed(p: &mut StreamParser, bytes: &[u8]) -> Vec<Result<Message, DecodeError>> {
        bytes.iter().filter_map(|&b| p.push(b)).collect()
    }

    fn samples() -> Vec<Message> {
        vec![
            Message::TargetCurrent { current: 1.5 },
            Message::TargetVelocity { velocity: -120.0, accel_ff: 30.5 },
            Message::TargetPosition { position: -98304, velocity_ff: 2.0, accel_ff: 0.0 },
            Message::SetMode { mode: 2 },
            Message::SetParam { id: 0x21, value: 8192.0 },
            Message::ResetFault,
            Message::SetOrigin,
            Message::Status(Status {
                mode: 3,
                flags: 0b101,
                current: 0.25,
                velocity: 0.0,
                position: 0,
                vdc: 24.0,
                temp: 2048,
            }),
            Message::Ack { command: kind::SET_MODE },
            Message::Nack { command: kind::SET_PARAM, reason: 3, param: 0x04 },
            Message::FaultNotice,
        ]
    }

    #[test]
    fn crc_is_crc8_smbus() {
        // CRC-8/SMBUS の検査値
        assert_eq!(CRC8.checksum(b"123456789"), 0xF4);
    }

    #[test]
    fn every_message_roundtrips() {
        for m in samples() {
            let mut p = StreamParser::new();
            assert_eq!(feed(&mut p, &encode(&m)), vec![Ok(m)], "{m:?}");
        }
    }

    #[test]
    fn encoded_lengths_match_spec() {
        let lengths: Vec<usize> = samples().iter().map(|m| encode(m).len()).collect();
        assert_eq!(lengths, vec![8, 12, 16, 5, 9, 4, 4, 24, 5, 7, 4]);
    }

    #[test]
    fn encoded_message_has_zero_only_at_the_end() {
        for m in samples() {
            let bytes = encode(&m);
            let (last, body) = bytes.split_last().unwrap();
            assert_eq!(*last, 0, "{m:?}");
            assert!(body.iter().all(|&b| b != 0), "{m:?}");
        }
    }

    #[test]
    fn kinds_match_spec() {
        let kinds: Vec<u8> = samples().iter().map(|m| m.kind()).collect();
        assert_eq!(kinds, vec![0x01, 0x02, 0x03, 0x10, 0x11, 0x12, 0x13, 0x20, 0x30, 0x31, 0x32]);
    }

    #[test]
    fn consecutive_messages_are_split() {
        let a = Message::SetMode { mode: 1 };
        let b = Message::TargetCurrent { current: -0.5 };
        let mut bytes = encode(&a);
        bytes.extend(encode(&b));
        let mut p = StreamParser::new();
        assert_eq!(feed(&mut p, &bytes), vec![Ok(a), Ok(b)]);
    }

    #[test]
    fn extra_delimiters_are_ignored() {
        let m = Message::ResetFault;
        let mut bytes = vec![0, 0];
        bytes.extend(encode(&m));
        bytes.push(0);
        let mut p = StreamParser::new();
        assert_eq!(feed(&mut p, &bytes), vec![Ok(m)]);
    }

    #[test]
    fn joining_mid_message_drops_only_the_broken_one() {
        let a = Message::TargetVelocity { velocity: 1.0, accel_ff: 2.0 };
        let b = Message::SetOrigin;
        let first = encode(&a);
        // 1つ目は末尾の3バイト(区切りを含む)しか受け取れなかった
        let mut bytes = first[first.len() - 3..].to_vec();
        bytes.extend(encode(&b));
        let mut p = StreamParser::new();
        let got = feed(&mut p, &bytes);
        assert_eq!(got.len(), 2);
        assert!(got[0].is_err());
        assert_eq!(got[1], Ok(b));
    }

    #[test]
    fn lost_byte_drops_only_the_broken_message() {
        let a = Message::TargetPosition { position: 65536, velocity_ff: 1.0, accel_ff: 2.0 };
        let b = Message::Ack { command: 0x10 };
        let mut bytes = encode(&a);
        bytes.remove(5);
        bytes.extend(encode(&b));
        let mut p = StreamParser::new();
        let got = feed(&mut p, &bytes);
        assert_eq!(got.len(), 2);
        assert!(got[0].is_err());
        assert_eq!(got[1], Ok(b));
    }

    #[test]
    fn corrupted_byte_is_rejected() {
        let mut bytes = encode(&Message::SetParam { id: 0x04, value: 6.0 });
        // 区切りを作らないよう、0にも1にもならないバイトを選んで1bit反転する
        let i = bytes.iter().position(|&b| b != 0 && b != 1).unwrap();
        bytes[i] ^= 1;
        let mut p = StreamParser::new();
        let got = feed(&mut p, &bytes);
        assert_eq!(got.len(), 1);
        assert!(got[0].is_err());
    }

    #[test]
    fn overlong_input_is_dropped_and_parser_recovers() {
        let m = Message::FaultNotice;
        let mut bytes = vec![0x55u8; 100];
        bytes.push(0);
        bytes.extend(encode(&m));
        let mut p = StreamParser::new();
        assert_eq!(feed(&mut p, &bytes), vec![Err(DecodeError::Overflow), Ok(m)]);
    }

    #[test]
    fn decode_body_rejects_wrong_crc() {
        let body = [kind::SET_MODE, 1, 0x00];
        assert_ne!(CRC8.checksum(&body[..2]), 0x00);
        assert_eq!(Message::decode_body(&body), Err(DecodeError::Crc));
    }

    #[test]
    fn decode_body_rejects_unknown_kind() {
        let mut body = [0x7F, 0];
        body[1] = CRC8.checksum(&body[..1]);
        assert_eq!(Message::decode_body(&body), Err(DecodeError::UnknownKind(0x7F)));
    }

    #[test]
    fn decode_body_rejects_wrong_length() {
        // SetModeのペイロードは1バイトだが、2バイトある
        let mut body = [kind::SET_MODE, 1, 2, 0];
        body[3] = CRC8.checksum(&body[..3]);
        assert_eq!(Message::decode_body(&body), Err(DecodeError::Length(kind::SET_MODE)));
    }

    #[test]
    fn decode_body_rejects_too_short_input() {
        assert_eq!(Message::decode_body(&[]), Err(DecodeError::TooShort));
        assert_eq!(Message::decode_body(&[0x12]), Err(DecodeError::TooShort));
    }
}
```

- [ ] **Step 3: txbuf の失敗するテストを書く**

`minishirasu-firm/src/txbuf.rs`:

```rust
//! 送信ストリームごとのリングバッファ

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pops_bytes_in_order_in_chunks_of_eight() {
        let mut q = TxQueues::new();
        let message: Vec<u8> = (1..=20).collect();
        assert!(q.push(TxStream::Status, &message));

        let mut out = [0u8; 8];
        assert_eq!(q.pop_chunk(true, &mut out), Some((TxStream::Status, 8)));
        assert_eq!(out, [1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(q.pop_chunk(true, &mut out), Some((TxStream::Status, 8)));
        assert_eq!(out, [9, 10, 11, 12, 13, 14, 15, 16]);
        assert_eq!(q.pop_chunk(true, &mut out), Some((TxStream::Status, 4)));
        assert_eq!(out[..4], [17, 18, 19, 20]);
        assert_eq!(q.pop_chunk(true, &mut out), None);
    }

    #[test]
    fn preferred_stream_goes_first() {
        let mut q = TxQueues::new();
        assert!(q.push(TxStream::Response, &[0xA0]));
        assert!(q.push(TxStream::Status, &[0xB0]));

        let mut out = [0u8; 8];
        assert_eq!(q.pop_chunk(true, &mut out), Some((TxStream::Status, 1)));
        assert_eq!(q.pop_chunk(true, &mut out), Some((TxStream::Response, 1)));

        assert!(q.push(TxStream::Response, &[0xA0]));
        assert!(q.push(TxStream::Status, &[0xB0]));
        assert_eq!(q.pop_chunk(false, &mut out), Some((TxStream::Response, 1)));
        assert_eq!(q.pop_chunk(false, &mut out), Some((TxStream::Status, 1)));
    }

    #[test]
    fn message_that_does_not_fit_is_dropped_whole() {
        let mut q = TxQueues::new();
        assert!(q.push(TxStream::Status, &[1u8; 60]));
        // 残り4バイトに24バイトは入らない。途中まで入れたりしない
        assert!(!q.push(TxStream::Status, &[2u8; 24]));
        assert!(q.push(TxStream::Status, &[3u8; 4]));

        let mut out = [0u8; 8];
        let mut all = Vec::new();
        while let Some((_, n)) = q.pop_chunk(true, &mut out) {
            all.extend_from_slice(&out[..n]);
        }
        assert_eq!(all.len(), 64);
        assert!(all.iter().all(|&b| b != 2));
    }

    #[test]
    fn streams_do_not_share_capacity() {
        let mut q = TxQueues::new();
        assert!(q.push(TxStream::Status, &[1u8; 64]));
        assert!(q.push(TxStream::Response, &[2u8; 64]));
        assert!(!q.push(TxStream::Response, &[2u8; 1]));
    }

    #[test]
    fn buffer_wraps_around() {
        let mut q = TxQueues::new();
        let mut out = [0u8; 8];
        // 書き込み位置がバッファの終端をまたぐまで繰り返す
        for round in 0..20u8 {
            assert!(q.push(TxStream::Response, &[round; 7]));
            assert_eq!(q.pop_chunk(true, &mut out), Some((TxStream::Response, 7)));
            assert_eq!(out[..7], [round; 7]);
        }
    }
}
```

- [ ] **Step 4: 失敗を確認する**

Run: `cargo test -p minishirasu-firm --lib`
Expected: コンパイルエラー（`cannot find type StreamParser`、`cannot find type TxQueues` など）。

- [ ] **Step 5: protocol を実装する**

`minishirasu-firm/src/protocol.rs` の先頭のコメント2行の直後に足す:

```rust

use crc::{CRC_8_SMBUS, Crc};

const CRC8: Crc<u8> = Crc::<u8>::new(&CRC_8_SMBUS);

/// 符号化後の1メッセージの最大長(区切りを含む)。パーサのバッファ長でもある
pub const MAX_ENCODED: usize = 32;
/// 種別 + ペイロード + CRC の最大長。Statusの22バイト
const MAX_BODY: usize = 22;

/// メッセージの種別
pub mod kind {
    pub const TARGET_CURRENT: u8 = 0x01;
    pub const TARGET_VELOCITY: u8 = 0x02;
    pub const TARGET_POSITION: u8 = 0x03;
    pub const SET_MODE: u8 = 0x10;
    pub const SET_PARAM: u8 = 0x11;
    pub const RESET_FAULT: u8 = 0x12;
    pub const SET_ORIGIN: u8 = 0x13;
    pub const STATUS: u8 = 0x20;
    pub const ACK: u8 = 0x30;
    pub const NACK: u8 = 0x31;
    pub const FAULT_NOTICE: u8 = 0x32;
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Status {
    pub mode: u8,
    /// bit0: 上側に飽和、bit1: 下側に飽和、bit2: 母線電圧が低く更新できていない
    pub flags: u8,
    pub current: f32,  // [A]
    pub velocity: f32, // [rad/s]
    pub position: i32, // [回転] Q16.16
    pub vdc: f32,      // [V]
    pub temp: u16,     // ADCの生値
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Message {
    TargetCurrent { current: f32 },
    TargetVelocity { velocity: f32, accel_ff: f32 },
    /// positionは回転単位のQ16.16
    TargetPosition { position: i32, velocity_ff: f32, accel_ff: f32 },
    SetMode { mode: u8 },
    SetParam { id: u8, value: f32 },
    ResetFault,
    SetOrigin,
    Status(Status),
    Ack { command: u8 },
    Nack { command: u8, reason: u8, param: u8 },
    FaultNotice,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum DecodeError {
    /// COBSとして復号できない
    Cobs,
    /// 種別とCRCの2バイトに満たない
    TooShort,
    Crc,
    UnknownKind(u8),
    /// 種別に対して長さが合わない
    Length(u8),
    /// 区切りが来ないままバッファがあふれた
    Overflow,
}

/// ペイロードの長さ。未知の種別はNone
fn payload_len(kind: u8) -> Option<usize> {
    Some(match kind {
        kind::TARGET_CURRENT => 4,
        kind::TARGET_VELOCITY => 8,
        kind::TARGET_POSITION => 12,
        kind::SET_MODE => 1,
        kind::SET_PARAM => 5,
        kind::RESET_FAULT | kind::SET_ORIGIN | kind::FAULT_NOTICE => 0,
        kind::STATUS => 20,
        kind::ACK => 1,
        kind::NACK => 3,
        _ => return None,
    })
}

struct Writer {
    buf: [u8; MAX_BODY],
    len: usize,
}

impl Writer {
    fn bytes(&mut self, v: &[u8]) {
        self.buf[self.len..self.len + v.len()].copy_from_slice(v);
        self.len += v.len();
    }

    fn u8(&mut self, v: u8) {
        self.bytes(&[v]);
    }

    fn u16(&mut self, v: u16) {
        self.bytes(&v.to_le_bytes());
    }

    fn i32(&mut self, v: i32) {
        self.bytes(&v.to_le_bytes());
    }

    fn f32(&mut self, v: f32) {
        self.bytes(&v.to_le_bytes());
    }
}

/// 長さを確かめたあとのペイロードを順に読む
struct Reader<'a> {
    buf: &'a [u8],
}

impl Reader<'_> {
    fn take<const N: usize>(&mut self) -> [u8; N] {
        let (head, rest) = self.buf.split_at(N);
        self.buf = rest;
        let mut out = [0u8; N];
        out.copy_from_slice(head);
        out
    }

    fn u8(&mut self) -> u8 {
        self.take::<1>()[0]
    }

    fn u16(&mut self) -> u16 {
        u16::from_le_bytes(self.take())
    }

    fn i32(&mut self) -> i32 {
        i32::from_le_bytes(self.take())
    }

    fn f32(&mut self) -> f32 {
        f32::from_le_bytes(self.take())
    }
}

impl Message {
    pub fn kind(&self) -> u8 {
        match self {
            Message::TargetCurrent { .. } => kind::TARGET_CURRENT,
            Message::TargetVelocity { .. } => kind::TARGET_VELOCITY,
            Message::TargetPosition { .. } => kind::TARGET_POSITION,
            Message::SetMode { .. } => kind::SET_MODE,
            Message::SetParam { .. } => kind::SET_PARAM,
            Message::ResetFault => kind::RESET_FAULT,
            Message::SetOrigin => kind::SET_ORIGIN,
            Message::Status(_) => kind::STATUS,
            Message::Ack { .. } => kind::ACK,
            Message::Nack { .. } => kind::NACK,
            Message::FaultNotice => kind::FAULT_NOTICE,
        }
    }

    /// 区切りの0x00まで含めて書き、その長さを返す
    pub fn encode(&self, out: &mut [u8; MAX_ENCODED]) -> usize {
        let mut w = Writer { buf: [0; MAX_BODY], len: 0 };
        w.u8(self.kind());
        match *self {
            Message::TargetCurrent { current } => w.f32(current),
            Message::TargetVelocity { velocity, accel_ff } => {
                w.f32(velocity);
                w.f32(accel_ff);
            }
            Message::TargetPosition { position, velocity_ff, accel_ff } => {
                w.i32(position);
                w.f32(velocity_ff);
                w.f32(accel_ff);
            }
            Message::SetMode { mode } => w.u8(mode),
            Message::SetParam { id, value } => {
                w.u8(id);
                w.f32(value);
            }
            Message::ResetFault | Message::SetOrigin | Message::FaultNotice => {}
            Message::Status(s) => {
                w.u8(s.mode);
                w.u8(s.flags);
                w.f32(s.current);
                w.f32(s.velocity);
                w.i32(s.position);
                w.f32(s.vdc);
                w.u16(s.temp);
            }
            Message::Ack { command } => w.u8(command),
            Message::Nack { command, reason, param } => {
                w.u8(command);
                w.u8(reason);
                w.u8(param);
            }
        }
        let crc = CRC8.checksum(&w.buf[..w.len]);
        w.u8(crc);

        let n = cobs::encode(&w.buf[..w.len], &mut out[..]);
        out[n] = 0;
        n + 1
    }

    /// COBSを解いたあとの 種別 | ペイロード | CRC を読む
    pub fn decode_body(body: &[u8]) -> Result<Message, DecodeError> {
        if body.len() < 2 {
            return Err(DecodeError::TooShort);
        }
        let (content, crc) = body.split_at(body.len() - 1);
        if CRC8.checksum(content) != crc[0] {
            return Err(DecodeError::Crc);
        }
        let kind = content[0];
        let payload = &content[1..];
        let len = payload_len(kind).ok_or(DecodeError::UnknownKind(kind))?;
        if payload.len() != len {
            return Err(DecodeError::Length(kind));
        }

        let mut r = Reader { buf: payload };
        Ok(match kind {
            kind::TARGET_CURRENT => Message::TargetCurrent { current: r.f32() },
            kind::TARGET_VELOCITY => Message::TargetVelocity { velocity: r.f32(), accel_ff: r.f32() },
            kind::TARGET_POSITION => {
                Message::TargetPosition { position: r.i32(), velocity_ff: r.f32(), accel_ff: r.f32() }
            }
            kind::SET_MODE => Message::SetMode { mode: r.u8() },
            kind::SET_PARAM => Message::SetParam { id: r.u8(), value: r.f32() },
            kind::RESET_FAULT => Message::ResetFault,
            kind::SET_ORIGIN => Message::SetOrigin,
            kind::STATUS => Message::Status(Status {
                mode: r.u8(),
                flags: r.u8(),
                current: r.f32(),
                velocity: r.f32(),
                position: r.i32(),
                vdc: r.f32(),
                temp: r.u16(),
            }),
            kind::ACK => Message::Ack { command: r.u8() },
            kind::NACK => Message::Nack { command: r.u8(), reason: r.u8(), param: r.u8() },
            // payload_lenがSomeを返す種別はここまでで尽きている
            _ => Message::FaultNotice,
        })
    }
}

/// 区切りのないバイト列からメッセージを切り出す
pub struct StreamParser {
    buf: [u8; MAX_ENCODED],
    len: usize,
    overflow: bool,
}

impl StreamParser {
    pub const fn new() -> StreamParser {
        StreamParser { buf: [0; MAX_ENCODED], len: 0, overflow: false }
    }

    /// 1バイト進める。区切りを受けてメッセージが1つ確定したら、その結果を返す
    pub fn push(&mut self, byte: u8) -> Option<Result<Message, DecodeError>> {
        if byte != 0 {
            if self.len < MAX_ENCODED {
                self.buf[self.len] = byte;
                self.len += 1;
            } else {
                self.overflow = true;
            }
            return None;
        }

        let len = core::mem::replace(&mut self.len, 0);
        if core::mem::replace(&mut self.overflow, false) {
            return Some(Err(DecodeError::Overflow));
        }
        // 区切りが続いただけ
        if len == 0 {
            return None;
        }

        let mut body = [0u8; MAX_ENCODED];
        Some(match cobs::decode(&self.buf[..len], &mut body) {
            Ok(report) => Message::decode_body(&body[..report.frame_size()]),
            Err(_) => Err(DecodeError::Cobs),
        })
    }
}
```

- [ ] **Step 6: txbuf を実装する**

`minishirasu-firm/src/txbuf.rs` の先頭のコメント行の直後に足す:

```rust

/// 1ストリームぶんのバッファ長
const CAPACITY: usize = 64;

struct Ring {
    buf: [u8; CAPACITY],
    head: usize,
    len: usize,
}

impl Ring {
    const fn new() -> Ring {
        Ring { buf: [0; CAPACITY], head: 0, len: 0 }
    }

    /// 全部入るときだけ入れる
    fn push_all(&mut self, data: &[u8]) -> bool {
        if data.len() > CAPACITY - self.len {
            return false;
        }
        for &b in data {
            self.buf[(self.head + self.len) % CAPACITY] = b;
            self.len += 1;
        }
        true
    }

    fn pop(&mut self, out: &mut [u8]) -> usize {
        let n = out.len().min(self.len);
        for o in out[..n].iter_mut() {
            *o = self.buf[self.head];
            self.head = (self.head + 1) % CAPACITY;
            self.len -= 1;
        }
        n
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum TxStream {
    Status,
    Response,
}

pub struct TxQueues {
    status: Ring,
    response: Ring,
}

impl TxQueues {
    pub const fn new() -> TxQueues {
        TxQueues { status: Ring::new(), response: Ring::new() }
    }

    /// 符号化済みのメッセージを積む。入りきらなければ何も積まずfalseを返す
    /// (途中で切れたメッセージをストリームに流さないため)
    pub fn push(&mut self, stream: TxStream, message: &[u8]) -> bool {
        match stream {
            TxStream::Status => self.status.push_all(message),
            TxStream::Response => self.response.push_all(message),
        }
    }

    /// 次に送る最大8バイトを取り出す。両方に溜まっていれば、status_firstの側を先にする
    pub fn pop_chunk(&mut self, status_first: bool, out: &mut [u8; 8]) -> Option<(TxStream, usize)> {
        let order = if status_first {
            [TxStream::Status, TxStream::Response]
        } else {
            [TxStream::Response, TxStream::Status]
        };
        for stream in order {
            let ring = match stream {
                TxStream::Status => &mut self.status,
                TxStream::Response => &mut self.response,
            };
            let n = ring.pop(out);
            if n > 0 {
                return Some((stream, n));
            }
        }
        None
    }
}
```

- [ ] **Step 7: テストを通す**

Run: `cargo test -p minishirasu-firm --lib`
Expected: すべて PASS。

- [ ] **Step 8: コミット**

```powershell
git add minishirasu-firm/src/lib.rs minishirasu-firm/src/protocol.rs minishirasu-firm/src/txbuf.rs Cargo.lock
git commit -m "Add byte-stream protocol and transmit buffers"
```

---

### Task 6: config

**Files:**
- Create: `minishirasu-firm/src/config.rs`
- Modify: `minishirasu-firm/src/lib.rs`

**Interfaces:**
- Consumes: `md_core::controller::{Config, ConfigError, CurrentParam, VelocityParam, PositionParam}`、`md_core::encoder::EncoderParam`（Task 3）、`sense::SenseParam`（Task 4）、`Q16_16::from_ratio` / `scale_int`（Task 2）。
- Produces:
  - 定数: `PWM_ARR: u16 = 1800`、`CURRENT_PERIOD: f32 = 50e-6`、`OUTER_DIV: u32 = 20`、`OUTER_PERIOD: f32 = 1e-3`、`CAN_ID_TARGET: u16 = 0x100`、`CAN_ID_STATUS: u16 = 0x101`、`CAN_ID_COMMAND: u16 = 0x200`、`CAN_ID_RESPONSE: u16 = 0x201`
  - `config::id::{VDCMAX, IBASE, WBASE, KE, CKP, CKI, DEAD_DUTY, I_THRESHOLD, DUTY_MAX, VMAX, IMAX, WKP, WKI, WB, WMAX, PKP, PMAX, ACCEL_TO_CURRENT, ENCODER_CPR, W_FILTER_ALPHA, ENCODER_REVERSED, STATUS_PERIOD_MS}: u8`
  - `Settings::new()`（`const fn`）、`Settings::set(&mut self, id: u8, value: f32) -> Result<(), SetError>`、`Settings::get(&self, id: u8) -> Option<f32>`、`Settings::first_unset(&self) -> Option<u8>`、`Settings::status_period_ms(&self) -> u32`、`Settings::build(&self) -> Result<Params, BuildError>`
  - `SetError { UnknownId, NotFinite }`、`BuildError { Unset(u8), Invalid(u8) }`、`BuildError::id(self) -> u8`
  - `Params { pub inner: InnerParams, pub outer: OuterParams }`
  - `InnerParams { pub current: CurrentParam, pub sense: SenseParam }`
  - `OuterParams { pub velocity: VelocityParam, pub position: PositionParam, pub encoder: EncoderParam, pub sense: SenseParam, pub scale: Scale }`
  - `Scale`（`Clone, Copy`）: `current_pu(&self, a: f32) -> Option<Q3_28>`、`velocity_pu(&self, w: f32) -> Option<Q3_28>`、`accel_ff_pu(&self, alpha: f32) -> Option<Q3_28>`、`current_a(&self, i: Q3_28) -> f32`、`velocity_rad_s(&self, w: Q3_28) -> f32`、`voltage_v(&self, v: Q3_28) -> f32`
  - `position_from_wire(raw: i32) -> Q16_16`、`position_to_wire(th: Q16_16) -> i32`
  - テスト用: `config::testutil::full_settings() -> Settings`（`#[cfg(test)] pub(crate)`）

- [ ] **Step 1: lib.rs にモジュールを足す**

`minishirasu-firm/src/lib.rs`:

```rust
//! minishirasu-firm のうち、レジスタに触らない部分。ホストでテストする
#![cfg_attr(not(test), no_std)]

pub mod config;
pub mod protocol;
pub mod pwm;
pub mod sense;
pub mod txbuf;
```

- [ ] **Step 2: 失敗するテストを書く**

`minishirasu-firm/src/config.rs`:

```rust
//! 設定値。既定値は持たず、SetParamで1つずつ設定する

/// ほかのモジュールのテストからも使う、全項目を設定済みのSettings
#[cfg(test)]
pub(crate) mod testutil {
    use super::*;

    /// 基準値は V_b=24V, I_b=10A, ω_b=300rad/s。md-coreのテストと同じ値
    pub(crate) fn full_settings() -> Settings {
        let mut s = Settings::new();
        for (id, value) in [
            (id::VDCMAX, 24.0),
            (id::IBASE, 10.0),
            (id::WBASE, 300.0),
            (id::KE, 0.02),
            (id::CKP, 6.0),
            (id::CKI, 1000.0),
            (id::DEAD_DUTY, 0.02),
            (id::I_THRESHOLD, 0.5),
            (id::DUTY_MAX, 0.95),
            (id::VMAX, 20.0),
            (id::IMAX, 8.0),
            (id::WKP, 0.05),
            (id::WKI, 0.5),
            (id::WB, 0.8),
            (id::WMAX, 250.0),
            (id::PKP, 20.0),
            (id::PMAX, 100.0),
            (id::ACCEL_TO_CURRENT, 0.01),
            (id::ENCODER_CPR, 8192.0),
            (id::W_FILTER_ALPHA, 0.2),
            (id::ENCODER_REVERSED, 0.0),
        ] {
            s.set(id, value).unwrap();
        }
        s
    }
}

#[cfg(test)]
mod tests {
    use super::testutil::full_settings;
    use super::*;

    fn assert_close(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < 1e-5, "left: {actual}, right: {expected}");
    }

    #[test]
    fn control_periods_are_consistent() {
        assert_close(CURRENT_PERIOD * OUTER_DIV as f32, OUTER_PERIOD);
        // 72MHz、センターアラインで20kHz
        assert_eq!(PWM_ARR as u32 * 2 * 20_000, 72_000_000);
    }

    #[test]
    fn nothing_is_set_at_start() {
        let s = Settings::new();
        assert_eq!(s.get(id::VDCMAX), None);
        assert_eq!(s.first_unset(), Some(id::VDCMAX));
        assert_eq!(s.build().err(), Some(BuildError::Unset(id::VDCMAX)));
    }

    #[test]
    fn set_stores_value() {
        let mut s = Settings::new();
        assert_eq!(s.set(id::CKP, 6.0), Ok(()));
        assert_eq!(s.get(id::CKP), Some(6.0));
        assert_eq!(s.set(id::STATUS_PERIOD_MS, 10.0), Ok(()));
        assert_eq!(s.get(id::STATUS_PERIOD_MS), Some(10.0));
    }

    #[test]
    fn set_rejects_unknown_id() {
        let mut s = Settings::new();
        assert_eq!(s.set(0x11, 1.0), Err(SetError::UnknownId));
        assert_eq!(s.set(0x1F, 1.0), Err(SetError::UnknownId));
        assert_eq!(s.set(0x25, 1.0), Err(SetError::UnknownId));
        assert_eq!(s.set(0xFF, 1.0), Err(SetError::UnknownId));
        assert_eq!(s.get(0x11), None);
    }

    #[test]
    fn set_rejects_nan_and_infinity_without_storing() {
        let mut s = Settings::new();
        s.set(id::CKP, 6.0).unwrap();
        assert_eq!(s.set(id::CKP, f32::NAN), Err(SetError::NotFinite));
        assert_eq!(s.set(id::CKP, f32::INFINITY), Err(SetError::NotFinite));
        assert_eq!(s.set(id::CKP, f32::NEG_INFINITY), Err(SetError::NotFinite));
        assert_eq!(s.get(id::CKP), Some(6.0));
    }

    #[test]
    fn first_unset_reports_smallest_missing_id() {
        let mut s = Settings::new();
        for i in 0x00..=0x10u8 {
            s.set(i, 1.0).unwrap();
        }
        assert_eq!(s.first_unset(), Some(id::ACCEL_TO_CURRENT));
        s.set(id::ACCEL_TO_CURRENT, 0.0).unwrap();
        s.set(id::ENCODER_CPR, 8192.0).unwrap();
        s.set(id::ENCODER_REVERSED, 0.0).unwrap();
        assert_eq!(s.first_unset(), Some(id::W_FILTER_ALPHA));
    }

    #[test]
    fn status_period_is_not_required() {
        let s = full_settings();
        assert_eq!(s.get(id::STATUS_PERIOD_MS), None);
        assert_eq!(s.first_unset(), None);
        assert!(s.build().is_ok());
    }

    #[test]
    fn status_period_defaults_to_stopped() {
        let mut s = Settings::new();
        assert_eq!(s.status_period_ms(), 0);
        s.set(id::STATUS_PERIOD_MS, 10.0).unwrap();
        assert_eq!(s.status_period_ms(), 10);
        s.set(id::STATUS_PERIOD_MS, 2.9).unwrap();
        assert_eq!(s.status_period_ms(), 2);
        s.set(id::STATUS_PERIOD_MS, 0.5).unwrap();
        assert_eq!(s.status_period_ms(), 0);
        s.set(id::STATUS_PERIOD_MS, -5.0).unwrap();
        assert_eq!(s.status_period_ms(), 0);
    }

    #[test]
    fn build_reports_invalid_field() {
        let mut s = full_settings();
        s.set(id::CKP, 0.0).unwrap();
        assert_eq!(s.build().err(), Some(BuildError::Invalid(id::CKP)));

        // 1 * 300 / 10 = 30 はQ3.28に入らない
        let mut s = full_settings();
        s.set(id::WKP, 1.0).unwrap();
        assert_eq!(s.build().err(), Some(BuildError::Invalid(id::WKP)));

        let mut s = full_settings();
        s.set(id::PMAX, -1.0).unwrap();
        assert_eq!(s.build().err(), Some(BuildError::Invalid(id::PMAX)));

        let mut s = full_settings();
        s.set(id::IBASE, 0.0).unwrap();
        assert_eq!(s.build().err(), Some(BuildError::Invalid(id::IBASE)));
    }

    #[test]
    fn build_reports_invalid_encoder_settings() {
        let mut s = full_settings();
        s.set(id::ENCODER_CPR, 0.5).unwrap();
        assert_eq!(s.build().err(), Some(BuildError::Invalid(id::ENCODER_CPR)));

        let mut s = full_settings();
        s.set(id::ENCODER_CPR, -8192.0).unwrap();
        assert_eq!(s.build().err(), Some(BuildError::Invalid(id::ENCODER_CPR)));

        let mut s = full_settings();
        s.set(id::W_FILTER_ALPHA, 2.0).unwrap();
        assert_eq!(s.build().err(), Some(BuildError::Invalid(id::W_FILTER_ALPHA)));
    }

    #[test]
    fn build_error_exposes_id() {
        assert_eq!(BuildError::Unset(0x03).id(), 0x03);
        assert_eq!(BuildError::Invalid(0x22).id(), 0x22);
    }

    #[test]
    fn scale_converts_physical_units_to_per_unit() {
        let scale = full_settings().build().ok().unwrap().outer.scale;
        assert_close(scale.current_pu(5.0).unwrap().to_f32(), 0.5);
        assert_close(scale.velocity_pu(-150.0).unwrap().to_f32(), -0.5);
        // 100 rad/s² * 0.01 A/(rad/s²) = 1A = 0.1pu
        assert_close(scale.accel_ff_pu(100.0).unwrap().to_f32(), 0.1);
    }

    #[test]
    fn scale_rejects_values_out_of_range() {
        let scale = full_settings().build().ok().unwrap().outer.scale;
        assert_eq!(scale.current_pu(1000.0), None);
        assert_eq!(scale.velocity_pu(f32::NAN), None);
        assert_eq!(scale.accel_ff_pu(f32::INFINITY), None);
    }

    #[test]
    fn scale_converts_per_unit_back_to_physical_units() {
        let scale = full_settings().build().ok().unwrap().outer.scale;
        let half = Q3_28::checked_from_f32(0.5).unwrap();
        assert_close(scale.current_a(half), 5.0);
        assert_close(scale.velocity_rad_s(half), 150.0);
        assert_close(scale.voltage_v(half), 12.0);
    }

    #[test]
    fn position_wire_format_is_q16_16() {
        assert_close(position_from_wire(98304).to_f32(), 1.5);
        assert_eq!(position_to_wire(position_from_wire(98304)), 98304);
        assert_eq!(position_to_wire(position_from_wire(-1)), -1);
        assert_eq!(position_to_wire(position_from_wire(i32::MIN)), i32::MIN);
    }
}
```

- [ ] **Step 3: 失敗を確認する**

Run: `cargo test -p minishirasu-firm --lib config`
Expected: コンパイルエラー（`cannot find type Settings` など）。

- [ ] **Step 4: 実装する**

`minishirasu-firm/src/config.rs` の先頭のコメント行の直後に足す:

```rust

use md_core::controller::{Config, ConfigError, CurrentParam, PositionParam, VelocityParam};
use md_core::encoder::EncoderParam;
use md_core::fixed::{Q3_28, Q16_16};

use crate::sense::SenseParam;

/// TIM1の周期。72MHz、センターアラインで 72e6 / (2 * 1800) = 20kHz
pub const PWM_ARR: u16 = 1800;
/// 電流制御の周期[s]。PWMの1周期
pub const CURRENT_PERIOD: f32 = 50e-6;
/// 速度・位置制御は電流制御の何回に1回か
pub const OUTER_DIV: u32 = 20;
/// 速度・位置制御の周期[s]
pub const OUTER_PERIOD: f32 = 1e-3;

// ストリームごとのCAN ID(標準ID)。小さいほど優先度が高い。複数台つなぐときは基板ごとに変える
pub const CAN_ID_TARGET: u16 = 0x100;
pub const CAN_ID_STATUS: u16 = 0x101;
pub const CAN_ID_COMMAND: u16 = 0x200;
pub const CAN_ID_RESPONSE: u16 = 0x201;

/// SetParamの設定ID
pub mod id {
    pub const VDCMAX: u8 = 0x00;
    pub const IBASE: u8 = 0x01;
    pub const WBASE: u8 = 0x02;
    pub const KE: u8 = 0x03;
    pub const CKP: u8 = 0x04;
    pub const CKI: u8 = 0x05;
    pub const DEAD_DUTY: u8 = 0x06;
    pub const I_THRESHOLD: u8 = 0x07;
    pub const DUTY_MAX: u8 = 0x08;
    pub const VMAX: u8 = 0x09;
    pub const IMAX: u8 = 0x0A;
    pub const WKP: u8 = 0x0B;
    pub const WKI: u8 = 0x0C;
    pub const WB: u8 = 0x0D;
    pub const WMAX: u8 = 0x0E;
    pub const PKP: u8 = 0x0F;
    pub const PMAX: u8 = 0x10;
    pub const ACCEL_TO_CURRENT: u8 = 0x20;
    pub const ENCODER_CPR: u8 = 0x21;
    pub const W_FILTER_ALPHA: u8 = 0x22;
    pub const ENCODER_REVERSED: u8 = 0x23;
    pub const STATUS_PERIOD_MS: u8 = 0x24;
}

/// 設定値の総数
const COUNT: usize = 22;
/// 有効化に必要な設定値の数。0x00..=0x10 と 0x20..=0x23。STATUS_PERIOD_MSは含まない
const REQUIRED: usize = 21;
/// 0x20番台が始まる添字
const FIRM_BASE: usize = 0x11;

fn index(id: u8) -> Option<usize> {
    match id {
        0x00..=0x10 => Some(id as usize),
        0x20..=0x24 => Some(FIRM_BASE + (id - 0x20) as usize),
        _ => None,
    }
}

fn id_at(index: usize) -> u8 {
    if index < FIRM_BASE { index as u8 } else { 0x20 + (index - FIRM_BASE) as u8 }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum SetError {
    UnknownId,
    /// NaNか無限大
    NotFinite,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum BuildError {
    /// まだ設定されていない値がある。最も小さいID
    Unset(u8),
    /// 検証に通らない値がある。原因のID
    Invalid(u8),
}

impl BuildError {
    pub fn id(self) -> u8 {
        match self {
            BuildError::Unset(id) | BuildError::Invalid(id) => id,
        }
    }
}

/// 物理単位とper-unitの換算
#[derive(Clone, Copy)]
pub struct Scale {
    ibase: f32,
    wbase: f32,
    vdcmax: f32,
    accel_to_current: f32,
}

impl Scale {
    /// 電流[A] -> pu。範囲外やNaNはNone
    pub fn current_pu(&self, a: f32) -> Option<Q3_28> {
        Q3_28::checked_from_f32(a / self.ibase)
    }

    /// 速度[rad/s] -> pu
    pub fn velocity_pu(&self, w: f32) -> Option<Q3_28> {
        Q3_28::checked_from_f32(w / self.wbase)
    }

    /// 加速度FF[rad/s²] -> トルクFF[pu]。per-unitではトルクと電流が同じ値になる
    pub fn accel_ff_pu(&self, alpha: f32) -> Option<Q3_28> {
        Q3_28::checked_from_f32(alpha * self.accel_to_current / self.ibase)
    }

    pub fn current_a(&self, i: Q3_28) -> f32 {
        i.to_f32() * self.ibase
    }

    pub fn velocity_rad_s(&self, w: Q3_28) -> f32 {
        w.to_f32() * self.wbase
    }

    pub fn voltage_v(&self, v: Q3_28) -> f32 {
        v.to_f32() * self.vdcmax
    }
}

/// 通信上の位置(回転単位のQ16.16)を位置にする
pub fn position_from_wire(raw: i32) -> Q16_16 {
    Q16_16::from_ratio(raw, 65536)
}

/// 位置を通信上の表現(回転単位のQ16.16)にする
pub fn position_to_wire(th: Q16_16) -> i32 {
    th.scale_int(65536)
}

/// 電流制御の割り込みが使うもの
pub struct InnerParams {
    pub current: CurrentParam,
    pub sense: SenseParam,
}

/// 速度・位置制御と通信が使うもの
pub struct OuterParams {
    pub velocity: VelocityParam,
    pub position: PositionParam,
    pub encoder: EncoderParam,
    pub sense: SenseParam,
    pub scale: Scale,
}

pub struct Params {
    pub inner: InnerParams,
    pub outer: OuterParams,
}

/// ConfigErrorのフィールド名を設定IDにする
fn id_of(e: ConfigError) -> u8 {
    let name = match e {
        ConfigError::NotPositive(n) | ConfigError::Negative(n) | ConfigError::OutOfRange(n) => n,
    };
    match name {
        "vdcmax" => id::VDCMAX,
        "ibase" => id::IBASE,
        "wbase" => id::WBASE,
        "ke" => id::KE,
        "ckp" => id::CKP,
        // kb は cki と ckp から決まる
        "cki" | "kb" => id::CKI,
        "dead_duty" => id::DEAD_DUTY,
        "i_threshold" => id::I_THRESHOLD,
        "duty_max" => id::DUTY_MAX,
        "vmax" => id::VMAX,
        "imax" => id::IMAX,
        "wkp" => id::WKP,
        "wki" => id::WKI,
        "wb" => id::WB,
        "wmax" => id::WMAX,
        "pkp" => id::PKP,
        "pmax" => id::PMAX,
        "encoder_cpr" => id::ENCODER_CPR,
        "w_filter_alpha" => id::W_FILTER_ALPHA,
        // 周期はファームで固定しているので、ここには来ない
        _ => 0xFF,
    }
}

pub struct Settings {
    values: [Option<f32>; COUNT],
}

impl Settings {
    pub const fn new() -> Settings {
        Settings { values: [None; COUNT] }
    }

    pub fn set(&mut self, id: u8, value: f32) -> Result<(), SetError> {
        let i = index(id).ok_or(SetError::UnknownId)?;
        if !value.is_finite() {
            return Err(SetError::NotFinite);
        }
        self.values[i] = Some(value);
        Ok(())
    }

    pub fn get(&self, id: u8) -> Option<f32> {
        self.values[index(id)?]
    }

    /// 有効化に必要な設定値のうち、未設定で最も小さいID
    pub fn first_unset(&self) -> Option<u8> {
        (0..REQUIRED).find(|&i| self.values[i].is_none()).map(id_at)
    }

    /// 状態を送る周期[ms]。未設定や1未満なら0(送らない)
    pub fn status_period_ms(&self) -> u32 {
        match self.get(id::STATUS_PERIOD_MS) {
            Some(v) if v >= 1.0 => v as u32,
            _ => 0,
        }
    }

    /// 制御用のパラメータを作る。全項目が設定済みで、検証に通ったときだけ成功する
    pub fn build(&self) -> Result<Params, BuildError> {
        if let Some(id) = self.first_unset() {
            return Err(BuildError::Unset(id));
        }
        // first_unsetがNoneなので、必要な値はすべてSome
        let v = |id: u8| self.get(id).unwrap_or(0.0);
        let invalid = |e: ConfigError| BuildError::Invalid(id_of(e));

        let c = Config {
            vdcmax: v(id::VDCMAX),
            ibase: v(id::IBASE),
            wbase: v(id::WBASE),
            ke: v(id::KE),
            cperiod: CURRENT_PERIOD,
            ckp: v(id::CKP),
            cki: v(id::CKI),
            dead_duty: v(id::DEAD_DUTY),
            i_threshold: v(id::I_THRESHOLD),
            duty_max: v(id::DUTY_MAX),
            vmax: v(id::VMAX),
            imax: v(id::IMAX),
            wperiod: OUTER_PERIOD,
            wkp: v(id::WKP),
            wki: v(id::WKI),
            wb: v(id::WB),
            wmax: v(id::WMAX),
            pkp: v(id::PKP),
            pmax: v(id::PMAX),
        };

        let cpr = v(id::ENCODER_CPR);
        // 1未満(負やほぼ0)は、整数にすると0になるか意味を持たない
        if !(cpr >= 1.0) {
            return Err(BuildError::Invalid(id::ENCODER_CPR));
        }

        let sense = SenseParam::new(c.ibase, c.vdcmax).map_err(invalid)?;
        Ok(Params {
            inner: InnerParams { current: CurrentParam::new(&c).map_err(invalid)?, sense },
            outer: OuterParams {
                velocity: VelocityParam::new(&c).map_err(invalid)?,
                position: PositionParam::new(&c).map_err(invalid)?,
                encoder: EncoderParam::new(
                    cpr as u32,
                    v(id::ENCODER_REVERSED) != 0.0,
                    OUTER_PERIOD,
                    c.wbase,
                    v(id::W_FILTER_ALPHA),
                )
                .map_err(invalid)?,
                sense,
                scale: Scale {
                    ibase: c.ibase,
                    wbase: c.wbase,
                    vdcmax: c.vdcmax,
                    accel_to_current: v(id::ACCEL_TO_CURRENT),
                },
            },
        })
    }
}
```

- [ ] **Step 5: テストを通す**

Run: `cargo test -p minishirasu-firm --lib`
Expected: すべて PASS。

- [ ] **Step 6: コミット**

```powershell
git add minishirasu-firm/src/lib.rs minishirasu-firm/src/config.rs
git commit -m "Add settings without defaults and parameter building"
```

---

### Task 7: state

**Files:**
- Create: `minishirasu-firm/src/state.rs`
- Modify: `minishirasu-firm/src/lib.rs`

**Interfaces:**
- Consumes: `protocol::Message`（Task 5）。
- Produces:
  - `state::Mode { Disabled, Current, Velocity, Position, Fault }`（`Clone, Copy, PartialEq, Debug`）、`Mode::code(self) -> u8`（0〜4）、`Mode::is_active(self) -> bool`
  - `state::nack::{FRAME, ACTIVE, CONFIG, FAULT, VALUE, NFAULT}: u8`（1〜6）
  - `state::Reject { pub reason: u8, pub param: u8 }`（`Clone, Copy, PartialEq, Debug`）
  - `state::Action { Nothing, Disable, Enter(Mode), SetParam { id: u8, value: f32 }, ResetFault, SetOrigin }`（`Clone, Copy, PartialEq, Debug`）
  - `state::decide(mode: Mode, message: &Message, config_error: Option<u8>) -> Result<Action, Reject>`。`config_error` は、制御用のパラメータが無効なときの原因の設定 ID。有効なら `None`。

- [ ] **Step 1: lib.rs にモジュールを足す**

`minishirasu-firm/src/lib.rs` の `pub mod sense;` の直後に `pub mod state;` を足す。全体は次のとおり:

```rust
//! minishirasu-firm のうち、レジスタに触らない部分。ホストでテストする
#![cfg_attr(not(test), no_std)]

pub mod config;
pub mod protocol;
pub mod pwm;
pub mod sense;
pub mod state;
pub mod txbuf;
```

- [ ] **Step 2: 失敗するテストを書く**

`minishirasu-firm/src/state.rs`:

```rust
//! 状態機械。コマンドを受けたときに何をするかを決める(実行はしない)

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
```

- [ ] **Step 3: 失敗を確認する**

Run: `cargo test -p minishirasu-firm --lib state`
Expected: コンパイルエラー（`cannot find type Mode` など）。

- [ ] **Step 4: 実装する**

`minishirasu-firm/src/state.rs` の先頭のコメント行の直後に足す:

```rust

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
```

- [ ] **Step 5: テストを通す**

Run: `cargo test -p minishirasu-firm --lib`
Expected: すべて PASS。

- [ ] **Step 6: コミット**

```powershell
git add minishirasu-firm/src/lib.rs minishirasu-firm/src/state.rs
git commit -m "Add mode state machine"
```

---

### Task 8: cascade

**Files:**
- Create: `minishirasu-firm/src/cascade.rs`
- Modify: `minishirasu-firm/src/lib.rs`

**Interfaces:**
- Consumes: `md_core::controller::{Measurement, PositionState, Saturated, VelocityState}`、`config::{OuterParams, Scale, position_from_wire}`（Task 6）、`protocol::Message`（Task 5）、`state::Mode`（Task 7）。
- Produces:
  - `cascade::Target { Current(Q3_28), Velocity(Q3_28), Position(Q16_16) }`（`Clone, Copy, PartialEq, Debug`）
  - `cascade::Setpoint { pub target: Target, pub w_ff: Q3_28, pub i_ff: Q3_28 }`（`Clone, Copy, PartialEq, Debug`）
  - `Setpoint::hold(mode: Mode, th: Q16_16) -> Setpoint`: モードに入った直後の目標値（電流 0 / 速度 0 / 現在位置、FF は 0）
  - `cascade::setpoint_from_message(message: &Message, mode: Mode, scale: &Scale) -> Option<Setpoint>`
  - `Cascade::new() -> Cascade`、`Cascade::reset(&mut self)`、`Cascade::update(&mut self, p: &OuterParams, sp: &Setpoint, m: &Measurement, saturated: Saturated) -> Q3_28`（電流目標 [pu]）

- [ ] **Step 1: lib.rs にモジュールを足す**

`minishirasu-firm/src/lib.rs`:

```rust
//! minishirasu-firm のうち、レジスタに触らない部分。ホストでテストする
#![cfg_attr(not(test), no_std)]

pub mod cascade;
pub mod config;
pub mod protocol;
pub mod pwm;
pub mod sense;
pub mod state;
pub mod txbuf;
```

- [ ] **Step 2: 失敗するテストを書く**

`minishirasu-firm/src/cascade.rs`:

```rust
//! 目標値と、位置 -> 速度 -> 電流目標の合成

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::testutil::full_settings;

    fn q(v: f32) -> Q3_28 {
        Q3_28::checked_from_f32(v).unwrap()
    }

    fn q16(v: f32) -> Q16_16 {
        Q16_16::checked_from_f32(v).unwrap()
    }

    fn assert_close(actual: Q3_28, expected: f32) {
        let a = actual.to_f32();
        assert!((a - expected).abs() < 1e-5, "left: {a}, right: {expected}");
    }

    /// 速度PIは kp=1.5, ki=0.015, b=0.8。位置Pは kp=0.41887903
    fn outer() -> OuterParams {
        full_settings().build().ok().unwrap().outer
    }

    fn meas(w: f32, th: f32) -> Measurement {
        Measurement { vdc: q(0.5), vdc_inv: q(2.0), i: Q3_28::ZERO, w: q(w), th: q16(th) }
    }

    fn setpoint(target: Target, w_ff: f32, i_ff: f32) -> Setpoint {
        Setpoint { target, w_ff: q(w_ff), i_ff: q(i_ff) }
    }

    // ---- 目標値 ----

    #[test]
    fn hold_keeps_the_motor_where_it_is() {
        let th = q16(12.5);
        let zero = Q3_28::ZERO;
        assert_eq!(Setpoint::hold(Mode::Current, th), Setpoint { target: Target::Current(zero), w_ff: zero, i_ff: zero });
        assert_eq!(Setpoint::hold(Mode::Velocity, th), Setpoint { target: Target::Velocity(zero), w_ff: zero, i_ff: zero });
        assert_eq!(Setpoint::hold(Mode::Position, th), Setpoint { target: Target::Position(th), w_ff: zero, i_ff: zero });
    }

    #[test]
    fn target_messages_convert_to_per_unit() {
        let scale = outer().scale;

        let sp = setpoint_from_message(&Message::TargetCurrent { current: 5.0 }, Mode::Current, &scale).unwrap();
        assert_eq!(sp.target, Target::Current(q(0.5)));
        assert_eq!((sp.w_ff, sp.i_ff), (Q3_28::ZERO, Q3_28::ZERO));

        // 150 rad/s = 0.5pu。加速度200 rad/s² * 0.01 = 2A = 0.2pu
        let m = Message::TargetVelocity { velocity: 150.0, accel_ff: 200.0 };
        let sp = setpoint_from_message(&m, Mode::Velocity, &scale).unwrap();
        assert_eq!(sp.target, Target::Velocity(q(0.5)));
        assert_eq!(sp.w_ff, Q3_28::ZERO);
        assert_close(sp.i_ff, 0.2);

        // 98304 = 1.5回転。速度FF 30 rad/s = 0.1pu
        let m = Message::TargetPosition { position: 98304, velocity_ff: 30.0, accel_ff: -100.0 };
        let sp = setpoint_from_message(&m, Mode::Position, &scale).unwrap();
        assert_eq!(sp.target, Target::Position(q16(1.5)));
        assert_close(sp.w_ff, 0.1);
        assert_close(sp.i_ff, -0.1);
    }

    #[test]
    fn target_for_another_mode_is_ignored() {
        let scale = outer().scale;
        let current = Message::TargetCurrent { current: 1.0 };
        let velocity = Message::TargetVelocity { velocity: 1.0, accel_ff: 0.0 };
        let position = Message::TargetPosition { position: 0, velocity_ff: 0.0, accel_ff: 0.0 };

        assert_eq!(setpoint_from_message(&velocity, Mode::Current, &scale), None);
        assert_eq!(setpoint_from_message(&position, Mode::Velocity, &scale), None);
        assert_eq!(setpoint_from_message(&current, Mode::Position, &scale), None);
        // 出力無効中や異常中は、どの目標値も受け取らない
        for m in [current, velocity, position] {
            assert_eq!(setpoint_from_message(&m, Mode::Disabled, &scale), None);
            assert_eq!(setpoint_from_message(&m, Mode::Fault, &scale), None);
        }
    }

    #[test]
    fn target_out_of_range_is_ignored() {
        let scale = outer().scale;
        // 1000A = 100pu はQ3.28に入らない
        assert_eq!(setpoint_from_message(&Message::TargetCurrent { current: 1000.0 }, Mode::Current, &scale), None);
        assert_eq!(setpoint_from_message(&Message::TargetCurrent { current: f32::NAN }, Mode::Current, &scale), None);
        // FFだけが範囲外でも、目標値ごと捨てる
        let m = Message::TargetVelocity { velocity: 1.0, accel_ff: f32::INFINITY };
        assert_eq!(setpoint_from_message(&m, Mode::Velocity, &scale), None);
        let m = Message::TargetPosition { position: 0, velocity_ff: 1e9, accel_ff: 0.0 };
        assert_eq!(setpoint_from_message(&m, Mode::Position, &scale), None);
    }

    #[test]
    fn non_target_message_is_ignored() {
        let scale = outer().scale;
        assert_eq!(setpoint_from_message(&Message::SetMode { mode: 1 }, Mode::Current, &scale), None);
    }

    // ---- カスケード ----

    #[test]
    fn current_mode_passes_target_through() {
        let p = outer();
        let mut c = Cascade::new();
        let sp = setpoint(Target::Current(q(0.3)), 0.0, 0.0);
        assert_close(c.update(&p, &sp, &meas(0.25, 0.0), Saturated::NotSaturated), 0.3);
    }

    #[test]
    fn velocity_mode_adds_torque_feedforward() {
        let p = outer();
        let mut c = Cascade::new();
        let sp = setpoint(Target::Velocity(q(0.5)), 0.0, 0.1);
        // 1.5 * (0.8 * 0.5 - 0.25) + 0.1 = 0.325
        assert_close(c.update(&p, &sp, &meas(0.25, 0.0), Saturated::NotSaturated), 0.325);
    }

    #[test]
    fn velocity_mode_integrates_error() {
        let p = outer();
        let mut c = Cascade::new();
        let sp = setpoint(Target::Velocity(q(0.5)), 0.0, 0.0);
        assert_close(c.update(&p, &sp, &meas(0.25, 0.0), Saturated::NotSaturated), 0.225);
        // 積分が 0.015 * 0.25 = 0.00375 だけ溜まる
        assert_close(c.update(&p, &sp, &meas(0.25, 0.0), Saturated::NotSaturated), 0.22875);
    }

    #[test]
    fn saturation_freezes_velocity_integral() {
        let p = outer();
        let mut c = Cascade::new();
        let sp = setpoint(Target::Velocity(q(0.5)), 0.0, 0.0);
        // 電流ループが上側に飽和していて誤差が正なら、積分を進めない
        assert_close(c.update(&p, &sp, &meas(0.25, 0.0), Saturated::Overflow), 0.225);
        assert_close(c.update(&p, &sp, &meas(0.25, 0.0), Saturated::Overflow), 0.225);
    }

    #[test]
    fn position_mode_adds_velocity_and_torque_feedforward() {
        let p = outer();
        let mut c = Cascade::new();
        let sp = setpoint(Target::Position(q16(1.0)), 0.1, 0.05);
        // 速度目標 = 0.41887903 * (1.0 - 0.5) + 0.1 = 0.3094395
        // 電流目標 = 1.5 * (0.8 * 0.3094395 - 0) + 0.05 = 0.4213274
        assert_close(c.update(&p, &sp, &meas(0.0, 0.5), Saturated::NotSaturated), 0.4213274);
    }

    #[test]
    fn reset_clears_velocity_integral() {
        let p = outer();
        let mut c = Cascade::new();
        let sp = setpoint(Target::Velocity(q(0.5)), 0.0, 0.0);
        c.update(&p, &sp, &meas(0.25, 0.0), Saturated::NotSaturated);
        c.reset();
        assert_close(c.update(&p, &sp, &meas(0.25, 0.0), Saturated::NotSaturated), 0.225);
    }

    #[test]
    fn feedforward_sum_saturates() {
        let p = outer();
        let mut c = Cascade::new();
        let sp = Setpoint { target: Target::Current(Q3_28::ZERO), w_ff: Q3_28::ZERO, i_ff: Q3_28::MAX };
        // 電流モードではFFを使わない
        assert_close(c.update(&p, &sp, &meas(0.0, 0.0), Saturated::NotSaturated), 0.0);

        let sp = Setpoint { target: Target::Velocity(q(0.5)), w_ff: Q3_28::ZERO, i_ff: Q3_28::MAX };
        // 0.6 + ほぼ8 はラップせず上限に張り付く
        assert_eq!(c.update(&p, &sp, &meas(0.0, 0.0), Saturated::NotSaturated), Q3_28::MAX);
    }
}
```

- [ ] **Step 3: 失敗を確認する**

Run: `cargo test -p minishirasu-firm --lib cascade`
Expected: コンパイルエラー（`cannot find type Cascade` など）。

- [ ] **Step 4: 実装する**

`minishirasu-firm/src/cascade.rs` の先頭のコメント行の直後に足す:

```rust

use md_core::controller::{Measurement, PositionState, Saturated, VelocityState};
use md_core::fixed::{Q3_28, Q16_16};

use crate::config::{self, OuterParams, Scale};
use crate::protocol::Message;
use crate::state::Mode;

/// 目標値。どの量かが制御モードに対応する
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Target {
    Current(Q3_28),
    Velocity(Q3_28),
    Position(Q16_16),
}

/// すべてper-unit
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Setpoint {
    pub target: Target,
    /// 速度FF。位置モードで位置Pの出力に足す
    pub w_ff: Q3_28,
    /// トルクFF。速度・位置モードで速度PIの出力に足す
    pub i_ff: Q3_28,
}

impl Setpoint {
    /// モードに入った直後の目標値。電流0、速度0、または現在位置を保つ
    pub fn hold(mode: Mode, th: Q16_16) -> Setpoint {
        let target = match mode {
            Mode::Velocity => Target::Velocity(Q3_28::ZERO),
            Mode::Position => Target::Position(th),
            _ => Target::Current(Q3_28::ZERO),
        };
        Setpoint { target, w_ff: Q3_28::ZERO, i_ff: Q3_28::ZERO }
    }
}

/// 目標値のメッセージをper-unitにする。
/// 現在のモードと合わない、値が範囲外、目標値のメッセージでない、のどれかならNone
pub fn setpoint_from_message(message: &Message, mode: Mode, scale: &Scale) -> Option<Setpoint> {
    match (*message, mode) {
        (Message::TargetCurrent { current }, Mode::Current) => Some(Setpoint {
            target: Target::Current(scale.current_pu(current)?),
            w_ff: Q3_28::ZERO,
            i_ff: Q3_28::ZERO,
        }),
        (Message::TargetVelocity { velocity, accel_ff }, Mode::Velocity) => Some(Setpoint {
            target: Target::Velocity(scale.velocity_pu(velocity)?),
            w_ff: Q3_28::ZERO,
            i_ff: scale.accel_ff_pu(accel_ff)?,
        }),
        (Message::TargetPosition { position, velocity_ff, accel_ff }, Mode::Position) => Some(Setpoint {
            target: Target::Position(config::position_from_wire(position)),
            w_ff: scale.velocity_pu(velocity_ff)?,
            i_ff: scale.accel_ff_pu(accel_ff)?,
        }),
        _ => None,
    }
}

/// 位置P -> (+速度FF) -> 速度PI -> (+トルクFF) -> 電流目標
pub struct Cascade {
    velocity: VelocityState,
    position: PositionState,
}

impl Cascade {
    pub fn new() -> Cascade {
        Cascade { velocity: VelocityState::new(), position: PositionState::new() }
    }

    pub fn reset(&mut self) {
        self.velocity.reset();
    }

    /// 電流目標[pu]を返す。saturatedは電流ループが前回報告した飽和
    pub fn update(&mut self, p: &OuterParams, sp: &Setpoint, m: &Measurement, saturated: Saturated) -> Q3_28 {
        match sp.target {
            Target::Current(i) => i,
            Target::Velocity(w) => self.velocity.update(&p.velocity, w, m, saturated) + sp.i_ff,
            Target::Position(th) => {
                let w = self.position.update(&p.position, th, m) + sp.w_ff;
                self.velocity.update(&p.velocity, w, m, saturated) + sp.i_ff
            }
        }
    }
}
```

- [ ] **Step 5: テストを通す**

Run: `cargo test -p minishirasu-firm --lib`
Expected: すべて PASS。

- [ ] **Step 6: コミット**

```powershell
git add minishirasu-firm/src/lib.rs minishirasu-firm/src/cascade.rs
git commit -m "Add setpoint conversion and outer loop cascade"
```

---

### Task 9: board と can（バイナリ部）

レジスタ操作と bxcan。ホストではテストできないので、thumbv7m でビルドが通ることを確認する。Task 10 で `app` から使うまでは未使用の警告が出るが、それでよい。

**Files:**
- Create: `minishirasu-firm/src/board.rs`
- Create: `minishirasu-firm/src/can.rs`
- Modify: `minishirasu-firm/src/main.rs`（`mod` 宣言を足す）

**Interfaces:**
- Consumes: `config::{PWM_ARR, CAN_ID_*}`（Task 6）、`txbuf::{TxQueues, TxStream}`（Task 5）。
- Produces（`board`）:
  - `board::init(p: &pac::Peripherals)`: クロック、GPIO、ADC、TIM2、TIM1 を設定し、ドライバを起こす。EN は Low のまま
  - `board::check_update_phase() -> bool`: 更新イベントが山で出ていれば true
  - `board::AdcSample { pub isense: u16, pub vsense: u16, pub temp: u16 }`、`board::read_adc() -> AdcSample`
  - `board::clear_update_flag()`、`board::update_flag() -> bool`、`board::counting_down() -> bool`
  - `board::set_compare(a: u16, b: u16)`
  - `board::encoder_count() -> u16`
  - `board::output_enable()`、`board::output_disable()`、`board::nfault_asserted() -> bool`、`board::driver_restart()`
- Produces（`can`）:
  - `can::Can1`、`can::init() -> (bxcan::Tx<Can1>, bxcan::Rx0<Can1>)`
  - `can::RxStream { Target, Command }`、`can::classify(frame: &bxcan::Frame) -> Option<RxStream>`
  - `can::pump(tx: &mut bxcan::Tx<Can1>, queues: &mut TxQueues)`、`can::kick()`

- [ ] **Step 1: board.rs を書く**

`minishirasu-firm/src/board.rs`:

```rust
//! レジスタ操作。ピン配置は docs/superpowers/specs/2026-10-03-minishirasu-firm-design.md の「基板」

use minishirasu_firm::config::PWM_ARR;
use stm32f1::stm32f103 as pac;

const SYSCLK_HZ: u32 = 72_000_000;

/// trueなら、RCRをカウンタの起動前に書く。更新イベント(CCRの反映)が山(CNT=ARR)で出る。
/// 起動時の確認で「update event is not at the peak」と出たらfalseにして試す
const RCR_BEFORE_START: bool = true;

/// ADCトリガ用のチャネル4のコンペア値。OC4REFは CNT < CCR4 の間Highなので、
/// ダウンカウントで谷の手前 (CCR4 - 1) カウントの時点で立ち上がる。
/// ISENSEBのサンプル時間7.5サイクル(12MHzで0.625us = 45カウント)の半分だけ手前にして、
/// サンプル窓の中央を谷(ONパルスの中央)に合わせる
const ADC_TRIGGER_CCR: u16 = 23;

const CH_TEMP: u8 = 6; // PA6
const CH_VSENSE: u8 = 8; // PB0
const CH_ISENSEB: u8 = 9; // PB1

// SAFETY(以下のアクセサ共通): init完了後、これらのレジスタは次の決まりで触る。
// - GPIOA: BSRRへの書き込み(それ自体がアトミック)と、IDRの読み出しだけ
// - TIM1: CCR1/CCR2/SRはCurrentLoopだけが書く
// - ADC1: CurrentLoopだけが読む
// - TIM2: CNTを読むだけ
fn gpioa() -> &'static pac::gpioa::RegisterBlock {
    unsafe { &*pac::GPIOA::ptr() }
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

/// 全ペリフェラルを設定し、ドライバを起こす。出力(EN)は無効のまま
pub fn init(p: &pac::Peripherals) {
    clock_init(p);
    gpio_init(p);
    adc_init(p);
    tim2_init(p);
    tim1_init(p);

    // NSLEEPを上げてドライバを起こす。起動に1msかかる
    gpioa().bsrr.write(|w| w.bs2().set_bit());
    delay_ms(2);
}

/// HSE(12MHz) -> PLL x6 = 72MHz, APB1 = 36MHz, FLASH wait 2
fn clock_init(p: &pac::Peripherals) {
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
/// PA0/PA1(エンコーダ): 入力フローティング(リセット値のまま)
/// PA6(TEMP), PB0(VSENSE), PB1(ISENSEB): アナログ入力
/// PB8(CAN RX): 入力プルアップ, PB9(CAN TX): AF push-pull
fn gpio_init(p: &pac::Peripherals) {
    p.RCC.apb2enr.modify(|_, w| {
        w.iopaen().set_bit().iopben().set_bit().tim1en().set_bit().afioen().set_bit().adc1en().set_bit()
    });
    p.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit().canen().set_bit());

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
    // GPIOB CRH: PB8 = 0b1000(入力プル), PB9 = 0b1011(AF出力PP 50MHz)。PB8はプルアップ
    p.GPIOB.bsrr.write(|w| w.bs8().set_bit());
    p.GPIOB.crh.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !((0xF << 0) | (0xF << 4));
        v |= (0b1000 << 0) | (0b1011 << 4);
        w.bits(v)
    });

    // CANをPB8/PB9へ(CAN_REMAP = 10)。
    // SWJ_CFGは書き込み専用で読み値が不定なので、modifyでも毎回000(SWD/JTAG有効)を書く。
    // 書かないと読んだゴミが書き戻され、デバッガがつながらなくなることがある
    p.AFIO.mapr.modify(|_, w| unsafe { w.can_remap().bits(0b10).swj_cfg().bits(0b000) });
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
    // ISENSEB: 7.5cyc(0b001), VSENSE: 28.5cyc(0b011), TEMP: 55.5cyc(0b101)
    // 変換は合計 (7.5 + 28.5 + 55.5) + 3 * 12.5 = 129cyc = 10.75us
    adc.smpr2.modify(|_, w| w.smp6().bits(0b101).smp8().bits(0b011).smp9().bits(0b001));

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

/// コンペア値を書く。プリロードなので、次の更新イベント(山)で反映される
pub fn set_compare(a: u16, b: u16) {
    // PA9(PWMA)=CH2, PA8(PWMB)=CH1
    tim1().ccr2().write(|w| w.ccr().bits(a));
    tim1().ccr1().write(|w| w.ccr().bits(b));
}

pub fn encoder_count() -> u16 {
    tim2().cnt.read().bits() as u16
}

/// EN(!EMS)を上げる。High=有効
pub fn output_enable() {
    gpioa().bsrr.write(|w| w.bs4().set_bit());
}

/// ENを下げる。出力はハイインピーダンスになり、モーターはフリーになる
pub fn output_disable() {
    gpioa().bsrr.write(|w| w.br4().set_bit());
}

/// NFAULT(PA3)はLowで異常
pub fn nfault_asserted() -> bool {
    gpioa().idr.read().idr3().bit_is_clear()
}

/// ドライバの異常ラッチを解く。MPQ6528の異常はnSLEEPでしか解除できない。
/// 1ms Lowにしてから起こし、起動時間(1ms)より長く待つ。約3msブロックする
pub fn driver_restart() {
    gpioa().bsrr.write(|w| w.br2().set_bit());
    delay_ms(1);
    gpioa().bsrr.write(|w| w.bs2().set_bit());
    delay_ms(2);
}
```

- [ ] **Step 2: can.rs を書く**

`minishirasu-firm/src/can.rs`:

```rust
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
```

- [ ] **Step 3: main.rs に mod 宣言を足す**

`minishirasu-firm/src/main.rs` の `use panic_probe as _;` の行の直後に足す:

```rust

#[cfg(target_os = "none")]
mod board;
#[cfg(target_os = "none")]
mod can;
```

- [ ] **Step 4: thumbv7m でビルドできることを確かめる**

Run:

```powershell
Push-Location minishirasu-firm; cargo build --release; Pop-Location
```

Expected: `Finished`。未使用（`never used`）の警告は出てよい。

レジスタやフィールドの名前でコンパイルエラーになった場合は、PAC のソース（`~/.cargo/registry/src/index.crates.io-*/stm32f1-0.15.1/src/stm32f103/` 以下。例: `tim1/`、`adc1/`、`afio/mapr.rs`）か bxcan のソース（`~/.cargo/registry/src/index.crates.io-*/bxcan-0.8.0/src/lib.rs`）で正しい名前を確かめて直す。レジスタに書く値とビット位置は変えない。

- [ ] **Step 5: ホストのテストが壊れていないことを確かめる**

Run（ルートで）: `cargo test -p minishirasu-firm --lib`
Expected: すべて PASS。

- [ ] **Step 6: コミット**

```powershell
git add minishirasu-firm/src/main.rs minishirasu-firm/src/board.rs minishirasu-firm/src/can.rs Cargo.lock
git commit -m "Add board register access and CAN transport"
```

---

### Task 10: RTICX の app

Task 1 の骨組みを、実際のタスクに置き換える。RTICX の書き方を Task 1 で直した場合は、ここでも同じ書き方にする。

**Files:**
- Create: `minishirasu-firm/src/shared.rs`
- Modify: `minishirasu-firm/src/main.rs`（全体を置き換え）

**Interfaces:**
- Consumes: これまでの全タスク。
  - `board::*`、`can::*`（Task 9）
  - `cascade::{Cascade, Setpoint, setpoint_from_message}`（Task 8）
  - `config::{self, InnerParams, OuterParams, Settings, BuildError, SetError}`（Task 6）
  - `protocol::{Message, Status, StreamParser, MAX_ENCODED}`（Task 5）
  - `pwm::{duty_to_compare, compare_sign}`、`sense::count_delta`（Task 4）
  - `state::{self, Action, Mode, Reject, nack}`（Task 7）
  - `txbuf::{TxQueues, TxStream}`（Task 5）
  - `md_core::controller::{CurrentState, Measurement, Saturated}`、`md_core::encoder::EncoderState`（Task 3）
- Produces: 書き込める ELF（`target/thumbv7m-none-eabi/release/minishirasu-firm`）。

- [ ] **Step 1: shared.rs を書く**

`minishirasu-firm/src/shared.rs`:

```rust
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
pub struct OuterInput {
    pub encoder: u16,
    pub vsense: u16,
    pub temp: u16,
}

pub enum CommandInput {
    Message(Message),
    /// コマンドストリームで復号できないものを受けた
    BadFrame,
}

pub enum ReportKind {
    Status,
    Fault,
}
```

- [ ] **Step 2: main.rs を置き換える**

`minishirasu-firm/src/main.rs`:

```rust
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
                },
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
            // 半周期は25usなので、割り込みが14us以上遅れない限り判定を誤らない
            if board::counting_down() {
                self.timing_ok = false;
            }

            // 起動直後はオフセットを測る。出力は無効(EN=Low)なのでシャントの電流は0
            if self.offset_count < OFFSET_SAMPLES {
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

            if board::nfault_asserted() || !self.timing_ok {
                let entered = s.mode.lock(|m| {
                    let entered = *m != Mode::Fault;
                    *m = Mode::Fault;
                    entered
                });
                if entered {
                    board::output_disable();
                    s.link.lock(|l| l.enabled = false);
                    let _ = Report::spawn(ReportKind::Fault);
                    defmt::error!("fault: output disabled");
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
            if board::update_flag() {
                // 書く前に山を越えてしまった。次の周期に出るのは1つ前に書いた値
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
    }

    impl RticSwTask for OuterLoop {
        type SpawnInput = OuterInput;

        fn exec(&mut self, input: OuterInput) {
            let mut s = self.shared();

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
                Err(reject) => Message::Nack { command, reason: reject.reason, param: reject.param },
            };
            let mut buf = [0u8; MAX_ENCODED];
            let n = response.encode(&mut buf);
            // 入りきらなければ捨てる
            s.tx.lock(|q| q.push(TxStream::Response, &buf[..n]));
            can::kick();
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
```

- [ ] **Step 3: thumbv7m でビルドする**

Run:

```powershell
Push-Location minishirasu-firm; cargo build --release; Pop-Location
```

Expected: `Finished`、警告なし。リンクが通れば、Flash 64K と RAM 20K に収まっている（`memory.x` の領域を超えるとリンカがエラーにする）。

借用のエラー（`self.shared()` の戻り値と `self` のフィールドを同時に使う箇所、入れ子の `lock`）が出た場合は、フィールドを先にローカル変数へ取り出す形に直す。処理の順序と、`mode` をロックしたまま出力を有効にする構造は変えない。

- [ ] **Step 4: サイズを記録する**

Run:

```powershell
Push-Location minishirasu-firm; cargo build --release 2>&1 | Out-Null; Get-Item ..\target\thumbv7m-none-eabi\release\minishirasu-firm | Select-Object Length; Pop-Location
```

ELF のファイルサイズ（デバッグ情報込み）が出る。Flash の使用量を見たい場合で `cargo size` が入っていれば `cargo size --release -- -A` を使う。入っていなければ省略してよい（収まっていることはリンクで確認済み）。

- [ ] **Step 5: ホストのテストとビルドが壊れていないことを確かめる**

Run（ルートで）: `cargo test`
Expected: md-core と minishirasu-firm のテストがすべて PASS。

Run（ルートで）: `cargo test -p md-core --features f32`
Expected: すべて PASS。

- [ ] **Step 6: コミット**

```powershell
git add minishirasu-firm/src/main.rs minishirasu-firm/src/shared.rs
git commit -m "Wire control loops and communication with RTICX"
```

---

### Task 11: 文書と最終確認

**Files:**
- Modify: `minishirasu-firm/spec.md`（全体を置き換え）

**Interfaces:**
- Consumes: 実装済みのファーム。
- Produces: 使い方と実機確認の手順を書いた文書。

- [ ] **Step 1: spec.md を書き換える**

`minishirasu-firm/spec.md`:

````markdown
# minishirasu-firm
モタドラmini-shirasuのファームウェア。

board/下に基板の回路図等(sch, brd)がある。マイコンはSTM32F103C8T6。

設計の詳細は `docs/superpowers/specs/2026-10-03-minishirasu-firm-design.md` にある。

## 制御

CANで目標値を読み、モーターを動かす。

制御モードは電流、速度、位置の3つ。
カスケードしており、速度制御モードでは加速度FFを、位置制御モードでは速度FFと加速度FFを一緒に渡す。
加速度FFは外乱を含まない慣性分で、`accel_to_current`(= J/Kt)を掛けて電流に直し、速度PIの出力に足す。

電流制御は20kHz(PWMの毎周期)、速度・位置制御は1kHz。

## PWMとADC

TIM1のセンターアラインPWM。片側だけをPWMし、反対側はLow固定にする。

シャントはローサイド共通の1本なので、電流を測れるのは片側のハイサイドがONの間だけ。
ADCはONパルスの中央(カウンタの谷)で発火するようにする。
これはstm32f1xx-halのADCではできないので、レジスタ直打ちする。

- コンペア値の反映(更新イベント)はカウンタの山。`RCR = 1`をカウンタの起動前に書く。
- ADCのトリガはTIM1のチャネル4。`OC4REF`をTRGOに出し、谷の直前で立ち上げる。
- モーター電流の符号は、その周期に出していたデューティの向きから復元する。
- デューティが小さいとONパルスがサンプル時間より短くなり、電流を実際より小さく読む。

## 向き

正方向は、正のデューティ(PWMA側を駆動)で回る向き。逆回りを正にしたい場合は上位側で符号を反転する。

モーターとエンコーダの向きが食い違っていると、速度・位置ループが正帰還になって暴走する。
食い違いは設定`encoder_reversed`で吸収する。

## 通信

区切りのないバイトストリームを4本使う。CANではストリームごとにIDを1つ割り当て、
フレームのデータをストリームのバイト列としてそのまま連結する。IDは`src/config.rs`の定数。

| ストリーム | 向き | 既定のID | メッセージ |
|---|---|---|---|
| 目標値 | 受信 | 0x100 | TargetCurrent, TargetVelocity, TargetPosition |
| 状態 | 送信 | 0x101 | Status |
| コマンド | 受信 | 0x200 | SetMode, SetParam, ResetFault, SetOrigin |
| 応答 | 送信 | 0x201 | Ack, Nack, FaultNotice |

メッセージは `COBS( 種別 | ペイロード | CRC-8/SMBUS )` のあとに区切りの0x00。数値はリトルエンディアン。
種別、ペイロード、設定ID、Nackの理由は設計文書の「プロトコル」を見ること。

## 使い方

設定値に既定値はない。起動したら、まず`SetParam`で全項目(0x00〜0x23)を設定する。
全項目が揃って検証に通るまで、有効化(`SetMode`)は受け付けず、`Status`も送られない。
`Status`を受け取るには`status_period_ms`(0x24)も設定する。

1. `SetParam`で全項目を設定する。
2. `SetMode`で制御モードに入る。入った直後の目標値は、電流0 / 速度0 / 現在位置。
3. 目標値を送る。現在のモードと合わない目標値は捨てられる。
4. `SetMode`(0)で出力を止める。出力を止めるとモーターはフリーになる(ブレーキは掛からない)。

ドライバIC(MPQ6528)が異常を検出すると、出力を止めて`FaultNotice`を送り、`ResetFault`が来るまで復帰しない。
通信が途絶えても出力は止めない。ソフト側の過電流・電圧・温度の保護もない。

## 実機での確認手順

モーターを回す前に、上から順に確かめる。

1. 書き込んで起動ログを見る。`isense offset=...`が出て、エラーが出ないこと。
   - `update event is not at the peak`が出たら、`src/board.rs`の`RCR_BEFORE_START`を反対にして書き込み直す。
   - `ADC trigger or update event timing is wrong`が出たら、ADCが谷でトリガされていない。
2. オシロでISENSEのテストパッドとPWMAを見て、ADCのサンプル時刻が電流波形の平らな部分に入っていることを確かめる。
3. `SetParam`で全項目を設定する。電流上限`imax`とゲインは小さい値から始める。
4. `Status`が届くこと。母線電圧が実測と合うこと。
5. 手でモーターを回し、位置と速度の大きさが合うことを見る(`encoder_cpr`の確認)。
6. 電流モードで小さい正の電流を流し、`Status`の速度が正になることを見る。負なら`encoder_reversed`を切り替える。
7. 速度モード、位置モードを順に試す。
8. ログに`current loop missed the peak`が出ないことを見る。出る場合は電流ループの計算が25µsの半周期に間に合っていない。
````

- [ ] **Step 2: 全体のテストとビルドをもう一度流す**

Run（ルートで）: `cargo test`
Expected: すべて PASS。

Run（ルートで）: `cargo test -p md-core --features f32`
Expected: すべて PASS。

Run:

```powershell
Push-Location minishirasu-firm; cargo build --release; Pop-Location
```

Expected: `Finished`、警告なし。

- [ ] **Step 3: コミット**

```powershell
git add minishirasu-firm/spec.md
git commit -m "Document minishirasu-firm usage and bring-up steps"
```

---

## Self-Review の結果

**Spec との対応**

| spec の項目 | タスク |
|---|---|
| md-core への追加（encoder、境界の演算） | 2、3 |
| ライブラリ部とバイナリ部の分離、依存の整理 | 1 |
| タスク構成、共有リソース、データの流れ | 10 |
| 状態機械（遷移、NFAULT、ResetFault） | 7（判定）、10（実行） |
| カスケードと FF | 8 |
| PWM（片側 PWM、山で更新） | 4（換算）、9（レジスタ） |
| ADC トリガ（OC4REF）、タイミングの自己確認 | 9、10 |
| 電流の復元（オフセット、符号の追跡） | 4、10 |
| 母線電圧、温度 | 4、10 |
| エンコーダ（TIM2、差分、向き、原点） | 3、4、9、10 |
| プロトコル（フレーミング、メッセージ） | 5 |
| 目標値とコマンドの扱い | 7、8、10 |
| 設定値（既定値なし、検証のタイミング） | 6、10 |
| CAN（ID、フィルタ、送信バッファ） | 5、9 |
| 文書の更新 | 2、11 |

**spec から変えた点**

- 共有リソースの `params` を、電流ループ用の `inner` と外側ループ・通信用の `outer` に分けた。1つにまとめると、外側ループがパラメータを使っている間、電流ループの割り込みが待たされるため。
- 共有リソースの `setpoint` は、原点設定の要求と状態送信の周期も運ぶので `outer_cmd` という名前にした。
- モード切替時の積分器リセットは、`link.epoch`（世代番号）1つで電流ループと外側ループの両方に伝える。
- `CurrentLoop` から `FaultNotice` を送る方法は、送信バッファへのフラグではなく `Report` タスクの起動にした。
- パラメータが揃う前は、エンコーダの差分を位置に積算しない（per-unit にする換算係数がないため）。設定後に `SetOrigin` を送れば原点は取り直せる。

**実装者が確かめる必要がある箇所**

- RTICX マクロの書き方（Task 1 の骨組みで先に確かめる）。
- PAC のレジスタ・フィールド名（Task 9 のビルドで確かめる）。
- ADC が TRGO の立ち上がりで掛かること、更新イベントが山で出ることは、実機でしか確かめられない。起動時の自己確認と、`RCR_BEFORE_START` の切り替えで対応する。
