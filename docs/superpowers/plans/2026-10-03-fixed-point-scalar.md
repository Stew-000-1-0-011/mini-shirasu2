# md-core 固定小数点化 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `md-core` の制御計算を per-unit の固定小数点（Q3.28 / Q3.60 / Q16.16）で行えるようにし、feature `f32` で浮動小数点版に切り替えられるようにする。

**Architecture:** 単一の `Scalar` をやめ、`md-core/src/fixed.rs` に 3 つの数値型を置く。型の中身は `cfg(feature = "f32")` で整数実装（`fixed/int.rs`）と浮動小数点実装（`fixed/float.rs`）を切り替え、公開 API と飽和の振る舞いは両方で同じにする。`Config` は物理単位の f32 で持ち、`Param::new(&Config)` が per-unit 化して固定小数点へ変換する。値の意味に依存する処理（積分器、デッドタイム補償、母線電圧の逆数）は `controller.rs` に置く。

**Tech Stack:** Rust 2024 edition、`#![no_std]`、外部クレートなし。テストは `cargo test`（ホスト上）。

**Spec:** `docs/superpowers/specs/2026-10-03-fixed-point-scalar-design.md`

## Global Constraints

- `md-core` に外部依存クレートを追加しない。`#![no_std]` を保つ。
- **`docs/制御.md` は絶対に編集しない**（読むのは可）。ドキュメントの追加は `docs/` 以下の新規ファイルで行う。
- インデントはタブ。コメントは日本語。既存コードの書き方に合わせる。
- 公開 API は固定小数点版と f32 版で同一。f32 版も演算結果を固定小数点版と同じ範囲に飽和させる。
- 型をまたぐ暗黙の `From` は作らない。変換は明示的な関数のみ。
- Q 型には、フォーマットだけで振る舞いが決まる汎用演算だけを置く。`recip` や `sat(threshold_inv)` のような意味依存・容易に飽和する演算は置かない（除算は `checked_div` と `unchecked_div` のみ）。
- 各タスクの最後で `cargo test -p md-core` が通ること。Task 2 以降は `cargo test -p md-core --features f32` も通ること。
- コマンドはすべてリポジトリのルート（`mini-shirasu2/`）で実行する。シェルは PowerShell。
- `minisirasu-firm` は `md_core::pwm` が無いため現時点で既にビルドできない。これは本計画の範囲外で、直さない。ターゲット向けの確認は `md-core` 単体のビルドで行う。

## Review Focus

仕様が暗に求めているが、素直に書いたテストでは踏まない入力。各行のテストは、担当タスクのテストコードに含めてある。

1. **母線電圧が下限ちょうど**：`vdcmin == vdcmax/8` だと逆数がちょうど 8 になり Q3.28 に収まらない。`CurrentParam::new` はこれをエラーにし、`vdc == vdc_min` での `update_vdc` は範囲内の `vdc_inv` を返すこと（Task 4）。
2. **デッドタイムしきい値が丸めで 0 になる**：`i_threshold` が極端に小さいと pu 変換後に 0 になる。`soft_sign` は 0 除算せず ±1 を返すこと（Task 3）。
3. **Config に NaN や無限大が入る**：パニックせず `ConfigError` を返すこと（Task 4）。
4. **位置誤差が 8 回転を超える**：ゲインを掛ける前に飽和して速度指令が弱まらないこと（Task 3）。
5. **積分器が上限まで溜まる**：ラップして符号が反転せず、上限で止まること（Task 3）。

## File Structure

| ファイル | 役割 |
|---|---|
| `md-core/Cargo.toml` | feature `f32` を追加 |
| `md-core/src/lib.rs` | `pub mod fixed;` を追加し、最後に `pub mod scalar;` を削除 |
| `md-core/src/fixed.rs`（新規） | backend の切り替えと `pub use`、両 backend 共通のテスト |
| `md-core/src/fixed/int.rs`（新規） | 固定小数点（i32 / i64）実装 |
| `md-core/src/fixed/float.rs`（新規） | 浮動小数点（f32 / f64）実装 |
| `md-core/src/controller.rs` | 新しい型への移行、`Integrator`、`soft_sign`、`Config`、`ConfigError`、各 `Param::new`、`Measurement::update_vdc` |
| `md-core/src/scalar.rs` | 最後に削除 |
| `docs/数値表現.md`（新規） | per-unit 基準値と型の使い分けの説明 |

---

### Task 1: 固定小数点型（整数 backend）

**Files:**
- Modify: `md-core/Cargo.toml`
- Modify: `md-core/src/lib.rs`
- Create: `md-core/src/fixed.rs`
- Create: `md-core/src/fixed/int.rs`

**Interfaces:**
- Consumes: なし
- Produces（`md_core::fixed`）:
  - `Q3_28`、`Q3_60`、`Q16_16`：いずれも `Clone + Copy + PartialEq + PartialOrd + Debug`
  - 全型共通：`ZERO`、`MAX`、`MIN`、`checked_from_f32(f32) -> Option<Self>`、`to_f32(self) -> f32`、`min`、`max`、`abs`、`+`、`-`、単項 `-`（すべて飽和）
  - `Q3_28`：`ONE`、`*`（飽和）、`widening_mul(self, Q3_28) -> Q3_60`、`checked_div(self, Q3_28) -> Option<Q3_28>`、`unchecked_div(self, Q3_28) -> Q3_28`、`to_q3_60(self) -> Q3_60`、`to_q16_16(self) -> Q16_16`
  - `Q3_60`：`to_q3_28(self) -> Q3_28`
  - `Q16_16`：`ONE`、`mul_q3_28(self, Q3_28) -> Q3_28`（飽和）、`to_q3_28(self) -> Q3_28`（飽和）

- [ ] **Step 1: feature とモジュールを宣言する**

`md-core/Cargo.toml` の末尾に追加:

```toml

[features]
# 数値型の中身を浮動小数点にする(既定は固定小数点)
f32 = []
```

`md-core/src/lib.rs` を次の内容にする（`scalar` は Task 5 まで残す）:

```rust
#![no_std]

pub mod controller;
pub mod fixed;
pub mod scalar;
```

- [ ] **Step 2: 失敗するテストを書く**

`md-core/src/fixed.rs` を新規作成:

```rust
//! 制御計算用の数値型。
//!
//! - `Q3_28`: per-unitの信号とゲイン。範囲は[-8, 8)
//! - `Q3_60`: 積分器。範囲は[-8, 8)で、Q3_28同士の積を丸めずに持てる
//! - `Q16_16`: 位置。1回転 = 1.0、範囲は[-32768, 32768)
//!
//! 既定は固定小数点で、feature `f32` を有効にすると中身が浮動小数点になる。
//! どちらでもAPIと飽和の振る舞いは同じ。
#![allow(non_camel_case_types)]

#[cfg(not(feature = "f32"))]
mod int;
#[cfg(not(feature = "f32"))]
pub use int::{Q3_28, Q3_60, Q16_16};

#[cfg(feature = "f32")]
mod float;
#[cfg(feature = "f32")]
pub use float::{Q3_28, Q3_60, Q16_16};

#[cfg(test)]
mod tests {
	use super::*;

	fn q(v: f32) -> Q3_28 {
		Q3_28::checked_from_f32(v).unwrap()
	}

	fn q60(v: f32) -> Q3_60 {
		Q3_60::checked_from_f32(v).unwrap()
	}

	fn q16(v: f32) -> Q16_16 {
		Q16_16::checked_from_f32(v).unwrap()
	}

	fn assert_close(actual: f32, expected: f32) {
		assert!((actual - expected).abs() < 1e-6, "left: {actual}, right: {expected}");
	}

	// ---- Q3_28 ----

	#[test]
	fn q3_28_roundtrips_f32() {
		assert_close(q(1.5).to_f32(), 1.5);
		assert_close(q(-0.375).to_f32(), -0.375);
		assert_close(Q3_28::ZERO.to_f32(), 0.0);
		assert_close(Q3_28::ONE.to_f32(), 1.0);
	}

	#[test]
	fn q3_28_from_f32_rejects_out_of_range() {
		assert!(Q3_28::checked_from_f32(8.0).is_none());
		assert!(Q3_28::checked_from_f32(-8.5).is_none());
		assert!(Q3_28::checked_from_f32(f32::NAN).is_none());
		assert!(Q3_28::checked_from_f32(f32::INFINITY).is_none());
		// 下限ちょうどは表せる
		assert_eq!(Q3_28::checked_from_f32(-8.0), Some(Q3_28::MIN));
	}

	#[test]
	fn q3_28_add_sub() {
		assert_close((q(1.5) + q(2.25)).to_f32(), 3.75);
		assert_close((q(1.5) - q(2.25)).to_f32(), -0.75);
		assert_close((-q(1.5)).to_f32(), -1.5);
	}

	#[test]
	fn q3_28_add_sub_neg_saturate() {
		assert_eq!(Q3_28::MAX + Q3_28::ONE, Q3_28::MAX);
		assert_eq!(Q3_28::MIN - Q3_28::ONE, Q3_28::MIN);
		assert_eq!(q(5.0) + q(5.0), Q3_28::MAX);
		assert_eq!(q(-5.0) - q(5.0), Q3_28::MIN);
		assert_eq!(-Q3_28::MIN, Q3_28::MAX);
	}

	#[test]
	fn q3_28_mul() {
		assert_close((q(1.5) * q(2.0)).to_f32(), 3.0);
		assert_close((q(-0.5) * q(0.25)).to_f32(), -0.125);
	}

	#[test]
	fn q3_28_mul_saturates() {
		assert_eq!(q(4.0) * q(4.0), Q3_28::MAX);
		assert_eq!(q(-4.0) * q(4.0), Q3_28::MIN);
	}

	#[test]
	fn q3_28_min_max_abs() {
		assert_eq!(q(1.0).min(q(2.0)), q(1.0));
		assert_eq!(q(1.0).max(q(2.0)), q(2.0));
		assert_eq!(q(-1.5).abs(), q(1.5));
		assert_eq!(q(1.5).abs(), q(1.5));
		assert_eq!(Q3_28::MIN.abs(), Q3_28::MAX);
	}

	#[test]
	fn q3_28_compares() {
		assert!(q(1.0) < q(2.0));
		assert!(q(-1.0) < Q3_28::ZERO);
		assert!(Q3_28::MIN < Q3_28::MAX);
	}

	#[test]
	fn q3_28_checked_div() {
		assert_close(q(1.0).checked_div(q(4.0)).unwrap().to_f32(), 0.25);
		assert_close(q(-3.0).checked_div(q(4.0)).unwrap().to_f32(), -0.75);
	}

	#[test]
	fn q3_28_checked_div_rejects_zero_and_overflow() {
		assert!(q(1.0).checked_div(Q3_28::ZERO).is_none());
		// 4 / 0.25 = 16 は範囲外
		assert!(q(4.0).checked_div(q(0.25)).is_none());
		// 1 / 0.125 = 8 もちょうど範囲外
		assert!(q(1.0).checked_div(q(0.125)).is_none());
	}

	#[test]
	fn q3_28_unchecked_div() {
		assert_close(q(3.0).unchecked_div(q(4.0)).to_f32(), 0.75);
	}

	#[test]
	#[cfg(debug_assertions)]
	#[should_panic]
	fn q3_28_unchecked_div_by_zero_panics_in_debug() {
		let _ = q(1.0).unchecked_div(Q3_28::ZERO);
	}

	#[test]
	#[cfg(debug_assertions)]
	#[should_panic]
	fn q3_28_unchecked_div_overflow_panics_in_debug() {
		let _ = q(4.0).unchecked_div(q(0.25));
	}

	#[test]
	fn widening_mul_keeps_product_below_q3_28_resolution() {
		// 1e-4 * 1e-5 = 1e-9 は Q3.28 の分解能(約3.7e-9)より小さい
		let p = q(1e-4).widening_mul(q(1e-5));
		assert!((p.to_f32() - 1e-9).abs() < 1e-11, "left: {}", p.to_f32());
	}

	#[test]
	#[cfg(not(feature = "f32"))]
	fn q3_28_mul_drops_product_below_resolution() {
		// 拡大乗算が必要な理由: 通常の乗算ではこの積が0になる
		assert_eq!(q(1e-4) * q(1e-5), Q3_28::ZERO);
	}

	#[test]
	fn widening_mul_saturates() {
		assert_eq!(q(4.0).widening_mul(q(4.0)), Q3_60::MAX);
		assert_eq!(q(-4.0).widening_mul(q(4.0)), Q3_60::MIN);
	}

	#[test]
	fn q3_28_converts_to_other_formats() {
		assert_close(q(1.25).to_q3_60().to_f32(), 1.25);
		assert_close(q(-1.5).to_q16_16().to_f32(), -1.5);
	}

	// ---- Q3_60 ----

	#[test]
	fn q3_60_roundtrips_f32() {
		assert_close(q60(1.25).to_f32(), 1.25);
		assert_close(Q3_60::ZERO.to_f32(), 0.0);
		assert!(Q3_60::checked_from_f32(8.0).is_none());
		assert!(Q3_60::checked_from_f32(f32::NAN).is_none());
	}

	#[test]
	fn q3_60_add_sub_neg() {
		assert_close((q60(1.5) + q60(2.25)).to_f32(), 3.75);
		assert_close((q60(1.5) - q60(2.25)).to_f32(), -0.75);
		assert_close((-q60(1.5)).to_f32(), -1.5);
	}

	#[test]
	fn q3_60_saturates() {
		assert_eq!(Q3_60::MAX + Q3_60::MAX, Q3_60::MAX);
		assert_eq!(Q3_60::MIN - Q3_60::MAX, Q3_60::MIN);
		assert_eq!(-Q3_60::MIN, Q3_60::MAX);
		assert_eq!(Q3_60::MIN.abs(), Q3_60::MAX);
	}

	#[test]
	fn q3_60_min_max() {
		assert_eq!(q60(1.0).min(q60(2.0)), q60(1.0));
		assert_eq!(q60(1.0).max(q60(2.0)), q60(2.0));
	}

	#[test]
	fn q3_60_converts_to_q3_28() {
		assert_close(q60(-2.75).to_q3_28().to_f32(), -2.75);
		assert_eq!(Q3_60::MAX.to_q3_28(), Q3_28::MAX);
		assert_eq!(Q3_60::MIN.to_q3_28(), Q3_28::MIN);
	}

	// ---- Q16_16 ----

	#[test]
	fn q16_16_roundtrips_f32() {
		assert_close(q16(1000.5).to_f32(), 1000.5);
		assert_close(q16(-0.25).to_f32(), -0.25);
		assert_close(Q16_16::ONE.to_f32(), 1.0);
	}

	#[test]
	fn q16_16_from_f32_rejects_out_of_range() {
		assert!(Q16_16::checked_from_f32(40000.0).is_none());
		assert!(Q16_16::checked_from_f32(32768.0).is_none());
		assert!(Q16_16::checked_from_f32(f32::NAN).is_none());
		assert_eq!(Q16_16::checked_from_f32(-32768.0), Some(Q16_16::MIN));
	}

	#[test]
	fn q16_16_add_sub_neg() {
		assert_close((q16(100.5) + q16(2.25)).to_f32(), 102.75);
		assert_close((q16(100.5) - q16(200.0)).to_f32(), -99.5);
		assert_close((-q16(3.5)).to_f32(), -3.5);
	}

	#[test]
	fn q16_16_saturates() {
		assert_eq!(Q16_16::MAX + Q16_16::ONE, Q16_16::MAX);
		assert_eq!(Q16_16::MIN - Q16_16::ONE, Q16_16::MIN);
		assert_eq!(-Q16_16::MIN, Q16_16::MAX);
		assert_eq!(Q16_16::MIN.abs(), Q16_16::MAX);
	}

	#[test]
	fn q16_16_min_max() {
		assert_eq!(q16(1.0).min(q16(2.0)), q16(1.0));
		assert_eq!(q16(1.0).max(q16(2.0)), q16(2.0));
	}

	#[test]
	fn q16_16_mul_q3_28() {
		// 100回転 * 0.0625 = 6.25。先にQ3_28へ変換すると100は入らない
		assert_close(q16(100.0).mul_q3_28(q(0.0625)).to_f32(), 6.25);
		assert_close(q16(-2.0).mul_q3_28(q(0.5)).to_f32(), -1.0);
	}

	#[test]
	fn q16_16_mul_q3_28_saturates() {
		assert_eq!(q16(1000.0).mul_q3_28(q(1.0)), Q3_28::MAX);
		assert_eq!(q16(-1000.0).mul_q3_28(q(1.0)), Q3_28::MIN);
	}

	#[test]
	fn q16_16_converts_to_q3_28() {
		assert_close(q16(1.5).to_q3_28().to_f32(), 1.5);
		assert_eq!(q16(100.0).to_q3_28(), Q3_28::MAX);
		assert_eq!(q16(-100.0).to_q3_28(), Q3_28::MIN);
	}
}
```

- [ ] **Step 3: テストが失敗することを確認する**

Run: `cargo test -p md-core`
Expected: コンパイルエラー `file not found for module `int``

- [ ] **Step 4: 整数 backend を実装する**

`md-core/src/fixed/int.rs` を新規作成:

```rust
//! 固定小数点版の実装

use core::ops::{Add, Mul, Neg, Sub};

/// i64をi32の範囲に飽和させる
const fn sat_i32(v: i64) -> i32 {
	if v > i32::MAX as i64 {
		i32::MAX
	} else if v < i32::MIN as i64 {
		i32::MIN
	} else {
		v as i32
	}
}

/// f32を2^frac倍して最も近い整数にする。有限でない、またはi64に収まらなければNone
fn scale_f32(v: f32, frac: u32) -> Option<i64> {
	if !v.is_finite() {
		return None;
	}
	let x = v as f64 * (1u64 << frac) as f64;
	let r = if x >= 0.0 { x + 0.5 } else { x - 0.5 };
	// `as i64` は飽和するので、範囲外は先に弾く
	if r >= i64::MAX as f64 || r < i64::MIN as f64 {
		return None;
	}
	Some(r as i64)
}

/// per-unitの信号とゲイン。Q3.28
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Q3_28(i32);

impl Q3_28 {
	const FRAC: u32 = 28;

	pub const ZERO: Self = Q3_28(0);
	pub const ONE: Self = Q3_28(1 << Self::FRAC);
	pub const MAX: Self = Q3_28(i32::MAX);
	pub const MIN: Self = Q3_28(i32::MIN);

	/// 設定時・テスト用。範囲外やNaNはNone
	pub fn checked_from_f32(v: f32) -> Option<Self> {
		let r = scale_f32(v, Self::FRAC)?;
		i32::try_from(r).ok().map(Q3_28)
	}

	pub fn to_f32(self) -> f32 {
		self.0 as f32 / (1u32 << Self::FRAC) as f32
	}

	pub fn min(self, rhs: Self) -> Self {
		Q3_28(self.0.min(rhs.0))
	}

	pub fn max(self, rhs: Self) -> Self {
		Q3_28(self.0.max(rhs.0))
	}

	pub fn abs(self) -> Self {
		Q3_28(self.0.saturating_abs())
	}

	/// 積を丸めずにQ3.60で返す
	pub fn widening_mul(self, rhs: Self) -> Q3_60 {
		// 積はQ6.56。Q3.60に揃える4bit左シフトであふれる場合は飽和させる
		let p = self.0 as i64 * rhs.0 as i64;
		if p > i64::MAX >> 4 {
			Q3_60::MAX
		} else if p < i64::MIN >> 4 {
			Q3_60::MIN
		} else {
			Q3_60(p << 4)
		}
	}

	/// 0除算、または結果が範囲に収まらないときNone
	pub fn checked_div(self, rhs: Self) -> Option<Self> {
		let q = ((self.0 as i64) << Self::FRAC).checked_div(rhs.0 as i64)?;
		i32::try_from(q).ok().map(Q3_28)
	}

	/// 結果が範囲に収まることを呼び出し側が保証する除算。
	/// debugビルドでは違反を検出する。releaseでは検査せず、違反時の結果は不定
	pub fn unchecked_div(self, rhs: Self) -> Self {
		debug_assert!(
			self.checked_div(rhs).is_some(),
			"unchecked_div out of range: {:?} / {:?}", self, rhs
		);
		let q = ((self.0 as i64) << Self::FRAC).checked_div(rhs.0 as i64).unwrap_or(0);
		Q3_28(q as i32)
	}

	pub fn to_q3_60(self) -> Q3_60 {
		Q3_60((self.0 as i64) << 32)
	}

	pub fn to_q16_16(self) -> Q16_16 {
		Q16_16(self.0 >> 12)
	}
}

impl Add for Q3_28 {
	type Output = Self;
	fn add(self, rhs: Self) -> Self {
		Q3_28(self.0.saturating_add(rhs.0))
	}
}

impl Sub for Q3_28 {
	type Output = Self;
	fn sub(self, rhs: Self) -> Self {
		Q3_28(self.0.saturating_sub(rhs.0))
	}
}

impl Neg for Q3_28 {
	type Output = Self;
	fn neg(self) -> Self {
		Q3_28(self.0.saturating_neg())
	}
}

impl Mul for Q3_28 {
	type Output = Self;
	fn mul(self, rhs: Self) -> Self {
		Q3_28(sat_i32((self.0 as i64 * rhs.0 as i64) >> Self::FRAC))
	}
}

/// 積分器用。Q3.60
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Q3_60(i64);

impl Q3_60 {
	const FRAC: u32 = 60;

	pub const ZERO: Self = Q3_60(0);
	pub const MAX: Self = Q3_60(i64::MAX);
	pub const MIN: Self = Q3_60(i64::MIN);

	/// 設定時・テスト用。範囲外やNaNはNone
	pub fn checked_from_f32(v: f32) -> Option<Self> {
		scale_f32(v, Self::FRAC).map(Q3_60)
	}

	pub fn to_f32(self) -> f32 {
		(self.0 as f64 / (1u64 << Self::FRAC) as f64) as f32
	}

	pub fn min(self, rhs: Self) -> Self {
		Q3_60(self.0.min(rhs.0))
	}

	pub fn max(self, rhs: Self) -> Self {
		Q3_60(self.0.max(rhs.0))
	}

	pub fn abs(self) -> Self {
		Q3_60(self.0.saturating_abs())
	}

	/// 下位32bitを切り捨ててQ3.28にする。範囲は同じなので飽和しない
	pub fn to_q3_28(self) -> Q3_28 {
		Q3_28((self.0 >> 32) as i32)
	}
}

impl Add for Q3_60 {
	type Output = Self;
	fn add(self, rhs: Self) -> Self {
		Q3_60(self.0.saturating_add(rhs.0))
	}
}

impl Sub for Q3_60 {
	type Output = Self;
	fn sub(self, rhs: Self) -> Self {
		Q3_60(self.0.saturating_sub(rhs.0))
	}
}

impl Neg for Q3_60 {
	type Output = Self;
	fn neg(self) -> Self {
		Q3_60(self.0.saturating_neg())
	}
}

/// 位置用。Q16.16、1回転 = 1.0
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Q16_16(i32);

impl Q16_16 {
	const FRAC: u32 = 16;

	pub const ZERO: Self = Q16_16(0);
	pub const ONE: Self = Q16_16(1 << Self::FRAC);
	pub const MAX: Self = Q16_16(i32::MAX);
	pub const MIN: Self = Q16_16(i32::MIN);

	/// 設定時・テスト用。範囲外やNaNはNone
	pub fn checked_from_f32(v: f32) -> Option<Self> {
		let r = scale_f32(v, Self::FRAC)?;
		i32::try_from(r).ok().map(Q16_16)
	}

	pub fn to_f32(self) -> f32 {
		self.0 as f32 / (1u32 << Self::FRAC) as f32
	}

	pub fn min(self, rhs: Self) -> Self {
		Q16_16(self.0.min(rhs.0))
	}

	pub fn max(self, rhs: Self) -> Self {
		Q16_16(self.0.max(rhs.0))
	}

	pub fn abs(self) -> Self {
		Q16_16(self.0.saturating_abs())
	}

	/// Q3.28のゲインを掛けてQ3.28にする
	pub fn mul_q3_28(self, k: Q3_28) -> Q3_28 {
		// Q16.16 * Q3.28 = Q19.44 -> 16bit右シフトでQ3.28
		Q3_28(sat_i32((self.0 as i64 * k.0 as i64) >> Self::FRAC))
	}

	pub fn to_q3_28(self) -> Q3_28 {
		Q3_28(sat_i32((self.0 as i64) << 12))
	}
}

impl Add for Q16_16 {
	type Output = Self;
	fn add(self, rhs: Self) -> Self {
		Q16_16(self.0.saturating_add(rhs.0))
	}
}

impl Sub for Q16_16 {
	type Output = Self;
	fn sub(self, rhs: Self) -> Self {
		Q16_16(self.0.saturating_sub(rhs.0))
	}
}

impl Neg for Q16_16 {
	type Output = Self;
	fn neg(self) -> Self {
		Q16_16(self.0.saturating_neg())
	}
}
```

- [ ] **Step 5: テストが通ることを確認する**

Run: `cargo test -p md-core`
Expected: `test result: ok.`。`fixed::tests::` のテストがすべて ok、既存の `controller::tests::` と `scalar::tests::` も ok のまま。

- [ ] **Step 6: ターゲット向けにビルドできることを確認する**

Run: `cargo build -p md-core --target thumbv7m-none-eabi`
Expected: `Finished`（エラーなし）

- [ ] **Step 7: コミット**

```powershell
git add md-core/Cargo.toml md-core/src/lib.rs md-core/src/fixed.rs md-core/src/fixed/int.rs
git commit -m "Add fixed-point number types Q3_28, Q3_60, Q16_16"
```

---

### Task 2: 浮動小数点 backend（feature `f32`）

**Files:**
- Create: `md-core/src/fixed/float.rs`

**Interfaces:**
- Consumes: Task 1 の `md-core/src/fixed.rs`（`cfg(feature = "f32")` で `mod float;` を宣言済み。テストも共通）
- Produces: Task 1 の Produces と同一の API を、`Q3_28(f32)`、`Q3_60(f64)`、`Q16_16(f64)` として提供する。範囲と飽和の振る舞いは整数 backend と同じ。

- [ ] **Step 1: f32 版でテストが失敗することを確認する**

Run: `cargo test -p md-core --features f32`
Expected: コンパイルエラー `file not found for module `float``

- [ ] **Step 2: 浮動小数点 backend を実装する**

`md-core/src/fixed/float.rs` を新規作成:

```rust
//! 浮動小数点版の実装。範囲と飽和の振る舞いは固定小数点版に合わせる

use core::ops::{Add, Mul, Neg, Sub};

/// per-unitの信号とゲイン。固定小数点版のQ3.28に相当
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Q3_28(f32);

impl Q3_28 {
	pub const ZERO: Self = Q3_28(0.0);
	pub const ONE: Self = Q3_28(1.0);
	/// 8未満で最大のf32
	pub const MAX: Self = Q3_28(f32::from_bits(0x40FF_FFFF));
	pub const MIN: Self = Q3_28(-8.0);

	fn sat(v: f32) -> Self {
		Q3_28(v.clamp(Self::MIN.0, Self::MAX.0))
	}

	/// NaNはfalse
	fn in_range(v: f32) -> bool {
		v >= Self::MIN.0 && v <= Self::MAX.0
	}

	/// 設定時・テスト用。範囲外やNaNはNone
	pub fn checked_from_f32(v: f32) -> Option<Self> {
		if Self::in_range(v) { Some(Q3_28(v)) } else { None }
	}

	pub fn to_f32(self) -> f32 {
		self.0
	}

	pub fn min(self, rhs: Self) -> Self {
		Q3_28(self.0.min(rhs.0))
	}

	pub fn max(self, rhs: Self) -> Self {
		Q3_28(self.0.max(rhs.0))
	}

	pub fn abs(self) -> Self {
		Self::sat(self.0.abs())
	}

	/// 積を丸めずにQ3_60で返す
	pub fn widening_mul(self, rhs: Self) -> Q3_60 {
		Q3_60::sat(self.0 as f64 * rhs.0 as f64)
	}

	/// 0除算、または結果が範囲に収まらないときNone
	pub fn checked_div(self, rhs: Self) -> Option<Self> {
		if rhs.0 == 0.0 {
			return None;
		}
		let q = self.0 / rhs.0;
		if Self::in_range(q) { Some(Q3_28(q)) } else { None }
	}

	/// 結果が範囲に収まることを呼び出し側が保証する除算。
	/// debugビルドでは違反を検出する。releaseでは検査せず、違反時の結果は不定
	pub fn unchecked_div(self, rhs: Self) -> Self {
		debug_assert!(
			self.checked_div(rhs).is_some(),
			"unchecked_div out of range: {:?} / {:?}", self, rhs
		);
		Q3_28(self.0 / rhs.0)
	}

	pub fn to_q3_60(self) -> Q3_60 {
		Q3_60(self.0 as f64)
	}

	pub fn to_q16_16(self) -> Q16_16 {
		Q16_16(self.0 as f64)
	}
}

impl Add for Q3_28 {
	type Output = Self;
	fn add(self, rhs: Self) -> Self {
		Self::sat(self.0 + rhs.0)
	}
}

impl Sub for Q3_28 {
	type Output = Self;
	fn sub(self, rhs: Self) -> Self {
		Self::sat(self.0 - rhs.0)
	}
}

impl Neg for Q3_28 {
	type Output = Self;
	fn neg(self) -> Self {
		Self::sat(-self.0)
	}
}

impl Mul for Q3_28 {
	type Output = Self;
	fn mul(self, rhs: Self) -> Self {
		Self::sat(self.0 * rhs.0)
	}
}

/// 積分器用。固定小数点版のQ3.60に相当。f32では微小な増分が消えるのでf64で持つ
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Q3_60(f64);

impl Q3_60 {
	pub const ZERO: Self = Q3_60(0.0);
	/// 8未満で最大のf64
	pub const MAX: Self = Q3_60(f64::from_bits(0x401F_FFFF_FFFF_FFFF));
	pub const MIN: Self = Q3_60(-8.0);

	fn sat(v: f64) -> Self {
		Q3_60(v.clamp(Self::MIN.0, Self::MAX.0))
	}

	/// 設定時・テスト用。範囲外やNaNはNone
	pub fn checked_from_f32(v: f32) -> Option<Self> {
		let x = v as f64;
		if x >= Self::MIN.0 && x <= Self::MAX.0 { Some(Q3_60(x)) } else { None }
	}

	pub fn to_f32(self) -> f32 {
		self.0 as f32
	}

	pub fn min(self, rhs: Self) -> Self {
		Q3_60(self.0.min(rhs.0))
	}

	pub fn max(self, rhs: Self) -> Self {
		Q3_60(self.0.max(rhs.0))
	}

	pub fn abs(self) -> Self {
		Self::sat(self.0.abs())
	}

	pub fn to_q3_28(self) -> Q3_28 {
		Q3_28::sat(self.0 as f32)
	}
}

impl Add for Q3_60 {
	type Output = Self;
	fn add(self, rhs: Self) -> Self {
		Self::sat(self.0 + rhs.0)
	}
}

impl Sub for Q3_60 {
	type Output = Self;
	fn sub(self, rhs: Self) -> Self {
		Self::sat(self.0 - rhs.0)
	}
}

impl Neg for Q3_60 {
	type Output = Self;
	fn neg(self) -> Self {
		Self::sat(-self.0)
	}
}

/// 位置用。固定小数点版のQ16.16に相当、1回転 = 1.0。多回転でも分解能を保つためf64で持つ
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Q16_16(f64);

impl Q16_16 {
	pub const ZERO: Self = Q16_16(0.0);
	pub const ONE: Self = Q16_16(1.0);
	/// 32768 - 2^-16
	pub const MAX: Self = Q16_16(32767.9999847412109375);
	pub const MIN: Self = Q16_16(-32768.0);

	fn sat(v: f64) -> Self {
		Q16_16(v.clamp(Self::MIN.0, Self::MAX.0))
	}

	/// 設定時・テスト用。範囲外やNaNはNone
	pub fn checked_from_f32(v: f32) -> Option<Self> {
		let x = v as f64;
		if x >= Self::MIN.0 && x <= Self::MAX.0 { Some(Q16_16(x)) } else { None }
	}

	pub fn to_f32(self) -> f32 {
		self.0 as f32
	}

	pub fn min(self, rhs: Self) -> Self {
		Q16_16(self.0.min(rhs.0))
	}

	pub fn max(self, rhs: Self) -> Self {
		Q16_16(self.0.max(rhs.0))
	}

	pub fn abs(self) -> Self {
		Self::sat(self.0.abs())
	}

	/// Q3_28のゲインを掛けてQ3_28にする
	pub fn mul_q3_28(self, k: Q3_28) -> Q3_28 {
		Q3_28::sat((self.0 * k.0 as f64) as f32)
	}

	pub fn to_q3_28(self) -> Q3_28 {
		Q3_28::sat(self.0 as f32)
	}
}

impl Add for Q16_16 {
	type Output = Self;
	fn add(self, rhs: Self) -> Self {
		Self::sat(self.0 + rhs.0)
	}
}

impl Sub for Q16_16 {
	type Output = Self;
	fn sub(self, rhs: Self) -> Self {
		Self::sat(self.0 - rhs.0)
	}
}

impl Neg for Q16_16 {
	type Output = Self;
	fn neg(self) -> Self {
		Self::sat(-self.0)
	}
}
```

- [ ] **Step 3: 両方の backend でテストが通ることを確認する**

Run: `cargo test -p md-core --features f32`
Expected: `test result: ok.`（`q3_28_mul_drops_product_below_resolution` は f32 版では対象外なので、件数は整数版より 1 少ない）

Run: `cargo test -p md-core`
Expected: `test result: ok.`

- [ ] **Step 4: ターゲット向けにビルドできることを確認する**

Run: `cargo build -p md-core --target thumbv7m-none-eabi --features f32`
Expected: `Finished`（エラーなし）

- [ ] **Step 5: コミット**

```powershell
git add md-core/src/fixed/float.rs
git commit -m "Add floating-point backend behind f32 feature"
```

---

### Task 3: 制御器の計算を新しい型へ移行する

`controller.rs` の制御計算を `Q3_28` / `Q3_60` / `Q16_16` に移す。`Config` と `Param::new` は Task 4 で作り直すので、このタスクでは古い `Param::new(&Config)` を削除し、`Config` 本体（`Scalar` のまま）は触らない。Param はテスト内でフィールドを直接組み立てる。

**Files:**
- Modify: `md-core/src/controller.rs`（`Config` 構造体以外のすべて）

**Interfaces:**
- Consumes: `md_core::fixed::{Q3_28, Q3_60, Q16_16}`（Task 1 の Produces）
- Produces（`md_core::controller`）:
  - `pub struct Measurement { pub vdc: Q3_28, pub vdc_inv: Q3_28, pub i: Q3_28, pub w: Q3_28, pub th: Q16_16 }`
  - `pub struct CurrentParam { kp, ki, kb, ke, dead_duty, i_threshold, duty_max, vmax, imax: Q3_28 }`（フィールド非公開。コンストラクタは Task 4）
  - `pub struct VelocityParam { kp, ki, b, wmax: Q3_28 }`（同上）
  - `pub struct PositionParam { kp: Q3_28, pmax: Q16_16 }`（同上）
  - `CurrentState::new()`、`CurrentState::update(&mut self, p: &CurrentParam, u: Q3_28, m: &Measurement) -> (Q3_28, Saturated)`、`CurrentState::reset(&mut self)`
  - `VelocityState::new()`、`VelocityState::update(&mut self, p: &VelocityParam, u: Q3_28, m: &Measurement, last_saturated: Saturated) -> Q3_28`、`VelocityState::reset(&mut self)`
  - `PositionState::new()`、`PositionState::update(&self, p: &PositionParam, u: Q16_16, m: &Measurement) -> Q3_28`
  - モジュール内部: `struct Integrator`（`new`、`add_product(&mut self, k: Q3_28, e: Q3_28)`、`value(&self) -> Q3_28`、`reset`）、`fn soft_sign(i: Q3_28, th: Q3_28) -> Q3_28`

- [ ] **Step 1: テストモジュールを新しい型で書き直す**

`md-core/src/controller.rs` の `#[cfg(test)] mod tests { ... }` を丸ごと次に置き換える:

```rust
#[cfg(test)]
mod tests {
	use super::*;

	fn q(v: f32) -> Q3_28 {
		Q3_28::checked_from_f32(v).unwrap()
	}

	fn q16(v: f32) -> Q16_16 {
		Q16_16::checked_from_f32(v).unwrap()
	}

	/// 計算過程で丸めが入る値の比較用
	fn assert_close(actual: Q3_28, expected: f32) {
		let a = actual.to_f32();
		assert!((a - expected).abs() < 1e-6, "left: {a}, right: {expected}");
	}

	/// i, w, th 以外は母線0.5pu固定の測定値
	fn meas(i: f32, w: f32, th: f32) -> Measurement {
		Measurement { vdc: q(0.5), vdc_inv: q(2.0), i: q(i), w: q(w), th: q16(th) }
	}

	/// 補償を無効にした素のパラメータ。duty_maxは1、電流制限は効かせない
	fn bare_current_param(kp: f32, ki: f32, kb: f32, vmax: f32) -> CurrentParam {
		CurrentParam {
			kp: q(kp),
			ki: q(ki),
			kb: q(kb),
			ke: Q3_28::ZERO,
			dead_duty: Q3_28::ZERO,
			i_threshold: q(1.0),
			duty_max: q(1.0),
			vmax: q(vmax),
			imax: q(7.0),
		}
	}

	/// kiは周期Tを含んだ値
	fn vparam(kp: f32, ki: f32, b: f32, wmax: f32) -> VelocityParam {
		VelocityParam { kp: q(kp), ki: q(ki), b: q(b), wmax: q(wmax) }
	}

	// ---- Integrator ----

	#[test]
	fn integrator_accumulates_products() {
		let mut it = Integrator::new();
		it.add_product(q(0.5), q(0.25));
		it.add_product(q(0.5), q(0.25));
		assert_close(it.value(), 0.25);
	}

	#[test]
	fn integrator_keeps_increments_below_q3_28_resolution() {
		// 1回の増分は 1e-4 * 1e-5 = 1e-9 で、Q3.28 の分解能(約3.7e-9)より小さい
		let mut it = Integrator::new();
		for _ in 0..1000 {
			it.add_product(q(1e-4), q(1e-5));
		}
		let v = it.value().to_f32();
		assert!((v - 1e-6).abs() < 1e-8, "left: {v}");
	}

	#[test]
	fn integrator_saturates_instead_of_wrapping() {
		let mut it = Integrator::new();
		for _ in 0..4 {
			it.add_product(q(4.0), q(4.0));
		}
		assert_eq!(it.value(), Q3_28::MAX);

		let mut it = Integrator::new();
		for _ in 0..4 {
			it.add_product(q(-4.0), q(4.0));
		}
		assert_eq!(it.value(), Q3_28::MIN);
	}

	#[test]
	fn integrator_reset_clears() {
		let mut it = Integrator::new();
		it.add_product(q(0.5), q(0.5));
		it.reset();
		assert_eq!(it.value(), Q3_28::ZERO);
	}

	// ---- soft_sign ----

	#[test]
	fn soft_sign_is_sign_outside_threshold() {
		assert_eq!(soft_sign(q(0.5), q(0.125)), Q3_28::ONE);
		assert_eq!(soft_sign(q(-0.5), q(0.125)), -Q3_28::ONE);
		// しきい値ちょうども±1
		assert_eq!(soft_sign(q(0.125), q(0.125)), Q3_28::ONE);
		assert_eq!(soft_sign(q(-0.125), q(0.125)), -Q3_28::ONE);
	}

	#[test]
	fn soft_sign_is_linear_inside_threshold() {
		assert_close(soft_sign(q(0.25), q(0.5)), 0.5);
		assert_close(soft_sign(q(-0.125), q(0.5)), -0.25);
		assert_close(soft_sign(Q3_28::ZERO, q(0.5)), 0.0);
	}

	#[test]
	fn soft_sign_with_zero_threshold_does_not_divide() {
		// しきい値がpu変換の丸めで0になっても0除算しない
		assert_eq!(soft_sign(q(0.5), Q3_28::ZERO), Q3_28::ONE);
		assert_eq!(soft_sign(q(-0.5), Q3_28::ZERO), -Q3_28::ONE);
		assert_eq!(soft_sign(Q3_28::ZERO, Q3_28::ZERO), Q3_28::ONE);
	}

	// ---- 電流制御 ----

	#[test]
	fn current_update_converts_voltage_to_duty() {
		let p = bare_current_param(0.5, 0.0, 0.0, 7.0);
		let mut st = CurrentState::new();
		// e = 0.75 - 0.25 = 0.5, v = 0.5*0.5 = 0.25, duty = 0.25/0.5 = 0.5
		let (duty, _) = st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0));
		assert_close(duty, 0.5);
	}

	#[test]
	fn current_update_limits_voltage_by_vmax() {
		let p = bare_current_param(0.5, 0.0, 0.0, 0.125);
		let mut st = CurrentState::new();
		// v = 0.25 だが vmax=0.125 で頭打ち -> duty = 0.125/0.5 = 0.25
		let (duty, _) = st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0));
		assert_close(duty, 0.25);
	}

	#[test]
	fn current_voltage_limit_follows_measured_bus_voltage() {
		// duty_max=0.5。母線が0.5puなら0.25pu、0.25puに下がれば0.125puが上限になる
		let mut p = bare_current_param(4.0, 0.0, 0.0, 7.0);
		p.duty_max = q(0.5);

		let mut st = CurrentState::new();
		let m_high = Measurement { vdc: q(0.5), vdc_inv: q(2.0), i: q(0.0), w: q(0.0), th: q16(0.0) };
		assert_close(st.update(&p, q(0.5), &m_high).0, 0.5);

		let mut st = CurrentState::new();
		let m_low = Measurement { vdc: q(0.25), vdc_inv: q(4.0), i: q(0.0), w: q(0.0), th: q16(0.0) };
		// 母線が下がってもデューティは duty_max で頭打ちのまま
		assert_close(st.update(&p, q(0.5), &m_low).0, 0.5);
	}

	#[test]
	fn current_update_compensates_back_emf() {
		let mut p = bare_current_param(0.5, 0.0, 0.0, 7.0);
		p.ke = q(0.25);
		let mut st = CurrentState::new();
		// v = 0.5*0.5 + 0.25*0.5 = 0.375 -> duty = 0.75
		assert_close(st.update(&p, q(0.75), &meas(0.25, 0.5, 0.0)).0, 0.75);
	}

	#[test]
	fn current_update_adds_dead_time_duty_after_conversion() {
		let mut p = bare_current_param(0.5, 0.0, 0.0, 7.0);
		p.dead_duty = q(0.0625);
		p.i_threshold = q(0.125);
		let mut st = CurrentState::new();
		// duty = 0.5 + 0.0625*soft_sign(0.25) = 0.5625
		assert_close(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).0, 0.5625);
	}

	#[test]
	fn current_dead_time_duty_is_softened_near_zero_current() {
		let mut p = bare_current_param(0.0, 0.0, 0.0, 7.0);
		p.dead_duty = q(0.125);
		p.i_threshold = q(0.5);
		let mut st = CurrentState::new();
		// i=0.25 はしきい値0.5の内側なので 0.125*0.5 = 0.0625
		assert_close(st.update(&p, q(0.25), &meas(0.25, 0.0, 0.0)).0, 0.0625);
		assert_close(st.update(&p, q(-0.25), &meas(-0.25, 0.0, 0.0)).0, -0.0625);
	}

	#[test]
	fn current_duty_is_clamped_to_plus_minus_one() {
		let mut p = bare_current_param(2.0, 0.0, 0.0, 7.0);
		p.dead_duty = q(0.0625);
		p.i_threshold = q(0.125);
		let mut st = CurrentState::new();
		// v = 2*0.5 = 1.0 -> 母線0.5puで頭打ち -> duty 1.0 + 0.0625 -> 1.0
		assert_close(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).0, 1.0);

		let mut st = CurrentState::new();
		// 逆向きも同様に -1.0 - 0.0625 -> -1.0
		assert_close(st.update(&p, q(-0.75), &meas(-0.25, 0.0, 0.0)).0, -1.0);
	}

	#[test]
	fn current_update_clamps_target_current() {
		let mut p = bare_current_param(0.5, 0.0, 0.0, 7.0);
		p.imax = q(0.5);
		let mut st = CurrentState::new();
		// u=2 は imax=0.5 に制限される -> e = 0.25, v = 0.125 -> duty 0.25
		assert_close(st.update(&p, q(2.0), &meas(0.25, 0.0, 0.0)).0, 0.25);

		let mut st = CurrentState::new();
		// e = -0.5-0.25 = -0.75, v = -0.375 -> duty -0.75
		assert_close(st.update(&p, q(-2.0), &meas(0.25, 0.0, 0.0)).0, -0.75);
	}

	#[test]
	fn current_clamped_target_also_applies_to_integral() {
		let mut p = bare_current_param(0.0, 1.0, 0.0, 7.0);
		p.imax = q(0.5);
		let mut st = CurrentState::new();
		st.update(&p, q(2.0), &meas(0.25, 0.0, 0.0));
		// 制限後のu=0.5を使うので i_sum = 1*(0.5-0.25) = 0.25
		assert_close(st.i_sum.value(), 0.25);
	}

	#[test]
	fn current_update_integrates_error() {
		let p = bare_current_param(0.0, 0.5, 0.0, 7.0);
		let mut st = CurrentState::new();
		// kp=0なので1回目の出力は0、積分だけが溜まる
		assert_close(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).0, 0.0);
		// i_sum = 0.5*0.5 = 0.25 -> 2回目は duty = 0.25/0.5 = 0.5
		assert_close(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).0, 0.5);
	}

	#[test]
	fn current_update_unwinds_integral_when_saturated() {
		let p = bare_current_param(0.5, 0.25, 0.5, 0.125);
		let mut st = CurrentState::new();
		st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0));
		// i_sum = ki*e + kb*(v_limited - v_raw) = 0.25*0.5 + 0.5*(0.125-0.25) = 0.0625
		assert_close(st.i_sum.value(), 0.0625);
	}

	#[test]
	fn current_reset_clears_integral() {
		let p = bare_current_param(0.0, 0.5, 0.0, 7.0);
		let mut st = CurrentState::new();
		st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0));
		st.reset();
		assert_close(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).0, 0.0);
	}

	#[test]
	fn current_reports_not_saturated_within_limits() {
		let p = bare_current_param(0.5, 0.0, 0.0, 7.0);
		let mut st = CurrentState::new();
		assert_eq!(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).1, Saturated::NotSaturated);
	}

	#[test]
	fn current_reports_overflow_when_voltage_clipped_high() {
		let p = bare_current_param(0.5, 0.0, 0.0, 0.125);
		let mut st = CurrentState::new();
		// v = 0.25 -> 0.125で頭打ち
		assert_eq!(st.update(&p, q(0.75), &meas(0.25, 0.0, 0.0)).1, Saturated::Overflow);
	}

	#[test]
	fn current_reports_underflow_when_voltage_clipped_low() {
		let p = bare_current_param(0.5, 0.0, 0.0, 0.125);
		let mut st = CurrentState::new();
		// e = -0.75 - 0.25 = -1, v = -0.5 -> -0.125で頭打ち
		assert_eq!(st.update(&p, q(-0.75), &meas(0.25, 0.0, 0.0)).1, Saturated::Underflow);
	}

	#[test]
	fn current_reports_saturation_when_target_current_clamped() {
		let mut p = bare_current_param(0.5, 0.0, 0.0, 7.0);
		p.imax = q(0.5);
		let mut st = CurrentState::new();
		assert_eq!(st.update(&p, q(2.0), &meas(0.25, 0.0, 0.0)).1, Saturated::Overflow);

		let mut st = CurrentState::new();
		assert_eq!(st.update(&p, q(-2.0), &meas(0.25, 0.0, 0.0)).1, Saturated::Underflow);

		let mut st = CurrentState::new();
		assert_eq!(st.update(&p, q(0.375), &meas(0.25, 0.0, 0.0)).1, Saturated::NotSaturated);
	}

	// ---- 速度制御 ----

	#[test]
	fn velocity_update_weights_target_in_p_term() {
		// b=1 なら普通のP制御: 0.5*(0.75 - 0.25) = 0.25
		let p = vparam(0.5, 0.0, 1.0, 7.0);
		let mut st = VelocityState::new();
		assert_close(st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 0.25);

		// b=0.5 なら目標値の寄与が半分: 0.5*(0.375 - 0.25) = 0.0625
		let p = vparam(0.5, 0.0, 0.5, 7.0);
		let mut st = VelocityState::new();
		assert_close(st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 0.0625);
	}

	#[test]
	fn velocity_update_integrates_unweighted_error_scaled_by_ki() {
		// 積分項には重みをかけない(定常偏差を残さないため)
		let p = vparam(0.0, 0.5, 0.5, 7.0);
		let mut st = VelocityState::new();
		assert_close(st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 0.0);
		// w_sum = 0.5*(0.75-0.25) = 0.25
		assert_close(st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 0.25);
	}

	#[test]
	fn velocity_update_clamps_target_speed() {
		let p = vparam(0.5, 0.0, 1.0, 0.5);
		let mut st = VelocityState::new();
		// u=2 は wmax=0.5 に制限される -> 0.5*(0.5-0.25) = 0.125
		assert_close(st.update(&p, q(2.0), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 0.125);

		let mut st = VelocityState::new();
		// 0.5*(-0.5-0.25) = -0.375
		assert_close(st.update(&p, q(-2.0), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), -0.375);
	}

	#[test]
	fn velocity_clamped_target_also_applies_to_integral() {
		let p = vparam(0.0, 1.0, 1.0, 0.5);
		let mut st = VelocityState::new();
		st.update(&p, q(2.0), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated);
		// 制限後のu=0.5を使うので w_sum = 1*(0.5-0.25) = 0.25
		assert_close(st.update(&p, q(2.0), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 0.25);
	}

	#[test]
	fn velocity_reset_clears_integral() {
		let p = vparam(0.0, 1.0, 1.0, 7.0);
		let mut st = VelocityState::new();
		st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated);
		st.reset();
		assert_close(st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 0.0);
	}

	#[test]
	fn velocity_freezes_integral_while_inner_overflow() {
		let p = vparam(1.0, 1.0, 1.0, 7.0);
		let mut st = VelocityState::new();
		// e = 0.5 > 0。上側に飽和中に正方向へ積分を進めると悪化するので止める
		st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::Overflow);
		// 積分が溜まっていなければ2回目もP項だけ: 0.5
		assert_close(st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::Overflow), 0.5);
	}

	#[test]
	fn velocity_freezes_integral_while_inner_underflow() {
		let p = vparam(1.0, 1.0, 1.0, 7.0);
		let mut st = VelocityState::new();
		// e = -1 < 0。下側に飽和中に負方向へ積分を進めると悪化するので止める
		st.update(&p, q(-0.75), &meas(0.0, 0.25, 0.0), Saturated::Underflow);
		assert_close(st.update(&p, q(-0.75), &meas(0.0, 0.25, 0.0), Saturated::Underflow), -1.0);
	}

	#[test]
	fn velocity_keeps_integrating_when_error_escapes_saturation() {
		let p = vparam(1.0, 1.0, 1.0, 7.0);
		let mut st = VelocityState::new();
		// まず正方向に積分を溜める(飽和なし)。w_sum = 0.5
		st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated);
		// 上側に飽和中でも誤差が負なら積分を戻せる必要がある
		// 出力 = (-0.25-0.25) + 0.5 = 0
		assert_close(st.update(&p, q(-0.25), &meas(0.0, 0.25, 0.0), Saturated::Overflow), 0.0);
		// w_sum = 0.5 + (-0.5) = 0 なので、次は P項だけ
		assert_close(st.update(&p, q(-0.25), &meas(0.0, 0.25, 0.0), Saturated::Overflow), -0.5);
	}

	#[test]
	fn velocity_integral_freeze_needs_saturation() {
		// 飽和していなければ、誤差が出力と同符号でも普通に積分する
		let p = vparam(1.0, 1.0, 1.0, 7.0);
		let mut st = VelocityState::new();
		st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated);
		// w_sum = 0.5 -> 0.5 + 0.5 = 1.0
		assert_close(st.update(&p, q(0.75), &meas(0.0, 0.25, 0.0), Saturated::NotSaturated), 1.0);
	}

	// ---- 位置制御 ----

	#[test]
	fn position_update_is_proportional() {
		let p = PositionParam { kp: q(0.5), pmax: q16(100.0) };
		let st = PositionState::new();
		// 0.5*(2 - 0.5) = 0.75
		assert_close(st.update(&p, q16(2.0), &meas(0.0, 0.0, 0.5)), 0.75);
		assert_close(st.update(&p, q16(0.0), &meas(0.0, 0.0, 0.5)), -0.25);
	}

	#[test]
	fn position_update_clamps_target_position() {
		let p = PositionParam { kp: q(0.5), pmax: q16(1.0) };
		let st = PositionState::new();
		// u=5 は pmax=1 に制限される -> 0.5*(1 - 0.5) = 0.25
		assert_close(st.update(&p, q16(5.0), &meas(0.0, 0.0, 0.5)), 0.25);
		// u=-5 は -1 に制限される -> 0.5*(-1 - 0.5) = -0.75
		assert_close(st.update(&p, q16(-5.0), &meas(0.0, 0.0, 0.5)), -0.75);
	}

	#[test]
	fn position_error_beyond_eight_turns_is_not_clipped_before_gain() {
		// 誤差100回転はQ3_28に入らないが、ゲインを掛けた結果6.25は入る
		let p = PositionParam { kp: q(0.0625), pmax: q16(1000.0) };
		let st = PositionState::new();
		assert_close(st.update(&p, q16(100.0), &meas(0.0, 0.0, 0.0)), 6.25);
	}

	#[test]
	fn position_output_saturates_for_huge_error() {
		let p = PositionParam { kp: q(1.0), pmax: q16(1000.0) };
		let st = PositionState::new();
		assert_eq!(st.update(&p, q16(1000.0), &meas(0.0, 0.0, 0.0)), Q3_28::MAX);
		assert_eq!(st.update(&p, q16(-1000.0), &meas(0.0, 0.0, 0.0)), Q3_28::MIN);
	}
}
```

- [ ] **Step 2: テストが失敗することを確認する**

Run: `cargo test -p md-core`
Expected: コンパイルエラー（`cannot find type `Q3_28` in this scope`、`cannot find struct ... `Integrator`` など）

- [ ] **Step 3: 実装を書き換える**

`md-core/src/controller.rs` のテストモジュールより上を、次の内容にする。**`Config` 構造体のブロック（`// TODO: バリデート` から閉じ括弧まで）は現状のまま一切変更しない。** 下のコードでは `Config` の位置を「（現状の Config をここに残す）」と示している。

```rust
// TODO: 上限による制限を上限と下限による制限に変更

use crate::fixed::{Q3_28, Q3_60, Q16_16};
use crate::scalar::Scalar;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Saturated {
	NotSaturated,
	Overflow,
	Underflow,
}

/// 測定値。すべてper-unit
pub struct Measurement {
	pub vdc: Q3_28,
	pub vdc_inv: Q3_28,
	pub i: Q3_28,
	pub w: Q3_28,
	pub th: Q16_16,  // 1回転 = 1.0
}

// （現状の Config をここに残す）

/// 積分器。積を丸めずにQ3.60で積算し、Q3.28の分解能より小さい増分が消えないようにする
struct Integrator(Q3_60);
impl Integrator {
	fn new() -> Integrator {
		Integrator(Q3_60::ZERO)
	}

	/// self += k * e
	fn add_product(&mut self, k: Q3_28, e: Q3_28) {
		self.0 = self.0 + k.widening_mul(e);
	}

	fn value(&self) -> Q3_28 {
		self.0.to_q3_28()
	}

	fn reset(&mut self) {
		self.0 = Q3_60::ZERO;
	}
}

/// デッドタイム補償用の符号。0付近で暴れないよう、|i| < th の間は i/th で線形に鈍らせる
fn soft_sign(i: Q3_28, th: Q3_28) -> Q3_28 {
	if i >= th {
		Q3_28::ONE
	} else if i <= -th {
		-Q3_28::ONE
	} else {
		// |i| < th なので結果は(-1, 1)に収まる
		i.unchecked_div(th)
	}
}

pub struct CurrentParam {
	kp: Q3_28,
	ki: Q3_28,  // 周期Tを掛けた値
	kb: Q3_28,  // アンチワインドアップ。ki(T込み) / kp
	ke: Q3_28,
	dead_duty: Q3_28,
	i_threshold: Q3_28,  // 電流値が[-i_threshold, i_threshold]の間は符号を[-1, 1]に
	duty_max: Q3_28,
	vmax: Q3_28,
	imax: Q3_28,
}

pub struct CurrentState {
	i_sum: Integrator,  // 積分
}
impl CurrentState {
	pub fn new() -> CurrentState {
		CurrentState { i_sum: Integrator::new() }
	}

	/// 電流 -> デューティ比。PI -> 逆起電力補償 -> 電圧制限 + デューティ上限 -> デューティ換算 + デッドタイム補償
	pub fn update(&mut self, p: &CurrentParam, u: Q3_28, m: &Measurement) -> (Q3_28, Saturated) {
		// 目標電流を制限する
		let u_clamped = u.min(p.imax).max(-p.imax);
		let e = u_clamped - m.i;

		// PI
		let v_pi = p.kp * e + self.i_sum.value();
		// 逆起電力補償
		let v1 = v_pi + p.ke * m.w;
		// 最大電圧制限 & デューティ上限
		let vlim = p.vmax.min(p.duty_max * m.vdc);
		let v2 = v1.min(vlim).max(-vlim);

		// アンチワインドアップを入れた積算
		self.i_sum.add_product(p.ki, e);
		self.i_sum.add_product(p.kb, v2 - v1);

		// デューティ換算してデッドタイム補償(のデューティー比)を足す
		let duty1 = v2 * m.vdc_inv + p.dead_duty * soft_sign(m.i, p.i_threshold);

		let duty2 = duty1.min(Q3_28::ONE).max(-Q3_28::ONE);

		let saturate = if u > p.imax || v1 > v2 {
			Saturated::Overflow
		} else if u < -p.imax || v1 < v2 {
			Saturated::Underflow
		} else {
			Saturated::NotSaturated
		};

		(duty2, saturate)
	}

	pub fn reset(&mut self) {
		self.i_sum.reset();
	}
}

pub struct VelocityParam {
	kp: Q3_28,  // P制御
	ki: Q3_28,  // I制御。周期Tを掛けた値
	b: Q3_28,  // P項の目標値への重み
	wmax: Q3_28,  // 目標速度上限
}

pub struct VelocityState {
	w_sum: Integrator,
}
impl VelocityState {
	pub fn new() -> VelocityState {
		VelocityState { w_sum: Integrator::new() }
	}

	/// 速度 -> トルク。2自由度PI: Kp(b*u - w) + Ki∫(u - w)。uは±wmaxに制限する。
	pub fn update(
		&mut self,
		p: &VelocityParam,
		u: Q3_28,
		m: &Measurement,
		last_saturated: Saturated
	) -> Q3_28 {
		// 目標速度制限
		let u_clamped = u.min(p.wmax).max(-p.wmax);

		let w = p.kp * (p.b * u_clamped - m.w) + self.w_sum.value();

		// 条件付き積分
		let e = u_clamped - m.w;
		let is_flow_plus = e > Q3_28::ZERO;
		match (last_saturated, is_flow_plus) {
			(Saturated::Overflow, true) | (Saturated::Underflow, false) => {},
			_ => {
				self.w_sum.add_product(p.ki, e);
			},
		}

		w
	}

	pub fn reset(&mut self) {
		self.w_sum.reset();
	}
}

pub struct PositionParam {
	kp: Q3_28,
	pmax: Q16_16,
}

pub struct PositionState {}
impl PositionState {
	pub fn new() -> PositionState {
		PositionState {}
	}

	/// 位置 -> 速度。対処できる外乱がないのでP制御。uは±pmaxに制限する。
	pub fn update(&self, p: &PositionParam, u: Q16_16, m: &Measurement) -> Q3_28 {
		// 目標位置制限
		let u_clamped = u.min(p.pmax).max(-p.pmax);

		// 誤差が8回転を超えても速度指令が弱まらないよう、ゲインを掛けてからQ3.28にする
		(u_clamped - m.th).mul_q3_28(p.kp)
	}
}
```

確認事項:
- 元のファイルにあった `// TODO: vdcが定格を下回った場合に…` のコメントは削除する（Task 4 で実装するため）。
- `impl CurrentParam { pub fn new(c: &Config) … }`、`impl VelocityParam { … }`、`impl PositionParam { … }` の 3 つは削除されている（Task 4 で作り直す）。

- [ ] **Step 4: 両方の backend でテストが通ることを確認する**

Run: `cargo test -p md-core`
Expected: `test result: ok.`

Run: `cargo test -p md-core --features f32`
Expected: `test result: ok.`

- [ ] **Step 5: コミット**

```powershell
git add md-core/src/controller.rs
git commit -m "Port controllers to fixed-point per-unit types"
```

---

### Task 4: Config を物理単位にし、Param::new で per-unit へ変換する

設計書からの修正点が 1 つある。設計書は「`vdcmin < vdcmax/8` でエラー」としているが、`vdcmin == vdcmax/8` ちょうどでも逆数が 8 になり Q3.28（8 未満）に収まらない。そこで条件を「`vdcmin <= vdcmax/8` でエラー」とし、丸めの影響を避けるため per-unit 変換後の値で判定する。

母線電圧の下限（per-unit）は専用の構造体を作らず、`CurrentParam` に `vdc_min` として持たせる。検証と変換は `CurrentParam::new` で行う。

**Files:**
- Modify: `md-core/src/controller.rs`

**Interfaces:**
- Consumes: Task 3 の `CurrentParam` / `VelocityParam` / `PositionParam` / `Measurement`、Task 1 の `Q3_28::checked_from_f32`、`Q16_16::checked_from_f32`、`Q3_28::unchecked_div`
- Produces（`md_core::controller`）:
  - `pub struct Config`（下記の全フィールドが `pub f32`）
  - `pub enum ConfigError { NotPositive(&'static str), OutOfRange(&'static str), VdcMinTooLow }`（`Clone + Copy + PartialEq + Debug`。文字列は `Config` のフィールド名）
  - `CurrentParam::new(c: &Config) -> Result<CurrentParam, ConfigError>`
  - `VelocityParam::new(c: &Config) -> Result<VelocityParam, ConfigError>`
  - `PositionParam::new(c: &Config) -> Result<PositionParam, ConfigError>`
  - `CurrentParam` に非公開フィールド `vdc_min: Q3_28` を追加
  - `Measurement::update_vdc(&mut self, p: &CurrentParam, vdc: Q3_28) -> bool`（更新したら `true`、下限未満でスキップしたら `false`）

- [ ] **Step 1: 失敗するテストを書く**

`md-core/src/controller.rs` のテストモジュール内の `bare_current_param` で、`imax: q(7.0),` の次の行に `vdc_min: q(0.5),` を追加する（`CurrentParam` に増えるフィールドのぶん）。

続けて、`vparam` 関数の直後に追加:

```rust
	/// 物理単位の設定例。基準値は V_b=24V, I_b=10A, ω_b=300rad/s
	fn config() -> Config {
		Config {
			vdcmax: 24.0,
			vdcmin: 12.0,
			ibase: 10.0,
			wbase: 300.0,
			ke: 0.02,
			cperiod: 5e-5,
			ckp: 6.0,
			cki: 1000.0,
			dead_duty: 0.02,
			i_threshold: 0.5,
			duty_max: 0.95,
			vmax: 20.0,
			imax: 8.0,
			wperiod: 1e-3,
			wkp: 0.05,
			wki: 0.5,
			wb: 0.8,
			wmax: 250.0,
			pkp: 20.0,
			pmax: 100.0,
		}
	}

	// ---- Param::new ----

	#[test]
	fn current_param_new_converts_to_per_unit() {
		let p = CurrentParam::new(&config()).unwrap();
		assert_close(p.kp, 2.5);                 // 6 * 10 / 24
		assert_close(p.ki, 0.020833334);         // 1000 * 5e-5 * 10 / 24
		assert_close(p.kb, 0.008333334);         // 1000 * 5e-5 / 6
		assert_close(p.ke, 0.25);                // 0.02 * 300 / 24
		assert_close(p.dead_duty, 0.02);
		assert_close(p.i_threshold, 0.05);       // 0.5 / 10
		assert_close(p.duty_max, 0.95);
		assert_close(p.vmax, 0.8333333);         // 20 / 24
		assert_close(p.imax, 0.8);               // 8 / 10
		assert_close(p.vdc_min, 0.5);            // 12 / 24
	}

	#[test]
	fn velocity_param_new_converts_to_per_unit() {
		let p = VelocityParam::new(&config()).unwrap();
		assert_close(p.kp, 1.5);                 // 0.05 * 300 / 10
		assert_close(p.ki, 0.015);               // 0.5 * 1e-3 * 300 / 10
		assert_close(p.b, 0.8);
		assert_close(p.wmax, 0.8333333);         // 250 / 300
	}

	#[test]
	fn position_param_new_converts_to_per_unit() {
		let p = PositionParam::new(&config()).unwrap();
		assert_close(p.kp, 0.41887903);          // 20 * 2π / 300
		assert!((p.pmax.to_f32() - 100.0).abs() < 1e-3);  // 回転のまま
	}

	#[test]
	fn param_new_rejects_non_positive_values() {
		let mut c = config();
		c.ckp = 0.0;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::NotPositive("ckp")));

		let mut c = config();
		c.i_threshold = 0.0;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::NotPositive("i_threshold")));

		let mut c = config();
		c.cperiod = -1e-4;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::NotPositive("cperiod")));

		let mut c = config();
		c.vdcmax = 0.0;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::NotPositive("vdcmax")));

		let mut c = config();
		c.ibase = 0.0;
		assert_eq!(VelocityParam::new(&c).err(), Some(ConfigError::NotPositive("ibase")));

		let mut c = config();
		c.wperiod = 0.0;
		assert_eq!(VelocityParam::new(&c).err(), Some(ConfigError::NotPositive("wperiod")));

		let mut c = config();
		c.wbase = 0.0;
		assert_eq!(PositionParam::new(&c).err(), Some(ConfigError::NotPositive("wbase")));
	}

	#[test]
	fn param_new_rejects_values_out_of_range() {
		// 100 * 10 / 24 = 41.7 は Q3.28 に入らない
		let mut c = config();
		c.ckp = 100.0;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::OutOfRange("ckp")));

		let mut c = config();
		c.wkp = 1.0;  // 1 * 300 / 10 = 30
		assert_eq!(VelocityParam::new(&c).err(), Some(ConfigError::OutOfRange("wkp")));

		let mut c = config();
		c.pmax = 1e6;
		assert_eq!(PositionParam::new(&c).err(), Some(ConfigError::OutOfRange("pmax")));
	}

	#[test]
	fn param_new_rejects_nan_and_infinity() {
		let mut c = config();
		c.ke = f32::NAN;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::OutOfRange("ke")));

		let mut c = config();
		c.vdcmax = f32::NAN;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::NotPositive("vdcmax")));

		let mut c = config();
		c.wki = f32::INFINITY;
		assert_eq!(VelocityParam::new(&c).err(), Some(ConfigError::OutOfRange("wki")));

		let mut c = config();
		c.pkp = f32::NAN;
		assert_eq!(PositionParam::new(&c).err(), Some(ConfigError::OutOfRange("pkp")));
	}

	// ---- 母線電圧 ----

	#[test]
	fn current_param_new_rejects_vdc_min_too_low() {
		// vdc_inv = V_b / vdc が8未満に収まるには vdcmin > vdcmax/8 が必要
		let mut c = config();
		c.vdcmin = 3.0;  // ちょうど 24/8
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::VdcMinTooLow));

		let mut c = config();
		c.vdcmin = 2.0;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::VdcMinTooLow));

		let mut c = config();
		c.vdcmin = 0.0;
		assert_eq!(CurrentParam::new(&c).err(), Some(ConfigError::NotPositive("vdcmin")));
	}

	#[test]
	fn update_vdc_sets_voltage_and_inverse() {
		let p = CurrentParam::new(&config()).unwrap();
		let mut m = meas(0.0, 0.0, 0.0);
		assert!(m.update_vdc(&p, q(0.75)));
		assert_close(m.vdc, 0.75);
		assert_close(m.vdc_inv, 1.3333334);
	}

	#[test]
	fn update_vdc_skips_below_minimum() {
		let p = CurrentParam::new(&config()).unwrap();
		let mut m = meas(0.0, 0.0, 0.0);
		// 下限0.5pu未満なら、前回の値(0.5, 2.0)を保つ
		assert!(!m.update_vdc(&p, q(0.25)));
		assert_close(m.vdc, 0.5);
		assert_close(m.vdc_inv, 2.0);
	}

	#[test]
	fn update_vdc_at_lowest_allowed_minimum_stays_in_range() {
		// 下限を許される限界近くまで下げても、逆数が範囲内に収まる
		let mut c = config();
		c.vdcmin = 3.1;  // 24/8 = 3.0 をわずかに上回る
		let p = CurrentParam::new(&c).unwrap();
		let mut m = meas(0.0, 0.0, 0.0);
		let vdc_min = p.vdc_min;
		assert!(m.update_vdc(&p, vdc_min));
		assert!(m.vdc_inv < Q3_28::MAX);
		// 8に近い値なので、丸めの分だけ許容誤差を広げる
		assert!((m.vdc_inv.to_f32() - 24.0 / 3.1).abs() < 1e-4);
	}
```

- [ ] **Step 2: テストが失敗することを確認する**

Run: `cargo test -p md-core`
Expected: コンパイルエラー（`no function or associated item named `new` found for struct `CurrentParam``、`struct `CurrentParam` has no field named `vdc_min``、`Config` のフィールド不一致など）

- [ ] **Step 3: Config を物理単位の f32 に置き換える**

`md-core/src/controller.rs` の先頭の `use crate::scalar::Scalar;` の行を削除する。

`// TODO: バリデート` のコメントと `pub struct Config { ... }` のブロック全体を、次に置き換える:

```rust
/// 設定値。物理単位で持ち、各Param::newでper-unitに変換する
pub struct Config {
	pub vdcmax: f32,  // [V] Vdcの最大値。電圧の基準値を兼ねる
	pub vdcmin: f32,  // [V] これ未満では母線電圧を更新しない。vdcmax/8より大きいこと
	pub ibase: f32,  // [A] 電流の基準値(測定フルスケール)
	pub wbase: f32,  // [rad/s] 速度の基準値(最高速度程度)
	pub ke: f32,  // [V/(rad/s)] 逆起電力

	pub cperiod: f32,  // [s] 電流制御周期
	pub ckp: f32,  // [V/A] 電流P制御
	pub cki: f32,  // [V/(A*s)] 電流I制御
	pub dead_duty: f32,  // デッドタイムによる誤差デューティー比
	pub i_threshold: f32,  // [A] 電流値が[-i_threshold, i_threshold]の間は符号を[-1, 1]に
	pub duty_max: f32,  // シャント抵抗に電流を流したり、ブートストラップするための上限(vmaxとminをとられる)
	pub vmax: f32,  // [V] 出力電圧上限(duty_maxとminをとられる)
	pub imax: f32,  // [A] 目標電流上限(出力が必ずしも超えないとは限らないことに注意！)

	pub wperiod: f32,  // [s] 速度制御周期
	pub wkp: f32,  // [A/(rad/s)] 速度P制御
	pub wki: f32,  // [A/(rad/s*s)] 速度I制御
	pub wb: f32,  // 速度P項の目標値への重み
	pub wmax: f32,  // [rad/s] 目標速度上限

	pub pkp: f32,  // [1/s] 位置P制御
	pub pmax: f32,  // [回転] 目標位置上限
}

/// Configからの変換に失敗した理由。文字列はConfigのフィールド名
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ConfigError {
	/// 正でなければならない値が0以下、またはNaN
	NotPositive(&'static str),
	/// per-unit変換後の値が型の範囲に収まらない、またはNaN
	OutOfRange(&'static str),
	/// vdcminがvdcmax/8以下で、母線電圧の逆数が範囲に収まらない
	VdcMinTooLow,
}

/// 正の有限値ならそのまま返す
fn positive(name: &'static str, v: f32) -> Result<f32, ConfigError> {
	if v > 0.0 && v.is_finite() { Ok(v) } else { Err(ConfigError::NotPositive(name)) }
}

fn to_q(name: &'static str, v: f32) -> Result<Q3_28, ConfigError> {
	Q3_28::checked_from_f32(v).ok_or(ConfigError::OutOfRange(name))
}
```

- [ ] **Step 4: 母線電圧の更新を実装する**

`Measurement` 構造体の直後に追加:

```rust
impl Measurement {
	/// 母線電圧を更新する。下限未満なら何も更新せずfalseを返す
	pub fn update_vdc(&mut self, p: &CurrentParam, vdc: Q3_28) -> bool {
		if vdc < p.vdc_min {
			return false;
		}
		self.vdc = vdc;
		// vdc >= vdc_min > 1/8 なので逆数は8未満に収まる
		self.vdc_inv = Q3_28::ONE.unchecked_div(vdc);
		true
	}
}
```

`CurrentParam` 構造体の `imax: Q3_28,` の次の行にフィールドを追加:

```rust
	vdc_min: Q3_28,  // これ未満では母線電圧を更新しない。1/8より大きい
```

- [ ] **Step 5: 各 Param::new を実装する**

`CurrentParam` 構造体の直後に追加:

```rust
impl CurrentParam {
	pub fn new(c: &Config) -> Result<CurrentParam, ConfigError> {
		let vb = positive("vdcmax", c.vdcmax)?;
		let vmin = positive("vdcmin", c.vdcmin)?;
		let vdc_min = to_q("vdcmin", vmin / vb)?;
		// 変換後の値で判定する。1/8ちょうどだと逆数が8になり範囲を外れる
		if vdc_min <= to_q("vdcmin", 0.125)? {
			return Err(ConfigError::VdcMinTooLow);
		}
		let ib = positive("ibase", c.ibase)?;
		let ob = positive("wbase", c.wbase)?;
		let t = positive("cperiod", c.cperiod)?;
		let ckp = positive("ckp", c.ckp)?;
		let ith = positive("i_threshold", c.i_threshold)?;

		Ok(CurrentParam {
			kp: to_q("ckp", ckp * ib / vb)?,
			ki: to_q("cki", c.cki * t * ib / vb)?,
			kb: to_q("cki", c.cki * t / ckp)?,
			ke: to_q("ke", c.ke * ob / vb)?,
			dead_duty: to_q("dead_duty", c.dead_duty)?,
			i_threshold: to_q("i_threshold", ith / ib)?,
			duty_max: to_q("duty_max", c.duty_max)?,
			vmax: to_q("vmax", c.vmax / vb)?,
			imax: to_q("imax", c.imax / ib)?,
			vdc_min,
		})
	}
}
```

`VelocityParam` 構造体の直後に追加:

```rust
impl VelocityParam {
	pub fn new(c: &Config) -> Result<VelocityParam, ConfigError> {
		let ib = positive("ibase", c.ibase)?;
		let ob = positive("wbase", c.wbase)?;
		let t = positive("wperiod", c.wperiod)?;

		Ok(VelocityParam {
			kp: to_q("wkp", c.wkp * ob / ib)?,
			ki: to_q("wki", c.wki * t * ob / ib)?,
			b: to_q("wb", c.wb)?,
			wmax: to_q("wmax", c.wmax / ob)?,
		})
	}
}
```

`PositionParam` 構造体の直後に追加:

```rust
impl PositionParam {
	pub fn new(c: &Config) -> Result<PositionParam, ConfigError> {
		let ob = positive("wbase", c.wbase)?;

		Ok(PositionParam {
			// 位置は回転単位なので、rad/sへ直す2πが入る
			kp: to_q("pkp", c.pkp * core::f32::consts::TAU / ob)?,
			pmax: Q16_16::checked_from_f32(c.pmax).ok_or(ConfigError::OutOfRange("pmax"))?,
		})
	}
}
```

- [ ] **Step 6: 両方の backend でテストが通ることを確認する**

Run: `cargo test -p md-core`
Expected: `test result: ok.`

Run: `cargo test -p md-core --features f32`
Expected: `test result: ok.`

- [ ] **Step 7: コミット**

```powershell
git add md-core/src/controller.rs
git commit -m "Hold Config in physical units and convert to per-unit in Param::new"
```

---

### Task 5: Scalar を削除し、ドキュメントを追加して最終確認する

**Files:**
- Delete: `md-core/src/scalar.rs`
- Modify: `md-core/src/lib.rs`
- Create: `docs/数値表現.md`

**Interfaces:**
- Consumes: Task 1〜4 の成果すべて
- Produces: `md_core::scalar` モジュールは存在しなくなる

- [ ] **Step 1: Scalar が md-core 内で使われていないことを確認する**

Run: `git grep -n "scalar" -- md-core/src`
Expected: `md-core/src/lib.rs` の `pub mod scalar;` と `md-core/src/scalar.rs` 自身だけがヒットする。`controller.rs` や `fixed` 以下がヒットした場合は、その参照を先に取り除く。

- [ ] **Step 2: Scalar を削除する**

```powershell
git rm md-core/src/scalar.rs
```

`md-core/src/lib.rs` を次の内容にする:

```rust
#![no_std]

pub mod controller;
pub mod fixed;
```

- [ ] **Step 3: ドキュメントを追加する**

**`docs/制御.md` は編集しない。** `docs/数値表現.md` を新規作成:

```markdown
# 数値表現

md-core の制御計算は、既定では固定小数点で行う。対象の STM32F103（Cortex-M3）には FPU がなく、f32 がソフトウェア演算になるため。
feature `f32` を有効にすると、同じ API のまま中身が浮動小数点になる（PC でのシミュレーションや FPU 付きマイコン用）。

## per-unit

信号とゲインは基準値で割って無次元化（per-unit 化）してから計算する。

| 量 | 基準値 | Config のフィールド |
|---|---|---|
| 電圧 | V_b | `vdcmax` |
| 電流 | I_b | `ibase` |
| 速度 | ω_b | `wbase` |
| トルク | Kt·I_b | （電流の per-unit 値と同じになるので持たない） |
| 位置 | 1 回転 = 1.0 | （基準値なし） |

`Config` は物理単位（V、A、rad/s、秒、回転）の f32 で持つ。per-unit への変換、周期 T の掛け込みは各 `Param::new` が行い、値が型に収まらなければ `ConfigError` を返す。

`Measurement` と各制御器の目標値は per-unit で渡す。ADC の生値や CAN の物理値からの変換はファーム側の仕事。

## 型の使い分け

| 型 | 中身（既定 / feature `f32`） | 範囲 | 用途 |
|---|---|---|---|
| `Q3_28` | i32 / f32 | [-8, 8) | per-unit の信号とゲイン |
| `Q3_60` | i64 / f64 | [-8, 8) | 積分器 |
| `Q16_16` | i32 / f64 | [-32768, 32768) | 位置（回転） |

- 信号は ±1 前後になるが、誤差（最大 ±2）や和、1 を超えるゲインのために整数部を 3bit 持たせている。
- 積分器は、1 周期ぶんの増分（ゲイン × 誤差）が Q3.28 の分解能より小さくなりうる。積を丸めずに Q3.60 で積算することで、小さな偏差でも積分が進む。
- 位置は多回転を扱うので整数部を広く取る。位置誤差にゲインを掛けるときは、掛けてから Q3.28 にする（先に変換すると 8 回転で飽和する）。

## 演算の約束

- 加減乗算と符号反転、`abs` はあふれたら型の上限・下限に飽和する。ラップして符号が反転することはない。これは安全策であり、通常動作で飽和に達しないよう基準値とゲインを選ぶ。
- 除算は `checked_div`（範囲外・0 除算で `None`）と `unchecked_div`（範囲内であることを呼び出し側が保証。debug ビルドでは違反を検出）だけ。逆数や除算は割る数が小さいだけで範囲を外れるので、値の範囲を知っている側が使い分ける。
  - 母線電圧の逆数：`vdcmin > vdcmax/8` を `CurrentParam::new` で検証し、下限未満の測定値では更新しないので、逆数は必ず範囲内に収まる。
  - デッドタイム補償：|i| がしきい値未満のときだけ `i / しきい値` を計算するので、結果は (-1, 1) に収まる。
- 型をまたぐ変換は明示的な関数（`to_q3_28` など）だけで、暗黙の変換はない。
```

- [ ] **Step 4: 全体を確認する**

Run: `cargo test -p md-core`
Expected: `test result: ok.`、failed は 0

Run: `cargo test -p md-core --features f32`
Expected: `test result: ok.`、failed は 0

Run: `cargo build -p md-core --target thumbv7m-none-eabi`
Expected: `Finished`。warning が出ていないこと

Run: `cargo build -p md-core --target thumbv7m-none-eabi --features f32`
Expected: `Finished`。warning が出ていないこと

Run: `git status --short docs/制御.md`
Expected: 何も出力されない（未変更）

- [ ] **Step 5: コミット**

```powershell
git add md-core/src/lib.rs docs/数値表現.md
git commit -m "Remove Scalar and document number representation"
```

---

## 範囲外

- `minisirasu-firm` の修正（`md_core::pwm` と `Scalar` の参照が残っており、本計画の前からビルドできない）
- ADC 生値 → per-unit、CAN 物理値 → per-unit の変換の実装
- トルク FF、ノッチフィルタ、速度オブザーバ、軌道生成
