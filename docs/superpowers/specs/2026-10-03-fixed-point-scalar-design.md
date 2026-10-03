# md-core 固定小数点化 設計書

日付: 2026-10-03
対象: `md-core/`（計算ロジック）。`minisirasu-firm/` はビルド確認と呼び出し側の変換のみ。

## 目的

STM32F103（Cortex-M3、FPUなし）では f32 がソフトウェア演算になるため、制御計算を固定小数点で行えるようにする。
feature フラグ `f32` で浮動小数点版にも切り替えられるようにし、PCでのシミュレーションやFPU付きマイコンでも使えるようにする。

## 方針の要約

- 単一の `Scalar` をやめ、値の意味に応じて型（フォーマット）を使い分ける。
- 信号とゲインは per-unit 化し、和差の余裕を持たせた Q3.28 で扱う。
- 積分器は微小な増分が丸めで消えないよう Q3.60（i64）で扱う。
- 位置は多回転を扱うため Q16.16（1回転 = 1.0）で扱う。
- Q 型はフォーマットだけで振る舞いが決まる汎用演算のみを持つ。値の意味に依存する処理（積分器、デッドタイム補償、逆数の範囲保証）は controller 側に置く。
- 外部依存は増やさず、newtype を自前で実装する。

## 1. 型と演算

### 型

| 型 | 既定（固定小数点） | feature `f32` | 用途 |
|---|---|---|---|
| `Q3_28` | i32, Q3.28（範囲 ±8, 分解能 約3.7e-9） | f32 | per-unit の信号（電圧・電流・速度・duty・誤差）とゲイン |
| `Q3_60` | i64, Q3.60 | f64 | 積分器 |
| `Q16_16` | i32, Q16.16（範囲 ±32768, 分解能 約1.5e-5） | f64 | 位置（1回転 = 1.0） |

feature `f32` で `Q3_60` と `Q16_16` を f64 にするのは、f32 の仮数部24bitでは積分器の微小増分や多回転位置の分解能が固定小数点版より粗くなるため。

### 演算

各型が持つ演算。f32 版も同じ API を持ち、演算結果を固定小数点版と同じ範囲（`Q3_28` なら ±8）に飽和させる。これにより両版で飽和の振る舞いが一致し、同じテストが通る:

| 演算 | 対象 | オーバーフロー時 |
|---|---|---|
| `+` `-` `neg` | 全型（同型同士） | 飽和 |
| `*` | `Q3_28 × Q3_28 → Q3_28`（i64で計算し28bit右シフト） | 飽和 |
| `min` `max` `abs` | 全型 | `abs(MIN)` は MAX に飽和 |
| 拡大乗算 `widening_mul` | `Q3_28 × Q3_28 → Q3_60`（丸めなし。積Q6.56を4bit左シフト） | 飽和 |
| 混合乗算 | `Q16_16 × Q3_28 → Q3_28` | 飽和 |
| フォーマット変換 | `Q3_60 → Q3_28`、`Q3_28 → Q3_60`、`Q16_16 ⇄ Q3_28` | 狭める方向は飽和 |
| `checked_div(self, rhs) -> Option<Self>` | `Q3_28` | 範囲外・0除算で `None` |
| `unchecked_div(self, rhs) -> Self` | `Q3_28` | 呼び出し側が範囲を保証。debug ビルドでは `debug_assert!` で0除算・範囲外を検出。release では検査せず結果は不定（未定義動作ではない） |
| 定数 | `ZERO` `ONE` `MAX` `MIN` | |
| `f32` との変換 | `checked_from_f32(f32) -> Option<Self>` / `to_f32`（設定時・テスト用） | 範囲外・NaN で `None` |

f32 版の `checked_div` / `unchecked_div` / `checked_from_f32` も、固定小数点版と同じ範囲（`Q3_28` なら ±8）で範囲外を判定する。これにより両版で同じ入力が同じ成否になる。

- 飽和は「暴走防止の安全策」であり、通常動作でここに達しないよう per-unit 化と Q3.28 の余裕で設計する。
- 型をまたぐ暗黙の `From` は作らない。変換は明示的な関数のみ。
- `recip` は置かない。逆数は `ONE.unchecked_div(x)` または `checked_div` で書く。
- `sat(threshold_inv)` は Q 型から削除し、controller 側の `soft_sign` に置き換える。

## 2. 単位、基準値、Config / Param / Measurement

### per-unit 基準値

| 基準値 | 意味 | 由来 |
|---|---|---|
| V_b | 電圧 | `Config.vdcmax` |
| I_b | 電流 | `Config.ibase`（電流測定フルスケール） |
| ω_b | 速度 | `Config.wbase`（最高速度程度） |
| T_b | トルク | Kt·I_b と定義。トルク pu = 電流 pu となり、`/Kt` は pu 上で恒等変換（ゲインに吸収） |
| 位置 | 1回転 = 1.0 | 基準値なし |

### Config（物理単位、f32、フィールドは pub）

| フィールド | 単位 | 備考 |
|---|---|---|
| `vdcmax` | V | V_b を兼ねる |
| `vdcmin` | V | 新規。これ未満では母線電圧を更新しない |
| `ibase` | A | 新規 |
| `wbase` | rad/s | 新規 |
| `ke` | V/(rad/s) | |
| `cperiod` | s | 電流制御周期 |
| `ckp` | V/A | |
| `cki` | V/(A·s) | T は掛けない |
| `dead_duty` | - | |
| `i_threshold` | A | 逆数をやめて物理値で持つ |
| `duty_max` | - | |
| `vmax` | V | |
| `imax` | A | |
| `wperiod` | s | 新規。速度制御周期 |
| `wkp` | A/(rad/s) | |
| `wki` | A/(rad/s·s) | T は掛けない |
| `wb` | - | |
| `wmax` | rad/s | |
| `pkp` | 1/s | |
| `pmax` | 回転 | |

`kb` は `cki·cperiod/ckp` で求まるため Config から削除する。

### Param::new(&Config) -> Result<Param, ConfigError>

起動時・設定変更時のみ呼ばれる。f32 で計算し、各値を固定小数点へ変換する。

| Param の値 | 計算式 | 型 |
|---|---|---|
| CurrentParam.kp | ckp·I_b/V_b | Q3_28 |
| CurrentParam.ki | cki·cperiod·I_b/V_b | Q3_28 |
| CurrentParam.kb | cki·cperiod/ckp | Q3_28 |
| CurrentParam.ke | ke·ω_b/V_b | Q3_28 |
| CurrentParam.dead_duty | dead_duty | Q3_28 |
| CurrentParam.i_threshold | i_threshold/I_b | Q3_28 |
| CurrentParam.duty_max | duty_max | Q3_28 |
| CurrentParam.vmax | vmax/V_b | Q3_28 |
| CurrentParam.imax | imax/I_b | Q3_28 |
| VelocityParam.kp | wkp·ω_b/I_b | Q3_28 |
| VelocityParam.ki | wki·wperiod·ω_b/I_b | Q3_28 |
| VelocityParam.b | wb | Q3_28 |
| VelocityParam.wmax | wmax/ω_b | Q3_28 |
| PositionParam.kp | pkp·2π/ω_b | Q3_28 |
| PositionParam.pmax | pmax | Q16_16 |
| （Measurement 用）vdc_min | vdcmin/V_b | Q3_28 |

`ConfigError` を返す条件（どのフィールドかを示す）:

- 変換後の値が型の範囲に収まらない
- `vdcmin < vdcmax/8`（vdc_inv が Q3.28 に収まらない）
- `i_threshold <= 0`、`ckp <= 0`、各周期 `<= 0`、基準値 `<= 0`

Param の型は pub、フィールドは非公開のまま（RTICX の shared に置く想定）。

### Measurement（per-unit 固定小数点）

| フィールド | 型 |
|---|---|
| `vdc` | Q3_28 |
| `vdc_inv` | Q3_28 |
| `i` | Q3_28 |
| `w` | Q3_28 |
| `th` | Q16_16 |

- ADC 生値 → pu の変換はボード依存（シャント抵抗、アンプゲイン）なので firm 側で行う。
- 母線電圧の更新は md-core の関数で行う: `vdc < vdc_min` なら `vdc` と `vdc_inv` を更新しない。それ以外は `vdc_inv = ONE.unchecked_div(vdc)`（下限が保証されているため範囲内）。

### 目標値

各制御器は pu の目標値を受け取る。CAN の物理値 → pu の変換は firm 側で行う。

## 3. 制御器の計算

### Integrator（controller 内、非公開）

```rust
struct Integrator(Q3_60);
impl Integrator {
    fn add_product(&mut self, k: Q3_28, e: Q3_28); // self += widening_mul(k, e)（飽和）
    fn value(&self) -> Q3_28;                       // Q3_28 へ変換（飽和）
    fn reset(&mut self);
}
```

### 電流制御 `CurrentState::update(&mut self, p, u: Q3_28, m) -> (Q3_28, Saturated)`

1. `u_clamped = clamp(u, ±imax)`、`e = u_clamped - i`
2. `v1 = kp*e + i_sum.value() + ke*w`
3. `vlim = min(vmax, duty_max*vdc)`、`v2 = clamp(v1, ±vlim)`
4. `i_sum.add_product(ki, e)`、`i_sum.add_product(kb, v2 - v1)`
5. `duty = clamp(v2*vdc_inv + dead_duty*soft_sign(i, i_threshold), ±1)`
6. Saturated 判定は現状通り（`u` が imax を超える、または v1 ≠ v2）

`soft_sign(i, th)`: `i >= th` なら `ONE`、`i <= -th` なら `-ONE`、それ以外は `i.unchecked_div(th)`（|i| < th かつ th > 0 なので結果は (-1, 1)）。

Q 型演算自体の飽和は Saturated に含めない。

### 速度制御 `VelocityState::update(&mut self, p, u: Q3_28, m, last_saturated) -> Q3_28`

1. `u_clamped = clamp(u, ±wmax)`、`e = u_clamped - w`
2. 出力 `kp*(b*u_clamped - w) + w_sum.value()`
3. 条件付き積分は現状通り（Overflow かつ e > 0、Underflow かつ e <= 0 のとき停止）。積分は `w_sum.add_product(ki, e)`

### 位置制御 `PositionState::update(&self, p, u: Q16_16, m) -> Q3_28`

1. `u_clamped = clamp(u, ±pmax)`
2. `err = u_clamped - th`（Q16_16）
3. 出力 = 混合乗算 `err × kp`（Q3_28、飽和）

ゲインを掛けてから Q3_28 にすることで、8回転を超える誤差でも速度指令が弱まらない。

## 4. feature フラグ

- `md-core/Cargo.toml` に `[features] f32 = []` を追加。既定は固定小数点。
- `cfg(feature = "f32")` で各型の内部表現と演算実装を切り替える。公開APIは両方で同一。

## 5. ファイル構成

- `md-core/src/fixed.rs`: `Q3_28`、`Q3_60`、`Q16_16` と演算。現行の `scalar.rs` は置き換えて削除。
- `md-core/src/controller.rs`: Config / Param / Measurement / 各制御器 / Integrator / soft_sign。
- `docs/数値表現.md`（新規）: per-unit 基準値と各型の使い分けの説明。**`docs/制御.md` は編集しない。**

## 6. テスト

- Q 型: 各演算の通常値と飽和境界、`checked_div` の `None`、`unchecked_div` の `debug_assert`（`#[should_panic]`、debug ビルド時）、拡大乗算の無丸め、フォーマット変換。
- controller: 現行 52 件のテストの意図を引き継ぎ、値を pu に置き換える。入力は補助関数で作り、許容誤差付き比較にして固定小数点版と f32 版の両方で同じテストが通るようにする。
- Param::new: 変換式の検証、`ConfigError` の各ケース。
- 実行: `cargo test`、`cargo test --features f32`、`minisirasu-firm` の thumbv7m ビルド。

## 範囲外（今回やらないこと）

- トルクFF、ノッチフィルタ、速度オブザーバ、軌道生成（未実装のまま）
- ADC 生値 → pu 変換、CAN 物理値 → pu 変換の firm 側実装（呼び出し側の対応は別タスク）
- 固定小数点版と f32 版の結果を同一ビルド内で比較するテスト
