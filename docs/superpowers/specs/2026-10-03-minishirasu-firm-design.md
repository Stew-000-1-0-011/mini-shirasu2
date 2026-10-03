# minishirasu-firm 設計

日付: 2026-10-03
対象: `minishirasu-firm/`（ファーム本体）と `md-core/`（最小限の追加）

## 目的

mini-shirasu 基板（STM32F103C8T6）で、上位から受けた目標値に従ってブラシ付き DC モーターを電流・速度・位置制御するファームを作る。制御計算は md-core、タスク管理は RTICX を使う。まず実機で3モードが動くところまでを目標とする。

## 方針

- md-core は薄く保つ。制御の計算と数値型だけを持つ。md-core を使って別の制御器構成を組みたい人が、余計なものを引き受けずに済むようにする。
- それ以外（プロトコル、状態機械、カスケードの合成、PWM・ADC の換算）は minishirasu-firm に置く。
- minishirasu-firm のうちレジスタに触らない部分はライブラリ部にまとめ、ホストでテストできるようにする。将来、マイコン非依存の汎用ファームウェアテンプレートを別クレートに切り出すとき、ファイルを移すだけで済む形にする。

## 今回やらないこと

通信タイムアウト、ソフト側の過電流・電圧・温度保護、速度オブザーバ、軌道生成、ノッチフィルタ、負荷モデル FF、加振生成、Flash への設定保存、UART トランスポート、WS2812、ボタン。

## 基板

回路図（`minishirasu-firm/board/mini_shirasu.sch`）のマイコンのシンボルは STM32F302/303 だが、実装されているのは STM32F103C8T6（Flash 64K、RAM 20K、FPU なし）。

| 機能 | ピン | 備考 |
|---|---|---|
| PWMA / PWMB | PA9 (TIM1_CH2) / PA8 (TIM1_CH1) | ゲートドライバ MPQ6528 の入力。H ブリッジ |
| EN (`!EMS`) | PA4 | High で出力有効 |
| NSLEEP | PA2 | High でドライバ起動 |
| NFAULT | PA3 | Low で異常。入力プルアップ |
| エンコーダ A / B | PA0 / PA1 | TIM2_CH1 / CH2 |
| ISENSEB | PB1 (ADC ch9) | ローサイド共通シャント 2mΩ、INA2181A1（20倍）、基準 約 0.118V |
| VSENSE | PB0 (ADC ch8) | 10k / 1k 分圧（11倍） |
| TEMP | PA6 (ADC ch6) | NTC 10k と 3.3k の分圧 |
| CAN RX / TX | PB8 / PB9 | AFIO の CAN リマップが必要 |
| 水晶 | 12MHz | PLL ×6 で 72MHz。APB1 は 36MHz |

シャントは両ローサイド FET のソースと GND の間に1本だけある。還流中（両ローサイド ON）は電流がシャントを通らないので、片側のハイサイドが ON の期間にしか電流を測れない。

## クレートの分担

### md-core への追加

追加は次の2点だけ。既存の `Config` と制御器は変えない。空の `velocity_observer.rs` と `profile_generator.rs` も触らない。

**1. `encoder` モジュール（差分カウント → 位置・速度）**

- `EncoderParam::new(counts_per_rev, reversed, period, wbase, alpha)` で検証と per-unit 変換を行う。失敗は既存の `ConfigError` で返す。
  - `counts_per_rev`: 1回転あたりのカウント数（4逓倍後）。0 は `NotPositive`。
  - `alpha`: 速度の1次ローパスの係数。範囲は (0, 1]、1 でフィルタなし。
  - 速度の換算係数 `2π / (counts_per_rev × period × wbase)` が `Q3_28` に収まらなければ `OutOfRange`。
- `EncoderState::update(&param, delta: i32)` は符号付きの差分カウントを受け取り、位置（`Q16_16`、回転単位）と速度（`Q3_28`、per-unit）を返す。
  - タイマのビット幅に依存するラップ込みの減算は呼び出し側で済ませる。
  - `reversed` が true なら差分の符号を反転する。
  - カウントの累積は飽和加算。位置は `Q16_16` の範囲（±32768 回転）で飽和する。
  - 速度は `w += alpha × (delta × 係数 − w)`。
- `EncoderState::set_origin()` で累積カウントを 0 にする。速度はそのまま。

**2. 固定小数点型と整数の境界の演算**

型の中身が非公開で、固定小数点版（`fixed/int.rs`）と浮動小数点版（`fixed/float.rs`）の2実装があるため、md-core に置くしかない。両方に同じ API で足す。

- `Q3_28::scale_int(self, n: i32) -> i32`: per-unit 値 × 整数を、最も近い整数に丸めて返す。あふれは飽和。デューティ × ARR → コンペア値に使う。
- `Q3_28::mul_int(self, n: i32) -> Q3_28`: per-unit のゲイン × 整数。あふれは飽和。ADC 生値 × 換算係数 → 電流・電圧、差分カウント × 係数 → 速度に使う。
- `Q16_16::from_ratio(num: i32, den: u32) -> Q16_16`: `num / den`。あふれは飽和、`den` が 0 のときの扱いは呼び出し側（`EncoderParam::new`）が弾く前提で、debug ビルドでは検出する。

### minishirasu-firm

**ライブラリ部**（レジスタに触らない。ホストでテストする）

| ファイル | 中身 |
|---|---|
| `src/lib.rs` | モジュール宣言。`#![cfg_attr(not(test), no_std)]` |
| `src/config.rs` | 設定値の構造体、既定値、`SetParam` の ID 対応、検証 |
| `src/protocol.rs` | メッセージ定義、符号化・復号、ストリームパーサ |
| `src/state.rs` | 状態機械（モードと遷移） |
| `src/cascade.rs` | 外側ループの合成 |
| `src/pwm.rs` | 符号付きデューティ → 2本のコンペア値 |
| `src/sense.rs` | ADC 生値 → per-unit、電流の符号の復元 |

**バイナリ部**

| ファイル | 中身 |
|---|---|
| `src/main.rs` | RTICX の `app`。共有リソースとタスクの定義、defmt、パニックハンドラ |
| `src/board.rs` | クロック、GPIO、TIM1、TIM2、ADC1、ドライバ IC の初期化とレジスタ操作 |
| `src/can.rs` | bxcan の初期化、ID ⇔ ストリームの対応、送信リングバッファ |

**依存**

- ライブラリ部: `md-core`、`cobs`、`crc`。
- バイナリ部だけが使うもの（`cortex-m`、`cortex-m-rt`、`stm32f1`、`bxcan`、`rticx-cortex-m`、`defmt`、`defmt-rtt`、`panic-probe`）は `[target.'cfg(target_os = "none")'.dependencies]` に置き、ホストでのテスト時にビルドされないようにする。
- `stm32f1xx-hal` は外す。PAC が2バージョン入るのを避けるため。`bxcan::Instance` などの実装は自前で書く。
- `rticx-cortex-m` は `swtasks` feature を有効にする。
- `defmt-test`、`semihosting` と、実機上テストの枠（`harness = false`、`tests/integration.rs`）は外す。
- `[profile]` はワークスペースルートの `Cargo.toml` のものを使う。

レジスタは `stm32f1` PAC を直接操作する。

## タスク構成

PWM は 20kHz。電流ループは毎 PWM 周期（50µs）、速度・位置ループは 1kHz（20周期に1回）。

| タスク | 種類 / 起動元 | 優先度 | 仕事 |
|---|---|---|---|
| `CurrentLoop` | ADC1_2 割り込み | 4 | ADC 読み出し、電流の符号復元、NFAULT 確認、電流制御、CCR 書き込み。20回に1回、エンコーダの `CNT` と VSENSE・TEMP の生値を取って `OuterLoop` を起動 |
| `OuterLoop` | ソフトウェアタスク | 3 | エンコーダ更新、母線電圧の更新、カスケードの計算、`telemetry` の更新。状態送信の周期ごとに `Report` を起動 |
| `CanRx` | USB_LP_CAN_RX0 割り込み | 2 | 受信フレームを ID でストリームに振り分けてパース。目標値は `setpoint` に書く。コマンドは `Command` を起動して渡す |
| `CanTx` | USB_HP_CAN_TX 割り込み | 2 | 送信リングバッファから最大 8 バイトずつメールボックスへ |
| `Command` | ソフトウェアタスク（容量 4） | 1 | モード切替、設定変更、異常リセット、原点設定。応答を送信バッファに積む |
| `Report` | ソフトウェアタスク | 1 | `Status` を作って送信バッファに積む |
| idle | — | 0 | `wfi` のみ |

ソフトウェアタスクの優先度は2種類なので、ディスパッチャは2本。未使用の割り込み（SPI1、SPI2）を割り当てる。

### 共有リソース

| 名前 | 中身 | 書く側 | 読む側 |
|---|---|---|---|
| `setpoint` | 目標値、速度 FF、トルク FF（すべて per-unit） | `CanRx`、`Command`（モード切替時の初期化） | `OuterLoop` |
| `link` | 電流目標、出力の可否 / 飽和フラグ、測定電流 | `OuterLoop`、`Command` / `CurrentLoop` | `CurrentLoop` / `OuterLoop` |
| `mode` | 状態機械 | `Command`、`CurrentLoop`（異常時） | 全タスク |
| `params` | `CurrentParam`、`VelocityParam`、`PositionParam`、`EncoderParam`、換算係数、設定値の原本（f32） | `Command`（出力無効中のみ） | 全タスク |
| `telemetry` | 電流、速度、位置、母線電圧、温度、フラグ | `OuterLoop` | `Report` |
| `tx` | 状態用・応答用の送信リングバッファ（各 64 バイト） | `Command`、`Report`、`CurrentLoop`（異常通知の要求フラグのみ） | `CanTx` |

- 積分器などの制御器の状態は各タスクのローカルに持つ。モード切替時のリセットは、`link` と `setpoint` に置いた世代番号が変わったことをループ側が見て自分で行う。
- 電流モードでも `OuterLoop` が目標値をそのまま電流目標として `link` に渡す。`CurrentLoop` は常に `link` だけを見る。

### データの流れ

```
CAN RX ─▶ CanRx ──(目標値)──▶ [setpoint] ──▶ OuterLoop ──▶ [link: 電流目標] ──▶ CurrentLoop ─▶ PWM
            │                                    ▲  ▲                                │
            └─(コマンド)─▶ Command ─▶ [params] ──┘  └──[link: 飽和フラグ・電流]──────┘
                              │
CAN TX ◀─ CanTx ◀─ [tx] ◀─────┴──── Report ◀── [telemetry] ◀── OuterLoop
```

## 状態機械

```
            SetMode(電流/速度/位置)
 Disabled ───────────────────────▶ Current / Velocity / Position
    ▲   ◀─────────────────────────        │（相互にも切替可）
    │        SetMode(Disabled)             │ NFAULT = Low
    │                                      ▼
    └────────── ResetFault ──────────── Fault（ラッチ）
```

- 起動直後は `Disabled`。EN = Low、コンペア値 0。
- `Disabled` から有効化するとき、設定値の原本から各 `Param::new` を作り直す。失敗したら `Disabled` のまま `Nack` を返す。
- 有効なモードに入るとき（`Disabled` から、または別のモードから）、積分器をリセットし、目標値を初期化する。電流は 0、速度は 0、位置は現在位置、FF は 0。
- `CurrentLoop` は毎周期 NFAULT を見る。Low ならその場で EN を Low、コンペア値を 0 にして `Fault` にする。`FaultNotice` を1回送る。
- `Fault` 中の `SetMode` は `Nack`。
- `ResetFault` は、NSLEEP を 1ms Low にしてドライバを再起動したあと NFAULT が High なら `Disabled` に戻す。Low のままなら `Nack`。
- 通信が途絶えても出力は止めない（今回の範囲外）。

## 制御

### カスケード

| モード | 目標値 | 一緒に届く FF |
|---|---|---|
| 電流 | 電流 | なし |
| 速度 | 速度 | 加速度 FF |
| 位置 | 位置 | 速度 FF、加速度 FF |

```
位置P(θ目標) ─▶ (+) ─▶ 速度PI ─▶ (+) ─▶ 電流PI ─▶ デューティ
                 ▲                ▲
              速度FF          トルクFF
```

- 加速度 FF は外乱を含まない慣性分。受信時に `accel_to_current`（= J/Kt、[A/(rad/s²)]）を掛けて電流に直し、per-unit にして `setpoint` に入れる。per-unit ではトルクと電流が同じ値になるので、これがトルク FF になる。
- `cascade` は md-core の `PositionState`、`VelocityState` を呼び、FF を足して電流目標を返す。速度 PI には `CurrentLoop` が報告した飽和フラグを渡す。
- 目標値の制限（`imax`、`wmax`、`pmax`）は md-core の各制御器が行う。FF の加算は飽和加算。

### PWM（TIM1）

- 72MHz、プリスケーラなし、`ARR = 1800`、センターアラインモード1、PWM モード1。出力は `CNT < CCR` の間 High で、ON パルスは `CNT = 0`（谷）を中心に広がる。
- 符号付きデューティ `d` は、片側だけ PWM、反対側は Low 固定で出す。`d > 0` なら PWMA（CH2）= `d × ARR`、PWMB（CH1）= 0。`d < 0` はその逆。
- CCR はプリロード有効。割り込みで書いた値は次の更新イベントで反映される。

### ADC を ON 期間の中央で起動する方法

- TIM1 の更新イベントを TRGO に出し、ADC1 のインジェクテッド変換の外部トリガ（`JEXTSEL = TIM1_TRGO`）にする。
- `RCR = 1` にして更新イベントを1周期に1回にする。RCR が奇数のとき、カウンタ起動後に RCR を書くと谷で、起動前に書くと山で更新イベントが出る。**カウンタを起動してから RCR を書き**、谷（ON 中央）に合わせる。
- 起動時、最初の変換完了割り込みで `DIR` ビットを見て、谷（アップカウント中）で取れていることを確認する。違えば defmt でエラーを出し、有効化を受け付けない（`Fault` にする）。
- ADC クロックは 12MHz（PCLK2 / 6）。起動時にキャリブレーションする。
- インジェクテッドのスキャンで ISENSEB → VSENSE → TEMP の順に変換する。サンプル時間は ISENSEB が 7.5 サイクル、VSENSE が 28.5 サイクル、TEMP が 55.5 サイクル。
- 変換完了（JEOC）割り込みが `CurrentLoop`。

### 電流

- 起動時、ドライバを起こしてデューティ 0 のまま 1024 回平均し、オフセットを求める。
- `i = sign(その周期に出していたデューティ) × (ADC − オフセット) × 換算係数`。
- 換算は 2mΩ × 20倍 = 40mV/A。12bit、3.3V で 1LSB あたり 約 0.0201A。
- デューティが 0 だった周期は符号が決まらないので、電流 0 とする。

**既知の制約**: `|d|` が小さいと ON 期間がサンプル時間より短くなり、電流を実際より小さく読む。ISENSEB のサンプル時間を最短にして下限を下げるが、数 % 以下のデューティでは正しく測れない。低速・低トルク域で電流制御が甘くなる可能性がある。今回は対処せず、実機で様子を見る。

### 母線電圧・温度

- VSENSE は 11倍の分圧。`OuterLoop` で per-unit にして `Measurement::update_vdc` を呼ぶ。`false`（母線電圧が `vdcmax / 8` 以下）のときは前回の値を使い続け、状態のフラグに立てる。
- TEMP は生値を状態に載せるだけ。

### エンコーダ（TIM2）

- エンコーダモード3（両エッジ、4逓倍）、`ARR = 0xFFFF`、入力フィルタあり。
- `CurrentLoop` が 20回に1回 `CNT` を読む。ON 中央に同期しているので周期のジッタがない。
- `OuterLoop` が前回値との差を `i16` のラップ込み減算で取り、md-core の `EncoderState::update` に渡す。

### 向き

- 正方向は「正のデューティ（PWMA 側を駆動）で回る向き」に固定する。逆回りを正にしたい場合は、上位側で符号を反転する。
- モーターとエンコーダの向きの食い違いは、設定 `encoder_reversed` で吸収する。
- 食い違ったままだと速度・位置ループが正帰還になる。確認手順: 電流モードで小さい正の電流を流し、状態の速度が正になること。負なら `encoder_reversed` を切り替える。

### 位置の範囲

位置は ±32768 回転で飽和する。速度モードで回し続けたあとに位置モードを使う場合は、先に `SetOrigin` で原点を取り直す。

## プロトコル

### ストリーム

区切りのないバイトストリームを4本使う。プロトコル層はトランスポートを知らない。

| # | 向き | 役割 | メッセージ |
|---|---|---|---|
| 0 | 受信 | 目標値 | `TargetCurrent`、`TargetVelocity`、`TargetPosition` |
| 1 | 受信 | コマンド | `SetMode`、`SetParam`、`ResetFault`、`SetOrigin` |
| 2 | 送信 | 状態 | `Status` |
| 3 | 送信 | 応答 | `Ack`、`Nack`、`FaultNotice` |

### フレーミング

```
COBS( [種別 1B] [ペイロード] [CRC-8 1B] )  0x00
```

- COBS は `cobs` クレート、CRC は `crc` クレートの CRC-8/SMBUS（多項式 0x07、初期値 0）。CRC の対象は種別とペイロード。
- 長さは種別から決まる。長さ不一致、CRC 不一致、未知の種別は捨てる。
- パーサはストリームごとに 32 バイトのバッファを持つ。1バイトずつ `push` し、0x00 を受けた時点で復号して結果を返す。バッファがあふれたら次の 0x00 まで読み捨てる。
- 数値はリトルエンディアン。

### メッセージ

| 種別 | 名前 | ペイロード | 符号化後の長さ |
|---|---|---|---|
| 0x01 | `TargetCurrent` | 電流 [A] f32 | 8 |
| 0x02 | `TargetVelocity` | 速度 [rad/s] f32、加速度 FF [rad/s²] f32 | 12 |
| 0x03 | `TargetPosition` | 位置 [回転] i32（Q16.16）、速度 FF [rad/s] f32、加速度 FF [rad/s²] f32 | 16 |
| 0x10 | `SetMode` | モード u8 | 5 |
| 0x11 | `SetParam` | ID u8、値 f32 | 9 |
| 0x12 | `ResetFault` | なし | 4 |
| 0x13 | `SetOrigin` | なし | 4 |
| 0x20 | `Status` | モード u8、フラグ u8、電流 [A] f32、速度 [rad/s] f32、位置 [回転] i32（Q16.16）、母線電圧 [V] f32、温度 ADC 生値 u16 | 24 |
| 0x30 | `Ack` | コマンドの種別 u8 | 5 |
| 0x31 | `Nack` | コマンドの種別 u8、理由 u8、設定 ID u8 | 7 |
| 0x32 | `FaultNotice` | なし | 4 |

- モード: 0 = 無効、1 = 電流、2 = 速度、3 = 位置。`Status` では 4 = 異常。
- `Status` のフラグ: bit0 = 上側に飽和、bit1 = 下側に飽和、bit2 = 母線電圧が低く更新できていない。
- `Nack` の理由: 1 = フレーム不正（CRC・長さ・未知の種別。コマンドの種別は 0）、2 = 出力有効中、3 = 設定エラー、4 = 異常ラッチ中、5 = 値が不正（未知のモード・未知の設定 ID）、6 = NFAULT が解除されない。設定 ID は理由 3 のときだけ意味を持つ。

### 目標値の扱い

- 物理単位で受け、`CanRx` で per-unit の固定小数点に変換する。範囲外や NaN は捨てる。
- 位置だけ Q16.16 の整数で送る。f32 の仮数は 24bit で、回転数が大きいとエンコーダ1カウントを表せないため。
- 現在のモードと合わない目標値は捨てる。
- 目標値ストリームのフレーム不正は黙って捨てる。`Nack` を返すのはコマンドストリームだけ。

### コマンドの扱い

| コマンド | 受け付ける条件 | 動作 |
|---|---|---|
| `SetMode` | `Fault` 以外 | 状態遷移 |
| `SetParam` | `Disabled` のみ | 設定値の原本を書き換える。検証は次の有効化時 |
| `ResetFault` | いつでも | `Fault` なら解除を試みる。`Fault` でなければ何もせず `Ack` |
| `SetOrigin` | `Disabled` のみ | 現在位置を 0 にする |

### 設定値

`SetParam` の値は f32。真偽値は 0 か非 0、整数は小数点以下を切り捨てて使う。

| ID | 名前 | 単位 | 既定値 |
|---|---|---|---|
| 0x00 | `vdcmax` | V | 36.0 |
| 0x01 | `ibase` | A | 20.0 |
| 0x02 | `wbase` | rad/s | 600.0 |
| 0x03 | `ke` | V/(rad/s) | 0.0 |
| 0x04 | `ckp` | V/A | 0.5 |
| 0x05 | `cki` | V/(A·s) | 500.0 |
| 0x06 | `dead_duty` | — | 0.0 |
| 0x07 | `i_threshold` | A | 0.2 |
| 0x08 | `duty_max` | — | 0.9 |
| 0x09 | `vmax` | V | 24.0 |
| 0x0A | `imax` | A | 2.0 |
| 0x0B | `wkp` | A/(rad/s) | 0.01 |
| 0x0C | `wki` | A/(rad/s·s) | 0.1 |
| 0x0D | `wb` | — | 1.0 |
| 0x0E | `wmax` | rad/s | 100.0 |
| 0x0F | `pkp` | 1/s | 5.0 |
| 0x10 | `pmax` | 回転 | 1000.0 |
| 0x20 | `accel_to_current` | A/(rad/s²) | 0.0 |
| 0x21 | `encoder_cpr` | カウント/回転 | 8192 |
| 0x22 | `w_filter_alpha` | — | 0.2 |
| 0x23 | `encoder_reversed` | 真偽 | false |
| 0x24 | `status_period_ms` | ms | 10（0 で停止） |

- 制御周期（`cperiod` = 50µs、`wperiod` = 1ms）はファームで固定し、変更できない。
- 既定値はモーターの実物に合わせたものではなく、低いゲインと低い電流上限にした仮の値。`encoder_cpr` を含め、使うモーターに合わせて調整する。既定値は `config.rs` の定数で、書き換えて再ビルドすれば初期値になる。

## CAN

- 1Mbps、標準 ID（11bit）、classic CAN。
- ストリームごとに ID を1つ割り当てる。フレームのデータ（1〜8 バイト）をストリームのバイト列としてそのまま連結する。フレーム境界に意味はない。
- ID は `config.rs` のコンパイル時定数。ID が小さいほど優先度が高い。複数台つなぐときは基板ごとに変える。

| ストリーム | 既定の ID |
|---|---|
| 0 目標値 | 0x100 |
| 2 状態 | 0x101 |
| 1 コマンド | 0x200 |
| 3 応答 | 0x201 |

- 受信フィルタは2つの受信 ID だけを通す。
- 送信はストリームごとに 64 バイトのリングバッファ。メッセージが丸ごと入らないときは、そのメッセージを捨てる（途中で切れたメッセージは流さない）。`CanTx` は ID の小さいストリームを先に送る。

## テストと確認

### md-core

- `encoder` と境界の演算はテストを先に書く。
- `cargo test -p md-core` を、既定と `--features f32` の両方で通す。

### minishirasu-firm ライブラリ部

- テストを先に書く。ホストで `cargo test -p minishirasu-firm --lib` を通す。
- `protocol`: 符号化 → 復号の往復、途中から受信したときの再同期、CRC 不一致、バイト欠落、バッファあふれ。
- `state`: 全遷移と、受け付けない条件。
- `cascade`: 各モードでの FF の加算と、飽和フラグの受け渡し。
- `pwm`、`sense`: 符号、0、上限、オフセット。
- `config`: ID 対応、既定値で各 `Param::new` が通ること。

### minishirasu-firm バイナリ部

- `minishirasu-firm/` で `cargo build --release` が通ること。
- Flash 64K、RAM 20K に収まること。

### 実機（ユーザーが実施）

1. 起動ログでオフセット値と、ADC が谷で取れていること（エラーが出ないこと）を見る。
2. `Status` が届くこと。母線電圧が実測と合うこと。
3. 手でモーターを回し、位置と速度の符号と大きさを見る。
4. 電流モードで小さい正の電流を流し、速度が正になることを見る。負なら `encoder_reversed` を切り替える。
5. 速度モード、位置モードを順に試す。
6. `CurrentLoop` の実行時間が 50µs に収まることを見る（GPIO のトグルをオシロで測る、または defmt でサイクル数を出す）。

## 文書の更新

- `minishirasu-firm/spec.md`: FF の記述を今回の内容に直し、プロトコルと向きの確認手順を追記する。
- `docs/制御.md` は編集しない。
