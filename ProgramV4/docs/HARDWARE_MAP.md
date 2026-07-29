# CurcuitV4 ハードウェア対応表 (ProgramV4 用)

CurcuitV4 の `CurcuitV4.kicad_pcb` / `production/bom.csv` から実際のネットを追って確認した内容。
回路図を開かずに firmware を書くときはここを見ること。

## 相の対応 (重要)

MCU の TIM1 チャンネル番号と基板の相ラベル (OUTA/B/C) は **一致していない**。

| TIM1 ch | MCU ピン (H/L) | DRV8300 入力 | 出力 | ソフト上の呼び名 |
|---|---|---|---|---|
| CH1 / CH1N | PA8 / PA7  | INHC / INLC | OUTC | U 相 (`u_pwm`) |
| CH2 / CH2N | PA9 / PB0  | INHB / INLB | OUTB | V 相 (`v_pwm`) |
| CH3 / CH3N | PA10 / PB1 | INHA / INLA | OUTA | W 相 (`w_pwm`) |

H/L の組み合わせ自体は正しくペアになっているので貫通は起きない。
A 相と C 相が入れ替わっているだけなので、3 相セットとしては正常。
回転方向が逆に見えるだけで制御上の問題はない。

## 電流センシング

- シャント: **R16 / R24 = 5 mΩ**。OUTB と OUTC のローサイドのみ。OUTA にはシャント無し。
  (BOM/回路図上は 10m だが、実装したのは 5 mΩ)
- アンプ: **U4 / U6 = INA181A1 (双方向, ゲイン 20 V/V, SOT-23-6)**
  - pin1 OUT / pin2 GND / pin3 IN+ / pin4 IN- / pin5 REF / pin6 VS
  - **IN+ = 下側 FET のソース (シャント上側)**, IN- = GNDPWR
  - REF = R28-R31 の **1k/1k 分圧で 1.65 V** → 無電流時の出力は約 ADC 2048
- ADC:

| ADC | ピン | ネット | 測っている相 |
|---|---|---|---|
| ADC1_IN1 (rank1) | PA0 | SENSEB | OUTB = ソフトの **V 相** |
| ADC1_IN2 (rank2) | PA1 | SENSEC | OUTC = ソフトの **U 相** |

W 相 (OUTA) は測れないので `Iw = -(Iu + Iv)` で算出する。

### 電流の符号

IN+ がシャント上側なので、**相電流が正 (インバータ → モータ) のときアンプ出力は REF より下がる**。
よって `I = -(ADC - offset) * ADC2CURRENT`。`config.h` の `CURRENT_SIGN = -1.0f` がこれ。

### レンジ

- 分解能 = 3.3/4095 / (0.005 × 20) = **約 0.0081 A/LSB**
- フルスケール = ±2048 LSB × 0.0081 = **約 ±16.5 A**

## その他のピン

| ネット | ピン |
|---|---|
| ENCODER | PA5 (ADC2_IN2, rank1 → `adc_val[0]`) |
| VOLTAGE | PA6 (ADC2_IN3, rank2 → `adc_val[1]`) |
| TEMP | PA4 (ADC2_IN1, rank3 → `adc_val[2]`) |
| BUTTON | **PA12** (V3 の PA0 から変更。PA0 は電流センスに使うので注意) |
| LED1-4 | PA15 / PB3 / PB4 / PB5 (色: LED1・LED2 = 青 / LED3 = 緑 / LED4 = 赤) |
| PC_TX / PC_RX (USART1, printf) | PB6 / PB7 |
| UART2_TX / RX | PA2 / PA3 |

## ADC 同期サンプリング

- TIM1: ARR = **1800**, prescaler 0, 72 MHz → **PWM 20 kHz**
  (**センター揃え** アップダウンカウント。1 周期 = 2 × ARR カウントなので
  `72M / (2 × 1800)`。エッジ揃えの `ARR+1` で割る式とは違うので注意)
- デューティ = **CCR / ARR**
- ローサイドシャントは低側 FET が ON の区間でしか正しい電流が流れない。
  センター揃えでは PWM1 モードで
  - `CNT < CCR` → 高側 ON。**カウンタの谷 (CNT=0) を中心**に広がる
  - `CNT >= CCR` → 低側 ON。**カウンタ頂点 (CNT=ARR) を中心**に広がる。片側 `(ARR - CCR)` カウント
- TIM1_CH4 を PWM2 モード・**CCR4 = ARR - 50 = 1750** にして OC4REF → TRGO2 → ADC1 外部トリガ。
  上りで通過するとき 1 回だけ立ち上がるので、トリガは 1 周期に 1 回
- **頂点が低側 ON 区間のちょうど中央**なので、そこでサンプリングすると
  三角波リプルの平均値 = その周期の平均電流がそのまま取れる
- `MAX_DUTY = 0.88` (`MAX_DUTY_CCR = 1584`)。頂点の手前側の予算で決まる:
  ```
  必要 = デッドタイム 20 + アンプ整定 144 + トリガ前倒し 50 = 214 カウント
  使える = ARR × (1 - MAX_DUTY) = 1800 × 0.12 = 216 カウント
  ```
  この関係は `config.h` の `_Static_assert` で検査している。**どれか 1 つを触ると
  ビルドが落ちる**ので、勘で数字を変えても静かには壊れない
- サンプリング時間 61.5 cycles @72 MHz → 1 変換 約 1.03 µs、2 ch で約 2.1 µs
- この変換完了 (DMA 転送完了) 割り込みが、そのまま 20 kHz の FOC 電流ループになる。
  詳細は [FOC.md](FOC.md) を参照。
- **CCR はプリロード付き** (HAL が PWM モードで OCxPE を立てる) なので、書き込みは
  次の更新イベントで反映される。センター揃えでは更新イベントが頂点と谷の**両方**で
  起きるため、割り込みが長引いて書き込みが谷を過ぎると次の頂点で反映されてしまい、
  低側 ON 区間が左右非対称になる。期限は `ISR_DEADLINE_US = 23.4 µs`
  (実測の最大は約 20 µs)。超えると起動中の `[PROF]` 表示が警告を出す
