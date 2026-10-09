#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include "main.h"

// 独立ウォッチドッグ (IWDG)。LSI で動くので、メインループがハングしても効く。
//
// このMDは 20kHz の割り込みが制御ループ本体で、メインループは「指令の受信・無通信タイムアウト・
// 異常の監視」を担当している。メインループだけが止まると、割り込みは最後に受け取った指令を
// 出し続け、止める手段が無くなる。リセットされれば出力ピンは Hi-Z に戻り、DRV8300 の入力は
// 内蔵プルダウンで Low = コーストになる。
//
// HAL のドライバではなくレジスタを直接叩いている (IWDG は .ioc で有効化されておらず、
// HAL モジュールを足すと CubeMX の再生成と衝突するため)。
// **一度起動すると停止できない。** ブロッキングする初期化・校正がすべて終わってから起動すること。

#define WATCHDOG_KEY_ENABLE 0x0000CCCCu
#define WATCHDOG_KEY_WRITE_ACCESS 0x00005555u
#define WATCHDOG_KEY_RELOAD 0x0000AAAAu

// プリスケーラ 64分周 (PR=4)。STM32F303 の LSI は標準 40kHz (30〜50kHz) なので 1カウント 1.6ms
#define WATCHDOG_PRESCALER 4u
#define WATCHDOG_TICK_US 1600u
#define WATCHDOG_RELOAD_MAX 0xFFFu  // RLR は12bit

// 起動する。以降 timeout_ms 以内に Watchdog_Refresh() を呼び続けないとリセットされる。
// LSI のばらつきで実際のタイムアウトは指定値の 0.8〜1.33 倍に振れる
static inline void Watchdog_Start(uint32_t timeout_ms) {
  uint32_t reload = timeout_ms * 1000u / WATCHDOG_TICK_US;
  if (reload > WATCHDOG_RELOAD_MAX) reload = WATCHDOG_RELOAD_MAX;
  if (reload == 0u) reload = 1u;

  // デバッガでコアを止めている間はカウントも止める (ブレークポイントのたびにリセットされないように)
  __HAL_DBGMCU_FREEZE_IWDG();

  IWDG->KR = WATCHDOG_KEY_ENABLE;
  IWDG->KR = WATCHDOG_KEY_WRITE_ACCESS;
  IWDG->PR = WATCHDOG_PRESCALER;
  IWDG->RLR = reload;
  // PR/RLR は LSI 側のクロックで反映されるため、完了するまで次の操作をしない
  while (IWDG->SR != 0u) {
  }
  IWDG->KR = WATCHDOG_KEY_RELOAD;
}

static inline void Watchdog_Refresh(void) {
  IWDG->KR = WATCHDOG_KEY_RELOAD;
}

#endif  // WATCHDOG_H_
