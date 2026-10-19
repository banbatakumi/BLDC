#include "fixfmt.h"

#include <stdbool.h>

#define FIXFMT_LEN 16  // 符号 + 整数10桁 + '.' + 小数 + '\0' に収まる (桁数を6までに制限している)

static char fixfmt_buf[FIXFMT_SLOTS][FIXFMT_LEN];
static uint8_t fixfmt_next;

static const char* Format(float value, uint8_t decimals, bool plus) {
  static const uint32_t POW10[7] = {1, 10, 100, 1000, 10000, 100000, 1000000};
  if (decimals > 6) decimals = 6;

  char* out = fixfmt_buf[fixfmt_next];
  fixfmt_next = (uint8_t)((fixfmt_next + 1) % FIXFMT_SLOTS);

  if (!(value == value)) return "nan";
  bool neg = value < 0.0f;
  float mag = neg ? -value : value;
  float scaled = mag * (float)POW10[decimals] + 0.5f;  // 四捨五入
  if (!(scaled < 4.0e9f)) return neg ? "-ovf" : "ovf";

  uint32_t n = (uint32_t)scaled;
  uint32_t ip = n / POW10[decimals];
  uint32_t fp = n % POW10[decimals];

  char* p = out;
  if (neg && n != 0) {
    *p++ = '-';
  } else if (plus) {
    *p++ = '+';
  }
  // 整数部 (逆順に作ってから反転)
  char tmp[10];
  uint8_t len = 0;
  do {
    tmp[len++] = (char)('0' + ip % 10);
    ip /= 10;
  } while (ip != 0);
  while (len > 0) *p++ = tmp[--len];
  // 小数部 (先頭の0も出す)
  if (decimals > 0) {
    *p++ = '.';
    for (uint8_t d = decimals; d > 0; d--) {
      p[d - 1] = (char)('0' + fp % 10);
      fp /= 10;
    }
    p += decimals;
  }
  *p = '\0';
  return out;
}

const char* FixFmt(float value, uint8_t decimals) { return Format(value, decimals, false); }
const char* FixFmtSigned(float value, uint8_t decimals) { return Format(value, decimals, true); }
