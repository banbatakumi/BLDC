#ifndef FIXFMT_H_
#define FIXFMT_H_

// float を固定小数点の文字列にする (printf の %f の代わり)。
// %f を1か所でも使うと newlib の浮動小数点書式 (_printf_float、約9.6KB) がリンクされ、
// ROM の残りを圧迫するので、表示はすべてこれと %s で行う:
//   printf("R %sΩ\n", FixFmt(r, 4));         → "R 0.1407Ω"
//   printf("%6s\n", FixFmtSigned(x, 2));      → " +1.25"  (幅は %s 側で指定する)
// 戻り値は内部の循環バッファ (FIXFMT_SLOTS 個) を指す。1回の printf で使えるのは
// FIXFMT_SLOTS 個まで。割り込みからは呼ばないこと (バッファを共有するため)。
// 桁数は 0〜6。絶対値 × 10^桁 が 4×10^9 を超えると "ovf"、NaN は "nan"。
#include <stdint.h>

#define FIXFMT_SLOTS 14

const char* FixFmt(float value, uint8_t decimals);
const char* FixFmtSigned(float value, uint8_t decimals);  // 正でも '+' を付ける

#endif  // FIXFMT_H_
