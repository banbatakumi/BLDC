#include "mymath.h"

const float sin_table[91] = {
    SIN0,
    SIN1,
    SIN2,
    SIN3,
    SIN4,
    SIN5,
    SIN6,
    SIN7,
    SIN8,
    SIN9,
    SIN10,
    SIN11,
    SIN12,
    SIN13,
    SIN14,
    SIN15,
    SIN16,
    SIN17,
    SIN18,
    SIN19,
    SIN20,
    SIN21,
    SIN22,
    SIN23,
    SIN24,
    SIN25,
    SIN26,
    SIN27,
    SIN28,
    SIN29,
    SIN30,
    SIN31,
    SIN32,
    SIN33,
    SIN34,
    SIN35,
    SIN36,
    SIN37,
    SIN38,
    SIN39,
    SIN40,
    SIN41,
    SIN42,
    SIN43,
    SIN44,
    SIN45,
    SIN46,
    SIN47,
    SIN48,
    SIN49,
    SIN50,
    SIN51,
    SIN52,
    SIN53,
    SIN54,
    SIN55,
    SIN56,
    SIN57,
    SIN58,
    SIN59,
    SIN60,
    SIN61,
    SIN62,
    SIN63,
    SIN64,
    SIN65,
    SIN66,
    SIN67,
    SIN68,
    SIN69,
    SIN70,
    SIN71,
    SIN72,
    SIN73,
    SIN74,
    SIN75,
    SIN76,
    SIN77,
    SIN78,
    SIN79,
    SIN80,
    SIN81,
    SIN82,
    SIN83,
    SIN84,
    SIN85,
    SIN86,
    SIN87,
    SIN88,
    SIN89,
    SIN90};

static inline float SinDeg(int deg) {
      deg = normalizeDegrees(deg);
      int theta_cal = deg % 90;
      if (deg >= 90 && deg < 180) {
            theta_cal = 90 - theta_cal;
      }
      if (deg >= 270 && deg < 360) {
            theta_cal = 90 - theta_cal;
      }

      if (deg >= 0 && deg <= 90) {  // 0~90 第一象限
            return sin_table[theta_cal];
      } else if (deg > 90 && deg <= 180) {  // 91~180 第二象限
            return sin_table[theta_cal];
      } else if (deg > 180 && deg <= 270) {  // 181 ~270 第三象限
            return -sin_table[theta_cal];
      } else if (deg > 270 && deg < 360) {  // 271~360 第四象限
            return -sin_table[theta_cal];
      } else {
            return 0;
      }
}

static inline float CosDeg(int deg) {
      return SinDeg(deg + 90);
}

static inline float Sin(float rad) {
      return SinDeg(Degrees(rad));
}

static inline float Cos(float rad) {
      return CosDeg(Degrees(rad));
}

static inline int NormalizeDegrees(int deg) {
      while (deg < 0) deg += 360;
      while (deg >= 360) deg -= 360;
      return deg;
}