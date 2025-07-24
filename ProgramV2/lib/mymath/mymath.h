#ifndef MYMATH_H_
#define MYMATH_H_

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define FOUR_PI 12.566370614359172953850573533118
#define TWO_THIRDS_PI 2.0943951023931954923084289221863
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define Abs(x) ((x) > 0 ? (x) : -(x))
#define Constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define Radians(deg) ((deg) * DEG_TO_RAD)
#define Degrees(rad) ((rad) * RAD_TO_DEG)

#define SIN0 0.0f
#define SIN1 0.0174524064372835
#define SIN2 0.034899496702501
#define SIN3 0.0523359562429438
#define SIN4 0.0697564737441253
#define SIN5 0.0871557427476582
#define SIN6 0.104528463267653
#define SIN7 0.121869343405147
#define SIN8 0.139173100960065
#define SIN9 0.156434465040231
#define SIN10 0.17364817766693
#define SIN11 0.190808995376545
#define SIN12 0.207911690817759
#define SIN13 0.224951054343865
#define SIN14 0.241921895599668
#define SIN15 0.258819045102521
#define SIN16 0.275637355816999
#define SIN17 0.292371704722737
#define SIN18 0.309016994374947
#define SIN19 0.325568154457157
#define SIN20 0.342020143325669
#define SIN21 0.3583679495453
#define SIN22 0.374606593415912
#define SIN23 0.390731128489274
#define SIN24 0.4067366430758
#define SIN25 0.422618261740699
#define SIN26 0.438371146789077
#define SIN27 0.453990499739547
#define SIN28 0.469471562785891
#define SIN29 0.484809620246337
#define SIN30 0.5f
#define SIN31 0.515038074910054
#define SIN32 0.529919264233205
#define SIN33 0.544639035015027
#define SIN34 0.559192903470747
#define SIN35 0.573576436351046
#define SIN36 0.587785252292473
#define SIN37 0.601815023152048
#define SIN38 0.615661475325658
#define SIN39 0.629320391049838
#define SIN40 0.642787609686539
#define SIN41 0.656059028990507
#define SIN42 0.669130606358858
#define SIN43 0.681998360062498
#define SIN44 0.694658370458997
#define SIN45 0.707106781186547
#define SIN46 0.719339800338651
#define SIN47 0.73135370161917
#define SIN48 0.743144825477394
#define SIN49 0.754709580222772
#define SIN50 0.766044443118978
#define SIN51 0.777145961456971
#define SIN52 0.788010753606722
#define SIN53 0.798635510047293
#define SIN54 0.809016994374947
#define SIN55 0.819152044288992
#define SIN56 0.829037572555042
#define SIN57 0.838670567945424
#define SIN58 0.848048096156426
#define SIN59 0.857167300702112
#define SIN60 0.866025403784439
#define SIN61 0.874619707139396
#define SIN62 0.882947592858927
#define SIN63 0.891006524188368
#define SIN64 0.898794046299167
#define SIN65 0.90630778703665
#define SIN66 0.913545457642601
#define SIN67 0.92050485345244
#define SIN68 0.927183854566787
#define SIN69 0.933580426497202
#define SIN70 0.939692620785908
#define SIN71 0.945518575599317
#define SIN72 0.951056516295154
#define SIN73 0.956304755963035
#define SIN74 0.961261695938319
#define SIN75 0.965925826289068
#define SIN76 0.970295726275996
#define SIN77 0.974370064785235
#define SIN78 0.978147600733806
#define SIN79 0.981627183447664
#define SIN80 0.984807753012208
#define SIN81 0.987688340595138
#define SIN82 0.99026806874157
#define SIN83 0.992546151641322
#define SIN84 0.994521895368273
#define SIN85 0.996194698091746
#define SIN86 0.997564050259824
#define SIN87 0.998629534754574
#define SIN88 0.999390827019096
#define SIN89 0.999847695156391
#define SIN90 1.0f

static const float sin_table[91] = {
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

static inline int NormalizeDegrees(int deg) {
      while (deg < 0) deg += 360;
      while (deg >= 360) deg -= 360;
      return deg;
}

static inline float NormalizeRadians(float rad) {
      while (rad < 0) rad += TWO_PI;
      while (rad >= TWO_PI) rad -= TWO_PI;
      return rad;
}

static inline float GapRadians(float rad1, float rad2) {
      float gap = rad1 - rad2;
      if (gap > PI) gap -= TWO_PI;
      if (gap < -PI) gap += TWO_PI;
      return gap;
}

static inline float SinDeg(int deg) {
      deg = NormalizeDegrees(deg);
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

static inline float Atan2(float y, float x) {
      if (x == 0) {
            if (y > 0) return HALF_PI;
            if (y < 0) return -HALF_PI;
            return 0;
      }

      float abs_y = Abs(y) + 1e-10f;  // 0除算防止
      float angle;

      if (Abs(x) >= abs_y) {
            float r = (y / x);
            angle = r / (1.0f + 0.28f * r * r);  // 近似式
            if (x < 0.0f) {
                  if (y >= 0)
                        angle += PI;
                  else
                        angle -= PI;
            }
      } else {
            float r = (x / y);
            angle = HALF_PI - r / (1.0f + 0.28f * r * r);
            if (y < 0) angle -= PI;
      }

      return angle;
}

#endif  // MYMATH_H_