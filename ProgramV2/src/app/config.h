#ifndef CONFIG_H_
#define CONFIG_H_

#define TEMP_LIMIT 70  // 温度制限 [°C]

#define SUPPLY_VOLTAGE_MIN_LIMIT 7
#define SUPPLY_VOLTAGE_MAX_LIMIT 20

#define SERIAL_SEND_INTERVAL_US 500  // シリアル送信間隔 [μs]

// BLDCのパラメーター
#define POLE_PAIRS 7              // 極対数 (磁石の数/2)
#define MAX_ANGULAR_SPEED 100.0f  // 最大角速度 [rad/s]
#define MAX_ANGULAR_ACCEL 50.0f   // 最大角加速度 [rad/s^2]
#define R_PHASE 0.1f              // 相抵抗 [Ω] ※要測定 (マルチメータで相間測定後 /2)
#define KE 0.02653f               // 逆起電力定数 [V/(rad/s)] (360KV → 30/(360×π))
#define KT KE                     // トルク定数 [N·m/A] (SI単位系でKE=KT)
#define MAX_CURRENT 4.0f          // 最大電流 [A]
#define MAX_AMP_VOLT 12.0f        // 最大印加電圧 [V] (供給電圧上限に合わせる)

#endif  // CONFIG_H_