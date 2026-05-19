#ifndef CONFIG_H_
#define CONFIG_H_

#define TEMP_LIMIT 60  // 温度制限 [°C]

#define SUPPLY_VOLTAGE_MIN_LIMIT 7
#define SUPPLY_VOLTAGE_MAX_LIMIT 20

#define SERIAL_SEND_INTERVAL_US 500  // シリアル送信間隔 [μs]

// BLDCのパラメーター
#define POLE_PAIRS 7              // 極対数 (磁石の数/2)
#define MAX_ANGULAR_SPEED 100.0f  // 最大角速度 [rad/s]
#define MAX_ANGULAR_ACCEL 50.0f   // 最大角加速度 [rad/s^2]
#define MAX_AMP_VOLT 1.0f         // 最大印加電圧 [V]

#endif  // CONFIG_H_