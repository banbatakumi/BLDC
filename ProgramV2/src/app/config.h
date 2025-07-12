#ifndef CONFIG_H_
#define CONFIG_H_

#define CONTROL_FREQ 5000
#define CONTROL_PERIOD (1.0f / CONTROL_FREQ)

#define TEMP_LIMIT 55  // 温度制限 [°C]

#define SUPPLY_VOLTAGE_MIN_LIMIT 8
#define SUPPLY_VOLTAGE_MAX_LIMIT 20

#endif  // CONFIG_H_