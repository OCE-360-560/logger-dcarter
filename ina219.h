#ifndef INA219_H
#define INA219_H

#include <stdint.h>
#include "hardware/i2c.h"

// ── Default I2C address (A0=GND, A1=GND) ──────────────────────────────────
#define INA219_ADDR_40  0x40
#define INA219_ADDR_41  0x41
#define INA219_ADDR_44  0x44
#define INA219_ADDR_45  0x45

// ── Register map ──────────────────────────────────────────────────────────
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNT_V      0x01
#define INA219_REG_BUS_V        0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

// ── Configuration register bits ───────────────────────────────────────────
// Bus voltage range
#define INA219_BRNG_16V  (0 << 13)   // 0–16 V
#define INA219_BRNG_32V  (1 << 13)   // 0–32 V  (default)

// PGA gain (shunt voltage range)
#define INA219_PGA_1    (0 << 11)    // ±40 mV
#define INA219_PGA_2    (1 << 11)    // ±80 mV
#define INA219_PGA_4    (2 << 11)    // ±160 mV
#define INA219_PGA_8    (3 << 11)    // ±320 mV  (default, up to ±3.2 A)

// ADC resolution / averaging
#define INA219_ADC_12BIT         (3 << 7)   // 12-bit, 532 µs (default)
#define INA219_ADC_12BIT_2AVG    (9 << 7)
#define INA219_ADC_12BIT_4AVG    (10 << 7)
#define INA219_ADC_12BIT_8AVG    (11 << 7)
#define INA219_ADC_12BIT_16AVG   (12 << 7)
#define INA219_ADC_12BIT_32AVG   (13 << 7)
#define INA219_ADC_12BIT_64AVG   (14 << 7)
#define INA219_ADC_12BIT_128AVG  (15 << 7)

// Operating mode
#define INA219_MODE_POWER_DOWN   0x00
#define INA219_MODE_SHUNT_TRIG   0x01
#define INA219_MODE_BUS_TRIG     0x02
#define INA219_MODE_SHUNT_BUS_TRIG 0x03
#define INA219_MODE_ADC_OFF      0x04
#define INA219_MODE_SHUNT_CONT   0x05
#define INA219_MODE_BUS_CONT     0x06
#define INA219_MODE_SHUNT_BUS_CONT 0x07   // default: continuous shunt + bus

// ── Driver struct ─────────────────────────────────────────────────────────
typedef struct {
    i2c_inst_t *i2c;          // i2c0 or i2c1
    uint8_t     addr;         // 7-bit I2C address
    float       current_lsb; // A per bit, set during init
} INA219;

// ── API ───────────────────────────────────────────────────────────────────

/**
 * Initialize the INA219.
 *
 * @param ina         Pointer to INA219 struct (caller allocates).
 * @param i2c         i2c0 or i2c1 (must already be initialized with i2c_init).
 * @param addr        7-bit I2C address (use INA219_ADDR_* defines).
 * @param r_shunt_ohm Shunt resistor value in ohms (0.1 for Adafruit breakout).
 * @param max_amps    Maximum expected current in amps (e.g. 3.2f).
 * @return            0 on success, -1 on I2C error.
 */
int ina219_init(INA219 *ina, i2c_inst_t *i2c, uint8_t addr,
                float r_shunt_ohm, float max_amps);

/**
 * Read bus voltage in volts (0 to +26 V).
 */
float ina219_read_bus_voltage(INA219 *ina);

/**
 * Read shunt voltage in millivolts.
 */
float ina219_read_shunt_voltage_mv(INA219 *ina);

/**
 * Read current in amps (requires calibration register to be set).
 */
float ina219_read_current_amps(INA219 *ina);

/**
 * Read power in watts (requires calibration register to be set).
 */
float ina219_read_power_watts(INA219 *ina);

/**
 * Reset the INA219 to power-on defaults.
 */
void ina219_reset(INA219 *ina);

#endif // INA219_H
