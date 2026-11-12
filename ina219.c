#include "ina219.h"
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// ── Internal helpers ──────────────────────────────────────────────────────

static int write_reg(INA219 *ina, uint8_t reg, uint16_t value)
{
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (value >> 8) & 0xFF;   // MSB first
    buf[2] = (value     ) & 0xFF;
    int ret = i2c_write_blocking(ina->i2c, ina->addr, buf, 3, false);
    return (ret == 3) ? 0 : -1;
}

static int read_reg(INA219 *ina, uint8_t reg, uint16_t *out)
{
    int ret;
    ret = i2c_write_blocking(ina->i2c, ina->addr, &reg, 1, true);
    if (ret != 1) return -1;

    uint8_t buf[2];
    ret = i2c_read_blocking(ina->i2c, ina->addr, buf, 2, false);
    if (ret != 2) return -1;

    *out = ((uint16_t)buf[0] << 8) | buf[1];
    return 0;
}

// ── Public API ────────────────────────────────────────────────────────────

int ina219_init(INA219 *ina, i2c_inst_t *i2c, uint8_t addr,
                float r_shunt_ohm, float max_amps)
{
    ina->i2c  = i2c;
    ina->addr = addr;

    // Reset to defaults first
    ina219_reset(ina);
    sleep_ms(1);

    // ── Calibration ──────────────────────────────────────────────────────
    // current_lsb = max_amps / 32768  (smallest representable current step)
    ina->current_lsb = max_amps / 32768.0f;

    // calibration register value = 0.04096 / (current_lsb * r_shunt)
    uint16_t cal = (uint16_t)(0.04096f / (ina->current_lsb * r_shunt_ohm));
    if (write_reg(ina, INA219_REG_CALIBRATION, cal) != 0) return -1;

    // ── Configuration ─────────────────────────────────────────────────────
    // 32V bus range | PGA /8 (±320 mV, up to ±3.2 A) |
    // 12-bit ADC | continuous shunt + bus measurement
    // uint16_t config = INA219_BRNG_32V      |
    //                   INA219_PGA_8         |
    //                   INA219_ADC_12BIT     |   // bus ADC
    //                   (INA219_ADC_12BIT >> 4) | // shunt ADC (bits 6:3)
    //                   INA219_MODE_SHUNT_BUS_CONT;

    // return write_reg(ina, INA219_REG_CONFIG, config);

    uint16_t config = INA219_BRNG_32V           |
                  INA219_PGA_8              |
                  INA219_ADC_12BIT_128AVG   |   // bus:   128-sample average
                  (15 << 3)                 |   // shunt: 128-sample average
                  INA219_MODE_SHUNT_BUS_CONT;

    return write_reg(ina, INA219_REG_CONFIG, config);
}

void ina219_reset(INA219 *ina)
{
    // Bit 15 of config register = RST
    write_reg(ina, INA219_REG_CONFIG, 0x8000);
}

float ina219_read_bus_voltage(INA219 *ina)
{
    uint16_t raw;
    if (read_reg(ina, INA219_REG_BUS_V, &raw) != 0) return 0.0f;

    // Bits 15:3 are the voltage result; bits 2:0 are flags
    // LSB = 4 mV
    int16_t voltage_counts = (int16_t)(raw >> 3);
    return voltage_counts * 0.004f;   // volts
}

float ina219_read_shunt_voltage_mv(INA219 *ina)
{
    uint16_t raw;
    if (read_reg(ina, INA219_REG_SHUNT_V, &raw) != 0) return 0.0f;

    // Signed 16-bit, LSB = 10 µV
    int16_t signed_raw = (int16_t)raw;
    return signed_raw * 0.01f;   // millivolts
}

float ina219_read_current_amps(INA219 *ina)
{
    uint16_t raw;
    if (read_reg(ina, INA219_REG_CURRENT, &raw) != 0) return 0.0f;

    int16_t signed_raw = (int16_t)raw;
    return signed_raw * ina->current_lsb;   // amps
}

float ina219_read_power_watts(INA219 *ina)
{
    uint16_t raw;
    if (read_reg(ina, INA219_REG_POWER, &raw) != 0) return 0.0f;

    // Power LSB = 20 × current_lsb
    return (float)raw * 20.0f * ina->current_lsb;   // watts
}
