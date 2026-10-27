# bno08x: BNO080/BNO085 IMU driver for the Pico SDK

A small polled I2C driver for the BNO08x family (BNO080, BNO085, BNO086) on the
Raspberry Pi Pico SDK, written for OCE360/560. The chip runs its own sensor
fusion and sends *reports*; this driver turns on the reports you ask for and
keeps the newest value of each. Plain C11, no dynamic memory, about 650 lines with comments.

**Status (2026-10-22):** tare added. Compiles with no warnings (SDK 2.3.0, `pico2`), passes
host tests against a simulated chip, and runs on a Pico 2 with an Adafruit BNO085
breakout (product 4754) at 100 kHz.

## Get it into your project

1. On this page: **Code → Download ZIP**, then unzip it.
2. Rename the folder from `bno08x-main` to `bno08x` and move it into your project's `lib/` folder,
   so you have `lib/bno08x/bno08x.c`, `lib/bno08x/bno08x.h`, and `lib/bno08x/CMakeLists.txt`.
3. Add the two CMake lines below, build, and commit `lib/bno08x` with your code, so a fresh clone still builds.

If you clone this repo instead of downloading the ZIP, delete the `.git` folder inside
`lib/bno08x` before you commit, or your repo will hold a repo inside a repo.

## Use it

```cmake
add_subdirectory(lib/bno08x)
target_link_libraries(<your project> pico_stdlib hardware_i2c bno08x)
```

```c
#include "bno08x.h"

// You set up the bus; the driver never does (so other I2C parts can share it).
i2c_init(i2c1, 100 * 1000);
gpio_set_function(2, GPIO_FUNC_I2C);
gpio_set_function(3, GPIO_FUNC_I2C);
gpio_pull_up(2);
gpio_pull_up(3);

bno08x_config_t config = {
    .i2c = i2c1, .address = BNO08X_ADDRESS_DEFAULT,
    .int_pin = 12, .reset_pin = 13,   // or BNO08X_PIN_NOT_WIRED
};
if (!bno08x_init(&config)) { /* not found */ }
bno08x_enable_report(BNO08X_REPORT_ROTATION_VECTOR, 20000);   // every 20 ms

while (true) {
    bno08x_service();
    bno08x_quaternion_t q;
    if (bno08x_get_quaternion(BNO08X_REPORT_ROTATION_VECTOR, &q)) {
        printf("%.4f %.4f %.4f %.4f %u\n", q.real, q.i, q.j, q.k, q.status);
    }
}
```

## Functions

| Function | What it does |
|---|---|
| `bno08x_init(&config)` | resets the IMU (RST pin, or a reset command), checks it answers a Product ID request; false if not |
| `bno08x_enable_report(id, period_us)` | asks for one report at a period, then asks back what was granted; false if no answer |
| `bno08x_report_period_us(id)` | the period the chip actually granted (it may pick a nearby one) |
| `bno08x_service()` | reads every waiting packet (up to 10) and keeps the newest value of each report; call it often |
| `bno08x_get_quaternion(id, &q)` | newest rotation vector, game rotation vector, or geomagnetic rotation vector; true if new since the last call |
| `bno08x_get_vector(id, &v)` | newest accelerometer, gyroscope, magnetometer, linear acceleration, or gravity; true if new |
| `bno08x_tare_now(axes, basis)` | makes the current orientation the new zero (all axes, or heading only); lasts until a reset or `bno08x_clear_tare()` |
| `bno08x_clear_tare()` | undoes the tare |

Hover over any of them in VS Code for the full documentation from `bno08x.h`.

| Report | ID | Units | Q point |
|---|---|---|---|
| Accelerometer | 0x01 | m/s² (with gravity) | 8 |
| Gyroscope (calibrated) | 0x02 | rad/s | 9 |
| Magnetic field (calibrated) | 0x03 | µT | 4 |
| Linear acceleration | 0x04 | m/s² (gravity removed) | 8 |
| Rotation vector | 0x05 | unit quaternion; heading accuracy in rad (Q12) | 14 |
| Gravity | 0x06 | m/s² | 8 |
| Game rotation vector | 0x08 | unit quaternion (no magnetometer) | 14 |
| Geomagnetic rotation vector | 0x09 | unit quaternion; accuracy in rad (Q12) | 14 |

`status` on every report is the chip's accuracy estimate: 0 unreliable, 1 low,
2 medium, 3 high.

## Design notes

- **The caller owns the bus.** The driver only reads and writes; it never calls
  `i2c_init` or sets pin functions. INT is set up as an input with a pull-up and
  RST as an output, only if you give their pins.
- **100 kHz.** The BNO08x stretches the I2C clock, and the datasheet requires
  the host to support that. 100 kHz is plenty: one rotation vector packet is
  about 2.5 ms on the bus.
- **Never hangs.** Every transfer uses `i2c_read_timeout_us` /
  `i2c_write_timeout_us`. A missing or stuck IMU gives `false` or a negative
  `PICO_ERROR_` code.
- **Reading a packet:** read the 4-byte SHTP header to learn the length, then
  read the whole packet (each read starts with the header again). Packets longer
  than the 300-byte buffer are read to the end in pieces; only the first 300
  bytes are kept (only the startup advertisement is that long, and it isn't used).
- **Unexpected resets:** if the chip says "reset complete" after init, the driver
  sends every Set Feature again.
- **INT and RST are optional.** What each one is for:
  - **INT** (the chip's H_INTN, active low): the chip pulls it low when it has a
    packet waiting. With INT wired, `bno08x_service()` checks the pin and does no
    I2C traffic when nothing is waiting. Without it, every call reads a 4-byte
    header over I2C just to find out, which keeps the bus busy in a fast loop.
  - **RST** (the chip's NRST, active low): with it wired, `bno08x_init()` resets the
    chip by pulling the pin low, which works even if the chip has stopped answering
    on I2C. Without it, the driver sends a reset command over I2C instead, which only
    works while the chip is still listening.
- **One IMU per program.** State lives in file-level variables.
- **Tare is not saved.** The driver sends "tare now" and "set reorientation" (to clear it),
  never "persist tare", so an IMU reset always clears a tare.
- Not supported: SPI and UART modes, batching, calibration commands (save/clear
  dynamic calibration), FRS records, the gyro-integrated rotation vector channel.

## Sources

- BNO080 Datasheet, Rev 1.3, Hillcrest Labs, October 2017, document 1000-3927:
  <https://cdn.sparkfun.com/assets/b/e/e/7/4/BNO080.pdf> (I2C and SHTP: section 1.3.2
  and 1.4; startup: 5.2; report types: 2.2).
- SH-2 Reference Manual, Rev 1.2, Hillcrest Labs, May 2017, document 1000-3625:
  <https://cdn.sparkfun.com/assets/4/d/9/3/8/SH-2-Reference-Manual-v1.2.pdf>
  (Set Feature, Get Feature, Product ID, and every input report layout: section 6).

Written fresh from those two documents. These libraries were read as guides to
the I2C read pattern and the report lengths; no code was copied from them:

- SparkFun BNO080 Arduino Library, MIT license, Copyright (c) 2016 SparkFun Electronics:
  <https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library>
- Adafruit BNO08x Arduino library, BSD license: <https://github.com/adafruit/Adafruit_BNO08x>
- CEVA sh2 library, Apache License 2.0: <https://github.com/ceva-dsp/sh2>

## License

MIT, see [LICENSE](LICENSE).
