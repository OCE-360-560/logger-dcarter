# A4 SD Logger
Logs light and temperature, as well as pitch, yaw, and roll (direction and orientation of the IMU sensor) to Logger_data_v1.csv once a second.
Each time step recoreded to the .csv file aslo prints out to USB serial. 
This project runs on RAsberry Pi Pico 2.

Each line has 'time_s, lux, temp_C, qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg, imu_accuracy'
Example:
time_s	 lux	temp_C	qw	    qx	       qy	    qz	    roll_deg	 pitch_deg	 yaw_deg    imu_accuracy

15.755,	 1.194,	22.845,	0.9476,	-0.0007,   -0.01,	0.3193	-0.4,	    -1.1,	    37.2,	    3
15.869,	 1.152,	22.707,	0.939,	-0.0007,   -0.0099,	0.3437	-0.5,	    -1,	        40.2,	    3


## Hardware and wiring
| Part                          | Pico 2 pin                 |
|-------------------------------S|----------------------------|
| Photocell divider             | GP26                       |
| LM19 Vout                     | GP27                       |
| SD CLK, DI, DO, CS            | GP18, 19, 20, 21           |
| VIN GND SDA SCL INT RST (IMU) | 3V3, GND, GP02, 03, 12, 13 |

## Build and run
1. Instal VS code and Rabserryy Pi Pico 2. extenison insid VS code
CLone this repo. Trust the code when prompted
3. Command Palette---> **Rasberry Pi Pico: Import Pico Project
4. Run the code 
## Files
1. Output will log readings in Logger_data_v1.csv.
## Third-party code
 bno08x: BNO080/BNO085 IMU driver for the Pico SDK
A small polled I2C driver for the BNO08x family (BNO080, BNO085, BNO086) on the Raspberry Pi Pico SDK, written for OCE360/560. The chip runs its own sensor fusion and sends reports; this driver turns on the reports you ask for and keeps the newest value of each. Plain C11, no dynamic memory, about 650 lines with comments
License: MIT
## Known issues