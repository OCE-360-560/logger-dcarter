# A4 SD Logger
Logs light and temperature, as well as pitch, yaw, and roll (direction and orientation of the IMU sensor) to Logger_data_v1.csv once a second.
Each time step recoreded to the .csv file aslo prints out to USB serial. 
This project runs on RAsberry Pi Pico 2.

Each line has 'time_s, lux, temp_C, qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg, imu_accuracy'
Example:
"time_s, lux,       temp_C, qw,     qx,     qy,     qz,     roll_deg,   pitch_deg, yaw_deg, imu_accuracy," "fix_date,       fix_time,       gps_valid, lat_deg,     lon_deg,    alt_m, 
2.808	6813888	    24.844	0.3515	-0.0502	0.0294	0.9344	1.1	        6.6	138.8	0	                    2024-1-06	    14:27:20.000	0	        32.910329	-42.02465	30.8
2.909	263652.812	23.258	0.3515	-0.0502	0.0293	0.9344	1.1	        6.6	138.8	0	                    2024-1-06	    14:27:20.000	0	        32.910329	-42.02465	30.8	        
satellites, hdop, speed_mps, course_deg\r\n"
4	        1.7	    0.07	    0
4	        1.7	    0.08	    0



## Hardware and wiring
| Part                          | Pico 2 pin                 |
|-------------------------------|----------------------------|
| Photocell divider             | GP26                       |
| LM19 Vout                     | GP27                       |
| SD CLK, DI, DO, CS            | GP18, 19, 20, 21           |
| VIN GND SDA SCL INT RST (IMU) | 3V3, GND, GP02, 03, 12, 13 |
| VIN GND Tx Rx (GPS)           | 3V3, GND, GP07, 06         |


## Build and run
1. Instal VS code and Rabserryy Pi Pico 2. extenison insid VS code
CLone this repo. Trust the code when prompted
3. Command Palette---> **Rasberry Pi Pico: Import Pico Project
4. Run the code 
## Files
1. Output will log readings in GPs_logger_data_2.csv.
## Third-party code
-bno08x: BNO080/BNO085 IMU driver for the Pico SDK
A small polled I2C driver for the BNO08x family (BNO080, BNO085, BNO086) on the Raspberry Pi Pico SDK, written for OCE360/560. The chip runs its own sensor fusion and sends reports; this driver turns on the reports you ask for and keeps the newest value of each. Plain C11, no dynamic memory, about 650 lines with comments
-lib/pico_fatfs (FatFs R0.15 ([http://elm-chan.org/fsw/ff/00index_e.html]))
-air530: https://github.com/OCE-360-560/air530
This is a function built to assist with the GPS UART data taking in the data and assisting with processing and exporting the data to be saved
License: MIT
## Known issues
No known issues