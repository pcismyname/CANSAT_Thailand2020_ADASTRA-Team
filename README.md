# CANSAT_Thailand_ADASTRA-Team
Code for CANSAT 

This is code for cansat which uses LoRa technology and capture aerial photo

All sensors use I2C connection so it easy to use
Battery use serial connection for both 3.7 V and 5 V

This cansat can collect these data and send through to the ground station 
-enviroment temperature
-CANSAT temperature
-Humidity
-UV Index
-Infared
-Light intensity
-3-axis accelerometer 
-3-axis gyroscope
-Direction (degree from North)
-CO2



Microcontroler : ESP32 Heltec WiFi LoRa V.2 * 2 

GPS : NEO6MV2

Camera : ESP32CAM

Sensor :
-10DOF IMU Breakout - L3GD20H + LSM303 + BMP180
-SCD30
-SI1145

Battery :  Lipo Battery 3.7v 1000mah * 2

3.3 V  ****** This board has built in regulator ******
  power from  battery
-ESP32 Heltec WiFi LoRa V.2 
  power from microcontroller
-SCD30
-SI1145


5 V (power from serial batteries)   ***** use voltage regulator from 7.2 V --> 5 V  *****
-NEO6MV2
-10DOF IMU Breakout - L3GD20H + LSM303 + BMP180
-ESP32CAM

