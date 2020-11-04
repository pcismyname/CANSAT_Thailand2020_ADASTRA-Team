#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <LSM303D.h>

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);


float temp2 = 0;
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;
int16_t accel[3];  // we'll store the raw acceleration values here
int16_t mag[3];  // raw magnetometer values stored here
float realAccel[3];  // calculated acceleration values here
float heading, titleHeading;


/*float humi2 = 0;
  float press2 = 0;
  float alti2 = 0;
  float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;*/


void setIMU() {
  bmp.begin();
  gyro.begin();
  gyro.enableAutoRange(true);
  Lsm303d.initI2C();


}

void sendIMU() {

  //sensor_t sensor;
  //bmp.getSensor(&sensor);
  //sensors_event_t event;
  // bmp.getEvent(&event);
  bmp.getTemperature(&temp2);
  //prees2 = event.pressure;
  //bmp.pressureToAltitude(seaLevelPressure,event.pressure);
  Serial.println(temp2);

  sensors_event_t event;
  gyro.getEvent(&event);
  gyroX = event.gyro.x;
  gyroY = event.gyro.y;
  gyroZ = event.gyro.z;

  Lsm303d.getAccel(accel);
  Lsm303d.getMag(mag);
  for (int i = 0; i < 3; i++) {
    realAccel[i] = accel[i] / pow(2, 15) * ACCELE_SCALE;  // calculate real acceleration values, in units of g
  }
  heading = Lsm303d.getHeading(mag);
  titleHeading = Lsm303d.getTiltHeading(mag, realAccel);

  /* Display the results (speed is measured in rad/s) */
  Serial.print("X: "); Serial.print(gyroX); Serial.print("  ");
  Serial.print("Y: "); Serial.print(gyroY); Serial.print("  ");
  Serial.print("Z: "); Serial.print(gyroZ); Serial.print("  ");
  Serial.println("rad/s ");

  Serial.println("Acceleration of X,Y,Z is");
    for (int i = 0; i < 3; i++) {
        Serial.print(realAccel[i]);
        Serial.println("g");
    }
    /* print both the level, and tilt-compensated headings below to compare */
    Serial.println("The clockwise angle between the magnetic north and x-axis: ");
    Serial.print(heading, 3); // this only works if the sensor is level
    Serial.println(" degrees");
    Serial.print("The clockwise angle between the magnetic north and the projection");
    Serial.println(" of the positive x-axis in the horizontal plane: ");
    Serial.print(titleHeading, 3);  // see how awesome tilt compensation is?!
    Serial.println(" degrees");

}
void setup() {
  Serial.begin(115200);
  setIMU();



}
void loop() {

  sendIMU();
  delay(5000);
}
