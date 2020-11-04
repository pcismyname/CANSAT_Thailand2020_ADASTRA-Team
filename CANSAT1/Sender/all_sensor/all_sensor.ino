#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ADXL345.h>
#include "ITG3200.h"
#include "SparkFun_SCD30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#include <BH1750FVI.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define SEALEVELPRESSURE_HPA (1013.25)
#define RXPin (16)
#define TXPin (17)

static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial ss(2);



BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);
SCD30 airSensor;
ITG3200 gyro;
ADXL345 adxl;
TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme;

// setting
const unsigned long eventIMU = 5000;
const unsigned long eventBME = 1000;
const unsigned long eventSCD30 = 2000;
const unsigned long eventLight = 3000;
const unsigned long eventGPS = 3000;


unsigned long previousTime1 = 0;
unsigned long previousTime2 = 0;
unsigned long previousTime3 = 0;
unsigned long previousTime4 = 0;
unsigned long previousTime5 = 0;






void setup() {

  Wire.begin();
  Serial.begin(115200);
  Serial.println(F("BME280 test"));
  I2CBME.begin(I2C_SDA, I2C_SCL, 100000);

  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76, &I2CBME);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Default Test --");


  Serial.println();

  //IMU
  adxl.powerOn();
  gyro.init();
  gyro.zeroCalibrate(200, 10); //sample 200 times to calibrate and it will take 200*10ms
  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?

  //look of activity movement on this axes - 1 == on; 0 == off
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);

  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);

  //look of tap movement on this axes - 1 == on; 0 == off
  adxl.setTapDetectionOnX(0);
  adxl.setTapDetectionOnY(0);
  adxl.setTapDetectionOnZ(1);

  //set values for what is a tap, and what is a double tap (0-255)
  adxl.setTapThreshold(50); //62.5mg per increment
  adxl.setTapDuration(15); //625us per increment
  adxl.setDoubleTapLatency(80); //1.25ms per increment
  adxl.setDoubleTapWindow(200); //1.25ms per increment

  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment

  //setting all interrupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );

  //register interrupt actions - 1 == on; 0 == off
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);

  Serial.println("SCD30 Example");

  if (airSensor.begin() == false)
  {
    Serial.println("Air sensor not detected. Please check wiring. Freezing...");
    while (1)
      ;
  }

  airSensor.setMeasurementInterval(4); //Change number of seconds between measurements: 2 to 1800 (30 minutes)

  //My desk is ~1600m above sealevel
  airSensor.setAltitudeCompensation(1600); //Set altitude of the sensor in m

  //Pressure in Boulder, CO is 24.65inHg or 834.74mBar
  airSensor.setAmbientPressure(835); //Current ambient pressure in mBar: 700 to 1200

  float offset = airSensor.getTemperatureOffset();
  Serial.print("Current temp offset: ");
  Serial.print(offset, 2);
  Serial.println("C");

  LightSensor.begin();
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin, false);
  Serial.println(TinyGPSPlus::libraryVersion());


}



void printBME280() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  // Convert temperature to Fahrenheit
  /*Serial.print("Temperature = ");
    Serial.print(1.8 * bme.readTemperature() + 32);
    Serial.println(" *F");*/

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();

}
void printIMU() {
  //Boring accelerometer stuff
  int x, y, z;
  adxl.readXYZ(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z
  // Output x,y,z values
  Serial.print("values of X , Y , Z: ");
  Serial.print(x);
  Serial.print(" , ");
  Serial.print(y);
  Serial.print(" , ");
  Serial.println(z);

  double xyz[3];
  double ax, ay, az;
  adxl.getAcceleration(xyz);
  ax = xyz[0];
  ay = xyz[1];
  az = xyz[2];
  Serial.print("X=");
  Serial.print(ax);
  Serial.println(" g");
  Serial.print("Y=");
  Serial.print(ay);
  Serial.println(" g");
  Serial.print("Z=");
  Serial.print(az);
  Serial.println(" g");
  Serial.println("**********************");
  //delay(500);

  //GYRO
  int16_t gx, gy, gz;
  gyro.getXYZ(&gx, &gy, &gz);
  Serial.print("values of X , Y , Z: ");
  Serial.print(gx);
  Serial.print(" , ");
  Serial.print(gy);
  Serial.print(" , ");
  Serial.println(gz);

  float a1x, a1y, a1z;
  gyro.getAngularVelocity(&a1x, &a1y, &a1z);
  Serial.print("Angular Velocity of X , Y , Z: ");
  Serial.print(a1x);
  Serial.print(" , ");
  Serial.print(a1y);
  Serial.print(" , ");
  Serial.print(a1z);
  Serial.println(" degrees per second");
  Serial.println("*************");


}

void printCO2() {

  Serial.print("co2(ppm):");
  Serial.print(airSensor.getCO2());

  Serial.print(" temp(C):");
  Serial.print(airSensor.getTemperature(), 1);

  Serial.print(" humidity(%):");
  Serial.print(airSensor.getHumidity(), 1);
  Serial.println();


}

void printLight() {
  uint16_t lux = LightSensor.GetLightIntensity();
  Serial.print("Light: ");
  Serial.println(lux);
  delay(250);
}
void displayGPS()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("0"));
  }

  Serial.println();
}

void printGPS() {

  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayGPS();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }


}

void loop() {

  if (millis() - previousTime1 > eventIMU) {
    printIMU();
    previousTime1 = millis();

  }
  if (millis() - previousTime2 > eventGPS) {
    printGPS();
    previousTime2 = millis();

  }
  if (millis() - previousTime3 > eventBME) {
    printBME280();

    previousTime3 = millis();

  }
  if (millis() - previousTime4 > eventSCD30) {
    printCO2();
    previousTime4 = millis();

  }
  if (millis() - previousTime5 > eventLight) {
    printLight();
    previousTime5 = millis();

  }






}
