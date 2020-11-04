0.#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ADXL345.h>
#include "ITG3200.h"
#include "SparkFun_SCD30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#include <BH1750FVI.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <LoRa.h>
#include <SPI.h>
#include "Adafruit_SI1145.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <LSM303D.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define SEALEVELPRESSURE_HPA (1013.25)
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26
#define BAND 441E6
#define RXPin (16)
#define TXPin (17)

TinyGPSPlus gps;
HardwareSerial ss(2);
//ITG3200 gyro;
ADXL345 adxl;
TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme;
BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);
SCD30 airSensor;
Adafruit_SI1145 uv = Adafruit_SI1145();
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);


static const uint32_t GPSBaud = 9600;
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

//float temp2 = 0;
float humi2 = 0;
float press2 = 0;
float alti2 = 0;
double ax, ay, az;
float g1x, g1y, g1z;
float co2;
float temp1;
float humi1;
uint16_t lux;
double lat1;
double lng1;
float UVindex;
float IR;
float VL;
float temp2 = 0;
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;
int16_t accel[3];  // we'll store the raw acceleration values here
int16_t mag[3];  // raw magnetometer values stored here
float realAccel[3];  // calculated acceleration values here
float heading, titleHeading;
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

int readingID = 0;
int counter = 0;
String LoRaMessage = "";

/*void startOLED(){
  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA SENDER");
  }*/

void startLoRA() {
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  while (!LoRa.begin(BAND) && counter < 10) {
    Serial.print(".");
    counter++;
    delay(500);
  }
  if (counter == 10) {
    // Increment readingID on every new reading
    readingID++;
    Serial.println("Starting LoRa failed!");
  }

  /* Serial.println("LoRa Initialization OK!");
    display.setCursor(0, 10);
    display.clearDisplay();
    display.print("LoRa Initializing OK!");
    display.display();
    delay(2000);*/
}

void startBME() {

  I2CBME.begin(I2C_SDA, I2C_SCL, 100000);
  bool status;

  status = bme.begin(0x76, &I2CBME);
  if (!status) {

    while (1);
  }

}


/*void startIMU() {
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
  }*/

void startSCD() {
  airSensor.begin();
  airSensor.setMeasurementInterval(4); //Change number of seconds between measurements: 2 to 1800 (30 minutes)

  //My desk is ~1600m above sealevel
  airSensor.setAltitudeCompensation(1600); //Set altitude of the sensor in m

  //Pressure in Boulder, CO is 24.65inHg or 834.74mBar
  airSensor.setAmbientPressure(835); //Current ambient pressure in mBar: 700 to 1200

  float offset = airSensor.getTemperatureOffset();
}

void startIMU10() {
  bmp.begin();
  gyro.begin();
  gyro.enableAutoRange(true);
  Lsm303d.initI2C();
}
void startLight() {
  LightSensor.begin();
}

void startUV() {
  uv.begin();
}


/*void readBME280() {
  temp2 = bme.readTemperature();
  press2 = bme.readPressure() / 100.0F;
  humi2 = bme.readHumidity();
  alti2 = bme.readAltitude(SEALEVELPRESSURE_HPA);
  alti2 -= 32;
  }*/

void startGPS() {
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin, false);

}

/*void readIMU() {
  double xyz[3];

  adxl.getAcceleration(xyz);
  ax = xyz[0];
  ay = xyz[1];
  az = xyz[2];

  gyro.getAngularVelocity(&g1x, &g1y, &g1z);
  }*/

void readSCD30() {
  co2 = airSensor.getCO2();
  temp1 = airSensor.getTemperature();
  humi1 =  airSensor.getHumidity();
}

void readLight() {
  lux = LightSensor.GetLightIntensity(); LightSensor.GetLightIntensity();
}

void readIMU10() {
  sensor_t sensor;
  //bmp.getSensor(&sensor);
  sensors_event_t event;
  sensors_event_t event2;
   bmp.getEvent(&event2);
  bmp.getTemperature(&temp2);
  press2 = event2.pressure;
  alti2 = bmp.pressureToAltitude(seaLevelPressure, event2.pressure);
  ///Serial.println(temp2);

  //  sensors_event_t event;
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

}

void displayGPS()
{

  if (gps.location.isValid())
  {
    lat1 = (gps.location.lat(), 6);

    lng1 = (gps.location.lng(), 6);
  }
  else
  {
    lat1 = 0;

    lng1 = 0;
  }


}
void readGPS() {
  while (ss.available() > 0) if (gps.encode(ss.read())) displayGPS();

}

void readUV() {
  UVindex = uv.readUV();
  UVindex /= 100.0;
  IR = uv.readIR();
  VL = uv.readVisible();
}



void sendReading() {

  /*LoRaMessage = String(readingID) + "," + String(temp2) + "," + String(humi2) + "," + String(press2) + "," + String(alti2) + "," +
                     String(ax) + "," + String(ay) + "," + String(az) + "," + String(g1x) + "," + String(g1y) + "," + String(g1z) + ","  + String(co2) + ","
                     + String(temp1) + "," + String(humi1) + "," + String(lux) + "," + String(10) + ","  + String(10) + ",";*/
  
  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      if (gps.location.isValid()) {
        LoRaMessage = String(readingID) + "," + String(temp2)  + "," + String(press2) + "," + String(alti2) + "," +
                      String( realAccel[0]) + "," + String( realAccel[1]) + "," + String( realAccel[2]) + "," + String(gyroX) + "," + String(gyroY) + "," + String(gyroZ) + "," + String(titleHeading, 3) + ","  + String(co2) + ","
                      + String(temp1) + "," + String(humi1) + "," + String(UVindex) + "," + String(IR) + "," + String(VL) + "," + String(gps.location.lat(), 6) + ","  + String(gps.location.lng(), 6) + "," +String( gps.altitude.isValid(), 2);

      }
      else {
        LoRaMessage = String(readingID) + "," + String(temp2)  + "," + String(press2) + "," + String(alti2) + "," +
                      String(realAccel[0]) + "," + String(realAccel[1]) + "," + String(realAccel[2]) + "," + String(gyroX) + "," + String(gyroY) + "," + String(gyroZ) + "," + String(titleHeading, 3) + ","  + String(co2) + ","
                      + String(temp1) + "," + String(humi1) + "," +  String(UVindex) + "," + String(IR) + "," + String(VL) + "," + String("NaN") + ","  + String("NaN") + "," + String("NaN") ;
      }
    }
  }
 
  LoRa.beginPacket();
  LoRa.print(LoRaMessage);
  LoRa.endPacket();

  readingID++;


}


void setup() {

  //Wire.begin();
  Serial.begin(115200);
  startLoRA();
  String Label = "No,Temp,Press,Alti,ax,ay,az,gx,gy,gz,compass,co2,temp1,hum1,UV,IR,VL,lat,lng";
  LoRa.beginPacket();
  LoRa.print(Label);
  LoRa.endPacket();
  Serial.println(Label);
  //startBME();
  //startIMU();
  startSCD();
  startLight();
  startGPS();
  startUV();
  startIMU10();

}

void loop() {

  /*if (millis() - previousTime1 > eventIMU) {
      readIMU();
      previousTime1 = millis();

    }
    if (millis() - previousTime2 > eventGPS) {
      readGPS();
      previousTime2 = millis();

    }
    if (millis() - previousTime3 > eventBME) {
      readBME280();

      previousTime3 = millis();

    }
    if (millis() - previousTime4 > eventSCD30) {
      readSCD30();
      previousTime4 = millis();

    }
    if (millis() - previousTime5 > eventLight) {
      readLight();
      previousTime5 = millis();

    }*/
  //readIMU();
  //readGPS();
  readSCD30();
  //readBME280();
  readLight();
  readUV();
  readIMU10();
  sendReading();
  /*while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      if (gps.location.isValid()) {
        LoRaMessage = String(readingID) + "," + String(temp2) + "," + String(humi2) + "," + String(press2) + "," + String(alti2) + "," +
                      String(ax) + "," + String(ay) + "," + String(az) + "," + String(g1x) + "," + String(g1y) + "," + String(g1z) + ","  + String(co2) + ","
                      + String(temp1) + "," + String(humi1) + "," + String(lux) + "," + String(gps.location.lat(), 6) + ","  + String(gps.location.lng(), 6) + ",";

      }
      else {
        LoRaMessage = String(readingID) + "," + String(temp2) + "," + String(humi2) + "," + String(press2) + "," + String(alti2) + "," +
                      String(ax) + "," + String(ay) + "," + String(az) + "," + String(g1x) + "," + String(g1y) + "," + String(g1z) + ","  + String(co2) + ","
                      + String(temp1) + "," + String(humi1) + "," + String(lux) + "," + String(10) + ","  + String(10) + ",";
      }
    }
    }*/

  /* LoRaMessage = String(readingID) + "," + String(temp2) + "," + String(humi2) + "," + String(press2) + "," + String(alti2) + "," +
                 String(ax) + "," + String(ay) + "," + String(az) + "," + String(g1x) + "," + String(g1y) + "," + String(g1z) + ","  + String(co2) + ","
                 + String(temp1) + "," + String(humi1) + "," + String(lux) + "," + String(gps.location.lat(), 6) + ","  + String(gps.location.lng(), 6) + ",";*/
  //Serial.println(LoRaMessage);
  //readingID++;
  delay(800);
}
