#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include "SSD1306.h"
#include<Arduino.h>
#include <Adafruit_BMP085_U.h>
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define BAND 441E6


Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);



float temp2 = 0;
int readingID = 0;
int counter = 0;
String LoRaMessage = "";

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
}

void setBMP() {
  bmp.begin();

}

void sendBMP() {

  //sensor_t sensor;
  //bmp.getSensor(&sensor);
  //sensors_event_t event;
  // bmp.getEvent(&event);
  bmp.getTemperature(&temp2);
  //prees2 = event.pressure;
  //bmp.pressureToAltitude(seaLevelPressure,event.pressure);
  Serial.println(temp2);

}

void sendReading() {

  LoRaMessage = String(readingID) + "," + String(temp2) ;



  LoRa.beginPacket();
  LoRa.print(LoRaMessage);
  LoRa.endPacket();
  readingID++;

}




void setup() {
  Serial.begin(115200);
  setBMP();

}

void loop() {

  sendBMP();
  delay(5000);
}
