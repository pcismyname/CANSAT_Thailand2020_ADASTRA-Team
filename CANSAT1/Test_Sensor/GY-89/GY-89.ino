
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>


Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(10);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

void displaySensorDetails(void)
{
  sensor_t sensor;
  gyro.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" rad/s");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" rad/s");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" rad/s");
  Serial.println("------------------------------------");
  Serial.println("");
   sensor_t sensor1;
  accel.getSensor(&sensor1);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor1.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor1.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor1.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor1.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor1.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor1.resolution); Serial.println(" m/s^2");
  Serial.println("-----");
  delay(500);
}

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Gyroscope Test"); Serial.println("");

  /* Enable auto-ranging */
  gyro.enableAutoRange(true);
 if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
   if(!accel.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  displaySensorDetails();
}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event1;
  sensors_event_t event2;
  gyro.getEvent(&event2);
  accel.getEvent(&event1);

  Serial.print("X: "); Serial.print(event1.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event1.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event1.acceleration.z); Serial.print("  "); Serial.println("m/s^2 ");

  /* Display the results (speed is measured in rad/s) */
  Serial.print("X: "); Serial.print(event2.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event2.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event2.gyro.z); Serial.print("  ");
  Serial.println("rad/s ");
  delay(500);
}
