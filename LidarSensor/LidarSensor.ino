#include "Adafruit_VL53L0X.h"
 
#include <Wire.h>
#include "I2C_MPU6886.h"
#include <Stepper.h>  


Adafruit_VL53L0X lox = Adafruit_VL53L0X();
I2C_MPU6886 imu(0x68, Wire);

 
void setup()
{
  Serial.begin(115200);
  Wire1.begin(21, 17);
  Wire.begin(23, 22);
  imu.begin();

  // wait until serial port opens for native USB devices
  while (! Serial)
  {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin(0x29, false, &Wire1))
  {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1); 
  }
  // power
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
  
}
 
void loop()
{
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float t;
  char acMsg[100];

  imu.getAccel(&ax, &ay, &az);
  imu.getGyro(&gx, &gy, &gz);
  imu.getTemp(&t);
  sprintf(acMsg,"%f,%f,%f,%f,%f,%f,%f\n", ax, ay, az, gx, gy, gz, t);

  

  Serial.print("gyro");
  Serial.print(acMsg);
  Serial.println("");

  VL53L0X_RangingMeasurementData_t measure;



  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4)
  { // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    Serial.println("");
  }
  else
  {
    Serial.println(" out of range ");
  }

  delay(1000);
}