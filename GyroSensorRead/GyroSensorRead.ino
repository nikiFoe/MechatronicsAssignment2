#include "I2C_MPU6886.h"
#include <Stepper.h>  
//I2C_MPU6886 imu(0x68, Wire1);
I2C_MPU6886 imu(I2C_MPU6886_DEFAULT_ADDRESS, Wire1);
const int buttonPin = 14;
int buttonState = 0;



const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution

// ULN2003 Motor Driver Pins
#define IN1 27
#define IN2 33
#define IN3 15  
#define IN4 32

Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire1.begin(23, 22);
  

  pinMode(buttonPin, INPUT);

  //declare the motor pins as outputs

   // set the speed at 5 rpm
  myStepper.setSpeed(5);

  imu.begin();
  Serial.printf("whoAmI() = 0x%02x\n", imu.whoAmI());
}

void loop() {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float t;
  char acMsg[100];
  buttonState = digitalRead(buttonPin);
  Serial.println(buttonState);

  Serial.println("clockwise");
  //myStepper.step(stepsPerRevolution);

  imu.getAccel(&ax, &ay, &az);
  imu.getGyro(&gx, &gy, &gz);
  imu.getTemp(&t);
  sprintf(acMsg,"%f,%f,%f,%f,%f,%f,%f\n", ax, ay, az, gx, gy, gz, t);
  Serial.println(acMsg);

  delay(100);
}
