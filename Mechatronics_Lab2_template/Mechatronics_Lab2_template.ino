/**
 * Mechatronics, 2022
 * This is a template for MQTT and ESP32.
 */

#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <string.h>
#include <Stepper.h>  
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "I2C_MPU6886.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

//Set other SCL and SDA Pin
I2C_MPU6886 imu(0x68, Wire);

const char* pcSSID = "iPSK-UMU"; // Enter your WiFi name
const char* pcPASSWORD =  "HejHopp!!"; // Enter WiFi password
const char* pcMqttServer = "tfe.iotwan.se";
const int   iMqttPort = 1883;
const char* pcMqttUser = "intro22";
const char* pcMqttPassword = "outro";
const double gravityZ = 1.05;
double angle = 0;
int distanceLidar;
bool lightPirLED = false;
bool newState = true;
bool sendInterrupCount = false;
int interruptsTotal = 0;

const int pirPin = 13;
const int interruptPin = 19;
const int pirLEDPin = 16;
const int ledLidarPin = 12;
const int buttonPin = 14;
int buttonState = 0;
int pirState = 0;
int pirStateLocal = 0;
int pirStatePrev = 2;
int remotebuttonstatus = 0;
double stepperAngle = 0; // in degree
int stepsPerPress = 4;

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution


// ULN2003 Motor Driver Pins
#define IN1 27
#define IN2 33
#define IN3 15  
#define IN4 32

Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

WiFiClient oEspClient;
PubSubClient oClient(oEspClient);

void interruptFunction() {
  char acMsg[100];
  Serial.println("How impolite to interrupt!");
  interruptsTotal += 1;
  sprintf(acMsg,"%i\n", interruptsTotal);
  sendInterrupCount = true;
  Serial.print(" Total interrupts: "); Serial.println(acMsg);
}

void fcMqttCallback(char* pcTopic, byte* pcPayload, unsigned int iLength)
{
  Serial.print("\nMessage arrived in topic: ");
  Serial.println(pcTopic);
  String topic = String(pcTopic);

  //if(strstr((char*)pcPayload,"remotebuttonState")!=0){
  //  Serial.println("Detected hello in message");
  //  oClient.publish("mechatronics\echo",pcTopic);
  //}
  Serial.print("Received MQTT message:");

  if(topic == "mechatronics/RemoteButton"){
    for (int i=0;i<iLength;i++){
      //Serial.print((topic)); Serial.println((char)pcPayload[i]);
      remotebuttonstatus = (char)pcPayload[i] - '0';
    }
  }else if(topic == "Niklas/pir"){
    char acMsg[100];
    //sprintf(string, "%d", (char)pcPayload);
    //Serial.print((char)pcPayload[0]);
    
    pirState = (char)pcPayload[0] - '0';
    sprintf(acMsg,"%i\n", pirState);
    Serial.print(acMsg);
    Serial.print("___");
    //for (int j=0;j<iLength;j++){
      //Serial.print((topic)); Serial.println((char)pcPayload[i]);
    //  pirState = (char)pcPayload[j];
      
    //}
  }
}

// the setup function runs once when you press reset or power the board
void setup() 
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.print("Initiating ESP32  ");
  Serial.print(__TIME__);
  Serial.print("  ");
  Serial.println(__DATE__);
  
  WiFi.begin(pcSSID,pcPASSWORD);
  while (WiFi.status() != WL_CONNECTED){
    delay(2000);
    Serial.print("Connecting to WiFi SSID:");
    Serial.println(pcSSID);
  }
  Serial.println("Connected to the WiFi network.");
  
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  oClient.setServer( pcMqttServer, iMqttPort);
  oClient.setCallback(fcMqttCallback);
  while (!oClient.connected()) {
    Serial.print("Connecting to MQTT...");
    Serial.print(pcMqttServer);
    if (oClient.connect("NiklasMainESP32", pcMqttUser, pcMqttPassword )) {
      Serial.println(".  Connection established."); 
    } 
    else {
      Serial.print(". failed to connect with state ");
      Serial.println(oClient.state());
      delay(2000);
    }
  }
  //oClient.subscribe("mechatronics/chat");
  oClient.subscribe("mechatronics/RemoteButton");
  oClient.subscribe("Niklas/pir");
  //oClient.subscribe("mechatronics/lab2");
  oClient.publish("Niklas/chat", "My ESP32 is initited and is up and running.");


  pinMode(buttonPin, INPUT);
  pinMode(ledLidarPin, OUTPUT);
  pinMode(pirLEDPin, OUTPUT);
  pinMode(pirPin, INPUT);
  pinMode(interruptPin, INPUT);
  attachInterrupt(interruptPin, interruptFunction, FALLING);
  
  
  myStepper.setSpeed(20);
  
  //Lidar
  Wire1.begin(21, 17);
  //Gyro
  Wire.begin(23, 22);
  imu.begin();

  //Setting up lidar
  if (!lox.begin(0x29, false, &Wire1))
  {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1); 
  }
  // power
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
  Serial.println("Sensor data are read");
  Serial.println("ESP32 is up and running.");
}

void loop() 
{
  static unsigned long ulNextTime=0+10000;   // Next time is 10 seconds after start
  static unsigned long ulNextTimePir=0+10000;
  static unsigned long ulNextTimePirSend=0+10000;
  char acMsg[100];
  char msg[100];

  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float t;

  imu.getAccel(&ax, &ay, &az);
  imu.getGyro(&gx, &gy, &gz);
  imu.getTemp(&t);

  buttonState = digitalRead(buttonPin);

  //Send Pir Receive MQTT
  long ulTimePirSend = millis();
  if(ulTimePirSend>=ulNextTimePirSend)
  {
    pirStateLocal = digitalRead(pirPin);
    sprintf(acMsg,"%d\n", pirStateLocal);
    Serial.print("Pub 0"); Serial.println(acMsg);
    oClient.publish("Niklas/pir", &acMsg[0]);
    ulNextTimePirSend += 1000;
  }
  


  //Send InterruptCount
  if(sendInterrupCount){
    sprintf(acMsg,"%i\n", interruptsTotal);
    oClient.publish("Niklas/hall", &acMsg[0]);
    sendInterrupCount = false;
  }
  //sprintf(acMsg,"%i\n", pirState);
  //Serial.print("See PIR"); Serial.println(acMsg);
  if(pirState == HIGH && newState == true)
  {
    Serial.println(" PIR HIGH ");
    digitalWrite(pirLEDPin, HIGH);
    ulNextTimePir += 3000;
    newState = false;

    //lightPirLED = true;
  }

  long ulTimePir = millis();
  if(ulTimePir>=ulNextTimePir && pirState == LOW)
  {
    //ulNextTimePir += 3000;
    digitalWrite(pirLEDPin, LOW);
    //Serial.println(" PIR LOW ");
    newState = true;
  }
  



  if(buttonState == HIGH && remotebuttonstatus == LOW){
    //Serial.println("clockwise");
    myStepper.step(stepsPerPress);
    stepperAngle = stepperAngle + (float)stepsPerPress * 360 / (float)stepsPerRevolution;
    sprintf(acMsg,"%f\n", stepperAngle);
    oClient.publish("Niklas/stepper", &acMsg[0]);
    //Serial.println("Stepper Angle: ");
    //Serial.print(stepperAngle);
    //Serial.println("_____");
  }else if(remotebuttonstatus == HIGH){
    myStepper.step(-stepsPerPress);
    //Serial.println("counterclockwise");
    stepperAngle = stepperAngle - (float)stepsPerPress * 360.0/ (float)stepsPerRevolution;
    sprintf(acMsg,"%f\n", stepperAngle);
    oClient.publish("Niklas/stepper", &acMsg[0]);
    //Serial.println("Stepper Angle: ");
    //Serial.print(stepperAngle);
    //Serial.println("_____");
  }
  
  
  long ulTime = millis();
  if(ulTime>=ulNextTime){
    static int iLoop=0;
    ulNextTime += 1000;
    if(abs(az) > gravityZ){
      angle = acos(1)*180/3.14;
    }else{
      angle = acos(az/gravityZ)*180/3.14;
    }
    
    
    //msg = "%f,%f,%f,%f,%f,%f,%f\n", ax, ay, az, gx, gy, gz, t;
    sprintf(acMsg,"%f,%f,%f,%f,%f,%f,%f\n", ax, ay, az, gx, gy, gz, t);
    oClient.publish("Niklas/gyro", &acMsg[0]);
    sprintf(acMsg,"%f", angle);
    oClient.publish("Niklas/angle", &acMsg[0]);

    //Measuring Distance with Lidar
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

    if (measure.RangeStatus != 4)
    { // phase failures have incorrect data
      distanceLidar = measure.RangeMilliMeter;
      //Serial.print("Distance (mm): "); Serial.println(distanceLidar);
      //Serial.println("");
      sprintf(acMsg,"%i\n", distanceLidar);
      oClient.publish("Niklas/lidar", &acMsg[0]);
      if(distanceLidar < 100){
        digitalWrite(ledLidarPin, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(1000);                       // wait for a second
        digitalWrite(ledLidarPin, LOW);
        //Serial.println("Distance Low");
      }else{
        digitalWrite(ledLidarPin, LOW);
      }
    }
    else
    {
      oClient.publish("Niklas/lidar", " out of range ");
      //Serial.println(" out of range ");
    }


    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // Toggle Diode, a way to show that the ESP is running
  }
  oClient.loop();
}
