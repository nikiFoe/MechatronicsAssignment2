/**
 * Mechatronics, 2022
 * This is a template for MQTT and ESP32.
 */

#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <string.h>
#include <Stepper.h>  

#include "I2C_MPU6886.h"

I2C_MPU6886 imu(I2C_MPU6886_DEFAULT_ADDRESS, Wire1);

const char* pcSSID = "iPSK-UMU"; // Enter your WiFi name
const char* pcPASSWORD =  "HejHopp!!"; // Enter WiFi password
const char* pcMqttServer = "tfe.iotwan.se";
const int   iMqttPort = 1883;
const char* pcMqttUser = "intro22";
const char* pcMqttPassword = "outro";
const double gravityZ = 1.05;
double angle = 0;



const int buttonPin = 14;
int buttonState = 0;
int remotebuttonstatus = 0;
double stepperAngle = 0; // in degree
int stepsPerPress = 8;

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution


// ULN2003 Motor Driver Pins
#define IN1 27
#define IN2 33
#define IN3 15  
#define IN4 32

Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

WiFiClient oEspClient;
PubSubClient oClient(oEspClient);

void fcMqttCallback(char* pcTopic, byte* pcPayload, unsigned int iLength)
{
  Serial.print("\nMessage arrived in topic: ");
  Serial.println(pcTopic);

  //if(strstr((char*)pcPayload,"remotebuttonState")!=0){
  //  Serial.println("Detected hello in message");
  //  oClient.publish("mechatronics\echo",pcTopic);
  //}
  Serial.print("Received MQTT message:");
  for (int i=0;i<iLength;i++){
    Serial.print((char)pcPayload[i]);
    remotebuttonstatus = (char)pcPayload[i] - '0';
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
  //oClient.subscribe("mechatronics/lab2");
  oClient.publish("Niklas/chat", "My ESP32 is initited and is up and running.");


  pinMode(buttonPin, INPUT);
  myStepper.setSpeed(20);

  Wire1.begin(23, 22);
  imu.begin();

  Serial.println("Sensor data are read");
  Serial.println("ESP32 is up and running.");
}

void loop() 
{
  static unsigned long ulNextTime=0+10000;   // Next time is 10 seconds after start
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
  //Serial.println(buttonState);

  if(buttonState == HIGH && remotebuttonstatus == LOW){
    Serial.println("clockwise");
    myStepper.step(stepsPerPress);
    stepperAngle = stepperAngle + (float)stepsPerPress * 360 / (float)stepsPerRevolution;
    sprintf(acMsg,"%f\n", stepperAngle);
    oClient.publish("Niklas/stepper", &acMsg[0]);
    Serial.println("Stepper Angle: ");
    Serial.print(stepperAngle);
    Serial.println("_____");
  }else if(remotebuttonstatus == HIGH){
    myStepper.step(-stepsPerPress);
    Serial.println("counterclockwise");
    stepperAngle = stepperAngle - 8.0 * 360.0/ 2048.0;
    sprintf(acMsg,"%f\n", stepperAngle);
    oClient.publish("Niklas/stepper", &acMsg[0]);
    Serial.println("Stepper Angle: ");
    Serial.print(stepperAngle);
    Serial.println("_____");
  }
  



  long ulTime = millis();
  if(ulTime>=ulNextTime){
    static int iLoop=0;
    ulNextTime += 5000;
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

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // Toggle Diode, a way to show that the ESP is running
  }
  oClient.loop();
}
