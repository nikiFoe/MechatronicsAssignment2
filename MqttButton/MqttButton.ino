/**
 * Mechatronics, 2022
 * This is a template for MQTT and ESP32.
 */

#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <string.h>
const char* pcSSID = "iPSK-UMU"; // Enter your WiFi name
const char* pcPASSWORD =  "HejHopp!!"; // Enter WiFi password
const char* pcMqttServer = "tfe.iotwan.se";
const int   iMqttPort = 1883;
const char* pcMqttUser = "intro22";
const char* pcMqttPassword = "outro";

const int buttonPin = 14;
int buttonState = 0;
int prevbuttonState = 2;

WiFiClient oEspClient;
PubSubClient oClient(oEspClient);

void fcMqttCallback(char* pcTopic, byte* pcPayload, unsigned int iLength)
{
  Serial.print("\nMessage arrived in topic: ");
  Serial.println(pcTopic);
  if(strstr((char*)pcPayload,"hello")!=0){
    Serial.println("Detected hello in message");
    oClient.publish("mechatronics\echo",pcTopic);
  }
  Serial.print("Received MQTT message:");
  for (int i=0;i<iLength;i++){
    Serial.print((char)pcPayload[i]);
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
    if (oClient.connect("NiklasRemoteButton", pcMqttUser, pcMqttPassword )) {
      Serial.println(".  Connection established."); 
    } 
    else {
      Serial.print(". failed to connect with state ");
      Serial.println(oClient.state());
      delay(2000);
    }
  }



  
  //oClient.subscribe("mechatronics/lab2");
  oClient.publish("mechatronics/chat", "My ESP32Button is initited and is up and running.");
  //oClient.subscribe("mechatronics/chat");
  Serial.println("ESP32 is up and running.");
  pinMode(buttonPin, INPUT);
}

void loop() 
{
  static unsigned long ulNextTime=0+10000;   // Next time is 10 seconds after start
  char acMsg[100];

  buttonState = digitalRead(buttonPin);
  Serial.println(buttonState);

  long ulTime = millis();
  
  if (buttonState != prevbuttonState){
    sprintf(acMsg,"%d",buttonState);
    oClient.publish("mechatronics/RemoteButton", &acMsg[0] );
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // Toggle Diode, a way to show that the ESP is running
    Serial.println("Send");
    prevbuttonState = buttonState;
  }
  
  //delay(1000);
  oClient.loop();
}
