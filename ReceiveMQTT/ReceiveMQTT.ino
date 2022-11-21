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

WiFiClient oEspClient;
PubSubClient oClient(oEspClient);

void fcMqttCallback(char* pcTopic, byte* pcPayload, unsigned int iLength)
{
  Serial.print("\nMessage arrived in topic: ");
  Serial.println(pcTopic);
  //if(strstr((char*)pcPayload,"hello")!=0){
  //  Serial.println("Detected hello in message");
  //  oClient.publish("mechatronics\echo",pcTopic);
  //}
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
    if (oClient.connect("Acreibo", pcMqttUser, pcMqttPassword )) {
      Serial.println(".  Connection established."); 
    } 
    else {
      Serial.print(". failed to connect with state ");
      Serial.println(oClient.state());
      delay(2000);
    }
  }
  
  //oClient.publish("mechatronics/RemoteButton", 0);
  //oClient.subscribe("mechatronics/RemoteButton");
  oClient.publish("mechatronics/chat", "My ESP32 Receive is initited and is up and running.");
  //oClient.setBufferSize(2048);
  Serial.println("ESP32 is up and running.");
  oClient.subscribe("mechatronics/chat");
}

void reconnect()
{
  while (!oClient.connected()) {
    Serial.print("Connecting to MQTT...");
    Serial.print(pcMqttServer);
    if (oClient.connect("ReceiverNiklas", pcMqttUser, pcMqttPassword )) {
      Serial.println(".  Connection established."); 
    } 
    else {
      Serial.print(". failed to connect with state ");
      Serial.println(oClient.state());
      delay(2000);
    }
  }
}

void loop() 
{
  static unsigned long ulNextTime=0+10000;   // Next time is 10 seconds after start
  char acMsg[100];
  
  if(!oClient.connected()){
    reconnect();
  }

  long ulTime = millis();
  if(ulTime>=ulNextTime){
    static int iLoop=0;
    ulNextTime += 5000;
    Serial.println("Receive");
    Serial.println(oClient.connected());
    Serial.println("_______________");
  }
  oClient.loop();
  
}
