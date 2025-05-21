#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "hmmm";
const char* pswd = "selalusalah";

const char* mqtt_server = "192.168.1.129";

WiFiClient espClient;
PubSubClient client(espClient);

const int ledPin = 2;

#define topic_led = "led/1"

// put function declarations here:
void setupWiFi();
void connectIndicator();

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  Serial.begin(115200);
  WiFi.begin(ssid, pswd);
  setupWiFi();
}

void loop() {
  // put your main code here, to run repeatedly:
  connectIndicator();
  delay(500);
}

// put function definitions here:
void setupWiFi(){
  delay(10);
  Serial.print("\nConnecting to "); Serial.print(ssid); Serial.println("...") ;
  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(100);
  }
  Serial.print("\nConnected "); Serial.print(ssid); Serial.println("!");
  Serial.print("IP Address : "); Serial.println(WiFi.localIP());
}

void connectIndicator(){
  if(WiFi.status() == WL_CONNECTED){
    digitalWrite(ledPin, HIGH);
  }
}