// run mosquito :
// "C:\Program Files\mosquitto\mosquitto.exe" -c "C:\Program Files\mosquitto\mosquitto.conf" -v

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

const char* ssid = "hmmm";
const char* pass = "selalusalah";

const char* mqtt_server = "192.168.1.129";
const char* mqtt_client_id = "drip_irrigation_monitoring";
const char* topic_soil1 = "soil/1";
const char* topic_soil2 = "soil/2";
const char* topic_soil3 = "soil/3";
const char* topic_soilAvg = "soil/avg";
const char* topic_temp1 = "temp/1";
const char* topic_temp2 = "temp/2";
const char* topic_temp3 = "temp/3";
const char* topic_tempAvg = "temp/avg";
const char* topic_pump = "pump/control";
const char* topic_pump_status = "pump/control/status";
const char* topic_pump_mode = "pump/mode";
const char* topic_pump_mode_status = "pump/mode/status";

WiFiClient espClient;
PubSubClient client(espClient);

#define THRESHOLD_MAX_SOIL 65.0
#define THRESHOLD_MIN_SOIL 40.0
#define THRESHOLD_MAX_TEMP 35.0
#define THRESHOLD_MIN_TEMP 20.0

const int soilPin1 = 32;
const int soilPin2 = 35;
const int soilPin3 = 34;

#define tempPin 4
OneWire oneWire(tempPin);
DallasTemperature sensors(&oneWire);
DeviceAddress tempSensors[3];

const int relayPin = 18;
const int ledPin = 2;

bool pumpStatus = false;
bool pumpAutoMode = true;

unsigned long lastPublish = 0;
const unsigned long publishInterval = 3000; // publish tiap 3 detik

float soil1, soil2, soil3, avgSoil, temp1, temp2, temp3, avgTemp;

// put function declarations here:
void setupWifi();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
float readSoil(int sensorPin);
void publishDatasensors();
void printAddress(DeviceAddress deviceAddress);


void setup() {
  // put your setup code here, to run once:
  setupWifi();
  Serial.begin(115200);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(relayPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(relayPin, HIGH);
  digitalWrite(ledPin, LOW);

  sensors.begin();
  for(int i = 0; i < 3; i++){
    // jika tidak menemukan alamat sensor temperatur maka, cetak pada serial monitor ("sensor suhu tidak ditemukan")
    if(!sensors.getAddress(tempSensors[i], i)){
      Serial.print("Sensor suhu "); Serial.print(i); Serial.println(" Tidak ditemukan.");
    }
    // cetak pada serial monitor (alamat sensor suhu) ketika alamat sensor ditemukan
    else{
      Serial.print("Address sensor "); Serial.print(i); Serial.println(" : ");
      printAddress(tempSensors[i]);
      Serial.println();
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if(!client.connected()){
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastPublish >= publishInterval) {
    lastPublish = now;
    publishDatasensors(); // kirim data setiap 3 detik
  }
}

// put function definitions here: 
void setupWifi(){
  delay(10);
  Serial.print("Connecting to "); Serial.print(ssid); Serial.print("...");
  WiFi.begin(ssid, pass);
  while(WiFi.status() != WL_CONNECTED){
    delay(500); 
    Serial.print(".");
  }
  Serial.println("WiFi connected!");
}

void callback(char* topic, byte* payload, unsigned int length){
  payload[length] = '\0';
  String message = String((char*)payload);
  Serial.print("Message arrived ["); Serial.print(topic); Serial.print("]: ");
  Serial.println(message);

  if(String(topic) == topic_pump){
    if(!pumpAutoMode){
      if(message == "ON"){
        digitalWrite(relayPin, LOW);
        digitalWrite(ledPin, HIGH);
        pumpStatus = true;
        Serial.println("Pompa dinyalakan manual!"); 
        client.publish(topic_pump_status, "ON");
      }
        else if(message == "OFF"){
        digitalWrite(relayPin, HIGH);
        digitalWrite(ledPin, LOW);
        pumpStatus = false;
        Serial.println("Pompa dimatikan manual!");
        client.publish(topic_pump_status, "OFF");
      }
    }
    else {
      Serial.println("Abaikan perintah manual, sedang dalam mode otomatis!");
    }
  }

  if(String(topic) == topic_pump_mode){
    if(message == "AUTO"){
      pumpAutoMode = true;
      Serial.println("Mode pompa : OTOMATIS!");
      client.publish(topic_pump_mode_status, "AUTO");
    }
    else if(message == "MANUAL"){
      pumpAutoMode = false;
      Serial.println("Mode Pompa : MANUAL!");
      client.publish(topic_pump_mode_status, "MANUAL");
    }
  }
}

void reconnect(){
  while(!client.connected()){
    Serial.print("Connecting to MQTT...");
    if(client.connect(mqtt_client_id)){
      Serial.println("Connected!");
      client.subscribe(topic_pump);
      client.subscribe(topic_pump_mode);
    }
    else{
      Serial.print("Failed!, RC = "); Serial.println(client.state());
      delay(2000);
    }
  }
}

float readSoil(int sensorPin){
  int rawData = analogRead(sensorPin);
  return (1.0 - (rawData / 4095.0)) * 100.0;
}

void publishDatasensors(){
  //soil moisture
  soil1 = readSoil(soilPin1);
  char msgS1[10]; dtostrf(soil1, 4, 2, msgS1);
  client.publish(topic_soil1, msgS1);

  soil2 = readSoil(soilPin2);
  char msgS2[10]; dtostrf(soil2, 4, 2, msgS2);
  client.publish(topic_soil2, msgS2);

  soil3 = readSoil(soilPin3);
  char msgS3[10]; dtostrf(soil3, 4, 2, msgS3);
  client.publish(topic_soil3, msgS3);

  avgSoil = (soil1 + soil2 + soil3) / 3.0;
  char msgSavg[10]; dtostrf(avgSoil, 4, 2, msgSavg);
  client.publish(topic_soilAvg, msgSavg);

  //temperature
  sensors.requestTemperatures();
  temp1 = sensors.getTempC(tempSensors[0]);
  char msgT1[10]; dtostrf(temp1, 4, 2, msgT1);
  client.publish(topic_temp1, msgT1);

  temp2 = sensors.getTempC(tempSensors[1]);
  char msgT2[10]; dtostrf(temp2, 4, 2, msgT2);
  client.publish(topic_temp2, msgT2);

  temp3 = sensors.getTempC(tempSensors[2]);
  char msgT3[10]; dtostrf(temp3, 4, 2, msgT3);
  client.publish(topic_temp3, msgT3);

  avgTemp = (temp1 + temp2 + temp3) / 3.0;
  char msgTavg[10]; dtostrf(avgTemp, 4, 2, msgTavg);
  client.publish(topic_tempAvg, msgTavg);

  if(pumpAutoMode){
    if(avgSoil > THRESHOLD_MAX_SOIL || avgTemp < THRESHOLD_MIN_TEMP && pumpStatus){
      digitalWrite(relayPin, HIGH);
      digitalWrite(ledPin, LOW);
      pumpStatus = false;
      Serial.println("Pompa dimatikan otomatis (tanah cukup basah)");
      client.publish(topic_pump, "OFF");
      client.publish(topic_pump_status, "OFF");
    }
    else if(avgSoil < THRESHOLD_MIN_SOIL && !pumpStatus){
      digitalWrite(relayPin, LOW);
      digitalWrite(ledPin, HIGH);
      pumpStatus = true;
      Serial.println("Pompa dinyalakan otomatas (tanah kurang air)");
      client.publish(topic_pump, "ON");
      client.publish(topic_pump_status, "ON");
    }
  }
}

void printAddress(DeviceAddress deviceAddress){
  for(uint8_t i = 0; i < 8; i++){
    Serial.print(deviceAddress[i], HEX);
    if(i<7) Serial.print(":");
  }
}