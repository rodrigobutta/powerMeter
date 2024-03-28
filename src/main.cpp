#include <Arduino.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <string>
#include "time.h"

#include "settings.h"
#include "wifiConnection.h"
#include <PZEM004Tv30.h>



// char* lastResetAt = "";

#define SENSOR_COUNT 3 

#define RXD2 16 
#define TXD2 17

uint8_t addr1=0x07;
uint8_t addr2=0x08;
uint8_t addr3=0x09;


// PZEM004Tv30 pzem(Serial2, RXD2, TXD2, addr1);

PZEM004Tv30 pzem[SENSOR_COUNT] = {
  PZEM004Tv30(Serial2, RXD2,TXD2, addr1),
  PZEM004Tv30(Serial2, RXD2,TXD2, addr2),
  PZEM004Tv30(Serial2, RXD2,TXD2, addr3)
};

// StaticJsonDocument<1200> state;

int address[SENSOR_COUNT];
float voltage[SENSOR_COUNT];
float current[SENSOR_COUNT];
float power[SENSOR_COUNT];
float energy[SENSOR_COUNT];
float frequency[SENSOR_COUNT];
float pf[SENSOR_COUNT];
String resetedAt[SENSOR_COUNT];



int stateUpdateDelay = 4000;


unsigned long powerMeterPrevMillis = 0;


#define STATUS_LED_GPIO 2

WiFiClient espClient;
PubSubClient client(espClient);

int statusLedState = LOW;             // statusLedState used to set the LED
unsigned long statusLedPrevMillis = 0;        // will store last time LED was updated




bool welcomeBroadcast = false;

void mqttSend(char* topic, char* message) {
    Serial.print("Sending MQTT: ");
    Serial.println(message);
    client.publish(topic, message);
}

void broadcastState() {
  Serial.println("Broadcasting state to MQTT..");

  StaticJsonDocument<1200> state;

  for (int i=0; i < SENSOR_COUNT; i++){  

    std::string sensorNode = "sensor_" + std::to_string(i);
  
    JsonObject values  = state.createNestedObject(sensorNode);
      values["voltage"] = round(voltage[i] * 100.0 ) / 100.0;
      values["current"] = round(current[i] * 100.0 ) / 100.0;
      values["power"] = round(power[i] * 100.0 ) / 100.0;
      values["energy"] = round(energy[i] * 100.0 ) / 100.0;
      values["frequency"] = round(frequency[i] * 100.0 ) / 100.0;
      values["pf"] = round(pf[i] * 100.0 ) / 100.0;
      values["address"] = address[i];
      values["resetedAt"] = resetedAt[i];
    
  }

  char jsonString[512];
  serializeJson(state, jsonString);

  mqttSend("esp32/powerMeter/state", jsonString);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.println("Receiving MQTT..");
  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Payload: ");
  
  char payloadString[length+1];
 
  int i=0;
  for (i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    payloadString[i]=(char)payload[i];
  }
  payloadString[i] = 0; // Null termination
  Serial.println();
  
  StaticJsonDocument <256> payloadJson;
  deserializeJson(payloadJson,payload);
   
  if (String(topic) == "esp32/powerMeter/set") {

    for(int i=0; i < SENSOR_COUNT; i++){

      std::string sensorKey = "sensor_" + std::to_string(i);
      // Serial.print("sensorKey ");
      // Serial.println(sensorKey.c_str());

      const char* sensorValue = payloadJson[sensorKey];
      if (sensorValue) {
        //  Serial.print("sensorValue ");
        //  Serial.println(sensorValue);
       
        if(String(sensorValue) == "reset") {
          Serial.print("Reseting sensor ");
          Serial.println(i);
          
          pzem[i].resetEnergy();

          // mqttSend("esp32/powerMeter/state/reseted", "");

          struct tm timeinfo;
          if(!getLocalTime(&timeinfo)){
            Serial.println("Failed to obtain time");
            return;
          }          
          // Serial.print("The local time direct ");
          // Serial.println(&timeinfo, "%Y-%m-%d %H:%M:%S");
          char resetedAtString[50];
          strftime(resetedAtString, 50, "%Y-%m-%dT%H:%M:%S-03:00", &timeinfo);
          resetedAt[i] = resetedAtString;

          // Serial.print("Reseted at ");
          // Serial.println(resetedAtString);
        }

      }
    }

    const int newStateUpdateDelay = payloadJson["updateFrequency"];
    if (newStateUpdateDelay) {
      if(newStateUpdateDelay > 500) {
        stateUpdateDelay = newStateUpdateDelay;
      }
    }

    broadcastState();
  }

}



void mqttSetup() {
  client.setServer(MQTT_HOST, 1883);
  client.setCallback(mqttCallback);

  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32_POWERMETER",MQTT_USER,MQTT_PASSWORD)) {
      Serial.println("MQTT connected");

      client.setBufferSize(512);

      // Subscribe
      client.subscribe("esp32/powerMeter/set");

      // if(welcomeBroadcast == false) {
      //   welcomeBroadcast == true;
      //   broadcastState();
      // }
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 10 seconds");
      delay(10000);
    }
  }
}

void mqttLoop() {
  client.loop();
}


void stateSetup () {

  for (int i=0; i < SENSOR_COUNT; i++){  

    voltage[i] = 0;
    current[i] = 0;
    power[i] = 0;
    energy[i] = 0;
    frequency[i] = 0;
    pf[i] = 0;

    address[i] = -1;
    resetedAt[i] = "";

  }
}


// ########################################################
// MAIN
// ########################################################


void setup() {

  pinMode(STATUS_LED_GPIO, OUTPUT);
  digitalWrite(STATUS_LED_GPIO, HIGH);

  Serial.begin(115200);
  Serial.println("Rodrigo Butta PowerMeter");

  wifiSetup();
  mqttSetup();


  stateSetup();
  
  Serial.print("PIN STATUS:");
  Serial.println(STATUS_LED_GPIO);
}


void statusLedLoop() {

  if (statusLedState == LOW) {
    statusLedState = HIGH;
  } else {
    statusLedState = LOW;
  }

  digitalWrite(STATUS_LED_GPIO, statusLedState);

}

void powerMeterLoop() {

  for (int i=0; i < SENSOR_COUNT; i++){  
    // delay(200);
    // Serial.print("PZEM No. "); Serial.println(i+1);
    // Serial.print("Connection Address:");
    // Serial.println(pzem[i].getAddress());
    // Serial.print("Device Address:");
    // Serial.println(pzem[i].readAddress(), HEX);

    float voltageMeasured = pzem[i].voltage();
    float currentMeasured = pzem[i].current();
    float powerMeasured = pzem[i].power();
    float energyMeasured = pzem[i].energy();
    float frequencyMeasured = pzem[i].frequency();
    float pfMeasured = pzem[i].pf();

    if(isnan(voltageMeasured)){
      Serial.println("Error reading sensor");
    } else {
      address[i] = pzem[i].readAddress();
      voltage[i] = voltageMeasured;
      current[i] = currentMeasured;
      power[i] = powerMeasured;
      energy[i] = energyMeasured;
      frequency[i] = frequencyMeasured;
      pf[i] = pfMeasured;
    }
  }

  broadcastState();
}


void loop() {
  if (!client.connected()) {
    esp_restart();
  }

  mqttLoop();

  unsigned long currentMillis = millis();
  if (currentMillis - statusLedPrevMillis >= 1000) {
    statusLedPrevMillis = currentMillis;
    statusLedLoop();
  }

  if (currentMillis - powerMeterPrevMillis >= stateUpdateDelay) {
    powerMeterPrevMillis = currentMillis;
    powerMeterLoop();
  }
}