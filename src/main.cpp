#include <Arduino.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <string>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "settings.h"
#include "wifiConnection.h"
#include <PZEM004Tv30.h>

#define RXD2 16 
#define TXD2 17


PZEM004Tv30 pzem(Serial2, RXD2, TXD2);

unsigned long powerMeterPrevMillis = 0;


#define STATUS_LED_GPIO 2

WiFiClient espClient;
PubSubClient client(espClient);
// StaticJsonDocument<1200> state;

int statusLedState = LOW;             // statusLedState used to set the LED
unsigned long statusLedPrevMillis = 0;        // will store last time LED was updated


// #############################
// ######## TEMP SENSOR ########
// #############################

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;     

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

unsigned long sensorPrevMillis = 0;



bool welcomeBroadcast = false;

void mqttSend(char* topic, char* message) {
    Serial.print("Sending MQTT: ");
    Serial.println(message);
    client.publish(topic, message);
}

void broadcastState() {
  Serial.println("Broadcasting state to MQTT..");
 
  // char jsonString[256]; // max 256 o no se manda el MQTT!!!!  
  // serializeJson(state, jsonString);

  // mqttSend("esp32/powerMeter/state", jsonString);

  // mqttSend("esp32/powerMeter/state", "{\"temp\":12}");
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
   
  if (String(topic) == "esp32/powerMeter/reset") {

    pzem.resetEnergy();

    mqttSend("esp32/powerMeter/state/reseted", "");

    broadcastState();
  }

}

void mqttReconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32_POWERMETER",MQTT_USER,MQTT_PASSWORD)) {
      Serial.println("MQTT connected");

      // Subscribe
      client.subscribe("esp32/powerMeter/reset");

      if(welcomeBroadcast == false) {
        welcomeBroadcast == true;
        broadcastState();
      }
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(10000);
    }
  }
}


void mqttSetup() {
  client.setServer(MQTT_HOST, 1883);
  client.setCallback(mqttCallback);
}

void mqttLoop() {
  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();
}


// ########################################################
// MAIN
// ########################################################


void setup() {

  pinMode(STATUS_LED_GPIO, OUTPUT);
  digitalWrite(STATUS_LED_GPIO, HIGH);

  Serial.begin(115200);
  Serial.println("Rodrigo Butta AC Controller");

  wifiSetup();
  mqttSetup();
  
  Serial.print("PIN STATUS:");
  Serial.println(STATUS_LED_GPIO);

  Serial.print("PIN TEMP SENSOR:");
  Serial.println(oneWireBus);

  // Start the DS18B20 sensor
  sensors.begin();
}


void statusLedLoop() {

  if (statusLedState == LOW) {
    statusLedState = HIGH;
  } else {
    statusLedState = LOW;
  }

  digitalWrite(STATUS_LED_GPIO, statusLedState);

}


void sensorLoop() {

  // sensors.requestTemperatures(); 
  // float temperatureC = sensors.getTempCByIndex(0);
  // // float temperatureF = sensors.getTempFByIndex(0);
  // Serial.print(temperatureC);
  // Serial.println("ºC");

}


void powerMeterLoop() {
    Serial.print("Custom Address:");
    Serial.println(pzem.readAddress(), HEX);

    float voltage = pzem.voltage();
    float current = pzem.current();
    float power = pzem.power();
    float energy = pzem.energy();
    float frequency = pzem.frequency();
    float pf = pzem.pf();

    if(isnan(voltage)){
        Serial.println("Error reading voltage");
    } else if (isnan(current)) {
        Serial.println("Error reading current");
    } else if (isnan(power)) {
        Serial.println("Error reading power");
    } else if (isnan(energy)) {
        Serial.println("Error reading energy");
    } else if (isnan(frequency)) {
        Serial.println("Error reading frequency");
    } else if (isnan(pf)) {
        Serial.println("Error reading power factor");
    } else {
        Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
        Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
        Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
        Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
        Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
        Serial.print("PF: ");           Serial.println(pf);

        StaticJsonDocument<1200> state;
        
        state["voltage"] = round(voltage * 100.0 ) / 100.0;
        state["current"] = round(current * 100.0 ) / 100.0;
        state["power"] = round(power * 100.0 ) / 100.0;
        state["energy"] = round(energy * 100.0 ) / 100.0;
        state["frequency"] = round(frequency * 100.0 ) / 100.0;
        state["pf"] = round(pf * 100.0 ) / 100.0;

        char jsonString[256]; // max 256 o no se manda el MQTT!!!!  
        serializeJson(state, jsonString);

        mqttSend("esp32/powerMeter/state", jsonString);
    }

    Serial.println();
}


void loop() {
  mqttLoop();

  long interval = 2000;
  if(!client.connected()) {
    interval = 300;
  }

  unsigned long currentMillis = millis();

  if (currentMillis - statusLedPrevMillis >= 1000) {
    statusLedPrevMillis = currentMillis;
    statusLedLoop();
  }

  if (currentMillis - sensorPrevMillis >= 5000) {
    sensorPrevMillis = currentMillis;
    sensorLoop();
  }

  if (currentMillis - powerMeterPrevMillis >= 2000) {
    powerMeterPrevMillis = currentMillis;
    powerMeterLoop();
  }
}