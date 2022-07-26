#include <Arduino.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <string>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "time.h"

#include "settings.h"
#include "wifiConnection.h"
#include <PZEM004Tv30.h>



// TIME


const char* ntpServer = "south-america.pool.ntp.org";
const long  gmtOffset_sec = -10800;
const int   daylightOffset_sec = 0;

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



unsigned long powerMeterPrevMillis = 0;


#define STATUS_LED_GPIO 2

WiFiClient espClient;
PubSubClient client(espClient);

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
DallasTemperature temperatureSensors(&oneWire);

unsigned long sensorPrevMillis = 0;



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
          strftime(resetedAtString, 50, "%Y-%m-%d %H:%M:%S", &timeinfo);
          resetedAt[i] = resetedAtString;

          // Serial.print("Reseted at ");
          // Serial.println(resetedAtString);
        }

      }
    }

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
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
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



void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }

  Serial.print("Local time is: ");
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
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

  Serial.print("PIN TEMP SENSOR:");
  Serial.println(oneWireBus);


  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  // Start the DS18B20 sensor
  temperatureSensors.begin();
}


void statusLedLoop() {

  if (statusLedState == LOW) {
    statusLedState = HIGH;
  } else {
    statusLedState = LOW;
  }

  digitalWrite(STATUS_LED_GPIO, statusLedState);

}


void temperatureSensorsLoop() {

  // temperatureSensors.requestTemperatures(); 
  // float temperatureC = temperatureSensors.getTempCByIndex(0);
  // // float temperatureF = temperatureSensors.getTempFByIndex(0);
  // Serial.print(temperatureC);
  // Serial.println("ºC");

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

  // if (currentMillis - sensorPrevMillis >= 5000) {
  //   sensorPrevMillis = currentMillis;
  //   temperatureSensorsLoop();
  // }

  if (currentMillis - powerMeterPrevMillis >= 4000) {
    powerMeterPrevMillis = currentMillis;
    powerMeterLoop();
  }
}