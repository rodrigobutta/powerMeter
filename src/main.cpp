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

char* lastResetAt = "";


#define RXD2 16 
#define TXD2 17

uint8_t addr1=0x07;
uint8_t addr2=0x08;
uint8_t addr3=0x09;


// PZEM004Tv30 pzem(Serial2, RXD2, TXD2, addr1);

PZEM004Tv30 pzem[3] = {
  PZEM004Tv30(Serial2, RXD2,TXD2, addr1),
  PZEM004Tv30(Serial2, RXD2,TXD2, addr2),
  PZEM004Tv30(Serial2, RXD2,TXD2, addr3)
};


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

    // pzem.resetEnergy();

    // mqttSend("esp32/powerMeter/state/reseted", "");

    // struct tm timeinfo;
    // if(!getLocalTime(&timeinfo)){
    //   Serial.println("Failed to obtain time");
    //   return;
    // }

    // // char buff[20];
    // // time_t now = time(NULL);
    // strftime(lastResetAt, 20, "%Y-%m-%d %H:%M:%S", &timeinfo);

    // broadcastState();
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



void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
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
  
  Serial.print("PIN STATUS:");
  Serial.println(STATUS_LED_GPIO);

  Serial.print("PIN TEMP SENSOR:");
  Serial.println(oneWireBus);


  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

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

  StaticJsonDocument<1200> state;

  for (int i=0; i<3; i++){  


    delay(200);
    Serial.print("PZEM No. "); Serial.println(i+1);

    Serial.print("Connection Address:");
    Serial.println(pzem[i].getAddress());
    
    Serial.print("Device Address:");
    Serial.println(pzem[i].readAddress(), HEX);

    float voltage = pzem[i].voltage();
    float current = pzem[i].current();
    float power = pzem[i].power();
    float energy = pzem[i].energy();
    float frequency = pzem[i].frequency();
    float pf = pzem[i].pf();

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

        // Serial.print("Last Reseted At: "); Serial.println(lastResetAt, "%A, %B %d %Y %H:%M:%S");
        Serial.print("Last Reseted At: "); Serial.println(lastResetAt);

        std::string sensorNode = "sensor_" + std::to_string(i);
        JsonObject values  = state.createNestedObject(sensorNode);
          values["voltage"] = round(voltage * 100.0 ) / 100.0;
          values["current"] = round(current * 100.0 ) / 100.0;
          values["power"] = round(power * 100.0 ) / 100.0;        
          values["energy"] = round(energy * 100.0 ) / 100.0;
          values["frequency"] = round(frequency * 100.0 ) / 100.0;
          values["pf"] = round(pf * 100.0 ) / 100.0;        
    }

    char jsonString[512];
    serializeJson(state, jsonString);

    mqttSend("esp32/powerMeter/state", jsonString);

    Serial.println();
  }
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

  if (currentMillis - powerMeterPrevMillis >= 4000) {
    powerMeterPrevMillis = currentMillis;
    powerMeterLoop();
  }
}