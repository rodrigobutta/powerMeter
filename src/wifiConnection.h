#ifndef WIFI_CONNECTION_H
  #define WIFI_CONNECTION_H
      
  #include <Arduino.h>
  #include <WiFi.h>
  #include "settings.h"

  void wifiSetup() {
    delay(10);
    
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }

#endif