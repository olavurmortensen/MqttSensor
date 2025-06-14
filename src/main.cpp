// Wi-Fi credentials (SSID and password) must be stored in a file that is not tracked
// by version control.
#include "wifi_credentials.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

#define DHTPIN 2
#define LED D0

#define MQTT_HOST IPAddress(192, 168, 11, 10)

#define MQTT_PORT 1883

#define MQTT_PUB_COUNT "esp/test/count"

// The sensor is initialized
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);
 
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

unsigned long previousMillis = 0;
const long interval = 1000;
unsigned long count = 0;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT ...");
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Conencted to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach();
  wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT");
  Serial.println("Session resent:");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if(WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.println("    packetId:");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(9600);

  Serial.println("KY-015 Test - Temperature and humidity test:");
  // Measurement is started
  dht.begin();
  Serial.println();

  pinMode(LED, OUTPUT);

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
}

bool ledState = false;
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    digitalWrite(LED, ledState ? HIGH : LOW);
    ledState = !ledState;

    // Humidity is measured
    float h = dht.readHumidity();
    // Temperature is measured
    float t = dht.readTemperature();
    // Checks whether the measurements have run through without errors
    // If an error is detected, an error message is output here
    if (isnan(h) || isnan(t)) {
      Serial.println("Error when reading out the sensor");
      return;
    }

    // Output to the serial console
    Serial.println("-----------------------------------------------------------");
    Serial.print(" Humidity: ");
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print(" Temperature: ");
    Serial.print(t);
    Serial.println(" °C ");
    Serial.println("-----------------------------------------------------------");
    Serial.println(" ");

    previousMillis = currentMillis;
    Serial.println();
    uint16_t packetIdPub = mqttClient.publish(MQTT_PUB_COUNT, 1, true, String(count).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i\n", MQTT_PUB_COUNT, packetIdPub);
    Serial.printf("Message: %ld\n", count);
    count = count + 1;
  }
}