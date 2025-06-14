#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <Arduino.h>

#define LED 2

#define WIFI_SSID "Heilo"
#define WIFI_PASSWORD "potato12"

#define MQTT_HOST IPAddress(192, 168, 11, 10)

#define MQTT_PORT 1883

#define MQTT_PUB_COUNT "esp/test/count"

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

unsigned long previousMillis = 0;
const long interval = 10000;
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

/*
FIXME: Sensor setup here.
*/

void setup() {
  Serial.begin(115200);
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

void loop() {
  unsigned long currentMillis = millis();
  bool ledState = false;
  if (currentMillis - previousMillis >= interval) {
    digitalWrite(LED, ledState ? HIGH : LOW);
    ledState = !ledState;
    previousMillis = currentMillis;
    Serial.println();
    uint16_t packetIdPub = mqttClient.publish(MQTT_PUB_COUNT, 1, true, String(count).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i\n", MQTT_PUB_COUNT, packetIdPub);
    Serial.printf("Message: %ld\n", count);
    count = count + 1;
  }
}


/*
#include <Arduino.h>

#define LED 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED, HIGH);
  Serial.println("LED is on");
  delay(500);
  digitalWrite(LED, LOW);
  Serial.println("LED is off");
  delay(500);
}
*/

/***************************************************
  This is an example for the HTU21D-F Humidity & Temp Sensor

  Designed specifically to work with the HTU21D-F sensor from Adafruit
  ----> https://www.adafruit.com/products/1899

  These displays use I2C to communicate, 2 pins are required to
  interface
 ****************************************************/

/*
#include <Wire.h>
#include "Adafruit_HTU21DF.h"

// Connect Vin to 3-5VDC
// Connect GND to ground
// Connect SCL to I2C clock pin (A5 on UNO)
// Connect SDA to I2C data pin (A4 on UNO)

Adafruit_HTU21DF htu = Adafruit_HTU21DF();

void setup() {
  Serial.begin(9600);
  Serial.println("HTU21D-F test");

  if (!htu.begin()) {
    Serial.println("Couldn't find sensor!");
    while (1);
  }
}

void loop() {
    float temp = htu.readTemperature();
    float rel_hum = htu.readHumidity();
    Serial.print("Temp: "); Serial.print(temp); Serial.print(" C");
    Serial.print("\t\t");
    Serial.print("Humidity: "); Serial.print(rel_hum); Serial.println(" \%");
    delay(500);
}
*/