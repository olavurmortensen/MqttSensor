#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <Arduino.h>

#define LED 2
#define DH_PIN 16 // D0

int data[5];

byte read_data() {
  byte i = 0;
  byte result = 0;
  for (i = 0; i < 8; i++) {
      while (digitalRead(DH_PIN) == LOW); // wait 50us
      delayMicroseconds(30); //The duration of the high level is judged to determine whether the data is '0' or '1'
      if (digitalRead(DH_PIN) == HIGH)
        result |= (1 << (8 - i)); //High in the former, low in the post
    while (digitalRead(DH_PIN) == HIGH); //Data '1', waiting for the next bit of reception
    }
  return result;
}

void start_test() {
  digitalWrite(DH_PIN, LOW); //Pull down the bus to send the start signal
  delay(30); //The delay is greater than 18 ms so that DHT 11 can detect the start signal
  digitalWrite(DH_PIN, HIGH);
  delayMicroseconds(40); //Wait for DHT11 to respond
  pinMode(DH_PIN, INPUT);
  while(digitalRead(DH_PIN) == HIGH);
  delayMicroseconds(80); //The DHT11 responds by pulling the bus low for 80us;
  
  if(digitalRead(DH_PIN) == LOW)
    delayMicroseconds(80); //DHT11 pulled up after the bus 80us to start sending data;
  for(int i = 0; i < 5; i++) //Receiving temperature and humidity data, check bits are not considered;
    data[i] = read_data();
  pinMode(DH_PIN, OUTPUT);
  digitalWrite(DH_PIN, HIGH); //After the completion of a release of data bus, waiting for the host to start the next signal
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  pinMode(LED, OUTPUT);
  pinMode(DH_PIN, OUTPUT);
}

bool ledState = false;
void loop() {
  digitalWrite(LED, ledState ? HIGH : LOW);
  ledState = !ledState;
  Serial.println();
  Serial.printf("LED: %d\n",ledState);
  start_test();
  Serial.print("Humdity = ");
  Serial.print(data[0], DEC); //Displays the integer bits of humidity;
  Serial.print('.');
  Serial.print(data[1], DEC); //Displays the decimal places of the humidity;
  Serial.println('%');
  Serial.print("Temperature = ");
  Serial.print(data[2], DEC); //Displays the integer bits of temperature;
  Serial.print('.');
  Serial.print(data[3], DEC); //Displays the decimal places of the temperature;
  Serial.println('C');

  byte checksum = data[0] + data[1] + data[2] + data[3];
  if (data[4] != checksum) 
    Serial.println("-- Checksum Error!");
  else
    Serial.println("-- OK");
 
  delay(1000);
}
/*
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

*/