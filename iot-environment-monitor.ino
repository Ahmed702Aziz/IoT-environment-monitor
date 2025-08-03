#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <DHTesp.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <time.h>   

#define SLEEP_TIME_MICROS 60000000 

const char* mqtt_server = "94fbaebfe12e4662be30e6222d1af83d.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "ahmedaziz";
const char* mqtt_pass = "Aziz1234567";

// MQTT topics
const char* temp_topic = "youssef/temp";
const char* hum_topic = "youssef/hum";
const char* alert_topic = "youssef/alert";
const char* current_topic = "datacenter/current";
const char* current_alert_topic = "datacenter/alerts";

// Hardware Pins
const int DHT_PIN = D4;
const int DHT_POWER_PIN = D2;
const int LED_PIN = D3;
const int analogPin = A0;
const int ACS712_POWER_PIN = D5;

// Thresholds
const float MAX_TEMP = 40.0;
const float MIN_TEMP = 14.0;
const float MAX_HUM = 70.0;
const float MIN_HUM = 20.0;
const float MAX_CURRENT = 25.0;
const float MIN_CURRENT = 0.1;

// ACS712 Constants
const float sensitivity = 0.066;
const float vcc = 1.0;



const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 0;

WiFiClientSecure espClient;
PubSubClient mqtt(espClient);
DHTesp dht;

float vref = 0.5;
unsigned long lastReconnectAttempt = 0;

//FUNCTIONS 

void powerOnDHT() {
  digitalWrite(DHT_POWER_PIN, HIGH);
  delay(2000);
}

void powerOffDHT() {
  digitalWrite(DHT_POWER_PIN, LOW);
}

void connectWiFi() {
  WiFiManager wifiManager;
  wifiManager.setTimeout(120);
  IPAddress apIP(192, 168, 4, 1);
  wifiManager.setAPStaticIPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

  if (!wifiManager.autoConnect("NodeMCU-Sensor")) {
    Serial.println("WiFi failed, sleeping...");
    ESP.deepSleep(SLEEP_TIME_MICROS);
  }
  Serial.println("WiFi connected!");
}

void connectMQTT() {
  espClient.setInsecure();
  mqtt.setServer(mqtt_server, mqtt_port);
  int retries = 0;
  while (!mqtt.connected() && retries < 3) {
    if (mqtt.connect("NodeMCU-DHT22", mqtt_user, mqtt_pass)) {
      Serial.println("MQTT connected!");
    } else {
      Serial.print("MQTT failed, rc=");
      Serial.println(mqtt.state());
      delay(2000);
      retries++;
    }
  }
}

void publishData(const char* topic, float value) {
  StaticJsonDocument<128> doc;
  doc["value"] = value;
  char payload[128];
  serializeJson(doc, payload);

  if (mqtt.publish(topic, payload)) {
    Serial.print("Published to ");
    Serial.println(topic);
  } else {
    Serial.println("Publish failed!");
  }
}

void checkAlerts(float temp, float hum) {
  if (temp > MAX_TEMP || temp < MIN_TEMP || hum > MAX_HUM || hum < MIN_HUM) {
    StaticJsonDocument<256> alert;
    alert["device"] = "NodeMCU-DHT22";
    alert["temperature"] = temp;
    alert["humidity"] = hum;

    if (temp > MAX_TEMP) alert["message"] = "High temperature alert!";
    else if (temp < MIN_TEMP) alert["message"] = "Low temperature alert!";
    else if (hum > MAX_HUM) alert["message"] = "High humidity alert!";
    else alert["message"] = "Low humidity alert!";

    char alertPayload[256];
    serializeJson(alert, alertPayload);
    mqtt.publish(alert_topic, alertPayload);
  }
}

void calibrateACS712() {
  Serial.println("Calibrating ACS712... Ensure NO current is flowing!");
  delay(2000);
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    sum += analogRead(analogPin);
    delay(2);
  }
  vref = (sum / 500.0) / 1024.0 * vcc;
  Serial.print("Calibrated Vref: ");
  Serial.println(vref, 4);
}

float getACRMSCurrent() {
  long sumSquares = 0;
  for (int i = 0; i < 1000; i++) {
    float voltage = (analogRead(analogPin) / 1024.0 * vcc) - vref;
    sumSquares += voltage * voltage;
    delayMicroseconds(800);
  }
  float rmsVoltage = sqrt(sumSquares / 1000.0);
  return rmsVoltage / sensitivity;
}

void sendCurrentAlert(float current) {
  String alertMsg = (current > MAX_CURRENT)
    ? "OVERLOAD: " + String(current, 2) + "A (MAX: " + String(MAX_CURRENT) + "A)"
    : "POWER LOSS: " + String(current, 2) + "A (MIN: " + String(MIN_CURRENT) + "A)";
  mqtt.publish(current_alert_topic, alertMsg.c_str());
  Serial.println("ALERT: " + alertMsg);
}

void setup() {
  Serial.begin(115200);
  pinMode(DHT_POWER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ACS712_POWER_PIN, OUTPUT);
  digitalWrite(ACS712_POWER_PIN, HIGH);
  delay(2000);
  pinMode(analogPin, INPUT);
  digitalWrite(LED_PIN, HIGH);

  powerOnDHT();
  dht.setup(DHT_PIN, DHTesp::DHT22);

  connectWiFi();
  connectMQTT();

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("Waiting for NTP time...");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("\nNTP time set.");

  calibrateACS712();

  float temperature = dht.getTemperature();
  float humidity = dht.getHumidity();
  powerOffDHT();

  float current = getACRMSCurrent();

  publishData(temp_topic, temperature);
  publishData(hum_topic, humidity);

  now = time(nullptr);

  StaticJsonDocument<200> currentDoc;
  currentDoc["timestamp"] = now;
  currentDoc["current_A"] = current;
  currentDoc["vref"] = vref;
  char currentPayload[256];
  serializeJson(currentDoc, currentPayload);

  if (mqtt.publish(current_topic, currentPayload)) {
    Serial.print("Published current data: ");
    Serial.println(currentPayload);
  } else {
    Serial.println("Failed to publish current data");
  }

  checkAlerts(temperature, humidity);
  if (current > MAX_CURRENT || current < MIN_CURRENT) {
    sendCurrentAlert(current);
  }

  mqtt.disconnect();
  digitalWrite(LED_PIN, LOW);
  digitalWrite(ACS712_POWER_PIN, LOW);
  Serial.println("Entering deep sleep...");
  ESP.deepSleep(SLEEP_TIME_MICROS);
}

void loop() {
   //empty
}

