/*esp 32’s + power bank is battery station. code esp 32 so that it subscribes to the topic “req” and whenever something is 
published on it, it publishes a json string to the topic “Sta1” or “Sta2”. 
also, we want it to serial print its mac address when it boots, and saves the ssid, pass and ip addy of MQTT broker 
when put in the serial monitor so it can connect. and it should serial print its ip when connected to the wifi
and it should also be streaming the video on its ip but that feature is not that needed its just a good to have
maybe the json string should also include an analog read value from a pin*/

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <OneWire.h>

#define ONE_WIRE_PIN 21
#define BUTTON_JSON_PIN 19
#define BUTTON_ABORT_PIN 18
#define LED_PIN 2

//Topics
const char* topicSta1 = "Sta1";
const char* topicSta2 = "Sta2";
const char* topicReq = "req";
const char* topicAttendance = "Attendance";
const char* ssid = "LunarIoTNet";
const char* password = "LunarIoTNet";
const char* mqtt_server = "10.3.141.1";

WiFiClient espClient;
PubSubClient client(espClient);
OneWire ds(ONE_WIRE_PIN);
byte deviceAddress[8];
long lastMsg = 0;
char msg[50];
int value = 0;
const int analogPin = 34;

const char* jsonA = R"({
  "Version": "1",
  "battery_type": "1",
  "capacity_mAh": 15000,
  "nominal_voltage": 12.8,
  "num_cells": 4,
  "max_charge_current": 15.0,
  "max_discharge_current": 30.0,
  "internal_resistance": 0.005,
  "temp_limit_C": 60,
  "voltage_max": 14.6,
  "voltage_min": 10.0,
  "thermal_conductivity": 0.30,
  "heat_capacity": 800.0,
  "cycle_count": 100,
  "unique_id": "RVR-PK2",
  "created_date": 1721100000,
  "CRC": 0
})";

const char* jsonB = R"({
  "Version": "2",
  "battery_type": "2",
  "capacity_mAh": 10000,
  "nominal_voltage": 5,
  "num_cells": 2,
  "max_charge_current": 2,
  "max_discharge_current": 3,
  "internal_resistance": 0.010,
  "temp_limit_C": 65,
  "voltage_max": 5.5,
  "voltage_min": 4.3,
  "thermal_conductivity": 0.40,
  "heat_capacity": 810.0,
  "cycle_count": 50,
  "unique_id": "RVR-PK3",
  "created_date": 1721100000,
  "CRC": 0
})";

String getInput() {
  String input = "";
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();    
      if (c == '\n') {
        if (input.length() > 0) break;         
      } else if (c == 8 || c == 127) {
        if (input.length() > 0) {
          input.remove(input.length() - 1);
          Serial.print("\b \b");
        }
      } else {
        input += c;
        Serial.write(c);                
      }
    }
  }
  input.trim();
  Serial.println("Received input.\n");
  return input;
}

void readMacAddress() {
    uint8_t baseMac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
    if (ret == ESP_OK) {
      Serial.printf("\nMAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    } else {
      Serial.println("Failed to read MAC address");
    }
}

void setup_wifi() {
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected\n");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(WiFi.macAddress());

    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(topicReq);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

/*bool findDS2431() {
  ds.reset_search();
  while (ds.search(deviceAddress)) {
    if (OneWire::crc8(deviceAddress, 7) != deviceAddress[7]) continue;
    if (deviceAddress[0] == 0x2D) return true;
  }
  return false;
}

void printROM(byte* addr) {
  for (int i = 0; i < 8; i++) {
    if (addr[i] < 16) Serial.print("0");
    Serial.print(addr[i], HEX);
    if (i < 7) Serial.print(":");
  }
  Serial.println();
}

const char* getEEPROMJson() {
  if (!findDS2431()) {
    Serial.println("DS2431 not found.");
    return nullptr;
  }

  Serial.print("Active DS2431 ");
  printROM(deviceAddress);

  const uint8_t checkLen = 10;
  char buffer[checkLen + 1] = {0};

  ds.reset();
  ds.select(deviceAddress);
  ds.write(0xF0);
  ds.write(0x00);
  ds.write(0x00);

  for (int i = 0; i < checkLen; i++) {
    buffer[i] = ds.read();
  }
  buffer[checkLen] = '\0';

  Serial.print("EEPROM Prefix: ");
  Serial.println(buffer);

  if (strncmp(buffer, "{\"temp", 6) == 0) {
    return jsonA;
  } else if (strncmp(buffer, "{\"humidi", 8) == 0) {
    return jsonB;
  } else {
    Serial.println("Unknown EEPROM contents");
    return nullptr;
  }
}

void publishJSON() {
  const char* jsonToSend = getEEPROMJson();
  if (!jsonToSend) return;

  // Publish selected JSON to Sta1 or Sta2
  client.publish(topicSta1, jsonToSend);
  Serial.println("Published to Sta1");
}*/

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageString;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageString += char(message[i]);
  }
  Serial.println();

  if (String(topic) == topicReq) {
    const char* jsonToSend = jsonB;
    client.publish(topicSta2, jsonToSend);
    Serial.print("Published to ");
    Serial.print(topic);
  }

}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  while (!Serial);
  delay(2000);
  Serial.println("Booting...\n");
  Serial.flush();

  setup_wifi();
  readMacAddress();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();
}