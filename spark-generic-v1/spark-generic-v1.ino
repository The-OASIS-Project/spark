/*
 * This file is part of the OASIS Project.
 * https://github.com/orgs/The-OASIS-Project/
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * By contributing to this project, you agree to license your contributions
 * under the GPLv3 (or any later version) or any future licenses chosen by
 * the project author(s). Contributions include any modifications,
 * enhancements, or additions to the project. These contributions become
 * part of the project and are adopted by the project author(s).
 */

//#define DEBUG

#define SUPPRESS_NOISE

#define CONN_RETRY_ATTEMPTS  5

#include <WiFi.h>
#include <ArduinoMqttClient.h>

#include <ArduinoJson.h>

#include <Adafruit_AHTX0.h>

#include "arduino_secrets.h"

Adafruit_AHTX0 aht;

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

// To connect with SSL/TLS:
// 1) Change WiFiClient to WiFiSSLClient.
// 2) Change port value from 1883 to 8883.
// 3) Change broker value to a server with a known SSL/TLS root certificate 
//    flashed in the WiFi module.

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

String brokerString;
const char *broker = NULL;
int port = 1883;

const char topic[]  = "shoulder/left";

const long interval = 100;
unsigned long previousMillis = 0, previousFireMillis = 0;

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32S2)
#define VOLT_PIN    A2
#else /* XIAO-ESP32-C3 */
#define VOLT_PIN    A0
#endif

void setup() {
   int retries = 0;
  
   // Initialize serial
   Serial.begin(115200);

   if (! aht.begin()) {
      Serial.println("Could not find AHT? Check wiring");
      while (1) {
         delay(10);
      }
   }
   Serial.println("AHT10 or AHT20 found");

   // attempt to connect to WiFi network:
   Serial.print("Attempting to connect to WPA SSID: ");
   Serial.println(ssid);
   WiFi.begin(ssid, pass);
   while (WiFi.status() != WL_CONNECTED) {
      // failed, retry
      Serial.print(".");
      delay(5000);
      retries++;
      if (retries == CONN_RETRY_ATTEMPTS) {
         Serial.println("Retry timeout.");
         break;
      }
   }
  
   Serial.println("WiFi connected");
   Serial.println("IP address: ");
   Serial.println(WiFi.localIP());

   Serial.print("GATEWAY: ");
   Serial.println(WiFi.gatewayIP());

   // You can provide a unique client ID, if not set the library uses Arduino-millis()
   // Each client must have a unique client ID
   // mqttClient.setId("clientId");

   // You can provide a username and password for authentication
   // mqttClient.setUsernamePassword("username", "password");

   brokerString = WiFi.gatewayIP().toString();
   broker = brokerString.c_str();

   Serial.print("Attempting to connect to the MQTT broker: ");
   Serial.println(broker);

   if (!mqttClient.connect(broker, port)) {
      Serial.print("MQTT connection failed! Error code = ");
      Serial.println(mqttClient.connectError());
   } else {
      Serial.println("You're connected to the MQTT broker!");
      Serial.println();
   }

   // Serial.print("Subscribing to topic: ");
   Serial.println(topic);
   Serial.println();

   // subscribe to a topic
   // mqttClient.subscribe(topic);

   // topics can be unsubscribed using:
   // mqttClient.unsubscribe(topic);

   // Serial.print("Waiting for messages on topic: ");
   // Serial.println(topic);
   // Serial.println();
}

unsigned long previousSendMillis = 0; // will store last time the messages were sent
const long sendInterval = 5000;       // interval at which to execute code block (milliseconds)

void loop() {
   static StaticJsonDocument<256> send_doc;
   static StaticJsonDocument<256> recv_doc;
   static char input_json[256];
   static char output_json[256];
   sensors_event_t humidity, temp;

#if 0
   // check for incoming messages
   int messageSize = mqttClient.parseMessage();

   if (messageSize) {
      // we received a message, print out the topic and contents
      Serial.print("Received a message with topic '");
      Serial.print(mqttClient.messageTopic());
      Serial.print("', length ");
      Serial.print(messageSize);
      Serial.println(" bytes:");

      // use the Stream interface to print the contents
      //while (mqttClient.available()) {
         //Serial.print((char)mqttClient.read());
         mqttClient.read((uint8_t *)input_json, 256);
         Serial.println(input_json);
         DeserializationError error = deserializeJson(recv_doc, input_json);
         if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.c_str());
         } else {
            const char* device = recv_doc["device"];
            int value = recv_doc["value"];
            recv_doc.clear();
            Serial.print("Device: ");
            Serial.print(device);
            Serial.print(" Value: ");
            Serial.println(value);
         }
   //}
   //}
   //Serial.println();

   Serial.println();
  }
#endif

   unsigned long currentMillis = millis();

   if (currentMillis - previousSendMillis >= sendInterval) {
      previousSendMillis = currentMillis;

      // Read the voltage in millivolts from pin A2
      uint32_t voltageMilliVolts = analogReadMilliVolts(VOLT_PIN);

      // Convert millivolts to volts for easier reading
      float voltage = 2 * voltageMilliVolts / 1000.0;

      send_doc["device"] =  topic;
      send_doc["voltage"] = roundf(voltage * 100) / 100.0;
      serializeJson(send_doc, output_json, 256);
      send_doc.clear();

      Serial.print(broker);
      Serial.print(":");
      Serial.print(port);
      Serial.print("/");
      Serial.println(topic);
      Serial.println(output_json);

      // send message, the Print interface can be used to set the message contents
      mqttClient.beginMessage(topic);
      mqttClient.print(output_json);
      mqttClient.endMessage();

      /* Get temp from device */
      aht.getEvent(&humidity, &temp);

      send_doc["device"] =   topic;
      send_doc["temp"] = roundf(temp.temperature * 100) / 100.0;
      serializeJson(send_doc, output_json, 256);
      send_doc.clear();

      Serial.print(broker);
      Serial.print(":");
      Serial.print(port);
      Serial.print("/");
      Serial.println(topic);
      Serial.println(output_json);

      // send message, the Print interface can be used to set the message contents
      mqttClient.beginMessage(topic);
      mqttClient.print(output_json);
      mqttClient.endMessage();
   }
}
