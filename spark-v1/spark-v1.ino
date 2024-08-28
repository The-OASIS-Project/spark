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

#include "esp_adc_cal.h"
#define CONN_RETRY_ATTEMPTS  10

#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
   #include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
   #include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
   #include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
   #include <WiFi.h>
   #include "esp_wifi.h"
#endif

#include <Adafruit_LSM6DSO32.h>
#include <ArduinoJson.h>

//#define FASTLED_ALL_PINS_HARDWARE_SPI
//#define FASTLED_ESP32_SPI_BUS SPI
#include <SPI.h>
#include <FastLED.h>

#include "arduino_secrets.h"
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

const char topic[]  = "repulsor/right";

const long interval = 100;
unsigned long previousMillis = 0, previousFireMillis = 0;

Adafruit_LSM6DSO32 lsm6ds; // can use any LSM6DS/ISM330 family chip!
Adafruit_Sensor *lsm_temp, *lsm_accel;

// LED DEFINES
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    19
#define NUM_INSIDE  7
#define NUM_OUTSIDE (NUM_LEDS-NUM_INSIDE)
#define BRIGHTNESS  80
#define FIRE_BRIGHTNESS   190
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32S2)
#define LED_PIN     35  /* MOSI */
#define VOLT_PIN    A2
#else /* XIAO-ESP32-C3 */
#define LED_PIN     10  /* MOSI */
#define VOLT_PIN    A0
#endif

#define BLAST_DURATION 1057

// STATES
enum states { POWER_OFF, POWERING_UP, POWERING_DOWN, POWER_ON };

uint8_t power_switch_state;
uint8_t fire_switch_state;
int firing;
uint8_t flash_delay;
uint8_t led_pin;
uint8_t state;
uint8_t last_state;
CRGB leds[NUM_LEDS];
uint8_t light_level;

// TODO: Change the delay to something fixed, time accordingly.
#define HAND_UP_TIME 1.064
#define HAND_STEP_SIZE 15
double hand_delay = (1000 * HAND_UP_TIME) / (255 / HAND_STEP_SIZE);

// SOUND FILE DEFINES
#define HAND_UP_SOUND      "hand_rep_on.ogg"
#define HAND_BLAST_SOUND   "hand_rep_fire2.ogg"
#define HAND_DOWN_SOUND    "hand_rep_off.ogg"

#define PLAY_UP_CMD        "play " HAND_UP_SOUND
#define PLAY_BLAST_CMD     "play " HAND_BLAST_SOUND
#define PLAY_DOWN_CMD      "play " HAND_DOWN_SOUND

#define STOP_UP_CMD        "stop " HAND_UP_SOUND
#define STOP_DOWN_CMD      "stop " HAND_DOWN_SOUND

// LED GLOBALS
CRGB outside_leds[NUM_LEDS];
CRGB inside_leds[NUM_LEDS];
CRGB all_on[NUM_LEDS];
CRGB all_off[NUM_LEDS];
CRGBPalette16 currentPalette;
TBlendType     currentBlending;
CLEDController *repulsor;

void setup() {
   int retries = 0;
  
   //Initialize serial and wait for port to open:
   Serial.begin(115200);

   /* Accel/Gyro Sensor Setup */
   Serial.println("Adafruit LSM6DS setup!");

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32S2)
   Wire1.setPins(SDA1, SCL1);

   if (!lsm6ds.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &Wire1, 0)) {
#else
   if (!lsm6ds.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &Wire, 0)) {
#endif
      Serial.println("Failed to find LSM6DS chip");
      while (1) {
         delay(10);
      }
   }

   Serial.println("LSM6DS Found!");

   lsm6ds.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
   lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS );
   lsm6ds.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
   lsm6ds.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  
   lsm_temp = lsm6ds.getTemperatureSensor();
   lsm_temp->printSensorDetails();

   lsm_accel = lsm6ds.getAccelerometerSensor();
   lsm_accel->printSensorDetails();
   /* End Sensor Setup */

#if defined(NEOPIXEL_POWER)
   // If this board has a power control pin, we must set it to output and high
   // in order to enable the NeoPixels. We put this in an #if defined so it can
   // be reused for other boards without compilation errors
   pinMode(NEOPIXEL_POWER, OUTPUT);
   digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

   repulsor = &FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(CRGB(225,255,255));
   memmove( &leds, &all_off, NUM_LEDS * sizeof(CRGB) );
   leds[12] = CRGB::Red;
   repulsor->showLeds(20);

   /* FastLED Setup */
   // set outside white
   for ( int i=0; i<NUM_OUTSIDE; i++ )
   {
      outside_leds[i] = CRGB::White; 
   }

   // set inside white
   for ( int i=NUM_OUTSIDE; i<NUM_LEDS; i++ )
   {
      inside_leds[i] = CRGB::White;
   }

   // set solids
   fill_solid(all_off, NUM_LEDS, CRGB::Black);
   fill_solid(all_on, NUM_LEDS, CRGB::White);

   // Populate Repulsor Data
   power_switch_state = 0;
   fire_switch_state = 0;
   firing = 0;
   flash_delay = 0;
   led_pin = LED_PIN;
   state = POWER_OFF;
   last_state = POWER_OFF;
   memmove( &leds, &all_off, NUM_LEDS * sizeof(CRGB) );
   light_level = 0;

#if 0
   // Run start-up test
//   for (int s = R_ID_START; s < R_ID_END; s++)
//   {
      for ( int l=0; l<(NUM_LEDS+4); l++ )
      {
         if ( l < NUM_LEDS )
         {
            Serial.print("Writing Red to ");
            Serial.println(l);
            leds[l] = CRGB::Red;
         }
         if ( (l>0) && ((l-1) < NUM_LEDS) )
         {
            Serial.print("Writing Green to ");
            Serial.println(l-1);
            leds[l-1] = CRGB::Green;
         }
         if ( (l>1) && ((l-2) < NUM_LEDS) )
         {
            Serial.print("Writing Blue to ");
            Serial.println(l-2);
            leds[l-2] = CRGB::Blue;
         }
         if ( (l>2) && ((l-3) < NUM_LEDS) )
         {
            Serial.print("Writing White to ");
            Serial.println(l-3);
            leds[l-3] = CRGB::White;
         }
         if ( (l>3) && ((l-4) < NUM_LEDS) )
         {
            Serial.print("Writing Black to ");
            Serial.println(l-4);
            leds[l-4] = CRGB::Black;
         }
         Serial.println("A");
         repulsor->showLeds( BRIGHTNESS );
         Serial.println("B");
         delay( 100 );
      }
      Serial.println("C");
      memmove( &leds, &all_off, NUM_LEDS * sizeof(CRGB) );
      Serial.println("D");
      repulsor->showLeds( BRIGHTNESS );
      Serial.println("E");
//   }
#endif

   // attempt to connect to WiFi network:
   Serial.print("Attempting to connect to WPA SSID: ");
   Serial.println(ssid);
   WiFi.begin(ssid, pass);
   retries = 0;
   while (WiFi.status() != WL_CONNECTED) {
      // failed, retry
      Serial.print(".");
      leds[12] = CRGB::Black;
      repulsor->showLeds(20);
      delay(50);
      leds[12] = CRGB::Red;
      repulsor->showLeds(20);
      delay(5000);

      retries++;
      if (retries == CONN_RETRY_ATTEMPTS) {
         Serial.println("Retry timeout.");
         break;
      }
   }
  
   if (retries < CONN_RETRY_ATTEMPTS) {
      leds[12] = CRGB::Green;
      repulsor->showLeds(20);
   }
  
   Serial.println("WiFi connected");
   Serial.println("IP address: ");
   Serial.println(WiFi.localIP());

   Serial.print("GATEWAY: ");
   Serial.println(WiFi.gatewayIP());

   // Set maximum WiFi transmission power
   esp_wifi_set_max_tx_power(84);  // 84 corresponds to 21 dBm, which is the maximum for ESP32-*

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
      leds[12] = CRGB::Blue;
      repulsor->showLeds(20);
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

   //memmove( &leds, &all_off, NUM_LEDS * sizeof(CRGB) );
   //repulsor->showLeds(20);
}

unsigned long previousSendMillis = 0; // will store last time the messages were sent
const long sendInterval = 1000;       // interval at which to execute code block (milliseconds)

void loop() {
   static JsonDocument send_doc;
   //static JsonDocument recv_doc;
   //static char input_json[256];
   static char output_json[256];
   sensors_event_t accel;
   sensors_event_t temp;

   int power_switch = 0;
   int fire_switch = 0;
   double stop_pos;
   char str_stop_pos[5];

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
      lsm_temp->getEvent(&temp);

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

   if (currentMillis - previousMillis >= hand_delay) {
      /* Get sensor data. */
      lsm_accel->getEvent(&accel);

      if (accel.acceleration.x > 7.0)
      {
         power_switch = 1;
      } else {
         power_switch = 0;
      }

      if (accel.acceleration.x > 9.2)
      {
         fire_switch = 1;
      } else {
         fire_switch = 0;
      }

#ifndef SUPPRESS_NOISE
      /* Display the results (acceleration is measured in m/s^2) */
      Serial.print("\t\tAccel X: ");
      Serial.print(accel.acceleration.x);
      Serial.print(" \tY: ");
      Serial.print(accel.acceleration.y);
      Serial.print(" \tZ: ");
      Serial.print(accel.acceleration.z);
      Serial.println(" m/s^2 ");
#endif

      // Begin LED processing
      if ((firing <= 0) && (fire_switch != fire_switch_state))
      {
         fire_switch_state = fire_switch;
         if ((state == POWER_ON) && (fire_switch_state == 1))
         {
            Serial.println(PLAY_BLAST_CMD);
           
            /* Play audio */
            send_doc["device"] =    "audio";
            send_doc["command"] =   "play";
            send_doc["arg1"] =      HAND_BLAST_SOUND;
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
            /* End Play */
           
            if ( firing <= 0 )
            {
               repulsor->showLeds( FIRE_BRIGHTNESS );
            } else {
               repulsor->showLeds( BRIGHTNESS );
               flash_delay = 2;
            }

            firing = BLAST_DURATION;
         }
      }

      if (firing >= 0)
      {
         if ( flash_delay )
         {
            flash_delay--;
            if ( flash_delay == 0 )
            {
               repulsor->showLeds( FIRE_BRIGHTNESS );
            }
         }

         firing-=hand_delay;
         if ( firing <= 0 )
         {
            repulsor->showLeds( BRIGHTNESS );
         }
      }

      if (power_switch != power_switch_state)
      {
         power_switch_state = power_switch;
#ifdef DEBUG
         Serial.print("Button change state: ");
#endif
         switch (state)
         {
            case POWER_OFF:
#ifdef DEBUG
               Serial.print("POWER_OFF->POWERING_UP\n");
#endif
               Serial.println(PLAY_UP_CMD);
               
               /* Play audio */
               send_doc["device"] =    "audio";
               send_doc["command"] =   "play";
               send_doc["arg1"] =      HAND_UP_SOUND;
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
               /* End Play */
               
               state = POWERING_UP;
               last_state = POWER_OFF;
#ifdef DEBUG_TIME
               startUpMillis = millis();
#endif
               break;
            case POWERING_UP:
#ifdef DEBUG
               Serial.print("POWERING_UP->POWERING_DOWN\n");
#endif
               Serial.println(STOP_UP_CMD);

               /* Play audio */
               send_doc["device"] =    "audio";
               send_doc["command"] =   "stop";
               send_doc["arg1"] =      HAND_UP_SOUND;
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
               /* End Play */
               
               stop_pos = 1 - (double)(light_level)/255;
               dtostrf( stop_pos, 4, 2, str_stop_pos );
               Serial.print(PLAY_DOWN_CMD" ");
               Serial.println(str_stop_pos);

               /* Play audio */
               send_doc["device"] =    "audio";
               send_doc["command"] =   "play";
               send_doc["arg1"] =      HAND_DOWN_SOUND;
               send_doc["arg2"] =      str_stop_pos;
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
               /* End Play */
               
               state = POWERING_DOWN;
               last_state = POWERING_UP;
               break;
            case POWERING_DOWN:
#ifdef DEBUG
               Serial.print("POWERING_DOWN->POWERING_UP\n");
#endif
               Serial.println(STOP_DOWN_CMD);

               /* Play audio */
               send_doc["device"] =    "audio";
               send_doc["command"] =   "stop";
               send_doc["arg1"] =      HAND_DOWN_SOUND;
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
               /* End Play */
               
               stop_pos = 1 - (double)(light_level)/255;
               dtostrf( stop_pos, 4, 2, str_stop_pos );
               Serial.print(PLAY_UP_CMD" ");
               Serial.println(str_stop_pos);

               /* Play audio */
               send_doc["device"] =    "audio";
               send_doc["command"] =   "play";
               send_doc["arg1"] =      HAND_UP_SOUND;
               send_doc["arg2"] =      str_stop_pos;
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
               /* End Play */
               
               state = POWERING_UP;
               last_state = POWERING_DOWN;
               break;
            case POWER_ON:
#ifdef DEBUG
               Serial.print("POWER_ON->POWERING_DOWN\n");
#endif
               Serial.println(PLAY_DOWN_CMD);

               /* Play audio */
               send_doc["device"] =    "audio";
               send_doc["command"] =   "play";
               send_doc["arg1"] =      HAND_DOWN_SOUND;
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
               /* End Play */
               
               state = POWERING_DOWN;
               last_state = POWER_ON;
#ifdef DEBUG_TIME
               startDownMillis = millis();
#endif
               break;
         }
      }

      switch (state)
      {
         case POWER_OFF:
            if ( last_state != POWER_OFF )
            {
               memmove( &leds, &all_off, NUM_LEDS * sizeof(CRGB) );
               repulsor->showLeds( BRIGHTNESS );
               last_state = POWER_OFF;
#ifdef DEBUG_TIME
               stopDownMillis = millis();
               seconds = (int)((stopDownMillis - startDownMillis)/1000);
               milliseconds = (int)(stopDownMillis - startDownMillis) - ((stopDownMillis - startDownMillis)/1000);
               Serial.print("Time Down: ");
               Serial.print(seconds);
               Serial.print(":");
               Serial.println(milliseconds);
#endif
            }
            break;
         case POWERING_UP:
            // brighten center circle to full
            //Serial.print("PRE light_level ");
            //Serial.println(rep_data[lset].light_level);
            light_level=light_level + HAND_STEP_SIZE;
#ifdef DEBUG
            Serial.print("light_level: ");
            Serial.println(light_level);
#endif
            memmove( &leds, &inside_leds, NUM_LEDS * sizeof(CRGB) );
            for ( int i=NUM_OUTSIDE-1; i<NUM_LEDS; i++ )
            {
               leds[i].nscale8(light_level);
            }
            repulsor->showLeds( BRIGHTNESS );
            if ( light_level == 255 )
            {
#ifdef DEBUG
               Serial.print("POWERING_UP->POWER_ON\n");
#endif
               state = POWER_ON;
#ifdef DEBUG_TIME
               stopUpMillis = millis();
               seconds = (int)((stopUpMillis - startUpMillis)/1000);
               milliseconds = (int)(stopUpMillis - startUpMillis) - ((stopUpMillis - startUpMillis)/1000);
               Serial.print("Time Up: ");
               Serial.print(seconds);
               Serial.print(":");
               Serial.println(milliseconds);
#endif
            }
            last_state = POWERING_UP;
            break;
         case POWERING_DOWN:
            // fade center circle to off
            light_level-=HAND_STEP_SIZE;
#ifdef DEBUG
            Serial.print("light_level: ");
            Serial.println(light_level);
#endif
            memmove( &leds, &inside_leds, NUM_LEDS * sizeof(CRGB) );
            for ( int i=NUM_OUTSIDE-1; i<NUM_LEDS; i++ )
            {
               leds[i].nscale8(light_level);
            }
            repulsor->showLeds( BRIGHTNESS );
            if ( light_level == 0 )
            {
#ifdef DEBUG
               Serial.print("POWERING_DOWN->POWER_OFF\n");
#endif
               state = POWER_OFF;
            }
            last_state = POWERING_DOWN;
            break;
         case POWER_ON:
            if ( last_state != POWER_ON )
            {
               memmove( &leds, &all_on, NUM_LEDS * sizeof(CRGB) );
               repulsor->showLeds( BRIGHTNESS );
               last_state = POWER_ON;
            }
            break;
      }
      
      // save the last time a message was sent
      previousMillis = currentMillis;
    
      //delay(hand_delay);
   }
}
