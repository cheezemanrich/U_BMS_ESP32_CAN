/*
  #############################################################################################
  # This file is part of U_BMS_ESP32_CAN <https://github.com/cheezemanrich/U_BMS_ESP32_CAN>.
  #
  # U_BMS_ESP32_CAN is free software: you can redistribute it and/or modify
  # it under the terms of the GNU General Public License as published by
  # the Free Software Foundation, either version 3 of the License, or
  # (at your option) any later version.
  #
  # BMScan is distributed in the hope that it will be useful,
  # but WITHOUT ANY WARRANTY; without even the implied warranty of
  # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  # GNU General Public License for more details.
  #############################################################################################
	
	This code is based on:
	Copyright (C) designer2k2 Stephan M.
  # This file is part of EMUcan <https://github.com/designer2k2/EMUcan>.
  #
  # EMUcanT4 is free software: you can redistribute it and/or modify
  # it under the terms of the GNU General Public License as published by
  # the Free Software Foundation, either version 3 of the License, or
  # (at your option) any later version.
  #
  # EMUcan is distributed in the hope that it will be useful,
  # but WITHOUT ANY WARRANTY; without even the implied warranty of
  # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  # GNU General Public License for more details.
  #
  # You should have received a copy of the GNU General Public License
  # along with EMUcan.  If not, see <http://www.gnu.org/licenses/>.
  
*/

//==================================================================================//
//
// ILI9341 Pin 1: 5V
// ILI9341 Pin 2: Gnd
// ILI9341 Pin 3: GPIO13 (CS)
// ILI9341 Pin 4: 3.3V
// ILI9341 Pin 5: GPIO33 (DC)
// ILI9341 Pin 6: GPIO23 (MOSI/SDI)
// ILI9341 Pin 7: GPIO18 (SCK)
// ILI9341 Pin 8: 10k resistor to 3.3V (Backlight)
// ILI9341 Pin 9: GPIO19 (MISO/SDO)
// ILI9341 Pins 10-14: Not Used
//
// TJA1051 Pin  VCC : 3.3V
// TJA1051 Pin  GND : Gnd
// TJA1051 Pin  CTX : GPIO21
// TJA1051 Pin  CRX : GPIO22
// TJA1051 Pin CANH : To BMS CAN Bus High
// TJA1051 Pin CANL : To BMS CAN Bus Low
// TJA1051 Pin    S : Not Used
// TJA1051 Pin   NC : Not Used
//
//==================================================================================//

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <CAN.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include <SPIFFS.h>

#include "U_BMS_ESP32_CAN.h"	

// Replace with your network credentials
const char* ssid = "Your WiFi SSID";
const char* password = "Your WiFi Password";

// Set web server port number to 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Used to keep track of the XML state machine
enum State { XML_Preamble, XML_Body, XML_Done };

// BMS initialized with base ID 0x0C0:
BMScan bmscan(0x001);

// Needed for the CAN Interface on the ESP32 (called TWAI):
#include "driver/twai.h"

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 22
#define TX_PIN 21

unsigned long previousMillis = 0;
const long interval = 1000; // milliseconds

// Set BMS thresholds - mV and C to change display color to red
int battHighV = 3600;  // 3600 Default
int battLowV = 2750;  // 2750 Default
int battHighT = 6000;  // 6000 Default
int battLowT = 0200;  // 0200 Default

// For the ILI9341, these are the default pins.
#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_DC  32
#define TFT_CS 33

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// Hard defined macros to make changing display text color easier
#define TFTSCRBLACK   tft.fillScreen(ILI9341_BLACK)
#define TFTTXTGREEN   tft.setTextColor(ILI9341_GREEN)
#define TFTTXTWHITE   tft.setTextColor(ILI9341_WHITE)
#define TFTTXTRED     tft.setTextColor(ILI9341_RED)
#define TFTTXTYELLOW  tft.setTextColor(ILI9341_YELLOW)
#define TFTTXTMAGENTA tft.setTextColor(ILI9341_MAGENTA)
#define TFTTXTCYAN    tft.setTextColor(ILI9341_CYAN)
#define TFTTXTORANGE  tft.setTextColor(ILI9341_ORANGE)
#define TFTTXTBLUE    tft.setTextColor(ILI9341_BLUE)

// Function prototypes
void drawDisplayNew(void);
void writeToDisplayNew(void);
void charSetCursor(uint16_t, uint16_t);
uint16_t charPosX(uint16_t);
uint16_t charPosY(uint16_t);
void charBlank(uint16_t, uint16_t, uint16_t);
String getBatteryReadings(void);
int calc_cell_delta (int, int, int, int);

// New Display Info
#define SETTEXTCURSOR

// Used for formatting console to binary display of integers
// Good for debugging bit fields
// Use: printf("Leading text "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(byte));
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

//==================================================================================//
void setup() {
  Serial.begin (115200);
  while (!Serial);
  delay (1000);
  
  Serial.print("BMSCAN_LIB_VERSION: ");
  Serial.println(BMSCAN_LIB_VERSION);

  Serial.println("CAN configure started");

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_LISTEN_ONLY);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("CAN Driver installed");
  } else {
    Serial.println("Failed to install CAN driver");
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("CAN Driver started");
  } else {
    Serial.println("Failed to start CAN driver");
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
  }
  
  // start up the LCD and initialize it
  tft.begin();
  drawDisplayNew();

  //Start up the SPIFFS file system
  if(!SPIFFS.begin(true)){
    Serial.println("SPIFFS_ERROR");
    return;
  }

  // Connect to Wi-Fi network with SSID and password
  Serial.print("WiFi Connecting to: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println(" ");
  Serial.print("Connected. IP: ");
  Serial.println(WiFi.localIP());

  // put the IP address on the display
  charBlank(4, 39, 16);  // (character X pos, character  pos, number of characters)
  TFTTXTCYAN;
  charSetCursor(4, 39); // Positions are 0 referenced x, y
  tft.print(WiFi.localIP());
  
  // Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/batt1.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/batt1.html", "text/html");
  });

  server.on("/batt2.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/batt2.html", "text/html");
  });

  server.on("/batt3.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/batt3.html", "text/html");
  });

  server.on("/batt4.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/batt4.html", "text/html");
  });

  server.on("/batt5.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/batt5.html", "text/html");
  });

  server.on("/batt6.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/batt6.html", "text/html");
  });

  server.on("/batt7.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/batt7.html", "text/html");
  });

  server.on("/batt8.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/batt8.html", "text/html");
  });
  
  server.serveStatic("/", SPIFFS, "/");

  // Request for the latest sensor readings
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = getBatteryReadings();
    request->send(200, "application/json", json);
    json = String();
  });

  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });

  server.addHandler(&events);
  server.begin();
  
  Serial.println("");
}

//==================================================================================//
void loop() {
  // Call the EMUcan lib with every received frame:
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1));
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    // One or more messages received. Handle all.
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      bmscan.checkBMScan(message.identifier, message.data_length_code, message.data);
    }
  }

  // Serial out every interval:
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (bmscan.BMScan_Status() == BMScan_RECEIVED_WITHIN_LAST_SECOND) {
//      Serial.print("SOC:");
//      Serial.print(bmscan.bms_data.bat_soc);
//      Serial.print(" #Mod:");
//      Serial.print(bmscan.bms_data.bat_num_mods);
//      Serial.print(" #Bal:");
//      Serial.print(bmscan.bms_data.bat_num_bal);
//      Serial.println(" ");
//      Serial.print("St1:");
//      Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(bmscan.bms_data.bat_status1));
//      Serial.print(" St2:");
//      Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(bmscan.bms_data.bat_status2));
//      Serial.print(" St3:");
//      Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(bmscan.bms_data.bat_status3));
//      Serial.print(" St4:");
//      Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(bmscan.bms_data.bat_status4));
//      Serial.print(" St5:");
//      Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(bmscan.bms_data.bat_status5));
//      Serial.println(" ");
//
//      Serial.print("Volt_1:");
//      Serial.print(bmscan.bms_data.mod1_cell_1_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod1_cell_2_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod1_cell_3_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod1_cell_4_V);
//      Serial.println(" ");
//      Serial.print("Volt_2:");
//      Serial.print(bmscan.bms_data.mod2_cell_1_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod2_cell_2_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod2_cell_3_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod2_cell_4_V);
//      Serial.println(" ");
//      Serial.print("Volt_3:");
//      Serial.print(bmscan.bms_data.mod3_cell_1_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod3_cell_2_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod3_cell_3_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod3_cell_4_V);
//      Serial.println(" ");
//      Serial.print("Volt_4:");
//      Serial.print(bmscan.bms_data.mod4_cell_1_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod4_cell_2_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod4_cell_3_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod4_cell_4_V);
//      Serial.println(" ");
//      Serial.print("Volt_5:");
//      Serial.print(bmscan.bms_data.mod5_cell_1_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod5_cell_2_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod5_cell_3_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod5_cell_4_V);
//      Serial.println(" ");
//      Serial.print("Volt_6:");
//      Serial.print(bmscan.bms_data.mod6_cell_1_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod6_cell_2_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod6_cell_3_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod6_cell_4_V);
//      Serial.println(" ");
//      Serial.print("Volt_7:");
//      Serial.print(bmscan.bms_data.mod7_cell_1_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod7_cell_2_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod7_cell_3_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod7_cell_4_V);
//      Serial.println(" ");
//      Serial.print("Volt_8:");
//      Serial.print(bmscan.bms_data.mod8_cell_1_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod8_cell_2_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod8_cell_3_V);
//      Serial.print(":");
//      Serial.print(bmscan.bms_data.mod8_cell_4_V);
//      Serial.println(" ");
//
//      Serial.print("Bal:");
//      Serial.print(" M1:");
//      Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(bmscan.bms_data.mod1_cell_bal_stat));
//      Serial.print(" M2:");
//      Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(bmscan.bms_data.mod2_cell_bal_stat));
//      Serial.print(" M3:");
//      Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(bmscan.bms_data.mod3_cell_bal_stat));
//      Serial.print(" M4:");
//      Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(bmscan.bms_data.mod4_cell_bal_stat));
//      Serial.println(" ");
//      Serial.print("Bal:");
//      Serial.print(" M5:");
//      Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(bmscan.bms_data.mod5_cell_bal_stat));
//      Serial.print(" M6:");
//      Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(bmscan.bms_data.mod6_cell_bal_stat));
//      Serial.print(" M7:");
//      Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(bmscan.bms_data.mod7_cell_bal_stat));
//      Serial.print(" M8:");
//      Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(bmscan.bms_data.mod8_cell_bal_stat));
//      Serial.println(" ");
//      
//      Serial.print("MaxT:");
//      Serial.print(bmscan.bms_data.bat_max_temp);
//      Serial.print(" MinT:");
//      Serial.print(bmscan.bms_data.bat_min_temp);
//      Serial.print(" MaxPCT:");
//      Serial.print(bmscan.bms_data.bat_pcb_max_temp);
//      Serial.print(" CellMaxV:");
//      Serial.print(bmscan.bms_data.cell_max_v);
//      Serial.print(" CellMinV:");
//      Serial.println(bmscan.bms_data.cell_min_v);

      // This function writes data to the LCD display
      writeToDisplayNew();
      
      // Send Events to the web client with the Battery Readings
      events.send("ping",NULL,millis());
      events.send(getBatteryReadings().c_str(),"new_readings" ,millis()); 
    } else {
      Serial.println("No BMS Comm!");
    }
  }
}

// *****************************************
// Writes data to SPI LCD to draw a new blank display
// *****************************************
void drawDisplayNew() {
  TFTSCRBLACK;
  TFTTXTGREEN;
  tft.setTextSize(1);
  tft.setCursor(0, 0);

  TFTTXTWHITE;
  tft.print("      Cel1:Cel2:Cel3:Cel4:Total: SOC\r\n"); // Y=0
  tft.print("Batt1:    :    :    :    :     \r\n"); // X pos starts at : which is 5
  tft.print("Batt2:    :    :    :    :     \r\n");
  tft.print("Batt3:    :    :    :    :     \r\n");
  tft.print("Batt4:    :    :    :    :     \r\n");
  tft.print("Batt5:    :    :    :    :     \r\n");
  tft.print("Batt6:    :    :    :    :     \r\n");
  tft.print("Batt7:    :    :    :    :     \r\n");
  tft.print("Batt8:    :    :    :    :     \r\n");
  tft.print(" \r\n");
  tft.print("      Tmp1 :Tmp2 :SOC :Curr    \r\n"); // Y=10
  tft.print("Batt1:     :     :             \r\n"); // X pos starts at : which is 5
  tft.print("Batt2:     :     :             \r\n");
  tft.print("Batt3:     :     :             \r\n");
  tft.print("Batt4:     :     :             \r\n");
  tft.print("Batt5:     :     :             \r\n");
  tft.print("Batt6:     :     :             \r\n");
  tft.print("Batt7:     :     :             \r\n");
  tft.print("Batt8:     :     :             \r\n");
  tft.print(" \r\n");
  tft.print("      Bal:San \r\n");  // Y=20
  tft.print("Batt1: : :    :   :    :     \r\n"); // X pos starts at : which is 5
  tft.print("Batt2: : :    :   :    :     \r\n");
  tft.print("Batt3: : :    :   :    :     \r\n");
  tft.print("Batt4: : :    :   :    :     \r\n");
  tft.print("Batt5: : :    :   :    :     \r\n");
  tft.print("Batt6: : :    :   :    :     \r\n");
  tft.print("Batt7: : :    :   :    :     \r\n");
  tft.print("Batt8: : :    :   :    :     \r\n");
  tft.print(" \r\n");
  tft.print("Upr:            Lwr:       \r\n");  // Y=30, X=5 and X=21
  tft.print("Vlt:            Cur:       \r\n");  // Y=31, X=5 and X=21
  tft.setTextSize(2);
  tft.print("                      \r\n"); // Location of Power
  tft.print("SOC:    \r\n");
  tft.setTextSize(1);
  tft.print("U-BMS Mode:      ChrgStage:    \r\n");  // Y=37, X=11 and X=28
  tft.print("IP:");
  TFTTXTYELLOW;
  tft.print("xxx.xxx.xxx.xxx   ");
  TFTTXTWHITE;
  tft.print("SWRev:    \r\n");
}

// *****************************************
// Convert character position to pixel position and set it there
// *****************************************
void charSetCursor(uint16_t posx, uint16_t posy) {
  tft.setCursor(charPosX(posx),charPosY(posy));
}

// *****************************************
// Convert character X position to pixel position
// *****************************************
uint16_t charPosX(uint16_t charPos) {
  uint16_t pixels;
  if (charPos > 0) {
    pixels = charPos * 6; // characters are 5 pixels wide plus 1 pad
  }
  else {
    pixels = 0; // position 0 has no padding
  }
  return pixels;
}

// *****************************************
// Convert character Y position to pixel position
// *****************************************
uint16_t charPosY(uint16_t charPos) {
  uint16_t pixels;
  if (charPos > 0) { 
    pixels = charPos * 8; // characters are 7 pixels wide plus 1 pad
  }
  else {
    pixels = 0; // position 0 has no padding
  }
  return pixels;
}

// *****************************************
// Convert character X and Y position to pixel position and blank number of characters
// *****************************************
void charBlank(uint16_t posx, uint16_t posy, uint16_t numChars) {
  uint16_t pixelsX = numChars * 6;  // characters width pixels
  uint16_t pixelsY = 8; // character height in pixels
  tft.fillRect(posx * 6, posy * 8, pixelsX, pixelsY, ILI9341_BLACK);  // pixel position x, pixel position y, pixels x, pixels y
}

// *****************************************
// Writes data to SPI LCD to refresh values
// *****************************************
void writeToDisplayNew() {
  // Write to display
  TFTTXTGREEN;
  tft.setTextSize(1);
  charBlank(5, 1, 36);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 1);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod1_cell_bal_stat & 0x01)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod1_cell_1_V >= battHighT) || (bmscan.bms_data.mod1_cell_1_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod1_cell_1_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod1_cell_bal_stat & 0x02)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod1_cell_2_V >= battHighT) || (bmscan.bms_data.mod1_cell_2_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod1_cell_2_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod1_cell_bal_stat & 0x04)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod1_cell_3_V >= battHighT) || (bmscan.bms_data.mod1_cell_3_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod1_cell_3_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod1_cell_bal_stat & 0x08)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod1_cell_4_V >= battHighT) || (bmscan.bms_data.mod1_cell_4_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod1_cell_4_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String((bmscan.bms_data.mod1_cell_1_V
                    +bmscan.bms_data.mod1_cell_2_V
                    +bmscan.bms_data.mod1_cell_3_V
                    +bmscan.bms_data.mod1_cell_4_V)));
  TFTTXTWHITE;
  tft.print(": ");
  TFTTXTCYAN;
  tft.print(bmscan.bms_data.mod_1_soc);
  tft.print("%");
  
  charBlank(5, 2, 36);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 2);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod2_cell_bal_stat & 0x01)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod2_cell_1_V >= battHighT) || (bmscan.bms_data.mod2_cell_1_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod2_cell_1_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod2_cell_bal_stat & 0x02)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod2_cell_2_V >= battHighT) || (bmscan.bms_data.mod2_cell_2_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod2_cell_2_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod2_cell_bal_stat & 0x04)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod2_cell_3_V >= battHighT) || (bmscan.bms_data.mod2_cell_3_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod2_cell_3_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod2_cell_bal_stat & 0x08)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod2_cell_4_V >= battHighT) || (bmscan.bms_data.mod2_cell_4_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod2_cell_4_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String((bmscan.bms_data.mod2_cell_1_V
                    +bmscan.bms_data.mod2_cell_2_V
                    +bmscan.bms_data.mod2_cell_3_V
                    +bmscan.bms_data.mod2_cell_4_V)));
  TFTTXTWHITE;
  tft.print(": ");
  TFTTXTCYAN;
  tft.print(bmscan.bms_data.mod_2_soc);
  tft.print("%");
  
  charBlank(5, 3, 36);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 3);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod3_cell_bal_stat & 0x01)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod3_cell_1_V >= battHighT) || (bmscan.bms_data.mod3_cell_1_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod3_cell_1_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod3_cell_bal_stat & 0x02)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod3_cell_2_V >= battHighT) || (bmscan.bms_data.mod3_cell_2_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod3_cell_2_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod3_cell_bal_stat & 0x04)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod3_cell_3_V >= battHighT) || (bmscan.bms_data.mod3_cell_3_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod3_cell_3_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod3_cell_bal_stat & 0x08)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod3_cell_4_V >= battHighT) || (bmscan.bms_data.mod3_cell_4_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod3_cell_4_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String((bmscan.bms_data.mod3_cell_1_V
                    +bmscan.bms_data.mod3_cell_2_V
                    +bmscan.bms_data.mod3_cell_3_V
                    +bmscan.bms_data.mod3_cell_4_V)));
  TFTTXTWHITE;
  tft.print(": ");
  TFTTXTCYAN;
  tft.print(bmscan.bms_data.mod_3_soc);
  tft.print("%");
  
  charBlank(5, 4, 36);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 4);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod4_cell_bal_stat & 0x01)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod4_cell_1_V >= battHighT) || (bmscan.bms_data.mod4_cell_1_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod4_cell_1_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod4_cell_bal_stat & 0x02)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod4_cell_2_V >= battHighT) || (bmscan.bms_data.mod4_cell_2_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod4_cell_2_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod4_cell_bal_stat & 0x04)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod4_cell_3_V >= battHighT) || (bmscan.bms_data.mod4_cell_3_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod4_cell_3_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod4_cell_bal_stat & 0x08)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod4_cell_4_V >= battHighT) || (bmscan.bms_data.mod4_cell_4_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod4_cell_4_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String((bmscan.bms_data.mod4_cell_1_V
                    +bmscan.bms_data.mod4_cell_2_V
                    +bmscan.bms_data.mod4_cell_3_V
                    +bmscan.bms_data.mod4_cell_4_V)));
  TFTTXTWHITE;
  tft.print(": ");
  TFTTXTCYAN;
  tft.print(bmscan.bms_data.mod_4_soc);
  tft.print("%");
  
  charBlank(5, 5, 36);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 5);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod5_cell_bal_stat & 0x01)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod5_cell_1_V >= battHighT) || (bmscan.bms_data.mod5_cell_1_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod5_cell_1_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod5_cell_bal_stat & 0x02)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod5_cell_2_V >= battHighT) || (bmscan.bms_data.mod5_cell_2_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod5_cell_2_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod5_cell_bal_stat & 0x04)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod5_cell_3_V >= battHighT) || (bmscan.bms_data.mod5_cell_3_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod5_cell_3_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod5_cell_bal_stat & 0x08)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod5_cell_4_V >= battHighT) || (bmscan.bms_data.mod5_cell_4_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod5_cell_4_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String((bmscan.bms_data.mod5_cell_1_V
                    +bmscan.bms_data.mod5_cell_2_V
                    +bmscan.bms_data.mod5_cell_3_V
                    +bmscan.bms_data.mod5_cell_4_V)));
  TFTTXTWHITE;
  tft.print(": ");
  TFTTXTCYAN;
  tft.print(bmscan.bms_data.mod_5_soc);
  tft.print("%");
  
  charBlank(5, 6, 36);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 6);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod6_cell_bal_stat & 0x01)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod6_cell_1_V >= battHighT) || (bmscan.bms_data.mod6_cell_1_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod6_cell_1_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod6_cell_bal_stat & 0x02)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod6_cell_2_V >= battHighT) || (bmscan.bms_data.mod6_cell_2_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod6_cell_2_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod6_cell_bal_stat & 0x04)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod6_cell_3_V >= battHighT) || (bmscan.bms_data.mod6_cell_3_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod6_cell_3_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod6_cell_bal_stat & 0x08)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod6_cell_4_V >= battHighT) || (bmscan.bms_data.mod6_cell_4_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod6_cell_4_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String((bmscan.bms_data.mod6_cell_1_V
                    +bmscan.bms_data.mod6_cell_2_V
                    +bmscan.bms_data.mod6_cell_3_V
                    +bmscan.bms_data.mod6_cell_4_V)));
  TFTTXTWHITE;
  tft.print(": ");
  TFTTXTCYAN;
  tft.print(bmscan.bms_data.mod_6_soc);
  tft.print("%");
  
  charBlank(5, 7, 36);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 7);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod7_cell_bal_stat & 0x01)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod7_cell_1_V >= battHighT) || (bmscan.bms_data.mod7_cell_1_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod7_cell_1_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod7_cell_bal_stat & 0x02)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod7_cell_2_V >= battHighT) || (bmscan.bms_data.mod7_cell_2_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod7_cell_2_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod7_cell_bal_stat & 0x04)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod7_cell_3_V >= battHighT) || (bmscan.bms_data.mod7_cell_3_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod7_cell_3_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod7_cell_bal_stat & 0x08)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod7_cell_4_V >= battHighT) || (bmscan.bms_data.mod7_cell_4_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod7_cell_4_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String((bmscan.bms_data.mod7_cell_1_V
                    +bmscan.bms_data.mod7_cell_2_V
                    +bmscan.bms_data.mod7_cell_3_V
                    +bmscan.bms_data.mod7_cell_4_V)));
  TFTTXTWHITE;
  tft.print(": ");
  TFTTXTCYAN;
  tft.print(bmscan.bms_data.mod_7_soc);
  tft.print("%");
  
  charBlank(5, 8, 36);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 8);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod8_cell_bal_stat & 0x01)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod8_cell_1_V >= battHighT) || (bmscan.bms_data.mod8_cell_1_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod8_cell_1_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod8_cell_bal_stat & 0x02)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod8_cell_2_V >= battHighT) || (bmscan.bms_data.mod8_cell_2_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod8_cell_2_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod8_cell_bal_stat & 0x04)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod8_cell_3_V >= battHighT) || (bmscan.bms_data.mod8_cell_3_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod8_cell_3_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(!(bmscan.bms_data.mod8_cell_bal_stat & 0x08)){
    TFTTXTYELLOW;
  }
  if ((bmscan.bms_data.mod8_cell_4_V >= battHighT) || (bmscan.bms_data.mod8_cell_4_V <= battLowT)) {
    TFTTXTRED;
  }
  tft.print(String(bmscan.bms_data.mod8_cell_4_V));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String((bmscan.bms_data.mod8_cell_1_V
                    +bmscan.bms_data.mod8_cell_2_V
                    +bmscan.bms_data.mod8_cell_3_V
                    +bmscan.bms_data.mod8_cell_4_V)));

  TFTTXTWHITE;
  tft.print(": ");
  TFTTXTCYAN;
  tft.print(bmscan.bms_data.mod_8_soc);
  tft.print("%");

  // Second section
  charBlank(5, 11, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 11);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod1_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod1_pcb_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod_1_soc));
  TFTTXTWHITE;
  tft.print("  :");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod1_curr));
  charBlank(5, 12, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 12);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod2_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod2_pcb_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod_2_soc));
  TFTTXTWHITE;
  tft.print("  :");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod2_curr));
  charBlank(5, 13, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 13);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod3_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod3_pcb_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod_3_soc));
  TFTTXTWHITE;
  tft.print("  :");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod3_curr));
  charBlank(5, 14, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 14);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod4_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod4_pcb_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod_4_soc));
  TFTTXTWHITE;
  tft.print("  :");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod4_curr));
  charBlank(5, 15, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 15);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod5_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod5_pcb_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod_5_soc));
  TFTTXTWHITE;
  tft.print("  :");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod5_curr));
  charBlank(5, 16, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 16);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod6_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod6_pcb_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod_6_soc));
  TFTTXTWHITE;
  tft.print("  :");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod6_curr));
  charBlank(5, 17, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 17);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod7_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod7_pcb_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod_7_soc));
  TFTTXTWHITE;
  tft.print("  :");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod7_curr));
  charBlank(5, 18, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5, 18);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod8_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod8_pcb_temp));
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod_8_soc));
  TFTTXTWHITE;
  tft.print("  :");
  TFTTXTGREEN;
  tft.print(String(bmscan.bms_data.mod8_curr));

  // Third section
  TFTTXTGREEN;
  charBlank(5, 21, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5,21);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_bal_stat & 0x01){
    TFTTXTYELLOW;
    tft.print("BAL");
  }else{
    TFTTXTGREEN;
    tft.print("OFF");
  }
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_sanity_error_flags & 0x01){
    TFTTXTYELLOW;
    tft.print("Err");
  }else{
    TFTTXTGREEN;
    tft.print("OK ");
  }
  charBlank(5, 22, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5,22);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_bal_stat & 0x02){
    TFTTXTYELLOW;
    tft.print("BAL");
  }else{
    TFTTXTGREEN;
    tft.print("OFF");
  }
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_sanity_error_flags & 0x02){
    TFTTXTYELLOW;
    tft.print("Err");
  }else{
    TFTTXTGREEN;
    tft.print("OK ");
  }
  charBlank(5, 23, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5,23);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_bal_stat & 0x04){
    TFTTXTYELLOW;
    tft.print("BAL");
  }else{
    TFTTXTGREEN;
    tft.print("OFF");
  }
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_sanity_error_flags & 0x04){
    TFTTXTYELLOW;
    tft.print("Err");
  }else{
    TFTTXTGREEN;
    tft.print("OK ");
  }
  charBlank(5, 24, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5,24);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_bal_stat & 0x08){
    TFTTXTYELLOW;
    tft.print("BAL");
  }else{
    TFTTXTGREEN;
    tft.print("OFF");
  }
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_sanity_error_flags & 0x08){
    TFTTXTYELLOW;
    tft.print("Err");
  }else{
    TFTTXTGREEN;
    tft.print("OK ");
  }
  charBlank(5, 25, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5,25);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_bal_stat & 0x10){
    TFTTXTYELLOW;
    tft.print("BAL");
  }else{
    TFTTXTGREEN;
    tft.print("OFF");
  }
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_sanity_error_flags & 0x10){
    TFTTXTYELLOW;
    tft.print("Err");
  }else{
    TFTTXTGREEN;
    tft.print("OK ");
  }
  charBlank(5, 26, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5,26);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_bal_stat & 0x20){
    TFTTXTYELLOW;
    tft.print("BAL");
  }else{
    TFTTXTGREEN;
    tft.print("OFF");
  }
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_sanity_error_flags & 0x20){
    TFTTXTYELLOW;
    tft.print("Err");
  }else{
    TFTTXTGREEN;
    tft.print("OK ");
  }
  charBlank(5, 27, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5,27);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_bal_stat & 0x40){
    TFTTXTYELLOW;
    tft.print("BAL");
  }else{
    TFTTXTGREEN;
    tft.print("OFF");
  }
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_sanity_error_flags & 0x40){
    TFTTXTYELLOW;
    tft.print("Err");
  }else{
    TFTTXTGREEN;
    tft.print("OK ");
  }
  charBlank(5, 28, 28);  // (character X pos, character  pos, number of characters)
  charSetCursor(5,28);
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_bal_stat & 0x80){
    TFTTXTYELLOW;
    tft.print("BAL");
  }else{
    TFTTXTGREEN;
    tft.print("OFF");
  }
  TFTTXTWHITE;
  tft.print(":");
  TFTTXTGREEN;
  if(bmscan.bms_data.mod_sanity_error_flags & 0x80){
    TFTTXTYELLOW;
    tft.print("Err");
  }else{
    TFTTXTGREEN;
    tft.print("OK ");
  }

  //bottom section
  charBlank(5, 30, 6);  // (character X pos, character  pos, number of characters)
  charSetCursor(5,30);
  TFTTXTMAGENTA;  // Total for Upper (+) side
  float Mod2V = (bmscan.bms_data.mod2_cell_1_V + bmscan.bms_data.mod2_cell_2_V + bmscan.bms_data.mod2_cell_3_V + bmscan.bms_data.mod2_cell_4_V);
  float Mod4V = (bmscan.bms_data.mod4_cell_1_V + bmscan.bms_data.mod4_cell_2_V + bmscan.bms_data.mod4_cell_3_V + bmscan.bms_data.mod4_cell_4_V);
  float Mod6V = (bmscan.bms_data.mod6_cell_1_V + bmscan.bms_data.mod6_cell_2_V + bmscan.bms_data.mod6_cell_3_V + bmscan.bms_data.mod6_cell_4_V);
  float Mod8V = (bmscan.bms_data.mod8_cell_1_V + bmscan.bms_data.mod8_cell_2_V + bmscan.bms_data.mod8_cell_3_V + bmscan.bms_data.mod8_cell_4_V);
  float totalBattVoltUpper = ((Mod2V + Mod4V + Mod6V + Mod8V) / 4) / 1000;
  tft.print((float)totalBattVoltUpper);
  tft.print("V");
  
  charBlank(21, 30, 6);  // (character X pos, character  pos, number of characters)
  charSetCursor(21,30);
  TFTTXTMAGENTA;  // Total for lower (-) side
  float Mod1V = (bmscan.bms_data.mod1_cell_1_V + bmscan.bms_data.mod1_cell_2_V + bmscan.bms_data.mod1_cell_3_V + bmscan.bms_data.mod1_cell_4_V);
  float Mod3V = (bmscan.bms_data.mod3_cell_1_V + bmscan.bms_data.mod3_cell_2_V + bmscan.bms_data.mod3_cell_3_V + bmscan.bms_data.mod3_cell_4_V);
  float Mod5V = (bmscan.bms_data.mod5_cell_1_V + bmscan.bms_data.mod5_cell_2_V + bmscan.bms_data.mod5_cell_3_V + bmscan.bms_data.mod5_cell_4_V);
  float Mod7V = (bmscan.bms_data.mod7_cell_1_V + bmscan.bms_data.mod7_cell_2_V + bmscan.bms_data.mod7_cell_3_V + bmscan.bms_data.mod7_cell_4_V);
  float totalBattVoltLower = ((Mod1V + Mod3V + Mod5V + Mod7V) / 4) / 1000;
  tft.print((float)totalBattVoltLower);
  tft.print("V");
  
  charBlank(5, 31, 6);  // (character X pos, character  pos, number of characters)
  charSetCursor(5,31);
  float totalBattVolt = totalBattVoltUpper + totalBattVoltLower;
  tft.setTextColor(ILI9341_GREENYELLOW);
  tft.print((float)totalBattVolt);
  tft.print("V");
  
  charBlank(21, 31, 8);  // (character X pos, character  pos, number of characters)
  charSetCursor(21,31);
  TFTTXTCYAN;
  float LowerCurrent = (bmscan.bms_data.mod1_curr + bmscan.bms_data.mod3_curr + bmscan.bms_data.mod5_curr + bmscan.bms_data.mod7_curr);
  float UpperCurrent = (bmscan.bms_data.mod2_curr + bmscan.bms_data.mod4_curr + bmscan.bms_data.mod6_curr + bmscan.bms_data.mod8_curr);
  float totalBattCurr = (LowerCurrent + UpperCurrent) / 2;
  tft.print((float)totalBattCurr);
  tft.print("A");
  
  tft.fillRect(0, 264, 200, 16, ILI9341_BLACK);  // pos x, pos y, pixels x, pixels y, color
  tft.setTextSize(2);
  charSetCursor(0,33);
  float totalBattWatt = totalBattCurr * totalBattVolt;
  if (totalBattCurr < 0) {
    TFTTXTYELLOW;
    tft.print("Disch ");
  }
  else if (totalBattCurr > 0) {
    tft.setTextColor(ILI9341_ORANGE);
    tft.print("Charg ");
  }
  else if (totalBattCurr == 0) {
    TFTTXTBLUE;
    tft.print("Idle ");
  }
  tft.print((float)totalBattWatt);
  tft.print("W");
  
  //charBlank(5, 33, 7);  // (character X pos, character  pos, number of characters)
  tft.fillRect(47, 288, 100, 16, ILI9341_BLACK);  // pos x, pos y, pixels x, pixels y, color
  tft.setTextSize(2);
  charSetCursor(8,36);
  TFTTXTCYAN;
  tft.print(String(bmscan.bms_data.bat_soc) + "%");
  
  tft.setTextSize(1);
  charBlank(11, 38, 6);  // (character X pos, character  pos, number of characters)
  charSetCursor(11,38);
  if (!(bmscan.bms_data.bat_status1 & 0x01) && !(bmscan.bms_data.bat_status1 & 0x02)) { // xxxxxx00
    TFTTXTGREEN;
    tft.print("Stb");
  } else if ((bmscan.bms_data.bat_status1 & 0x01) && !(bmscan.bms_data.bat_status1 & 0x02)){  // xxxxxx01
    TFTTXTMAGENTA;
    tft.print("Chg");
  } else if (!(bmscan.bms_data.bat_status1 & 0x01) && (bmscan.bms_data.bat_status1 & 0x02)){  // xxxxxx10
    TFTTXTMAGENTA;
    tft.print("Drv");
  } else if ((bmscan.bms_data.bat_status1 & 0x01) && (bmscan.bms_data.bat_status1 & 0x02)){  // xxxxxx11
    TFTTXTMAGENTA;
    tft.print("!Sig");
  }
  
  charBlank(27, 38, 6);  // (character X pos, character  pos, number of characters)
  charSetCursor(27,38);
  if (!(bmscan.bms_data.bat_status1 & 0x04) && !(bmscan.bms_data.bat_status1 & 0x08)) { // xxxx00xx
    TFTTXTGREEN;
    tft.print("Blk");
  } else if ((bmscan.bms_data.bat_status1 & 0x04) && !(bmscan.bms_data.bat_status1 & 0x08)){  // xxxx01xx
    TFTTXTMAGENTA;
    tft.print("Eq");
  } else if (!(bmscan.bms_data.bat_status1 & 0x04) && (bmscan.bms_data.bat_status1 & 0x08)){  // xxxx10xx
    TFTTXTMAGENTA;
    tft.print("Flt");
  } else if ((bmscan.bms_data.bat_status1 & 0x04) && (bmscan.bms_data.bat_status1 & 0x08)){  // xxxx11xx
    TFTTXTMAGENTA;
    tft.print("!Sig");
  }

  charBlank(3, 39, 16);  // (character X pos, character  pos, number of characters)
  charSetCursor(3,39);
  TFTTXTCYAN;
  tft.print(WiFi.localIP());

  charBlank(27, 39, 8);  // (character X pos, character  pos, number of characters)
  charSetCursor(27,39);
  tft.setTextColor(ILI9341_GREENYELLOW);
  tft.print("V ");
  float code_rev = (float)bmscan.bms_data.fw_code_rev;
  tft.print(code_rev / 10);
}

// *****************************************
// Calculate battery cell delta
// *****************************************
int calc_cell_delta (int battcellvolt1, int battcellvolt2, int battcellvolt3, int battcellvolt4){
  int battcellhighest;
  int battcelllowest;
  int battcelldelta;
  battcellhighest = battcellvolt1;
  if (battcellvolt2 >= battcellhighest) {
    battcellhighest = battcellvolt2;
  }
  if (battcellvolt3 >= battcellhighest) {
    battcellhighest = battcellvolt3;
  }
  if (battcellvolt4 >= battcellhighest) {
    battcellhighest = battcellvolt4;
  }
  battcelllowest = battcellvolt1;
  if (battcellvolt2 <= battcelllowest) {
    battcelllowest = battcellvolt2;
  }
  if (battcellvolt3 <= battcelllowest) {
    battcelllowest = battcellvolt3;
  }
  if (battcellvolt4 <= battcelllowest) {
    battcelllowest = battcellvolt4;
  }
  battcelldelta = battcellhighest - battcelllowest;
  return battcelldelta;
}

// *****************************************
// Get battery readings and return JSON object for web server
// *****************************************
String getBatteryReadings(){
  
  int batt_delta_1 = calc_cell_delta(bmscan.bms_data.mod1_cell_1_V,
                                     bmscan.bms_data.mod1_cell_2_V,
                                     bmscan.bms_data.mod1_cell_3_V,
                                     bmscan.bms_data.mod1_cell_4_V);
  int batt_delta_2 = calc_cell_delta(bmscan.bms_data.mod2_cell_1_V,
                                     bmscan.bms_data.mod2_cell_2_V,
                                     bmscan.bms_data.mod2_cell_3_V,
                                     bmscan.bms_data.mod2_cell_4_V);
  int batt_delta_3 = calc_cell_delta(bmscan.bms_data.mod3_cell_1_V,
                                     bmscan.bms_data.mod3_cell_2_V,
                                     bmscan.bms_data.mod3_cell_3_V,
                                     bmscan.bms_data.mod3_cell_4_V);
  int batt_delta_4 = calc_cell_delta(bmscan.bms_data.mod4_cell_1_V,
                                     bmscan.bms_data.mod4_cell_2_V,
                                     bmscan.bms_data.mod4_cell_3_V,
                                     bmscan.bms_data.mod4_cell_4_V);
  int batt_delta_5 = calc_cell_delta(bmscan.bms_data.mod5_cell_1_V,
                                     bmscan.bms_data.mod5_cell_2_V,
                                     bmscan.bms_data.mod5_cell_3_V,
                                     bmscan.bms_data.mod5_cell_4_V);
  int batt_delta_6 = calc_cell_delta(bmscan.bms_data.mod6_cell_1_V,
                                     bmscan.bms_data.mod6_cell_2_V,
                                     bmscan.bms_data.mod6_cell_3_V,
                                     bmscan.bms_data.mod6_cell_4_V);
  int batt_delta_7 = calc_cell_delta(bmscan.bms_data.mod7_cell_1_V,
                                     bmscan.bms_data.mod7_cell_2_V,
                                     bmscan.bms_data.mod7_cell_3_V,
                                     bmscan.bms_data.mod7_cell_4_V);
  int batt_delta_8 = calc_cell_delta(bmscan.bms_data.mod8_cell_1_V,
                                     bmscan.bms_data.mod8_cell_2_V,
                                     bmscan.bms_data.mod8_cell_3_V,
                                     bmscan.bms_data.mod8_cell_4_V);
                                      
  readings["batt1data1"] = String(bmscan.bms_data.mod1_cell_1_V); // Cell 1 mV
  readings["batt1data2"] = String(bmscan.bms_data.mod1_cell_2_V); // Cell 2 mV
  readings["batt1data3"] = String(bmscan.bms_data.mod1_cell_3_V); // Cell 3 mV
  readings["batt1data4"] = String(bmscan.bms_data.mod1_cell_4_V); // Cell 4 mV
  readings["batt1data5"] = String((bmscan.bms_data.mod1_cell_1_V
                                  +bmscan.bms_data.mod1_cell_2_V
                                  +bmscan.bms_data.mod1_cell_3_V
                                  +bmscan.bms_data.mod1_cell_4_V)); // Total voltage
  readings["batt1data6"] = String(bmscan.bms_data.mod1_temp); // Cell 'C
  readings["batt1data7"] = String(bmscan.bms_data.mod1_temp); // Cell 'C
  readings["batt1data8"] = String(bmscan.bms_data.mod1_temp); // Cell 'C
  readings["batt1data9"] = String(bmscan.bms_data.mod1_temp); // Cell 'C
  readings["batt1data10"] = String(bmscan.bms_data.mod1_pcb_temp); // PCB 'C
  readings["batt1data11"] = 0;  // Vstat
  readings["batt1data12"] = 0;  // Tstat
  readings["batt1data13"] = String(bmscan.bms_data.mod1_curr);  // Current
  readings["batt1data14"] = String(bmscan.bms_data.mod_bal_stat & 0x01);  // Balance
  readings["batt1data15"] = String(bmscan.bms_data.mod_1_soc);  // SOC
  readings["batt1data16"] = String(batt_delta_1);  // Delta
  
  readings["batt2data1"] = String(bmscan.bms_data.mod2_cell_1_V);
  readings["batt2data2"] = String(bmscan.bms_data.mod2_cell_2_V);
  readings["batt2data3"] = String(bmscan.bms_data.mod2_cell_3_V);
  readings["batt2data4"] = String(bmscan.bms_data.mod2_cell_4_V);
  readings["batt2data5"] = String((bmscan.bms_data.mod2_cell_1_V
                                  +bmscan.bms_data.mod2_cell_2_V
                                  +bmscan.bms_data.mod2_cell_3_V
                                  +bmscan.bms_data.mod2_cell_4_V));
  readings["batt2data6"] = String(bmscan.bms_data.mod2_temp);
  readings["batt2data7"] = String(bmscan.bms_data.mod2_temp);
  readings["batt2data8"] = String(bmscan.bms_data.mod2_temp);
  readings["batt2data9"] = String(bmscan.bms_data.mod2_temp);
  readings["batt2data10"] = String(bmscan.bms_data.mod2_pcb_temp);
  readings["batt2data11"] = 0;
  readings["batt2data12"] = 0;
  readings["batt2data13"] = String(bmscan.bms_data.mod2_curr);
  readings["batt2data14"] = String(bmscan.bms_data.mod_bal_stat & 0x02);
  readings["batt2data15"] = String(bmscan.bms_data.mod_2_soc);
  readings["batt2data16"] = String(batt_delta_2);
  
  readings["batt3data1"] = String(bmscan.bms_data.mod3_cell_1_V);
  readings["batt3data2"] = String(bmscan.bms_data.mod3_cell_2_V);
  readings["batt3data3"] = String(bmscan.bms_data.mod3_cell_3_V);
  readings["batt3data4"] = String(bmscan.bms_data.mod3_cell_4_V);
  readings["batt3data5"] = String((bmscan.bms_data.mod3_cell_1_V
                                  +bmscan.bms_data.mod3_cell_2_V
                                  +bmscan.bms_data.mod3_cell_3_V
                                  +bmscan.bms_data.mod3_cell_4_V));
  readings["batt3data6"] = String(bmscan.bms_data.mod3_temp);
  readings["batt3data7"] = String(bmscan.bms_data.mod3_temp);
  readings["batt3data8"] = String(bmscan.bms_data.mod3_temp);
  readings["batt3data9"] = String(bmscan.bms_data.mod3_temp);
  readings["batt3data10"] = String(bmscan.bms_data.mod3_pcb_temp);
  readings["batt3data11"] = 0;
  readings["batt3data12"] = 0;
  readings["batt3data13"] = String(bmscan.bms_data.mod3_curr);
  readings["batt3data14"] = String(bmscan.bms_data.mod_bal_stat & 0x04);
  readings["batt3data15"] = String(bmscan.bms_data.mod_3_soc);
  readings["batt3data16"] = String(batt_delta_3);
  
  readings["batt4data1"] = String(bmscan.bms_data.mod4_cell_1_V);
  readings["batt4data2"] = String(bmscan.bms_data.mod4_cell_2_V);
  readings["batt4data3"] = String(bmscan.bms_data.mod4_cell_3_V);
  readings["batt4data4"] = String(bmscan.bms_data.mod4_cell_4_V);
  readings["batt4data5"] = String((bmscan.bms_data.mod4_cell_1_V
                                  +bmscan.bms_data.mod4_cell_2_V
                                  +bmscan.bms_data.mod4_cell_3_V
                                  +bmscan.bms_data.mod4_cell_4_V));
  readings["batt4data6"] = String(bmscan.bms_data.mod4_temp);
  readings["batt4data7"] = String(bmscan.bms_data.mod4_temp);
  readings["batt4data8"] = String(bmscan.bms_data.mod4_temp);
  readings["batt4data9"] = String(bmscan.bms_data.mod4_temp);
  readings["batt4data10"] = String(bmscan.bms_data.mod4_pcb_temp);
  readings["batt4data11"] = 0;
  readings["batt4data12"] = 0;
  readings["batt4data13"] = String(bmscan.bms_data.mod4_curr);
  readings["batt4data14"] = String(bmscan.bms_data.mod_bal_stat & 0x08);
  readings["batt4data15"] = String(bmscan.bms_data.mod_4_soc);
  readings["batt4data16"] = String(batt_delta_4);
  
  readings["batt5data1"] = String(bmscan.bms_data.mod5_cell_1_V);
  readings["batt5data2"] = String(bmscan.bms_data.mod5_cell_2_V);
  readings["batt5data3"] = String(bmscan.bms_data.mod5_cell_3_V);
  readings["batt5data4"] = String(bmscan.bms_data.mod5_cell_4_V);
  readings["batt5data5"] = String((bmscan.bms_data.mod5_cell_1_V
                                  +bmscan.bms_data.mod5_cell_2_V
                                  +bmscan.bms_data.mod5_cell_3_V
                                  +bmscan.bms_data.mod5_cell_4_V));
  readings["batt5data6"] = String(bmscan.bms_data.mod5_temp);
  readings["batt5data7"] = String(bmscan.bms_data.mod5_temp);
  readings["batt5data8"] = String(bmscan.bms_data.mod5_temp);
  readings["batt5data9"] = String(bmscan.bms_data.mod5_temp);
  readings["batt5data10"] = String(bmscan.bms_data.mod5_pcb_temp);
  readings["batt5data11"] = 0;
  readings["batt5data12"] = 0;
  readings["batt5data13"] = String(bmscan.bms_data.mod5_curr);
  readings["batt5data14"] = String(bmscan.bms_data.mod_bal_stat & 0x10);
  readings["batt5data15"] = String(bmscan.bms_data.mod_5_soc);
  readings["batt5data16"] = String(batt_delta_5);
  
  readings["batt6data1"] = String(bmscan.bms_data.mod6_cell_1_V);
  readings["batt6data2"] = String(bmscan.bms_data.mod6_cell_2_V);
  readings["batt6data3"] = String(bmscan.bms_data.mod6_cell_3_V);
  readings["batt6data4"] = String(bmscan.bms_data.mod6_cell_4_V);
  readings["batt6data5"] = String((bmscan.bms_data.mod6_cell_1_V
                                  +bmscan.bms_data.mod6_cell_2_V
                                  +bmscan.bms_data.mod6_cell_3_V
                                  +bmscan.bms_data.mod6_cell_4_V));
  readings["batt6data6"] = String(bmscan.bms_data.mod6_temp);
  readings["batt6data7"] = String(bmscan.bms_data.mod6_temp);
  readings["batt6data8"] = String(bmscan.bms_data.mod6_temp);
  readings["batt6data9"] = String(bmscan.bms_data.mod6_temp);
  readings["batt6data10"] = String(bmscan.bms_data.mod6_pcb_temp);
  readings["batt6data11"] = 0;
  readings["batt6data12"] = 0;
  readings["batt6data13"] = String(bmscan.bms_data.mod6_curr);
  readings["batt6data14"] = String(bmscan.bms_data.mod_bal_stat & 0x20);
  readings["batt6data15"] = String(bmscan.bms_data.mod_6_soc);
  readings["batt6data16"] = String(batt_delta_6);
  
  readings["batt7data1"] = String(bmscan.bms_data.mod7_cell_1_V);
  readings["batt7data2"] = String(bmscan.bms_data.mod7_cell_2_V);
  readings["batt7data3"] = String(bmscan.bms_data.mod7_cell_3_V);
  readings["batt7data4"] = String(bmscan.bms_data.mod7_cell_4_V);
  readings["batt7data5"] = String((bmscan.bms_data.mod7_cell_1_V
                                  +bmscan.bms_data.mod7_cell_2_V
                                  +bmscan.bms_data.mod7_cell_3_V
                                  +bmscan.bms_data.mod7_cell_4_V));
  readings["batt7data6"] = String(bmscan.bms_data.mod7_temp);
  readings["batt7data7"] = String(bmscan.bms_data.mod7_temp);
  readings["batt7data8"] = String(bmscan.bms_data.mod7_temp);
  readings["batt7data9"] = String(bmscan.bms_data.mod7_temp);
  readings["batt7data10"] = String(bmscan.bms_data.mod7_pcb_temp);
  readings["batt7data11"] = 0;
  readings["batt7data12"] = 0;
  readings["batt7data13"] = String(bmscan.bms_data.mod7_curr);
  readings["batt7data14"] = String(bmscan.bms_data.mod_bal_stat & 0x40);
  readings["batt7data15"] = String(bmscan.bms_data.mod_7_soc);
  readings["batt7data16"] = String(batt_delta_7);
  
  readings["batt8data1"] = String(bmscan.bms_data.mod8_cell_1_V);
  readings["batt8data2"] = String(bmscan.bms_data.mod8_cell_2_V);
  readings["batt8data3"] = String(bmscan.bms_data.mod8_cell_3_V);
  readings["batt8data4"] = String(bmscan.bms_data.mod8_cell_4_V);
  readings["batt8data5"] = String((bmscan.bms_data.mod8_cell_1_V
                                  +bmscan.bms_data.mod8_cell_2_V
                                  +bmscan.bms_data.mod8_cell_3_V
                                  +bmscan.bms_data.mod8_cell_4_V));
  readings["batt8data6"] = String(bmscan.bms_data.mod8_temp);
  readings["batt8data7"] = String(bmscan.bms_data.mod8_temp);
  readings["batt8data8"] = String(bmscan.bms_data.mod8_temp);
  readings["batt8data9"] = String(bmscan.bms_data.mod8_temp);
  readings["batt8data10"] = String(bmscan.bms_data.mod8_pcb_temp);
  readings["batt8data11"] = 0;
  readings["batt8data12"] = 0;
  readings["batt8data13"] = String(bmscan.bms_data.mod8_curr);
  readings["batt8data14"] = String(bmscan.bms_data.mod_bal_stat & 0x80);
  readings["batt8data15"] = String(bmscan.bms_data.mod_1_soc);
  readings["batt8data16"] = String(batt_delta_8);
  
  readings["battchrelay"] = "OFF";
  readings["battldrelay"] = "OFF";
  
  String jsonString = JSON.stringify(readings);
  return jsonString;
}
