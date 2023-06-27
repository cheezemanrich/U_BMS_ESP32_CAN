# U_BMS_ESP32_CAN
Arduino project for an ESP32-DevKitC to listen to and decode CAN bus messages from a Valence U-BMS, process the info, and then display on a ILI9341 display and web page. The CAN interface used is a TJA1051 as it can be connected direct using 3.3V. Do not use a TJA1050 as it needs 5V and requires level translation. The software is setup for a 2S4P U27-12XP configuration and 250K CAN Bus.
