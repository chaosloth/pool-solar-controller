# ESP32 Pool pump and solar pump controller
Simple Arduino based application for ESP32 that provides a simple pool controller

Features:
- NTP Client to get current time
- mDNS advertizment for easy access
- ESP Async Web Server with SPIFFS
- OTA updates for firmware
- Dallas OneWire temp sensor (DS18b20 based)
- Controls 2x 5v relays
- Writes settings to EEPROM
- Pump switching time based lock out to prevent flip flops
- Mobile UI in JS/HTML
- Serial command interface
