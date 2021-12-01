
// ============================================================
// Project:     Pump Controller
// Author:      Christopher Connolly
// Last update: 17.11.2021
// Description: Control solar and pool pumps
// ============================================================
// Dependencies:
// Uses DallasTemperature https://github.com/milesburton/Arduino-Temperature-Control-Library
// Uses TaskScheduler     https://github.com/arkhipenko/TaskScheduler
// Uses ESPAsyncWebServer https://github.com/me-no-dev/ESPAsyncWebServer
// Uses AsyncTCP          https://github.com/me-no-dev/AsyncTCP
//  ============================================================
#include <ArduinoOTA.h>
#include <ArduinoJson.h>    // Used to output data to JSON
#include <EEPROM.h>         // Reading and Writing to EEPROM
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TaskScheduler.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FS.h>
#include <SPIFFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFSEditor.h>

#ifndef STASSID
#define STASSID "wifi-ap-name"
#define STAPSK  "wifi-password"
#endif

#define DEBUG;
#define _APP_NAME       "Conno Pump Controller"
#define _APP_VERSION    5.0

//// Data wire is plugged into pin 2 on the ESP32
#define ONE_WIRE_BUS 14

// define the pump
#define POOL_PUMP 13
#define SOLAR_PUMP 12

// time between switching pump on or off in milliseconds
#define MIN_SWITCHING_TIME 5000

// *********************************************************************
// Task Scheduling
// *********************************************************************
//#define _TASK_TIMECRITICAL
//#define _TASK_SLEEP_ON_IDLE_RUN // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass 
#define _TASK_TIMEOUT

const char* ssid = STASSID;
const char* password = STAPSK;
const char* http_username = "admin";
const char* http_password = "admin";

const boolean daylightSavings = false;

WiFiUDP ntpUDP;

// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
NTPClient timeClient(ntpUDP, "au.pool.ntp.org", 3600, 60000);


// ********************************************************************
// Temp Sensors
// ********************************************************************
// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);


// ********************************************************************
// Web server
// ********************************************************************
AsyncWebServer server(80);
AsyncEventSource events("/events");

// *********************************************************************
// GLOBALS
// *********************************************************************
boolean g_dirty = false;      // True when EEPROM memory needs to be saved

int g_clock_hour = 0;
int g_clock_minute = 0;

int g_poolOnHour = 11;        // Turn pool ON   XX:00
int g_poolOnMinute = 0;       // Turn pool ON   00:XX
int g_poolOffHour = 16;       // Turn pool OFF  XX:00
int g_poolOffMinute = 0;      // Turn pool OFF  00:XX

int g_solarOnHour = 7;        // Turn pool ON   XX:00
int g_solarOnMinute = 0;      // Turn pool ON   00:XX
int g_solarOffHour = 17;      // Turn pool OFF  XX:00
int g_solarOffMinute = 0;     // Turn pool OFF  00:XX

float g_solarOnTemp = 21.0;   // Temperature    XX.XX C
float g_solarOffTemp = 30.0;  // Temperature    XX.XX C

float tempC = 0.0;
float minC = 100.0;
float maxC = 0.0;

int g_poolPumpState = LOW;
int g_solarPumpState = LOW;

boolean g_callForPoolPump = false;
boolean g_callForSolarPump = false;

boolean g_poolOnOverride = false;
boolean g_solarOnOverride = false;

unsigned long g_lastPoolPumpChange = 0; // Last on or off event, point in time
unsigned long g_lastSolarPumpChange = 0; // Last on or off event, point in time

// *********************************************************************
// Task Prototypes
// *********************************************************************
// Callback methods prototypes
void t1CheckTemp();
void t2SaveMem();
void t4RemoteCommands();
void t5Clock();
void t6Callback();

// Task Scheduler
Scheduler runner;

// Tasks
Task t1(TASK_SECOND * 10, TASK_FOREVER, &t1CheckTemp);
Task t2(TASK_SECOND, TASK_FOREVER, &t2SaveMem);
Task t4(250, TASK_FOREVER, &t4RemoteCommands);
Task t5(TASK_MINUTE, TASK_FOREVER, &t5Clock);
Task t6(TASK_SECOND, TASK_FOREVER, &t6Callback);

// *********************************************************************
// SETUP
// *********************************************************************
void setup()
{
  // Set pump pins
  pinMode(POOL_PUMP, OUTPUT); // Set pump pin
  pinMode(SOLAR_PUMP, OUTPUT); // Set pump pin
  
  // initialize EEPROM with predefined size
  EEPROM.begin(11);
  loadSavedOrDefaults(); // Load saved or default values
  
  // serial init; only be needed if serial control is used
  Serial.begin(115200);                // start serial
  Serial.println("Connotron ESP32 Pump Controller, Booting");

  // Mount SPIFFS
  if(!SPIFFS.begin()){
     Serial.println("An Error has occurred while mounting SPIFFS");
     return;
   }
   
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Configure Arduino OTA
  ArduinoOTA.onStart([]() { 
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) type = "sketch";
      else type = "filesystem"; // U_SPIFFS
      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
      events.send("Update Start", "ota"); 
    });
    
  ArduinoOTA.onEnd([]() { 
    events.send("Update End", "ota"); 
    Serial.println("\nOTA updated End");
    });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
 
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Set up mDNS responder:
  // - first argument is the domain name, in this example
  //   the fully-qualified domain name is "esp8266.local"
  // - second argument is the IP address to advertise
  //   we send our IP address on the WiFi network
  if (MDNS.begin("pool")) {
    Serial.println("MDNS responder started");
  }
  
  Serial.println("mDNS responder started");

  timeClient.begin();
  if(daylightSavings) {
    timeClient.setTimeOffset(36000);
  } else {
    timeClient.setTimeOffset(36000+3600);
  }
  Serial.println("NTP Client started");

  // Add service to MDNS-SD
  MDNS.addService("http", "tcp", 80);

  SPIFFS.begin();
  Serial.println("SPIFFS begin");

  events.onConnect([](AsyncEventSourceClient *client){
    client->send("hello!",NULL,millis(),1000);
  });
  server.addHandler(&events);

  server.addHandler(new SPIFFSEditor(SPIFFS, http_username,http_password));

  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });

  server.on("/api/set/pool", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasParam("cmd")) {
      AsyncWebParameter* p = request->getParam("cmd");
      AsyncWebParameter* h = request->getParam("hour");
      AsyncWebParameter* m = request->getParam("minute");
      if (p->value() == "on" || p->value() == "off") {
        if(request->hasParam("hour") && request->hasParam("minute")) {
          if (p->value() == "on") {
            g_poolOnHour = h->value().toInt();
            g_poolOnMinute = m->value().toInt();
            request->send(200, "application/json", String("{\"success\":\"Pool on time set\"}"));
          } else if (p->value() == "off") {
            g_poolOffHour = h->value().toInt();
            g_poolOffMinute = m->value().toInt();
            request->send(200, "application/json", String("{\"success\":\"Pool off time set\" }"));
          } else {
            request->send(400, "application/json", String("{\"error\":\"Pool cmd param unknown\"}"));
          }
        } else {
          request->send(400, "application/json", String("{\"error\":\"Missing hour or minute params\"}"));
        }
      } else {
        request->send(400, "application/json", String("{\"error\":\"Missing cmd param not found\"}"));
      }
    } else {
      request->send(400, "application/json", String("{\"error\":\"API cmd params missing\"}"));
    }
  });

  server.on("/api/manual/pool", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasParam("cmd")) {
      AsyncWebParameter* p = request->getParam("cmd");
      if (p->value() == "on") {
        g_poolOnOverride = true;
        request->send(200, "application/json", String("{\"success\":\"Pool manual mode on\" }"));
      } else if (p->value() == "off") {
        g_poolOnOverride = false;
        request->send(200, "application/json", String("{\"success\":\"Pool manual mode off\" }"));
      } else {
        request->send(400, "application/json", String("{\"error\":\"Invalid cmd param value\"}"));
      }
    } else {
      request->send(400, "application/json", String("{\"error\":\"Missing cmd param\"}"));
    }
  });
  
  server.on("/api/manual/solar", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasParam("cmd")) {
      AsyncWebParameter* p = request->getParam("cmd");
      if (p->value() == "on") {
        g_solarOnOverride = true;
        request->send(200, "application/json", String("{\"success\":\"Solar manual mode on\" }"));
      } else if (p->value() == "off") {
        g_solarOnOverride = false;
        request->send(200, "application/json", String("{\"success\":\"Solar manual mode off\" }"));
      } else {
        request->send(400, "application/json", String("{\"error\":\"Invalid cmd param value\"}"));
      }
    } else {
      request->send(400, "application/json", String("{\"error\":\"Missing cmd param\"}"));
    }
  });
  
  server.on("/api/set/solar", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasParam("cmd")) {
      AsyncWebParameter* p = request->getParam("cmd");
      AsyncWebParameter* h = request->getParam("hour");
      AsyncWebParameter* m = request->getParam("minute");
      AsyncWebParameter* v = request->getParam("value");
      if (p->value() == "on" || p->value() == "off") {
        if(request->hasParam("hour") && request->hasParam("minute")) {
          if (p->value() == "on") {
            g_solarOnHour = h->value().toInt();
            g_solarOnMinute = m->value().toInt();
            request->send(200, "application/json", String("{\"success\":\"Solar on time set\"}"));
          } else if (p->value() == "off") {
            g_solarOffHour = h->value().toInt();
            g_solarOffMinute = m->value().toInt();
            request->send(200, "application/json", String("{\"success\":\"Solar off time set\" }"));
          } else {
            request->send(400, "application/json", String("{\"error\":\"Solar cmd param unknown\"}"));
          }
        } else {
          request->send(400, "application/json", String("{\"error\":\"Missing hour or minute params\"}"));
        }
      } else if(p->value() == "min") {
        g_solarOnTemp = v->value().toInt();
        request->send(200, "application/json", String("{\"success\":\"Solar min temp set\"}"));
      } else if(p->value() == "max") {
        g_solarOffTemp = v->value().toInt();
        request->send(200, "application/json", String("{\"success\":\"Solar max temp set\"}"));
      } else {
        request->send(400, "application/json", String("{\"error\":\"Missing cmd param not found\"}"));
      }
    } else {
      request->send(400, "application/json", String("{\"error\":\"API cmd params missing\"}"));
    }
  });
  
  
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");

  server.onNotFound([](AsyncWebServerRequest *request){
    Serial.printf("NOT_FOUND: ");
    if(request->method() == HTTP_GET)
      Serial.printf("GET");
    else if(request->method() == HTTP_POST)
      Serial.printf("POST");
    else if(request->method() == HTTP_DELETE)
      Serial.printf("DELETE");
    else if(request->method() == HTTP_PUT)
      Serial.printf("PUT");
    else if(request->method() == HTTP_PATCH)
      Serial.printf("PATCH");
    else if(request->method() == HTTP_HEAD)
      Serial.printf("HEAD");
    else if(request->method() == HTTP_OPTIONS)
      Serial.printf("OPTIONS");
    else
      Serial.printf("UNKNOWN");
    Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());

    if(request->contentLength()){
      Serial.printf("_CONTENT_TYPE: %s\n", request->contentType().c_str());
      Serial.printf("_CONTENT_LENGTH: %u\n", request->contentLength());
    }

    int headers = request->headers();
    int i;
    for(i=0;i<headers;i++){
      AsyncWebHeader* h = request->getHeader(i);
      Serial.printf("_HEADER[%s]: %s\n", h->name().c_str(), h->value().c_str());
    }

    int params = request->params();
    for(i=0;i<params;i++){
      AsyncWebParameter* p = request->getParam(i);
      if(p->isFile()){
        Serial.printf("_FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
      } else if(p->isPost()){
        Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
      } else {
        Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
      }
    }

    if (request->method() == HTTP_OPTIONS) {
      request->send(200);
    } else {
      request->send(404);
    }
  });
  
  server.onFileUpload([](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final){
    if(!index)
      Serial.printf("UploadStart: %s\n", filename.c_str());
    Serial.printf("%s", (const char*)data);
    if(final)
      Serial.printf("UploadEnd: %s (%u)\n", filename.c_str(), index+len);
  });
  
  server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    if(!index)
      Serial.printf("BodyStart: %u\n", total);
    Serial.printf("%s", (const char*)data);
    if(index + len == total)
      Serial.printf("BodyEnd: %u\n", total);
  });

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  
  server.begin();

  // wait for MAX chip to stabilize
  // Start up the library
  sensors.begin();

  delay(250);
  
  runner.init();

  runner.addTask(t1); // Temperature check
  runner.addTask(t2); // Save memory
  runner.addTask(t4); // Remote commands
  runner.addTask(t5); // Clock
  runner.addTask(t6); // Pump controllers
  
  t1.enable();
  t2.enable();
  t4.enable();
  t5.enable();
  t6.enable();
}

// *********************************************************************
// EEPROM
// *********************************************************************
void writeIntIntoEEPROM(int address, int number)
{ 
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
  EEPROM.commit();
}

int readIntFromEEPROM(int address)
{
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}

void loadSavedOrDefaults() {
  if(readIntFromEEPROM(1) == 1) {
    loadDefaults();
  } else {
     saveDefaults();
  }
}

void loadDefaults() {
  g_poolOnHour = readIntFromEEPROM(2);
  g_poolOnMinute = readIntFromEEPROM(3);
  g_poolOffHour = readIntFromEEPROM(4);
  g_poolOffMinute = readIntFromEEPROM(5);
  
  g_solarOnHour = readIntFromEEPROM(6);
  g_solarOnMinute = readIntFromEEPROM(7);
  g_solarOffHour = readIntFromEEPROM(8);
  g_solarOffMinute = readIntFromEEPROM(9);
  
  g_solarOnTemp = readIntFromEEPROM(10) / 10;
  g_solarOffTemp = readIntFromEEPROM(11) / 10;
}

void saveDefaults() {
  writeIntIntoEEPROM(1, 1);
  writeIntIntoEEPROM(2, g_poolOnHour);
  writeIntIntoEEPROM(3, g_poolOnMinute);
  writeIntIntoEEPROM(4, g_poolOffHour);
  writeIntIntoEEPROM(5, g_poolOffMinute);
  
  writeIntIntoEEPROM(6, g_solarOnHour);
  writeIntIntoEEPROM(7, g_solarOnMinute);
  writeIntIntoEEPROM(8, g_solarOffHour);
  writeIntIntoEEPROM(9, g_solarOffMinute);
  
  writeIntIntoEEPROM(10, g_solarOnTemp * 10);
  writeIntIntoEEPROM(11, g_solarOffTemp * 10);
}


// *********************************************************************
// LOOP
// *********************************************************************
void loop()
{
  ArduinoOTA.handle();
  runner.execute(); // this function must called here, do not delete it
  timeClient.update();
}

// *********************************************************************
// PRINT STATE
// *********************************************************************
char* getStateJson() {
  StaticJsonDocument<410> doc;
  doc["currentHour"] = g_clock_hour;
  doc["currentMinute"] = g_clock_minute;
  doc["temp"] = tempC;
  doc["minTemp"] = minC;
  doc["maxTemp"] = maxC;
    
  // POOL
  doc["poolOverride"] = g_poolOnOverride == true ? 1 : 0;
  doc["poolCall"] = g_callForPoolPump ==  true ? 1 : 0;
  doc["poolState"] = g_poolPumpState;
  doc["poolOnHour"] = g_poolOnHour;
  doc["poolOnMinute"] = g_poolOnMinute;
  doc["poolOffHour"] = g_poolOffHour;
  doc["poolOffMinute"] = g_poolOffMinute;
  
  // SOLAR
  doc["solarOverride"] = g_solarOnOverride == true ? 1 : 0;
  doc["solarCall"] = g_callForSolarPump  ==  true ? 1 : 0;
  doc["solarState"] = g_solarPumpState;
  doc["solarOnHour"] = g_solarOnHour;
  doc["solarOnMinute"] = g_solarOnMinute;
  doc["solarOffHour"] = g_solarOffHour;
  doc["solarOffMinute"] = g_solarOffMinute;
  doc["solarOnTemp"] = g_solarOnTemp;
  doc["solarOffTemp"] = g_solarOffTemp;

  static char buf [400];
  serializeJson(doc, buf);
  return buf;
}

void printJsonState() {
  char * j = getStateJson();
  Serial.println(j);
  events.send(j,"status", millis()); 
}


// -------------------------------------
// TASK: Get the temp
// -------------------------------------
void t1CheckTemp() {
  // Get temp
  sensors.requestTemperatures(); // Send the command to get temperature readings
  tempC = sensors.getTempCByIndex(0);

  if (tempC > maxC) maxC = tempC;
  if (tempC < minC) minC = tempC;

  Serial.print(F("INFO: t1: Check temp: "));
  Serial.println(tempC);
}

// -------------------------------------
// TASK: Save memory if dirty
// -------------------------------------
void t2SaveMem() {
  if(g_dirty == true) {
    saveDefaults(); // Save memory if dirty
    g_dirty = false;
  }
}

// -------------------------------------
// TASK: Process remote commands
// -------------------------------------
void t4RemoteCommands() {
  doRemoteCommands();
}

// -------------------------------------
// TASK: Increment internal clock
// -------------------------------------
void t5Clock() {
  Serial.print(F("INFO: T5 Clock minute increment time client says it's "));
  Serial.println(timeClient.getFormattedTime());
  
  if(timeClient.isTimeSet()) {
    g_clock_hour = timeClient.getHours();
    g_clock_minute = timeClient.getMinutes();
  } else {
    g_clock_minute++;
    if(g_clock_minute > 59) {
      g_clock_hour++;
      g_clock_minute = 0;
      }
    if(g_clock_hour >= 24) {
      g_clock_hour = 0;
      }
    }
    
  if(g_clock_hour == 0 && g_clock_minute == 0) {
    minC = tempC;
    maxC = tempC;
  }
}

// -------------------------------------
// TASK: Pool and Solar Pump Tasks
// -------------------------------------
void t6Callback() {
  // Check if pool and solar pumps should be on

  // POOL
  g_callForPoolPump = false;
  
  if (g_clock_hour == g_poolOnHour) {
    if (g_clock_minute >= g_poolOnMinute) {
      // Call for pump
      g_callForPoolPump = HIGH;
    }
  } else if (g_clock_hour > g_poolOnHour) {
    g_callForPoolPump = HIGH;
  }
  
  if (g_clock_hour == g_poolOffHour ) {
    if (g_clock_minute >= g_poolOffMinute) {
      // Call for pump
      g_callForPoolPump = LOW;
    }
  } else if(g_clock_hour > g_poolOffHour ) {
    g_callForPoolPump = LOW;
  }

  // Check manual override
  if (g_poolOnOverride) g_callForPoolPump = true;

  // IMPORTANT -- This logic handles the pump minimum on/off time
  unsigned long currentMillis = millis();
  unsigned long diffPool = currentMillis - g_lastPoolPumpChange;

  if (g_callForPoolPump != g_poolPumpState && (MIN_SWITCHING_TIME >= diffPool )) {
//    Serial.print(F("WARN: t2: Minimum pool pump switching on/off time no yet reached , wait another "));
//    Serial.println(MIN_SWITCHING_TIME - diff);
  }
  else if (g_callForPoolPump != g_lastPoolPumpChange) {
    g_poolPumpState = g_callForPoolPump;
    digitalWrite(POOL_PUMP, g_poolPumpState);
    g_lastPoolPumpChange = millis();
  }

  
  // SOLAR
  g_callForSolarPump = false;
  
  if (g_clock_hour == g_solarOnHour) {
    if (g_clock_minute >= g_solarOnMinute) {
      // Call for pump
      g_callForSolarPump = HIGH;
    }
  } else if (g_clock_hour > g_solarOnHour) {
    g_callForSolarPump = HIGH;
  }

    // Overwrite solar control based on temp
  if(tempC <= g_solarOnTemp) g_callForSolarPump = LOW;
  if(tempC >= g_solarOffTemp) g_callForSolarPump = LOW;

  if (g_clock_hour == g_solarOffHour ) {
    if (g_clock_minute >= g_solarOffMinute) {
      // Call for pump
      g_callForSolarPump = LOW;
    }
  } else if(g_clock_hour > g_solarOffHour ) {
    g_callForSolarPump = LOW;
  }

  // Check manual override
  if (g_solarOnOverride) g_callForSolarPump = true;
  
  // IMPORTANT -- This logic handles the pump minimum on/off time
  unsigned long diffSolar = currentMillis - g_lastSolarPumpChange;
  
  if (g_callForSolarPump != g_solarPumpState && (MIN_SWITCHING_TIME >= diffSolar )) {
    Serial.print(F("WARN: t3: Minimum solar pump switching on/off time no yet reached , wait another "));
    Serial.println(MIN_SWITCHING_TIME - diffSolar);
  }
  else if (g_callForSolarPump != g_solarPumpState) {
    g_solarPumpState = g_callForSolarPump;
    digitalWrite(SOLAR_PUMP, g_solarPumpState);
    Serial.print(F("DEBUG: t3: Controlling Solar Pump val: "));
    Serial.println(g_lastSolarPumpChange);
    g_lastSolarPumpChange = millis();
  }
  printJsonState();
}
