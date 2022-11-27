#include <ArduinoJson.h>        // https://github.com/bblanchon/ArduinoJson
#include <AsyncTCP.h>           // https://github.com/me-no-dev/AsyncTCP
#include <ESPAsyncWebServer.h>  // https://github.com/me-no-dev/ESPAsyncWebServer
#include <SPIFFS.h>
#include <WiFi.h>

#include "LD2410.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
LD2410 radar(Serial1);

const uint8_t RADAR_RX_PIN = 26;
const uint8_t RADAR_TX_PIN = 27;

const char *ssid     = "SSID";
const char *password = "PASSWORD";

// update websocket client if radar has been factory reset
bool sendRadarSettings;

// handle incoming websocket messages
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  bool result = false;

  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    StaticJsonDocument<128> doc;

    DeserializationError error = deserializeJson(doc, data);

    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    const char *command = doc["command"];

    if (strcmp(command, "setGateSensConf") == 0) {
      int datasetIndex = doc["datasetIndex"];
      int index        = doc["index"];
      int value        = doc["value"];

      if (datasetIndex == 1) {
        // update moving sensitivity
        result = radar.setGateSensConf(index, value, radar.data.parameter.stationarySensitivity[index]);
      } else if (datasetIndex == 3) {
        // update stationary sensitivity
        result = radar.setGateSensConf(index, radar.data.parameter.movingSensitivity[index], value);
      }

      sendRadarSettings = true;
      return;

    } else if (strcmp(command, "maxMovingGate") == 0) {
      uint8_t value = doc["value"];
      result        = radar.setMaxDistAndDur(value, radar.data.parameter.maxStationaryGate, radar.data.parameter.detectionTime);
    } else if (strcmp(command, "maxStationaryGate") == 0) {
      uint8_t value = doc["value"];
      result        = radar.setMaxDistAndDur(radar.data.parameter.maxMovingGate, value, radar.data.parameter.detectionTime);
    } else if (strcmp(command, "detectionTime") == 0) {
      uint32_t value = doc["value"];
      result         = radar.setMaxDistAndDur(radar.data.parameter.maxMovingGate, radar.data.parameter.maxStationaryGate, value);
    } else if (strcmp(command, "Restart") == 0) {
      result = radar.restart();
      // update baud rate
      if (result) {
        Serial1.end();
        Serial1.begin(0, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN, false, 1000);
      }
    } else if (strcmp(command, "Factory Reset") == 0) {
      result = radar.factoryReset();
    } else if (strcmp(command, "Enable Engineering Mode") == 0) {
      result = radar.enableEngMode(true);
    } else if (strcmp(command, "Disable Engineering Mode") == 0) {
      result = radar.enableEngMode(false);
    } else if (strcmp(command, "baudRate") == 0) {
      BaudRateIndex eBaud = doc["value"];
      result              = radar.setBaudRate(eBaud);
    } else if (strcmp(command, "Restart ESP") == 0) {
      result = true;
    }
    // Return if the command was executed or not
    StaticJsonDocument<50> docResult;
    String resultJson;
    JsonObject res = docResult.createNestedObject("result");

    res.getOrAddMember("success").set(result);
    res.getOrAddMember("id").set(command);
    serializeJson(docResult, resultJson);

    ws.textAll(resultJson);

    if (strcmp(command, "Restart ESP") == 0) {
      delay(50);
      ESP.restart();
    }

    // update settings on next data frame
    sendRadarSettings = true;
  }
}

// Send cyclic radar data to the clients
void wsSendCyclicData() {
  String radarDataJson;
  StaticJsonDocument<768> doc;

  // Cyclic radar data
  JsonObject radarData = doc.createNestedObject("radarData");
  radarData.getOrAddMember("radarInEngineeringMode").set(radar.data.cyclicData.radarInEngineeringMode);
  radarData.getOrAddMember("targetState").set(radar.data.cyclicData.targetState);
  radarData.getOrAddMember("movingTargetDistance").set(radar.data.cyclicData.movingTargetDistance);
  radarData.getOrAddMember("movingTargetEnergy").set(radar.data.cyclicData.movingTargetEnergy);
  radarData.getOrAddMember("stationaryTargetDistance").set(radar.data.cyclicData.stationaryTargetDistance);
  radarData.getOrAddMember("stationaryTargetEnergy").set(radar.data.cyclicData.stationaryTargetEnergy);
  radarData.getOrAddMember("detectionDistance").set(radar.data.cyclicData.detectionDistance);

  // Engineering data
  JsonObject radar_engData           = doc.createNestedObject("engineeringData");
  JsonArray radar_engData_movEnergy  = radar_engData.createNestedArray("movingEnergyGateN");
  JsonArray radar_engData_statEnergy = radar_engData.createNestedArray("stationaryEnergyGateN");

  radar_engData.getOrAddMember("maxMovingGate").set(radar.data.engineeringData.maxMovingGate);
  radar_engData.getOrAddMember("maxStationaryGate").set(radar.data.engineeringData.maxStationaryGate);

  radar_engData.getOrAddMember("maxMovingEnergy").set(radar.data.engineeringData.maxMovingEnergy);
  radar_engData.getOrAddMember("maxStationaryEnergy").set(radar.data.engineeringData.maxStationaryEnergy);

  for (uint8_t gate = 0; gate <= 8; gate++) {
    radar_engData_movEnergy.add(radar.data.engineeringData.movingEnergyGateN[gate]);
    radar_engData_statEnergy.add(radar.data.engineeringData.stationaryEnergyGateN[gate]);
  }

  serializeJson(doc, radarDataJson);
  ws.textAll(radarDataJson);
}

// Send radar firmware version to the websocket client
void wsSendRadarFirmwareVersion() {
  String firmwareVersionJson;

  StaticJsonDocument<512> doc;

  JsonObject firmwareVersion = doc.createNestedObject("firmwareVersion");

  firmwareVersion.getOrAddMember("major").set(radar.data.firmwareVersion.majorVersion);
  firmwareVersion.getOrAddMember("minor").set(radar.data.firmwareVersion.minorVersion);
  firmwareVersion.getOrAddMember("bugFix").set(radar.data.firmwareVersion.bugFixVersion);

  serializeJson(doc, firmwareVersionJson);
  ws.textAll(firmwareVersionJson);
}

// Send radar settings to the websocket client
void wsSendRadarSettings() {
  String settingsJson;

  StaticJsonDocument<512> doc;
  JsonObject settings = doc.createNestedObject("settings");

  radar.readParameter();

  settings.getOrAddMember("maxGate").set(radar.data.parameter.maxGate);
  settings.getOrAddMember("maxMovingGate").set(radar.data.parameter.maxMovingGate);
  settings.getOrAddMember("maxStationaryGate").set(radar.data.parameter.maxStationaryGate);
  settings.getOrAddMember("detectionTime").set(radar.data.parameter.detectionTime);
  settings.getOrAddMember("baudRate").set(Serial1.baudRate());
  JsonArray settings_movingSens     = settings.createNestedArray("movingSensitivity");
  JsonArray settings_stationarySens = settings.createNestedArray("stationarySensitivity");

  for (uint8_t gate = 0; gate < 9; gate++) {
    settings_movingSens.add(radar.data.parameter.movingSensitivity[gate]);
    settings_stationarySens.add(radar.data.parameter.stationarySensitivity[gate]);
  }

  serializeJson(doc, settingsJson);
  ws.textAll(settingsJson);
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      wsSendRadarSettings();
      wsSendRadarFirmwareVersion();
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;

    case WS_EVT_PONG:
      break;

    case WS_EVT_ERROR:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\nLD2410-Radar Example\n");

  // Start hardware serial on rx pin 25 and tx pin 26 with automatic baud rate detection
  Serial1.begin(0, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN, false, 1000);

  if (Serial1.baudRate()) {
    Serial.printf("\nDetected Radar Baud is: %d \n", Serial1.baudRate());
  } else {
    Serial.printf("\nFailed to detect Baud Rate of the radar, make sure you have connected the radar correctly!\n");
    Serial1.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN, false, 1000);
  }

  if (radar.begin()) {
    Serial.printf("Radar Firmware Version is: %d.%d.%d \n",
                  radar.data.firmwareVersion.majorVersion,
                  radar.data.firmwareVersion.minorVersion,
                  radar.data.firmwareVersion.bugFixVersion);

    Serial.printf("Radar detection time is: %ds.\n",
                  radar.data.parameter.detectionTime);

    Serial.printf("Radars max detection Gate is: %d.\n",
                  radar.data.parameter.maxGate);

    Serial.printf("Radars max moving Gate is %d.\n",
                  radar.data.parameter.maxMovingGate);

    Serial.printf("Radars max stationary Gate is %d.\n",
                  radar.data.parameter.maxStationaryGate);

    Serial.printf("Treshold/energy values per Gate:\n");
    for (uint8_t gate = 0; gate <= 8; gate++) {
      Serial.printf(
          "Gate %1d: moving treshold: %3d%%, stationary treshold: "
          "%3d%%.\n",
          gate, radar.data.parameter.movingSensitivity[gate],
          radar.data.parameter.stationarySensitivity[gate]);
    }
  } else {
    Serial.println("Failed to get firmware version and parameters from radar");
  }

  // Initialize SPIFFS
  if (!SPIFFS.begin()) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Start wifi
  Serial.printf("\nConnecting to Wifi ...\n");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.printf("\nConnected to wifi, IP-Address is: ");
  Serial.println(WiFi.localIP());

  // Route for root
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", String(), false);
  });

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();

  Serial.print("Web-Server started please open http://");
  Serial.print(WiFi.localIP());
  Serial.println(" in your browser.");
}

void loop() {
  // read must be called cyclically
  if (radar.read()) {
    wsSendCyclicData();

    if (sendRadarSettings) {
      sendRadarSettings = false;
      wsSendRadarSettings();
    }
  }

  ws.cleanupClients();
}
