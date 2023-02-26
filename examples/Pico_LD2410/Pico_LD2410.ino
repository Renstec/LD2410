/*
Raspberry Pi Pico and LD2410 Radar Sensor Example
Tauno Erik
26.02.2023

Pins:
Pico GPIO0 (TX) -> Radar RX
Pico GPIO1 (RX) -> Radar TX
Pico VBUS (5V)  -> Radar VCC
Pico GND        -> Radar GND

*/
#include <Arduino.h>
#include <LD2410.h>  // https://github.com/Renstec/LD2410/


LD2410 radar(Serial1);

// Enable Radar Enginering Mode
bool is_radar_eng_bode = false; // true or false

//const uint8_t RADAR_RX_PIN = 1;  // GPIO1 RX Pico defaults
//const uint8_t RADAR_TX_PIN = 0;  // GPIO0 TX

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("LD2410-Radar Example");

  Serial1.begin(256000);  // UART1 to radar

  delay(2000);

  if (radar.begin()) {
    Serial.print("Radar Firmware Version ");
    Serial.print(radar.firmwareVersion.majorVersion);
    Serial.print(".");
    Serial.print(radar.firmwareVersion.minorVersion);
    Serial.print(".");
    Serial.println(radar.firmwareVersion.bugFixVersion);

    Serial.print("Radar detection time: ");
    Serial.println(radar.parameter.detectionTime);

    Serial.print("Radar max detection Gate: ");
    Serial.println(radar.parameter.maxGate);

    Serial.print("Radar max moving Gate: ");
    Serial.println(radar.parameter.maxMovingGate);

    Serial.print("Radar max stationary Gate: ");
    Serial.println(radar.parameter.maxStationaryGate);

    Serial.println("Treshold/energy values per Gate:");

    for (uint8_t gate = 0; gate <= 8; gate++) {
      Serial.print("Gate ");
      Serial.print(gate);
      Serial.print(": moving treshold:");
      Serial.print(radar.parameter.movingSensitivity[gate]);
      Serial.print(", stationary treshold:");
      Serial.println(radar.parameter.stationarySensitivity[gate]);
    }

  } else {
    Serial.println("Failed to get firmware version and parameters from radar.");
  }

  Serial.println();

  radar.enableEngMode(is_radar_eng_bode);  // If true enables
}

void loop() {
  delay(1000);
  // read must be called cyclically
  if (radar.read()) {
    // Cyclic radar data
    Serial.println();

    Serial.print("Target state: ");
    Serial.print(radar.cyclicData.targetState);

    switch (radar.cyclicData.targetState) {
      case 0x00:
        Serial.println(" no target detected");
        break;
      case 0x01:
        Serial.println(" moving target detected");
        break;
      case 0x02:
        Serial.println(" stationary target detected");
        break;
      case 0x03:
        Serial.println(" moving and stationary target detected");
        break;
    }

    Serial.print("Moving taget distance in cm: ");
    Serial.println(radar.cyclicData.movingTargetDistance);

    Serial.print("Moving target energy valu 0-100%: ");
    Serial.println(radar.cyclicData.movingTargetEnergy);

    Serial.print("Statsionary target distance in cm: ");
    Serial.println(radar.cyclicData.stationaryTargetDistance);

    Serial.print("Statsionary target energy valu 0-100%: ");
    Serial.println(radar.cyclicData.stationaryTargetEnergy);

    Serial.print("Detection distance in cm: ");
    Serial.println(radar.cyclicData.detectionDistance);

    // Engineering Mode data
    if (radar.cyclicData.radarInEngineeringMode) {
      Serial.println("--Radar is in Engineering Mode--");

      Serial.print("Max moving distance: ");
      Serial.println(radar.engineeringData.maxMovingGate);

      Serial.print("Max statsionary distance: ");
      Serial.println(radar.engineeringData.maxStationaryGate);

      Serial.print("Max moving energy: ");
      Serial.println(radar.engineeringData.maxMovingEnergy);

      Serial.print("Max statsionary energy: ");
      Serial.println(radar.engineeringData.maxStationaryEnergy);

      Serial.println("Energy per Gate:");
      Serial.print("Moving: ");
      for (uint8_t gate = 0; gate <= 8; gate++) {
        Serial.print(radar.engineeringData.movingEnergyGateN[gate]);
        Serial.print(" ");
      }
      Serial.println();
      Serial.print("Statsionary: ");
      for (uint8_t gate = 0; gate <= 8; gate++) {
        Serial.print(radar.engineeringData.stationaryEnergyGateN[gate]);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
}