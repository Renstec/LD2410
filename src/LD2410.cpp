#include "LD2410.h"

LD2410::LD2410(Stream &radarUart) {
  _radarUart = &radarUart;
}

LD2410::~LD2410() {
}

bool LD2410::begin() {
  return readFirmwareVersion() && readParameter();
}

bool LD2410::read() {
  return (parse() == 1);
}

bool LD2410::sendCommand(RadarCommand cmd, const uint8_t *data, size_t dataSize) {
  if (enableConfigMode()) {
    if (sendRequestToRadar(cmd, data, dataSize)) {
      // radar restarted so we don´t need to disable config mode
      if (cmd == RESTART) {
        return true;
      }

      return (disableConfigMode());
    }

    // Disable config mode even if the command has failed
    disableConfigMode();
  }

  return false;
}

bool LD2410::sendRequestToRadar(RadarCommand cmd, const uint8_t *data, size_t dataSize) {
  // send command Header
  _radarUart->write(commandHeader, sizeof(commandHeader));

  // send frame data length
  _radarUart->write(sizeof(cmd) + dataSize);
  _radarUart->write(uint8_t(0x00));

  // send command
  _radarUart->write(highByte(cmd));
  _radarUart->write(lowByte(cmd));

  // send frame data
  _radarUart->write(data, dataSize);

  // send command tail (mfr)
  _radarUart->write(commandTail, sizeof(commandTail));

  // wait send is completed
  _radarUart->flush();

  unsigned long timeout = millis();
  while (millis() - timeout < 100) {
    uint16_t res = parse();
    // command was successfully executed
    if (res == cmd) {
      return true;
      // command has failed
    } else if (res == (cmd + 1)) {
      return false;
    }
  }

  return false;
}

bool LD2410::sendCommand(RadarCommand cmd) {
  return sendCommand(cmd, NULL, 0);
}

uint16_t LD2410::charToUint(char c1, char c2) {
  return (uint16_t)(c1 | c2 << 8);
}

uint16_t LD2410::parse() {
  static bool dataPayload;
  static ParserState parserState;
  static uint8_t dataLength, receivedBytes;
  static uint8_t dataBuffer[40];

  while (_radarUart->available()) {
    uint8_t readChar = _radarUart->read();

    switch (parserState) {
      case FIND_FRAME_HEADER:

        // move data until frame header is found
        memmove(&dataBuffer[0], &dataBuffer[1], sizeof(dataHeader) - 1);
        dataBuffer[3] = readChar;

        // Check for data header
        if (!memcmp(dataBuffer, dataHeader, sizeof(dataHeader))) {
          dataPayload   = true;
          parserState   = RECEIVE_DATA_LENGTH;
          receivedBytes = 0;
        }

        // Check for command header
        if (!memcmp(dataBuffer, commandHeader, sizeof(commandHeader))) {
          dataPayload   = false;
          parserState   = RECEIVE_DATA_LENGTH;
          receivedBytes = 0;
        }
        break;

      case RECEIVE_DATA_LENGTH:

        dataBuffer[receivedBytes++] = readChar;

        if (receivedBytes >= 2) {
          dataLength = charToUint(dataBuffer[0], dataBuffer[1]);

          // buffer overflow check
          if (dataLength > sizeof(dataBuffer)) {
            parserState = FIND_FRAME_HEADER;
            return 0;
          }

          parserState   = RECEIVE_DATA;
          receivedBytes = 0;
        }
        break;

      case RECEIVE_DATA:
        dataBuffer[receivedBytes++] = readChar;

        if (receivedBytes == dataLength + sizeof(dataTail)) {
          if (dataPayload) {
            // Tail not found
            if (memcmp(&dataBuffer[dataLength], dataTail, sizeof(dataTail))) {
              parserState = FIND_FRAME_HEADER;
              return 0;
            }

            // Engineering mode active
            radar.cyclicData.radarInEngineeringMode = dataBuffer[0] == 0x01;

            // cyclicData Header 0XAA
            if (dataBuffer[1] != 0xAA) {
              parserState = FIND_FRAME_HEADER;
              return 0;
            }

            // Target State
            radar.cyclicData.targetState = (TargetState)dataBuffer[2];

            // moving target distance
            radar.cyclicData.movingTargetDistance = charToUint(dataBuffer[3], dataBuffer[4]);

            // moving target energy value
            radar.cyclicData.movingTargetEnergy = dataBuffer[5];

            // stationary target distance
            radar.cyclicData.stationaryTargetDistance = charToUint(dataBuffer[6], dataBuffer[7]);

            // stationary target energy value
            radar.cyclicData.stationaryTargetEnergy = dataBuffer[8];

            // detection distance
            radar.cyclicData.detectionDistance = charToUint(dataBuffer[9], dataBuffer[10]);

            if (radar.cyclicData.radarInEngineeringMode) {
              // Maximum distance gate
              radar.engineeringData.maxMovingGate     = dataBuffer[11];
              radar.engineeringData.maxStationaryGate = dataBuffer[12];

              // Moving energy per gate
              for (uint8_t gate = 0; gate <= 8; gate++) {
                radar.engineeringData.movingEnergyGateN[gate] = dataBuffer[13 + gate];
              }

              // Stationary energy per gate
              for (uint8_t gate = 0; gate <= 8; gate++) {
                radar.engineeringData.stationaryEnergyGateN[gate] = dataBuffer[22 + gate];
              }

              // max energy per gate
              radar.engineeringData.maxMovingEnergy     = dataBuffer[31];
              radar.engineeringData.maxStationaryEnergy = dataBuffer[32];

              // 0x55 cyclicData tail and check (0x00)
              if (dataBuffer[33] == 0x55 && dataBuffer[34] == 0x00) {
                parserState = FIND_FRAME_HEADER;
                return 1;
              }

            } else {
              memset(&radar.engineeringData, 0, sizeof(radar.engineeringData));

              // 0x55 cyclicData tail and check (0x00)
              if (dataBuffer[11] == 0x55 && dataBuffer[12] == 0x00) {
                parserState = FIND_FRAME_HEADER;
                return 1;
              }
            }
            parserState = FIND_FRAME_HEADER;
            return 0;

          } else {  // Command data

            // Tail not found
            if (memcmp(&dataBuffer[dataLength], commandTail, sizeof(commandTail))) {
              parserState = FIND_FRAME_HEADER;
              return false;
            }

            // Acknowledge for command data
            uint16_t cmd = charToUint(dataBuffer[1], dataBuffer[0]) - 1;

            bool fail = charToUint(dataBuffer[2], dataBuffer[3]) != 0;

            switch (cmd) {
              case READ_PARAMETER:
                // parameter header
                if (dataBuffer[4] != 0xAA) {
                  return 0;
                }

                radar.parameter.maxGate           = dataBuffer[5];
                radar.parameter.maxMovingGate     = dataBuffer[6];
                radar.parameter.maxStationaryGate = dataBuffer[7];

                for (uint8_t gate = 0; gate <= 8; gate++) {
                  radar.parameter.movingSensitivity[gate] = dataBuffer[8 + gate];
                }

                for (uint8_t gate = 0; gate <= 8; gate++) {
                  radar.parameter.stationarySensitivity[gate] = dataBuffer[17 + gate];
                }

                radar.parameter.detectionTime = charToUint(dataBuffer[26], dataBuffer[27]);
                break;
              case READ_FIRMWARE_VERSION:

                radar.firmwareVersion.minorVersion  = dataBuffer[6];
                radar.firmwareVersion.majorVersion  = dataBuffer[7];
                radar.firmwareVersion.bugFixVersion = uint32_t(
                    dataBuffer[8] | dataBuffer[9] << 8 |
                    dataBuffer[10] << 16 | dataBuffer[11] << 24);

                break;

              default:
                // TODO Protocol version, radar buffer size...
                break;
            }

            parserState = FIND_FRAME_HEADER;
            return cmd + fail;
          }
        }

        break;
    }
  }
  return 0;  // no data
}

bool LD2410::enableConfigMode() {
  uint8_t data[2] = {0x01, 0x00};
  sendRequestToRadar(ENABLE_CONFIG_MODE, data, sizeof(data));
  return true; // TODO
}

bool LD2410::disableConfigMode() {
  sendRequestToRadar(DISABLE_CONFIG_MODE, NULL, 0);  
  return true; // TODO
}

bool LD2410::setMaxDistAndDur(uint8_t maxMovingRange, uint8_t maxStationaryRange, uint16_t duration) {
  uint8_t data[18]{

      // maxMovingRange Command word 0
      0x00,  // low  byte command word 0
      0x00,  // high byte command word 0

      maxMovingRange,
      0x00,
      0x00,  // fill byte
      0x00,  // fill byte

      // maxStationaryRange command word 1
      0x01,  // low  byte command word 1
      0x00,  // high byte command word 1

      maxStationaryRange,
      0x00,
      0x00,  // fill byte
      0x00,  // fill byte

      // duration Command word 2
      0x02,  // low  byte command word 2
      0x00,  // high byte command word 2

      lowByte(duration), highByte(duration),
      0x00,  // fill byte
      0x00   // fill byte
  };

  return sendCommand(SET_MAX_DIST_AND_DUR, data, sizeof(data));
}

bool LD2410::readParameter() {
  return sendCommand(READ_PARAMETER);
}

bool LD2410::enableEngMode(bool enable) {
  if (enable) {
    return sendCommand(ENABLE_ENGINEERING_MODE);
  }
  return sendCommand(DISABLE_ENGINEERING_MODE);
}

bool LD2410::setGateSensConf(uint8_t gate, uint8_t movingSensitivity, uint8_t stationarySensitivity) {
  uint8_t data[18] = {
      // gate Command word 0
      0x00,  // low  byte command word 0
      0x00,  // high byte command word 0

      gate,  // gate
      0x00,  // fill byte
      0x00,  // fill byte
      0x00,  // fill byte

      // moving sensitivity command word 1
      0x01,  // low  byte command word 1
      0x00,  // high byte command word 1

      movingSensitivity,  // moving sensitivity
      0x00,               // fill byte
      0x00,               // fill byte
      0x00,               // fill byte

      // stationary sensitivity command word 2
      0x02,  // low  byte command word 2
      0x00,  // high byte command word 2

      stationarySensitivity,  // stationary sensitivity
      0x00,                   // fill byte
      0x00,                   // fill byte
      0x00                    // fill byte
  };

  return sendCommand(SET_GATE_SENS_CONFIG, data, sizeof(data));
}

bool LD2410::setBaudRate(BaudRateIndex baudRate) {
  uint8_t data[2] = {
      baudRate,
      0x00};

  return sendCommand(SET_BAUDRATE, data, sizeof(data));
}

bool LD2410::factoryReset() {
  return sendCommand(FACTORY_RESET);
}

bool LD2410::restart() {
  return sendCommand(RESTART);
}

bool LD2410::readFirmwareVersion() {
  return sendCommand(READ_FIRMWARE_VERSION);
}
