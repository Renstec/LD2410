#pragma once

#include <Arduino.h>

/**
 * @brief Radar Target State
 */
enum TargetState {
  NO_TARGET                    = 0x00,  // no target detected
  MOVING_TARGET                = 0x01,  // moving target detected
  STATIONARY_TARGET            = 0x02,  // stationary target detected
  MOVING_AND_STATIONARY_TARGET = 0x03,  // moving and stationary target detected
};

/**
 * @brief Radars baud rate index
 */
enum BaudRateIndex : uint8_t {
  BAUD_9600   = 0X0001,
  BAUD_19200  = 0X0002,
  BAUD_38400  = 0X0003,
  BAUD_57600  = 0X0004,
  BAUD_115200 = 0X0005,
  BAUD_230400 = 0X0006,
  BAUD_256000 = 0X0007,  // radars default baud rate
  BAUD_460800 = 0X0008
};

class LD2410 {
 private:
  /**
   * @brief Stucture of Parameters from Radar
   */
  struct Parameter {
    uint8_t maxGate;                   // maximum distance detection gate
    uint8_t maxMovingGate;             // maximum gate which detects moving targets
    uint8_t maxStationaryGate;         // maximum gate which detects static targets
    uint8_t movingSensitivity[9];      // Energy settings per gate
    uint8_t stationarySensitivity[9];  // Energy settings per gate
    uint16_t detectionTime;            // Detection time in seconds
  };

  /**
   * @brief cyclic Data from the Radar
   */
  struct CyclicData {
    bool radarInEngineeringMode;        // radar is in Engineering Mode
    TargetState targetState;            // target state
    uint16_t movingTargetDistance;      // moving target distance in cm
    uint8_t movingTargetEnergy;         // moving target energy value 0-100 %
    uint16_t stationaryTargetDistance;  // stationary target distance in cm
    uint8_t stationaryTargetEnergy;     // stationary target energy value 0-100 %
    uint8_t detectionDistance;          // detection distance in cm
  };

  /**
   * @brief Engineering data from the Radar
   */
  struct EngineeringData {
    uint8_t maxMovingGate;             // maximum moving distance
    uint8_t maxStationaryGate;         // maximum stationary distance
    uint8_t maxMovingEnergy;           // maximum moving energy
    uint8_t maxStationaryEnergy;       // maximum stationary energy
    uint8_t movingEnergyGateN[9];      // moving energy per gate
    uint8_t stationaryEnergyGateN[9];  // stationary energy per gate
  };

  /**
   * @brief Radars firmware version
   */
  struct FirmwareVersion {
    uint8_t majorVersion;    // major version of the radar firmware
    uint8_t minorVersion;    // minor version of the radar firmware
    uint32_t bugFixVersion;  // bug fix version of the radar firmware
  };

  /**
   * @brief List of the radar commands
   */
  enum RadarCommand : uint16_t {
    ENABLE_CONFIG_MODE       = 0xFF00,  // enable configuration mode
    DISABLE_CONFIG_MODE      = 0xFE00,  // disable configuration mode
    SET_MAX_DIST_AND_DUR     = 0x6000,  // set the maximum Distance Gate and Unmanned Duration Parameter
    READ_PARAMETER           = 0x6100,  // read the parameters from the radar
    ENABLE_ENGINEERING_MODE  = 0x6200,  // enable engineering mode of the radar
    DISABLE_ENGINEERING_MODE = 0x6300,  // disable engineering mode
    SET_GATE_SENS_CONFIG     = 0x6400,  // set sensitivity 0-100% for moving and stationary gates
    READ_FIRMWARE_VERSION    = 0xA000,  // read  the firmware version
    SET_BAUDRATE             = 0xA100,  // set serial baud rate
    FACTORY_RESET            = 0xA200,  // Factory Reset
    RESTART                  = 0xA300,  // Restart the radar
  };

  /**
   * @brief Parser State
   */
  enum ParserState : uint8_t {
    FIND_FRAME_HEADER,
    RECEIVE_DATA_LENGTH,
    RECEIVE_DATA
  };

  /**
   * @brief Helper function to send a command to the radar with data
   *
   * @param cmd command to send
   * @param data data/payload to send
   * @param dataSize size of the data
   * @return true Command executed successfully
   * @return false Command executed with errors
   */

  bool _sendCommand(RadarCommand cmd, const uint8_t* data, size_t dataSize);

  /**
   * @brief Helper function to send and command to the radar without data
   *
   * @param cmd command to send
   * @return true Command executed successfully
   * @return false Command executed with errors
   */
  bool _sendCommand(RadarCommand cmd);

  /**
   * @brief Funtion to send the data to the radar
   * @param cmd request command to send
   * @param data data to send
   * @param dataSize size of the data
   * @return true Request was executed successfully
   * @return false Request executed with errors
   */
  bool _sendRequestToRadar(RadarCommand cmd, const uint8_t* data, size_t dataSize);

  /**
   * @brief Helper function to convert tow char to an uint16_t
   *
   * @param c1 Char 1
   * @param c2 Char 2
   * @return uint16_t converted value
   */
  uint16_t _charToUint(char c1, char c2);

  /**
   * @brief Receive and parse data from the radar
   *
   * @return uint16_t > 0 Received new data or the Acknowledge for a command
   */
  uint16_t _parse();

  /**
   * @brief Enables the configuration mode on the LD2410
   *
   * @return true Enabled configuration mode successfully
   * @return false Failed to enable configuration mode
   */
  bool _enableConfigMode();

  /**
   * @brief Disables the configuration mode on the LD2410
   *
   * @return true Disabled the configuration mode successfully
   * @return false Failed to disable the configuration mode
   */
  bool _disableConfigMode();

  // readed firmware version of the radar
  FirmwareVersion _firmwareVersion;

  // parameters from the radar
  Parameter _parameter;     

  // cyclic data of from the radar         
  CyclicData _cyclicData; 

  // engineering data from the radar          
  EngineeringData _engineeringData;  

  // Data Header
  const uint8_t _dataHeader[4] = {0XF4, 0xF3, 0XF2, 0xF1};

  // Data tail
  const uint8_t _dataTail[4] = {0xF8, 0xF7, 0xF6, 0xF5};

  // Command Header
  const uint8_t _commandHeader[4] = {0XFD, 0xFC, 0XFB, 0xFA};

  // Command tail
  const uint8_t _commandTail[4] = {0x04, 0x03, 0x02, 0x01};

  // radars uart port
  Stream* _radarUart;

 public:
  /**
   * @brief Constructor
   *
   * @param radarUart Uart Interface where the radar is connected to
   */
  LD2410(Stream& radarUart);

  /**
   * @brief Destroy the LD2410 object
   *
   */
  ~LD2410();

  /**
   * @brief Reads the firmware version and the parameters from the radars
   *
   * @return true Received the firmware version and the parameters
   * @return false failed to receive the firmware version or the parameters
   */
  bool begin();

  /**
   * @brief Check if received data from the radar (needs to be called in loop)
   *
   * @return true Received a new data frame from the radar
   * @return false no new data frame received from the radar
   */
  bool read();

  /**
   * @brief Configure the radars maximums detection range for moving and
   * stationary targets.
   *
   * @param maxMovingRange  maximum detection gate for moving targets(Gate 0-8)
   * @param maxStationaryRange maximum detection gate for stationary targets(Gate 2-8)
   * @param duration radar stationary detection timeout in seconds
   * @return true Command executed successfully
   * @return false Command executed with errors
   */
  bool setMaxDistAndDur(uint8_t maxMovingRange, uint8_t maxStationaryRange, uint16_t duration);

  /**
   * @brief This command reads the current configuration parameters of the radar.
   *
   * @return true Command executed successfully
   * @return false Command executed with errors
   */
  bool readParameter();

  /**
   * @brief Enable or disable the engineering mode
   *
   * @param enable If true the engineering mode gets enabled otherwise disabled
   * @return true Command executed successfully
   * @return false Command executed with errors
   */
  bool enableEngMode(bool enable);

  /**
   * @brief This command will set the sensitivity/thresholds for the moving
   * target and stationary target detection
   *
   * @param gate Distance Gate 0-8
   * @param movingSensitivity Moving sensitivity 0-100%
   * @param stationarySensitivity Stationary sensitivity 0-100%
   * @return true Command executed successfully
   * @return false Command executed with errors
   */
  bool setGateSensConf(uint8_t gate, uint8_t movingSensitivity, uint8_t stationarySensitivity);

  /**
   * @brief Set the Baud Rate of the radar
   *
   * @param BaudRateIndex
   * @return true Command executed successfully
   * @return false Command executed with errors
   */
  bool setBaudRate(BaudRateIndex eBaudRate);

  /**
   * @brief This command is used to restore all configuration values to
   * their original values, and the configuration values will take effect after
   * restarting the module.
   *
   * @return true Command executed successfully
   * @return false Command executed with errors
   */
  bool factoryReset();

  /**
   * @brief Restarts the radar
   *
   * @return true Command executed successfully
   * @return false Command executed with errors
   */
  bool restart();

  /**
   * @brief Reads the radars firmware version
   *
   * @return true Command executed successfully
   * @return false Command executed with errors
   */
  bool readFirmwareVersion();

  // Reference to the radars cyclic Data
  const CyclicData& cyclicData = _cyclicData;

  // Reference to the radars engineering Data
  const EngineeringData& engineeringData = _engineeringData;

  // Reference to the radars parameters
  const Parameter& parameter = _parameter;

  // Reference to the radars firmware version
  const FirmwareVersion& firmwareVersion = _firmwareVersion;
};
