# LD2410
Arduino Library to read out and parameterize a sensor from type HiLink-LD2410.

## Electrical connection of the sensor
The module is connected via its serial interface which has a logic level of 3V3 DC. The power supply is from 5V to 12V DC.

| MCU |LD2410|
| --- | ---  |
| 5V  | VCC  |
| GND | GND  |
| TX  | RX   |
| RX  | TX   |

## Methods
All public methods return a boolean value, if the method is executed successfully.
The following methods are publicly available.

```
LD2410(Stream &radarUart);            // Constructor Stream must be set up outside the lib
bool begin();                         // Reads the firmware version and the parameters from the radar.	
read();                               // Check if received data from the radar 
bool enableEngMode(bool enable);      // Enables or disables the engineering mode.
bool factoryReset();                  // Factory reset the radar
bool readFirmwareVersion();           // Reads the radars firmware version.
bool readParameter();                 // This command reads the current configuration parameters of the radar.
bool restart();                       // Restarts the radar.
bool setBaudRate(BaudRateIndex eIdx); // Set the Baud Rate of the radar.

// This command will set the sensitivity/thresholds for the moving target and stationary target detection.
bool setGateSensConf(uint8_t gate,uint8_t movingSensitivity,uint8_t stationarySensitivity); 	

// Configure the radars maximums detection range for moving and stationary targets and the detection timeout.
bool setMaxDistAndDur(uint8_t maxMovingRange,uint8_t maxStationaryRange,uint16_t duration);
```

## Data and structures
The senor data is provided in structures.
The following structures are available.

### LD2410.cyclicData
The cyclic data from the sensor are stored in the structure cyclicData, which are fetched from the sensor with the function read().

```
bool radarInEngineeringMode;        // radar is in Engineering Mode
TargetState targetState;            // target state
uint16_t movingTargetDistance;      // moving target distance in cm
uint8_t movingTargetEnergy;         // moving target energy value 0-100 %
uint16_t stationaryTargetDistance;  // stationary target distance in cm
uint8_t stationaryTargetEnergy;     // stationary target energy value 0-100 %
uint8_t detectionDistance;          // detection distance in cm
```

### LD2410.engineeringData
The engineering data of the sensor is stored in the structure engineeringData
after the engineering mode was activated with the function enableEngMode().

```
uint8_t maxMovingGate;             // maximum moving distance
uint8_t maxStationaryGate;         // maximum stationary distance
uint8_t maxMovingEnergy;           // maximum moving energy
uint8_t maxStationaryEnergy;       // maximum stationary energy
uint8_t movingEnergyGateN[9];      // moving energy per gate
uint8_t stationaryEnergyGateN[9];  // stationary energy per gate
```

### LD2410.firmwareVersion
In the structure firmwareVersion the firmware version of the sensor is stored after the function begin or readFirmwareVersion() was successfully.

```
uint8_t majorVersion;    // major version of the radar firmware
uint8_t minorVersion;    // minor version of the radar firmware
uint32_t bugFixVersion;  // bug fix version of the radar firmware
```

### LD2410.parameter
In the structure parameter the read sensor parameters are stored after the call begin() or readParameter() was successfully.

```
uint8_t maxGate;                   // maximum distance detection gate
uint8_t maxMovingGate;             // maximum gate which detects moving targets
uint8_t maxStationaryGate;         // maximum gate which detects static targets
uint8_t movingSensitivity[9];      // Energy settings per gate
uint8_t stationarySensitivity[9];  // Energy settings per gate
uint16_t detectionTime;            // Detection time in seconds
```

### How to configure the sensor
A simple web interface is provided as example for the sensor configuration.

* Moving Target: Shows where and with how much energy the moving target is detected.
* Stationary Target: Shows where and with how much energy the stationary target is detected.
* Target Distance: Displays the current detected distance to the target.
* Moving Energy: Shows the moving energy per gate.
* Set Moving Energy: Shows the set thresholds for the moving energy per gate.
* Stationary Energy: Shows the stationary energy per gate.
* Set Stationary Energy: Shows the set thresholds for the stationary energy per gate.

![Showcase Gif](https://raw.githubusercontent.com/Renstec/LD2410/main/pics/WebIfAnimation.gif)

Thanks for the awesome arduino library's  [ArduinoJson](https://github.com/bblanchon/ArduinoJson), [AsyncTCP](https://github.com/me-no-dev/AsyncTCP) and [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer) and also for the great [ChartJs](https://github.com/chartjs) [Plugin](https://github.com/chrispahm/chartjs-plugin-dragdata) for dragging data.  