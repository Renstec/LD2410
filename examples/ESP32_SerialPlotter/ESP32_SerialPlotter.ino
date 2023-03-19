#if !defined(ARDUINO_ARCH_ESP32)
#error "This example requires an ESP-32 architecture"
#endif

#include <LD2410.h>

/* Simple Serial Plotter Example.

Commands for setting the radar parameters over the serial interface.
+---------+------------+------------+--------------------------------------+--------------------------------------------------------------+
| command | arg 0      | arg 1      | arg 2                                | description                                                  |
+---------+------------+------------+--------------------------------------+--------------------------------------------------------------+
| mov     | true/false | -          | -                                    | enables or disables the plotting of the moving energy values |
| set     | mov        | gate (0-8) | moving energy threshold (0-100%)     | sets the moving energy value for the specified gate          |
| sta     | true/false | -          | -                                    | enables or disables the plotting of the moving energy values |
| set     | sta        | gate (0-8) | stationary energy threshold (0-100%) | sets the stationary energy value for the specified gate      |
| sep     | true/false | -          | -                                    | enables or disables the plotting of the separator line       |
| restart | -          | -          | -                                    | restarts the radar                                           |
| reset   |            | -          | -                                    | resets the radar to factory defaults                         |
+---------+------------+------------+--------------------------------------+--------------------------------------------------------------+ 
*/

// Radar is connected to Serial1
LD2410 radar(Serial1);

const byte RADAR_RX_PIN = 26;
const byte RADAR_TX_PIN = 27;

// flags to enable the plotting of the specified values/parameters
bool plotMovingEnergy     = true;
bool plotStationaryEnergy = true;
bool plotSeparator        = true;

bool readParameters = true;

void setup() {
  Serial.begin(250000);
  Serial.setTimeout(10);

  // Start hardware serial on rx pin 26 and tx pin 27
  Serial1.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
}


void plotValues() {
  static int toggleValue;

  // toggle value which is used to make separation line more visible
  toggleValue < 0 ? toggleValue = 1 : toggleValue = -1;

  for (uint8_t gate = 0; gate <= 8; gate++) {
    // print symbol values for each gate
    float distanceFromGate = gate * 0.75;
    uint16_t offset        = gate * 103;

    Serial.printf(
        "sep%d:%u,movEnergy(%1.2fcm):%d,setMovEnergy(%1.2fcm):%u,staEnergy(%1.2fcm):%d, setStatEnergy(%1.2fcm):%d,",
        // sep%d separator line on top of each gate
        gate,
        plotSeparator ? offset + 102 + toggleValue : 0,

        // movEnergy(%1.2fcm):%d  Symbol, actual moving energy value from 0 to 100%
        distanceFromGate,
        plotMovingEnergy ? radar.engineeringData.movingEnergyGateN[gate] + offset : 0,

        // setMovEnergy(%1.2fcm):%d Symbol, set moving energy value from 0 to 100%
        distanceFromGate,
        plotMovingEnergy ? radar.parameter.movingSensitivity[gate] + offset : 0,

        // staEnergy(%1.2fcm):%d Symbol, actual stationary energy value from 0 to 100%
        distanceFromGate,
        plotStationaryEnergy ? radar.engineeringData.stationaryEnergyGateN[gate] + offset : 0,

        // setStatEnergy(%1.2fcm):%d Symbol, set stationary energy value from 0 to 100%
        distanceFromGate,
        plotStationaryEnergy ? radar.parameter.stationarySensitivity[gate] + offset : 0);
  }
  Serial.println();
}

void readSerialCommand() {
  String command = Serial.readStringUntil('\n');

  // check if command is completed by carriage return
  if (command.charAt(command.length() - 1) == '\r') {
    command.remove(command.length() - 1);
  } else {
    return;
  }

  if (command.length() == 0) {
    // command is empty
    return;
  }

  const byte MAX_ARGS = 4;
  int argsCount = 0;
  String args[MAX_ARGS];

  int startIndex = 0;
  int endIndex   = command.indexOf(' ');

  while (endIndex != -1 && argsCount < MAX_ARGS) {
    args[argsCount++] = command.substring(startIndex, endIndex);
    startIndex        = endIndex + 1;
    endIndex          = command.indexOf(' ', startIndex);
  }

  if (argsCount < MAX_ARGS) {
    // get the last arguments
    args[argsCount++] = command.substring(startIndex);
  } else {
    // to many arguments
    return;
  }

  if (args[0].equalsIgnoreCase("set")) {
    // set moving energy for gate (set mov gate energy)
    if (args[1].equalsIgnoreCase("mov")) {
      int gate   = args[2].toInt();
      int energy = args[3].toInt();

      if ((gate >= 0 && gate <= 8) && (energy >= 0 && energy <= 100)) {
        radar.setGateSensConf(gate, energy, radar.parameter.stationarySensitivity[gate]);
        plotMovingEnergy = true;
      }

      // set stationary energy for gate (set sta gate energy)
    } else if (args[1].equalsIgnoreCase("sta")) {
      int gate   = args[2].toInt();
      int energy = args[3].toInt();

      if ((gate >= 0 && gate <= 8) && (energy >= 0 && energy <= 100)) {
        radar.setGateSensConf(gate, radar.parameter.stationarySensitivity[gate], energy);
        plotStationaryEnergy = true;
      }
    }

    // enable/disable plotting moving energy values
  } else if (args[0].equalsIgnoreCase("mov")) {
    if (argsCount >= 2) {
      plotMovingEnergy = (args[1].toInt() == 1 || args[1].equalsIgnoreCase("true"));
    } else {
      plotMovingEnergy = !plotMovingEnergy;
    }

    // enable/disable plotting stationary energy values
  } else if (args[0].equalsIgnoreCase("sta")) {
    if (argsCount >= 2) {
      plotStationaryEnergy = (args[1].toInt() == 1 || args[1].equalsIgnoreCase("true"));
    } else {
      plotStationaryEnergy = !plotStationaryEnergy;
    }

    // enable/disable plotting separation line
  } else if (args[0].equalsIgnoreCase("sep")) {
    if (argsCount >= 2) {
      plotSeparator = (args[1].toInt() == 1 || args[1].equalsIgnoreCase("true"));
    } else {
      plotSeparator = !plotSeparator;
    }

    // restart radar (cmd: restart)
  } else if (args[0].equalsIgnoreCase("restart")) {
    radar.restart();
    // reset radar (cmd: reset)
  } else if (args[0].equalsIgnoreCase("reset")) {
    radar.factoryReset();
  }

  // get the latest parameters from the radar
  readParameters = true;
}

void loop() {
  readSerialCommand();

  if (radar.read()) {
    // radar should be always in engineering mode to plot the values
    if (!radar.cyclicData.radarInEngineeringMode) {
      radar.enableEngMode(true);
    }
    // plot the values to the serial plotter
    plotValues();
  }

  // on parameter change we want get the latest parameter values from the radar
  if (readParameters && radar.readParameter()) {
    readParameters = false;
  }
}
