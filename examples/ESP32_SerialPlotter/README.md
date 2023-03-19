# Example to configure the radar sensor via the serial plotter of the Arduino IDE.

Please make sure that you have enabled new line feed and carriage return on the serial plotter when sending an command.

| command | arg 0      | arg 1      | arg 2                                | description                                                   |
|---------|------------|------------|--------------------------------------|--------------------------------------------------------------|
| mov     | true/false | -          | -                                    | enables or disables the plotting of the moving energy values |
| set     | mov        | gate (0-8) | moving energy threshold (0-100%)     | sets the moving energy value for the specified gate          |
| sta     | true/false | -          | -                                    | enables or disables the plotting of the moving energy values |
| set     | sta        | gate (0-8) | stationary energy threshold (0-100%) | sets the stationary energy value for the specified gate      |
| sep     | true/false | -          | -                                    | enables or disables the plotting of the separator line       |
| restart | -          | -          | -                                    | restarts the radar                                           |
| reset   |            | -          | -                                    | resets the radar to factory defaults                         |



![](https://raw.githubusercontent.com/Renstec/LD2410/main/pics/SerialPlotterExample.png)