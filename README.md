<img align="right" src="arduino_cbus_logo.png"  width="150" height="75">

# CANAutoLdrSpotDetectors

This project implements a CBUS module that contains the self adjusting LDR spot
detectors as provided by the library AutoLdrSpotDetectors.

## Using CANAutoLdrSpotDetectors

Connect LDR's to the analogue pins that have internal pull-up. 
The other LDR leg to ground.
Define the pins to use for LDRs in the sketch.
Find and change the following piece of code:
```
// Define the pins that are connected to LDRs:
auto LED_PINS = {A0, A1, A2, A3, A4};
```

The MCP2515 interface requires five Arduino pins to be allocated.
Three of these are fixed in the architecture of the Arduino processor.
One pin must be connected to an interrupt
capable Arduino pin.

As the code stands, there are no CBUS switch or LEDs. 
Instead, use the "renegotiate" command with the serial monitor as described below.

### Library Dependencies

The following third party libraries are required:

Library | Purpose
---------------|-----------------
AutoLdrSpotDetectors | Provides the logic for detecting LDR state and automatic adjustments.
CBUS2515   | Provides CBUS communication over MCP2515 boards

### Application Configuration

The module can be configured to the users specific configuration in a section of code 
starting at line 118 with the title DEFINE MODULE. The following parameters can be changed 
as necessary:

```
// module name
unsigned char mname[7] = { 'A', 'L', 'D', 'R' };
```
This can be adjusted as required to reflect the module configuration.  For example, 'm' & 'n' 
could be change to any number between 0 & 9. However, only one character is allowed between 
each pair of ' ' and the total number of characters must not exceed seven.
This name will show up in FCU when the module identifies itself.

```
const byte MODULE_ID = 147;      // CBUS module type
```
The ID of the CBUS module.
This is used by FCU to distinguish CBUS module types and to know
how to treat them. 
As there is no support for CANAutoLdrSpotDetectors in FCU, this number
must not be a number that FCU recognizes.

```
const byte VER_MAJ = 1;         // code major version
const char VER_MIN = 'b';       // code minor version
const byte VER_BETA = 1;        // code beta sub-version
```
These constants define version of the module.
These will show up in FCU.

```
const unsigned long CAN_OSC_FREQ = 8000000;
```
If the oscillator frequency on your CAN2515 board is not 8MHz, change the number to match. The 
module will not work if this number does not match the oscillator frequency.

### CBUS Op Codes

The following Op codes are supported:

OP_CODE | HEX | Function
----------|---------|---------
 OPC_ACON | 0x90 | On event
 OPC_ACOF | 0x91 | Off event
 OPC_ASON | 0x98 | Short event on
 OPC_ASOF | 0x99 | Short event off

### Event Variables

There is only one event variable.

 EV Value | Function
--------|-----------
 1 | Send events that reflect the state of each LDR.
 
### Node Variables

There are 10 node variables.
Each of them can have a value from 0 to 254.
255 is interpreted as "not initialized" and a default value is used instead.

NV Index | Description | Value use | default value
--------|--------|--------|------
 1 | Interval between updates. | * 10ms | 5 (=50ms)
 2 | Threshold level for when to trigger a state change. | * 4 | 50 (=200)
 3 | P - Defines decline in moving average of current values. | / 100 (i.e. a percentage) | 2 (=2%)
 4 | Q - Defines decline in moving average of value differences. | / 100 (i.e. a percentage) | 20 (=20%)
 5 | How much to weigh the changing LDR vs all other LDRs. | / 100 (i.e. a percentage) | 80 (=80%)
 6 | Time to change state when LDR is getting covered. | * 10ms | 20 (=200ms)
 7 | Time to change state when LDR is getting opened. | * 10ms | 40 (=400ms)
 8 | Scaling of threshold levels depending on ambient light levels. 0% => Use threshold level as is; 100% => scales fully. |  / 100 (i.e. a percentage) | 80 (=80%)
 9 | How often to send out events as response to StartOfDay events.| * 1ms | 20 (=20ms)
 10 | Reserved for future uses
 
## Set Up and the Serial Monitor

Without a CBUS switch, it is necessary to have another means of registering the module on 
the CBUS and acquiring a node number.  This is accomplished by sending a single character to 
the Arduino using the serial send facility in the serial monitor.

#### 'r'
This character will cause the module to renegotiate its CBUS status by requesting a node number.
The FCU will respond as it would for any other unrecognised module.

#### 'z'
This character needs to be sent twice within 2 seconds so that its action is confirmed.
This will reset the module and clear the EEPROM.  It should thus be used with care.

Other information is available using the serial monitor using other commands:

#### 'n'
This character will return the node configuration.

#### 'e'
This character will return the learned event table in the EEPROM.

#### 'v'
This character will return the node variables.

#### 'c'
This character will return the CAN bus status.

#### 'h'
This character will return the event hash table.

#### 'y'
This character will reset the CAN bus and CBUS message processing.

#### '\*'
This character will reboot the module.

#### 'm'
This character will return the amount of free memory. 
 
## TODO
* Review how NodeVariables are updated. Is it OK to read NVs every iteration or is caching needed.
  Maybe use a callback from CBUS library when a NV is updated.