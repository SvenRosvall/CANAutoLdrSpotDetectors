// CANAutoLdrSpotDetectors


/*
  Copyright (C) 2021 Sven Rosvall
  Including copyrights from CBUS_1in1out and Arduino CBUS Libraries


  This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

/*
      3rd party libraries needed for compilation:

      Streaming   -- C++ stream style output, v5, (http://arduiniana.org/libraries/streaming/)
      ACAN2515    -- library to support the MCP2515/25625 CAN controller IC
      CBUSSwitch  -- library access required by CBUS and CBUS Config
      CBUSLED     -- library access required by CBUS and CBUS Config
*/
///////////////////////////////////////////////////////////////////////////////////
// Pin Use map UNO:
// Digital pin 2          Interupt CAN
// Digital pin 3 (PWM)    Not Used
// Digital pin 4          Not Used
// Digital pin 5 (PWM)    Not Used
// Digital pin 6 (PWM)    Not Used
// Digital pin 7          Not Used
// Digital pin 8          Not Used
// Digital pin 9 (PWM)    Not Used
// Digital pin 10 (SS)    CS    CAN
// Digital pin 11 (MOSI)  SI    CAN
// Digital pin 12 (MISO)  SO    CAN
// Digital pin 13 (SCK)   Sck   CAN

// Digital / Analog pin 0     LDR input
// Digital / Analog pin 1     LDR input
// Digital / Analog pin 2     LDR input
// Digital / Analog pin 3     LDR input
// Digital / Analog pin 4     LDR input
// Digital / Analog pin 5     LDR input
//////////////////////////////////////////////////////////////////////////

// Choose what set of output is wanted.
//#define PLOT_ALL_VALUES
//#define PLOT_DETAILS
//#define DEBUG       // set for serial debug

// Tuning parameters
const int INTERVAL = 100; // ms
const float P = 0.050f;  // for moving average
const int THRESHOLD = 150;

// 3rd party libraries
#include <Streaming.h>
#include <AutoLdrSpotDetectors.h>

// CBUS library header files
#include <CBUS2515.h>            // CAN controller and CBUS class
#include <CBUSconfig.h>          // module configuration
#include <cbusdefs.h>            // MERG CBUS constants
#include <CBUSParams.h>

////////////DEFINE MODULE/////////////////////////////////////////////////

// module name
unsigned char mname[7] = { 'A', 'L', 'D', 'R' };

// constants
const byte VER_MAJ = 1;         // code major version
const char VER_MIN = ' ';       // code minor version
const byte VER_BETA = 0;        // code beta sub-version
const byte MODULE_ID = 147;      // CBUS module type

const unsigned long CAN_OSC_FREQ = 8000000;     // Oscillator frequency on the CAN2515 board

#ifdef DEBUG
#define DEBUG_PRINT(S) Serial << S << endl
#else
#define DEBUG_PRINT(S)
#endif
#if !defined(PLOT_ALL_VALUES) && !defined(PLOT_DETAILS)
#define PRINT_INFO
#endif

class CbusEventEmitter : public SensorChangeAction
{
public:
  void onChange(int ldrIndex, bool covered) override;
};

bool sendEvent(byte opCode, unsigned int eventNo);
void CbusEventEmitter::onChange(int ldrIndex, bool covered)
{
  byte opCode = covered ? OPC_ACON : OPC_ACOF;
  sendEvent(opCode, (unsigned int) ldrIndex + 1);
}

// module objects
CbusEventEmitter cbusEventEmitter;
// Using 5 LDR sensors.
AutoLdrSpotDetectors detectors(cbusEventEmitter, {A0, A1, A2, A3, A4});

//////////////////////////////////////////////////////////////////////////

//CBUS pins
const byte CAN_INT_PIN = 2;  // Only pin 2 and 3 support interrupts
const byte CAN_CS_PIN = 10;
//const byte CAN_SI_PIN = 11;  // Cannot be changed
//const byte CAN_SO_PIN = 12;  // Cannot be changed
//const byte CAN_SCK_PIN = 13;  // Cannot be changed

// CBUS objects
CBUS2515 CBUS;                      // CBUS object
CBUSConfig config;                  // configuration object

//
///  setup CBUS - runs once at power on called from setup()
//
void setupCBUS()
{
  // set config layout parameters
  config.EE_NVS_START = 10;
  config.EE_NUM_NVS = 0;
  config.EE_EVENTS_START = 50;
  config.EE_MAX_EVENTS = 64;
  config.EE_NUM_EVS = 0;
  config.EE_BYTES_PER_EVENT = (config.EE_NUM_EVS + 4);

  // initialise and load configuration
  config.setEEPROMtype(EEPROM_INTERNAL);
  config.begin();

#ifdef PRINT_INFO
  Serial << F("> mode = ") << ((config.FLiM) ? "FLiM" : "SLiM") << F(", CANID = ") << config.CANID;
  Serial << F(", NN = ") << config.nodeNum << endl;

  // show code version and copyright notice
  printConfig();
#endif

  // set module parameters
  CBUSParams params(config);
  params.setVersion(VER_MAJ, VER_MIN, VER_BETA);
  params.setModuleId(MODULE_ID);
  params.setFlags(PF_FLiM | PF_COMBI);

  // assign to CBUS
  CBUS.setParams(params.getParams());
  CBUS.setName(mname);

  // register our CBUS event handler, to receive event messages of learned events
  CBUS.setEventHandler(eventhandler);

  // configure and start CAN bus and CBUS message processing
  CBUS.setNumBuffers(2);         // more buffers = more memory used, fewer = less
  CBUS.setOscFreq(CAN_OSC_FREQ);   // select the crystal frequency of the CAN module
  CBUS.setPins(CAN_CS_PIN, CAN_INT_PIN);           // select pins for CAN bus CE and interrupt connections
  CBUS.begin();
}

//
///  setup Module - runs once at power on called from setup()
//

void setupModule()
{
  detectors.setMovingAverageP(P);
  detectors.setThresholdLevel(THRESHOLD);
  detectors.setup();
#ifdef PLOT_DETAILS
  detectors.plotTitleDetailed(2);
#endif
#ifdef PLOT_ALL_VALUES
  detectors.plotTitleAll();
#endif

}

void setup()
{
  Serial.begin (115200);
#ifdef PRINT_INFO
  Serial << endl << endl << F("> ** CANALDR v1 ** ") << __FILE__ << endl;
#endif

  setupCBUS();
  setupModule();

  // end of setup
#ifdef PRINT_INFO
  Serial << F("> ready") << endl << endl;
#endif
}


void loop()
{
  detectors.update();
#ifdef PLOT_DETAILS
  detectors.plotDetailed(2);
#endif
#ifdef PLOT_ALL_VALUES
  detectors.plotAll();
#endif

  // do CBUS message, switch and LED processing
  CBUS.process();

  // process console commands
  processSerialInput();
}

// Send an event routine according to Module Switch
bool sendEvent(byte opCode, unsigned int eventNo)
{
  CANFrame msg;
  msg.id = config.CANID;
  msg.len = 5;
  msg.data[0] = opCode;
  msg.data[1] = highByte(config.nodeNum);
  msg.data[2] = lowByte(config.nodeNum);
  msg.data[3] = highByte(eventNo);
  msg.data[4] = lowByte(eventNo);

#ifdef PRINT_INFO
  if (CBUS.sendMessage(&msg)) {
    Serial << F("> sent CBUS message with Event Number ") << eventNo << endl;
  } else {
    Serial << F("> error sending CBUS message") << endl;
  }
#endif
}

//
/// called from the CBUS library when a learned event is received
/// The only event that is handled here is StartOfDay events.
//
void eventhandler(byte index, CANFrame *msg)
{
  byte opc = msg->data[0];
  byte ev;
  byte evval;

#ifdef DEBUG
  Serial << F("> event handler: index = ") << index << F(", opcode = 0x") << _HEX(opc) << endl;
#endif

  switch (opc) {

    case OPC_ACON:
    case OPC_ASON:
        //evval = config.getEventEVval(index, ev);
        // TODO: Send current status events for each LDR.

      break;
  }
}


void printConfig(void)
{
  // code version
  Serial << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << endl;
  Serial << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;

  // copyright
  Serial << F("> © Martin Da Costa (MERG M6223) 2020") << endl;
}

//
/// command interpreter for serial console input
//

void processSerialInput(void)
{
  byte uev = 0;
  char msgstr[32];

  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {

      case 'n':
        // node config
        printConfig();

        // node identity
        Serial << F("> CBUS node configuration") << endl;
        Serial << F("> mode = ") << (config.FLiM ? "FLiM" : "SLiM") << F(", CANID = ") << config.CANID << F(", node number = ") << config.nodeNum << endl;
        Serial << endl;
        break;

      case 'e':
        // EEPROM learned event data table
        Serial << F("> stored events ") << endl;
        Serial << F("  max events = ") << config.EE_MAX_EVENTS << F(" EVs per event = ") << config.EE_NUM_EVS << F(" bytes per event = ") << config.EE_BYTES_PER_EVENT << endl;

        for (byte j = 0; j < config.EE_MAX_EVENTS; j++) {
          if (config.getEvTableEntry(j) != 0) {
            ++uev;
          }
        }

        Serial << F("  stored events = ") << uev << F(", free = ") << (config.EE_MAX_EVENTS - uev) << endl;
        Serial << F("  using ") << (uev * config.EE_BYTES_PER_EVENT) << F(" of ") << (config.EE_MAX_EVENTS * config.EE_BYTES_PER_EVENT) << F(" bytes") << endl << endl;

        Serial << F("  Ev#  |  NNhi |  NNlo |  ENhi |  ENlo | ");

        for (byte j = 0; j < (config.EE_NUM_EVS); j++) {
          sprintf(msgstr, "EV%03d | ", j + 1);
          Serial << msgstr;
        }

        Serial << F("Hash |") << endl;

        Serial << F(" --------------------------------------------------------------") << endl;

        // for each event data line
        for (byte j = 0; j < config.EE_MAX_EVENTS; j++) {
          if (config.getEvTableEntry(j) != 0) {
            sprintf(msgstr, "  %03d  | ", j);
            Serial << msgstr;

            // for each data byte of this event
            for (byte e = 0; e < (config.EE_NUM_EVS + 4); e++) {
              sprintf(msgstr, " 0x%02hx | ", config.readEEPROM(config.EE_EVENTS_START + (j * config.EE_BYTES_PER_EVENT) + e));
              Serial << msgstr;
            }

            sprintf(msgstr, "%4d |", config.getEvTableEntry(j));
            Serial << msgstr << endl;
          }
        }

        Serial << endl;

        break;

      // NVs
      case 'v':
        // note NVs number from 1, not 0
        Serial << "> Node variables" << endl;
        Serial << F("   NV   Val") << endl;
        Serial << F("  --------------------") << endl;

        for (byte j = 1; j <= config.EE_NUM_NVS; j++) {
          byte v = config.readNV(j);
          sprintf(msgstr, " - %02d : %3hd | 0x%02hx", j, v, v);
          Serial << msgstr << endl;
        }

        Serial << endl << endl;

        break;

      // CAN bus status
      case 'c':
        CBUS.printStatus();
        break;

      case 'h':
        // event hash table
        config.printEvHashTable(false);
        break;

      case 'y':
        // reset CAN bus and CBUS message processing
        CBUS.reset();
        break;

      case '*':
        // reboot
        config.reboot();
        break;

      case 'm':
        // free memory
        Serial << F("> free SRAM = ") << config.freeSRAM() << F(" bytes") << endl;
        break;

      case 'r':
        // renegotiate
        CBUS.renegotiate();
        break;

      case 'z':
        // Reset module, clear EEPROM
        static bool ResetRq = false;
        static unsigned long ResWaitTime;
        if (!ResetRq) {
          // start timeout timer
          Serial << F(">Reset & EEPROM wipe requested. Press 'z' again within 2 secs to confirm") << endl;
          ResWaitTime = millis();
          ResetRq = true;
        }
        else {
          // This is a confirmed request
          // 2 sec timeout
          if (ResetRq && ((millis() - ResWaitTime) > 2000)) {
            Serial << F(">timeout expired, reset not performed") << endl;
            ResetRq = false;
          }
          else {
            //Request confirmed within timeout
            Serial << F(">RESETTING AND WIPING EEPROM") << endl;
            config.resetModule();
            ResetRq = false;
          }
        }
        break;

      default:
        // Serial << F("> unknown command ") << c << endl;
        break;
    }
  }
}
