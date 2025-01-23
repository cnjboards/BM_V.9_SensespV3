
// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.
#include <Arduino.h>
#include <ESP.h>
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/digital_output.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/sensors/constant_sensor.h"
#include "sensesp/signalk/signalk_listener.h"
#include "sensesp/signalk/signalk_value_listener.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include <sensesp/system/valueconsumer.h>
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/hysteresis.h"
#include "sensesp/transforms/enable.h"
#include "sensesp_app_builder.h"

// specific to this design
#include "EngineMonitorHardware.h"
#include "EngineMonitorN2K.h"

using namespace sensesp;

// forward declarations
void checkButton(void);

// unique chip id
int32_t chipId=0;

// used for N2K Messages
double AltRPM = 0;
double EngRPM = 0;
double OilPres = 0;
double AltVolts = 0;
double HouseVolts = 0;
double FuelLevel = 0;
double B1A3 = 0;
double B2A0 = 0;
double B2A1 = 0;
double B2A2 = 0;
double B2A3 = 0;
bool chkEng = 0;
bool digIn2 = 0;
double engineCoolantTemp = 0;
double engineBlockTemp = 0;
double engineRoomTemp = 0;
double engineExhaustTemp = 0;

// variables to handle the dial push button debounce etc
int buttonState;            // the current reading from the input pin
int buttonStateLong;            // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin
int shortButtonStateLatched = LOW;  // latch if button is pressed for short interval
int longButtonStateLatched = LOW;  // latch if pressed for long interval
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled 
unsigned long shortDebounceDelay = 80;    // the short debounce time
unsigned long longDebounceDelay = 1000;    // the long debounce time
unsigned long latchClearDelay = 500;    // the time to allow latched states to be consumed befor autonomous reset

// The setup function performs one-time application initialization.
void setup() {

  // seupt outputs at start
  pinMode(Out1, OUTPUT_OPEN_DRAIN); // output 1
  digitalWrite(Out1, true);

  #ifndef LITE_V3
  pinMode(Out2, OUTPUT_OPEN_DRAIN); // output 1
  digitalWrite(Out2, true);
  #endif
  // used to create a unique id
  uint8_t chipid [ 6 ];

  // setup the logging infra
  SetupLogging();

  // derive a unique chip id from the burned in MAC address
  esp_efuse_mac_get_default ( chipid );
  for ( int i = 0 ; i < 6 ; i++ )
    chipId += ( chipid [ i ] << ( 7 * i ));
  
  // dump chip id at startup
  debugI("Poweron reset - Chip ID: %x" , chipId);
  debugI("Flash Size: %d bytes" , ESP.getFlashChipSize());
  
  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("BM V.9")
                    // Optionally, hard-code the WiFi and Signal K server
                    // ->set_wifi("ssid","password")
                    // settings. This is normally not needed.
                    //->set_sk_server("ip address", port#)
                    ->enable_ota("1Qwerty!")
                    ->set_button_pin(0)
                    ->get_app();

// for debounce init
pinMode ( In2 , INPUT_PULLDOWN );
lastButtonState = digitalRead(In2);
buttonState = lastButtonState;
buttonStateLong = lastButtonState;

// Oil Pressure Sender Config //
// transducer has .5V - 4.5V for 0-100psi (689476 Pa).
// Analog input is 3.3V raw with a resister divider of 7.5K and 11K.
// analog input returns 0raw=0V and 4095raw=3.3V @ the pin.
// thru resister divider, .5V xducer ~= .2V at chip = .2*4095/3.3 ~= 248 (125 actual)
// 4.5V xducer ~= 1.82V at chip = 1.82*4095/3.3 ~= 2258
const char *kOilPressureADCConfigPath = "/propulsion/Engine Oil Pressure/ADC/scale";
const char *kOilPressureLinearConfigPath = "/propulsion/oil pressure/calibrate";
const char *kOilPressureSKPath = "/propulsion/oil pressure/sk_path";
const float OilPmultiplier = 228;
const float OilPoffset = -82080;

// Oil Pressure ESP32 Analog Input
auto analog_input = new AnalogInput(OIL_PRESSURE, 500, kOilPressureADCConfigPath, 4096);

auto analog_input_linear = new Linear(OilPmultiplier, OilPoffset, kOilPressureLinearConfigPath);
ConfigItem(analog_input_linear)
      ->set_title("Engine Oil Pressure Scale");

analog_input
      // scale using linear transform
      ->connect_to(analog_input_linear)
      // send to SK, value is in Pa
      ->connect_to(new SKOutputFloat("/propulsion/oil pressure", kOilPressureSKPath))
      // for N2K use, value is in Pa
      ->connect_to(
        new LambdaConsumer<float>([](float engOilPresValue) {
          OilPres = engOilPresValue;
          // if negative then make zero
          if (OilPres <= 0.0) OilPres = 0; // for some reason n2k does not like negative oil pressure??
        }));


// *********** digital inputs
// use pull down so a 1 is the input on - non inverted
// Use In1 for High Temp/Low Oil, when on (+12V) we are in alarm
const uint8_t DigInput1 = In1;
pinMode ( DigInput1 , INPUT_PULLDOWN );

// sampling interval for digital inputs
const unsigned int kDigitalInput1Interval = 250;
const char *kCheckEngineEnableSKPath = "/propulsion/check engine/enable";
const char *kCheckEngineSKPath = "/propulsion/check engine/sk_path";
// Digital input 1  - connect to high temp/low oil alarm
// when input low, no alarm
auto digitalInput1 = new RepeatSensor<bool>(
    kDigitalInput1Interval,
    [DigInput1]() { return !digitalRead(DigInput1); });
 
auto digitalInput1Enable = new Enable<bool>(true, kCheckEngineEnableSKPath);
ConfigItem(digitalInput1Enable)
      ->set_title("Check Engine Alarm Enable");

// INput 1 send out to SK
digitalInput1
  
  // bool to enable/disable the check engine
  ->connect_to(digitalInput1Enable)
  
  ->connect_to(new SKOutputBool(
      "/propulsion/check engine",          // Signal K path
      kCheckEngineSKPath,         // configuration path
      new SKMetadata("",                       // No units for boolean values
                    "Digital Input High Temp/Low Oil Pres")  // Value description
    ))
  
  ->connect_to(
    new LambdaConsumer<bool>([](bool checkEngineValue) {
      chkEng = checkEngineValue;
    }));


  // Digital input 2, sample periodically
  const unsigned int kDigitalInput2Interval = 250;
  const uint8_t DigInput2 = In2;
  pinMode ( DigInput2 , INPUT_PULLDOWN );
  const char *kDigitalInput2SKPath = "/sensors/digital in2/sk_path";
  auto digitalInput2 = new RepeatSensor<bool>(
      kDigitalInput2Interval,
      [DigInput2]() { return digitalRead(DigInput2); });

  // Input 2 send out to SK
  digitalInput2->connect_to(new SKOutputBool(
      "/sensors/digital in2",          // Signal K path
      kDigitalInput2SKPath,         // configuration path
      new SKMetadata("",                       // No units for boolean values
                     "Digital input 2 value")  // Value description
      ))

    ->connect_to(
      new LambdaConsumer<bool>([](bool digInput2Value) {
        digIn2 = digInput2Value;
      }));

// ******** end digital input

// now start n2k
setupN2K();

// No need to parse the messages at every single loop iteration; 1 ms will do
SensESPBaseApp::get_event_loop()->onRepeat(100, []() { doN2Kprocessing(); });

} // end setup

void loop() { 					 
  // changed for V3
  static auto event_loop = SensESPBaseApp::get_event_loop();
  event_loop->tick();

  // see is pb is pushed
  checkButton();
} // end loop

// this debounces the dial PB and will latch both a short and long press
void checkButton(void){
  // read the state of the switch into a local variable:
  int reading = (int)digIn2; /* digitalRead(BUTTON_PIN);
 */
  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  } // end if

  if ((millis() - lastDebounceTime) > shortDebounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // if we timeout on debounce then latch short button pressed
      if (buttonState == HIGH) {
        shortButtonStateLatched = HIGH;
        debugV("Button pressed, short debounce");
      } // end if
    } // end if
  } // end if

  if ((millis() - lastDebounceTime) > longDebounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonStateLong) {
      buttonStateLong = reading;

      // if we timeout on debounce then latch short button pressed
      if (buttonState == HIGH) {
        longButtonStateLatched = HIGH;
        debugV("Button pressed, long debounce");
      } // end if
    } // end if
  } // end if

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
} // end checkButton