
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
#include "INA226.h"

// specific to this design
#include "EngineMonitorHardware.h"
#include "EngineMonitorN2K.h"

using namespace sensesp;

// unique chip id
int32_t chipId=0;

// used for n2k instance field in messages
int32_t n2kInstance;

/* need the following data pids
 * from 127508 - Battery Status
 * \param BatteryInstance     BatteryInstance.
 * \param BatteryVoltage      Battery voltage in V
 * \param BatteryCurrent      Current in A
 * \param BatteryTemperature  Battery temperature in K. Use function
  bool ParseN2kPGN127508(const tN2kMsg &N2kMsg, unsigned char &BatteryInstance, double &BatteryVoltage, double &BatteryCurrent,
                     double &BatteryTemperature, unsigned char &SID);


 * These are all settings that get set by the user
 * from 127513 - Battery Configuration Status
 * \param BatInstance          BatteryInstance
 * \param BatType              Type of battery. See \ref tN2kBatType
 * \param SupportsEqual        Supports equalization. See \ref tN2kBatEqSupport
 * \param BatNominalVoltage    Battery nominal voltage. See \ref tN2kBatNomVolt
 * \param BatChemistry         Battery Chemistry See \ref tN2kBatChem
 * \param BatCapacity          Battery capacity in Coulombs. Use AhToCoulombs,
 *                             if you have your value in Ah.
 * \param BatTemperatureCoefficient Battery temperature coefficient in %
 * \param PeukertExponent      Peukert Exponent
 * \param ChargeEfficiencyFactor      Charge efficiency factor
  bool ParseN2kPGN127513(const tN2kMsg &N2kMsg, unsigned char &BatInstance, tN2kBatType &BatType, tN2kBatEqSupport &SupportsEqual,
                     tN2kBatNomVolt &BatNominalVoltage, tN2kBatChem &BatChemistry, double &BatCapacity, int8_t &BatTemperatureCoefficient,
				double &PeukertExponent, int8_t &ChargeEfficiencyFactor);
*/

// globals used for N2K Messages
double HouseVolts = 0;
double BattVolts = 0;
double ShuntVolts = 0;
double ShuntResistence = .001;
double BattAmps = 0;

// instantiate INA226
INA226 INA(0x40);
// call backs for INA226
float read_Current_callback() { return (INA.getCurrent());}
float read_Voltage_callback() { return (INA.getBusVoltage());}
float read_ShuntVoltage_callback() { return (INA.getShuntVoltage());}
float read_Power_callback() { return (INA.getPower());}

// The setup function performs one-time application initialization.
void setup() {

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
  
  // setup the INA226 (or INA219)
  Wire.begin();
  if (!INA.begin() )
  {
    debugI("could not connect to INA device. Fix and Reboot");
  } else {
    // setup shunt
    INA.setMaxCurrentShunt(50, 0.001, true);
    INA.setAverage(1);
  } // end if

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

// for led, using gpio25 as ground (open drain output), gpio26 toggles led
// because no grounds added to header duh!
pinMode ( GPIO_NUM_25 , OUTPUT_OPEN_DRAIN );
pinMode ( GPIO_NUM_26 , OUTPUT );
digitalWrite(GPIO_NUM_25, false);
digitalWrite(GPIO_NUM_26, true);

// Constant values used for Battery Monitor
const char *n2kInstanceSKPath = "/n2k/Instance/sk_path";
auto n2kInst = new StringConstantSensor("0", 1, "/n2k/Instance");

ConfigItem(n2kInst)
      ->set_title("N2K Instance")
      ->requires_restart();

n2kInst 
  ->connect_to(new SKOutputString(
            "/n2k/Instance/sk_path",         // Signal K path
            n2kInstanceSKPath,        
                                                    // web UI and for storing the
                                                    // configuration
            new SKMetadata("Number",          // Define output units
                          "n2k Instance")))  // Value description
  
  ->connect_to(
            new LambdaConsumer<String>([](String Altn2kInstance) {
              n2kInstance = Altn2kInstance.toInt();
            }));


 // Create a new sensor for INA Voltage
  const char *kBatteryVoltageLinearConfigPath = "/propulsion/battery voltage/calibrate";
  const char *kBatteryVoltageSKPath = "/propulsion/battery voltage/sk_path";
  const float AltBmultiplier = 1; // resitor divider ratio, measured
  const float AltBoffset = 0;

  auto ina226_Voltage = new RepeatSensor<float>(
      500, read_Voltage_callback);

  auto ina226_Voltage_linear = new Linear(AltBmultiplier, AltBoffset, kBatteryVoltageLinearConfigPath);
  ConfigItem(ina226_Voltage_linear)
        ->set_title("battery Voltage Scale");

  ina226_Voltage
      ->connect_to(ina226_Voltage_linear)
      
      ->connect_to(new SKOutputFloat(
          "/propulsion/battery voltage/sk_path",         // Signal K path
          kBatteryVoltageSKPath,        
                                                  // web UI and for storing the
                                                  // configuration
          new SKMetadata("Volts",                     // Define output units
                        "Battery Voltage")))  // Value description

      // connect for n2k purposes
      ->connect_to(
        new LambdaConsumer<float>([](float BattVoltsValue) {
          BattVolts = BattVoltsValue;
        }));                     

      //#ifdef SERIALDEBUG
      // debug - add an observer that prints out the current value of the analog input
      // every time it changes.
      ina226_Voltage->attach([ina226_Voltage]() {
        debugD("Battery Voltage value: %f Raw Volts", ina226_Voltage->get());
      });
      //#endif

 // Create a new sensor for INA Voltage
  const char *kShuntVoltageLinearConfigPath = "/propulsion/shunt voltage/calibrate";
  const char *kShuntVoltageSKPath = "/propulsion/shunt voltage/sk_path";
  const float AltSVmultiplier = 1; // resitor divider ratio, measured
  const float AltSVoffset = 0;

  auto ina226_ShuntVoltage = new RepeatSensor<float>(
      500, read_ShuntVoltage_callback);

  auto ina226_ShuntVoltage_linear = new Linear(AltSVmultiplier, AltSVoffset, kShuntVoltageLinearConfigPath);
  ConfigItem(ina226_ShuntVoltage_linear)
        ->set_title("battery Shunt Voltage Scale");

  ina226_ShuntVoltage
      ->connect_to(ina226_ShuntVoltage_linear)
      
      ->connect_to(new SKOutputFloat(
          "/propulsion/shunt voltage/sk_path",         // Signal K path
          kShuntVoltageSKPath,        
                                                  // web UI and for storing the
                                                  // configuration
          new SKMetadata("Volts",                     // Define output units
                        "Shunt Voltage")))  // Value description

      // connect for n2k purposes
      ->connect_to(
        new LambdaConsumer<float>([](float ShuntVoltsValue) {
          ShuntVolts = ShuntVoltsValue;
        }));                     

      //#ifdef SERIALDEBUG
      // debug - add an observer that prints out the current value of the analog input
      // every time it changes.
      ina226_ShuntVoltage->attach([ina226_ShuntVoltage]() {
        debugD("Shunt Voltage value: %f Raw Volts", ina226_ShuntVoltage->get());
      });
      //#endif

// now start n2k
setupN2K();

// setup N2K processing to happen every 100 msec
SensESPBaseApp::get_event_loop()->onRepeat(100, []() { doN2Kprocessing(); });

} // end setup

void loop() { 					 
  // changed for V3
  static auto event_loop = SensESPBaseApp::get_event_loop();
  event_loop->tick();

} // end loop