// if N2K enabled
#include "BatteryMonitorHardware.h"
#include "Globals.h"
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
#include <N2KDeviceList.h>
#include "BatteryMonitorN2K.h"


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


// Forward declarations
void SendN2kBattery( void );
void SendN2kBatteryConfig( void );
void setupN2K( void );

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = { 127508L, // Battery Status
                                                   127513L, // Battery Configuration Status
                                                   0
                                                 };

// Define schedulers for messages. Define schedulers here disabled. Schedulers will be enabled
// on OnN2kOpen so they will be synchronized with system.
// We use own scheduler for each message so that each can have different offset and period.
// Setup periods according PGN definition (see comments on IsDefaultSingleFrameMessage and
// IsDefaultFastPacketMessage) and message first start offsets. Use a bit different offset for
// each message so they will not be sent at same time.
tN2kSyncScheduler BatteryStatusScheduler(false,500,2000);
tN2kSyncScheduler BatteryConfigScheduler(false,1000,500);
tN2kDeviceList *locN2KDeviceList;
uint8_t n2kConnected = 0;

// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()
void OnN2kOpen() {
  // Start schedulers now.
  BatteryConfigScheduler.UpdateNextTime();
  BatteryStatusScheduler.UpdateNextTime();
// OnN2kOpen
} // end OnN2KOpen

// setup for N2k
void setupN2K() {
  
  // Set Product information
  // Tweaked to display properly on Raymarine...
  NMEA2000.SetProductInformation("BM V.9", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "SN00000001",  // Manufacturer's Model ID
                                 "1.0.0.01 (2024-01-15)",  // Manufacturer's Software version code
                                 "BM V.9" // Manufacturer's Model version
                                 );
  // Set device information
  NMEA2000.SetDeviceInformation(chipId, // Unique number. Use e.g. Serial number.
                                132, // Device function=Analog -> NMEA2000. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25, // Device class=Sensor Communication Interface. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040 // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode/*N2km_NodeOnly*/,47);
  locN2KDeviceList = new tN2kDeviceList(&NMEA2000);
  NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  // Here we tell library, which PGNs we transmit
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.
  NMEA2000.SetOnOpen(OnN2kOpen);

  // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
  NMEA2000 . SetN2kCANMsgBufSize ( 32 );
  NMEA2000 . SetN2kCANReceiveFrameBufSize ( 500 );
  NMEA2000 . SetN2kCANSendFrameBufSize ( 500 );

  NMEA2000.Open();
} // end setupN2K

// *****************************************************************************
void SendN2kBattery() {
  tN2kMsg N2kMsg;
  if ( BatteryStatusScheduler.IsTime() ) {
    BatteryStatusScheduler.UpdateNextTime();
    BattAmps = ShuntVolts / ShuntResistence;
    SetN2kDCBatStatus(N2kMsg, n2kInstance, (float)BattVolts, (float)BattAmps,          
                     0/*BatteryTemperature=N2kDoubleNA*/, 0xff /* SID ??*/);
    NMEA2000.SendMsg(N2kMsg);
  } // endif

} // end SendN2kBattery
#define AhToCoulombs 
// *****************************************************************************
void SendN2kBatteryConfig() {
  tN2kMsg N2kMsg;
  if ( BatteryConfigScheduler.IsTime() ) {
    BatteryConfigScheduler.UpdateNextTime();
    SetN2kBatConf(N2kMsg,  n2kInstance, N2kDCbt_Flooded /* flooded */, N2kDCES_No /* no equal */,
                     N2kDCbnv_12v, N2kDCbc_LeadAcid, AhToCoulomb(200) /* capacity in AH */, 10 /* temp coeff ??*/,
				1.2 /* peukert exp */, 80 /*charge eficiency */);    
    NMEA2000.SendMsg(N2kMsg);
  } // endif

} // end SendN2kBattery


// does all N2K send/recieve processing. Send message cadence is controlled with timers.
// This helper is called on a regular timer, Send messages are senbt when thier 
// specifiec timer is done.
void doN2Kprocessing(){
  SendN2kBattery();
  SendN2kBatteryConfig();
  NMEA2000.ParseMessages();
  // record the number of devices on n2k bus
  n2kConnected = locN2KDeviceList->Count();
} // end doN2kprocessing
