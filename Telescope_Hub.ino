/*
  Arduino Ethernet Telescope Hub
        interfaces  Steve and Jamie Gould's Telescope to a PC ASCOM compliant software driver
		Declination = Altitude = north/south = up down
		Right Ascension = Azimuth  = east/west = left right
		Communications to the two motor controllers is made through this HUB.

		Version 1.0 30/01/2018
*/
/* Version Control ---------------------------------------------------------------------------------------------------
	Version	Date		Description								Debug Status	Release Date
	1.1		27/10/2018
	1.1.1	21/12/2018	Focuser resolution removed from protocol
	1.1.2	12/01/2019	Recoded Focuser command handling
*/
/* Functionality Checklist -------------------------------------------------------------------------------------------
	Hub display turns off and on
	running led flashes
	Focuser display turns off and on 
*/
// Inclusions ------------------------------------------------------------------------------
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Ethernet2.h>
#include "HUB_Commands.h"

#define DEBUG_OUTPUT_SETUP
#define DEBUG_OUTPUT_MOTOR_DRIVERS
#define DEBUG_OUTPUT_FOCUSER
#define DEBUG_OUTPUT_CONTROLLER
#define DEBUG_OUTPUT_COMMANDS
#define DEBUG_OUTPUT_FIELDS
#define DEBUG_OUTPUR_COMMANDS
#define WDT									// enable WatchDog Timer
#define ETHERNET

#ifdef WDT
	#include <avr/wdt.h>
#endif
// Definitions ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define display_DIN				11		// MAX7219 DATA pin		D11 green
#define display_CS				10		// MAX7219 CS/LOAD pin	D10 blue
#define display_CLK				12		// MAX7219 CLK pin		D12 yellow
#define MOTOR_DRIVER_LED_pin	49		// 49 = Green
#define FOCUSER_LED_pin			48		// 48 = Blue
#define CONTROLLER_LED_pin		46		// 46 = Red

#define CONTROLLER		1
#define AZIMUTH			2
#define ALTITUDE		3
#define FOCUSER			4

#define altitude_serial Serial1
#define altitude_RXpin  19
#define altitude_TXpin  18
#define altitude_baud   38400
#define azimuth_serial  Serial2
#define azimuth_RXpin   17
#define azimuth_TXpin   16
#define azimuth_baud    38400
#define focuser_serial	Serial3
#define focuser_RXpin	15
#define focuser_TXpin	14
#define focuser_baud	38400
#define STX             0x02
#define ETX             0x03

#define LED_FLASH_TIME		20      // number of milli seconds to light the LEDs
// Constants ------------------------------------------------------------------------------------------
byte mac[] = { 0x90, 0xA2, 0xDA, 0x11, 0x23, 0x40 };				// MAC address of Ethernet controller, found on a sticker on the back of the Ethernet shield.
//byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };				// MAC address of Jamie's Ethernet Controller
//IPAddress ip(192, 168, 0, 30);									// IP Address (FIXED) of this server
//IPAddress gateway(192, 168, 0, 100);
//IPAddress subnet(255, 255, 255, 0);

// Instantiations -----------------------------------------------------------------------------------------------------------------------------------------
#ifdef ETHERNET
EthernetServer server(80);                                      // Create a server listening on the given port.
#endif

//-- Working Variables --------------------------------------------------------------------------------------------------------------------
int state = HIGH;      // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin
long time = 0;         // the last time the output pin was toggled
long debounce = 300;   // the debounce time, increase if the output flickers
// ASCOM Variables ========================================================================================================

// Internal Variables (pre ASCOM) -----------------------------------------------------------------------------------------

bool hub_display_state = true;				// default to on

union sg_double_union {
  char sg_chars[4];
  int sg_int[2];
  double sg_double;
};
union sg_int_union {
  char sg_chars[2];
  int sg_int;
};
typedef struct {
  unsigned char header;             // [0] STX
  unsigned char source;				// [1] source of message
  unsigned char command_number;     // [1] Command Number
  double parameter_one;				// [3 - 6]
  double parameter_two;				// [7 - 10]
  unsigned char footer;             // [11] ETX
} Packet_Structure;
#define Message_Structure_Length 12
union Packet {
  Packet_Structure contents;
  unsigned char sg_chars[Message_Structure_Length];
};

union Packet Incoming_Message_from_Azimuth;
union Packet Incoming_Message_from_Controller;
union Packet Incoming_Message_from_Altitude;
union Packet Incoming_Message_from_Focuser;
union Packet Outgoing_Message_to_Motor_Drivers;
union Packet Outgoing_Message_to_Focuser;
union Packet Outgoing_Message_to_Controller;

bool Altitude_Incoming_Message_Available = false;
bool Azimuth_Incoming_Message_Available = false;
bool Focuser_Incoming_Message_Available = false;

char altitude_in_buffer_counter = 0;
char azimuth_in_buffer_counter = 0;
byte altitude_inptr;					// must be 8 bit byte so that it overflows at 256
byte altitude_outptr;					// must be 8 bit byte so that it overflows at 256
unsigned char altitude_inbuffer[256];
int altitude_string_ptr;
byte azimuth_inptr;						// must be 8 bit byte so that it overflows at 256
byte azimuth_outptr;					// must be 8 bit byte so that it overflows at 256
unsigned char azimuth_inbuffer[256];
int azimuth_string_ptr;
char focuser_in_buffer_counter = 0;
byte focuser_inptr;
byte focuser_outptr;
unsigned char focuser_inbuffer[256];
int focuser_string_ptr;
long xloop = 0;
bool command_sent = false;
long gps_time_LED_On = 0;
bool gps_status = false;
long running_time_LED_On = (long)0;
bool running_status = false;
long hub_time_LED_On = 0;
bool hub_status = false;
unsigned long last_status_update = 0;
char altitude_status = 0;
char azimuth_status = 0;
char focuser_status = 0;
enum {
	display_REG_DECODE = 0x09,
	display_REG_INTENSITY = 0x0A,
	display_REG_SCANLIMIT = 0x0B,
	display_REG_SHUTDOWN = 0x0C,
	display_REG_DISPTEST = 0x0F,
};
enum { OFF = 0, ON = 1 };
bool display_flag = false;
const byte DP = 0b10000000;
const byte t = 0b00001111;
const byte h = 0b00010111;
const byte P = 0b01100111;
const byte dash = 0b00000001;
const byte E = 0b01001111;
const byte R = 0b01110111;
const byte O = 0b01111110;
const byte space = 0b00000000;
bool focuser_display_on = true;
char display_string[9];
// Prototypes -----------------------------------------------------------------------------------------------------------------------------
void Parse_Command_to_Fieldf(String);	// convert String field to its comma delimited individual double fields
void Parse_Altitude_Incoming_Message(void);	// convert the received altitude message to its individual fields
void Parse_Azimuth_Incoming_Message(void);	// convert the received azimuth message to its individual fields
void Parse_Focuser_Incoming_Message(void);  // convert the received focuser message to its undividual fields
void Send_Message_to_Driver(int,  int, double, double );	// send message to the specified (int) motor driver
void Send_Message_to_Focuser(unsigned int,unsigned int,unsigned int, unsigned int);
//-- Setup --------------------------------------------------------------------------------------------------------------------------------
void setup() {
#ifdef WDT
  wdt_disable();													// disable watchdog timer during setup
#endif
  Serial.begin(115200);												// Start monitor serial communication with the given baud rate.
  while (!Serial) {
    ;
  }
  altitude_serial.begin(altitude_baud, SERIAL_8N1);					// initialise the altitude serial port
  azimuth_serial.begin(azimuth_baud, SERIAL_8N1);					// initialise the azimuth serial port
  focuser_serial.begin(focuser_baud, SERIAL_8N1);					// initialise the focuser serial port
  altitude_serial.flush();											// clear the altitude serial buffer
  azimuth_serial.flush();											// clear the azimuth serial buffer
  focuser_serial.flush();											// clear the focuser serial buffer
#ifdef DEBUG_OUTPUT
  Serial.print(millis(), DEC);
  Serial.print("\t");
  Serial.println("Serial Ports setup");
  Serial.print(millis(), DEC);
  Serial.print("\t");
  Serial.println("Setting Up Ethernet Server");
#endif
#ifdef ETHERNET
  if (Ethernet.begin(mac) == 0) {									//  Ethernet.begin(mac, ip, gateway, subnet);                       // Initialize the Ethernet shield
#ifdef DEBUG_OUTPUT
    Serial.print(millis(), DEC);
    Serial.print("\t");
    Serial.println("Failed to configure Ethernet using DHCP");	// no point in carrying on, so do nothing forevermore, try to configure using IP address instead of DHCP:
#endif
    // Error(Ethernet_Error);
  }

  server.begin();
#ifdef DEBUG_OUTPUT
  Serial.print(millis(), DEC);
  Serial.print("\t");
  Serial.print("Ethernet Server Setup Complete, at ");
  Serial.println(Ethernet.localIP());
#endif
#endif // end ETHERNET
#ifdef DEBUG_OUTPUT
  Serial.print(millis(), DEC);
  Serial.print("\t");
  Serial.println("ASCOM setup complete");
#endif
#ifdef WDT
  Serial.print(millis(), DEC);
  Serial.print("\t");
  Serial.println("Setting Up Watchdog");
  wdt_enable(WDTO_2S);											// enable a 2 second watchdog timeout
  Serial.print(millis(), DEC);
  Serial.print("\t");
  Serial.println("Watchdog Setup Complete");
#endif
#ifdef DEBUG_OUTPUT
  Serial.print(millis(), DEC);
  Serial.print("\t");
  Serial.println("Setup Complete");
#endif
  digitalWrite(CONTROLLER_LED_pin, LOW);
  pinMode(CONTROLLER_LED_pin, OUTPUT);
  digitalWrite(MOTOR_DRIVER_LED_pin, LOW);
  pinMode(MOTOR_DRIVER_LED_pin, OUTPUT);
  digitalWrite(FOCUSER_LED_pin, LOW);
  pinMode(FOCUSER_LED_pin, OUTPUT);
  pinMode(display_DIN, OUTPUT);
  pinMode(display_CS, OUTPUT);
  pinMode(display_CLK, OUTPUT);
  resetDisplay();							// reset the MAX7219 display
  send_update_to_SEVEN_SEGMENT();
} // end setup
// Main -----------------------------------------------------------------------------------------------------------------------------------
void loop() {
#ifdef ETHERNET
	int ethernet_status = (int) Ethernet.maintain();
#endif
#ifdef WDT
	wdt_reset();													// reset the watchdog timer
#endif
	if (Check_Controller_Packet() == true) Parse_Incoming_Controller_Packet();
	if (Check_Altitude_Packet() == true) Parse_Incoming_Motor_Driver_Packet((char) 1);
	if (Check_Azimuth_Packet() == true) Parse_Incoming_Motor_Driver_Packet((char) 2);
	if (Check_Focuser_Packet() == true) Parse_Incoming_Focuser_Packet;
		
	//-- TEST ---------------------------------------------------------------------------------------------------------------------------------
	if (millis() == 20000)  {   // run the test after 20 seconds
#ifdef DEBUG_COMMAND
    if (command_sent == false) {     // make sure commands are only sent once
//		Serial.print(millis(), DEC);
//		Serial.println("\tSending Target Altitude");  // c2 last tested 30/04/2018 
//		TargetAltitude_f("100.0");
//		Serial.print(millis(), DEC);
//		Serial.println("\tSending Target Azimuth");   // c3
//		TargetAzimuth_f("0.0");
//		Serial.print(millis(), DEC);
//		Serial.println("\tSending Slew to Target");           // c10 Last tested 30/04/2018
//		SlewToTarget_f("");
//      Serial.println("Sending Telescope_Is_Tracking ");        // c4 last tested 30/04/2018
//      Telescope_Is_Tracking_f("1.0,0.0,0,0.0");
//      Serial.println("Setting Telescope_Is_Tracking Rates");   // c5 last tested 30/04/2018
//      Telescope_Is_TrackingRates_f("2650.0,3000.0,0.0,0.0");
//      Serial.println("Sending AbortSlew");        // c6 last tested 30/04/2018
//      AbortSlew_f("");
//      Serial.println("Sending FindHome");         // c7 last tested
//      FindHome_f("");
//      Serial.println("Parking");                  // c8 Last tested 30/04/2018
//      Park_f("");
//      Serial.println("Sending SlewToAltAz");      // c9 Last tested 30/04/2018
//      SlewToAltAz_f("10.0,100.0,0.0,0.0");

//		Serial.println("Sync to AltAz");            // c11 Last tested 30/04/2018
//      SyncToAltAz_f("0.0,90.0,0.0,0.0");
//      Serial.println("GuideRateRightAscension");  // c13
//      GuideRateRightAscension_f("800.0,0.0,0.0,0.0");
//      Serial.println("GuideRateDeclination");     // c14  Last tested 30/04/2018
//      GuideRateDeclination_f("800.0,0.0,0.0,0.0");
//      Serial.println(" Sending PulseGuide");      // c12
//      PulseGuide_f("0.0,30000.0,0.0,0.0");
//      Serial.println(" Unpark");                  // c15 Last tested 30/04/2018
//      UnPark_f("");
//		Serial.print(" Focuser Halt");
//		Focuser_Halt_f();							// f16
//		Serial.print(" Focuser Calibrate");
//		Focuser_Calibrate_f()						// f17
//		Serial.print(" Focuser Move");
//		Focuser_Move_f(5);							// f18
		command_sent = true;
		xloop = 0;
    }
#endif
  }
	xloop += 1;
		//--End of Debug ---------------------------------------------------------------------------------------------------------------------------
} // end of main loop

// Controller Functions ---------------------------------------------------------------------------------------------------
int Hub_Display_On() {
#ifdef DEBUG_OUTPUT_COMMANDS
	Serial.print(millis(), DEC);
	Serial.println("\tHub Display Turned On");
#endif
	hub_display_state = 1;
}
int Hub_Display_Off() {
#ifdef DEBUG_OUTPUT_COMMANDS
	Serial.print(millis(), DEC);
	Serial.println("\tHub Display Turned Off");
#endif
	hub_display_state = 0;
}
// End of Controller Functions -------------------------------------------------------------------------------------------
bool Check_Altitude_Packet(void) {
  while (altitude_outptr != altitude_inptr) {                                        // check altitude serial buffer for data
    char thisbyte = altitude_inbuffer[altitude_outptr++];                           // take a character from the input buffer and increment pointer
    if ((thisbyte == (char)STX) && (altitude_string_ptr == 0)) {                    // look for the STX, but only if the output string is empty
      Incoming_Message_from_Altitude.sg_chars[altitude_string_ptr++] = STX;                // store the STX and increment the string pointer
      Altitude_Incoming_Message_Available = false;
    } else {
      if (thisbyte == (char)ETX) {												// character was not an STX check for ETX
        Incoming_Message_from_Altitude.sg_chars[altitude_string_ptr++] = ETX;            // save the ETX and increment the string pointer
        if (altitude_string_ptr == Message_Structure_Length) {                  // does it mean end of packet (we just saved the ETX at 20!
          altitude_string_ptr = 0;                                            // zero the string pointer
          Altitude_Incoming_Message_Available = true;
        }
      } else {
        Incoming_Message_from_Altitude.sg_chars[altitude_string_ptr++] = thisbyte;       // Not a valid STX or a valid ETX so save it and increment string pointer
      }
    }
	return Altitude_Incoming_Message_Available;
  } // end of while altitude
}
bool Check_Azimuth_Packet(void) {
  while (azimuth_outptr != azimuth_inptr) {                                        // check altitude serial buffer for data
    char thisbyte = azimuth_inbuffer[azimuth_outptr++];                           // take a character from the input buffer and increment pointer
    if ((thisbyte == (char)STX) && (azimuth_string_ptr == 0)) {                    // look for the STX, but only if the output string is empty
      Incoming_Message_from_Azimuth.sg_chars[azimuth_string_ptr++] = STX;                // store the STX and increment the string pointer
    } else {
      if (thisbyte == (char)ETX) {												// character was not an STX check for ETX
        Incoming_Message_from_Azimuth.sg_chars[azimuth_string_ptr++] = ETX;            // save the ETX and increment the string pointer
        if (azimuth_string_ptr == Message_Structure_Length) {                                        // does it mean end of packet (we just saved the ETX at 20!
          azimuth_string_ptr = 0;                                            // zero the string pointer
		  Azimuth_Incoming_Message_Available = true;
        }
      } else {
        Incoming_Message_from_Azimuth.sg_chars[azimuth_string_ptr++] = thisbyte;           // Not a valid STX or a valid ETX so save it and increment string pointer
      }
    }
  } // end of while azimuth
  return Azimuth_Incoming_Message_Available;
}
bool Check_Focuser_Packet(void) {
	while (focuser_outptr != focuser_inptr) {											// check focuser serial buffer for data
		char thisbyte = focuser_inbuffer[focuser_outptr++];								// take a character from the input buffer and increment pointer
		if ((thisbyte == (char)STX) && (focuser_string_ptr == 0)) {						// look for the STX, but only if the output string is empty
			Incoming_Message_from_Focuser.sg_chars[focuser_string_ptr++] = STX;				// store the STX and increment the string pointer
			Focuser_Incoming_Message_Available = false;
		}
		else {
			if (thisbyte == (char)ETX) {												// character was not an STX check for ETX
				Incoming_Message_from_Focuser.sg_chars[focuser_string_ptr++] = ETX;     // save the ETX and increment the string pointer
				if (focuser_string_ptr == Message_Structure_Length) {  // does it mean end of packet (we just saved the ETX at 20!
					focuser_string_ptr = 0;												// zero the string pointer
					Focuser_Incoming_Message_Available = true;
				}
			}
			else {
				Incoming_Message_from_Focuser.sg_chars[focuser_string_ptr++] = thisbyte; // Not a valid STX or a valid ETX so save it and increment string pointer
			}
		}
		return Focuser_Incoming_Message_Available;
	} // end of while focuser
}
bool Check_Controller_Packet(void) {
#ifdef ETHERNET
	EthernetClient client = server.available();                  // Listen for incoming client requests.
	if (client) {;
		for (char i = 1; i < Message_Structure_Length-1; i++) {
			Incoming_Message_from_Controller.sg_chars[i] = client.read();
		}
		return true;
	}
#endif
	return false;
}
// Interrupt Service Routines ------------------------------------------------------------------------------------------------------------------------
void serialEvent1() {
	while (altitude_serial.available()) {
		altitude_inbuffer[altitude_inptr++] = (unsigned char) altitude_serial.read();        // add the received character to the buffer and increment character count
	}
}
void serialEvent2() {
	while (azimuth_serial.available()) {
		azimuth_inbuffer[azimuth_inptr++] = (unsigned char) azimuth_serial.read();        // add the received character to the buffer and increment character count
	}
}
void serialEvent3() {
	while (focuser_serial.available()) {
		focuser_inbuffer[focuser_inptr++] = (unsigned char)focuser_serial.read();        // add the received character to the buffer and increment character count
	}
}
void Parse_Incoming_Motor_Driver_Packet(char motor) {// process an update message from the Altitude controller
	int command_number = 0;
	double parameter_one = 0;
	double parameter_two = 0;
	Flash_Motor_Driver_Light();
	Altitude_Incoming_Message_Available = false;
#ifdef DEBUG_OUTPUT_MOTOR_DRIVER
	Serial.print(millis(), DEC);
	Serial.println("\tPacket Received from Altitude Motor");
#endif
	if (motor == 1) {
		command_number = (int) Incoming_Message_from_Azimuth.contents.command_number;
		parameter_one = Incoming_Message_from_Azimuth.contents.parameter_one;
		parameter_two =  Incoming_Message_from_Azimuth.contents.parameter_two;
	}
	else {
		command_number = Incoming_Message_from_Altitude.contents.command_number;
		parameter_one = Incoming_Message_from_Altitude.contents.parameter_one;
		parameter_two = Incoming_Message_from_Altitude.contents.parameter_two;
	}
	switch (command_number) {
		case  Telescope_Azimuth:
		case  Telescope_Declination:
		case  Telescope_CanUnpark:
		case  Telescope_CanSyncAltAz:
		case  Telescope_CanSync:
		case  Telescope_CanSlewAsync:
		case  Telescope_CanSlewAltAzAsync:
		case  Telescope_CanSlewAltAz:
		case  Telescope_CanSlew:
		case  Telescope_CanSetTracking:
		case  Telescope_CanSetRightAscensionRate:
		case  Telescope_CanSetPierSide:
		case  Telescope_CanSetPark:
		case  Telescope_CanSetGuideRates:
		case  Telescope_CanSetDeclinationRate:
		case  Telescope_CanPulseGuide:
		case  Telescope_DeclinationRate:
		case  Telescope_DoesRefraction:
		case  Telescope_EquatorialSystem:
		case  Telescope_FocalLength:
		case  Telescope_TrackingRate:
		case  Telescope_Tracking:
		case  Telescope_TargetRightAscension:
		case  Telescope_TargetDeclination:
		case  Telescope_SlewSettleTime:
		case  Telescope_Slewing:
		case  Telescope_SiteLongitude:
		case  Telescope_CanPark:
		case  Telescope_SiteLatitude:
		case  Telescope_SiderealTime:
		case  Telescope_SideOfPier:
		case  Telescope_RightAscensionRate:
		case  Telescope_RightAscension:
		case  Telescope_IsPulseGuiding:
		case  Telescope_GuideRateRightAscension:
		case  Telescope_GuideRateDeclination:
		case  Telescope_SiteElevation:
		case  Telescope_CanFindHome:
		case  Telescope_UTCDATE:
		case  Telescope_AtPark:
		case  Telescope_AtHome:
		case  Telescope_ApertureDiameter:
		case  Telescope_ApertureArea:
		case  Telescope_Altitude:
		case  Telescope_AlignmentMode:
		case  Telescope_InterfaceVersion:
		case  Telescope_DriverVersion:
		case  Telescope_Connected:
		case  Telescope_TrackingRates:
		case  Telescope_AbortSlew:
		case  Telescope_Action:
		case  Telescope_AxisRates:
		case  Telescope_CanMoveAxis:
		case  Telescope_CommandBlind:
		case  Telescope_Command:
		case  Telescope_DestinationSideOfPier:
		case  Telescope_Dispose:
		case  Telescope_FindHome:
		case  Telescope_MoveAxis:
		case  Telescope_Park:
		case  Telescope_PulseGuide:
		case  Telescope_SetPark:
		case  Telescope_SetupDialog:
		case  Telescope_SlewToAltAz:
		case  Telescope_SlewToAltAzAsync:
		case  Telescope_SlewToCoordinates:
		case  Telescope_SlewToCoordinatesAsync:
		case  Telescope_SlewToTarget:
		case  Telescope_SlewToTargetAsync:
		case  Telescope_SyncToAltAz:
		case  Telescope_SyncToCoordinates:
		case  Telescope_SyncToTarget:
		case  Telescope_Unpark:
			Prepare_Packet_for_Controller(motor,command_number,parameter_one,parameter_two);
			Send_Packet_to_Controller();
			break;
	}
}
void Parse_Incoming_Focuser_Packet() {                                       // process an update message from the Azimuth controller
	Flash_Focuser_Light();
	Focuser_Incoming_Message_Available = false;
#ifdef DEBUG_FOCUSER
	Serial.print(millis(), DEC);
	Serial.println("\tPacket Received from Focuser");
#endif
	/*
	focuser_status = incoming_focuser_message.focuser_incoming_message.focuser_status;
	Focuser_Current_Position_value = incoming_focuser_message.focuser_incoming_message.focuser_position;
	if (incoming_focuser_message.focuser_incoming_message.focuser_status && focuser_temperature_valid) {
		Focuser_Current_Temperature_value = incoming_focuser_message.focuser_incoming_message.temperature;
	}
	else {
		Focuser_Current_Temperature_value = 0;
	}
	if (incoming_focuser_message.focuser_incoming_message.focuser_status && focuser_humidity_valid) {
		Focuser_Current_Humidity_value = incoming_focuser_message.focuser_incoming_message.humidity;
	}
	else {
		Focuser_Current_Humidity_value = 0;
	}
	if (incoming_focuser_message.focuser_incoming_message.focuser_status && focuser_date_valid) {
		Focuser_Year_value = incoming_focuser_message.focuser_incoming_message.year;
		Focuser_Month_value = incoming_focuser_message.focuser_incoming_message.month;
		Focuser_Day_value = incoming_focuser_message.focuser_incoming_message.day;
	}
	else {
		Focuser_Year_value = 0;
		Focuser_Month_value = 0;
		Focuser_Day_value = 0;
	}
	if (incoming_focuser_message.focuser_incoming_message.focuser_status && focuser_time_valid) {
		Focuser_Hour_value = incoming_focuser_message.focuser_incoming_message.hour;
		Focuser_Minute_value = incoming_focuser_message.focuser_incoming_message.minute;
		Focuser_Second_value = incoming_focuser_message.focuser_incoming_message.second;
	}
	else {
		Focuser_Hour_value = 0;
		Focuser_Minute_value = 0;
		Focuser_Second_value = 0;
	}
	if (incoming_focuser_message.focuser_incoming_message.focuser_status && focuser_latitude_valid) {
		Focuser_Latitude_value = incoming_focuser_message.focuser_incoming_message.latitude;
	}
	else {
		Focuser_Latitude_value = 0;
	}
	if (incoming_focuser_message.focuser_incoming_message.focuser_status && focuser_longitude_valid) {
		Focuser_Longitude_value = incoming_focuser_message.focuser_incoming_message.longitude;
	}
	else {
		Focuser_Longitude_value = 0;
	}
	if (incoming_focuser_message.focuser_incoming_message.focuser_status && focuser_altitude_valid) {
		Focuser_Telescope_Current_Altitude = incoming_focuser_message.focuser_incoming_message.altitude;
	}
	else {
		Focuser_Telescope_Current_Altitude = 0;
	}
	switch (incoming_focuser_message.focuser_incoming_message.command_number) {
	case Focuser_Update_command: {
#ifdef DEBUG_FOCUSER_UPDATE
		Serial.print(millis(), DEC);
		Serial.print("\tUpdate Packet from Focuser: Position ");
		Serial.println(Focuser_Position_value, DEC);
		Serial.print(millis(), DEC);
		Serial.print(" Temperature:");
		Serial.print(Focuser_Temperature_value, DEC);
		Serial.print(" Humidity:");
		Serial.println(Focuser_Humidity_value, DEC);
		Serial.print(millis(), DEC);
		Serial.print("\tLatitude:");
		Serial.println(Focuser_Latitude_value, DEC);
		Serial.print(millis(), DEC);
		Serial.print("\tLongitude:");
		Serial.println(Focuser_Longitude_value, DEC);
		Serial.print(millis(), DEC);
		Serial.print("\tFocuser Altitude:");
		Serial.println(Focuser_Telescope_Current_Altitude, DEC);		
		Serial.print(millis(), DEC);
		Serial.print("\tDate:");
		Serial.print(Focuser_Day_value, DEC);
		Serial.print("/");
		Serial.print(Focuser_Month_value, DEC);
		Serial.print("/20");
		Serial.print(Focuser_Year_value, DEC);
		Serial.print(" ");
		Serial.print(Focuser_Hour_vslue, DEC);
		Serial.print(":");
		Serial.print(Focuser_Minute_value, DEC);
		Serial.print(":");
		Serial.println(Focuser_Second_value, DEC);
#endif
		break;
	}
	case Focuser_Home_command: {
#ifdef DEBUG_OUTPUT_FOCUSER
		Serial.print(millis(), DEC);
		Serial.println("\tFocuser - Home Reply Received");
#endif
		break;
	}
	case Focuser_Move_to_command: {
#ifdef DEBUG_OUTPUT_FOCUSER
		Serial.print(millis(), DEC);
		Serial.println("\tFocuser - Move To Reply Received");
#endif
		break;
	}
	case Focuser_Set_Display_command: {
#ifdef DEBUG_OUTPUT_FOCUSER
		Serial.print(millis(), DEC);
		Serial.println("\tFocuser - Display On Reply Received");
#endif
		break;
	}
	case Focuser_Display_Off_command: {
#ifdef DEBUG_OUTPUT_FOCUSER
		Serial.print(millis(), DEC);
		Serial.println("\tFocuser - Display Off Reply Received");
#endif
		break;
	}
	case Focuser_Telescope_Emergency_Halt: {
#ifdef DEBUG_OUTPUT_FOCUSER
		Serial.print(millis(), DEC);
		Serial.println("\tFocuser - Halt Reply Received");
#endif
		break;
	}
	}
	*/
	}
void Parse_Incoming_Controller_Packet() {
	Flash_Controller_Light();
#ifdef DEBUG_CONTROLLER
	Serial.print(millis(), DEC);
	Serial.println("\tPacket Received from Controller");
#endif
	switch (Incoming_Message_from_Controller.contents.command_number) {
		case Focuser_Current_Focuser_Position:
		case Focuser_Motor_Moving_Status:
		case Focuser_Motor_Controller_Status:
		case Focuser_New_Target_Position:	
		case Focuser_Temperature:			
		case Focuser_MaxStep:				
		case Focuser_MaxIncrement:			
		case Focuser_Coil_Powerting:		
		case Focuser_Reverse_Directionting:
		case Focuser_MotorSpeed:								
		case Focuser_Display_in_Celsius:						
		case Focuser_Display_in_Fahrenheit:					
		case Focuser_User_specified_StepSize:				
		case Focuser_StepSize_value:						
		case Focuser_Temperature_Coefficient:				
		case Focuser_Temperature_Compensation:				
		case Focuser_State_of_Temperature_Compensation:		
		case Focuser_Temperature_Compensation_available:	
		case Focuser_Home_Motor_Position_To_ZERO:			
		case Focuser_Step_Mode:				
		case Focuser_Current_Motor_Position:				
		case Focuser_StepSize_is_Enabled:					
		case Focuser_Step_Size:						
		case Focuser_Display_Status:						
		case Focuser_Temperature_mode:						
		case Focuser_Target_Motor_Position:
		case Focuser_Reset_Arduino_Controller:
		case Focuser_Reset_Focuser_defaults:
		case Focuser_Motor_Speed:
		case Focuser_Motor_Speed_Threshold_When_Moving:
		case Focuser_Motor_Speed_Change_When_Moving:	
		case Focuser_Motor_Speed_Change_Enabled	:		
		case Focuser_Save_Settings_To_EEPROM:			
		case Focuser_Humidity:
		case Focuser_Longitude:
		case Focuser_Latitude:
		case Focuser_Altitude:		
		case Focuser_BEMF:
		case Focuser_Update_of_Position_When_Moving:
		case Focuser_Status_of_Home_Position_Switch:
		case Focuser_Move_Steps:
		case Focuser_Jogging_State:
		case Focuser_Jogging_Direction:
		case Focuser_Delay_After_Move:
		case Focuser_Backlash_In:
		case Focuser_Backlash_Out:
		case Focuser_Backlash_In_Steps_In:
		case Focuser_Number_Of_Backlash_Steps_In:
		case Focuser_Number_Of_Backlash_Steps_Out:
		case Focuser_Temperature_Compensation_Direction:
			Prepare_Packet_for_Focuser();
			Send_Packet_to_Focuser();
			break;
		case HUB_Display:
			hub_display_state = (bool) Incoming_Message_from_Controller.contents.parameter_one;
			break;
		case Telescope_UTCDATE:
		case Telescope_Azimuth:
		case Telescope_Declination:
		case Telescope_CanUnpark:
		case Telescope_CanSyncAltAz:
		case Telescope_CanSync:
		case Telescope_CanSlewAsync:
		case Telescope_CanSlewAltAzAsync:
		case Telescope_CanSlewAltAz:
		case Telescope_CanSlew:
		case Telescope_CanSetTracking:
		case Telescope_CanSetRightAscensionRate:
		case Telescope_CanSetPierSide:
		case Telescope_CanSetPark:
		case Telescope_CanSetGuideRates:
		case Telescope_CanSetDeclinationRate:
		case Telescope_CanPulseGuide:
		case Telescope_DeclinationRate:
		case Telescope_DoesRefraction:
		case Telescope_EquatorialSystem:
		case Telescope_FocalLength:
		case Telescope_TrackingRate:
		case Telescope_Tracking:
		case Telescope_TargetRightAscension:
		case Telescope_TargetDeclination:
		case Telescope_SlewSettleTime:
		case Telescope_Slewing:
		case Telescope_SiteLongitude:
		case Telescope_CanPark:
		case Telescope_SiteLatitude:
		case Telescope_SiderealTime:
		case Telescope_SideOfPier:
		case Telescope_RightAscensionRate:
		case Telescope_RightAscension:
		case Telescope_IsPulseGuiding:
		case Telescope_GuideRateRightAscension:
		case Telescope_GuideRateDeclination:
		case Telescope_SiteElevation:
		case Telescope_CanFindHome:
		case Telescope_AtPark:
		case Telescope_AtHome:
		case Telescope_ApertureDiameter:
		case Telescope_ApertureArea:
		case Telescope_Altitude:
		case Telescope_AlignmentMode:
		case Telescope_InterfaceVersion:
		case Telescope_DriverVersion:
		case Telescope_Connected:
		case Telescope_TrackingRates:	
		case Telescope_Action:
		case Telescope_AxisRates:
		case Telescope_CanMoveAxis:
		case Telescope_CommandBlind:
		case Telescope_Command:
		case Telescope_DestinationSideOfPier:
		case Telescope_Dispose:
		case Telescope_FindHome:
		case Telescope_MoveAxis:
		case Telescope_Park:
		case Telescope_PulseGuide:
		case Telescope_SetPark:
		case Telescope_SetupDialog:
		case Telescope_SlewToAltAz:
		case Telescope_SlewToAltAzAsync:
		case Telescope_SlewToCoordinates:
		case Telescope_SlewToCoordinatesAsync:
		case Telescope_SlewToTarget:
		case Telescope_SlewToTargetAsync:
		case Telescope_SyncToAltAz:
		case Telescope_SyncToCoordinates:
		case Telescope_SyncToTarget:
		case Telescope_Unpark:
			Prepare_Packet_for_Motor_Drivers();
			Send_Packet_to_Motor_Drivers();
			break;
	}
}
/*
void Parse_Command_to_Fieldf(String command) {				// this subroutine parses the rest string into fields command_fieldf[n]
  char commandc[40];                                        // space for the command conerted to a char array
  char delimiters[] = "!:,";
  char* valPosition;
  int i = 0;
  command.toCharArray(commandc, command.length());          // convert String to char[]
  valPosition  = strtok(commandc, delimiters);              // unpack the char[] into separate command_field doubles
  for (i = 1; i < 5; i++) {
    command_fieldf[i] = atof(valPosition);
#ifdef DEBUG_OUTPUT_FIELDS
	Serial.print(command_fieldf[i],DEC);
    Serial.print(",");
#endif
    valPosition = strtok(NULL, delimiters);
  }
#ifdef DEBUG_OUTPUT_FIELDS
  Serial.println("");
#endif
}
*/
void Prepare_Packet_for_Focuser() {
	Outgoing_Message_to_Focuser.contents.header = STX;
	Outgoing_Message_to_Focuser.contents.source = CONTROLLER;
	Outgoing_Message_to_Focuser.contents.command_number = Incoming_Message_from_Controller.contents.command_number;
	Outgoing_Message_to_Focuser.contents.parameter_one = Incoming_Message_from_Controller.contents.parameter_one;
	Outgoing_Message_to_Focuser.contents.parameter_two = Incoming_Message_from_Controller.contents.parameter_two;
	Outgoing_Message_to_Focuser.contents.footer = ETX;
}
void Prepare_Packet_for_Motor_Drivers() {
	Outgoing_Message_to_Motor_Drivers.contents.header = STX;
	Outgoing_Message_to_Motor_Drivers.contents.source = CONTROLLER;
	Outgoing_Message_to_Motor_Drivers.contents.command_number = Incoming_Message_from_Controller.contents.command_number;
	Outgoing_Message_to_Motor_Drivers.contents.parameter_one = Incoming_Message_from_Controller.contents.parameter_one;
	Outgoing_Message_to_Motor_Drivers.contents.parameter_two = Incoming_Message_from_Controller.contents.parameter_two;
	Outgoing_Message_to_Motor_Drivers.contents.footer = ETX;
}
void Prepare_Packet_for_Controller(byte source, byte command_number, double parameter_one, double parameter_two) {
	Outgoing_Message_to_Controller.contents.header = STX;
	Outgoing_Message_to_Controller.contents.source = source;
	Outgoing_Message_to_Controller.contents.command_number = (byte) command_number;
	Outgoing_Message_to_Controller.contents.parameter_one = parameter_one;
	Outgoing_Message_to_Controller.contents.parameter_two = parameter_two;
	Outgoing_Message_to_Controller.contents.footer = ETX;
}
void Send_Packet_to_Motor_Drivers() {
#ifdef DEBUG_OUTPUT_MOTOR_DRIVERS
	Serial.print(millis(), DEC);
	Serial.print("\tMotor Driver Message to be sent: ");
	for (int i = 0; i < Message_Structure_Length; i++) {
		Serial.print(Outgoing_Message_to_Motor_Drivers.sg_chars[i], HEX);
		Serial.print(",");
	}
	Serial.println();
	Serial.print(millis(), DEC);
	Serial.print("\t");
#endif
#ifdef DEBUG_OUTPUT_MOTOR_DRIVERS
	Serial.print("Message Sent to Motor Drivers ");
#endif
	for (int i = 0; i < Message_Structure_Length; i++) {
#ifdef DEBUG_OUTPUT_MOTOR_DRIVERS
		Serial.print(Outgoing_Message_to_Motor_Drivers.sg_chars[i], HEX);
		Serial.print(",");
#endif
		altitude_serial.write(Outgoing_Message_to_Motor_Drivers.sg_chars[i]);
		azimuth_serial.write(Outgoing_Message_to_Motor_Drivers.sg_chars[i]);
	}
#ifdef DEBUG_OUTPUT_MOTOR_DRIVERS
	Serial.println("");
#endif
}
void Send_Packet_to_Focuser() {
#ifdef DEBUG_OUTPUT_FOCUSER
	Serial.print(millis(), DEC);
	Serial.print("\tFocuser Message to be sent: ");
	for (int i = 0; i < Message_Structure_Length; i++) {
		Serial.print((char)Outgoing_Message_to_Focuser.sg_chars, HEX);
		Serial.print(",");
	}
	Serial.println();
	Serial.print("Message Sent to Focuser:");
#endif
	for (int i = 0; i < Message_Structure_Length; i++) {
		focuser_serial.write((char)Outgoing_Message_to_Focuser.sg_chars);
	}
#ifdef DEBUG_OUTPUT_FOCUSER
	Serial.println("");
#endif
}
void Send_Packet_to_Controller() {
#ifdef DEBUG_OUTPUT_CONTROLLER
	Serial.print(millis(), DEC);
	Serial.print("\tController Message to be sent: ");
	for (int i = 0; i < Message_Structure_Length; i++) {
		Serial.print(Outgoing_Message_to_Controller.sg_chars[i], HEX);
		Serial.print(",");
	}
	Serial.println();
	Serial.print(millis(), DEC);
	Serial.print("\t");
#endif
#ifdef DEBUG_OUTPUT_CONTROLLER
	Serial.print("Message Sent to Controller ");
#endif
	for (int i = 0; i < Message_Structure_Length; i++) {
#ifdef DEBUG_OUTPUT_CONTROLLER
		Serial.print(Outgoing_Message_to_Controller.sg_chars[i], HEX);
		Serial.print(",");
#endif
		server.write(Outgoing_Message_to_Controller.sg_chars[i]);
	}
#ifdef DEBUG_OUTPUT_CONTROLLER
	Serial.println("");
#endif
}
void Flash_Controller_Light(void) {
  if (millis() - time > debounce) {
    reading = digitalRead(CONTROLLER_LED_pin);
    if (reading == HIGH) {
      digitalWrite(CONTROLLER_LED_pin, LOW);
    }
    else {
      digitalWrite(CONTROLLER_LED_pin, HIGH);
    }
    time = millis();
  }
}
void Flash_Motor_Driver_Light(void) {
	if (millis() - time > debounce) {
		reading = digitalRead(MOTOR_DRIVER_LED_pin);
		if (reading == HIGH) {
			digitalWrite(MOTOR_DRIVER_LED_pin, LOW);
		}
		else {
			digitalWrite(MOTOR_DRIVER_LED_pin, HIGH);
		}
		time = millis();
	}
}
void Flash_Focuser_Light(void) {
	if (millis() - time > debounce) {
		reading = digitalRead(FOCUSER_LED_pin);
		if (reading == HIGH) {
			digitalWrite(FOCUSER_LED_pin, LOW);
		}
		else {
			digitalWrite(FOCUSER_LED_pin, HIGH);
		}
		time = millis();
	}
}
// send update to SEVEN_SEGMENT ---------------------------------------------------------------------------------------
void send_update_to_SEVEN_SEGMENT() {
	char displayaltitude_s[2];
	char displayazimuth_s[2];
	char displayfocuser_s[2];
	dtostrf(altitude_status, 2, 0, displayaltitude_s);
	dtostrf(azimuth_status, 2, 0, displayazimuth_s);
	dtostrf(focuser_status, 2, 0, displayfocuser_s);
	sprintf(display_string, "%s %s %s", displayaltitude_s,displayazimuth_s,displayfocuser_s);
	display(display_string);
}
void error(double error_number) {
	char error_s[5];
	char error_number_s[2];
	dtostrf(error_number, 2, 0, error_number_s);
	sprintf(display_string, "%s", error_number_s);
	Serial.print("Error Line:");
	Serial.println(display_string);
	error_display(display_string);
}
void set_register(byte reg, byte value) {   // ... write a value into a max7219 register See MAX7219 Datasheet, Table 1, page 6
	digitalWrite(display_CS, LOW);
	shiftOut(display_DIN, display_CLK, MSBFIRST, reg);
	shiftOut(display_DIN, display_CLK, MSBFIRST, value);
	digitalWrite(display_CS, HIGH);
}
void display(String thisString) { // ... display on the 7-segment display
	set_register(display_REG_SHUTDOWN, OFF);  // turn off display
	set_register(display_REG_SCANLIMIT, 7);   // scan limit 8 digits
	if (display_flag == false) {							// display temp & humidity
		set_register(display_REG_DECODE, 0b01111011);		// 
		if ((thisString.charAt(0) < 0x30) || (thisString.charAt(0) > 0x39)) thisString.setCharAt(0, 0x30);
		if ((thisString.charAt(1) < 0x30) || (thisString.charAt(1) > 0x39)) thisString.setCharAt(1, 0x30);
		if ((thisString.charAt(3) < 0x30) || (thisString.charAt(3) > 0x39)) thisString.setCharAt(3, 0x30);
		if ((thisString.charAt(4) < 0x30) || (thisString.charAt(4) > 0x39)) thisString.setCharAt(4, 0x30);
		if ((thisString.charAt(8) < 0x30) || (thisString.charAt(8) > 0x39)) thisString.setCharAt(8, 0x30);
		if ((thisString.charAt(9) < 0x30) || (thisString.charAt(9) > 0x39)) thisString.setCharAt(9, 0x30);
		set_register(1, thisString.charAt(7));
		set_register(2, thisString.charAt(6));
		set_register(3, h);                         // h
		set_register(4, thisString.charAt(4));      // 0
		set_register(5, thisString.charAt(3));      // 2
		set_register(6, thisString.charAt(1) | 0x80); // 3.
		set_register(7, thisString.charAt(0));      // 2
		set_register(8, t);                         // t
	}
	else {												// display position
		set_register(display_REG_DECODE, 0b01111101);		// 
		if ((thisString.charAt(0) < 0x30) || (thisString.charAt(0) > 0x39)) thisString.setCharAt(0, 0x30);
		if ((thisString.charAt(1) < 0x30) || (thisString.charAt(1) > 0x39)) thisString.setCharAt(1, 0x30);
		if ((thisString.charAt(2) < 0x30) || (thisString.charAt(2) > 0x39)) thisString.setCharAt(2, 0x30);
		if ((thisString.charAt(3) < 0x30) || (thisString.charAt(3) > 0x39)) thisString.setCharAt(3, 0x30);
		if ((thisString.charAt(4) < 0x30) || (thisString.charAt(4) > 0x39)) thisString.setCharAt(4, 0x30);
		set_register(1, thisString.charAt(7));  //  1
		set_register(2, dash);                  //  -
		set_register(3, thisString.charAt(4));
		set_register(4, thisString.charAt(3));  //  9
		set_register(5, thisString.charAt(2));  //  9
		set_register(6, thisString.charAt(1));  //  9
		set_register(7, thisString.charAt(0));  //  9
		set_register(8, P);                     //  p
	}
	set_register(display_REG_SHUTDOWN, ON);   // Turn on display
}
void error_display(String thisString) { // ... display on the 7-segment display, thisString is the error number
	set_register(display_REG_SHUTDOWN, OFF);  // turn off display
	set_register(display_REG_SCANLIMIT, 7);   // scan limit 8 digits
	set_register(display_REG_DECODE, 0b00000011);		// 
	if ((thisString.charAt(0) < 0x30) || (thisString.charAt(0) > 0x39)) thisString.setCharAt(0, 0x30);
	if ((thisString.charAt(1) < 0x30) || (thisString.charAt(1) > 0x39)) thisString.setCharAt(1, 0x30);
	if ((thisString.charAt(3) < 0x30) || (thisString.charAt(3) > 0x39)) thisString.setCharAt(3, 0x30);
	if ((thisString.charAt(4) < 0x30) || (thisString.charAt(4) > 0x39)) thisString.setCharAt(4, 0x30);
	if ((thisString.charAt(8) < 0x30) || (thisString.charAt(8) > 0x39)) thisString.setCharAt(8, 0x30);
	if ((thisString.charAt(9) < 0x30) || (thisString.charAt(9) > 0x39)) thisString.setCharAt(9, 0x30);
	set_register(1, thisString.charAt(1));
	set_register(2, thisString.charAt(0));
	set_register(3, dash);
	set_register(4, R);		// R  
	set_register(5, O);     // O
	set_register(6, R);     // R
	set_register(7, R);		// R
	set_register(8, E);     // E
	set_register(display_REG_SHUTDOWN, ON);   // Turn on display
}
void resetDisplay() {
	set_register(display_REG_SHUTDOWN, OFF);   // turn off display
	set_register(display_REG_DISPTEST, OFF);   // turn off test mode
	set_register(display_REG_INTENSITY, 0x0D); // display intensity
}