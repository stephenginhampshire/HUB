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
	1.		27/0/2018
	1.1		12/12/2018	Focuser resolution removed from protocol
	1.2		20/01/2019	Recoded command handling
*/
// Inclusions ------------------------------------------------------------------------------

#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Ethernet2.h>
#include <SoftwareSerial.h>
#include "HUB_Commands.h"

#define DEBUG_OUTPUT_SETUP
//#define DEBUG_OUTPUT_MOTOR_DRIVERS
//#define DEBUG_OUTPUT_FOCUSER
//#define DEBUG_OUTPUT_CONTROLLER
//#define DEBUG_OUTPUT_COMMANDS
//#define DEBUG_OUTPUT_FIELDS
//#define DEBUG_OUTPUR_COMMANDS
//#define DEBUG_MOTOR_COMMANDS
//#define DEBUG_FOCUSER_COMMANDS
//#define DEBUG_HUB_COMMANDS
#define DEBUG_LOGGER
#define DEBUG_DUMMY_DATETIME

//#define WDT						// enable WatchDog Timer
//#define ETHERNET				// enable ethernet			

#ifdef WDT
	#include <avr/wdt.h>
#endif
// Definitions ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define display_DIN		8		// MAX729 DATA pin		blue wire 
#define display_CS		9		// MAX729 CS/LOAD pin	yellow wire 
#define display_CLK		10		// MAX729 CLK pin		green wire
#define logger_RX_pin	11		// yellow
#define logger_TX_pin	12		// blue
#define GREEN_LED_pin	2		
#define ORANGE_LED_pin	3		
#define RED_LED_pin		4		

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
#define logger_baud		38400
#define STX             0x02
#define ETX             0x03
#define TO				0
#define FROM			1

#define stage_day_month				0
#define stage_year_hour				1
#define stage_minute_second			2
#define stage_wait_day_month		3
#define stage_wait_year_hour		4
#define stage_wait_minute_second	5
#define stage_wait_complete			6

#define LED_FLASH_TIME		20      // number of milli seconds to light the LEDs
// Constants ------------------------------------------------------------------------------------------
byte mac[] = { 0x90, 0xA2, 0xDA, 0x11, 0x23, 0x40 };				// MAC address of Ethernet controller, found on a sticker on the back of the Ethernet shield.
//byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };				// MAC address of Jamie's Ethernet Controller
//IPAddress ip(92, 68, 0, 30);									// IP Address (FIXED) of this server
//IPAddress gateway(92, 68, 0, 00);
//IPAddress subnet(255, 255, 255, 0);

// Instantiations -------------------------------------------------------------------------------------------------------------------
#ifdef ETHERNET
EthernetServer server(80);                                      // Create a server listening on the given port.
#endif
SoftwareSerial logger_serial(logger_RX_pin,logger_TX_pin); // RX, TX, Create the serial port for logging data

// Working Variables ----------------------------------------------------------------------------------------------------------------
int state = HIGH;						// the current state of the output pin
int reading;							// the current reading from the input pin
int previous = LOW;						// the previous reading from the input pin
long red_time = 0;						// the last time the output pin was toggled
long blue_time = 0;
long green_time = 0;
long debounce = 300;					// the debounce time, increase if the output flickers
bool hub_display_state = true;			// default to on

typedef struct {
  unsigned char header;					// [0] STX
  unsigned char source;					// [1] source of message
  unsigned char command_number;			// [2] Command Number
  double parameter_one;					// [3 - 6]
  double parameter_two;					// [7 - 10]
  unsigned char footer;					// [11] ETX
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

int date_required = 0;						// check track of stage obtaining date and time
long system_day = 0;
long system_month = 0;
long system_year = 0;
long system_hour = 0;
long system_minute = 0;
long system_second = 0;

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
bool command_sent = false;
long gps_time_LED_On = 0;
bool gps_status = false;
long running_time_LED_On = (long)0;
bool running_status = false;
long hub_time_LED_On = 0;
bool hub_status = false;
unsigned long last_status_update = 0;
char altitude_count = 0;
char azimuth_count = 0;
char focuser_count = 0;
enum {
	display_REG_DECODE = 0x09,
	display_REG_INTENSITY = 0x0A,
	display_REG_SCANLIMIT = 0x0B,
	display_REG_SHUTDOWN = 0x0C,
	display_REG_DISPTEST = 0x0F,
};
enum { OFF = 0, ON = 1};
bool display_flag = false;
const byte DP = 0b10000000;
const byte t = 0b00001111;
const byte h = 0b00010111;
const byte P = 0b01100111;
const byte dash = 0b00000001;
const byte E = 0b01001111;
const byte R = 0b01110111;
const byte S = 0b01011011;
const byte O = 0b01111110;
const byte space = 0b00000000;
bool focuser_display_on = true;
char display_string[9];
// Prototypes -----------------------------------------------------------------------------------------------------------------------------
//-- Setup --------------------------------------------------------------------------------------------------------------------------------
void setup() {
#ifdef WDT
  wdt_disable();													// disable watchdog timer during setup
#endif
#ifdef DEBUG_OUTPUT_SETUP
  Serial.print(millis(), DEC);
  Serial.println("\tSetup Commenced");
#endif
  Serial.begin(115200);												// Start monitor serial communication with the given baud rate.
  while (!Serial) {
    ;
  }
  altitude_serial.begin(altitude_baud, SERIAL_8N2);					// initialise the altitude serial port
  azimuth_serial.begin(azimuth_baud, SERIAL_8N2);					// initialise the azimuth serial port
  focuser_serial.begin(focuser_baud, SERIAL_8N2);					// initialise the focuser serial port
  logger_serial.begin(logger_baud);									// initialise the logger serial port
  altitude_serial.flush();											// clear the altitude serial buffer
  azimuth_serial.flush();											// clear the azimuth serial buffer
  focuser_serial.flush();											// xlear the focuser serial buffer
  logger_serial.flush();											// clear the logger serial buffer
#ifdef DEBUG_OUTPUT_SETUP
  Serial.print(millis(), DEC);
  Serial.println("\tSerial Ports setup");
  Serial.print(millis(), DEC);
  Serial.println("\tSetting Up Ethernet Server");
#endif
#ifdef ETHERNET
  if (Ethernet.begin(mac) == 0) {									//  Ethernet.begin(mac, ip, gateway, subnet);                       // Initialize the Ethernet shield
#ifdef DEBUG_OUTPUT_SETUP
    Serial.print(millis(), DEC);
    Serial.println("\tFailed to configure Ethernet using DHCP");	// no point in carrying on, so do nothing forevermore, try to configure using IP address instead of DHCP:
#endif
	while (1);														// Error(Ethernet_Error);
  }
  server.begin();
#ifdef DEBUG_OUTPUT_SETUP
  Serial.print(millis(), DEC);
  Serial.print("\tEthernet Server Setup Complete, at ");
  Serial.println(Ethernet.localIP());
#endif
#endif // end ETHERNET
#ifdef DEBUG_OUTPUT_SETUP
  Serial.print(millis(), DEC);
  Serial.println("\tASCOM setup complete");
#endif
#ifdef WDT
  Serial.print(millis(), DEC);
  Serial.println("\tSetting Up Watchdog");
  wdt_enable(WDTO_2S);											// enable a 2 second watchdog timeout
  Serial.print(millis(), DEC);
  Serial.println("\tWatchdog Setup Complete");
#endif
  digitalWrite(ORANGE_LED_pin, LOW);
  pinMode(ORANGE_LED_pin, OUTPUT);
  digitalWrite(GREEN_LED_pin, LOW);
  pinMode(GREEN_LED_pin, OUTPUT);
  digitalWrite(RED_LED_pin, LOW);
  pinMode(RED_LED_pin, OUTPUT);
  pinMode(display_DIN, OUTPUT);
  pinMode(display_CS, OUTPUT);
  pinMode(display_CLK, OUTPUT);
  resetDisplay();							// reset the MAX729 display
  send_update_to_SEVEN_SEGMENT();
#ifdef DEBUG_OUTPUT_SETUP
  Serial.print(millis(), DEC);
  Serial.println("\tSetup Complete");
#endif
} // end setup
// Main -----------------------------------------------------------------------------------------------------------------------------------
void loop() {
#ifdef ETHERNET
	int ethernet_status = (int)Ethernet.maintain();
	switch (ethernet_status) {
	case 0: {						// nothing happened
		break;
	}
	case 1: {						// renew failed
		Ethernet.begin(mac);
		break;
	}
	case 2: {						// renew success
		break;
	}
	case 3: {						// rebind fail
		Ethernet.begin(mac);
		break;
	}
	case 4: {						// rebind success
		break;
	}
	}
#endif
#ifdef WDT
	wdt_reset();													// reset the watchdog timer
#endif
	if (Check_Controller_Packet() == true) Process_Incoming_Packet_from_Controller();
	if (Check_Altitude_Packet() == true) Process_Incoming_Packet_from_Motor_Driver((unsigned char)SOURCE_ALTITUDE_MOTOR);
	if (Check_Azimuth_Packet() == true) Process_Incoming_Packet_from_Motor_Driver((unsigned char)SOURCE_AZIMUTH_MOTOR);
	if (Check_Focuser_Packet() == true) Process_Incoming_Packet_from_Focuser();

	if ((!(millis() % (long)60000)) || date_required != (int)stage_wait_complete) {				// get the date and time from the controller every minute
		if (date_required == (int)stage_wait_complete) date_required = (int)stage_day_month;
		get_datetime();
	}
	//-- TEST/DIAGNOSTICS --------------------------------------------------------------------------------------------------------
	if (millis() == 20000) {   // run the test after 20 seconds
#ifdef DEBUG_FOCUSER_COMMANDS
		unsigned char source = 0;
		unsigned char motor = 0;
		unsigned char command_number = 0;
		double parameter_one = 0;
		double parameter_two = 0;

		//		command_number = (unsigned char)Focuser_Current_Focuser_Position;
		//		command_number = (unsigned char)Focuser_Motor_Moving_Status;
		//		command_number = (unsigned char)Focuser_Motor_Controller_Status;
		//		command_number = (unsigned char)Focuser_Firmware_Version_String;
		//		command_number = (unsigned char)Focuser_Firmware_Name_and_Version_String;
		//		command_number = (unsigned char)Focuser_New_Target_Position;
		//		command_number = (unsigned char)Focuser_Temperature;
		//		command_number = (unsigned char)Focuser_Maximum_Step;									// parameter_one == 0 GET, else SET
		//		command_number = (unsigned char)Focuser_Maximum_Increment;
		//		command_number = (unsigned char)Focuser_Coil_Power;									// parameter_one == 0 GET, else SET
		//		command_number = (unsigned char)Focuser_Reverse_Direction;
		//		command_number = (unsigned char)Focuser_Motor_Speed;
		//		command_number = (unsigned char)Focuser_Display_Unit;
		//		command_number = (unsigned char)Focuser_User_Specified_Step_Size;
		//		command_number = (unsigned char)Focuser_Step_Size;
		//		command_number = (unsigned char)Focuser_Temperature_Coefficient;
		//		command_number = (unsigned char)Focuser_Temperature_Compensation;
		//		command_number = (unsigned char)Focuser_State_of_Temperature_Compensation;
		//		command_number = (unsigned char)Focuser_Temperature_Compensation_Available;
		//		command_number = (unsigned char)Focuser_Home_Motor_Position_To_ZERO;
		//		command_number = (unsigned char)Focuser_Step_Mode;
		//		command_number = (unsigned char)Focuser_Current_Motor_Position;
		//		command_number = (unsigned char)Focuser_Step_Size_is_Enabled;
		//		command_number = (unsigned char)Focuser_Display;
		//		command_number = (unsigned char)Focuser_Display_Status;
		//		command_number = (unsigned char)Focuser_Temperature_Mode;
		//		command_number = (unsigned char)Focuser_Target_Motor_Position;
		//		command_number = (unsigned char)Focuser_Reset_Arduino_Controller;
		//		command_number = (unsigned char)Focuser_Reset_Focuser_Defaults;
		//		command_number = (unsigned char)Focuser_Motor_Speed_Threshold_When_Moving;
		//		command_number = (unsigned char)Focuser_Motor_Speed_Change_When_Moving;
		//		command_number = (unsigned char)Focuser_Motor_Speed_Change_Enabled;
		//		command_number = (unsigned char)Focuser_Save_Settings_to_EEPROM;
		//		command_number = (unsigned char)Focuser_Humidity;
		//		command_number = (unsigned char)Focuser_Longitude;
		//		command_number = (unsigned char)Focuser_Latitude;
		//		command_number = (unsigned char)Focuser_Altitude;
		//		command_number = (unsigned char)Focuser_BEMF;
		//		command_number = (unsigned char)Focuser_Update_of_Position_When_Moving;
		//		command_number = (unsigned char)Focuser_Status_of_Home_Position_Switch;
		//		command_number = (unsigned char)Focuser_Jogging_Steps;
		//		command_number = (unsigned char)Focuser_Jogging_State;
		//		command_number = (unsigned char)Focuser_Jogging_Direction;
		//		command_number = (unsigned char)Focuser_Delay_After_Move;
		//		command_number = (unsigned char)Focuser_Backlash_In;
		//		command_number = (unsigned char)Focuser_Backlash_Out;
		//		command_number = (unsigned char)Focuser_Number_of_Backlash_Steps_In;
		//		command_number = (unsigned char)Focuser_Number_of_Backlash_Steps_Out;
		//		command_number = (unsigned char)Focuser_Temperature_Compensation_Direction;
		//		command_number = (unsigned char)Focuser_to_Controller_Heartbeat;

		switch ((int)command_number) {
		case Telescope_Azimuth: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Current_Focuser_Position: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Motor_Moving_Status: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Motor_Controller_Status: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Firmware_Version_String: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Firmware_Name_and_Version_String: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Target_Position: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Temperature: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Maximum_Step: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}									// parameter_one == 0 GET, else SET
		case Focuser_Maximum_Increment: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Coil_Power: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}									// parameter_one == 0 GET, else SET
		case Focuser_Reverse_Direction: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Motor_Speed: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Display_Unit: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_User_Specified_Step_Size: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Step_Size: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Temperature_Coefficient: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Temperature_Compensation: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Temperature_Compensation_Available: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Home_Motor_Position_To_ZERO: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Step_Mode: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Step_Size_is_Enabled: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Display: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Temperature_Mode: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Reset_Arduino_Controller: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Reset_Focuser_Defaults: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Motor_Speed_Threshold_When_Moving: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Motor_Speed_Change_When_Moving: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Save_Settings_to_EEPROM: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Humidity: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Longitude: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Latitude: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Altitude: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_BEMF: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Update_of_Position_When_Moving: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Status_of_Home_Position_Switch: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Jogging_Steps: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Jogging_State: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Jogging_Direction: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Delay_After_Move: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Backlash_In: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Backlash_Out: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Number_of_Backlash_Steps_In: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Number_of_Backlash_Steps_Out: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_Temperature_Compensation_Direction: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Focuser_to_Controller_Heartbeat: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		}			// end of switch
		if (command_number != (unsigned char)NULL) {
			if (command_sent == false) {
				Serial.print(millis(), DEC);
				Serial.print("\tSending Command to Focuser : ");
				Serial.print(command_number, DEC);
				Serial.print(", Source : ");
				Serial.print(source, DEC);
				Serial.print(" Parameter One : ");
				Serial.print(parameter_one, DEC);
				Serial.print(", Parameter Two : ");
				Serial.println(parameter_two, DEC);
				Prepare_Packet_for_Output(source, command_number, parameter_one, parameter_two);
				Send_Packet_to_Focuser();
				command_sent = true;
			}
		}
		else {
			Serial.print(millis(), DEC);
			Serial.println("\tNo Command Sent");
			command_sent = true;
		}
#endif
#ifdef DEBUG_MOTOR_COMMANDS
		//		motor = (unsigned char)SOURCE_ALTITUDE_MOTOR;
		//		motor = (unsigned char)SOURCE_AZIMUTH_MOTOR;
		//		motor = (unsigned char)SOURCE_BOTH_MOTORS;

		//		command_number = (unsigned char)Telescope_Azimuth;
		//		command_number = (int)Telescope_Declination;
		//		command_number = (int)Telescope_Can_Unpark;
		//		command_number = (int)Telescope_Can_Sync_AltAz;
		//		command_number = (int)Telescope_Can_Sync;
		//		command_number = (int)Telescope_Can_Slew_Async;
		//		command_number = (int)Telescope_Can_Slew_AltAz_Async;
		//		command_number = (int)Telescope_Can_Slew_AltAz;
		//		command_number = (int)Telescope_Can_Slew;
		//		command_number = (int)Telescope_Can_Set_Tracking;
		//		command_number = (int)Telescope_Can_Set_Right_Ascension_Rate;
		//		command_number = (int)Telescope_Can_Set_Pier_Side;
		//		command_number = (int)Telescope_Can_Set_Park;
		//		command_number = (int)Telescope_Can_Set_Guide_Rates;
		//		command_number = (int)Telescope_Can_Set_Declination_Rate;
		//		command_number = (int)Telescope_Can_Pulse_Guide;
		//		command_number = (int)Telescope_Declination_Rate;
		//		command_number = (int)Telescope_Does_Refraction;
		//		command_number = (int)Telescope_Equatorial_System;
		//		command_number = (int)Telescope_Focal_Length;
		//		command_number = (int)Telescope_Tracking_Rate;
		//		command_number = (int)Telescope_Tracking;
		//		command_number = (int)Telescope_Target_Right_Ascension;
		//		command_number = (int)Telescope_Target_Declination;
		//		command_number = (int)Telescope_Slew_Settle_Time;
		//		command_number = (int)Telescope_Slewing;
		//		command_number = (int)Telescope_Site_Longitude;
		//		command_number = (int)Telescope_Can_Park;
		//		command_number = (int)Telescope_Site_Latitude;
		//		command_number = (int)Telescope_Sidereal_Time;
		//		command_number = (int)Telescope_Side_of_Pier;
		//		command_number = (int)Telescope_Right_Ascension_Rate;
		//		command_number = (int)Telescope_Right_Ascension;
		//		command_number = (int)Telescope_Is_Pulse_Guiding;
		//		command_number = (int)Telescope_Guide_Rate_Right_Ascension;
		//		command_number = (int)Telescope_Guide_Rate_Declination;
		//		command_number = (int)Telescope_Site_Elevation;
		//		command_number = (int)Telescope_Can_Find_Home ;
		//		command_number = (int)Telescope_UTCDATE;
		//		command_number = (int)Telescope_At_Park;
		//		command_number = (int)Telescope_At_Home;
		//		command_number = (int)Telescope_Aperture_Diameter;
		//		command_number = (int)Telescope_Aperture_Area;
		//		command_number = (int)Telescope_Altitude;
		//		command_number = (int)Telescope_Alignment_Mode;
		//		command_number = (int)Telescope_Interface_Version;
		//		command_number = (int)Telescope_Driver_Version;
		//		command_number = (int)Telescope_Connected;
		//		command_number = (int)Telescope_Tracking_Rates;
		//		command_number = (int)Telescope_Abort_Slew;
		//		command_number = (int)Telescope_Action;
		//		command_number = (int)Telescope_Axis_Rates;
		//		command_number = (int)Telescope_Can_Move_Axis;
		//		command_number = (int)Telescope_Command_Blind;
		//		command_number = (int)Telescope_Command;
		//		command_number = (int)Telescope_Destination_Side_of_Pier;
		//		command_number = (int)Telescope_Dispose;
		//		command_number = (int)Telescope_Find_Home;
		//		command_number = (int)Telescope_Move_Axis;
		//		command_number = (int)Telescope_Park;
		//		command_number = (int)Telescope_Pulse_Guide;
		//		command_number = (int)Telescope_Set_Park;
		//		command_number = (int)Telescope_Setup_Dialog;
		//		command_number = (int)Telescope_Slew_to_AltAz;
		//		command_number = (int)Telescope_Slew_to_AltAz_Async;
		//		command_number = (int)Telescope_Slew_to_Coordinates;
		//		command_number = (int)Telescope_Slew_to_Coordinates_Async;
		//		command_number = (int)Telescope_Slew_to_Target;
		//		command_number = (int)Telescope_Slew_to_Target_Asyn;
		//		command_number = (int)Telescope_Sync_to_AltAz;
		//		command_number = (int)Telescope_Sync_to_Coordinates;
		//		command_number = (int)Telescope_Sync_to_Target;
		//		command_number = (int)Telescope_Unpark;
		//		command_number = (int)Telescope_Altitude_to_Controller_Heartbest;
		//		command_number = (int)Telescope_Azimuth_to_Controller_Heartbeat;

		switch ((int)command_number) {
		case Telescope_Azimuth: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Declination: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Unpark: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Sync_AltAz: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Sync: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Slew_Async: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Slew_AltAz_Async: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Slew_AltAz: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Slew: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Set_Tracking: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Set_Right_Ascension_Rate: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Set_Pier_Side: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Set_Park: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Set_Guide_Rates: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Set_Declination_Rate: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Pulse_Guide: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Declination_Rate: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Does_Refraction: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Equatorial_System: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Focal_Length: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Tracking_Rate: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Tracking: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Target_Right_Ascension: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Target_Declination: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Slew_Settle_Time: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Slewing: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Site_Longitude: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Park: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Site_Latitude: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Sidereal_Time: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Side_of_Pier: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Right_Ascension_Rate: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Right_Ascension: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Is_Pulse_Guiding: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Guide_Rate_Right_Ascension: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Guide_Rate_Declination: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Site_Elevation: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Find_Home: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_UTCDATE: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_At_Park: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_At_Home: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Aperture_Diameter: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Aperture_Area: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Altitude: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Alignment_Mode: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Interface_Version: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Driver_Version: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Connected: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Tracking_Rates: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Abort_Slew: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Action: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Axis_Rates: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Can_Move_Axis: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Command_Blind: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Command: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Destination_Side_of_Pier: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Dispose: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Find_Home: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Move_Axis: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Park: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Pulse_Guide: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Set_Park: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Setup_Dialog: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Slew_to_AltAz: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Slew_to_AltAz_Async: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Slew_to_Coordinates: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Slew_to_Coordinates_Async: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Slew_to_Target: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Slew_to_Target_Async: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Sync_to_AltAz: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Sync_to_Coordinates: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Sync_to_Target: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Unpark: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Altitude_to_Controller_Heartbest: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		case Telescope_Azimuth_to_Controller_Heartbeat: {
			parameter_one = 0;
			parameter_two = 0;
			break;
		}
		}		// end of switch
		if (command_number != (unsigned char)command_number) {
			if (command_sent == false) {
				Serial.print(millis(), DEC);
				Serial.print("\tSending ");
				if (motor == SOURCE_ALTITUDE_MOTOR) {
					Serial.print("the Altitude Motor Command : ");
				}
				else if (motor == SOURCE_AZIMUTH_MOTOR) {
					Serial.print("the Azimuth Motor Command : ");
				}
				else if (motor == SOURCE_BOTH_MOTORS) {
					Serial.print("Both Motors Command : ");
				}
				Serial.print(command_number, DEC);
				Serial.print(", Source : ");
				Serial.print(source, DEC);
				Serial.print(" Parameter One : ");
				Serial.print(parameter_one, DEC);
				Serial.print(", Parameter Two : ");
				Serial.print(parameter_two, DEC);
				Prepare_Packet_for_Output(source, command_number, parameter_one, parameter_two);
				Send_Packet_to_Motor_Drivers((unsigned char)motor);
				command_sent = true;
			}
		}
#endif	//--End of Debug ---------------------------------------------------------------------------------------------------------------------------
	}
} // end of main loop
// Controller Functions ---------------------------------------------------------------------------------------------------
void get_datetime(void) {
#ifdef DEBUG_DUMMY_DATETIME
	date_required = (unsigned char) stage_wait_complete;
	system_day = 1;
	system_month = 1;
	system_year = 2019;
	system_hour = 9;
	system_minute = 30;
	system_second = 55;
#else
	unsigned char source = (unsigned char)SOURCE_HUB;
	unsigned char command_number = 0;
	double parameter_one = 0;
	double parameter_two = 0;
	switch (date_required) {
	case 0: {												// day and month required
		command_number = (unsigned char)HUB_Get_Day_Month;
		Prepare_Packet_for_Output(source, command_number, parameter_one, parameter_two);
		Send_Packet_to_Controller();
		date_required = (int) stage_wait_day_month;
		break;
	}
	case (int) stage_wait_day_month: {						// do nothing, waiting for day and month
		break;
	}
	case (int) stage_year_hour: {							// year and hour required
		command_number = (unsigned char)HUB_Get_Year_Hour;
		Prepare_Packet_for_Output(source, command_number, parameter_one, parameter_two);
		Send_Packet_to_Controller();
		date_required = (int)stage_wait_year_hour;
		break;
	}
	case (int) stage_wait_year_hour: {						// do nothing, waiting for year and hour
		break;
	}
	case (int) stage_minute_second: {						// minute and second required
		command_number = (unsigned char)HUB_Get_Minute_Second;
		Prepare_Packet_for_Output(source, command_number, parameter_one, parameter_two);
		Send_Packet_to_Controller();
		date_required = (int) stage_wait_minute_second;
		break;
	}
	case (int) stage_wait_minute_second: {						// do nothing, waiting for minute and second
		break;
	}
	}
#endif
}
void Hub_Display_On() {
#ifdef DEBUG_OUTPUT_COMMANDS
	Serial.print(millis(), DEC);
	Serial.println("\tHub Display Turned On");
#endif
	hub_display_state = true;
}
void Hub_Display_Off() {
#ifdef DEBUG_OUTPUT_COMMANDS
	Serial.print(millis(), DEC);
	Serial.println("\tHub Display Turned Off");
#endif
	hub_display_state = false;
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
  } // end of while altitude
  return Altitude_Incoming_Message_Available;
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
	} // end of while focuser
	return Focuser_Incoming_Message_Available;
}
bool Check_Controller_Packet(void) {
#ifdef ETHERNET
	EthernetClient client = server.available();                  // Listen for incoming client requests.
	if (client) {;
		for (int i = 1; i < Message_Structure_Length-1; i++) {
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
void write_logger(unsigned char packet_type, unsigned char direction, unsigned char source, unsigned char command_number, double parameter_one, double parameter_two) {
	char temp_str[2];
	int i,x;
	String Logger_Message = "";
	Logger_Message.reserve(60);
	dtostrf(system_day, 2, 0, temp_str);
	x = sizeof(temp_str) - 1;
	for (i = 0; i < x; i++) {
		Logger_Message += temp_str[i];					// dd
	}
	Logger_Message += "/";								// dd/
	dtostrf(system_month, 2, 0, temp_str);
	x = sizeof(temp_str) - 1;
	for (i = 0; i < x; i++) {
		Logger_Message += temp_str[i];					// dd/mm
	}
	Logger_Message += "/";								// dd/mm/
	dtostrf(system_year, 4, 0, temp_str);
	x = sizeof(temp_str) - 1;
	for (i = 0; i < x; i++) {
		Logger_Message += temp_str[i];					// dd/mm/yy
	}
	Logger_Message += " ";								// dd/mm/yy 
	dtostrf(system_hour, 2, 0, temp_str);
	x = sizeof(temp_str) - 1;
	for (i = 0; i < x; i++) {
		Logger_Message += temp_str[i];					// dd/mm/yy hh
	}
	Logger_Message += ":";								// dd/mm/yy hh:
	dtostrf(system_minute, 2, 0, temp_str);
	x = sizeof(temp_str) - 1;
	for (i = 0; i < x; i++) {
		Logger_Message += temp_str[i];					// dd/mm/yy hh:mm
	}
	Logger_Message += ":";								// dd/mm/yy hh:mm:
	dtostrf(system_second, 2, 0, temp_str);
	x = sizeof(temp_str) - 1;
	for (i = 0; i < x; i++) {
		Logger_Message += temp_str[i];					// dd/mm/yy hh:mm:ss
	}
	if (direction == (unsigned char)TO) {
		Logger_Message += " to ";					// dd/mm/yy hh:mm:ss TO
	}
	else {
		Logger_Message += " from ";					// dd/mm/yy hh:mm:ss FROM
	}
	switch ((int)source) {
	case SOURCE_CONTROLLER: {
		Logger_Message += "Controller ";				// dd/mm/yy hh:mm:ss TO Controller 
		break;
	}
	case SOURCE_ALTITUDE_MOTOR: {
		Logger_Message += "Altitude Motor";
		break;
	}
	case SOURCE_AZIMUTH_MOTOR: {
		Logger_Message += "Azimuth Motor";
		break;
	}
	case SOURCE_FOCUSER: {
		Logger_Message += "Focuser";
		break;
	}
	case SOURCE_BOTH_MOTORS: {
		Logger_Message += "Both Motors";
		break;
	}
	case SOURCE_HUB: {
		Logger_Message += "Hub";
		break;
	}
	}
	Logger_Message += ",";									// dd/mm/yy hh:mm:ss TO Controller, 
	dtostrf(command_number, 2, 0, temp_str);
	x = sizeof(temp_str) - 1;
	for (int i = 0; i < x; i++) {
		Logger_Message += temp_str[i];						// dd/mm/yy hh:mm:ss TO Controller,99 
	}
	Logger_Message += ",";									// dd/mm/yy hh:mm:ss To Controller,99,
	dtostrf(parameter_one, 7, 2, temp_str);
	x = sizeof(temp_str) - 1;
	for (int i = 0; i < x; i++) {
		Logger_Message += temp_str[i];						// dd/mm/yy hh:mm:ss TO Controller,99,9999.99 
	}
	Logger_Message += ",";									// dd/mm/yy hh:mm:ss To Controller,99,9999.99,
	dtostrf(parameter_two, 7, 2, temp_str);
	x = sizeof(temp_str) - 1;
	for (int i = 0; i < x; i++) {
		Logger_Message += temp_str[i];					
	}
	x = sizeof(Logger_Message) - 1;
	if (packet_type == (unsigned char)HUB_Delete_Log_File) {
		logger_serial.write((unsigned char)HUB_Delete_Log_File);
	}
	else {
		logger_serial.write((unsigned char)STX);
	}
	for (int i = 0; i < x; i++) {		// send message to logger port
		logger_serial.write(Logger_Message.charAt(i));
	}
	logger_serial.write((unsigned char) ETX);
}
void Process_Incoming_Packet_from_Motor_Driver(unsigned char motor) {// process an update message from a motor driver
	int command_number = 0;
	double parameter_one = 0;
	double parameter_two = 0;
	Flash_Green_Light();
#ifdef DEBUG_OUTPUT_MOTOR_DRIVER
	Serial.print(millis(), DEC);
	Serial.println("\tPacket Received from Altitude Motor");
#endif
	if (motor == (unsigned char)SOURCE_AZIMUTH_MOTOR) {
		write_logger((unsigned char)NULL,(unsigned char)FROM, (unsigned char)SOURCE_ALTITUDE_MOTOR, (unsigned char)command_number, parameter_one, parameter_two);
		Altitude_Incoming_Message_Available = false;
#ifdef DEBUG_OUTPUT_MOTOR_DRIVER
		Serial.print(millis(), DEC);
		Serial.println("\tPacket Received from Altitude Motor");
#endif
		command_number = (int) Incoming_Message_from_Azimuth.contents.command_number;
		parameter_one = Incoming_Message_from_Azimuth.contents.parameter_one;
		parameter_two =  Incoming_Message_from_Azimuth.contents.parameter_two;
	}
	else {
		write_logger((unsigned char)NULL,(unsigned char)FROM, (unsigned char)SOURCE_AZIMUTH_MOTOR, (unsigned char)command_number, parameter_one, parameter_two);
		Azimuth_Incoming_Message_Available = false;
#ifdef DEBUG_OUTPUT_MOTOR_DRIVER
		Serial.print(millis(), DEC);
		Serial.println("\tPacket Received from Azimuth Motor");
#endif
		command_number = Incoming_Message_from_Altitude.contents.command_number;
		parameter_one = Incoming_Message_from_Altitude.contents.parameter_one;
		parameter_two = Incoming_Message_from_Altitude.contents.parameter_two;
	}
	switch (command_number) {
		case  Telescope_Azimuth:
		case  Telescope_Declination:
		case  Telescope_Can_Unpark:
		case  Telescope_Can_Sync_AltAz:
		case  Telescope_Can_Sync:
		case  Telescope_Can_Slew_Async:
		case  Telescope_Can_Slew_AltAz_Async:
		case  Telescope_Can_Slew_AltAz:
		case  Telescope_Can_Slew:
		case  Telescope_Can_Set_Tracking:
		case  Telescope_Can_Set_Right_Ascension_Rate:
		case  Telescope_Can_Set_Pier_Side:
		case  Telescope_Can_Set_Park:
		case  Telescope_Can_Set_Guide_Rates:
		case  Telescope_Can_Set_Declination_Rate:
		case  Telescope_Can_Pulse_Guide:
		case  Telescope_Declination_Rate:
		case  Telescope_Does_Refraction:
		case  Telescope_Equatorial_System:
		case  Telescope_Focal_Length:
		case  Telescope_Tracking_Rate:
		case  Telescope_Tracking:
		case  Telescope_Target_Right_Ascension:
		case  Telescope_Target_Declination:
		case  Telescope_Slew_Settle_Time:
		case  Telescope_Slewing:
		case  Telescope_Site_Longitude:
		case  Telescope_Can_Park:
		case  Telescope_Site_Latitude:
		case  Telescope_Sidereal_Time:
		case  Telescope_Side_of_Pier:
		case  Telescope_Right_Ascension_Rate:
		case  Telescope_Right_Ascension:
		case  Telescope_Is_Pulse_Guiding:
		case  Telescope_Guide_Rate_Right_Ascension:
		case  Telescope_Guide_Rate_Declination:
		case  Telescope_Site_Elevation:
		case  Telescope_Can_Find_Home:
		case  Telescope_UTCDATE:
		case  Telescope_At_Park:
		case  Telescope_At_Home:
		case  Telescope_Aperture_Diameter:
		case  Telescope_Aperture_Area:
		case  Telescope_Altitude:
		case  Telescope_Alignment_Mode:
		case  Telescope_Interface_Version:
		case  Telescope_Driver_Version:
		case  Telescope_Connected:
		case  Telescope_Tracking_Rates:
		case  Telescope_Abort_Slew:
		case  Telescope_Action:
		case  Telescope_Axis_Rates:
		case  Telescope_Can_Move_Axis:
		case  Telescope_Command_Blind:
		case  Telescope_Command:
		case  Telescope_Destination_Side_of_Pier:
		case  Telescope_Dispose:
		case  Telescope_Find_Home:
		case  Telescope_Move_Axis:
		case  Telescope_Park:
		case  Telescope_Pulse_Guide:
		case  Telescope_Set_Park:
		case  Telescope_Setup_Dialog:
		case  Telescope_Slew_to_AltAz:
		case  Telescope_Slew_to_AltAz_Async:
		case  Telescope_Slew_to_Coordinates:
		case  Telescope_Slew_to_Coordinates_Async:
		case  Telescope_Slew_to_Target:
		case  Telescope_Slew_to_Target_Async:
		case  Telescope_Sync_to_AltAz:
		case  Telescope_Sync_to_Coordinates:
		case  Telescope_Sync_to_Target:
		case  Telescope_Unpark:
			Prepare_Packet_for_Output((unsigned char) motor,(unsigned char)command_number,(double)parameter_one,(double)parameter_two);
			Send_Packet_to_Controller();
			break;
	}
}
void Process_Incoming_Packet_from_Focuser() {                                       // process an update message from the Focuser
	int command_number = 0;
	double parameter_one = 0;
	double parameter_two = 0;
	Flash_Orange_Light();
	Focuser_Incoming_Message_Available = false;
	write_logger((unsigned char)NULL,(unsigned char)FROM, (unsigned char)SOURCE_FOCUSER, (unsigned char)command_number, parameter_one, parameter_two);
#ifdef DEBUG_FOCUSER
	Serial.print(millis(), DEC);
	Serial.println("\tPacket Received from Focuser");
#endif
	command_number = (int)Incoming_Message_from_Focuser.contents.command_number;
	parameter_one = Incoming_Message_from_Focuser.contents.parameter_one;
	parameter_two = Incoming_Message_from_Focuser.contents.parameter_two;
#ifdef DEBUG_FOCUSER_COMMANDS
	Serial.print(millis(), DEC);
	Serial.print("\tFocuser Command Reply : ");
	Serial.print(command_number, DEC);
	Serial.print(" Parameter_one : ");
	Serial.print(parameter_one, DEC);
	Serial.print(" Parameter_two : ");
	Serial.print(parameter_two, DEC);
#endif
	switch (command_number) {
		case Focuser_Current_Focuser_Position:
		case Focuser_Motor_Moving_Status:
		case Focuser_Motor_Controller_Status:
		case Focuser_Firmware_Version_String:
		case Focuser_Firmware_Name_and_Version_String:
		case Focuser_Temperature:
		case Focuser_Maximum_Step:								// parameter_one == 0 GET, else SET
		case Focuser_Maximum_Increment:
		case Focuser_Coil_Power:									// parameter_one == 0 GET, else SET
		case Focuser_Reverse_Direction:
		case Focuser_Motor_Speed:
		case Focuser_Display_Unit:
		case Focuser_User_Specified_Step_Size:
		case Focuser_Step_Size:
		case Focuser_Temperature_Coefficient:
		case Focuser_Temperature_Compensation:
		case Focuser_Temperature_Compensation_Available:
		case Focuser_Home_Motor_Position_To_ZERO:
		case Focuser_Step_Mode:
		case Focuser_Step_Size_is_Enabled:
		case Focuser_Display:
		case Focuser_Temperature_Mode:
		case Focuser_Reset_Arduino_Controller:
		case Focuser_Reset_Focuser_Defaults:
		case Focuser_Motor_Speed_Threshold_When_Moving:
		case Focuser_Motor_Speed_Change_When_Moving:
		case Focuser_Save_Settings_to_EEPROM:
		case Focuser_Humidity:
		case Focuser_Longitude:
		case Focuser_Latitude:
		case Focuser_Altitude:
		case Focuser_BEMF:
		case Focuser_Update_of_Position_When_Moving:
		case Focuser_Status_of_Home_Position_Switch:
		case Focuser_Jogging_Steps:
		case Focuser_Jogging_State:
		case Focuser_Jogging_Direction:
		case Focuser_Delay_After_Move:
		case Focuser_Backlash_In:
		case Focuser_Backlash_Out:
		case Focuser_Number_of_Backlash_Steps_In:
		case Focuser_Number_of_Backlash_Steps_Out:
		case Focuser_Temperature_Compensation_Direction:
		case Focuser_to_Controller_Heartbeat:
			Prepare_Packet_for_Output((unsigned char)SOURCE_FOCUSER, (unsigned char)command_number, (double)parameter_one, (double)parameter_two);
			Send_Packet_to_Controller();
			break;
	}
}
void Process_Incoming_Packet_from_Controller() {
	int command_number = 0;
	double parameter_one = 0;
	double parameter_two = 0;
	Flash_Red_Light();
#ifdef DEBUG_CONTROLLER
	Serial.print(millis(), DEC);
	Serial.println("\tPacket Received from Controller");
#endif
	command_number = (int)Incoming_Message_from_Controller.contents.command_number;
	parameter_one = Incoming_Message_from_Controller.contents.parameter_one;
	parameter_two = Incoming_Message_from_Controller.contents.parameter_two;
	switch (Incoming_Message_from_Controller.contents.command_number) {
	case HUB_Get_Day_Month: {
		system_day = (long) parameter_one;
		system_month = (long) parameter_two;
		date_required = (int) stage_year_hour;
		break;
	}
	case HUB_Get_Year_Hour: {
		system_year = (long) parameter_one;
		system_hour = (long) parameter_two;
		date_required = (int) stage_minute_second;
		break;
	}
	case HUB_Get_Minute_Second: {
		system_minute = (long) parameter_one;
		system_second = (long) parameter_two;
		date_required = (int) stage_wait_complete;
		break;
	}
	case HUB_Delete_Log_File: {
		write_logger((unsigned char)HUB_Delete_Log_File, (unsigned char)NULL, (unsigned char)SOURCE_CONTROLLER, (unsigned char)HUB_Delete_Log_File, (double)NULL, (double)NULL);
		break;
	}
		case Focuser_Current_Focuser_Position:
		case Focuser_Motor_Moving_Status:
		case Focuser_Motor_Controller_Status:
		case Focuser_Target_Position:
		case Focuser_Temperature:			
		case Focuser_Maximum_Step:				
		case Focuser_Maximum_Increment:			
		case Focuser_Coil_Power:		
		case Focuser_Reverse_Direction:
		case Focuser_Motor_Speed:								
		case Focuser_Display_Unit:											
		case Focuser_User_Specified_Step_Size:				
		case Focuser_Step_Size:						
		case Focuser_Temperature_Coefficient:				
		case Focuser_Temperature_Compensation:					
		case Focuser_Temperature_Compensation_Available:	
		case Focuser_Home_Motor_Position_To_ZERO:			
		case Focuser_Step_Mode:								
		case Focuser_Step_Size_is_Enabled:										
		case Focuser_Temperature_Mode:						
		case Focuser_Reset_Arduino_Controller:
		case Focuser_Reset_Focuser_Defaults:
		case Focuser_Motor_Speed_Threshold_When_Moving:
		case Focuser_Motor_Speed_Change_When_Moving:			
		case Focuser_Save_Settings_to_EEPROM:			
		case Focuser_Humidity:
		case Focuser_Longitude:
		case Focuser_Latitude:
		case Focuser_Altitude:		
		case Focuser_BEMF:
		case Focuser_Update_of_Position_When_Moving:
		case Focuser_Status_of_Home_Position_Switch:
		case Focuser_Jogging_Steps:
		case Focuser_Jogging_State:
		case Focuser_Jogging_Direction:
		case Focuser_Delay_After_Move:
		case Focuser_Backlash_In:
		case Focuser_Backlash_Out:
		case Focuser_Number_of_Backlash_Steps_In:
		case Focuser_Number_of_Backlash_Steps_Out:
		case Focuser_Temperature_Compensation_Direction:
			Prepare_Packet_for_Output((unsigned char)SOURCE_CONTROLLER, (unsigned char)command_number, (double)parameter_one, (double)parameter_two);
			Send_Packet_to_Focuser();
			break;
		case HUB_Display:
			hub_display_state = (bool) Incoming_Message_from_Controller.contents.parameter_one;
			break;
		case Telescope_UTCDATE:
		case Telescope_Azimuth:
		case Telescope_Declination:
		case Telescope_Can_Unpark:
		case Telescope_Can_Sync_AltAz:
		case Telescope_Can_Sync:
		case Telescope_Can_Slew_Async:
		case Telescope_Can_Slew_AltAz_Async:
		case Telescope_Can_Slew_AltAz:
		case Telescope_Can_Slew:
		case Telescope_Can_Set_Tracking:
		case Telescope_Can_Set_Right_Ascension_Rate:
		case Telescope_Can_Set_Pier_Side:
		case Telescope_Can_Set_Park:
		case Telescope_Can_Set_Guide_Rates:
		case Telescope_Can_Set_Declination_Rate:
		case Telescope_Can_Pulse_Guide:
		case Telescope_Declination_Rate:
		case Telescope_Does_Refraction:
		case Telescope_Equatorial_System:
		case Telescope_Focal_Length:
		case Telescope_Tracking_Rate:
		case Telescope_Tracking:
		case Telescope_Target_Right_Ascension:
		case Telescope_Target_Declination:
		case Telescope_Slew_Settle_Time:
		case Telescope_Slewing:
		case Telescope_Site_Longitude:
		case Telescope_Can_Park:
		case Telescope_Site_Latitude:
		case Telescope_Sidereal_Time:
		case Telescope_Side_of_Pier:
		case Telescope_Right_Ascension_Rate:
		case Telescope_Right_Ascension:
		case Telescope_Is_Pulse_Guiding:
		case Telescope_Guide_Rate_Right_Ascension:
		case Telescope_Guide_Rate_Declination:
		case Telescope_Site_Elevation:
		case Telescope_Can_Find_Home:
		case Telescope_At_Park:
		case Telescope_At_Home:
		case Telescope_Aperture_Diameter:
		case Telescope_Aperture_Area:
		case Telescope_Altitude:
		case Telescope_Alignment_Mode:
		case Telescope_Interface_Version:
		case Telescope_Driver_Version:
		case Telescope_Connected:
		case Telescope_Tracking_Rates:	
		case Telescope_Action:
		case Telescope_Axis_Rates:
		case Telescope_Can_Move_Axis:
		case Telescope_Command_Blind:
		case Telescope_Command:
		case Telescope_Destination_Side_of_Pier:
		case Telescope_Dispose:
		case Telescope_Find_Home:
		case Telescope_Move_Axis:
		case Telescope_Park:
		case Telescope_Pulse_Guide:
		case Telescope_Set_Park:
		case Telescope_Setup_Dialog:
		case Telescope_Slew_to_AltAz:
		case Telescope_Slew_to_AltAz_Async:
		case Telescope_Slew_to_Coordinates:
		case Telescope_Slew_to_Coordinates_Async:
		case Telescope_Slew_to_Target:
		case Telescope_Slew_to_Target_Async:
		case Telescope_Sync_to_AltAz:
		case Telescope_Sync_to_Coordinates:
		case Telescope_Sync_to_Target:
		case Telescope_Unpark: {
			Prepare_Packet_for_Output((unsigned char)SOURCE_CONTROLLER, (unsigned char)command_number, (double)parameter_one, (double)parameter_two);;
			Send_Packet_to_Motor_Drivers((unsigned char)SOURCE_BOTH_MOTORS);
			break;
		}
	}
	write_logger((unsigned char)NULL, (unsigned char)FROM, (unsigned char)SOURCE_CONTROLLER, (unsigned char)command_number, parameter_one, parameter_two);
}
void Prepare_Packet_for_Output(byte source, byte command_number, double parameter_one, double parameter_two) {
	Outgoing_Message_to_Controller.contents.header = STX;
	Outgoing_Message_to_Controller.contents.source = source;
	Outgoing_Message_to_Controller.contents.command_number = (byte) command_number;
	Outgoing_Message_to_Controller.contents.parameter_one = parameter_one;
	Outgoing_Message_to_Controller.contents.parameter_two = parameter_two;
	Outgoing_Message_to_Controller.contents.footer = ETX;
}
void Send_Packet_to_Motor_Drivers(unsigned char motors) {
	unsigned char command_number = Outgoing_Message_to_Motor_Drivers.contents.command_number;
	double parameter_one = Outgoing_Message_to_Motor_Drivers.contents.parameter_one;
	double parameter_two = Outgoing_Message_to_Motor_Drivers.contents.parameter_two;
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
		if (motors == (unsigned char)SOURCE_ALTITUDE_MOTOR) {
			write_logger((unsigned char)NULL,(unsigned char)TO, (unsigned char)SOURCE_ALTITUDE_MOTOR, command_number, parameter_one, parameter_two);
			altitude_serial.write(Outgoing_Message_to_Motor_Drivers.sg_chars[i]);
			altitude_count++;
			if (altitude_count > 99) altitude_count = 1;
		}
		if (motors == (unsigned char)SOURCE_AZIMUTH_MOTOR) {
			write_logger((unsigned char)NULL,(unsigned char)TO, (unsigned char)SOURCE_AZIMUTH_MOTOR, command_number, parameter_one, parameter_two);
			azimuth_serial.write(Outgoing_Message_to_Motor_Drivers.sg_chars[i]);
			azimuth_count++;
			if (azimuth_count > 99) azimuth_count = 1;
		}
		if (motors == (unsigned char)SOURCE_BOTH_MOTORS) {
			write_logger((unsigned char)NULL,(unsigned char)TO, (unsigned char)SOURCE_BOTH_MOTORS, command_number, parameter_one, parameter_two);
			altitude_serial.write(Outgoing_Message_to_Motor_Drivers.sg_chars[i]);
			azimuth_serial.write(Outgoing_Message_to_Motor_Drivers.sg_chars[i]);
			altitude_count++;
			azimuth_count++;
			if (altitude_count > 99) altitude_count = 1;
			if (azimuth_count > 99) azimuth_count = 1;
		}
	}
#ifdef DEBUG_OUTPUT_MOTOR_DRIVERS
	Serial.println("");
#endif
}
void Send_Packet_to_Focuser() {
	unsigned char command_number = Outgoing_Message_to_Focuser.contents.command_number;
	double parameter_one = Outgoing_Message_to_Focuser.contents.parameter_one;
	double parameter_two = Outgoing_Message_to_Focuser.contents.parameter_two;
	write_logger((unsigned char) 0,(unsigned char)TO, (unsigned char)SOURCE_FOCUSER, command_number, parameter_one, parameter_two);
	focuser_count++;
	if (focuser_count > 99) focuser_count = 1;
#ifdef DEBUG_OUTPUT_FOCUSER
	Serial.print(millis(), DEC);
	Serial.print("\tFocuser Message to be sent: ");
	for (int i = 0; i < Message_Structure_Length; i++) {
		Serial.print(Outgoing_Message_to_Focuser.sg_chars[i], HEX);
		Serial.print(",");
	}
	Serial.println();
	Serial.print("Message Sent to Focuser:");
#endif
	for (int i = 0; i < Message_Structure_Length; i++) {
		focuser_serial.write(Outgoing_Message_to_Focuser.sg_chars[i]);
	}
#ifdef DEBUG_OUTPUT_FOCUSER
	Serial.println("");
#endif
}
void Send_Packet_to_Controller() {
	unsigned char command_number = Outgoing_Message_to_Controller.contents.command_number;
	double parameter_one = Outgoing_Message_to_Controller.contents.parameter_one;
	double parameter_two = Outgoing_Message_to_Controller.contents.parameter_two;
	write_logger((unsigned char)NULL,(unsigned char)TO, (unsigned char)SOURCE_CONTROLLER, command_number, parameter_one, parameter_two);
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
#ifdef ETHERNET
		server.write(Outgoing_Message_to_Controller.sg_chars[i]);
#endif
	}
#ifdef DEBUG_OUTPUT_CONTROLLER
	Serial.println("");
#endif
}
void Flash_Red_Light(void) {
  if (millis() - red_time > (unsigned long)debounce) {
    reading = digitalRead(RED_LED_pin);
    if (reading == HIGH) {
      digitalWrite(RED_LED_pin, LOW);
    }
    else {
      digitalWrite(RED_LED_pin, HIGH);
    }
    red_time = millis();
  }
}
void Flash_Green_Light(void) {
	if (millis() - green_time > (unsigned long)debounce) {
		reading = digitalRead(GREEN_LED_pin);
		if (reading == HIGH) {
			digitalWrite(GREEN_LED_pin, LOW);
		}
		else {
			digitalWrite(GREEN_LED_pin, HIGH);
		}
		green_time = millis();
	}
}
void Flash_Orange_Light(void) {
	if (millis() - blue_time > (unsigned long)debounce) {
		reading = digitalRead(ORANGE_LED_pin);
		if (reading == HIGH) {
			digitalWrite(ORANGE_LED_pin, LOW);
		}
		else {
			digitalWrite(ORANGE_LED_pin, HIGH);
		}
		blue_time = millis();
	}
}
// send update to SEVEN_SEGMENT ---------------------------------------------------------------------------------------
void send_update_to_SEVEN_SEGMENT() {
	char displayaltitude_s[2];
	char displayazimuth_s[2];
	char displayfocuser_s[2];
	String Display_String = "        ";
	itoa((int)altitude_count, displayaltitude_s, 10);
	Display_String = displayaltitude_s;
	Display_String = Display_String + '-';
	itoa((int)azimuth_count, displayazimuth_s, 10);
	Display_String = Display_String + displayazimuth_s;
	Display_String = Display_String + '-';
	itoa((int)focuser_count, displayfocuser_s, 10);
	Display_String = Display_String + displayfocuser_s;
	display(Display_String);
}
void error(double error_number) {
	char error_number_s[2];
	dtostrf(error_number, 2, 0, error_number_s);
	sprintf(display_string, "%s", error_number_s);
	Serial.print("Error Line:");
	Serial.println(display_string);
	error_display(display_string);
}
void set_register(byte reg, byte value) {   // ... write a value into a max729 register See MAX729 Datasheet, Table , page 6
	digitalWrite(display_CS, LOW);
	shiftOut(display_DIN, display_CLK, MSBFIRST, reg);
	shiftOut(display_DIN, display_CLK, MSBFIRST, value);
	digitalWrite(display_CS, HIGH);
}
void display(String thisString) { // ... display on the 7-segment display
	set_register(display_REG_SHUTDOWN, OFF);  // turn off display
	set_register(display_REG_SCANLIMIT, 7);   // scan limit 8 digits
	set_register(display_REG_DECODE, 0b11011011);		// 
	if ((thisString.charAt(0) < 0x30) || (thisString.charAt(0) > 0x39)) thisString.setCharAt(0, 0x30);
	if ((thisString.charAt(1) < 0x30) || (thisString.charAt(1) > 0x39)) thisString.setCharAt(1, 0x30);
//	if ((thisString.charAt(2) < 0x30) || (thisString.charAt(2) > 0x39)) thisString.setCharAt(2, 0x30);
	if ((thisString.charAt(3) < 0x30) || (thisString.charAt(3) > 0x39)) thisString.setCharAt(3, 0x30);
	if ((thisString.charAt(4) < 0x30) || (thisString.charAt(4) > 0x39)) thisString.setCharAt(4, 0x30);
//	if ((thisString.charAt(5) < 0x30) || (thisString.charAt(5) > 0x39)) thisString.setCharAt(5, 0x30);
	if ((thisString.charAt(6) < 0x30) || (thisString.charAt(6) > 0x39)) thisString.setCharAt(6, 0x30);
	if ((thisString.charAt(7) < 0x30) || (thisString.charAt(7) > 0x39)) thisString.setCharAt(7, 0x30);
	set_register(1, thisString.charAt(7));
	set_register(2, thisString.charAt(6));
	set_register(3, dash);	// 6
	set_register(4, thisString.charAt(4));	// 5
	set_register(5, thisString.charAt(3));  // 4
	set_register(6, dash);	// 3
	set_register(7, thisString.charAt(1));  // 2
	set_register(8, thisString.charAt(0));  // 1
	set_register(display_REG_SHUTDOWN, ON);   // Turn on display
}
void error_display(String thisString) { // ... display on the 7-segment display, thisString is the error number
	set_register(display_REG_SHUTDOWN, OFF);  // turn off display
	set_register(display_REG_SCANLIMIT, 7);   // scan limit 8 digits
	set_register(display_REG_DECODE, 0b000000);		// 
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