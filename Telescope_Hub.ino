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
#include <aREST.h>
#include "HUB_Commands.h"
#include "Arest_Variables_Functions.h"

#define NUMBER_VARIABLES 40						// maximum number of variables that can be exposed to rest
#define NUMBER_FUNCTIONS 40						// maximum number of functions that can be exposed to rest
#define DEBUG_OUTPUT
//#define DEBUG_OUTPUT_ALTITUDE
//#define DEBUG_OUTPUT_AZIMUTH
//#define DEBUG_OUTPUT_FOCUSER
//#define DEBUG_OUTPUT_COMMANDS
//#define DEBUG_OUTPUT_FIELDS
//#define DEBUG_COMMAND
//#define DEBUG_ALTITUDE						// enable monitor output
//#define DEBUG_AZIMUTH
//#define DEBUG_FOCUSER
#define WDT									// enable WatchDog Timer
#define ETHERNET
#define REST									// include REST functionality

#ifdef WDT
	#include <avr/wdt.h>
#endif
// Definitions ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
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
#define Altitude_motor	0
#define Azimuth_motor	1
#define Box_led 11				// pin number of Box LED
#define HIGH 1
#define LOW 0
#define North 0
#define South 1
#define East 2
#define West 3
#define CW 0
#define ACW 1

#define OUTWARDS	0
#define INWARDS		1

#define focuser_temperature_valid	0x01
#define focuser_humidity_valid		0x02
#define focuser_date_valid			0x04
#define focuser_time_valid			0x08
#define focuser_longitude_valid		0x10
#define focuser_latitude_valid		0x20
#define focuser_altitude_valid		0x40
#define focuser_at_home				0x80

#define display_DIN			11		// MAX7219 DATA pin		D11 green
#define display_CS			10		// MAX7219 CS/LOAD pin	D10 blue
#define display_CLK			12		// MAX7219 CLK pin		D12 yellow
#define RUNNING_LED_pin		49		// 49 = Green
#define HUB_LED_pin			48		// 48 = Blue
#define GPS_LED_pin			46		// 46 = Red
#define LED_FLASH_TIME		20           // number of milli seconds to light the LEDs

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
#ifdef REST
aREST rest = aREST();
#endif
// Stored values obtained from or about the motor controllers, to be exposed to the API
float Altitude_value;					// v1
int AtHome_value;						// v2
int AtPark_value;						// v3
float Azimuth_value;					// v4
int Slewing_value;						// v5
float TargetAltitude_value;				// v6
float TargetAzimuth_value;				// v7
int Tracking_value;						// v8
float TrackingRateAzimuth_value;		// v9
float TrackingRateAltitude_value;		// v10
int Hub_display_state = 1;				// v11
int Focuser_display_state = 1;			// v12
int Focuser_Current_Position_value;		// f1
float Focuser_Current_Temperature_value;// f2
float Focuser_Current_Humidity_value;	// f3
float Focuser_Latitude_value;			// f4
float Focuser_Longitude_value;			// f5
float Focuser_Altitude_value;			// f6
int Focuser_Year_value;					// f7
int Focuser_Month_value;				// f8
int Focuser_Day_value;					// f9
int Focuser_Hour_value;					// f10
int Focuser_Minute_value;				// f11
int Focuser_Second_value;				// f12

//-- Working Variables --------------------------------------------------------------------------------------------------------------------
int state = HIGH;      // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin
long time = 0;         // the last time the output pin was toggled
long debounce = 300;   // the debounce time, increase if the output flickers

int al_AtHome_value;
int al_AtPark_value;
int al_Slewing_value;
int al_Running_value;
int al_Homing_value;
int al_Tracking_value;
int al_PulseGuiding_value;
double al_Altitude_value = 0;
double al_Azimuth_value = 0;
double al_Field3_value = 0;
double al_Field4_value = 0;
int az_AtHome_value;
int az_AtPark_value;
int az_Slewing_value;
int az_Running_value;
int az_Homing_value;
int az_Tracking_value;
int az_PulseGuiding_value;
double az_Azimuth_value = 0;
double az_Altitude_value = 0;
double az_Field3_value = 0;
double az_Field4_value = 0;
int hub_display_state = 1;

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
  unsigned char command_number;     // [1] Command Number
  unsigned char motor_status;       // [2]
  double azimuth_bearing;			// [3 - 6]
  double altitude_bearing;			// [7 - 10]
  double field3;					// [11 - 14]
  double field4;					// [15 - 18]
  unsigned char footer;             // [19] ETX
} Incoming_Message_Structure;
#define Incoming_Message_Structure_Length 20
union incoming_message_from_altitude {
  Incoming_Message_Structure message;
  unsigned char sg_chars[Incoming_Message_Structure_Length];
};
union incoming_message_from_altitude Incoming_Message_from_Altitude;
union incoming_message_from_azimuth {
  Incoming_Message_Structure message;
  unsigned char sg_chars[Incoming_Message_Structure_Length];
};
union incoming_message_from_azimuth Incoming_Message_from_Azimuth;

typedef struct {
  unsigned char header;             // [0] STX
  unsigned char command_number;     // [1] Command Number
  unsigned char motor_status;       // [2]
  double parameter_one;				// [3 - 6]
  double parameter_two;           	// [7 - 10]
  unsigned char footer;             // [11] ETX
} Outgoing_Message_Structure;  
#define Outgoing_Message_Structure_Length 12
union outgoing_message {
    Outgoing_Message_Structure message;
    unsigned char sg_chars[Outgoing_Message_Structure_Length];
};
union outgoing_message Outgoing_Message;

bool Altitude_Incoming_Message_Available = false;
bool Azimuth_Incoming_Message_Available = false;
bool Focuser_Incoming_Message_Available = false;
// Message structure for messages TO the focuser -------------------------------------------------------------------------
typedef struct {
	unsigned char header;					// [0] STX
	unsigned char focuser_command_number;	// [1] Command Number
	unsigned int focuser_target;			// [2 - 3] absolute position, or increment(outwards)/decrement (inwards) (in microsteps)
	unsigned int focuser_direction;			// [4 - 5] outwards = 0, inwards = 1 
	unsigned char footer;					// [6] ETX
}Focuser_Outgoing_Message_Structure;
#define Focuser_Outgoing_Message_Structure_Length 7
union Focuser_outgoing_message {
	Focuser_Outgoing_Message_Structure focuser_message;
	unsigned char sg_chars[Focuser_Outgoing_Message_Structure_Length];
};
union Focuser_outgoing_message Focuser_Outgoing_Message;

// Message structure for messages FROM the focuser -----------------------------------------------------------------------
typedef struct {
	unsigned char header;			// [0] STX
	unsigned char command_number;	// [1] Command Number
	unsigned char focuser_status;	// [2]
	unsigned int focuser_position;	// [3 - 4]
	float temperature;				// [5 - 8]
	float humidity;					// [9 - 12]
	unsigned char year;				// [13] year
	unsigned char month;			// [14] month
	unsigned char day;				// [15] day
	unsigned char hour;				// [16] hour
	unsigned char minute;			// [17] minute
	unsigned char second;			// [18] second
	float latitude;					// [19,22] latitude
	float longitude;				// [23,26] longitude
	float altitude;					// [27,30] altitude (above mean sea level
	unsigned char footer;			// [31] ETX
} focuser_incoming_message_structure;
#define focuser_incoming_message_length 32
union focuser_incoming_message {
	focuser_incoming_message_structure focuser_incoming_message;
	unsigned char sg_chars[focuser_incoming_message_length];
};
focuser_incoming_message incoming_focuser_message;

double command_fieldf[4];
union sg_double_union command_fieldfc;
int COUNT = 0;
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
    Ethernet.begin(mac);
  }
  server.begin();
#ifdef DEBUG_OUTPUT
  Serial.print(millis(), DEC);
  Serial.print("\t");
  Serial.print("Ethernet Server Setup Complete, at ");
  Serial.println(Ethernet.localIP());
#endif
#endif // end ETHERNET
  pinMode(Box_led, OUTPUT);
#ifdef DEBUG_OUTPUT
  Serial.print(millis(), DEC);
  Serial.print("\t");
  Serial.println("rest/ASCOM setup complete");
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
  digitalWrite(GPS_LED_pin, LOW);
  pinMode(GPS_LED_pin, OUTPUT);
  digitalWrite(RUNNING_LED_pin, LOW);
  pinMode(RUNNING_LED_pin, OUTPUT);
  digitalWrite(HUB_LED_pin, LOW);
  pinMode(HUB_LED_pin, OUTPUT);
  pinMode(display_DIN, OUTPUT);
  pinMode(display_CS, OUTPUT);
  pinMode(display_CLK, OUTPUT);
  resetDisplay();							// reset the MAX7219 display
  send_update_to_SEVEN_SEGMENT();
} // end setup
// Main -----------------------------------------------------------------------------------------------------------------------------------
void loop() {
#ifdef WDT
  wdt_reset();													// reset the watchdog timer
#endif
#ifdef ETHERNET
  EthernetClient client = server.available();                  // Listen for incoming client requests.
#endif
#ifdef REST
  rest.handle(client);
#endif
	if (millis() > (last_status_update + 1000)) {					// send an update to the HUB every second
		if (hub_display_state == 1) {							// flash the running led
			running_status = true;
			running_time_LED_On = millis();
			digitalWrite(RUNNING_LED_pin, HIGH);
		}
		last_status_update = millis();
	}
	if (Check_Altitude_Message() == true) Parse_Altitude_Incoming_Message();
	if (Check_Azimuth_Message() == true) Parse_Azimuth_Incoming_Message();
	if (Check_Focuser_Message() == true) Parse_Focuser_Incoming_Message();
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
//      Serial.println("Sending Tracking ");        // c4 last tested 30/04/2018
//      Tracking_f("1.0,0.0,0,0.0");
//      Serial.println("Setting Tracking Rates");   // c5 last tested 30/04/2018
//      TrackingRates_f("2650.0,3000.0,0.0,0.0");
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

// Custom functions accessible by the API ------------------------------------------------------------
int TargetAltitude_f(String command) {	// f2
  Parse_Command_to_Fieldf(command);
#ifdef DEBUG_OUTPUT_COMMANDS
  Serial.print(millis(), DEC);
  Serial.print("\tSending TargetAltitude, Altitude=");
  Serial.println(command_fieldf[1], DEC);
#endif
  Send_Message_to_Driver(Altitude_motor, TargetAltitude_command, command_fieldf[1], 0);
  return (int)true;
}
int TargetAzimuth_f(String command) { // f3
  Parse_Command_to_Fieldf(command);
#ifdef DEBUG_OUTPUT_COMMANDS
  Serial.print(millis(), DEC);
  Serial.print("\tSending TargetAzimuth, Azimuth=");
  Serial.println(command_fieldf[1], DEC);
#endif
  Send_Message_to_Driver(Azimuth_motor, TargetAzimuth_command, command_fieldf[1], 0);
  return (int)true;
}
int Tracking_f(String command) { // f4
  Parse_Command_to_Fieldf(command);
#ifdef DEBUG_OUTPUT_COMMANDS
  Serial.print(millis(), DEC);
  Serial.print("\tSetting Tracking ");
#endif
  Send_Message_to_Driver(Altitude_motor, Tracking_command, command_fieldf[1], 0);    // send message to driver
  Send_Message_to_Driver(Azimuth_motor, Tracking_command, command_fieldf[1], 0);    // send message to driver
  return (int) true;
}
int TrackingRates_f(String command) { // f5
  Parse_Command_to_Fieldf(command);
#ifdef DEBUG_OUTPUT_COMMANDS
  Serial.print(millis(), DEC);
  Serial.print("\tSetting TrackingRates Azimuth ");
  Serial.print(command_fieldf[1], DEC);
  Serial.print(", Altitude ");
  Serial.println(command_fieldf[2], DEC);
#endif
  Send_Message_to_Driver(Azimuth_motor, TrackingRates_command, command_fieldf[1], 0);  // send message to driver
  Send_Message_to_Driver(Altitude_motor, TrackingRates_command, command_fieldf[2], 0);
  return (int)true;
}
int AbortSlew_f(String command) { // f6
#ifdef DEBUG_OUTPUT_COMMANDS
  Serial.print(millis(), DEC);
  Serial.println("\tSending AbortSlew");
#endif
  Send_Message_to_Driver(Altitude_motor, AbortSlew_command,  0, 0);      // send message to drivers
  Send_Message_to_Driver(Azimuth_motor, AbortSlew_command,  0, 0);
  return (int)true;
}
int FindHome_f(String command) { // f6
#ifdef DEBUG_OUTPUT_COMMANDS
  Serial.print(millis(), DEC);
  Serial.println("\tSending FindHome ");
#endif
  Send_Message_to_Driver(Altitude_motor, FindHome_command, 0, 0);      // send message to drivers
  Send_Message_to_Driver(Azimuth_motor, FindHome_command, 0, 0);
  return (int)true;
}
int Park_f(String command) { // f7
#ifdef DEBUG_OUTPUT_COMMANDS
  Serial.print(millis(), DEC);
  Serial.println("\tSending Park");
#endif
  Send_Message_to_Driver(Altitude_motor, Park_command, 0, 0);      // send message to drivers
  Send_Message_to_Driver(Azimuth_motor, Park_command, 0, 0);      // send message to drivers
  return (int)true;
}
int SlewToAltAz_f(String command) { // f8
  Parse_Command_to_Fieldf(command);                                   // parse the command string into double command_fieldf
#ifdef DEBUG_OUTPUT_COMMANDS
  Serial.print(millis(), DEC);
  Serial.print("\tSending SlewToAltAz - Azimuth ");
  Serial.print(command_fieldf[1], DEC);
  Serial.print(", Altitude ");
  Serial.println(command_fieldf[2], DEC);
#endif
  Send_Message_to_Driver(Azimuth_motor, SlewToAltAz_command, command_fieldf[1], 0);      // send message to drivers
  Send_Message_to_Driver(Altitude_motor, SlewToAltAz_command, command_fieldf[2], 0);
  return (int)true;
}
int SlewToTarget_f(String command) { // f12
#ifdef DEBUG_OUTPUT_COMMANDS
  Serial.print(millis(), DEC);
  Serial.println("\tSending SlewToTarget");
#endif
  Send_Message_to_Driver(Azimuth_motor, SlewToTarget_command, 0, 0);      // send message to drivers
  Send_Message_to_Driver(Altitude_motor, SlewToTarget_command, 0, 0);
  return (int)true;
}
int SyncToAltAz_f(String command) { // f14 This command tells the mount what its ALT/AZ should be
  Parse_Command_to_Fieldf(command);
#ifdef DEBUG_OUTPUT_COMMANDS
  Serial.print(millis(), DEC);
  Serial.print("\tSyncToAltAz - Azimuth ");
  Serial.print(command_fieldf[1], DEC);
  Serial.print(", Altitude ");
  Serial.println(command_fieldf[2], DEC);
#endif
  Send_Message_to_Driver(Azimuth_motor, SyncToAltAz_command, command_fieldf[1], 0);      // send message to drivers
  Send_Message_to_Driver(Altitude_motor, SyncToAltAz_command, command_fieldf[2], 0);
  return (int)true;
}
int PulseGuide_f(String command) { // f12}
  Parse_Command_to_Fieldf(command);
#ifdef DEBUG_OUTPUT_COMMANDS
  Serial.print(millis(), DEC);
  Serial.print("\tPulseGuide - Direction ");
  Serial.print(command_fieldf[1], DEC);
  Serial.print(", Duration (ms) ");
  Serial.println(command_fieldf[2], DEC);
#endif
  switch ((int) command_fieldf[1]) {
    case North:
      Send_Message_to_Driver(Altitude_motor, PulseGuide_command, command_fieldf[1], command_fieldf[2]);      // send message to driver, direction, duration
      break;
    case South:
      Send_Message_to_Driver(Altitude_motor, PulseGuide_command, command_fieldf[1], command_fieldf[2]);      // send message to drivers
      break;
    case East:
      Send_Message_to_Driver(Azimuth_motor, PulseGuide_command, command_fieldf[1], command_fieldf[2]);      // send message to drivers
      break;
    case West:
      Send_Message_to_Driver(Azimuth_motor, PulseGuide_command, command_fieldf[1], command_fieldf[2]);      // send message to drivers
      break;
  }
}
int GuideRateRightAscension_f(String command) {  // f13
  Parse_Command_to_Fieldf(command);
  #ifdef DEBUG_OUTPUT_COMMANDS
	Serial.print(millis(), DEC);
	Serial.print("\tGuideRateRightAscension ");
	Serial.println(command_fieldf[1], DEC);
#endif
	Send_Message_to_Driver(Azimuth_motor, GuideRateRightAscension_command, command_fieldf[1], 0);     // send message to drivers
}
int GuideRateDeclination_f(String command) { // f14
  Parse_Command_to_Fieldf(command);
  #ifdef DEBUG_OUTPUT_COMMANDS
	Serial.print(millis(), DEC);
	Serial.print("\tGuideRateDeclination ");
	Serial.println(command_fieldf[1], DEC);
#endif
	Send_Message_to_Driver(Altitude_motor, GuideRateDeclination_command, command_fieldf[1], 0);     // send message to drivers
}
int UnPark_f(String command) { // f15
#ifdef DEBUG_OUTPUT_COMMANDS
  Serial.print(millis(), DEC);
  Serial.println("\tUnPark Sent");
#endif
  Send_Message_to_Driver(Azimuth_motor, UnPark_command, 0, 0);      // send message to drivers
  Send_Message_to_Driver(Altitude_motor, UnPark_command, 0, 0);
}
int Hub_Display_On_f(String command) {
#ifdef DEBUG_OUTPUT_COMMANDS
	Serial.print(millis(), DEC);
	Serial.println("\tHub Display Turned On");
#endif
	hub_display_state = 1;
}
int Hub_Display_Off_f(String command) {
#ifdef DEBUG_OUTPUT_COMMANDS
	Serial.print(millis(), DEC);
	Serial.println("\tHub Display Turned Off");
#endif
	hub_display_state = 0;
}
int Focuser_Halt_f(String command) {				// f16
#ifdef DEBUG_OUTPUT_COMMANDS
	Serial.print(millis(), DEC);
	Serial.println("\tFocuser Halt Sent");
#endif
	Send_Message_to_Focuser((unsigned int) Focuser_Halt_command, 0, 0, 0);				// send message to focuser
}
int Focuser_Home_f(String command) {			// f17
#ifdef DEBUG_OUTPUT_COMMANDS
	Serial.print(millis(), DEC);
	Serial.println("\tFocuser Home Sent");
#endif
	Send_Message_to_Focuser((unsigned int) Focuser_Home_command, 0, 0, 0);
}
int Focuser_Move_f(String command) {				// f18
	Parse_Command_to_Fieldf(command);
#ifdef DEBUG_OUTPUT_COMMANDS
	Serial.print(millis(), DEC);
	Serial.print("\tFocuser Move Sent:");
	Serial.println(command_fieldf[1], DEC);
#endif
	Send_Message_to_Focuser((unsigned int) Focuser_Move_command, (unsigned int) command_fieldf[1], (unsigned int) command_fieldf[2], (unsigned int) command_fieldf[3]);
}
int Focuser_Move_to_f(String command) {				// f19
	Parse_Command_to_Fieldf(command);
#ifdef DEBUG_OUTPUT_COMMANDS
	Serial.print(millis(), DEC);
	Serial.print("\tFocuser Move to ");
	Serial.print(command_fieldf[1], DEC);
	Serial.println("Sent:");
#endif
	Send_Message_to_Focuser(Focuser_Move_to_command, (unsigned int) command_fieldf[1], (unsigned int) command_fieldf[2], (unsigned int) command_fieldf[3]);
}
int Focuser_Display_On_f(String command) {				// f20
#ifdef DEBUG_OUTPUT_COMMANDS
	Serial.print(millis(), DEC);
	Serial.println("\tFocuser Display Turned On");
#endif
	Send_Message_to_Focuser((unsigned int) Focuser_Display_On_command, 0, 0, 0);
}
int Focuser_Display_Off_f(String command) {				// f18
#ifdef DEBUG_OUTPUT_COMMANDS
	Serial.print(millis(), DEC);
	Serial.println("\tFocuser Display Turned Off");
#endif
	Send_Message_to_Focuser((unsigned int) Focuser_Display_Off_command, 0,0,0);
}
// End of Custom functions accessible to the API-------------------------------------------------------------------------------------------
bool Check_Altitude_Message(void) {
  while (altitude_outptr != altitude_inptr) {                                        // check altitude serial buffer for data
    char thisbyte = altitude_inbuffer[altitude_outptr++];                           // take a character from the input buffer and increment pointer
    if ((thisbyte == (char)STX) && (altitude_string_ptr == 0)) {                    // look for the STX, but only if the output string is empty
      Incoming_Message_from_Altitude.sg_chars[altitude_string_ptr++] = STX;                // store the STX and increment the string pointer
      Altitude_Incoming_Message_Available = false;
    } else {
      if (thisbyte == (char)ETX) {												// character was not an STX check for ETX
        Incoming_Message_from_Altitude.sg_chars[altitude_string_ptr++] = ETX;            // save the ETX and increment the string pointer
        if (altitude_string_ptr == Incoming_Message_Structure_Length) {                  // does it mean end of packet (we just saved the ETX at 20!
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
bool Check_Azimuth_Message(void) {
  while (azimuth_outptr != azimuth_inptr) {                                        // check altitude serial buffer for data
    char thisbyte = azimuth_inbuffer[azimuth_outptr++];                           // take a character from the input buffer and increment pointer
    if ((thisbyte == (char)STX) && (azimuth_string_ptr == 0)) {                    // look for the STX, but only if the output string is empty
      Incoming_Message_from_Azimuth.sg_chars[azimuth_string_ptr++] = STX;                // store the STX and increment the string pointer
    } else {
      if (thisbyte == (char)ETX) {												// character was not an STX check for ETX
        Incoming_Message_from_Azimuth.sg_chars[azimuth_string_ptr++] = ETX;            // save the ETX and increment the string pointer
        if (azimuth_string_ptr == Incoming_Message_Structure_Length) {                                        // does it mean end of packet (we just saved the ETX at 20!
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
bool Check_Focuser_Message(void) {
	while (focuser_outptr != focuser_inptr) {											// check focuser serial buffer for data
		char thisbyte = focuser_inbuffer[focuser_outptr++];								// take a character from the input buffer and increment pointer
		if ((thisbyte == (char)STX) && (focuser_string_ptr == 0)) {						// look for the STX, but only if the output string is empty
			incoming_focuser_message.sg_chars[focuser_string_ptr++] = STX;				// store the STX and increment the string pointer
			Focuser_Incoming_Message_Available = false;
		}
		else {
			if (thisbyte == (char)ETX) {												// character was not an STX check for ETX
				incoming_focuser_message.sg_chars[focuser_string_ptr++] = ETX;     // save the ETX and increment the string pointer
				if (focuser_string_ptr == focuser_incoming_message_length) {  // does it mean end of packet (we just saved the ETX at 20!
					focuser_string_ptr = 0;												// zero the string pointer
					Focuser_Incoming_Message_Available = true;
				}
			}
			else {
				incoming_focuser_message.sg_chars[focuser_string_ptr++] = thisbyte; // Not a valid STX or a valid ETX so save it and increment string pointer
			}
		}
		return Focuser_Incoming_Message_Available;
	} // end of while focuser
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
void Parse_Altitude_Incoming_Message() {// process an update message from the Altitude controller
	Flash_Status_Light();
	Altitude_Incoming_Message_Available = false;
#ifdef DEBUG_OUTPUT_ALTITUDE
	Serial.print(millis(), DEC);
	Serial.println("\tPacket Received from Altitude Motor");
#endif
	altitude_status = (Incoming_Message_from_Altitude.message.motor_status);			// move incoming status to HUB 7 segment display
	al_Running_value = (Incoming_Message_from_Altitude.message.motor_status & 0x01);       // status A0 - running
	al_Slewing_value = (Incoming_Message_from_Altitude.message.motor_status & 0x02) >> 1;     // status A1 - slewing
	al_Tracking_value = (Incoming_Message_from_Altitude.message.motor_status & 0x04) >> 2;    // status A2 - tracking
	al_Homing_value = (Incoming_Message_from_Altitude.message.motor_status & 0x08) >> 3;    // status A3 - homing
	al_AtHome_value = (Incoming_Message_from_Altitude.message.motor_status & 0x10) >> 4;       // status A4 - athome
	al_AtPark_value = (Incoming_Message_from_Altitude.message.motor_status & 0x20) >> 5;    // status A5 - atpark
	al_PulseGuiding_value = (Incoming_Message_from_Altitude.message.motor_status & 0x40) >> 6;	// status A6 - pulse guiding
	Altitude_value = ((double)Incoming_Message_from_Altitude.message.altitude_bearing);
	al_Azimuth_value = ((double)Incoming_Message_from_Altitude.message.azimuth_bearing);
	al_Field3_value = ((double)Incoming_Message_from_Altitude.message.field3);
	al_Field4_value = ((double)Incoming_Message_from_Altitude.message.field4);
	switch (Incoming_Message_from_Altitude.message.command_number) {
		case Motor_update_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
        Serial.print(millis(), DEC);
        Serial.print("\tUpdate Packet from AL/C: azimuth ");
        Serial.print(al_Azimuth_value, DEC);
		Serial.print("\taltitude ");
		Serial.print(Altitude_value, DEC);
		Serial.print("\tField3 ");
        Serial.print(al_Field3_value, DEC);
        Serial.print("\tField4 ");
        Serial.print(al_Field4_value, DEC);
		Serial.print("\t");
        if (al_Running_value == 0x01) Serial.print("R,"); else Serial.print("!R,");
        if (al_Slewing_value == 0x01) Serial.print("S,"); else Serial.print("!S,");
        if (al_Tracking_value == 0x01) Serial.print("T,"); else Serial.print("!T,");
        if (al_PulseGuiding_value == 0x01) Serial.print("P,"); else Serial.print("!P,");
        if (al_Homing_value == 0x01) Serial.print("H,"); else Serial.print("!H,");
        if (al_AtHome_value == 0x01) Serial.print("AH,"); else Serial.print("!AH,");
        if (al_AtPark_value == 0x01) Serial.print("AP"); else Serial.print("!AP");
        Serial.println();
#endif
        Slewing_value = az_Slewing_value & al_Slewing_value;
        Tracking_value = az_Tracking_value & al_Tracking_value;
        AtHome_value = az_AtHome_value & al_AtHome_value;
        AtPark_value = az_AtPark_value & al_AtPark_value;
        break;
      }
	case AZ_Home_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
		Serial.print(millis(), DEC);
		Serial.println("\tAL - i1 - AZHome command received");
		Send_Message_to_Driver(Azimuth_motor, (double)AZ_Home_command, 0, 0);
#endif
		break;
	}
	case AZ_Halt_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
		Serial.print(millis(), DEC);
		Serial.println("\tAL - i2 - AZ Halt command received");
		Send_Message_to_Driver(Azimuth_motor, (double)AZ_Halt_command, 0, 0);
#endif
		break;
	}
	case Azimuth_Display_command: { // i4
#ifdef DEBUG_OUTPUT_ALTITUDE
//		Serial.print(millis(), DEC);
//		Serial.println("\tAL - Azimuth_Display Reply Received");
#endif
		break;
	}
    case TargetAltitude_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
		Serial.print(millis(), DEC);
        Serial.println("\tAL - TargetAltitude Reply Received ");
#endif
        break;
      }
    case TargetAzimuth_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
        Serial.print(millis(), DEC);
        Serial.println("\tAL - TargetAzimuth Reply Received ");
#endif
        break;
      }
    case Tracking_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
        Serial.print(millis(), DEC);
        Serial.println("\tAL - Tracking Reply Received ");
#endif
        break;
      }
    case TrackingRates_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
        Serial.print(millis(), DEC);
        Serial.println("\tAL - TrackingRate Reply Received ");
#endif
        break;
      }
    case AbortSlew_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
        Serial.print(millis(), DEC);
        Serial.println("\tAL - AbortSlew Reply Received ");
#endif
        break;
      }
    case FindHome_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
        Serial.print(millis(), DEC);
        Serial.println("\tAL - FindHome Reply Received ");
#endif
        break;
      }
    case Park_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
        Serial.print(millis(), DEC);
        Serial.println("\tAL - Park Reply Received ");
#endif
        break;
      }
    case SlewToAltAz_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
        Serial.print(millis(), DEC);
        Serial.println("\tAL - SlewToAltAz Reply Received ");
#endif
        break;
      }
    case SlewToTarget_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
        Serial.print(millis(), DEC);
        Serial.println("\tAL - SlewToTarget Reply Received ");
#endif
        break;
      }
    case SyncToAltAz_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
        Serial.print(millis(), DEC);
        Serial.println("\tAL - SyncToTarget Reply Received ");
#endif
        break;
      }
    case PulseGuide_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
        Serial.print(millis(), DEC);
        Serial.println("\tAL - PulseGuide Reply Received ");
#endif
        break;
      }
    case GuideRateRightAscension_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
        Serial.print(millis(), DEC);
        Serial.println("\tAL - GuideRateRightAscension Reply Received ");
#endif
        break;
      }
    case GuideRateDeclination_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
        Serial.print(millis(), DEC);
        Serial.println("\tAL - GuideRateDeclination Reply Received ");
#endif
        break;
      }
    case UnPark_command: {
#ifdef DEBUG_OUTPUT_ALTITUDE
        Serial.print(millis(), DEC);
        Serial.println("\tAL - UnPark Reply Received ");
#endif
        break;
      }
  }
}
void Parse_Azimuth_Incoming_Message() {                                       // process an update message from the Azimuth controller
  Flash_Status_Light();
  Azimuth_Incoming_Message_Available = false;
#ifdef DEBUG_OUTPUT_AZIMUTH
	Serial.print(millis(), DEC);
	Serial.println("\tPacket Received from Azimuth Motor");
#endif
	azimuth_status = (Incoming_Message_from_Azimuth.message.motor_status);
	az_Running_value = (Incoming_Message_from_Azimuth.message.motor_status & 0x01);       // status A0 - running
	az_Slewing_value = (Incoming_Message_from_Azimuth.message.motor_status & 0x02) >> 1;     // status A1 - slewing
	az_Tracking_value = (Incoming_Message_from_Azimuth.message.motor_status & 0x04) >> 2;    // status A2 - tracking
	az_Homing_value = (Incoming_Message_from_Azimuth.message.motor_status & 0x08) >> 3;    // status A3 - homing
	az_AtHome_value = (Incoming_Message_from_Azimuth.message.motor_status & 0x10) >> 4;       // status A4 - athome
	az_AtPark_value = (Incoming_Message_from_Azimuth.message.motor_status & 0x20) >> 5;    // status A5 - atpark
	az_PulseGuiding_value = (Incoming_Message_from_Azimuth.message.motor_status & 0x40) >> 6;  // status A6 - pulse guiding
	Azimuth_value = ((double)Incoming_Message_from_Azimuth.message.azimuth_bearing);
	az_Altitude_value = ((double)Incoming_Message_from_Azimuth.message.altitude_bearing);
	az_Field3_value = ((double)Incoming_Message_from_Azimuth.message.field3);
	az_Field4_value = ((double)Incoming_Message_from_Azimuth.message.field4);
	switch (Incoming_Message_from_Azimuth.message.command_number) {
		case Motor_update_command: {
#ifdef DEBUG_OUTPUT_AZIMUTH
        Serial.print(millis(), DEC);
        Serial.print("\tUpdate Packet from AZ/C: azimuth ");
        Serial.print(Azimuth_value, DEC);
        Serial.print("\taltitude ");
        Serial.print(az_Altitude_value, DEC);
        Serial.print("\tField3 ");
        Serial.print(az_Field3_value, DEC);
        Serial.print("\tField4 ");
        Serial.print(az_Field4_value, DEC);
		Serial.print("\t");
        if (az_Running_value == 0x01) Serial.print("R,"); else Serial.print("!R,");
        if (az_Slewing_value == 0x01) Serial.print("S,"); else Serial.print("!S,");
        if (az_Tracking_value == 0x01) Serial.print("T,"); else Serial.print("!T,");
        if (az_PulseGuiding_value == 0x01) Serial.print("P,"); else Serial.print("!P,");
        if (az_Homing_value == 0x01) Serial.print("H,"); else Serial.print("!H,");
        if (az_AtHome_value == 0x01) Serial.print("AH,"); else Serial.print("!AH,");
        if (az_AtPark_value == 0x01) Serial.print("AP"); else Serial.print("!AP");
        Serial.println();
#endif
        Slewing_value = al_Slewing_value & az_Slewing_value;
        Tracking_value = al_Tracking_value & az_Tracking_value;
        AtHome_value = al_AtHome_value & az_AtHome_value;
        AtPark_value = al_AtPark_value & az_AtPark_value;
		Send_Message_to_Driver(Altitude_motor, Azimuth_Display_command, Azimuth_value, 0);      // send message to driver
        break;
      }
	case Azimuth_Display_command: { // i4
#ifdef DEBUG_OUTPUT_AZIMUTH
//		Serial.print(millis(), DEC);
//		Serial.println("\tAZ - Azimuth_Display Reply Received");
#endif
		break;
	}
    case TargetAltitude_command: { // c2
#ifdef DEBUG_OUTPUT_AZIMUTH
        Serial.print(millis(), DEC);
        Serial.println("\tAZ - TargetAltitude Reply Received ");
#endif
        break;
      }
    case TargetAzimuth_command: { // c3
#ifdef DEBUG_OUTPUT_AZIMUTH
        Serial.print(millis(), DEC);
        Serial.println("\tAZ - TargetAzimuth Reply Received ");
#endif
        break;
      }
    case Tracking_command: { // c4
#ifdef DEBUG_OUTPUT_AZIMUTH
        Serial.print(millis(), DEC);
        Serial.println("\tAZ - Tracking Reply Received ");
#endif
        break;
      }
    case TrackingRates_command: { // c5
#ifdef DEBUG_OUTPUT_AZIMUTH
        Serial.print(millis(), DEC);
        Serial.println("\tAZ - TrackingRates Reply Recieved ");
#endif
        break;
      }
    case AbortSlew_command: { // c6
#ifdef DEBUG_OUTPUT_AZIMUTH
        Serial.print(millis(), DEC);
        Serial.println("\tAZ - AbortSlew Reply Received ");
#endif
        break;
      }
    case FindHome_command: { // c7
#ifdef DEBUG_OUTPUT_AZIMUTH
        Serial.print(millis(), DEC);
        Serial.println("\tAZ - FindHome Reply Received ");
#endif
        break;
      }
    case Park_command: { // c8
#ifdef DEBUG_OUTPUT_AZIMUTH
        Serial.print(millis(), DEC);
        Serial.println("\tAZ - Park Reply Received ");
#endif
        break;
      }
    case SlewToAltAz_command: { // c9
#ifdef DEBUG_OUTPUT_AZIMUTH
        Serial.print(millis(), DEC);
        Serial.println("\tAZ - SlewToAltAz Reply Received ");
#endif
        break;
      }
    case SlewToTarget_command: { // c10
#ifdef DEBUG_OUTPUT_AZIMUTH
        Serial.print(millis(), DEC);
        Serial.println("\tAZ - SlewToTarget Reply Received ");
#endif
        break;
      }
    case SyncToAltAz_command: { // c11
#ifdef DEBUG_OUTPUT_AZIMUTH
        Serial.print(millis(), DEC);
        Serial.println("\tAZ - SyncToAltAz Reply Received ");
#endif
        break;
      }
    case PulseGuide_command: {
#ifdef DEBUG_OUTPUT_AZIMUTH
        Serial.print(millis(), DEC);
        Serial.println("\tAz- PulseGuide Reply Received ");
#endif
        break;
      }
    case GuideRateRightAscension_command: {
#ifdef DEBUG_OUTPUT_AZIMUTH
        Serial.print(millis(), DEC);
        Serial.println("\tAz- GuideRateRightAscension Reply Received ");
#endif
        break;
      }
    case GuideRateDeclination_command: {
#ifdef DEBUG_OUTPUT_AZIMUTH
        Serial.print(millis(), DEC);
        Serial.println("\tAz - GuideRateDeclination Reply Received ");
#endif
        break;
      }
    case UnPark_command: { // c12
#ifdef DEBUG_OUTPUT_AZIMUTH
        Serial.print(millis(), DEC);
        Serial.println("\tAZ - UnPark Reply Received ");
#endif
        break;
      }
  }
}
void Parse_Focuser_Incoming_Message() {                                       // process an update message from the Azimuth controller
	Flash_Status_Light();
	Focuser_Incoming_Message_Available = false;
#ifdef DEBUG_FOCUSER
	Serial.print(millis(), DEC);
	Serial.println("\tPacket Received from Focuser");
#endif
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
		Focuser_Altitude_value = incoming_focuser_message.focuser_incoming_message.altitude;
	}
	else {
		Focuser_Altitude_value = 0;
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
		Serial.println(Focuser_Altitude_value, DEC);		
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
	case Focuser_Display_On_command: {
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
	case Focuser_Halt_command: {
#ifdef DEBUG_OUTPUT_FOCUSER
		Serial.print(millis(), DEC);
		Serial.println("\tFocuser - Halt Reply Received");
#endif
		break;
	}
	}
}
void Send_Message_to_Driver(int motor, int command_number, double field3, double field4) {
  Outgoing_Message.message.header = STX;
  Outgoing_Message.message.footer = ETX;
  Outgoing_Message.message.command_number = command_number;
  Outgoing_Message.message.motor_status = 0;
  Outgoing_Message.message.parameter_one = field3;
  Outgoing_Message.message.parameter_two = field4;
#ifdef DEBUG_OUTPUT
	Serial.print(millis(), DEC);
	Serial.print("\tMotor Message to be sent: ");
	for (int i = 0; i < Outgoing_Message_Structure_Length; i++) {
		Serial.print(Outgoing_Message.sg_chars[i], HEX);
		Serial.print(",");
	}
	Serial.println();
	Serial.print(millis(), DEC);
	Serial.print("\t");
#endif
  switch (motor) {													// send the message tothe required motor
    case Altitude_motor:
#ifdef DEBUG_OUTPUT
//      Serial.print("Message Sent to Altitude motor ");
#endif
      for (int i = 0; i < Outgoing_Message_Structure_Length; i++)	{
#ifdef DEBUG_OUTPUT
//        Serial.print(Outgoing_Message.sg_chars[i], HEX);
//        Serial.print(",");
#endif
        altitude_serial.write(Outgoing_Message.sg_chars[i]);
      }
#ifdef DEBUG_OUTPUT
	Serial.println("");
#endif
      break;
    case Azimuth_motor:
#ifdef DEBUG_OUTPUT
      Serial.print("Message Sent to Azimuth motor ");
#endif
      for (int i = 0; i < Outgoing_Message_Structure_Length; i++) {
#ifdef DEBUG_OUTPUT
	Serial.print(Outgoing_Message.sg_chars[i], HEX);
	Serial.print(",");
#endif
        azimuth_serial.write(Outgoing_Message.sg_chars[i]);
      }
#ifdef DEBUG_OUTPUT
	Serial.println("");
#endif
      break;
  }
}
void Send_Message_to_Focuser(unsigned int command_number, unsigned int target, unsigned int direction, unsigned int resolution) {
	Focuser_Outgoing_Message.focuser_message.header = STX;
	Focuser_Outgoing_Message.focuser_message.footer = ETX;
	Focuser_Outgoing_Message.focuser_message.focuser_command_number = command_number;
	Focuser_Outgoing_Message.focuser_message.focuser_target = target;
	Focuser_Outgoing_Message.focuser_message.focuser_direction = direction;
#ifdef DEBUG_OUTPUT_FOCUSER
	Serial.print(millis(), DEC);
	Serial.print("\tFocuser Message to be sent: ");
	for (int i = 0; i < Focuser_Outgoing_Message_Structure_Length; i++) {
		Serial.print((char) Focuser_Outgoing_Message.sg_chars , HEX);
		Serial.print(",");
	}
	Serial.println();
	Serial.print("Message Sent to Focuser:");
#endif
	for (int i = 0; i < Focuser_Outgoing_Message_Structure_Length; i++) {
		focuser_serial.write((char)Focuser_Outgoing_Message.sg_chars);
	}
#ifdef DEBUG_OUTPUT_FOCUSER
	Serial.println("");
#endif
}
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
void Flash_Status_Light(void) {
  if (millis() - time > debounce) {
    reading = digitalRead(Box_led);
    if (reading == HIGH) {
      digitalWrite(Box_led, LOW);
    }
    else {
      digitalWrite(Box_led, HIGH);
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