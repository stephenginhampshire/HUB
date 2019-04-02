/*
  Arduino Ethernet Telescope Hub
        interfaces  Steve and Jamie Gould's Telescope to a PC ASCOM compliant software driver
		Declination = Altitude = north/south = up down
		Right Ascension = Azimuth  = east/west = left right
		Communications to the two motor controllers is made through this HUB.

		Version 1.0 30/01/2018
*/
/* Version Control ------------------------------------------------------------------------------------------
	Version	Date		Description								Debug Status	Release Date
	1.		27/0/2018
	1.1		12/12/2018	Focuser resolution removed from protocol
	1.2		20/01/2019	Recoded command handling
*/
// Definitions ----------------------------------------------------------------------------------------------
#define MONITOR_PRINT_SETUP
#define MONITOR_PRINT_DATETIME
#define MONITOR_PRINT_MOTOR_DRIVERS
#define MONITOR_PRINT_FOCUSER
#define MONITOR_PRINT_CONTROLLER
#define MONITOR_PRINT_COMMANDS
#define DEBUG_FOCUSER_COMMANDS
#define DEBUG_TELESCOPE_COMMANDS
#define MONITOR_PRINT_FIELDS
#define MONITOR_PRINT_COMMANDS
#define MONITOR_PRINT_SEVEN_SEGMENT
#define MONITOR_PRINT_MOTOR_COMMANDS
#define MONITOR_PRINT_FOCUSER_COMMANDS
#define MONITOR_PRINT_HUB_COMMANDS
#define MONITOR_PRINT_LOGGER
//#define MONITOR_DUMMY_DATETIME					// force a default time and date

#define WDT						// enable WatchDog Timer
#define ETHERNET				// enable ethernet			

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

#define stage_year_month			0
#define stage_day_hour				1
#define stage_minute_second			2
#define stage_wait_year_month		3
#define stage_wait_day_hour			4
#define stage_wait_minute_second	5
#define stage_wait_complete			6

#define LED_FLASH_TIME		20      // number of milli seconds to light the LEDs
// Inclusions -----------------------------------------------------------------------------------------------
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Ethernet2.h>
#include <SoftwareSerial.h>
#include "Seven_Segment_Display.h"
#include "HUB_Commands.h"
#ifdef WDT
#include <avr/wdt.h>
#endif
// Constants ------------------------------------------------------------------------------------------------
byte mac[] = { 0x90, 0xA2, 0xDA, 0x11, 0x23, 0x40 };	// MAC address of Ethernet controller, found on a sticker on the back of the Ethernet shield.
//byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };	// MAC address of Jamie's Ethernet Controller
//IPAddress ip(92, 68, 0, 30);							// IP Address (FIXED) of this server
//IPAddress gateway(92, 68, 0, 00);
//IPAddress subnet(255, 255, 255, 0);

// Instantiations -------------------------------------------------------------------------------------------
#ifdef ETHERNET
EthernetServer server(80);                              // Create a server listening on the given port.
#endif
SoftwareSerial logger_serial(logger_RX_pin,logger_TX_pin); // RX, TX, Create the serial port for logging data
// Working Variables ----------------------------------------------------------------------------------------
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

int date_stage = 0;						// check track of stage obtaining date and time
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
long Red_LED_Time_On = (long)0;
bool Red_LED_State = false;
long Orange_LED_Time_On = (long)0;
bool Orange_LED_State = false;
long Green_LED_Time_On = (long)0;
bool Green_LED_State = false;
unsigned long last_status_update = 0;
char telescope_packet_count = 0;
char controller_packet_count = 0;
char focuser_packet_count = 0;
unsigned long last_datetime = millis();

bool focuser_display_on = true;
char display_string[9];
// Prototypes -----------------------------------------------------------------------------------------------
void Prepare_Packet_for_Output(unsigned char, unsigned char, double, double);
void Send_Packet_to_Controller(void);
void send_update_to_SEVEN_SEGMENT(void);
void write_logger(unsigned char, unsigned char, unsigned char, unsigned char, double, double);
void Send_Packet_to_Telescope(unsigned char);
void Turn_Red_LED(bool);
void Turn_Green_LED(bool);
void Turn_Orange_LED(bool);
void Send_Packet_to_Focuser(void);
void error_display(String);
void display(String);
bool Check_Controller_Packet(void);
void Process_Incoming_Packet_from_Controller(void);
bool Check_Altitude_Packet(void);
void Process_Incoming_Packet_from_Telescope(unsigned char);
bool Check_Azimuth_Packet(void);
void Process_Incoming_Packet_from_Telescope(unsigned char);
bool Check_Focuser_Packet(void);
void Process_Incoming_Packet_from_Focuser(void);
//-- Setup --------------------------------------------------------------------------------------------------
void setup() {
#ifdef WDT
	wdt_disable();													// disable watchdog timer during setup
#endif
	Serial.print(millis(), DEC); Serial.println("\tSetup Commenced");
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
	Serial.print(millis(), DEC); Serial.println("\tSerial Ports setup");
	Serial.print(millis(), DEC); Serial.println("\tSetting Up Ethernet Server");
#ifdef ETHERNET
	if (Ethernet.begin(mac) == 0) {									//  Ethernet.begin(mac, ip, gateway, subnet);                       // Initialize the Ethernet shield
#ifdef MONITOR_PRINT_SETUP
    Serial.print(millis(), DEC); Serial.println("\tFailed to configure Ethernet using DHCP");	// no point in carrying on, so do nothing forevermore, try to configure using IP address instead of DHCP:
#endif
	while (1);														// Error(Ethernet_Error);
  }
	server.begin();
	Serial.print(millis(), DEC); Serial.print("\tEthernet Server Setup Complete, at "); Serial.println(Ethernet.localIP());
#endif // end ETHERNET
	Serial.print(millis(), DEC); Serial.println("\tASCOM setup complete");
#ifdef WDT
  Serial.print(millis(), DEC); Serial.println("\tSetting Up Watchdog");
  wdt_enable(WDTO_2S);											// enable a 2 second watchdog timeout
  Serial.print(millis(), DEC); Serial.println("\tWatchdog Setup Complete");
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
  Serial.print(millis(), DEC); Serial.println("\tSetup Complete");
} // end setup
// Main -----------------------------------------------------------------------------------------------------
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
	if (Check_Altitude_Packet() == true) Process_Incoming_Packet_from_Telescope((unsigned char)SOURCE_ALTITUDE);
	if (Check_Azimuth_Packet() == true) Process_Incoming_Packet_from_Telescope((unsigned char)SOURCE_AZIMUTH);
	if (Check_Focuser_Packet() == true) Process_Incoming_Packet_from_Focuser();
	Update_Datetime();													// check if time and date requires updating
	Clear_LEDS();
} // end of main loop
// Update Date and Time from the Focuser --------------------------------------------------------------------
void Update_Datetime(void) {
	unsigned char source = (unsigned char)SOURCE_HUB;
	unsigned char command_number = 0;
	double parameter_one = 0;
	double parameter_two = 0;
	if ((millis() >= last_datetime + (long)60000) || date_stage != (int)stage_wait_complete) {				// get the date and time from the controller every minute
		if (date_stage == (int)stage_wait_complete) date_stage = (int)stage_year_month;
		last_datetime = millis();
		Serial.print(millis(), DEC); Serial.print("\tRequesting Time and Date. Stage:"); Serial.println(date_stage, DEC);
		switch (date_stage) {
		case 0: {											// day and month required
			Serial.print(millis(), DEC); Serial.println("\tRequesting Year and Month");
			command_number = (unsigned char)HUB_Get_Year_Month;
			Prepare_Packet_for_Output(source, command_number, parameter_one, parameter_two);
			Send_Packet_to_Focuser();
			date_stage = (int)stage_wait_year_month;
			break;
		}
		case (int)stage_wait_year_month: {					// do nothing, waiting for day and month
			break;
		}
		case (int)stage_day_hour: {						// year and hour required
			Serial.print(millis(), DEC); Serial.println("\tRequesting Day and Hour");
			command_number = (unsigned char)HUB_Get_Day_Hour;
			Prepare_Packet_for_Output(source, command_number, parameter_one, parameter_two);
			Send_Packet_to_Focuser();
			date_stage = (int)stage_wait_day_hour;
			break;
		}
		case (int)stage_wait_day_hour: {					// do nothing, waiting for year and hour
			break;
		}
		case (int)stage_minute_second: {					// minute and second required
			Serial.print(millis(), DEC); Serial.println("\tRequesting Minute and Second");
			command_number = (unsigned char)HUB_Get_Minute_Second;
			Prepare_Packet_for_Output(source, command_number, parameter_one, parameter_two);
			Send_Packet_to_Controller();
			date_stage = (int)stage_wait_minute_second;
			break;
		}
		case (int)stage_wait_minute_second: {				// do nothing, waiting for minute and second
			break;
		}
		}
		Serial.print(millis(), DEC); Serial.println("\tRequested Time and Date");
	}
}
void Clear_LEDS(void) {
	if ((Red_LED_State == true) && (millis() >= (Red_LED_Time_On + (unsigned long) LED_FLASH_TIME))) Turn_Red_LED(false);
	if ((Orange_LED_State == true) && (millis() >= (Orange_LED_Time_On + (unsigned long)LED_FLASH_TIME))) Turn_Orange_LED(false);
	if ((Green_LED_State == true) && (millis() >= (Green_LED_Time_On + (unsigned long) LED_FLASH_TIME))) Turn_Green_LED(false);
}
// Turn Hub Display On --------------------------------------------------------------------------------------
void Hub_Display_On(bool on_off) {
#ifdef MONITOR_PRINT_COMMANDS
	Serial.print(millis(), DEC); Serial.print("\tTurn Hub Display: ");
	if (on_off == true) Serial.println("On");
	if (on_off == false) Serial.println("Off");
#endif
	hub_display_state = on_off;
}
// End of Controller Functions ------------------------------------------------------------------------------
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
// Check Altitude Packet Received ---------------------------------------------------------------------------
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
// Check Azimuth_Packet -------------------------------------------------------------------------------------
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
// Check Focuser Packet -------------------------------------------------------------------------------------
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
// Interrupt Service Routines -------------------------------------------------------------------------------
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
// Write Logger ---------------------------------------------------------------------------------------------
void write_logger(unsigned char packet_type, unsigned char direction, unsigned char source, unsigned char command_number, double parameter_one, double parameter_two) {
	Serial.print(millis(), DEC); Serial.println("\tWriting Log Message");
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
	case SOURCE_ALTITUDE: {
		Logger_Message += "Altitude Motor";
		break;
	}
	case SOURCE_AZIMUTH: {
		Logger_Message += "Azimuth Motor";
		break;
	}
	case SOURCE_FOCUSER: {
		Logger_Message += "Focuser";
		break;
	}
	case SOURCE_ALTAZI: {
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
		for (int i = 0; i < x; i++) {		// send message to logger port
			logger_serial.write(Logger_Message.charAt(i));
		}
		logger_serial.write((unsigned char)ETX);
#ifdef MONITOR_PRINT_DATETIME
		Serial.print(millis(), DEC); Serial.println("\tLog Written");
		Serial.print(millis(), DEC); Serial.print("\tLogger_Message: "); Serial.println(Logger_Message);
#endif
	}
}
// Process Incoming Packet from the Telescope ---------------------------------------------------------------
void Process_Incoming_Packet_from_Telescope(unsigned char motor) {// process an update message from a motor driver
	int command_number = 0;
	double parameter_one = 0;
	double parameter_two = 0;
	bool forward_packet = true;
	Turn_Green_LED(true);
	if (motor == (unsigned char)SOURCE_AZIMUTH) {
		write_logger((unsigned char)NULL, (unsigned char)FROM, (unsigned char)SOURCE_AZIMUTH, (unsigned char)command_number, parameter_one, parameter_two);
		Azimuth_Incoming_Message_Available = false;
		command_number = (int)Incoming_Message_from_Azimuth.contents.command_number;
		parameter_one = Incoming_Message_from_Azimuth.contents.parameter_one;
		parameter_two = Incoming_Message_from_Azimuth.contents.parameter_two;
		Serial.print(millis(), DEC); Serial.print("\tPacket Received from Azimuth Motor: ");
	}
	else {
		write_logger((unsigned char)NULL, (unsigned char)FROM, (unsigned char)SOURCE_ALTITUDE, (unsigned char)command_number, parameter_one, parameter_two);
		Altitude_Incoming_Message_Available = false;
		command_number = Incoming_Message_from_Altitude.contents.command_number;
		parameter_one = Incoming_Message_from_Altitude.contents.parameter_one;
		parameter_two = Incoming_Message_from_Altitude.contents.parameter_two;
		Serial.print(millis(), DEC); Serial.print("\tPacket Received from Altitude Motor");
	}
	Serial.print("Command: "); Serial.print(command_number, DEC);
	Serial.print(" Parameter_one: "); Serial.print(parameter_one, DEC);
	Serial.print(" Parameter_two: "); Serial.println(parameter_two, DEC);
	telescope_packet_count++;
	if (telescope_packet_count > (unsigned char)99) telescope_packet_count = 0;
	switch (command_number) {							// take any necessary HUB action, none normally required
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
	case Telescope_Future_Target_Right_Ascension:
	case Telescope_Future_Target_Declination:
	case Telescope_Slew_Settle_Time:
	case Telescope_Slewing:
	case Telescope_Site_Longitude:
	case Telescope_Can_Park:
	case Telescope_Site_Latitude:
	case Telescope_Sidereal_Time_Hours:
	case Telescope_Sidereal_Time_Minutes_Seconds:
	case Telescope_Side_of_Pier:
	case Telescope_Right_Ascension_Rate:
	case Telescope_Right_Ascension:
	case Telescope_Is_Pulse_Guiding:
	case Telescope_Guide_Rate_Right_Ascension:
	case Telescope_Guide_Rate_Declination:
	case Telescope_Site_Elevation:
	case Telescope_Can_Find_Home:
	case Telescope_UTCDATE_Year_Month:
	case Telescope_UTCDATE_Day_Hour:
	case Telescope_UTCDATE_Minute_Second:
	case Telescope_At_Park:
	case Telescope_At_Home:
	case Telescope_Aperture_Diameter:
	case Telescope_Aperture_Area:
	case Telescope_Altitude:
	case Telescope_Alignment_Mode:
	case Telescope_Array_List:
	case Telescope_Name:
	case Telescope_Interface_Version:
	case Telescope_Driver_Version:
	case Telescope_Driver_Info:
	case Telescope_Description:
	case Telescope_Connected:
	case Telescope_Tracking_Rates_0_1:
	case Telescope_Tracking_Rates_2_3:
	case Telescope_Abort_Slew:
	case Telescope_Action:
	case Telescope_Axis_Rates:
	case Telescope_Can_Move_Axis:
	case Telescope_Command_Blind:
	case Telescope_Command_Bool:
	case Telescope_Command_String:
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
	case Telescope_Unpark:
	case Telescope_Altitude_to_Controller_Heartbeat:
	case Telescope_Azimuth_to_Controller_Heartbeat:
	case Telescope_Get_Year_Month:
	case Telescope_Get_Day_Hour:
	case Telescope_Get_Minute_Second:
	case Telescope_Display_Visible:
	case Telescope_Command_Error_Response:
	default:
		break;
	}
	if (forward_packet == true) {
		Prepare_Packet_for_Output((unsigned char)motor, (unsigned char)command_number, (double)parameter_one, (double)parameter_two);
		Send_Packet_to_Controller();
		Serial.print(millis(), DEC); Serial.println("\tPacket Sent to Controller");
	}
}
// Process Incoming Packet from the Focuser -----------------------------------------------------------------
void Process_Incoming_Packet_from_Focuser() {                   // process an update message from the Focuser
	int command_number = 0;
	double parameter_one = 0;
	double parameter_two = 0;
	bool reply = true;
	Turn_Orange_LED(true);
	Focuser_Incoming_Message_Available = false;
	write_logger((unsigned char)NULL,(unsigned char)FROM, (unsigned char)SOURCE_FOCUSER, (unsigned char)command_number, parameter_one, parameter_two);
	command_number = (int)Incoming_Message_from_Focuser.contents.command_number;
	parameter_one = Incoming_Message_from_Focuser.contents.parameter_one;
	parameter_two = Incoming_Message_from_Focuser.contents.parameter_two;
	Serial.print(millis(), DEC); Serial.print("\tPacket Received from Focuser: Command: "); Serial.print(command_number, DEC);
	Serial.print(" Parameter_one: "); Serial.print(parameter_one, DEC);
	Serial.print(" Parameter_two: ");Serial.println(parameter_two, DEC);
	focuser_packet_count++;
	if (focuser_packet_count > 99) focuser_packet_count = 0;
	switch (command_number) {
	case HUB_Get_Year_Month:
		system_year = parameter_one;
		system_month = parameter_two;
		reply = false;
		break;
	case HUB_Get_Day_Hour:
		system_day = parameter_one;
		system_hour = parameter_two;
		reply = false;
		break;
	case HUB_Get_Minute_Second:
		system_minute = parameter_one;
		system_second = parameter_two;
		reply = false;
		break;
	case Focuser_Current_Focuser_Position:
		break;
	case Focuser_Motor_Moving_Status:
		break;
	case Focuser_Is_Moving:
		break;
	case Focuser_Firmware_Version_String:
		break;
	case Focuser_Firmware_Name_and_Version_String:
		break;
	case Focuser_Maximum_Step:
		break;
	case Focuser_Maximum_Increment:
		break;
	case Focuser_Coil_Power:
		break;
	case Focuser_Motor_Speed:
		break;
	case Focuser_Step_Size:
		break;
	case Focuser_Temperature_Coefficient:
		break;
	case Focuser_Temperature_Compensation:
		break;
	case Focuser_Move_to_Position:
		break;
	case Focuser_Step_Mode:
		break;
	case Focuser_Step_Size_Enabled:
		break;
	case Focuser_Display_Visible:
		break;
	case Focuser_Temperature_Mode:
		break;
	case Focuser_Reset_Arduino_Controller:
		break;
	case Focuser_Reset_Focuser_Defaults:
		break;
	case Focuser_Motor_Speed_Threshold_When_Moving:
		break;
	case Focuser_Motor_Speed_Change_Enabled_When_Moving:
		break;
	case Focuser_Save_Settings_to_EEPROM:
		break;
	case Focuser_Humidity:
		break;
	case Focuser_Longitude:
		break;
	case Focuser_Latitude:
		break;
	case Focuser_Altitude:
		break;
	case Focuser_Voltages:
		break;
	case Focuser_Update_of_Position_When_Moving:
		break;
	case Focuser_Status_of_Home_Position_Switch:
		break;
	case Focuser_Jogging_Steps:
		break;
	case Focuser_Jogging_State:
		break;
	case Focuser_Jogging_Direction:
		break;
	case Focuser_Delay_After_Step:
		break;
	case Focuser_Backlash_In:
		break;
	case Focuser_Backlash_Out:
		break;
	case Focuser_Number_of_Backlash_Steps_In:
		break;
	case Focuser_Number_of_Backlash_Steps_Out:
		break;
	case Focuser_Temperature_Compensation_Direction:
		break;
	case Focuser_to_Controller_Heartbeat:
		break;
	default:
		break;
	}
	if (reply == true) {
		Prepare_Packet_for_Output((unsigned char)SOURCE_FOCUSER, (unsigned char)command_number, (double)parameter_one, (double)parameter_two);
		Send_Packet_to_Controller();
		Serial.print(millis(), DEC); Serial.println("\tPacket Sent to Controller");
	}
}
// Process Incoming Packet from the Controller --------------------------------------------------------------
void Process_Incoming_Packet_from_Controller() {
	int command_number = 0;
	double parameter_one = 0;
	double parameter_two = 0;
	Turn_Red_LED(true);
	command_number = (int)Incoming_Message_from_Controller.contents.command_number;
	parameter_one = Incoming_Message_from_Controller.contents.parameter_one;
	parameter_two = Incoming_Message_from_Controller.contents.parameter_two;
	Serial.print(millis(), DEC); Serial.print("\tPacket Received from Controller: ");
	Serial.print("Command: "); Serial.print(command_number, DEC);
	Serial.print(" Parameter_one: "); Serial.print(parameter_one, DEC);
	Serial.print(" Parameter_two: "); Serial.println(parameter_two, DEC);
	controller_packet_count++;
	if (controller_packet_count > 99) controller_packet_count = 0;
	switch (command_number) {
		case HUB_Display_Visible: {			// (01)
			hub_display_state = (bool)Incoming_Message_from_Controller.contents.parameter_one;
			break;
		}
		case HUB_Delete_Log_File: {
			write_logger((unsigned char)HUB_Delete_Log_File, (unsigned char)NULL, (unsigned char)SOURCE_CONTROLLER, (unsigned char)HUB_Delete_Log_File, (double)NULL, (double)NULL);
			break;
		}
		case Focuser_Current_Focuser_Position:
			break;
		case Focuser_Motor_Moving_Status:
			break;
		case Focuser_Is_Moving:
			break;        
		case Focuser_Firmware_Version_String:
			break;        
		case Focuser_Firmware_Name_and_Version_String:
			break;        
		case Focuser_Target_Position:
			break;        
		case Focuser_Temperature:
			break;        
		case Focuser_Maximum_Step:
			break;        
		case Focuser_Maximum_Increment:
			break;        
		case Focuser_Coil_Power:
			break;        
		case Focuser_Motor_Speed:
			break;        
		case Focuser_Step_Size:
			break;        
		case Focuser_Temperature_Coefficient:
			break;        
		case Focuser_Temperature_Compensation:
			break;        
		case Focuser_Move_to_Position:
			break;        
		case Focuser_Step_Mode:
			break;        
		case Focuser_Step_Size_Enabled:
			break;        
		case Focuser_Display_Visible:
			break;        
		case Focuser_Temperature_Mode:
			break;        
		case Focuser_Reset_Arduino_Controller:
			break;        
		case Focuser_Reset_Focuser_Defaults:
			break;        
		case Focuser_Motor_Speed_Threshold_When_Moving:
			break;        
		case Focuser_Motor_Speed_Change_Enabled_When_Moving:
			break;        
		case Focuser_Save_Settings_to_EEPROM:
			break;        
		case Focuser_Humidity:
			break;        
		case Focuser_Longitude:
			break;        
		case Focuser_Latitude:
			break;        
		case Focuser_Altitude:
			break;        
		case Focuser_Voltages:
			break;        
		case Focuser_Controller_Current:
			break;        
		case Focuser_Update_of_Position_When_Moving:
			break;        
		case Focuser_Status_of_Home_Position_Switch:
			break;        
		case Focuser_Jogging_Steps:
			break;        
		case Focuser_Jogging_State:
			break;        
		case Focuser_Jogging_Direction:
			break;        
		case Focuser_Delay_After_Step:
			break;        
		case Focuser_Backlash_In:
			break;        
		case Focuser_Backlash_Out:
			break;        
		case Focuser_Number_of_Backlash_Steps_In:
			break;        
		case Focuser_Number_of_Backlash_Steps_Out:
			break;        
		case Focuser_Temperature_Compensation_Direction:
			break;        
		case Telescope_Azimuth:
			break;        
		case Telescope_Declination:
			break;        
		case Telescope_Can_Unpark:
			break;        
		case Telescope_Can_Sync_AltAz:
			break;        
		case Telescope_Can_Sync:
			break;        
		case Telescope_Can_Slew_Async:
			break;        
		case Telescope_Can_Slew_AltAz_Async:
			break;        
		case Telescope_Can_Slew_AltAz:
			break;        
		case Telescope_Can_Slew:
			break;        
		case Telescope_Can_Set_Tracking:
			break;        
		case Telescope_Can_Set_Right_Ascension_Rate:
			break;        
		case Telescope_Can_Set_Pier_Side:
			break;        
		case Telescope_Can_Set_Park:
			break;        
		case Telescope_Can_Set_Guide_Rates:
			break;        
		case Telescope_Can_Set_Declination_Rate:
			break;        
		case Telescope_Can_Pulse_Guide:
			break;        
		case Telescope_Declination_Rate:
			break;        
		case Telescope_Does_Refraction:
			break;        
		case Telescope_Equatorial_System:
			break;        
		case Telescope_Focal_Length:
			break;        
		case Telescope_Tracking_Rate:
			break;        
		case Telescope_Tracking:
			break;        
		case Telescope_Future_Target_Right_Ascension:
			break;        
		case Telescope_Future_Target_Declination:
			break;        
		case Telescope_Slew_Settle_Time:
			break;        
		case Telescope_Slewing:
			break;        
		case Telescope_Site_Longitude:
			break;        
		case Telescope_Can_Park:
			break;        
		case Telescope_Site_Latitude:
			break;        
		case Telescope_Sidereal_Time_Hours:
			break;        
		case Telescope_Sidereal_Time_Minutes_Seconds:
			break;
		case Telescope_Side_of_Pier:
			break;        
		case Telescope_Right_Ascension_Rate:
			break;        
		case Telescope_Right_Ascension:
			break;        
		case Telescope_Is_Pulse_Guiding:
			break;        
		case Telescope_Guide_Rate_Right_Ascension:
			break;        
		case Telescope_Guide_Rate_Declination:
			break;        
		case Telescope_Site_Elevation:
			break;        
		case Telescope_Can_Find_Home:
			break;        
		case Telescope_UTCDATE_Year_Month:
			break;        
		case Telescope_UTCDATE_Day_Hour:
			break;        
		case Telescope_UTCDATE_Minute_Second:
			break;        
		case Telescope_At_Park:
			break;        
		case Telescope_At_Home:
			break;        
		case Telescope_Aperture_Diameter:
			break;        
		case Telescope_Aperture_Area:
			break;        
		case Telescope_Altitude:
			break;        
		case Telescope_Alignment_Mode:
			break;        
		case Telescope_Array_List:
			break;        
		case Telescope_Name:
			break;        
		case Telescope_Interface_Version:
			break;        
		case Telescope_Driver_Version:
			break;        
		case Telescope_Driver_Info:
			break;        
		case Telescope_Description:
			break;        
		case Telescope_Connected:
			break;        
		case Telescope_Tracking_Rates_0_1:
			break;        
		case Telescope_Tracking_Rates_2_3:
			break;
	    case Telescope_Abort_Slew:
			break;        
		case Telescope_Action:
			break;        
		case Telescope_Axis_Rates:
			break;        
		case Telescope_Can_Move_Axis:
			break;        
		case Telescope_Command_Blind:
			break;        
		case Telescope_Command_Bool:
			break;        
		case Telescope_Command_String:
			break;        
		case Telescope_Destination_Side_of_Pier:
			break;        
		case Telescope_Dispose:
			break;        
		case Telescope_Find_Home:
			break;        
		case Telescope_Move_Axis:
			break;        
		case Telescope_Park:
			break;        
		case Telescope_Pulse_Guide:
			break;        
		case Telescope_Set_Park:
			break;        
		case Telescope_Setup_Dialog:
			break;        
		case Telescope_Slew_to_AltAz:
			break;        
		case Telescope_Slew_to_AltAz_Async:
			break;        
		case Telescope_Slew_to_Coordinates:
			break;        
		case Telescope_Slew_to_Coordinates_Async:
			break;        
		case Telescope_Slew_to_Target:
			break;        
		case Telescope_Slew_to_Target_Async:
			break;        
		case Telescope_Sync_to_AltAz:
			break;        
		case Telescope_Sync_to_Coordinates:
			break;        
		case Telescope_Sync_to_Target:
			break;        
		case Telescope_Unpark:
			break;
		default:
			break;
	}
	Prepare_Packet_for_Output((unsigned char)SOURCE_CONTROLLER, (unsigned char)command_number, (double)parameter_one, (double)parameter_two);
	if ((command_number > Focuser_Start) && (command_number < Focuser_End)) {
			Send_Packet_to_Focuser();
			Serial.print(millis(), DEC); Serial.println("\tPacket Sent to Focuser");
		}
	else if ((command_number > Telescope_Start) && (command_number < Telescope_End)) {
		Send_Packet_to_Telescope(SOURCE_ALTAZI);
		Serial.print(millis(), DEC); Serial.println("\tPacket Sent to Both Motors");
	}
	write_logger((unsigned char)NULL, (unsigned char)FROM, (unsigned char)SOURCE_CONTROLLER, (unsigned char)command_number, parameter_one, parameter_two);
}
// Process Packet for Output --------------------------------------------------------------------------------
void Prepare_Packet_for_Output(byte source, byte command_number, double parameter_one, double parameter_two) {
	Outgoing_Message_to_Controller.contents.header = STX;
	Outgoing_Message_to_Controller.contents.source = source;
	Outgoing_Message_to_Controller.contents.command_number = (byte) command_number;
	Outgoing_Message_to_Controller.contents.parameter_one = parameter_one;
	Outgoing_Message_to_Controller.contents.parameter_two = parameter_two;
	Outgoing_Message_to_Controller.contents.footer = ETX;
}
// Send Packet to Telescope ---------------------------------------------------------------------------------
void Send_Packet_to_Telescope(unsigned char motors) {
	unsigned char command_number = Outgoing_Message_to_Motor_Drivers.contents.command_number;
	double parameter_one = Outgoing_Message_to_Motor_Drivers.contents.parameter_one;
	double parameter_two = Outgoing_Message_to_Motor_Drivers.contents.parameter_two;
	Serial.print(millis(), DEC);
	Serial.print("\tMotor Driver Message to be sent: "); 
	for (int i = 0; i < Message_Structure_Length; i++) {
		Serial.print(Outgoing_Message_to_Motor_Drivers.sg_chars[i], HEX);
		Serial.print(",");
	}
	Serial.println();
	Serial.print(millis(), DEC);
	Serial.print("\t");
	Serial.print("Message Sent to Motor Drivers ");
	telescope_packet_count++;
	if (telescope_packet_count > 99) telescope_packet_count = 1;
	for (int i = 0; i < Message_Structure_Length; i++) {
		Serial.print(Outgoing_Message_to_Motor_Drivers.sg_chars[i], HEX);
		Serial.print(",");
		if (motors == (unsigned char)SOURCE_ALTITUDE) {
			write_logger((unsigned char)NULL,(unsigned char)TO, (unsigned char)SOURCE_ALTITUDE, command_number, parameter_one, parameter_two);
			altitude_serial.write(Outgoing_Message_to_Motor_Drivers.sg_chars[i]);
		}
		if (motors == (unsigned char)SOURCE_AZIMUTH) {
			write_logger((unsigned char)NULL,(unsigned char)TO, (unsigned char)SOURCE_AZIMUTH, command_number, parameter_one, parameter_two);
			azimuth_serial.write(Outgoing_Message_to_Motor_Drivers.sg_chars[i]);
		}
		if (motors == (unsigned char)SOURCE_ALTAZI) {
			write_logger((unsigned char)NULL,(unsigned char)TO, (unsigned char)SOURCE_ALTAZI, command_number, parameter_one, parameter_two);
			altitude_serial.write(Outgoing_Message_to_Motor_Drivers.sg_chars[i]);
			azimuth_serial.write(Outgoing_Message_to_Motor_Drivers.sg_chars[i]);
		}
	}
	Serial.println("");
}
// Send Packet to Focuser -----------------------------------------------------------------------------------
void Send_Packet_to_Focuser() {
	unsigned char command_number = Outgoing_Message_to_Focuser.contents.command_number;
	double parameter_one = Outgoing_Message_to_Focuser.contents.parameter_one;
	double parameter_two = Outgoing_Message_to_Focuser.contents.parameter_two;
	write_logger((unsigned char) SOURCE_CONTROLLER,(unsigned char)TO, (unsigned char)SOURCE_FOCUSER, command_number, parameter_one, parameter_two);
	focuser_packet_count++;
	if (focuser_packet_count > 99) focuser_packet_count = 1;
	Serial.print(millis(), DEC);
	Serial.print("\tFocuser Message to be sent: ");
	for (int i = 0; i < Message_Structure_Length; i++) {
		Serial.print(Outgoing_Message_to_Focuser.sg_chars[i], HEX);
		Serial.print(",");
	}
	Serial.println();
	Serial.print("Message Sent to Focuser:");
	for (int i = 0; i < Message_Structure_Length; i++) {
		focuser_serial.write(Outgoing_Message_to_Focuser.sg_chars[i]);
	}
	Serial.println("");
}
// Send Packet to Congroller --------------------------------------------------------------------------------
void Send_Packet_to_Controller() {
	unsigned char command_number = Outgoing_Message_to_Controller.contents.command_number;
	double parameter_one = Outgoing_Message_to_Controller.contents.parameter_one;
	double parameter_two = Outgoing_Message_to_Controller.contents.parameter_two;
	write_logger((unsigned char)NULL,(unsigned char)TO, (unsigned char)SOURCE_CONTROLLER, command_number, parameter_one, parameter_two);
#ifdef MONITOR_PRINT_CONTROLLER
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
#ifdef MONITOR_PRINT_CONTROLLER
	Serial.print("Message Sent to Controller ");
#endif
	for (int i = 0; i < Message_Structure_Length; i++) {
#ifdef MONITOR_PRINT_CONTROLLER
		Serial.print(Outgoing_Message_to_Controller.sg_chars[i], HEX);
		Serial.print(",");
#endif
#ifdef ETHERNET
		server.write(Outgoing_Message_to_Controller.sg_chars[i]);
#endif
	}
#ifdef MONITOR_PRINT_CONTROLLER
	Serial.println("");
#endif
}
// LED Functions --------------------------------------------------------------------------------------------
void Turn_Red_LED(bool on_off) {
	if (on_off == false) {
		digitalWrite(RED_LED_pin, LOW);
		Red_LED_State = false;
	}
	if (hub_display_state == true) {
		digitalWrite(RED_LED_pin, HIGH);
		Red_LED_Time_On = millis();
		Red_LED_State = true;
	}
}
void Turn_Orange_LED(bool on_off) {
	if (on_off == false) {
		digitalWrite(ORANGE_LED_pin, LOW);
		Orange_LED_State = false;
	}
	if (hub_display_state == true) {
		digitalWrite(ORANGE_LED_pin, HIGH);
		Orange_LED_Time_On = millis();
		Orange_LED_State = true;
	}
}
void Turn_Green_LED(bool on_off) {
	if (on_off == false) {
		digitalWrite(GREEN_LED_pin, LOW);
		Green_LED_State = false;
	}
	if (hub_display_state == true) {
		digitalWrite(RED_LED_pin, HIGH);
		Green_LED_Time_On = millis();
		Green_LED_State = true;
	}
}
// send update to SEVEN_SEGMENT -----------------------------------------------------------------------------
void send_update_to_SEVEN_SEGMENT() {
	char displaytelescope_s[2];
	char displaycontroller_s[2];
	char displayfocuser_s[2];
	if (hub_display_state == false) return;
	String Display_String = "        ";
	itoa((int)telescope_packet_count, displaytelescope_s, 10);
	Display_String = displaytelescope_s;
	Display_String += '-';
	itoa((int)controller_packet_count, displaycontroller_s, 10);
	Display_String += displaycontroller_s;
	Display_String += '-';
	itoa((int)focuser_packet_count, displayfocuser_s, 10);
	Display_String += displayfocuser_s;
	HUB_display(Display_String);
#ifdef MONITOR_PRINT_SEVEN_SEGMENT
	Serial.print(millis(), DEC); Serial.print("Seven Segment Display: "); Serial.println(Display_String);
#endif
}
// Publish an Error -----------------------------------------------------------------------------------------
void error(double error_number) {
	char error_number_s[2];
	if (hub_display_state == false) return;
	dtostrf(error_number, 2, 0, error_number_s);
	sprintf(display_string, "%s", error_number_s);
	Serial.print("Error Number: ");	Serial.println(display_string);
	HUB_error_display(display_string);
}
// End of Programme -----------------------------------------------------------------------------------------
