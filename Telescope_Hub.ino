/*
  Arduino Ethernet Telescope Hub
        interfaces Steve and Jamie Gould's Telescope to a PC ASCOM compliant software driver
        Declination = Altitude = north/south = up down
        Right Ascension = Azimuth  = east/west = left right
        Communications to the three motor controllers is made through this HUB.
        The HUB also drives an operator's panel with three push switches (with lights),
            3 Seven Segment displays, one each for Altitude Motor, Azimuth Motor and Focuser
            Motor, 12 LEDs, three sets of Red, Green, Blue and Yellow, a 4 x 20 characters/row
            LCD display

        Version 1.0 30/01/2018
*/
/* Version Control ------------------------------------------------------------------------------------------
    Version	Date		Description
    1.		27/01/2018
    1.1		11/05/2021	Updated to be compatible with Telescope and Focuser
    1.2		06/11/2021	Introduction of Panel Functionality, Code Tidy up
    1.3		14/07/2022	Code Tidy Up, made compatible with current telescope commands, status display now via API
    1.4     19/08/2022  Log File Support Added
    1.5     21/09/2022  Log File replaced with logging to serial line 4
    1.6     05/02/2023  Recommenced review
    1.7     16/02/2023  Added Pseudo serial connector so that the Exerciser can emulate the PC_AP!
*/
constexpr double Firmware_Version = (double)1.6;
// Inclusions ---------------------------------------------------------------------------------------------------------
#include <avr/wdt.h>
#include <Bounce2.h>
#include <DHT_U.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Softwareserial.h>
#include <Ethernet2.h>
#include <Hardwareserial.h>
#include <C:\Users\Stephen\Dropbox\Projects\Combined_Telescope\Common_Files\Telescope_Commands.h>
// BUILD Switches/Definitions -----------------------------------------------------------------------------------------
#define USE_CONSOLE                   // output messages to console
#define PRINT_RECEIVED
#define PRINT_TRANSMITTED
// Constants ----------------------------------------------------------------------------------------------------------
constexpr int machine = axisTertiary;
int freeMemory() {
    extern char* __brkval;
    char top;
    return __brkval ? &top - __brkval : &top - __malloc_heap_start;
}
constexpr double MAJORVERSION = 1;
constexpr double MINORVERSION = 5;
#define console Serial
constexpr int Altitude_baud = (int)38400;
constexpr int Azimuth_baud = (int)38400;
constexpr int Focuser_baud = (int)38400;
constexpr int Panel_baud = (int)38400;
constexpr int Emulator_baud = (int)38400;
constexpr int Camera_baud = (int)38400;
constexpr unsigned long Led_On_Time = (unsigned long)250;
// Constants ------------------------------------------------------------------
//byte mac[] = { 0x90, 0xA2, 0xDA, 0x11, 0x23, 0x40 };	// MAC address of Ethernet controller, found on a sticker on the back of the Ethernet shield.
//byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };	// MAC address of Jamie's Ethernet PC_API
//IPAddress ip(92, 68, 0, 30);							// IP Address (FIXED) of this server
//IPAddress gateway(92, 68, 0, 00);
//IPAddress subnet(255, 255, 255, 0);

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);
// Hardware configuration ---------------------------------------------------------------------------------------------
constexpr byte Voltage_pin = A2;		// A2	motor_voltage
constexpr byte Green_led_pin = 03;      // Green led
constexpr byte Temperature_pin = 04;	// ambient temperature pin
constexpr byte TX_0_pin = 1;            // console output
constexpr byte RX_0_pin = 0;            // console input
constexpr byte TX_1_pin = 18;           // Altitude Port TX
constexpr byte RX_1_pin = 19;           // Altitude Port RX
constexpr byte TX_2_pin = 16;           // Azimuth Port TX
constexpr byte RX_2_pin = 17;           // Azimuth Port RX
constexpr byte TX_3_pin = 14;           // Focuser Port TX
constexpr byte RX_3_pin = 15;           // Focuser Port RX 
constexpr byte Panel_TX_pin = 12;       // Control Panel Port TX
constexpr byte Panel_RX_pin = 13;       // Control Panel Port RX,10,11,12,13,14,15,50,51,52,53,A8(62),A9(63),A10(64),A11(65),A12(66),A13(67),A14(68),A15(69
constexpr byte Emulator_TX_pin = 10;    // TX Port for incoming PC_API messages when not using Ethernet
constexpr byte Emulator_RX_pin = 11;    // RX Port for incoming PC_API messages when not using Ethernet
constexpr byte Camera_TX_pin = A8;      // TX Port fpr incoming Camera Packets
constexpr byte Camera_RX_pin = A9;      // RX Port for incoming Camera Packets
// --------------------------------------------------------------------------------------------------------------------
unsigned int localPort = 8888;          // local port to listen on
// --------------------------------------------------------------------------------------------------------------------
double Ambient_Temperature = 0;			// temperature value
double Ambient_Humidity = 0;
double Motor_Voltage = 0;
double Altitude_Packets_Received = 0;
double Azimuth_Packets_Received = 0;
double Focuser_Packets_Received = 0;
double Panel_Packets_Received = 0;
double PC_API_Packets_Received = 0;
double Emulator_Packets_Received = 0;
double Camera_Packets_Received = 0;
double Packets_Sent_to_Altitude = 0;
double Packets_Sent_to_Azimuth = 0;
double Packets_Sent_to_Focuser = 0;
double Packets_Sent_to_PC_API = 0;
double Packets_Sent_to_Panel = 0;
double Packets_Sent_to_Emulator = 0;
double Packets_Sent_to_Camera = 0;
// byte messageBuffer[48]; // 48 byte array to hold incoming/outgoing NTP time messages
// Instantiations -------------------------------------------------------------
HardwareSerial Altitude_Port = Serial1;                     // Altitude Port
HardwareSerial Azimuth_Port = Serial2;                      // Azimuth Port
HardwareSerial Focuser_Port = Serial3;                      // Focuser Port
SoftwareSerial Panel_Port(Panel_RX_pin, Panel_TX_pin);   // Logge Port RX, TX
SoftwareSerial Emulator_Port(Emulator_RX_pin, Emulator_TX_pin);
SoftwareSerial Camera_Port(Camera_RX_pin, Camera_TX_pin);
EthernetServer PC_API_Port(80);                                  // Create a server listening on port 80.
// Communications Variables ---------------------------------------------------
PacketUnion Incoming_Message_from_Azimuth;
PacketUnion Incoming_Message_from_PC_API;
PacketUnion Incoming_Message_from_Altitude;
PacketUnion Incoming_Message_from_Focuser;
PacketUnion Incoming_Message_from_Panel;
PacketUnion Incoming_Message_from_Emulator;
PacketUnion Incoming_Message_from_Camera;
// PacketUnion Outgoing_Message_to_Azimuth;
// PacketUnion Outgoing_Message_to_Altitude;
// PacketUnion Outgoing_Message_to_Focuser;
PacketUnion Outgoing_Message_to_PC_API;
PacketUnion Outgoing_Message_to_Panel;
PacketUnion Outgoing_Message_to_Emulator;
// PacketUnion Outgoing_Message_to_Camera;
bool Altitude_Incoming_Message_Available = false;
bool Azimuth_Incoming_Message_Available = false;
bool Focuser_Incoming_Message_Available = false;
bool PC_API_Incoming_Message_Available = false;
bool Emulator_Incoming_Message_Available = false;
bool Panel_Incoming_Message_Available = false;
bool Camera_Incoming_Message_Available = false;
char Altitude_in_buffer_counter = 0;
char Azimuth_in_buffer_counter = 0;
char Focuser_in_buffer_counter = 0;
char Control_panel_in_buffer_counter = 0;
char Emulator_in_buffer_couner = 0;
char Camera_in_buffer_counter = 0;
byte Altitude_inptr;					// must be 8 bit byte so that it overflows at 256
byte Altitude_outptr;					// must be 8 bit byte so that it overflows at 256
unsigned char Altitude_inbuffer[0xff];
int Altitude_string_ptr;
byte Azimuth_inptr;						// must be 8 bit byte so that it overflows at 256
byte Azimuth_outptr;					// must be 8 bit byte so that it overflows at 256
unsigned char Azimuth_inbuffer[0xff];
int Azimuth_string_ptr;
byte Focuser_inptr;
byte Focuser_outptr;
unsigned char Focuser_inbuffer[0xff];
int Focuser_string_ptr;
byte Panel_inptr;					// must be 8 bit byte so that it overflows at 256
byte Panel_outptr;					// must be 8 bit byte so that it overflows at 256
unsigned char Panel_inbuffer[0xff];
int Panel_string_ptr;
byte Emulator_inptr;
byte Emulator_outptr;
int Emulator_string_ptr;
unsigned char Emulator_inbuffer[0xff];
byte Camera_inptr;					// must be 8 bit byte so that it overflows at 256
byte Camera_outptr;					// must be 8 bit byte so that it overflows at 256
unsigned char Camera_inbuffer[0xff];
int Camera_string_ptr;
// --------------------------------------------------------------------------------------------------------------------
unsigned long Time_of_Last_Heartbeat = 0;
unsigned long Green_Led_Start_Time = 0;
// Instantiations -----------------------------------------------------------------------------------------------------
DHT_Unified Temperature_sensor(Temperature_pin, DHT22);
DHT_Unified Humidity_sensor(Temperature_pin, DHT22);
sensors_event_t event;
sensor_t sensor;
Bounce Reset_button = Bounce();
//-- Setup ------------------------------------------------------------------------------------------------------------
void setup() {
    console.begin(115200);
#ifdef USE_CONSOLE
    console_print("\tSetup Commenced");
#endif
    PC_API_Port.begin();                                                 // Start Ethernet
    Emulator_Port.begin(Emulator_baud);
    Emulator_Port.flush();
    Camera_Port.begin(Camera_baud);
    Camera_Port.flush();
    pinMode(Green_led_pin, OUTPUT);
    digitalWrite(Green_led_pin, LOW);
    pinMode(Temperature_pin, INPUT);
    pinMode(Voltage_pin, INPUT);
    Temperature_sensor.begin();
    Temperature_sensor.temperature().getSensor(&sensor);
    Temperature_sensor.humidity().getSensor(&sensor);
    Altitude_Port.begin(Altitude_baud, SERIAL_8N2);					// initialise the Altitude serial port    
    Altitude_Port.flush();                                          // clear the Altitude serial buffer
    Azimuth_Port.begin(Azimuth_baud, SERIAL_8N2);					// initialise the Azimuth serial port
    Azimuth_Port.flush();											// clear the Azimuth serial buffer
    Focuser_Port.begin(Focuser_baud, SERIAL_8N2);					// initialise the Focuser serial port
    Focuser_Port.flush();											// clear the Focuser serial buffer
    Panel_Port.begin(Panel_baud);
    Panel_Port.flush();
    // ----------------------------------------------------------------------------------------------------------------
#ifdef USE_CONSOLE
    console_print("Setup Complete");
#endif
    wdt_enable(WDTO_1S);
} // end setup
// Main ---------------------------------------------------------------------------------------------------------------
void loop() {
    wdt_reset();                                                                            // keep watch dog timer active
    Maintain_Internet();
    Green_Led_Flash();                                                                      // toggle the green led
    if (Check_PC_API_Packet()) Process_Incoming_Packet_from_PC_API();
    if (Check_Emulator_Packet()) Process_Incoming_Packet_from_Emulator();
    if (Check_Altitude_Packet()) Process_Incoming_Packet_from_Altitude();
    if (Check_Azimuth_Packet()) Process_Incoming_Packet_from_Azimuth();
    if (Check_Focuser_Packet()) Process_Incoming_Packet_from_Focuser();
    if (Check_Panel_Packet()) Process_Incoming_Packet_from_Panel();
    if (Check_Camera_Packet()) Process_Incoming_Packet_from_Camera();
    if (millis() >= Time_of_Last_Heartbeat + Heartbeat_Period) Send_Heartbeat();
} // end of main loop -------------------------------------------------------------------------------------------------
void console_print(String message) {
#ifdef USE_CONSOLE
    console_print(message);
#endif
}
void Maintain_Internet() {
    int ethernet_status = (int)Ethernet.maintain();				// keep ethernet link open
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
}
void Send_Heartbeat() {
#ifdef USE_CONSOLE
    console_print("Transmit Heartbeat");
#endif
    UpdateEnvironmentalSensors();
    Outgoing_Message_to_PC_API.field.Header = STX;	                                // [0] STX
    Outgoing_Message_to_PC_API.field.MessageTarget = Device_PC_API;	                // [1] source of message
    Outgoing_Message_to_PC_API.field.MessageSource = Device_Hub;                    // [2] target for message
    Outgoing_Message_to_PC_API.field.CommandNumber = Heartbeat;		                // [3] command character
    Outgoing_Message_to_PC_API.field.PacketType = HRT;		                        // [4] packet type  
    Outgoing_Message_to_PC_API.field.CurrentStatus = Hub_status.word;               // [5 - 6]
    Outgoing_Message_to_PC_API.field.ParameterOne = Ambient_Temperature;            // [7 - 10]
    Outgoing_Message_to_PC_API.field.ParameterTwo = Motor_Voltage;                  // [11 - 14]
    Outgoing_Message_to_PC_API.field.ParameterThree = Altitude_Packets_Received;	// [15 - 18]
    Outgoing_Message_to_PC_API.field.ParameterFour = Azimuth_Packets_Received;	    // [19 - 22]
    Outgoing_Message_to_PC_API.field.ParameterFive = Focuser_Packets_Received;      // [23 - 26]
    Outgoing_Message_to_PC_API.field.ParameterSix = PC_API_Packets_Received;        // [27 - 30]
    Outgoing_Message_to_PC_API.field.Footer;			                            // [31] ETX
    for (int i = 0; i <= packet_length; i++) {                                      // copy the input packet to the output packet buffer
        Emulator_Port.write(Outgoing_Message_to_PC_API.character[i]);
        PC_API_Port.write(Outgoing_Message_to_PC_API.character[i]);
        Panel_Port.write(Outgoing_Message_to_PC_API.character[i]);
    }
    Time_of_Last_Heartbeat = millis();
#ifdef USE_CONSOLE
    console_print("Heartbeat Transmtted");
#endif
}
bool Check_Panel_Packet(void) {
    Panel_Port.listen();
    while (Panel_Port.available()) {
        Panel_inbuffer[Panel_inptr++] = (unsigned char)Panel_Port.read();        // add the received characters to the buffer and increment characters count
    }
    while (Panel_outptr != Panel_inptr) {                                       // check Altitude serial buffer for data
        Hub_status.bit.Panel = true;
        char thisbyte = Panel_inbuffer[Panel_outptr++];                         // take a characters from the input buffer and increment pointer
        if ((thisbyte == (char)STX) && (Panel_string_ptr == 0)) {                       // look for the STX, but only if the output string is empty
            Incoming_Message_from_Panel.character[Panel_string_ptr++] = STX;    // store the STX and increment the string pointer
            Panel_Incoming_Message_Available = false;
        }
        else {
            if (thisbyte == (char)ETX) {												        // characters was not an STX check for ETX
                Incoming_Message_from_Panel.character[Panel_string_ptr++] = ETX;// save the ETX and increment the string pointer
                if (Panel_string_ptr == packet_length) {                                // does it mean end of packet (we just saved the ETX at 20!
                    Panel_string_ptr = 0;                                               // zero the string pointer
                    Panel_Incoming_Message_Available = true;
                }
            }
            else {
                Incoming_Message_from_Panel.character[Panel_string_ptr++] = thisbyte;       // Not a valid STX or a valid ETX so save it and increment string pointer
            }
        }
    } // end of while Altitude
    return Panel_Incoming_Message_Available;
}
bool Check_Altitude_Packet(void) {
    while (Altitude_outptr != Altitude_inptr) { // check Altitude serial buffer for data
        Hub_status.bit.Altitude = true;
        char thisbyte = Altitude_inbuffer[Altitude_outptr++];                           // take a characters from the input buffer and increment pointer
        if ((thisbyte == (char)STX) && (Altitude_string_ptr == 0)) {                    // look for the STX, but only if the output string is empty
            Incoming_Message_from_Altitude.character[Altitude_string_ptr++] = STX;      // store the STX and increment the string pointer
            Altitude_Incoming_Message_Available = false;
        }
        else {
            if (thisbyte == (char)ETX) {												// characters was not an STX check for ETX
                Incoming_Message_from_Altitude.character[Altitude_string_ptr++] = ETX;  // save the ETX and increment the string pointer
                if (Altitude_string_ptr == packet_length) {                             // does it mean end of packet (we just saved the ETX at 20!
                    Altitude_string_ptr = 0;                                            // zero the string pointer
                    Altitude_Incoming_Message_Available = true;
                }
            }
            else {
                Incoming_Message_from_Altitude.character[Altitude_string_ptr++] = thisbyte;       // Not a valid STX or a valid ETX so save it and increment string pointer
            }
        }
    } // end of while Altitude
    return Altitude_Incoming_Message_Available;
}
bool Check_Azimuth_Packet(void) {
    while (Azimuth_outptr != Azimuth_inptr) {                                        // check Altitude serial buffer for data
        Hub_status.bit.Azimuth = true;
        char thisbyte = Azimuth_inbuffer[Azimuth_outptr++];                           // take a characters from the input buffer and increment pointer
        if ((thisbyte == (char)STX) && (Azimuth_string_ptr == 0)) {                    // look for the STX, but only if the output string is empty
            Incoming_Message_from_Azimuth.character[Azimuth_string_ptr++] = STX;                // store the STX and increment the string pointer
        }
        else {
            if (thisbyte == (char)ETX) {												// characters was not an STX check for ETX
                Incoming_Message_from_Azimuth.character[Azimuth_string_ptr++] = ETX;            // save the ETX and increment the string pointer
                if (Azimuth_string_ptr == packet_length) {                                        // does it mean end of packet (we just saved the ETX at 20!
                    Azimuth_string_ptr = 0;                                            // zero the string pointer
                    Azimuth_Incoming_Message_Available = true;
                }
            }
            else {
                Incoming_Message_from_Azimuth.character[Azimuth_string_ptr++] = thisbyte;           // Not a valid STX or a valid ETX so save it and increment string pointer
            }
        }
    } // end of while Azimuth
    return Azimuth_Incoming_Message_Available;
}
bool Check_Focuser_Packet(void) {
    while (Focuser_outptr != Focuser_inptr) {											// check Focuser serial buffer for data
        Hub_status.bit.Focuser = true;
        char thisbyte = Focuser_inbuffer[Focuser_outptr++];								// take a characters from the input buffer and increment pointer
        if ((thisbyte == (char)STX) && (Focuser_string_ptr == 0)) {						// look for the STX, but only if the output string is empty
            Incoming_Message_from_Focuser.character[Focuser_string_ptr++] = STX;		// store the STX and increment the string pointer
            Focuser_Incoming_Message_Available = false;
        }
        else {
            if (thisbyte == (char)ETX) {												// characters was not an STX check for ETX
                Incoming_Message_from_Focuser.character[Focuser_string_ptr++] = ETX;     // save the ETX and increment the string pointer
                if (Focuser_string_ptr == packet_length) {  // does it mean end of packet (we just saved the ETX at 20!
                    Focuser_string_ptr = 0;												// zero the string pointer
                    Focuser_Incoming_Message_Available = true;
                }
            }
            else {
                Incoming_Message_from_Focuser.character[Focuser_string_ptr++] = thisbyte; // Not a valid STX or a valid ETX so save it and increment string pointer
            }
        }
    } // end of while Focuser
    return Focuser_Incoming_Message_Available;
}
bool Check_Emulator_Packet(void) {
    Emulator_Port.listen();
    while (Emulator_outptr != Emulator_inptr) {											// check Focuser serial buffer for data
        Hub_status.bit.Emulator = true;
        char thisbyte = Emulator_inbuffer[Emulator_outptr++];								// take a characters from the input buffer and increment pointer
        if ((thisbyte == (char)STX) && (Emulator_string_ptr == 0)) {						// look for the STX, but only if the output string is empty
            Incoming_Message_from_Emulator.character[Emulator_string_ptr++] = STX;		// store the STX and increment the string pointer
            Emulator_Incoming_Message_Available = false;
        }
        else {
            if (thisbyte == (char)ETX) {												// characters was not an STX check for ETX
                Incoming_Message_from_Emulator.character[Emulator_string_ptr++] = ETX;     // save the ETX and increment the string pointer
                if (Emulator_string_ptr == packet_length) {  // does it mean end of packet (we just saved the ETX at 20!
                    Emulator_string_ptr = 0;												// zero the string pointer
                    Emulator_Incoming_Message_Available = true;
                }
            }
            else {
                Incoming_Message_from_Emulator.character[Emulator_string_ptr++] = thisbyte; // Not a valid STX or a valid ETX so save it and increment string pointer
            }
        }
    } // end of while Focuser
    return PC_API_Incoming_Message_Available;
}
bool Check_PC_API_Packet(void) {
    EthernetClient client = PC_API_Port.available();                  // Listen for incoming client requests.
    if (client) {
        for (int i = 1; i < packet_length - 1; i++) {
            Incoming_Message_from_PC_API.character[i] = client.read();
        }
        PC_API_Incoming_Message_Available = true;
        return true;
    }
    return false;
}
bool Check_Camera_Packet(void) {
    Camera_Port.listen();
    while (Camera_outptr != Camera_inptr) { // check Altitude serial buffer for data
        Hub_status.bit.Camera = true;
        char thisbyte = Camera_inbuffer[Camera_outptr++];                           // take a characters from the input buffer and increment pointer
        if ((thisbyte == (char)STX) && (Camera_string_ptr == 0)) {                    // look for the STX, but only if the output string is empty
            Incoming_Message_from_Camera.character[Camera_string_ptr++] = STX;      // store the STX and increment the string pointer
            Camera_Incoming_Message_Available = false;
        }
        else {
            if (thisbyte == (char)ETX) {												// characters was not an STX check for ETX
                Incoming_Message_from_Camera.character[Camera_string_ptr++] = ETX;  // save the ETX and increment the string pointer
                if (Camera_string_ptr == packet_length) {                             // does it mean end of packet (we just saved the ETX at 20!
                    Camera_string_ptr = 0;                                            // zero the string pointer
                    Camera_Incoming_Message_Available = true;
                }
            }
            else {
                Incoming_Message_from_Camera.character[Camera_string_ptr++] = thisbyte;       // Not a valid STX or a valid ETX so save it and increment string pointer
            }
        }
    } // end of while Camera
    return Camera_Incoming_Message_Available;
}
void serialEvent1() {
    while (Altitude_Port.available()) {
        Altitude_inbuffer[Altitude_inptr++] = (unsigned char)Altitude_Port.read();        // add the received characters to the buffer and increment characters count
    }
}
void serialEvent2() {
    while (Azimuth_Port.available()) {
        Azimuth_inbuffer[Azimuth_inptr++] = (unsigned char)Azimuth_Port.read();        // add the received characters to the buffer and increment characters count
    }
}
void serialEvent3() {
    while (Focuser_Port.available()) {
        Focuser_inbuffer[Focuser_inptr++] = (unsigned char)Focuser_Port.read();        // add the received characters to the buffer and increment characters count
    }
}
// Process Received Packets -------------------------------------------------------------------------------------------
void Process_Incoming_Packet_from_Azimuth() {					            // process an update message from the Azimuth motor driver
    Azimuth_Incoming_Message_Available = false;                             // clear the packet received flag
    Azimuth_Packets_Received++;                                             // increment the Aimuth packets received count
    for (int i = 0; i <= packet_length; i++) {                              // copy the received packet to the output devices
        Emulator_Port.write(Incoming_Message_from_Azimuth.character[i]);    // send to the emulator
        PC_API_Port.write(Incoming_Message_from_Azimuth.character[i]);      // else send to the PC_API
        Panel_Port.write(Incoming_Message_from_Azimuth.character[i]);       // also send to the Panel
    }
#ifdef PRINT_RECEIVED
    console.print(millis(), DEC); console.println("\tPacket Received from Azimuth");
    console.print("\t\tMessage Source: "); console.print(Device_Names[Incoming_Message_from_Azimuth.field.MessageSource]); // [1] source of message
    console.print("\t\tMessage Target: "); console.print(Device_Names[Incoming_Message_from_Altitude.field.MessageTarget]); // [1] source of message
    ***********************************************
        byte MessageSource;     // [2]
    byte CommandNumber;		// [3] command character
    byte PacketType;		// [4]  
    int CurrentStatus;		// [5 - 6]
    double ParameterOne;	// [7 - 10]
    double ParameterTwo;	// [11 - 14]
    double ParameterThree;	// [15 - 18]
    double ParameterFour;	// [19 - 22]
    double ParameterFive;	// [23 - 26]
    double ParameterSix;	// [27 - 30]
#endif
#ifdef USE_CONSOLE
    console_print("\tPacket Received from Azimuth forwarded to PC_API, Emulator and Panel");
#endif
}
void Process_Incoming_Packet_from_Altitude() {					            // process an update message from a motor driver
    Altitude_Incoming_Message_Available = false;                            // clear the packet received flag
    Altitude_Packets_Received++;                                            // increment the Altitude packets received count
    for (int i = 0; i <= packet_length; i++) {                              // copy the received packet to the output devices
        Emulator_Port.write(Incoming_Message_from_Altitude.character[i]);   // send to the emulator
        PC_API_Port.write(Incoming_Message_from_Altitude.character[i]);     // also send to the PC_API
        Panel_Port.write(Incoming_Message_from_Altitude.character[i]);      // also send to the Panel
    }
#ifdef USE_CONSOLE
    console_print("\tPacket Received from Altitude forwarded to PC_API, Emulator and Panel");
#endif
}
void Process_Incoming_Packet_from_Focuser() {                               // process an update message from the Focuser
    Focuser_Incoming_Message_Available = false;                             // clear the packet received flag
    Focuser_Packets_Received++;                                             // increment the Focuser packets received count
    for (int i = 0; i <= packet_length; i++) {                              // copy the received packet to the output devices
        Emulator_Port.write(Incoming_Message_from_Focuser.character[i]);    // send to the emulator
        PC_API_Port.write(Incoming_Message_from_Focuser.character[i]);      // also send to the PC_API
        Panel_Port.write(Incoming_Message_from_Focuser.character[i]);       // also send to the Panel
    }
#ifdef USE_CONSOLE
    console_print("\tPacket Received from Focuser forwarded to PC_API, Emulator and Panel");
#endif
}
void Process_Incoming_Packet_from_PC_API() {                                // Process a pcket from the PC_API or the Emulator  
    PC_API_Incoming_Message_Available = false;                              // clear rhe packet received flag
    PC_API_Packets_Received++;                                              // increment the PC_API packets received count
    if (Incoming_Message_from_PC_API.field.MessageTarget == (byte)Device_Hub) {     // Packet should be processed by the HUB
        switch ((int)Incoming_Message_from_PC_API.field.CommandNumber) {            // Process cpmmand number    
        case (int)Request_Firmware_Version:                                         // request hub firmware version
#ifdef USE_CONSOLE
            console_print("\tPacket received from PC_API, Hub Status Requested");
#endif
            UpdateEnvironmentalSensors();                                           // update the environmental variables
            Outgoing_Message_to_PC_API.field.MessageTarget = Device_PC_API;         // prepare reply packet
            Outgoing_Message_to_PC_API.field.MessageSource = Device_Hub;
            Outgoing_Message_to_PC_API.field.CommandNumber = Request_Firmware_Version;
            Outgoing_Message_to_PC_API.field.PacketType = REP;
            Outgoing_Message_to_PC_API.field.CurrentStatus = Hub_status.word;
            Outgoing_Message_to_PC_API.field.ParameterOne = (double)Firmware_Version;
            Outgoing_Message_to_PC_API.field.ParameterTwo = (double)Ambient_Temperature;
            Outgoing_Message_to_PC_API.field.ParameterThree = (double)Ambient_Humidity;
            Outgoing_Message_to_PC_API.field.ParameterFour = (double)Motor_Voltage;
            Outgoing_Message_to_PC_API.field.ParameterFive = (double)freeMemory();
            Outgoing_Message_to_PC_API.field.ParameterSix = (double)0;
            for (int i = 0; i <= packet_length; i++) {
                PC_API_Port.write(Outgoing_Message_to_PC_API.character[i]);         // send reply packet to PC_API
                Emulator_Port.write(Outgoing_Message_to_PC_API.character[i]);       // send to Emulator
                Panel_Port.write(Outgoing_Message_to_PC_API.character[i]);          // send to Panel
            }
            break;
        case (int)Request_Status: {
#ifdef USE_CONSOLE
            console_print("\tPacket received from PC_API, Hub Status Requested");
#endif
            UpdateEnvironmentalSensors();
            Outgoing_Message_to_PC_API.field.MessageTarget = Device_PC_API;
            Outgoing_Message_to_PC_API.field.MessageSource = Device_Hub;
            Outgoing_Message_to_PC_API.field.CommandNumber = Firmware_Version;
            Outgoing_Message_to_PC_API.field.PacketType = REP;
            Outgoing_Message_to_PC_API.field.CurrentStatus = Hub_status.word;
            Outgoing_Message_to_PC_API.field.ParameterOne = (double)Firmware_Version;
            Outgoing_Message_to_PC_API.field.ParameterTwo = (double)Ambient_Temperature;
            Outgoing_Message_to_PC_API.field.ParameterThree = (double)Ambient_Humidity;
            Outgoing_Message_to_PC_API.field.ParameterFour = (double)Motor_Voltage;
            Outgoing_Message_to_PC_API.field.ParameterFive = (double)freeMemory();
            Outgoing_Message_to_PC_API.field.ParameterSix = (double)0;
            for (int i = 0; i <= packet_length; i++) {
                PC_API_Port.write(Outgoing_Message_to_PC_API.character[i]);
                Emulator_Port.write(Outgoing_Message_to_PC_API.character[i]);
                Panel_Port.write(Outgoing_Message_to_PC_API.character[i]);
            }
            break;
        }
        case (int)Request_Traffic: {
#ifdef USE_CONSOLE
            console_print("\tPacket received from PC_API, Hub Traffic Requested");
#endif
            UpdateEnvironmentalSensors();
            Outgoing_Message_to_PC_API.field.MessageTarget = Device_PC_API;
            Outgoing_Message_to_PC_API.field.MessageSource = Device_Hub;
            Outgoing_Message_to_PC_API.field.CommandNumber = Request_Traffic;
            Outgoing_Message_to_PC_API.field.PacketType = REP;
            Outgoing_Message_to_PC_API.field.CurrentStatus = Hub_status.word;
            Outgoing_Message_to_PC_API.field.ParameterOne = Altitude_Packets_Received;
            Outgoing_Message_to_PC_API.field.ParameterTwo = Azimuth_Packets_Received;
            Outgoing_Message_to_PC_API.field.ParameterThree = Focuser_Packets_Received;
            Outgoing_Message_to_PC_API.field.ParameterFour = PC_API_Packets_Received;
            Outgoing_Message_to_PC_API.field.ParameterFive = Panel_Packets_Received;
            for (int i = 0; i <= packet_length; i++) {
                PC_API_Port.write(Outgoing_Message_to_PC_API.character[i]);
                Emulator_Port.write(Outgoing_Message_to_PC_API.character[i]);
                Panel_Port.write(Outgoing_Message_to_PC_API.character[i]);
            }
            break;
        }
        case (int)Request_Reset: {
#ifdef USE_CONSOLE
            console_print("\tPacket received from PC_API, Reset Requested");
#endif
            delay(200);
            while (1);                      // causes watchdog driven reboot
        }
        }       // end of switch commandnumber
    }
    else if (Incoming_Message_from_PC_API.field.MessageTarget == (byte)Device_Altitude) {
        for (int i = 0; i <= packet_length; i++) {
            Altitude_Port.write(Incoming_Message_from_PC_API.character[i]);
            Emulator_Port.write(Incoming_Message_from_PC_API.character[i]);
            Panel_Port.write(Incoming_Message_from_PC_API.character[i]);
        }
#ifdef USE_CONSOLE
        console_print("\tPacket Received from PC_API forwarded to Altitude");
#endif
    }
    else if (Incoming_Message_from_PC_API.field.MessageTarget == (byte)Device_Azimuth) {
        for (int i = 0; i <= packet_length; i++) {
            Azimuth_Port.write(Incoming_Message_from_PC_API.character[i]);
            Emulator_Port.write(Incoming_Message_from_PC_API.character[i]);
            Panel_Port.write(Incoming_Message_from_PC_API.character[i]);
        }
#ifdef USE_CONSOLE
        console_print("\tPacket Received from PC_API forwarded to Azimuth");
#endif
    }
    else if (Incoming_Message_from_PC_API.field.MessageTarget == (byte)Device_Focuser) {
        for (int i = 0; i <= packet_length; i++) {
            Focuser_Port.write(Incoming_Message_from_PC_API.character[i]);
            Emulator_Port.write(Incoming_Message_from_PC_API.character[i]);
            Panel_Port.write(Incoming_Message_from_PC_API.character[i]);
        }
#ifdef USE_CONSOLE
        console_print("\tPacket Received from PC_API forwarded to Altitude");
#endif
    }
    else if (Incoming_Message_from_PC_API.field.MessageTarget == (byte)Device_AltAzi) {
        for (int i = 0; i <= packet_length; i++) {
            Altitude_Port.write(Incoming_Message_from_PC_API.character[i]);
            Azimuth_Port.write(Incoming_Message_from_PC_API.character[i]);
            Emulator_Port.write(Incoming_Message_from_PC_API.character[i]);
            Panel_Port.write(Incoming_Message_from_PC_API.character[i]);
        }
#ifdef USE_CONSOLE
        console_print("\tPacket Received from PC_API forwarded to Altitude and Azimuth");
#endif
    }
    else if (Incoming_Message_from_PC_API.field.MessageTarget == (byte)Device_Emulator) {
        for (int i = 0; i <= packet_length; i++) {
            Emulator_Port.write(Incoming_Message_from_PC_API.character[i]);
            Panel_Port.write(Incoming_Message_from_PC_API.character[i]);
        }
#ifdef USE_CONSOLE
        console_print("\tPacket Received from PC_API forwarded to Emulator");
#endif
    }
    else if (Incoming_Message_from_PC_API.field.MessageTarget == (byte)Device_Panel) {
        for (int i = 0; i <= packet_length; i++) {
            Panel_Port.write(Incoming_Message_from_PC_API.character[i]);
            Emulator_Port.write(Incoming_Message_from_PC_API.character[i]);
        }
#ifdef USE_CONSOLE
        console_print("\tPacket Received from PC_API forwarded to Panel");
#endif
    }
    else if (Incoming_Message_from_PC_API.field.MessageTarget == (byte)Device_Camera) {
        for (int i = 0; i <= packet_length; i++) {
            Camera_Port.write(Incoming_Message_from_PC_API.character[i]);
            Emulator_Port.write(Incoming_Message_from_PC_API.character[i]);
        }
#ifdef USE_CONSOLE
        console_print("\tPacket Received from PC_API forwarded to Camera");
#endif
    }
}
void Process_Incoming_Packet_from_Emulator() {
    Emulator_Incoming_Message_Available = false;
    Emulator_Packets_Received++;
    if (Incoming_Message_from_Emulator.field.MessageTarget == (byte)Device_Hub) {           // Packet should be processed by the HUB
        switch ((int)Incoming_Message_from_Emulator.field.CommandNumber) {
        case (int)Request_Firmware_Version:
#ifdef USE_CONSOLE
            console_print("\tPacket Received from Emulator, Hub Firmware Version Requested");
#endif
            Outgoing_Message_to_Emulator.field.MessageTarget = Device_Emulator;
            Outgoing_Message_to_Emulator.field.MessageSource = Device_Hub;
            Outgoing_Message_to_Emulator.field.CommandNumber = Request_Firmware_Version;
            Outgoing_Message_to_Emulator.field.PacketType = REP;
            Outgoing_Message_to_Emulator.field.CurrentStatus = Hub_status.word;
            Outgoing_Message_to_Emulator.field.ParameterOne = (double)Firmware_Version;
            Outgoing_Message_to_Emulator.field.ParameterTwo = (double)0;
            Outgoing_Message_to_Emulator.field.ParameterThree = (double)0;
            Outgoing_Message_to_Emulator.field.ParameterFour = (double)0;
            Outgoing_Message_to_Emulator.field.ParameterFive = (double)0;
            Outgoing_Message_to_Emulator.field.ParameterSix = (double)0;
            for (int i = 0; i <= packet_length; i++) {
                Emulator_Port.write(Outgoing_Message_to_Emulator.character[i]);
                Panel_Port.write(Outgoing_Message_to_Emulator.character[i]);
            }
            break;
        case (int)Request_Status: {
#ifdef USE_CONSOLE
            console_print("\tPacket Received from Emulator, Hub Status Requested");
#endif
            UpdateEnvironmentalSensors();
            Outgoing_Message_to_Emulator.field.MessageTarget = Device_Emulator;
            Outgoing_Message_to_Emulator.field.MessageSource = Device_Hub;
            Outgoing_Message_to_Emulator.field.CommandNumber = Firmware_Version;
            Outgoing_Message_to_Emulator.field.PacketType = REP;
            Outgoing_Message_to_Emulator.field.CurrentStatus = Hub_status.word;
            Outgoing_Message_to_Emulator.field.ParameterOne = (double)Motor_Voltage;
            Outgoing_Message_to_Emulator.field.ParameterTwo = (double)freeMemory();
            Outgoing_Message_to_Emulator.field.ParameterThree = (double)0;
            Outgoing_Message_to_Emulator.field.ParameterFour = (double)0;
            Outgoing_Message_to_Emulator.field.ParameterFive = (double)0;
            Outgoing_Message_to_Emulator.field.ParameterSix = (double)0;
            for (int i = 0; i <= packet_length; i++) {
                Emulator_Port.write(Outgoing_Message_to_Emulator.character[i]);
                Panel_Port.write(Outgoing_Message_to_Emulator.character[i]);
            }
            break;
        }
        case (int)Request_Traffic: {
#ifdef USE_CONSOLE
            console_print("\tPacket Received from Emulator, Hub Traffic Requested");
#endif
            UpdateEnvironmentalSensors();
            Outgoing_Message_to_Emulator.field.MessageTarget = Device_Emulator;
            Outgoing_Message_to_Emulator.field.MessageSource = Device_Hub;
            Outgoing_Message_to_Emulator.field.CommandNumber = Request_Traffic;
            Outgoing_Message_to_Emulator.field.PacketType = REP;
            Outgoing_Message_to_Emulator.field.CurrentStatus = Hub_status.word;
            Outgoing_Message_to_Emulator.field.ParameterOne = (double)Altitude_Packets_Received;
            Outgoing_Message_to_Emulator.field.ParameterTwo = (double)Azimuth_Packets_Received;
            Outgoing_Message_to_Emulator.field.ParameterThree = (double)Focuser_Packets_Received;
            Outgoing_Message_to_Emulator.field.ParameterFour = (double)PC_API_Packets_Received;
            Outgoing_Message_to_Emulator.field.ParameterFive = (double)Panel_Packets_Received;
            Outgoing_Message_to_Emulator.field.ParameterSix = (double)Emulator_Packets_Received;
            for (int i = 0; i <= packet_length; i++) {
                Emulator_Port.write(Outgoing_Message_to_Emulator.character[i]);
                Panel_Port.write(Outgoing_Message_to_Emulator.character[i]);
            }
            break;
        }
        case (int)Request_Environment: {
#ifdef USE_CONSOLE
            console_print("\tPacket Received from Emulator, Hub Environment Requested");
#endif
            UpdateEnvironmentalSensors();
            Outgoing_Message_to_Emulator.field.MessageTarget = Device_Emulator;
            Outgoing_Message_to_Emulator.field.MessageSource = Device_Hub;
            Outgoing_Message_to_Emulator.field.CommandNumber = Request_Traffic;
            Outgoing_Message_to_Emulator.field.PacketType = REP;
            Outgoing_Message_to_Emulator.field.CurrentStatus = Hub_status.word;
            Outgoing_Message_to_Emulator.field.ParameterOne = (double)Motor_Voltage;
            Outgoing_Message_to_Emulator.field.ParameterTwo = (double)freeMemory();
            Outgoing_Message_to_Emulator.field.ParameterThree = (double)Ambient_Temperature;
            Outgoing_Message_to_Emulator.field.ParameterFour = (double)Ambient_Humidity;
            Outgoing_Message_to_Emulator.field.ParameterFive = (double)0;
            Outgoing_Message_to_Emulator.field.ParameterSix = (double)0;
            for (int i = 0; i <= packet_length; i++) {
                Emulator_Port.write(Outgoing_Message_to_Emulator.character[i]);
                Panel_Port.write(Outgoing_Message_to_Emulator.character[i]);
            }
            break;
        }
        case (int)Heartbeat: {
#ifdef USE_CONSOLE
            console_print("\tPacket Received from Emulator, Emulator Heartbeat Received");
#endif
            break;
        }
        case (int)Request_Reset: {
#ifdef USE_CONSOLE
            console_print("\tPacket Received from Emulator, Reset Requested");
#endif
            delay(200);
            while (1);                      // causes watchdog driven reboot
        }
        case (int)Get_Millis: {
#ifdef USE_CONSOLE
            console_print("\tPacket Received from Emulator, Hub Millis Requested");
#endif
            Outgoing_Message_to_Emulator.field.MessageTarget = Device_Emulator;
            Outgoing_Message_to_Emulator.field.MessageSource = Device_Hub;
            Outgoing_Message_to_Emulator.field.CommandNumber = Get_Millis;
            Outgoing_Message_to_Emulator.field.PacketType = REP;
            Outgoing_Message_to_Emulator.field.CurrentStatus = Hub_status.word;
            Outgoing_Message_to_Emulator.field.ParameterOne = (double)millis();
            Outgoing_Message_to_Emulator.field.ParameterTwo = (double)freeMemory();
            Outgoing_Message_to_Emulator.field.ParameterThree = (double)0;
            Outgoing_Message_to_Emulator.field.ParameterFour = (double)0;
            Outgoing_Message_to_Emulator.field.ParameterFive = (double)0;
            Outgoing_Message_to_Emulator.field.ParameterSix = (double)0;
            for (int i = 0; i <= packet_length; i++) {
                Emulator_Port.write(Outgoing_Message_to_Emulator.character[i]);
                Panel_Port.write(Outgoing_Message_to_Emulator.character[i]);
            }
            break;
        }
        }       // end of switch commandnumber
    }
    else if (Incoming_Message_from_Emulator.field.MessageTarget == (byte)Device_PC_API) {       // send packet to Altitude
#ifdef USE_CONSOLE
        console_print("\tPacket Received from Emulator forwarded to PC_API and Panel");
#endif        
        for (int i = 0; i <= packet_length; i++) {
            PC_API_Port.write(Incoming_Message_from_Emulator.character[i]);
            Panel_Port.write(Incoming_Message_from_Emulator.character[i]);
        }
    }
    else if (Incoming_Message_from_Emulator.field.MessageTarget == (byte)Device_Altitude) {       // send packet to Altitude
        for (int i = 0; i <= packet_length; i++) {
            Altitude_Port.write(Incoming_Message_from_Emulator.character[i]);
            Panel_Port.write(Incoming_Message_from_Emulator.character[i]);
        }
#ifdef USE_CONSOLE
        console_print("\tPacket Received from Emulator forwarded to Altitude");
#endif
    }
    else if (Incoming_Message_from_Emulator.field.MessageTarget == (byte)Device_Azimuth) {
        for (int i = 0; i <= packet_length; i++) {
            Azimuth_Port.write(Incoming_Message_from_Emulator.character[i]);
            Panel_Port.write(Incoming_Message_from_Emulator.character[i]);
        }
#ifdef USE_CONSOLE
        console_print("\tPacket Received from Emulator forwarded to Azimuth");
#endif
    }
    else if (Incoming_Message_from_Emulator.field.MessageTarget == (byte)Device_Focuser) {
        for (int i = 0; i <= packet_length; i++) {
            Focuser_Port.write(Incoming_Message_from_Emulator.character[i]);
            Panel_Port.write(Incoming_Message_from_Emulator.character[i]);
        }
#ifdef USE_CONSOLE
        console_print("\tPacket Received from Emulator forwarded to Focuser");
#endif
    }
    else if (Incoming_Message_from_Emulator.field.MessageTarget == (byte)Device_Panel) {
        for (int i = 0; i <= packet_length; i++) {
            Panel_Port.write(Incoming_Message_from_Emulator.character[i]);
        }
#ifdef USE_CONSOLE
        console_print("\tPacket Received from Emulator forwarded to Panel");
#endif
    }
    else if (Incoming_Message_from_Emulator.field.MessageTarget == (byte)Device_AltAzi) {
        for (int i = 0; i <= packet_length; i++) {
            Altitude_Port.write(Incoming_Message_from_Emulator.character[i]);
            Azimuth_Port.write(Incoming_Message_from_Emulator.character[i]);
            Panel_Port.write(Incoming_Message_from_Emulator.character[i]);
        }
#ifdef USE_CONSOLE
        console_print("\tPacket Received from Emulator forwarded to Altitude and Azimuth");
#endif
    }
    else if (Incoming_Message_from_Emulator.field.MessageTarget == (byte)Device_Camera) {
        for (int i = 0; i <= packet_length; i++) {
            Camera_Port.write(Incoming_Message_from_Emulator.character[i]);
            Panel_Port.write(Incoming_Message_from_Emulator.character[i]);
        }
#ifdef USE_CONSOLE
        console_print("\tPacket Received from Emulator forwarded to Altitude and Azimuth");
#endif
    }
}
void Process_Incoming_Packet_from_Panel() {
    Panel_Incoming_Message_Available = false;
    Panel_Packets_Received++;
    if (Incoming_Message_from_Panel.field.MessageTarget == (byte)Device_Hub) {
        switch ((int)Incoming_Message_from_Panel.field.CommandNumber) {
        case (int)Request_Status: {                                                             // construct reply
#ifdef USE_CONSOLE
            console_print("\tRequest Received from Panel for Hub Status");
#endif
            UpdateEnvironmentalSensors();
            Outgoing_Message_to_Panel.field.MessageTarget = Device_Panel;
            Outgoing_Message_to_Panel.field.MessageSource = Device_Hub;
            Outgoing_Message_to_Panel.field.CommandNumber = Request_Status;
            Outgoing_Message_to_Panel.field.PacketType = REP;
            Outgoing_Message_to_Panel.field.CurrentStatus = Hub_status.word;
            Outgoing_Message_to_Panel.field.ParameterOne = (double)Firmware_Version;
            Outgoing_Message_to_Panel.field.ParameterTwo = (double)Ambient_Temperature;
            Outgoing_Message_to_Panel.field.ParameterThree = (double)Ambient_Humidity;
            Outgoing_Message_to_Panel.field.ParameterFour = (double)Motor_Voltage;
            Outgoing_Message_to_Panel.field.ParameterFive = (double)freeMemory();
            Outgoing_Message_to_Panel.field.ParameterSix = (double)0;
            for (int i = 0; i <= packet_length; i++) {
                Panel_Port.write(Outgoing_Message_to_Panel.character[i]);
            }
            break;
        }
        case (int)Request_Traffic: {
#ifdef USE_CONSOLE
            console_print("\tRequest Received from Panel for Hub Traffic");
#endif
            UpdateEnvironmentalSensors();
            Outgoing_Message_to_Panel.field.MessageTarget = Device_Panel;
            Outgoing_Message_to_Panel.field.MessageSource = Device_Hub;
            Outgoing_Message_to_Panel.field.CommandNumber = Request_Traffic;
            Outgoing_Message_to_Panel.field.PacketType = REP;
            Outgoing_Message_to_Panel.field.CurrentStatus = Hub_status.word;
            Outgoing_Message_to_Panel.field.ParameterOne = Ambient_Temperature;
            Outgoing_Message_to_Panel.field.ParameterTwo = Motor_Voltage;
            Outgoing_Message_to_Panel.field.ParameterThree = Altitude_Packets_Received;
            Outgoing_Message_to_Panel.field.ParameterFour = Azimuth_Packets_Received;
            Outgoing_Message_to_Panel.field.ParameterFive = Focuser_Packets_Received;
            Outgoing_Message_to_Panel.field.ParameterSix = Panel_Packets_Received;
            Outgoing_Message_to_Panel.field.ParameterSix = PC_API_Packets_Received;
            for (int i = 0; i <= packet_length; i++) {
                Panel_Port.write(Outgoing_Message_to_Panel.character[i]);
            }
            break;
        }
        case (int)Request_Reset: {
#ifdef USE_CONSOLE
            console_print("\tRequest Received from Panel for Reset");
#endif
            delay(200);
            while (1);                      // causes watchdog driven reboot
        }
        }       // end of switch commandnumber
    }
    else if (Incoming_Message_from_Panel.field.MessageTarget == (byte)Device_PC_API) {
#ifdef USE_CONSOLE
        console_print("\tPacket Received from Panel forwarded to PC_API");
#endif
        for (int i = 0; i <= packet_length; i++) {
            PC_API_Port.write(Incoming_Message_from_Panel.character[i]);
        }
    }
    else if (Incoming_Message_from_Panel.field.MessageTarget == (byte)Device_Altitude) {
#ifdef USE_CONSOLE
        console_print("\tPacket Received from Panel forwarded to Altitude");
#endif
        for (int i = 0; i <= packet_length; i++) {
            Altitude_Port.write(Incoming_Message_from_Panel.character[i]);
        }
    }
    else if (Incoming_Message_from_Panel.field.MessageTarget == (byte)Device_Azimuth) {
#ifdef USE_CONSOLE
        console_print("\tPacket Received from Panel forwarded to Azimuth");
#endif
        for (int i = 0; i <= packet_length; i++) {
            Azimuth_Port.write(Incoming_Message_from_Panel.character[i]);
        }
    }
    else if (Incoming_Message_from_Panel.field.MessageTarget == (byte)Device_Focuser) {
#ifdef USE_CONSOLE
        console_print("\tPacket Received from Panel forwarded to Focuser");
#endif
        for (int i = 0; i <= packet_length; i++) {
            Focuser_Port.write(Incoming_Message_from_Panel.character[i]);
        }
    }
    else if (Incoming_Message_from_Panel.field.MessageTarget == (byte)Device_Emulator) {
#ifdef USE_CONSOLE
        console_print("\tPacket Received from Panel forwarded to Emulator");
#endif
        for (int i = 0; i <= packet_length; i++) {
            Emulator_Port.write(Incoming_Message_from_Panel.character[i]);
        }
    }
    else if (Incoming_Message_from_PC_API.field.MessageTarget == (byte)Device_AltAzi) {
#ifdef USE_CONSOLE
        console_print("\tPacket Received from Panel forwarded to Altitude and Azimuth");
#endif
        for (int i = 0; i <= packet_length; i++) {
            Altitude_Port.write(Incoming_Message_from_Panel.character[i]);
            Azimuth_Port.write(Incoming_Message_from_Panel.character[i]);
        }

    }
    else if (Incoming_Message_from_PC_API.field.MessageTarget == (byte)Device_Camera) {
#ifdef USE_CONSOLE
        console_print("\tPacket Received from Panel forwarded to Camera");
#endif
        for (int i = 0; i <= packet_length; i++) {
            Camera_Port.write(Incoming_Message_from_Panel.character[i]);
        }
    }
}
void Process_Incoming_Packet_from_Camera() {					// process an update message from the Azimuth motor driver
    Camera_Incoming_Message_Available = false;
    Camera_Packets_Received++;
#ifdef USE_CONSOLE
    console_print("\tPacket Received from Camera forwarded to Emulator, PC_API and Panel");
#endif
    for (int i = 0; i <= packet_length; i++) {                  // copy the input packet to the output packet buffer
        Emulator_Port.write(Incoming_Message_from_Camera.character[i]);
        PC_API_Port.write(Incoming_Message_from_Camera.character[i]);
        Panel_Port.write(Incoming_Message_from_Camera.character[i]);
    }
}
void UpdateEnvironmentalSensors() {
    sensors_event_t event;
    Temperature_sensor.temperature().getEvent(&event);
    Humidity_sensor.humidity().getEvent(&event);			// Get humidity event and print its value.
    Ambient_Temperature = event.temperature;
    Ambient_Humidity = event.relative_humidity;
    Motor_Voltage = digitalRead(Voltage_pin);
}
void Green_Led_Flash() {
    if (millis() >= Green_Led_Start_Time + Led_On_Time) {
        digitalWrite(Green_led_pin, !digitalRead(Green_led_pin));       // toggle the green led
    }
}
// End of Programme----------------------------------------------------------------------------------------------------
