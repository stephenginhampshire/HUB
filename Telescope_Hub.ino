/*
  Arduino Ethernet Telescope Hub
        interfaces Steve and Jamie Gould's Telescope to a PC ASCOM compliant software driver
        Declination = Altitude = north/south = up down
        Right Ascension = Azimuth  = east/west = left right
        Communications to the motor controllers is made through this HUB.

    Functionality:
    1. Receive packets of information from the Operator
        a)  If the target is the Hub execute the contained command
        b)  Otherwise forward the received packet to the indicated target
        c)  Forward all received packets to the panel to add to the sd log and update displays
    2. Receive packets of information from the attached devices
        a)  If the target is the Hub execute the contained command, this would normally only be required if the Emulator was attached.
        b)  Otherwise forward the received packets to the indicated target
    3. Send and receive Heartbeat packets to attached devices
    4. Periodically, and on power on, send a "Are you connected message" to all attached devices, and process replies.
*/
/* Version Control ----------------------------------------------------------------------------------------------------
Date		Version Description
27/01/2018  1
11/05/2021	1.1     Updated to be compatible with Telescope and Focuser
06/11/2021	1.2     Introduction of Panel Functionality, Code Tidy up
14/07/2022	1.3     Code Tidy Up, made compatible with current telescope commands, status display now via API
19/08/2022  1.4     Log File Support Added
21/09/2022  1.5     Log File replaced with logging to serial line 4
05/02/2023  1.6     Recommenced review
16/02/2023  1.7     Added Pseudo serial connector so that the Exerciser can emulate the Operator
09/03/2023  1.8     Ability to selectively send heartbeat messages to the exerciser and/or the panel, they are always sent to Operator
30/03/2023  1.9     Hub only sends its own heartbeat to the Operator (when connected), but receives from all devices, except the Panel
18/07/2023  1.10    Exerciser removed as it's packets should be handled by the Port it is connected to
20/07/2023  1.11    Introduced configuration piano switch
*/
constexpr double Firmware_Version = (double)1.10;
// Inclusions ---------------------------------------------------------------------------------------------------------
#include <avr/wdt.h>
#include <Bounce2.h>
#include <DHT_U.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Ethernet2.h>
#include <util.h>
#include <EthernetUdp2.h>
#include <EthernetServer.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>
#include <NeoSWSerial.h>
#include <Hardwareserial.h>
#include <C:\Users\Stephen\Dropbox\Projects\Combined_Telescope\Common_Files\Telescope_Commands.h>
// Configuration Switches Signifance ----------------------------------------------------------------------------------
#define console Serial
constexpr int Operator_Available = 0x01;       // configuration switch setting to include Operator communications
constexpr int Altitude_Available = 0x02;
constexpr int Azimuth_Available = 0x04;
constexpr int Focuser_Available = 0x08;
constexpr int Panel_Available = 0x10;
constexpr int Print_Received = 0x20;
constexpr int Print_Transmitted = 0x40;
constexpr int Print_General = 0x80;
// Constants ----------------------------------------------------------------------------------------------------------
constexpr int Altitude_baud = (int)38400;
constexpr int Azimuth_baud = (int)38400;
constexpr int Focuser_baud = (int)38400;
constexpr int Panel_baud = (int)38400;
constexpr unsigned long Led_On_Time = (unsigned long)250;
// Constants ------------------------------------------------------------------
//uint8_t mac[] = { 0x90, 0xA2, 0xDA, 0x11, 0x23, 0x40 };	// MAC address of Ethernet controller, found on a sticker on the back of the Ethernet shield.
//uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };	// MAC address of Jamie's Ethernet Operator
//IPAddress ip(92, 68, 0, 30);							// IP Address (FIXED) of this server
//IPAddress gateway(92, 68, 0, 00);
//IPAddress subnet(255, 255, 255, 0);

uint8_t mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);
// Freememory calculater - Returns the current amount of free memory in bytes -----------------------------------------
extern unsigned int __bss_end;
extern void* __brkval;
int freeMemory() {
    int free_memory;
    if ((int)__brkval)
        return ((int)&free_memory) - ((int)__brkval);
    return ((int)&free_memory) - ((int)&__bss_end);
}
// Hardware configuration ---------------------------------------------------------------------------------------------
constexpr uint8_t Voltage_pin = A2;		    // A2	motor_voltage
constexpr uint8_t Green_led_pin = 3;        // Green led
constexpr uint8_t Temperature_pin = 4;	    // ambient temperature pin
constexpr uint8_t Fan_pin = 5;              // fan (relay) pin
constexpr uint8_t Altitude_TX_pin = 18;     // Altitude Port TX
constexpr uint8_t Altitude_RX_pin = 19;     // Altitude Port RX
constexpr uint8_t Azimuth_TX_pin = 16;      // Azimuth Port TX
constexpr uint8_t Azimuth_RX_pin = 17;      // Azimuth Port RX
constexpr uint8_t Focuser_TX_pin = 14;      // Focuser Port TX
constexpr uint8_t Focuser_RX_pin = 15;      // Focuser Port RX 
constexpr uint8_t Panel_TX_pin = 12;        // Control Panel Port TX
constexpr uint8_t Panel_RX_pin = 13;        // Control Panel Port RX
constexpr uint8_t Config_Bit_0_pin = 42;    // Configuration pin bit 0
constexpr uint8_t Config_Bit_1_pin = 43;    // Configuration pin bit 1
constexpr uint8_t Config_Bit_2_pin = 44;    // Configuration pin bit 2
constexpr uint8_t Config_Bit_3_pin = 45;    // Configuration pin bit 3
constexpr uint8_t Config_Bit_4_pin = 46;    // Configuration pin bit 4
constexpr uint8_t Config_Bit_5_pin = 47;    // Configuration pin bit 5
constexpr uint8_t Config_Bit_6_pin = 48;    // Configuration pin bit 6
constexpr uint8_t Config_Bit_7_pin = 49;    // Configuration pin bit 7

//--------------------------------------------------------------------------------------------------------------------
double Ambient_Temperature = 0;			// temperature value
double Ambient_Humidity = 0;
double Motor_Voltage = 0;
// Instantiations -----------------------------------------------------------------------------------------------------
EthernetServer Operator_Port(80);                                           // Create a server listening on port 80.
HardwareSerial Altitude_Port = Serial1;                                     // Altitude Port
HardwareSerial Azimuth_Port = Serial2;                                      // Azimuth Port
HardwareSerial Focuser_Port = Serial3;                                      // Focuser Port
NeoSWSerial Panel_Port(Panel_RX_pin, Panel_TX_pin);                         // Panel Software Port
// Communications Variables -------------------------------------------------------------------------------------------
DEVICE_Status System_Connectivity;
PacketUnion Incoming_Packet_from_Azimuth;
PacketUnion Incoming_Packet_from_Operator;
PacketUnion Incoming_Packet_from_Altitude;
PacketUnion Incoming_Packet_from_Focuser;
PacketUnion Incoming_Packet_from_Panel;
PacketUnion Outgoing_Message;
bool Altitude_Incoming_Packet_Available = false;
bool Azimuth_Incoming_Packet_Available = false;
bool Focuser_Incoming_Packet_Available = false;
bool Operator_Incoming_Packet_Available = false;
bool Panel_Incoming_Packet_Available = false;
uint8_t Altitude_in_buffer_counter = 0;
uint8_t Azimuth_in_buffer_counter = 0;
uint8_t Focuser_in_buffer_counter = 0;
uint8_t Panel_in_buffer_counter = 0;
uint8_t Altitude_inptr;					// must be 8 bit uint8_t so that it overflows at 256
uint8_t Altitude_outptr;					// must be 8 bit uint8_t so that it overflows at 256
uint8_t Altitude_inbuffer[0xff];
uint8_t Altitude_string_ptr;
uint8_t Azimuth_inptr;						// must be 8 bit uint8_t so that it overflows at 256
uint8_t Azimuth_outptr;					// must be 8 bit uint8_t so that it overflows at 256
uint8_t Azimuth_inbuffer[0xff];
uint8_t Azimuth_string_ptr;
uint8_t Focuser_inptr;
uint8_t Focuser_outptr;
uint8_t Focuser_inbuffer[0xff];
uint8_t Focuser_string_ptr;
uint8_t Panel_inptr;					// must be 8 bit uint8_t so that it overflows at 256
uint8_t Panel_outptr;					// must be 8 bit uint8_t so that it overflows at 256
uint8_t Panel_inbuffer[0xff];
uint8_t Panel_string_ptr;
// --------------------------------------------------------------------------------------------------------------------
struct HubStatusStructure {
    bool Lights : 1;            // A0
    bool Fan : 1;		        // A1
    bool Running : 1;			// A2
}__attribute__((packed));
union Status {
    HubStatusStructure bit;
    int word;
};
volatile Status Hub_Status;
unsigned long Time_of_Last_Heartbeat = 0;
unsigned long  Time_of_Last_Connectivity_Check = 0;
unsigned long Green_Led_Start_Time = 0;
bool Heartbeat_Enabled = true;
bool Fan_Enabled = true;
bool Lights_Enabled = true;
bool Hub_Heartbeat_to_Operator_Enabled = true;
bool Hub_Heartbeat_to_Azimuth_Enabled = true;
bool Hub_Heartbeat_to_Altitude_Enabled = true;
bool Hub_Heartbeat_to_Focuser_Enabled = true;
bool Hub_Heartbeat_to_Panel_Enabled = true;
// Instantiations -----------------------------------------------------------------------------------------------------
#ifdef INCLUDE_TEMPERATURE
DHT_Unified Temperature_sensor(Temperature_pin, DHT22);
DHT_Unified Humidity_sensor(Temperature_pin, DHT22);
sensors_event_t event;
sensor_t sensor;
#endif
Bounce Reset_button = Bounce();
// Interrupt Service Routines -----------------------------------------------------------------------------------------
void serialEvent1() {
    while (Altitude_Port.available()) {
        Altitude_inbuffer[Altitude_inptr++] = Altitude_Port.read();        // add the received characters to the buffer and increment characters count
    }
}
void serialEvent2() {
    while (Azimuth_Port.available()) {
        Azimuth_inbuffer[Azimuth_inptr++] = Azimuth_Port.read();        // add the received characters to the buffer and increment characters count
    }
}
void serialEvent3() {
    while (Focuser_Port.available()) {
        Focuser_inbuffer[Focuser_inptr++] = Focuser_Port.read();        // add the received characters to the buffer and increment characters count
    }
}
static void handle_Panel_RXChar(uint8_t received) {
    Panel_inbuffer[Panel_inptr] = received;
    Panel_inptr++;
}
//-- Setup ------------------------------------------------------------------------------------------------------------
void setup() {
    console.begin(115200);
    console_print("Setup Commenced");
    pinMode(Config_Bit_0_pin, INPUT_PULLUP);    // Configuration pin bit 0
    pinMode(Config_Bit_1_pin, INPUT_PULLUP);    // Configuration pin bit 1
    pinMode(Config_Bit_2_pin, INPUT_PULLUP);    // Configuration pin bit 2
    pinMode(Config_Bit_3_pin, INPUT_PULLUP);    // Configuration pin bit 3
    pinMode(Config_Bit_4_pin, INPUT_PULLUP);    // Configuration pin bit 4
    pinMode(Config_Bit_5_pin, INPUT_PULLUP);    // Configuration pin bit 5
    pinMode(Config_Bit_6_pin, INPUT_PULLUP);    // Configuration pin bit 6
    pinMode(Config_Bit_7_pin, INPUT_PULLUP);    // Configuration pin bit 7
    Read_Configuration();
    //    Operator_Port.begin();                                                 // Start Ethernet
    console_print("Setup Leds");
    pinMode(Green_led_pin, OUTPUT);
    digitalWrite(Green_led_pin, LOW);
    pinMode(Temperature_pin, INPUT);
    pinMode(Voltage_pin, INPUT);
    pinMode(Fan_pin, OUTPUT);                                       // specify the fan pin as an output
    console_print("Setup Serial Ports");
    System_Connectivity.word = 0;                                   // nothing attached at the start
    Altitude_Port.begin(Altitude_baud, SERIAL_8N2);					// initialise the Altitude serial port    
    Altitude_Port.flush();                                          // clear the Altitude serial buffer
    console_print("Altitude serial port started");
    Azimuth_Port.begin(Azimuth_baud, SERIAL_8N2);					// initialise the Azimuth serial port
    Azimuth_Port.flush();											// clear the Azimuth serial buffer
    console_print("Azimuth serial port started");
    Focuser_Port.begin(Focuser_baud, SERIAL_8N2);					// initialise the Focuser serial port
    Focuser_Port.flush();											// clear the Focuser serial buffer
    console_print("Focuser serial port started");
    // Software Ports -----------------------------------------------------------------------------------------------------
    Panel_Port.attachInterrupt(handle_Panel_RXChar);
    Panel_Port.begin(Panel_baud);
    console_print("Panel serial port started");
    console_print("Serial Port Setup Complete");
    console_print("Checking Connected Devices");
    Check_Connected_Devices();
    console_print("Enabling WatchDog Timer");
    wdt_enable(WDTO_4S);                                    // 4 second timeout
    console_print("Setup Complete");
} // end setup
void(*resetFunc) (void) = 0;                                // reset function
// Main ---------------------------------------------------------------------------------------------------------------
void loop() {
    wdt_reset();                                                            // keep watch dog timer active
    Green_Led_Flash();                                                      // toggle the green led
    if ((Read_Configuration() & Operator_Available) == Operator_Available) if (Check_Operator_Packet()) Process_Incoming_Packet_from_Operator();
    if ((Read_Configuration() & Altitude_Available) == Altitude_Available) if (Check_Altitude_Packet()) Process_Incoming_Packet_from_Altitude();
    if ((Read_Configuration() & Azimuth_Available) == Azimuth_Available) if (Check_Azimuth_Packet()) Process_Incoming_Packet_from_Azimuth();
    if ((Read_Configuration() & Focuser_Available) == Focuser_Available) if (Check_Focuser_Packet()) Process_Incoming_Packet_from_Focuser();
    if ((Read_Configuration() & Panel_Available) == Panel_Available) if (Check_Panel_Packet()) Process_Incoming_Packet_from_Panel();
    if ((millis() >= Time_of_Last_Connectivity_Check + Connectivity_Check_Period) || (Time_of_Last_Connectivity_Check == 0)) Check_Connected_Devices();
}// end of main loop -------------------------------------------------------------------------------------------------
void console_print(String message) {
    console.print(millis(), DEC); console.print("\t"); console.println(message);
}
int Read_Configuration() {
    uint8_t configuration = digitalRead(Config_Bit_0_pin);
    configuration |= digitalRead(Config_Bit_1_pin) << 1;
    configuration |= digitalRead(Config_Bit_2_pin) << 2;
    configuration |= digitalRead(Config_Bit_3_pin) << 3;
    configuration |= digitalRead(Config_Bit_4_pin) << 4;
    configuration |= digitalRead(Config_Bit_5_pin) << 5;
    configuration |= digitalRead(Config_Bit_6_pin) << 6;
    configuration |= digitalRead(Config_Bit_7_pin) << 7;
    return configuration;
}
void Check_Connected_Devices() {
    Time_of_Last_Connectivity_Check = millis();
    // Prepare "Are You Connected" Packet ---------------------------------------------------------------------------------
    Outgoing_Message.field.Header = STX;	                            // [0] STX
    Outgoing_Message.field.MessageSource = Device_Hub;                  // [2] target for message
    Outgoing_Message.field.CommandNumber = Are_You_Connected;		    // [3] command character
    Outgoing_Message.field.PacketType = GET;		                    // [4] packet type  
    Outgoing_Message.field.CurrentStatus = System_Connectivity.word;    // [5 - 6]
    Outgoing_Message.field.ParameterOne = (double)millis();             // [7 - 10]
    Outgoing_Message.field.ParameterTwo = (double)0;                    // [11 - 14]
    Outgoing_Message.field.ParameterThree = (double)0;                  // [15 - 18]
    Outgoing_Message.field.ParameterFour = (double)0;               	// [19 - 22]
    Outgoing_Message.field.ParameterFive = (double)0;                   // [23 - 26]
    Outgoing_Message.field.ParameterSix = (double)0;                    // [27 - 30]
    Outgoing_Message.field.Footer = ETX;		                        // [31] ETX
    // Packet Created -----------------------------------------------------------------------------------------------------
    if ((Read_Configuration() & Operator_Available) == Operator_Available) {
        Outgoing_Message.field.MessageTarget = Device_Operator;             // Set target device
        for (int i = 0; i <= packet_length; i++) {                          // Transmit the Are You Connected Packet" to the target device
            while (!Operator_Port.availableForWrite()) {                    // make sure the output port is available
                delay(5);
            }
            Operator_Port.write(Outgoing_Message.character[i]);             // Transmit byte to the Operator Port
        }
        System_Connectivity.bit.Operator = false;                           // make the connectivity flag false, will be set by the reply packet 
        if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
            console_print("Are You Connected Transmitted to Operator");
        }
    }
    if ((Read_Configuration() & Altitude_Available) == Altitude_Available) {
        Outgoing_Message.field.MessageTarget = Device_Altitude;             // Set target device
        for (int i = 0; i <= packet_length; i++) {                          // Transmit the Are You Connected Packet" to the target device
            while (!Altitude_Port.availableForWrite()) {                    // make sure the output port is available
                delay(5);
            }
            Altitude_Port.write(Outgoing_Message.character[i]);             // Transmit byte to the target Port
        }
        System_Connectivity.bit.Altitude = false;                           // make the connectivity flag false, will be set by the reply packet 
        if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
            console_print("Are You Connected Transmitted to Altitude");
        }
    }
    if ((Read_Configuration() & Azimuth_Available) == Azimuth_Available) {
        Outgoing_Message.field.MessageTarget = Device_Azimuth;              // Set target device
        for (int i = 0; i <= packet_length; i++) {                          // Transmit the Are You Connected Packet" to the target device
            while (!Azimuth_Port.availableForWrite()) {                     // make sure the output port is available
                delay(5);
            }
            Azimuth_Port.write(Outgoing_Message.character[i]);              // Transmit byte to the target Port
        }
        System_Connectivity.bit.Azimuth = false;                            // make the connectivity flag false, will be set by the reply packet 
        if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
            console_print("Are You Connected Transmitted to Azimuth");
        }
    }
    if ((Read_Configuration() & Focuser_Available) == Focuser_Available) {
        Outgoing_Message.field.MessageTarget = Device_Focuser;              // Set target device
        for (int i = 0; i <= packet_length; i++) {                          // Transmit the Are You Connected Packet" to the target device
            while (!Focuser_Port.availableForWrite()) {                     // make sure the output port is available
                delay(5);
            }
            Focuser_Port.write(Outgoing_Message.character[i]);              // Transmit byte to the target Port
        }
        System_Connectivity.bit.Focuser = false;                            // make the connectivity flag false, will be set by the reply packet 
        if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
            console_print("Are You Connected Transmitted to Focuser");
        }
    }
    if ((Read_Configuration() & Panel_Available) == Panel_Available) {
        Outgoing_Message.field.MessageTarget = Device_Panel;                // Set target device
        for (int i = 0; i <= packet_length; i++) {                          // Transmit the Are You Connected Packet" to the target device
            while (!Panel_Port.availableForWrite()) {                       // make sure the output port is available
                delay(5);
            }
            Panel_Port.write(Outgoing_Message.character[i]);                // Transmit byte to the target Port
        }
        System_Connectivity.bit.Panel = false;                           // make the connectivity flag false, will be set by the reply packet 
        if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
            console_print("Are You Connected Transmitted to Panel");
        }
    }
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
    UpdateEnvironmentalSensors();
    Outgoing_Message.field.Header = STX;	                            // [0] STX
    Outgoing_Message.field.MessageTarget = Device_Operator;	            // [1] source of message
    Outgoing_Message.field.MessageSource = Device_Hub;                  // [2] target for message
    Outgoing_Message.field.CommandNumber = Heartbeat;		            // [3] command character
    Outgoing_Message.field.PacketType = HRT;		                    // [4] packet type  
    Outgoing_Message.field.CurrentStatus = Hub_Status.word;             // [5 - 6]
    Outgoing_Message.field.ParameterOne = Ambient_Temperature;          // [7 - 10]
    Outgoing_Message.field.ParameterTwo = Motor_Voltage;                // [11 - 14]
    Outgoing_Message.field.ParameterThree = (double)0;                  // [15 - 18]
    Outgoing_Message.field.ParameterFour = (double)0;               	// [19 - 22]
    Outgoing_Message.field.ParameterFive = (double)System_Connectivity.word;  // [23 - 26]
    Outgoing_Message.field.ParameterSix = (double)millis();             // [27 - 30]
    Outgoing_Message.field.Footer = ETX;		                        // [31] ETX
    if (System_Connectivity.bit.Operator == true) {
        if (Hub_Heartbeat_to_Operator_Enabled) {
            for (int i = 0; i <= packet_length; i++) {
                Operator_Port.write(Outgoing_Message.character[i]);
            }
            console_print("Heartbeat Transmitted to Operator");
            wdt_reset();                                                // keep watch dog timer active
        }
    }
    Time_of_Last_Heartbeat = millis();
}
// Received Packet Handling -------------------------------------------------------------------------------------------
bool Check_Operator_Packet(void) {
    Maintain_Internet();
    Operator_Incoming_Packet_Available = false;
    EthernetClient client = Operator_Port.available();                  // Listen for incoming client requests.
    if (client) {
        for (int i = 1; i < packet_length - 1; i++) {
            Incoming_Packet_from_Operator.character[i] = client.read();
        }
        Operator_Incoming_Packet_Available = true;
    }
    return Operator_Incoming_Packet_Available;
}
bool Check_Altitude_Packet(void) {
    while (Altitude_outptr != Altitude_inptr) { // check Altitude serial buffer for data
        uint8_t thisbyte = Altitude_inbuffer[Altitude_outptr++];                           // take a characters from the input buffer and increment pointer
        if ((thisbyte == (char)STX) && (Altitude_string_ptr == 0)) {                    // look for the STX, but only if the output string is empty
            Incoming_Packet_from_Altitude.character[Altitude_string_ptr++] = STX;      // store the STX and increment the string pointer
            Altitude_Incoming_Packet_Available = false;
        }
        else {
            if (thisbyte == (char)ETX) {												// characters was not an STX check for ETX
                Incoming_Packet_from_Altitude.character[Altitude_string_ptr++] = ETX;  // save the ETX and increment the string pointer
                if (Altitude_string_ptr == packet_length) {                             // does it mean end of packet (we just saved the ETX at 20!
                    Altitude_string_ptr = 0;                                            // zero the string pointer
                    Altitude_Incoming_Packet_Available = true;
                }
            }
            else {
                Incoming_Packet_from_Altitude.character[Altitude_string_ptr++] = thisbyte;       // Not a valid STX or a valid ETX so save it and increment string pointer
            }
        }
    } // end of while Altitude
    return Altitude_Incoming_Packet_Available;
}
bool Check_Azimuth_Packet(void) {
    while (Azimuth_outptr != Azimuth_inptr) {                                        // check Altitude serial buffer for data
        uint8_t thisbyte = Azimuth_inbuffer[Azimuth_outptr++];                           // take a characters from the input buffer and increment pointer
        if ((thisbyte == (char)STX) && (Azimuth_string_ptr == 0)) {                    // look for the STX, but only if the output string is empty
            Incoming_Packet_from_Azimuth.character[Azimuth_string_ptr++] = STX;                // store the STX and increment the string pointer
        }
        else {
            if (thisbyte == (char)ETX) {												// characters was not an STX check for ETX
                Incoming_Packet_from_Azimuth.character[Azimuth_string_ptr++] = ETX;            // save the ETX and increment the string pointer
                if (Azimuth_string_ptr == packet_length) {                                        // does it mean end of packet (we just saved the ETX at 20!
                    Azimuth_string_ptr = 0;                                            // zero the string pointer
                    Azimuth_Incoming_Packet_Available = true;
                }
            }
            else {
                Incoming_Packet_from_Azimuth.character[Azimuth_string_ptr++] = thisbyte;           // Not a valid STX or a valid ETX so save it and increment string pointer
            }
        }
    } // end of while Azimuth
    return Azimuth_Incoming_Packet_Available;
}
bool Check_Focuser_Packet(void) {
    while (Focuser_outptr != Focuser_inptr) {											// check Focuser serial buffer for data
        uint8_t thisbyte = Focuser_inbuffer[Focuser_outptr++];								// take a characters from the input buffer and increment pointer
        if ((thisbyte == (char)STX) && (Focuser_string_ptr == 0)) {						// look for the STX, but only if the output string is empty
            Incoming_Packet_from_Focuser.character[Focuser_string_ptr++] = STX;		// store the STX and increment the string pointer
            Focuser_Incoming_Packet_Available = false;
        }
        else {
            if (thisbyte == (char)ETX) {												// characters was not an STX check for ETX
                Incoming_Packet_from_Focuser.character[Focuser_string_ptr++] = ETX;     // save the ETX and increment the string pointer
                if (Focuser_string_ptr == packet_length) {  // does it mean end of packet (we just saved the ETX at 20!
                    Focuser_string_ptr = 0;												// zero the string pointer
                    Focuser_Incoming_Packet_Available = true;
                }
            }
            else {
                Incoming_Packet_from_Focuser.character[Focuser_string_ptr++] = thisbyte; // Not a valid STX or a valid ETX so save it and increment string pointer
            }
        }
    } // end of while Focuser
    return Focuser_Incoming_Packet_Available;
}
bool Check_Panel_Packet(void) {
    while (Panel_outptr != Panel_inptr) {                                       // check Altitude serial buffer for data
        uint8_t thisbyte = Panel_inbuffer[Panel_outptr++];                         // take a characters from the input buffer and increment pointer
        if ((thisbyte == (char)STX) && (Panel_string_ptr == 0)) {                       // look for the STX, but only if the output string is empty
            Incoming_Packet_from_Panel.character[Panel_string_ptr++] = STX;    // store the STX and increment the string pointer
            Panel_Incoming_Packet_Available = false;
        }
        else {
            if (thisbyte == (char)ETX) {												        // characters was not an STX check for ETX
                Incoming_Packet_from_Panel.character[Panel_string_ptr++] = ETX;// save the ETX and increment the string pointer
                if (Panel_string_ptr == packet_length) {                                // does it mean end of packet (we just saved the ETX at 20!
                    Panel_string_ptr = 0;                                               // zero the string pointer
                    Panel_Incoming_Packet_Available = true;
                }
            }
            else {
                Incoming_Packet_from_Panel.character[Panel_string_ptr++] = thisbyte;       // Not a valid STX or a valid ETX so save it and increment string pointer
            }
        }
    } // end of while Altitude
    return Panel_Incoming_Packet_Available;
}
// Process Received Packets -------------------------------------------------------------------------------------------
void Process_Incoming_Packet_from_Operator() {                                    // Process a packet from the Operator  
    if ((Read_Configuration() & Print_Received) == Print_Received) {
        console_print("Packet Received from Operator");
    }
    System_Connectivity.bit.Operator = true;
    Operator_Incoming_Packet_Available = false;                                       // clear rhe packet received flag
    // Packet Received from Operator--------------------------------------------------------------------------------------
    if (Incoming_Packet_from_Operator.field.MessageTarget == (uint8_t)Device_Altitude) {
        if (System_Connectivity.bit.Altitude == true) {                        // transmit to the Altitude if connected
            for (int i = 0; i <= packet_length; i++) {
                while (!Altitude_Port.availableForWrite()) {
                    delay(10);
                }
                Altitude_Port.write(Incoming_Packet_from_Operator.character[i]);
            }
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet delivered to Altitude");
            }
        }
        else {
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet not delivered because Target Altitude not connected");
            }
        }
        if (System_Connectivity.bit.Panel == true) {
            for (int i = 0; i <= packet_length; i++) {
                while (!Panel_Port.availableForWrite()) {
                    delay(10);
                }
                Panel_Port.write(Incoming_Packet_from_Operator.character[i]);
            }
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet delivered to Panel");
            }
        }
        else {
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet not delivered because Target Panel not connected");
            }
        }
    }
    else if (Incoming_Packet_from_Operator.field.MessageTarget == (uint8_t)Device_Azimuth) {
        if (System_Connectivity.bit.Azimuth == true) {
            for (int i = 0; i <= packet_length; i++) {
                while (!Azimuth_Port.availableForWrite()) {
                    delay(10);
                }
                Azimuth_Port.write(Incoming_Packet_from_Operator.character[i]);
            }
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet delivered to Azimuth");
            }
        }
        else {
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet not delivered because Target Azimuth not connected");
            }
        }
        if (System_Connectivity.bit.Panel == true) {
            for (int i = 0; i <= packet_length; i++) {
                while (!Panel_Port.availableForWrite()) {
                    delay(10);
                }
                Panel_Port.write(Incoming_Packet_from_Operator.character[i]);
            }
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet delivered to Panel");
            }
        }
        else {
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet not delivered because Target Panel not connected");
            }
        }
    }
    else if (Incoming_Packet_from_Operator.field.MessageTarget == (uint8_t)Device_Focuser) {
        if (System_Connectivity.bit.Focuser == true) {
            for (int i = 0; i <= packet_length; i++) {
                while (!Focuser_Port.availableForWrite()) {
                    delay(10);
                }
                Focuser_Port.write(Incoming_Packet_from_Operator.character[i]);
            }
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet delivered to Focuser");
            }
        }
        else {
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet not delivered because Target Focuser not connected");
            }
        }
        if (System_Connectivity.bit.Panel == true) {
            for (int i = 0; i <= packet_length; i++) {
                while (!Panel_Port.availableForWrite()) {
                    delay(10);
                }
                Panel_Port.write(Incoming_Packet_from_Operator.character[i]);
            }
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet delivered to Panel");
            }
        }
        else {
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet not delivered because Target Panel not connected");
            }
        }
    }
    else if (Incoming_Packet_from_Operator.field.MessageTarget == (uint8_t)Device_Hub) {
        // Target Device = Hub, therefore response required ------------------------------------------------------------------
        switch ((int)Incoming_Packet_from_Operator.field.CommandNumber) {               // Process cpmmand number    
        case (int)Request_Firmware_Version: {                                            // request hub firmware version
            if ((Read_Configuration() & Print_Received) == Print_Received) {
                console_print("Operator Requested Firmware Version from Hub");
            }
            UpdateEnvironmentalSensors();                                               // update the environmental variables
            Outgoing_Message.field.MessageTarget = Device_Operator;                     // prepare reply packet
            Outgoing_Message.field.MessageSource = Device_Hub;
            Outgoing_Message.field.CommandNumber = Request_Firmware_Version;
            Outgoing_Message.field.PacketType = REP;
            Outgoing_Message.field.CurrentStatus = Hub_Status.word;
            Outgoing_Message.field.ParameterOne = (double)Firmware_Version;
            Outgoing_Message.field.ParameterTwo = (double)0;
            Outgoing_Message.field.ParameterThree = (double)0;
            Outgoing_Message.field.ParameterFour = (double)0;
            Outgoing_Message.field.ParameterFive = (double)freeMemory();
            Outgoing_Message.field.ParameterSix = (double)millis();
            if (System_Connectivity.bit.Operator == true) {
                for (int i = 0; i <= packet_length; i++) {
                    while (!Operator_Port.availableForWrite()) {
                        delay(10);
                    }
                    Operator_Port.write(Outgoing_Message.character[i]);         // send reply packet to Operator
                }
                if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                    console_print("Firmware Version Reply sent to the Operator");
                }
            }
            else {
                if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                    console_print("Firmware Verion Reply not sent because Operator not connected");
                }
            }
            if (System_Connectivity.bit.Panel == true) {
                for (int i = 0; i <= packet_length; i++) {
                    while (!Panel_Port.availableForWrite()) {
                        delay(10);
                    }
                    Panel_Port.write(Outgoing_Message.character[i]);            // send to Panel
                }
                if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                    console_print("Operator Request Firmware Verion Reply sent to Panel");
                }
            }
            else {
                if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                    console_print("Operator Request Firmware Version Reply not sent because Panel not connected");
                }
            }
            break;
        }
        case (int)Request_Status: {
            if ((Read_Configuration() & Print_Received) == Print_Received) {
                console_print("Status Request received from Operator");
            }
            UpdateEnvironmentalSensors();
            Outgoing_Message.field.MessageTarget = Device_Operator;
            Outgoing_Message.field.MessageSource = Device_Hub;
            Outgoing_Message.field.CommandNumber = Firmware_Version;
            Outgoing_Message.field.PacketType = REP;
            Outgoing_Message.field.CurrentStatus = Hub_Status.word;
            Outgoing_Message.field.ParameterOne = (double)0;
            Outgoing_Message.field.ParameterTwo = (double)0;
            Outgoing_Message.field.ParameterThree = (double)Ambient_Temperature;
            Outgoing_Message.field.ParameterFour = (double)Motor_Voltage;
            Outgoing_Message.field.ParameterFive = (double)0;
            Outgoing_Message.field.ParameterSix = (double)millis();
            if (System_Connectivity.bit.Operator == true) {
                for (int i = 0; i <= packet_length; i++) {
                    while (!Operator_Port.availableForWrite()) {
                        delay(10);
                    }
                    Operator_Port.write(Outgoing_Message.character[i]);
                }
                if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                    console_print("Status Request Reply Sent to Operator");
                }
            }
            else {
                if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                    console_print("Reply not sent because Operator not connected");
                }
            }
            if (System_Connectivity.bit.Panel == true) {
                for (int i = 0; i <= packet_length; i++) {
                    while (!Panel_Port.availableForWrite()) {
                        delay(10);
                    }
                    Panel_Port.write(Outgoing_Message.character[i]);
                }
                if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                    console_print("Hub Status Request Reply sent to Panel");
                }
            }
            else {
                if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                    console_print("Hub Status Request Reply not sent because Panel not connected");
                }
            }
            break;
        }
        case (int)Heartbeat: {
            if ((Read_Configuration() & Print_Received) == Print_Received) {
                console_print("Heartbeat Received from Operator");
            }
            if (Incoming_Packet_from_Operator.field.PacketType == (uint8_t)SET) {
                if ((bool)Incoming_Packet_from_Operator.field.ParameterOne == true) {
                    Hub_Heartbeat_to_Operator_Enabled = true;
                    if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                        console_print("Hub Heartbeat to Operator Enabled");
                    }
                }
                else {
                    Hub_Heartbeat_to_Operator_Enabled = false;
                    if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                        console_print("Hub Heartbeat to Operator Disabled");
                    }
                }
            }
            if (System_Connectivity.bit.Panel == true) {
                for (int i = 0; i <= packet_length; i++) {
                    while (!Panel_Port.availableForWrite()) {
                        delay(10);
                    }
                    Panel_Port.write(Outgoing_Message.character[i]);
                }
                if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                    console_print("Operator Heartbeat sent to Panel");
                }
            }
            else {
                if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                    console_print("Operator Heartbeat not sent because Panel not connected");
                }
            }
            break;
        }
        case (int)Request_Reset: {
            if ((Read_Configuration() & Print_Received) == Print_Received) {
                console_print("Operator Requested Hub Reset");
            }
            // no reply required
            resetFunc();
        }
        case (int)Reset_to_Defaults: {
            if ((Read_Configuration() & Print_Received) == Print_Received) {
                console_print("Operator Request Hub to Reset to Default Values");
            }
            // no reply required
            break;
        }
        case (int)Lights_Fan: {
            if (Incoming_Packet_from_Operator.field.PacketType == (uint8_t)GET) {                  // GET
                if ((Read_Configuration() & Print_Received) == Print_Received) {
                    console_print("Operator Requested Hub's Lights/Fan Status");
                }
                Outgoing_Message.field.MessageTarget = Device_Operator;
                Outgoing_Message.field.MessageSource = Device_Hub;
                Outgoing_Message.field.CommandNumber = Lights_Fan;
                Outgoing_Message.field.PacketType = REP;
                Outgoing_Message.field.CurrentStatus = Hub_Status.word;
                Outgoing_Message.field.ParameterOne = (double)Lights_Enabled;
                Outgoing_Message.field.ParameterTwo = (double)Fan_Enabled;
                Outgoing_Message.field.ParameterThree = (double)0;
                Outgoing_Message.field.ParameterFour = (double)0;
                Outgoing_Message.field.ParameterFive = (double)0;
                Outgoing_Message.field.ParameterSix = (double)0;;
                if (System_Connectivity.bit.Operator == true) {
                    for (int i = 0; i <= packet_length; i++) {
                        while (!Operator_Port.availableForWrite()) {
                            delay(10);
                        }
                        Operator_Port.write(Outgoing_Message.character[i]);
                    }
                    if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                        console_print("Operator Request for Hub's Lights/Fan Status Reply sent to Operator");
                    }
                }
                else {
                    if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                        console_print("Operator Request for Hub's Lights/Fan Ststua Reply not sent because Operator not connected");
                    }
                }
                if (System_Connectivity.bit.Panel == true) {
                    for (int i = 0; i <= packet_length; i++) {
                        while (!Panel_Port.availableForWrite()) {
                            delay(10);
                        }
                        Panel_Port.write(Outgoing_Message.character[i]);
                    }
                    if ((Read_Configuration() & Print_Received) == Print_Received) {
                        console_print("Operator Request for Hub's Light/Fan Status Reply sent to Panel");
                    }
                }
                else {
                    if ((Read_Configuration() & Print_Received) == Print_Received) {
                        console_print("Operator Request for Hub's Light/Fan Status Reply not sent because Panel not connected");
                    }
                }
            }
            else {                                                                          // Set
                if ((Read_Configuration() & Print_Received) == Print_Received) {
                    console_print("Operator Attempts to Set Hub's Lights/Fan Status");
                }
                Lights_Enabled = (bool)Incoming_Packet_from_Operator.field.ParameterOne;
                if (Lights_Enabled) {
                    if ((Read_Configuration() & Print_Received) == Print_Received) {
                        console_print("Hub Light Enabled by Operator");
                    }
                }
                else {
                    if ((Read_Configuration() & Print_Received) == Print_Received) {
                        console_print("Hub Light Disabled by Operator");
                    }
                }
                Fan_Enabled = (bool)Incoming_Packet_from_Operator.field.ParameterTwo;
                if (Fan_Enabled) {
                    if ((Read_Configuration() & Print_Received) == Print_Received) {
                        console_print("Hub Fan Enabled by Operator");
                    }
                    digitalWrite(Fan_pin, HIGH);
                    Hub_Status.bit.Fan = true;
                }
                else {
                    if ((Read_Configuration() & Print_Received) == Print_Received) {
                        console_print("Hub Fan Disabled by Operator");
                    }
                    digitalWrite(Fan_pin, LOW);
                    Hub_Status.bit.Fan = false;
                }
            }
            break;
        }
        case (int)Are_You_Connected: {                                              // Operator asked if Hub is connected
            if (Incoming_Packet_from_Operator.field.PacketType == GET) {
                if ((Read_Configuration() & Print_Received) == Print_Received) {
                    console_print("Operator asked Hub Are you connected");
                }
                Outgoing_Message.field.MessageTarget = Device_Operator;
                Outgoing_Message.field.MessageSource = Device_Hub;
                Outgoing_Message.field.CommandNumber = Are_You_Connected;
                Outgoing_Message.field.PacketType = REP;
                Outgoing_Message.field.CurrentStatus = Hub_Status.word;
                Outgoing_Message.field.ParameterOne = (double)true;             // Hub is connected
                Outgoing_Message.field.ParameterTwo = (double)0;
                Outgoing_Message.field.ParameterThree = (double)0;
                Outgoing_Message.field.ParameterFour = (double)0;
                Outgoing_Message.field.ParameterFive = (double)0;
                Outgoing_Message.field.ParameterSix = (double)millis();;
                if (System_Connectivity.bit.Operator == true) {
                    for (int i = 0; i <= packet_length; i++) {
                        while (!Operator_Port.availableForWrite()) {
                            delay(10);
                        }
                        Operator_Port.write(Outgoing_Message.character[i]);
                    }
                    if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                        console_print("Hub's Are You Connected Reply sent to Operator");
                    }
                }
                else {
                    if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                        console_print("Reply not sent because Operator not connected");
                    }
                }
                if (System_Connectivity.bit.Panel == true) {
                    for (int i = 0; i <= packet_length; i++) {
                        while (!Panel_Port.availableForWrite()) {
                            delay(10);
                        }
                        Panel_Port.write(Outgoing_Message.character[i]);
                    }
                    if ((Read_Configuration() & Print_Received) == Print_Received) {
                        console_print("Operator Request for Hub's Are You Connected Reply sent to Panel");
                    }
                }
                else {
                    if ((Read_Configuration() & Print_Received) == Print_Received) {
                        console_print("Operator Request for Hub's Are You Connected Reply not sent to Panel because Panel not connected");
                    }
                }
            }
            if (Incoming_Packet_from_Operator.field.PacketType == REP) {            // Reply
                if (Incoming_Packet_from_Operator.field.MessageSource == Device_Operator) {
                    System_Connectivity.bit.Operator = true;
                    if ((Read_Configuration() & Print_Received) == Print_Received) {
                        console_print("Operator Replied to Hub's Are You Connected");
                    }
                }
            }
            break;
        }
        }       // end of switch commandnumber
    }
    else if (Incoming_Packet_from_Operator.field.MessageTarget == (uint8_t)Device_AltAzi) {
        if (System_Connectivity.bit.Altitude == true) {
            for (int i = 0; i <= packet_length; i++) {
                while (!Operator_Port.availableForWrite()) {
                    delay(10);
                }
                Altitude_Port.write(Incoming_Packet_from_Operator.character[i]);
            }
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet delivered to Altitude");
            }
        }
        else {
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet not delivered because Target Altitude not connected");
            }
        }
        if (System_Connectivity.bit.Azimuth == true) {
            for (int i = 0; i <= packet_length; i++) {
                while (!Azimuth_Port.availableForWrite()) {
                    delay(10);
                }
                Azimuth_Port.write(Incoming_Packet_from_Operator.character[i]);
            }
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet sent to Azimuth");
            }
        }
        else {
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet not sent to Azimuth because Azimuth not connected");
            }
        }
        if (System_Connectivity.bit.Panel == true) {
            for (int i = 0; i <= packet_length; i++) {
                while (!Panel_Port.availableForWrite()) {
                    delay(10);
                }
                Panel_Port.write(Incoming_Packet_from_Operator.character[i]);
            }
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet sent to Panel");
            }
        }
        else {
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet not sent to Panel because Panel not connected");
            }
        }
    }
}
void Process_Incoming_Packet_from_Altitude() {					            // process an update message from a motor driver  
    if ((Read_Configuration() & Print_Received) == Print_Received) {
        console_print("Packet Received from Altitude");
    }
    Altitude_Incoming_Packet_Available = false;                            // clear the packet received flag
    System_Connectivity.bit.Altitude = true;
    if ((Read_Configuration() & Print_Received) == Print_Received) {
        console_print("Message Source: " + Device_Names[Incoming_Packet_from_Altitude.field.MessageSource]);    // [1] source of message
        console_print("Message Target: " + Device_Names[Incoming_Packet_from_Altitude.field.MessageTarget]);    // [2] target of message
        console_print("Command Number: " + Device_Names[Incoming_Packet_from_Altitude.field.CommandNumber]);    // [3] CommandNumber;
        console_print("Packet Type: " + Device_Names[Incoming_Packet_from_Altitude.field.PacketType]);          // [4] PacketType;  
        console_print("Current Status: " + Device_Names[Incoming_Packet_from_Altitude.field.CurrentStatus]);    // [5 - 6] CurrentStatus;
        console_print("Parameter One: " + String(Incoming_Packet_from_Altitude.field.ParameterOne));            // [7 - 10] ParameterOne;
        console_print("Parameter Two: " + String(Incoming_Packet_from_Altitude.field.ParameterTwo));            // [11 - 14] ParameterTwo
        console_print("Parameter Three: " + String(Incoming_Packet_from_Altitude.field.ParameterThree));		// [15 - 18] ParameterThree;
        console_print("Parameter Four: " + String(Incoming_Packet_from_Altitude.field.ParameterFour));          // [19 - 22] ParameterFour;	
        console_print("Parameter Five: " + String(Incoming_Packet_from_Altitude.field.ParameterFive));          // [23 - 26]
        console_print("Parameter Six: " + String(Incoming_Packet_from_Altitude.field.ParameterSix));		    // [27 - 30]
    }
    if (System_Connectivity.bit.Operator == true) {                             // transmit to the operator if connected
        for (int i = 0; i <= packet_length; i++) {
            while (!Operator_Port.availableForWrite()) {
                delay(10);
            }
            Operator_Port.write(Incoming_Packet_from_Altitude.character[i]);
        }
        if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
            console_print("Packet sent to the Operator");
        }
    }
    else {
        if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
            console_print("Packet not sent to Operator because Operator not connected");
        }
    }
    if (System_Connectivity.bit.Panel == true) {                                // transmit to the panel if connected
        for (int i = 0; i <= packet_length; i++) {
            while (!Panel_Port.availableForWrite()) {
                delay(10);
            }
            Panel_Port.write(Incoming_Packet_from_Altitude.character[i]);
        }
        if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
            console_print("Packet sent to the Panel");
        }
    }
    else {
        if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
            console_print("Packet not sent to Panel because Panel not connected");
        }
    }
}
void Process_Incoming_Packet_from_Azimuth() {					                // process an update message from the Azimuth motor driver
    if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
        console_print("Packet Received from Azimuth");
    }
    System_Connectivity.bit.Azimuth = true;                                     // must be connected to the Azimuth
    Azimuth_Incoming_Packet_Available = false;                                  // clear the packet received flag
    if ((Read_Configuration() & Print_Received) == Print_Received) {
        console_print("Message Source: " + Device_Names[Incoming_Packet_from_Azimuth.field.MessageSource]);     // [1] source of message
        console_print("Message Target: " + Device_Names[Incoming_Packet_from_Azimuth.field.MessageTarget]);     // [2] target of message
        console_print("Command Number: " + Command_Names[Incoming_Packet_from_Azimuth.field.CommandNumber]);    // [3] CommandNumber;
        console_print("Packet Type: " + Packet_Type_Names[Incoming_Packet_from_Azimuth.field.PacketType]);      // [4] PacketType;  
        console_print("Current Status: " + String(Incoming_Packet_from_Azimuth.field.CurrentStatus));           // [5 - 6] CurrentStatus;
        console_print("Parameter One: " + String(Incoming_Packet_from_Azimuth.field.ParameterOne));             // [7 - 10] ParameterOne;
        console_print("Parameter Two: " + String(Incoming_Packet_from_Azimuth.field.ParameterTwo));             // [11 - 14] ParameterTwo
        console_print("Parameter Three: " + String(Incoming_Packet_from_Azimuth.field.ParameterThree));		    // [15 - 18] ParameterThree;
        console_print("Parameter Four: " + String(Incoming_Packet_from_Azimuth.field.ParameterFour));           // [19 - 22] ParameterFour;	
        console_print("Parameter Five: " + String(Incoming_Packet_from_Azimuth.field.ParameterFive));           // [23 - 26]
        console_print("Parameter Six: " + String(Incoming_Packet_from_Azimuth.field.ParameterSix));		        // [27 - 30]
    }
    if (Incoming_Packet_from_Azimuth.field.MessageTarget == Device_Operator) {
        if (System_Connectivity.bit.Operator == true) {                         // transmit to operator if connected
            for (int i = 0; i <= packet_length; i++) {
                while (!Operator_Port.availableForWrite()) {
                    delay(10);
                }
                Operator_Port.write(Incoming_Packet_from_Azimuth.character[i]);
            }
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet sent to Operator");
            }
        }
        else {
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet not sent to Operator because Operator not connected");
            }
        }
        if (System_Connectivity.bit.Panel == true) {                            // transmit to the panel if connected
            for (int i = 0; i <= packet_length; i++) {
                while (!Panel_Port.availableForWrite()) {
                    delay(10);
                }
                Panel_Port.write(Incoming_Packet_from_Azimuth.character[i]);
            }
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet sent to Panel");
            }
        }
        else {
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet not sent to Panel because Panel not connected");
            }
        }
    }
}
void Process_Incoming_Packet_from_Focuser() {                                   // process an update message from the Focuser
    if ((Read_Configuration() & Print_Received) == Print_Received) {
        console_print("Packet Received from Focuser");
    }
    System_Connectivity.bit.Focuser = true;
    Focuser_Incoming_Packet_Available = false;                                  // clear the packet received flag
    if ((Read_Configuration() & Print_Received) == Print_Received) {
        console_print("Message Source: " + Device_Names[Incoming_Packet_from_Focuser.field.MessageSource]);     // [1] source of message
        console_print("Message Target: " + Device_Names[Incoming_Packet_from_Focuser.field.MessageTarget]);     // [2] target of message
        console_print("Command Number: " + Command_Names[Incoming_Packet_from_Focuser.field.CommandNumber]);    // [3] CommandNumber;
        console_print("Packet Type: " + Packet_Type_Names[Incoming_Packet_from_Focuser.field.PacketType]);      // [4] PacketType;  
        console_print("Current Status: " + String(Incoming_Packet_from_Focuser.field.CurrentStatus));           // [5 - 6] CurrentStatus;
        console_print("Parameter One: " + String(Incoming_Packet_from_Focuser.field.ParameterOne));             // [7 - 10] ParameterOne;
        console_print("Parameter Two: " + String(Incoming_Packet_from_Focuser.field.ParameterTwo));             // [11 - 14] ParameterTwo
        console_print("Parameter Three: " + String(Incoming_Packet_from_Focuser.field.ParameterThree));		    // [15 - 18] ParameterThree;
        console_print("Parameter Four: " + String(Incoming_Packet_from_Focuser.field.ParameterFour));           // [19 - 22] ParameterFour;	
        console_print("Parameter Five: " + String(Incoming_Packet_from_Focuser.field.ParameterFive));           // [23 - 26]
        console_print("Parameter Six: " + String(Incoming_Packet_from_Focuser.field.ParameterSix));		        // [27 - 30]
    }
    if (System_Connectivity.bit.Operator == true) {
        for (int i = 0; i <= packet_length; i++) {                              // copy the received packet to the output devices
            while (!Operator_Port.availableForWrite()) {
                delay(10);
            }
            Operator_Port.write(Incoming_Packet_from_Focuser.character[i]);     // also send to the Operator
        }
        if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
            console_print("Packet delivered to the Operator");
        }
    }
    else {
        if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
            console_print("Packet not delivered because Target Operator not connected");
        }
    }
    if (System_Connectivity.bit.Panel == true) {
        for (int i = 0; i <= packet_length; i++) {                              // copy the received packet to the output devices
            while (!Panel_Port.availableForWrite()) {
                delay(10);
            }
            Panel_Port.write(Incoming_Packet_from_Focuser.character[i]);        // also send to the Panel
        }
        if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
            console_print("Packet sent to the Panel");
        }
    }
    else {
        if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
            console_print("Packet not sent to the Panel because Panel not connected");
        }
    }
}
void Process_Incoming_Packet_from_Panel() {
    if ((Read_Configuration() & Print_Received) == Print_Received) {
        console_print("Packet Received from the Panel");
    }
    System_Connectivity.bit.Panel = true;
    Panel_Incoming_Packet_Available = false;
    if ((Read_Configuration() & Print_Received) == Print_Received) {
        console_print("Message Source: " + Device_Names[Incoming_Packet_from_Panel.field.MessageSource]);     // [1] source of message
        console_print("Message Target: " + Device_Names[Incoming_Packet_from_Panel.field.MessageTarget]);     // [2] target of message
        console_print("Command Number: " + Command_Names[Incoming_Packet_from_Panel.field.CommandNumber]);    // [3] CommandNumber;
        console_print("Packet Type: " + Packet_Type_Names[Incoming_Packet_from_Panel.field.PacketType]);      // [4] PacketType;  
        console_print("Current Status: " + String(Incoming_Packet_from_Panel.field.CurrentStatus));           // [5 - 6] CurrentStatus;
        console_print("Parameter One: " + String(Incoming_Packet_from_Panel.field.ParameterOne));             // [7 - 10] ParameterOne;
        console_print("Parameter Two: " + String(Incoming_Packet_from_Panel.field.ParameterTwo));             // [11 - 14] ParameterTwo
        console_print("Parameter Three: " + String(Incoming_Packet_from_Panel.field.ParameterThree));		    // [15 - 18] ParameterThree;
        console_print("Parameter Four: " + String(Incoming_Packet_from_Panel.field.ParameterFour));           // [19 - 22] ParameterFour;	
        console_print("Parameter Five: " + String(Incoming_Packet_from_Panel.field.ParameterFive));           // [23 - 26]
        console_print("Parameter Six: " + String(Incoming_Packet_from_Panel.field.ParameterSix));		        // [27 - 30]
    }
    if (Incoming_Packet_from_Panel.field.MessageTarget == (uint8_t)Device_Operator) {
        if (System_Connectivity.bit.Operator == true) {
            for (int i = 0; i <= packet_length; i++) {
                while (!Operator_Port.availableForWrite()) {
                    delay(10);
                }
                Operator_Port.write(Incoming_Packet_from_Panel.character[i]);
            }
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet delivered to Operator");
            }
        }
        else {
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet not sent to Operator because Operator not connected");
            }
        }
    }
    else if (Incoming_Packet_from_Panel.field.MessageTarget == (uint8_t)Device_Altitude) {
        if (System_Connectivity.bit.Altitude == true) {
            for (int i = 0; i <= packet_length; i++) {
                while (!Altitude_Port.availableForWrite()) {
                    delay(10);
                }
                Altitude_Port.write(Incoming_Packet_from_Panel.character[i]);
            }
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet from Panel sent to Altitude");
            }
        }
        else {
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet not sent to Altitude because Altitude not connected");
            }
        }
    }
    else if (Incoming_Packet_from_Panel.field.MessageTarget == (uint8_t)Device_Azimuth) {
        if (System_Connectivity.bit.Azimuth == true) {
            for (int i = 0; i <= packet_length; i++) {
                while (!Azimuth_Port.availableForWrite()) {
                    delay(10);
                }
                Azimuth_Port.write(Incoming_Packet_from_Panel.character[i]);
            }
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet from Panel sent to Azimuth");
            }
        }
        else {
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet not sent to Azimuth because Azimuth not connected");
            }
            }
        }
    else if (Incoming_Packet_from_Panel.field.MessageTarget == (uint8_t)Device_Focuser) {
        if (System_Connectivity.bit.Focuser == true) {
            for (int i = 0; i <= packet_length; i++) {
                while (!Focuser_Port.availableForWrite()) {
                    delay(10);
                }
                Focuser_Port.write(Incoming_Packet_from_Panel.character[i]);
            }
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("Packet from Panel sent to Focuser");
            }
        }
        else {
            if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                console_print("PAcket not sent to Focuser because Focuser not connected");
            }
        }
    }
    else if (Incoming_Packet_from_Panel.field.MessageTarget == (uint8_t)Device_Hub) {
        switch ((int)Incoming_Packet_from_Panel.field.CommandNumber) {      // switch
        case (int)Request_Status: {                                                             // construct reply
            UpdateEnvironmentalSensors();
            Outgoing_Message.field.MessageTarget = Device_Panel;
            Outgoing_Message.field.MessageSource = Device_Hub;
            Outgoing_Message.field.CommandNumber = Request_Status;
            Outgoing_Message.field.PacketType = REP;
            Outgoing_Message.field.CurrentStatus = Hub_Status.word;
            Outgoing_Message.field.ParameterOne = (double)Firmware_Version;
            Outgoing_Message.field.ParameterTwo = (double)Ambient_Temperature;
            Outgoing_Message.field.ParameterThree = (double)Ambient_Humidity;
            Outgoing_Message.field.ParameterFour = (double)Motor_Voltage;
            Outgoing_Message.field.ParameterFive = (double)freeMemory();
            Outgoing_Message.field.ParameterSix = (double)0;
            if (System_Connectivity.bit.Panel == true) {
                for (int i = 0; i <= packet_length; i++) {
                    while (!Panel_Port.availableForWrite()) {
                        delay(10);
                    }
                    Panel_Port.write(Outgoing_Message.character[i]);
                }
                if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                    console_print("Panel request for Hub Status Reply Sent to Panel");
                }
            }
            else {
                if ((Read_Configuration() & Print_Transmitted) == Print_Transmitted) {
                    console_print("Panel request for Hub STatus Reply not sent because Panel not connected");
                }
            }
            break;
        }       // end of switch commandnumber
        }                                                                   // end of switch
    }
    }
void UpdateEnvironmentalSensors() {
#ifdef INCLUDE_TEMPERATURE
    sensors_event_t event;
    Temperature_sensor.temperature().getEvent(&event);
    Humidity_sensor.humidity().getEvent(&event);			// Get humidity event and print its value.
    Ambient_Temperature = event.temperature;
    Ambient_Humidity = event.relative_humidity;
    Motor_Voltage = digitalRead(Voltage_pin);
#endif
}
void Green_Led_Flash() {
    if (Lights_Enabled) {
        if (millis() >= Green_Led_Start_Time + Led_On_Time) {
            digitalWrite(Green_led_pin, !digitalRead(Green_led_pin));       // toggle the green led
        }
    }
    else {
        digitalWrite(Green_led_pin, LOW);
    }
}
// End of Programme----------------------------------------------------------------------------------------------------
