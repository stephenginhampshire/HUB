/*
  Arduino Ethernet Telescope Hub
        Interfaces Steve and Jamie Gould's Telescope to a ASCOM compliant software driver
        Declination = Altitude = north/south = up down
        Right Ascension = Azimuth  = east/west = left right
        Communications to the motor controllers is made through this HUB.
    Functionality:
    1. Receive packets of information from the Operator (Windows PC)
        a) Receive packets from Operator and respond with ACK
        a) If the target is the Hub execute the contained command
        b) Otherwise forward the received packet to the indicated target
    2. Receive packets of information from the attached devices
        a)  Forward the received packets to the indicated target

        Version Control --------------------------------------------------------------------------------
Date		Version Description
27/01/2018  1.0
11/05/2021	1.1     Updated to be compatible with Telescope and Focuser
06/11/2021	1.2     Introduction of Panel Functionality, Code Tidy up
14/07/2022	1.3     Code Tidy Up, made compatible with current telescope commands, status display now via API
19/08/2022  1.4     Log File Support Added
21/09/2022  1.5     Log File replaced with logging to serial line, now removed
05/02/2023  1.6     Recommenced review
16/02/2023  1.7     Added Pseudo serial connector so that the Exerciser can emulate the Operator
09/03/2023  1.8     Ability to selectively send heartbeat messages to the exerciser and/or the panel, they are always sent to Operator
30/03/2023  1.9     Hub only sends its own heartbeat to the Operator (when connected), but receives from all devices, except the Panel
18/07/2023  1.10    Exerciser removed as it's packets should be handled by the Port it is connected to
20/07/2023  1.11    Introduced configuration piano switch
01/10/2024  2.0     Development Restarted
24/10/2024  2.1     Functionality reduced to support only Hub, Altitude, Azimuth and Focuser communications
26/10/2024  2.2     Ethernet protocol changed to TCP
29/10/2024  2.3     Log File re-introduced, all functionality tested, appears to be ok
30/10/2024  2.4     Added functionality to get Time and Date Information
01/11/2024  2.5     Added free memory to environmental parameters
07/11/2024  2.6     Day of Week calculation added
09/11/2024  2.7     Simulations Debugged and all working
16/11/2024  2.8     Simulation code removed
27/11/2024  2.9     Retrieve removed, client.print replaced with client.write
28/11/2024  2.10    Unnecessary functionality (Retrieve, Temperature and Voltage) removed
22/08/2025  2.11    Log File and Date and Time Functionality, updated to version 2.3 of Telescope_Commands
04/09/2025  3.0     Log File functionality removed and coms changed to use packethandler.h
04/09/2025  3.1     Communications with Operator syncronised with Jamie's New Exerciser
*/
constexpr double Firmware_Version = (double)3.1;
// Inclusions -------------------------------------------------------------------------------------
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_Sensor.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <Bounce2.h>
#include <SD.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>
#include <time.h>
#include <TimeLib.h>
// Compiler Definitions ---------------------------------------------------------------------------
//#define SD_DEBUG
//#define DISPLAY_FREE_MEMORY
#define PRINT_IO
#include <G:\My Drive\Telescope\Common_Files\Telescope_Commands.h>
#include <G:\My Drive\\Telescope\\Common_Files\PacketHandler.h>
#define console Serial
// Constants --------------------------------------------------------------------------------------
constexpr int Altitude_baud = (int)38400;
constexpr int Azimuth_baud = (int)38400;
constexpr int Focuser_baud = (int)38400;
constexpr unsigned long Led_On_Time = (unsigned long)250;

// Hardware Configuration -------------------------------------------------------------------------
// Communications Connections ---------------------------------------------------------------------
constexpr uint8_t Altitude_TX_pin = 18;     // Altitude Port TX
constexpr uint8_t Altitude_RX_pin = 19;     // Altitude Port RX
constexpr uint8_t Azimuth_TX_pin = 16;      // Azimuth Port TX
constexpr uint8_t Azimuth_RX_pin = 17;      // Azimuth Port RX
constexpr uint8_t Focuser_TX_pin = 14;      // Focuser Port TX
constexpr uint8_t Focuser_RX_pin = 15;      // Focuser Port RX
constexpr uint8_t RUN_Active_led_pin = 2;   // RUN led                          Blue
constexpr uint8_t Reset_Switch_pin = 5;     // Reset Switch pin                 Yellow
constexpr uint8_t W5500_CS = 10;            // Ethernet chip select             Internal
constexpr uint16_t wdtTimeouts[] = { 16, 32, 64, 125, 250, 500, 1000, 2000, 4000, 8000 };
// Instantiations ---------------------------------------------------------------------------------
HardwareSerial Altitude_Port = Serial1;                     // Altitude Port
HardwareSerial Azimuth_Port = Serial2;                      // Azimuth Port
HardwareSerial Focuser_Port = Serial3;                      // Focuser Port
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 68, 177);
IPAddress my_dns(192, 168, 68, 1);
IPAddress gateway(192, 168, 68, 1);
IPAddress subnet(255, 255, 0, 0);
EthernetServer server(80);                                  // Create a server listening on port 80.
EthernetClient client;                                      // Create an Ethernet Client (Operator), NTP server from https://tf.nist.gov/tf-cgi/servers.cgi
//EthernetUDP ethernet_UDP;                                 // define Ethernet UDP object and local port 8888
//unsigned int localPort = 8888;
//EthernetUDP UDP;                                          // A UDP instance to let us send and receive packets over UDP
Bounce Reset_Switch = Bounce();
// Communications Variables -----------------------------------------------------------------------
PacketUnion Incoming_Packet_from_Operator;
PacketUnion Incoming_Packet_from_Altitude;
PacketUnion Incoming_Packet_from_Azimuth;
PacketUnion Incoming_Packet_from_Focuser;
PacketUnion Outgoing_Packet_to_Operator;
PacketUnion Outgoing_Packet_to_Altitude;
PacketUnion Outgoing_Packet_to_Azimuth;
PacketUnion Outgoing_Packet_to_Focuser;
unsigned long ALT_Packet_Received_Count = 0;
unsigned long ALT_Packet_Transmitted_Count = 0;
unsigned long AZI_Packet_Received_Count = 0;
unsigned long AZI_Packet_Transmitted_Count = 0;
unsigned long FOC_Packet_Received_Count = 0;
unsigned long FOC_Packet_Transmitted_Count = 0;
uint8_t Operator_string_ptr;
uint8_t Altitude_inptr;					// must be 8 bit uint8_t so that it overflows at 256
uint8_t Altitude_outptr;				// must be 8 bit uint8_t so that it overflows at 256
uint8_t Altitude_inbuffer[0xff];
uint8_t Altitude_string_ptr;
uint8_t Azimuth_inptr;					// must be 8 bit uint8_t so that it overflows at 256
uint8_t Azimuth_outptr;					// must be 8 bit uint8_t so that it overflows at 256
uint8_t Azimuth_inbuffer[0xff];
uint8_t Azimuth_string_ptr;
uint8_t Focuser_inptr;
uint8_t Focuser_outptr;
uint8_t Focuser_inbuffer[0xff];
uint8_t Focuser_string_ptr;
bool Focuser_Packet_Direction = false;
uint16_t Device_Status = 0;
// ------------------------------------------------------------------------------------------------
char Display_Buffer[100];
enum { OFF = 0, ON = 1 };
unsigned long RUN_Active_Led_Start_Time = 0;
unsigned long Shield_Led_Start_Time = 0;
int Free_Memory = 0;                              // space to hold the amount of free memory
// Interrupt Service Routines ---------------------------------------------------------------------
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
//-- Setup ----------------------------------------------------------------------------------------
void setup() {
    console.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    console_print(true, F("Setup Commenced"), true);
    pinMode(RUN_Active_led_pin, OUTPUT);
    Led_Control(RUN_Active_led_pin, ON);            // turn the run led on
    Reset_Switch.attach(Reset_Switch_pin);
    Reset_Switch.interval(5);
    console_print(true, F("Starting Ethernet Initialisation"), true);
    Ethernet.begin(mac, ip, my_dns, gateway, subnet);                         // Start Ethernet
    delay(1000);
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Wait_for_Reset(F("Ethernet Shield not found"));
    }
    else {
        console_print(true, F("\tEthernet Shield Found"), true);
    }
    if (Ethernet.linkStatus() == LinkOFF) {
        console_print(true, F("\tEthernet cable not connected"), true);
    }
    else {
        console_print(true, F("\tEthernet cable connected"), true);
    }
    //EthernetClient client = server.accept();
    Status.bit.Operator = 1;
    console_print(true, F("Ethernet Initialisation Complete"), true);
    console_print(true, F("Serial Port Initialisation"), true);
    pinMode(Azimuth_RX_pin, INPUT);
    if (digitalRead(Azimuth_RX_pin)) {
        console_print(true, F("\tAzimuth Communication Line Connected"), true);
        Status.bit.Azimuth = 1;
    }
    else {
        console_print(true, F("\tAzimuth Communication Line not Connected"), true);
        Status.bit.Azimuth = 0;
    }
    Azimuth_Port.begin(Azimuth_baud, SERIAL_8N2);					// initialise the Altitude serial port    
    Azimuth_Port.flush();                                          // clear the Altitude serial buffer
    pinMode(Altitude_RX_pin, INPUT);
    if (digitalRead(Altitude_RX_pin)) {
        console_print(true, F("\tAltitude Communication Line Connected"), true);
        Status.bit.Altitude = 1;
    }
    else {
        console_print(true, F("\tAltitude Communication Line not Connected"), true);
        Status.bit.Altitude = 0;
    }
    Altitude_Port.begin(Azimuth_baud, SERIAL_8N2);					// initialise the Azimuth serial port
    Altitude_Port.flush();											// clear the Azimuth serial buffer
    pinMode(Focuser_RX_pin, INPUT);
    if (digitalRead(Focuser_RX_pin)) {
        console_print(true, F("\tFocuser Communication Line Connected"), true);
        Status.bit.Focuser = 1;
    }
    else {
        console_print(true, F("\tFocuser Communication Line not Connected"), true);
        Status.bit.Focuser = 0;
    }
    Focuser_Port.begin(Focuser_baud, SERIAL_8N2);					// initialise the Focuser serial port
    Focuser_Port.flush();											// clear the Focuser serial buffer
    console_print(true, F("Serial Port Initialisation Complete"), true);
    console_print(true, F("Enabling WatchDog Timer"), true);
    wdt_enable(WDTO_4S);                                    // 4 second timeout
    if (getWdtTimeoutMs()) {
        snprintf(Display_Buffer, sizeof(Display_Buffer), "\tCurrent Watchdog Timeout: %d (mS)", getWdtTimeoutMs());
        console_print(true, Display_Buffer, true);
    }
    else {
        console_print(true, F("\tWatchdog Timer Initialisation failure"), true);
    }
    console_print(true, F("Watchdog Timer Initialisation Complete"), true);
    Led_Control(RUN_Active_led_pin, ON);
    console_print(true, F("Setup Complete"), true);
    console_print(true, F("Starting Main Loop"), true);
} // end setup
// Main -------------------------------------------------------------------------------------------
void loop() {
    wdt_reset();                                                            // keep watch dog timer active
    Maintain_Internet();
    Led_Control(RUN_Active_led_pin, ON);
    if (Check_Operator_Packet_Received()) Process_Packet_Received_from_Operator();
    if (Check_Altitude_Packet_Received()) Transmit_Packet_to_Target(Incoming_Packet_from_Altitude);
    if (Check_Azimuth_Packet_Received()) Transmit_Packet_to_Target(Incoming_Packet_from_Azimuth);
    if (Check_Focuser_Packet_Received()) Transmit_Packet_to_Target(Incoming_Packet_from_Focuser);
}// end of main loop ------------------------------------------------------------------------------
void Maintain_Internet() {
    int ethernet_status = (int)Ethernet.maintain();     // keep ethernet link open
    switch (ethernet_status) {
    case 0: {                                           // nothing happened
        break;
    }
    case 1: {                                           // renew failed
        Ethernet.begin(mac, ip);                        // Start Ethernet
        break;
    }
    case 2: {                                           // renew success
        break;
    }
    case 3: {                                           // rebind fail
        Ethernet.begin(mac);                            // Start Ethernet
        break;
    }
    case 4: {                                           // rebind success
        break;
    }
    }
}
bool Check_Operator_Packet_Received(void) {
    if (server.available()) {
        client = server.accept();
        console_print(true, "Client connected", true);
    }
    while (client && client.connected() && client.available() > 0) {
        uint8_t thisbyte = client.read();                                   // read the character
        if (thisbyte == (uint8_t)STX) {                                     // look for start character
            Operator_string_ptr = 0;                                        // start character received so zero the string pointer
            Incoming_Packet_from_Operator.character[Operator_string_ptr++] = (uint8_t)STX;      // start of packet, store and increment the string pointer
        }
        else {
            if (thisbyte == (uint8_t)EOT) {                                 // characters was not an STX check for ETX
                Incoming_Packet_from_Operator.character[Operator_string_ptr++] = (uint8_t)EOT;  // save the EOT and increment the string pointer
                console_print(true, "\tIncoming Packet from Operator:", true);
                Operator_string_ptr = 0;                                         // zero the string pointer
                return true;                                                // return true because we have a packet
            }
            else {
                Incoming_Packet_from_Operator.character[Operator_string_ptr++] = thisbyte;      // Not a control so save it and increment string pointer
            }
        }
    }
    return false;
}
bool Check_Altitude_Packet_Received(void) {
    while (Altitude_outptr != Altitude_inptr) {                                 // check Altitude serial buffer for data
        uint8_t thisbyte = Altitude_inbuffer[Altitude_outptr++];                // take a characters from the input buffer and increment pointer
        if (thisbyte == (uint8_t)SOH) {                                         // look for the SOH
            Incoming_Packet_from_Altitude.character[Altitude_string_ptr++] = (uint8_t)SOH; // store the SOH and increment the string pointer
        }
        else {
            if (thisbyte == (uint8_t)EOT) {                                         // characters was not an SOH check for ETX
                Incoming_Packet_from_Altitude.character[Altitude_string_ptr++] = (uint8_t)EOT; // save the EOT and increment the string pointer
                console_print(true, "\tIncoming Packet from Altitude:", true);
                Altitude_string_ptr = 0;                                            // zero the string pointer
                ALT_Packet_Received_Count++;
                return true;                                                        // return true because we have a packet
            }
            else {
                Incoming_Packet_from_Altitude.character[Altitude_string_ptr++] = thisbyte;    // Not a control so save it and increment string pointer
            }
        }
    }
    return false;
}
bool Check_Azimuth_Packet_Received(void) {
    while (Azimuth_outptr != Azimuth_inptr) {                                   // check Azimuth serial buffer for data
        uint8_t thisbyte = Azimuth_inbuffer[Azimuth_outptr++];                  // take a characters from the input buffer and increment pointer
        if (thisbyte == (uint8_t)SOH) {                                         // look for the SOH
            Incoming_Packet_from_Azimuth.character[Azimuth_string_ptr++] = (uint8_t)SOH;  // store the SOH and increment the string pointer
        }
        else {
            if (thisbyte == (uint8_t)EOT) {                                          // characters was not an STX check for ETX
                Incoming_Packet_from_Azimuth.character[Azimuth_string_ptr++] = (uint8_t)EOT;  // save the EOT and increment the string pointer
                console_print(true, "\tIncoming Packet from Azimuth:", true);
                Azimuth_string_ptr = 0;                                             // zero the string pointer
                AZI_Packet_Received_Count++;
                return true;
            }
            else {
                Incoming_Packet_from_Azimuth.character[Azimuth_string_ptr++] = thisbyte;  // Not a control so save it and increment string pointer
            }
        }
    } // end of while Azimuth
    return false;
}
bool Check_Focuser_Packet_Received(void) {
    while (Focuser_outptr != Focuser_inptr) {                                       // check Focuser serial buffer for data
        uint8_t thisbyte = Focuser_inbuffer[Focuser_outptr++];                      // take a characters from the input buffer and increment pointer
        if (thisbyte == (char)SOH) {                                                // look for the SOH
            Incoming_Packet_from_Focuser.character[Focuser_string_ptr++] = (uint8_t)SOH;      // store the SOH and increment the string pointer
        }
        else {
            if (thisbyte == (char)EOT) {                                            // characters was not an STX check for ETX
                Incoming_Packet_from_Focuser.character[Focuser_string_ptr++] = (uint8_t)EOT;  // save the EOT and increment the string pointer
                console_print(true, "\tIncoming Packet from Focuser:", true);
                Focuser_string_ptr = 0;                                             // zero the string pointer
                FOC_Packet_Received_Count++;
                return true;
            }
            else {
                Incoming_Packet_from_Focuser.character[Focuser_string_ptr++] = thisbyte;      // Not a control so save it and increment string pointer
            }
        }
    } // end of while Focuser
    return false;
}
bool Process_Packet_Received_from_Operator() {                                   // Process a packet from the Operator  
    console_print(true, F("Processing Packet Received from Operator"), true);
    switch (Incoming_Packet_from_Operator.field.MessageTarget) {                 // switch on the target
    case Devices::Hub: {
        console_print(false, F("Destination HUB: "), true);
        switch (Incoming_Packet_from_Operator.field.CommandNumber) {             // switch on the Command Number
        case (Commands::CMD_Reset): {
            console_print(true, F("Reset Command Received from Operator"), true);
            if (Incoming_Packet_from_Operator.field.PacketType == CMD) {
                Wait_for_Reset(F("Reset Requested by Operator"));
            }
            break;
        }
        case (Commands::Request_Firmware_Version): {
            console_print(true, F("Firmware Version Get Received from Operator"), true);
            if (Incoming_Packet_from_Operator.field.PacketType == GET) {
                Prepare_and_Send_Reply_to_Operator(Operator,
                    Request_Firmware_Version,
                    Firmware_Version,
                    0,
                    0,
                    0,
                    0,
                    0
                );
            }
            break;
        }
        case (Commands::Statistics): {
            console_print(true, F("Statistics Get Received from Operator"), true);
            if (Incoming_Packet_from_Operator.field.PacketType == GET) {
                Prepare_and_Send_Reply_to_Operator(Operator,
                    Statistics,
                    ALT_Packet_Received_Count,
                    ALT_Packet_Transmitted_Count,
                    AZI_Packet_Received_Count,
                    AZI_Packet_Transmitted_Count,
                    FOC_Packet_Received_Count,
                    FOC_Packet_Transmitted_Count
                );
                break;
            }
        }
        }
    case Devices::Altitude: {                                                     // send the packet to the ALT
        console_print(true, F("Packet Destination Altitude"), true);
        Transmit_Packet_to_Target(Incoming_Packet_from_Operator);
        break;
    }
    case Devices::Azimuth: {
        console_print(true, F("Packet Destination Azimuth"), true);
        Transmit_Packet_to_Target(Incoming_Packet_from_Operator);
        break;
    }
    case Devices::Focuser: {
        console_print(true, F("Packet Destination Focuser"), true);
        Transmit_Packet_to_Target(Incoming_Packet_from_Operator);
        break;
    }
    }
    } // end of switch
    return true;
}
void Transmit_to_Operator(PacketUnion data) {
    if (Status.bit.Operator) {                           // check if Operator is available
        while (!server.availableForWrite()) {
            console_print(true, F("Waiting for server available"), true);
            delay(10);
        }
        console_print(false, F("Sending Packet to Operator"), true);
        for (int i = 0; i < Packet_Length; i++) {
            client.write(Outgoing_Packet_to_Operator.character[i]);
        }
    }
}
void Prepare_and_Send_Reply_to_Operator(uint8_t source, uint8_t command, uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p4, uint16_t p5, uint16_t p6) {
    Outgoing_Packet_to_Operator.field.MessageSource = source;
    Outgoing_Packet_to_Operator.field.MessageTarget = Operator;
    Outgoing_Packet_to_Operator.field.CommandNumber = command;
    Outgoing_Packet_to_Operator.field.PacketType = REP;
    Outgoing_Packet_to_Operator.field.CurrentStatus = Device_Status;
    Outgoing_Packet_to_Operator.field.ParameterOne = p1;
    Outgoing_Packet_to_Operator.field.ParameterTwo = p2;
    Outgoing_Packet_to_Operator.field.ParameterThree = p3;
    Outgoing_Packet_to_Operator.field.ParameterFour = p4;
    Outgoing_Packet_to_Operator.field.ParameterFive = p5;
    Outgoing_Packet_to_Operator.field.ParameterSix = p6;
    Transmit_Packet_to_Target(Outgoing_Packet_to_Operator);
}
void Transmit_Packet_to_Target(PacketUnion data) {
    switch (data.field.MessageTarget) {
    case Devices::Operator: {
        for (int i = 0; i < Packet_Length; i++) {
            Outgoing_Packet_to_Operator.character[i] = data.character[i];
        }
        Transmit_to_Operator(Outgoing_Packet_to_Operator);
        break;
    }
    case Devices::Azimuth: {
        for (int i = 0; i < Packet_Length; i++) {
            while (!Azimuth_Port.availableForWrite()) {
                delay(10);
            }
            Azimuth_Port.write(data.character[i]);
        }
        console_print(true, F("Packet sent to Azimuth"), true);
        AZI_Packet_Transmitted_Count++;
        break;
    }
    case Devices::Altitude: {
        for (int i = 0; i < Packet_Length; i++) {
            while (!Altitude_Port.availableForWrite()) {
                delay(10);
            }
            Altitude_Port.write(data.character[i]);
        }
        console_print(true, F("Packet sent to Altitude"), true);
        ALT_Packet_Transmitted_Count++;
        break;
    }
    case Devices::Focuser: {
        for (int i = 0; i < Packet_Length; i++) {
            while (!Focuser_Port.availableForWrite()) {
                delay(10);
            }
            Focuser_Port.write(data.character[i]);
        }
        console_print(true, F("Packet sent to Focuser"), true);
        FOC_Packet_Received_Count++;
        break;
    }
    }
}
void Led_Control(uint8_t led, bool state) {
    switch (led) {
    case (RUN_Active_led_pin): {
        digitalWrite(RUN_Active_led_pin, state);
        break;
    }
    }
}
uint16_t getWdtTimeoutMs() {
    // Mask out only the WDP bits from WDTCSR (bits 0-3)
    uint8_t wdpBits = WDTCSR & 0x0F;  // Mask to get only WDP3:WDP0 bits
    if (wdpBits < sizeof(wdtTimeouts) / sizeof(wdtTimeouts[0])) {
        return wdtTimeouts[wdpBits];
    }
    else {
        return 0;  // Undefined timeout
    }
}
void Wait_for_Reset(const __FlashStringHelper* message) {
    int trys = 0;
    console_print(false, F("Failure: "), true);
    Serial.print(message);
    console_print(true, F(", Press Reset"), true);
    do {
        delay(500);
        digitalWrite(RUN_Active_led_pin, !digitalRead(RUN_Active_led_pin));         // Toggle the run light to signal problem
    } while (trys < 60);
    wdt_enable(WDTO_15MS);  // Enable the watchdog timer with a timeout of 15 ms
    while (true) {}         // Infinite loop to allow the watchdog to reset the microcontroller
}