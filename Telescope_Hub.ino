/*
  Arduino Ethernet Telescope Hub
        Interfaces Steve and Jamie Gould's Telescope to a ASCOM compliant software driver
        Declination = Altitude = north/south = up down
        Right Ascension = Azimuth  = east/west = left right
        Communications to the motor controllers is made through this HUB.
    Functionality:
    1. Receive packets of information from the CPU (Windows PC)
        a) Receive packets from CPU and respond with ACK
        a) If the target is the Hub execute the contained command
        b) Otherwise forward the received packet to the indicated target
    2. Receive packets of information from the attached devices
        a)  Forward the received packets to the indicated target
*/
/* Version Control --------------------------------------------------------------------------------
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
*/
constexpr double Firmware_Version = (double)2.5;
// -------------------------------------------------------------------------------------------------
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

// Inclusions -------------------------------------------------------------------------------------
//#define SD_DEBUG
//#define DISPLAY_FREE_MEMORY
//#define PRINT_CPU_INCOMING
//#define SIMULATE_CPU_INCOMING_PACKETS           // Simulate the receipt of packets from the CPU
//#define SIMULATE_ALT_INCOMING_PACKETS
//#define SIMULATE_AZI_INCOMING_PACKETS
//#define SIMULATE_FOC_INCOMING_PACKETS
#include <C:\Users\Stephen\Dropbox\Projects\Combined_Telescope\Common_Files\Telescope_Commands.h>
#define PRINT_CONSOLE_MESSAGES
#define console Serial
// Constants --------------------------------------------------------------------------------------
constexpr int Altitude_baud = (int)38400;
constexpr int Azimuth_baud = (int)38400;
constexpr int Focuser_baud = (int)38400;
constexpr unsigned long Led_On_Time = (unsigned long)250;
// Constants ---------------------------------------------------------------------------------------
// Freememory calculater - Returns the current amount of free memory in bytes ----------------------
extern unsigned int __bss_end;
extern void* __brkval;
int freeMemory() {
    int free_memory;
    if ((int)__brkval)
        return ((int)&free_memory) - ((int)__brkval);
    return ((int)&free_memory) - ((int)&__bss_end);
}
// Hardware configuration -------------------------------------------------------------------------
// Communications Connections ---------------------------------------------------------------------
constexpr uint8_t Altitude_TX_pin = 18;     // Altitude Port TX
constexpr uint8_t Altitude_RX_pin = 19;     // Altitude Port RX
constexpr uint8_t Azimuth_TX_pin = 16;      // Azimuth Port TX
constexpr uint8_t Azimuth_RX_pin = 17;      // Azimuth Port RX
constexpr uint8_t Focuser_TX_pin = 14;      // Focuser Port TX
constexpr uint8_t Focuser_RX_pin = 15;      // Focuser Port RX 
// Peripheral Connections --------------------------------------------------------------------------
constexpr uint8_t RUN_Active_led_pin = 3;   // RUN led
constexpr uint8_t SD_CS = 4;                // SD chip select
constexpr uint8_t Ambient_Sensor_pin = 5;	// ambient temperature and humidity pin
constexpr uint8_t Fan_pin = 6;              // fan (relay) pin
constexpr uint8_t Reset_Switch_pin = 7;     // reset switch pin
constexpr uint8_t Shield_led_pin = 9;       // led on the Ethernet Shield 2
constexpr uint8_t W5500_CS = 10;            // Ethernet chip select
constexpr uint8_t Voltage_pin = A2;         // A2	motor_voltage
// -------------------------------------------------------------------------------------------------
constexpr uint8_t MAXIMUM_FIELDS_IN_PACKET = 10;        //
constexpr uint8_t MAX_FIELD_LENGTH = 20;                // Maximum length of each field (adjust as needed)
constexpr double Fan_Switch_On_Temperature = 30.00;     // Temperature at which fan should turn on
constexpr double Fan_Switch_Off_Temperature = 25.00;    // Temperature at which fan should turn off
// -------------------------------------------------------------------------------------------------
double Ambient_Temperature = 0;             // Temperature value
double Ambient_Humidity = 0;                // Humidity value
double Motor_Voltage = 0;                   // Voltage value
String Retrieved_Timestamp;
String Retrieved_Message;
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
EthernetClient client;                                      // Create an Ethernet Client (CPU)
IPAddress timeServer(216, 23, 247, 62);                     // NTP server from https://tf.nist.gov/tf-cgi/servers.cgi
EthernetUDP ethernet_UDP;                                   // define Ethernet UDP object and local port 8888
unsigned int localPort = 8888;
unsigned int ntpSyncTime = 3600;
const long timeZoneOffset = -14400L;                        // offset (in seconds) to GMT - 4 */
const int NTP_PACKET_SIZE = 48;                             // NTP time stamp is in the first 48 bytes of the message
byte NTP_Packet_Buffer[NTP_PACKET_SIZE];                    // Buffer to hold incoming and outgoing packets
EthernetUDP UDP;                                            // A UDP instance to let us send and receive packets over UDP
unsigned long ntpLastUpdate = 0;                            // Keeps track of how long ago we updated the NTP server
time_t prevDisplay = 0;                                     // last time the Date and Time were displayed
char Display_Buffer[100];                                   // space for formatted monitor display

// -------------------------------------------------------------------------------------------------
Sd2Card card;
SdVolume volume;
SdFile root;
File LogFile;
//struct tm timeinfo;
// -------------------------------------------------------------------------------------------------
DHT_Unified Ambient_Sensor(Ambient_Sensor_pin, DHT22);
Bounce Reset_Switch = Bounce();
// Communications Variables -----------------------------------------------------------------------
char Incoming_Packet_from_CPU[0xFF];
char Incoming_Packet_from_Altitude[0xFF];
char Incoming_Packet_from_Azimuth[0xFF];
char Incoming_Packet_from_Focuser[0xFF];
char Outgoing_Packet[0xFF];
unsigned long CPU_Packet_Received_Count = 0;
unsigned long CPU_Packet_Transmitted_Count = 0;
unsigned long ALT_Packet_Received_Count = 0;
unsigned long ALT_Packet_Transmitted_Count = 0;
unsigned long AZI_Packet_Received_Count = 0;
unsigned long AZI_Packet_Transmitted_Count = 0;
unsigned long FOC_Packet_Received_Count = 0;
unsigned long FOC_Packet_Transmitted_Count = 0;
uint8_t CPU_string_ptr;
uint8_t CPU_packet_length = 0;
uint8_t Altitude_inptr;					// must be 8 bit uint8_t so that it overflows at 256
uint8_t Altitude_outptr;				// must be 8 bit uint8_t so that it overflows at 256
uint8_t Altitude_inbuffer[0xff];
uint8_t Altitude_string_ptr;
uint8_t Altitude_packet_length = 0;
uint8_t Azimuth_inptr;					// must be 8 bit uint8_t so that it overflows at 256
uint8_t Azimuth_outptr;					// must be 8 bit uint8_t so that it overflows at 256
uint8_t Azimuth_inbuffer[0xff];
uint8_t Azimuth_string_ptr;
uint8_t Azimuth_packet_length = 0;
uint8_t Focuser_inptr;
uint8_t Focuser_outptr;
uint8_t Focuser_inbuffer[0xff];
uint8_t Focuser_string_ptr;
uint8_t Focuser_packet_length = 0;
// Packet Fields ----------------------------------------------------------------------------------
char Packet_Field[MAXIMUM_FIELDS_IN_PACKET][MAX_FIELD_LENGTH]; // space for the decoded command string, used when packet target = hub
// ------------------------------------------------------------------------------------------------
uint16_t Device_Status = 0;
enum { OFF = 0, ON = 1 };
unsigned long RUN_Active_Led_Start_Time = 0;
unsigned long Shield_Led_Start_Time = 0;
// Date and Time Fields -----------------------------------------------------------------------------------------------0
struct Date_Time {
    int Second;                     // [0 - 2]
    int Minute;                     // [3 - 4]
    int Hour;                       // [5 - 6]
    int Day;                        // [7 - 8]
    int Month;                      // [9 - 10]
    int Year;                       // [11 - 12]
    char Date[11];                  // [13 - 23]    "18/11/1951"
    char Time[9];                   // [24 - 32]    "00:00:00"
}__attribute__((packed));
constexpr int Date_Time_Record_Length = 32;
union Date_Time_Union {
    Date_Time field;
    unsigned char character[Date_Time_Record_Length + 1];
};
Date_Time_Union Current_Date_Time_Data;
String Current_Date;
String Current_Time;
String Current_Date_and_Time;
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
    console_print(true, F("Setup Commenced"));
    pinMode(RUN_Active_led_pin, OUTPUT);
    Led_Control(RUN_Active_led_pin, ON);            // turn the run led on
    Reset_Switch.attach(Reset_Switch_pin);
    Reset_Switch.interval(5);
    pinMode(Shield_led_pin, OUTPUT);
    console_print(true, F("Initialising SD Drive"));
    if (!card.init(SPI_HALF_SPEED, SD_CS)) {
        console_print(true, F("Initialisation Failed"));
        Wait_for_Reset_Switch();
    }
    else {
        console_print(true, F("\tInitialisation Succeeded"));
        console_print(true, F("\tWiring is correct and a card is present."));
    }
    switch (card.type()) {
    case SD_CARD_TYPE_SD1:
        console_print(true, F("\tCard Type: \t\tSD1"));
        break;
    case SD_CARD_TYPE_SD2:
        console_print(true, F("\tCard Type: \t\tSD2"));
        break;
    case SD_CARD_TYPE_SDHC:
        console_print(true, F("\tCard Type:\t\tSDHC"));
        break;
    default:
        console_print(true, F("\tCard Type:\tUnknown"));
    }
    if (!volume.init(card)) {
        console_print(true, F("\tCould not find FAT16/FAT32 partition.\nMake sure the card is formatted (FAT16/FAT32)"));
        Wait_for_Reset_Switch();
    }
    uint32_t volumesize;
    volumesize = (volume.blocksPerCluster() * volume.clusterCount());
    snprintf(Display_Buffer, sizeof(Display_Buffer), "\tTotal Blocks:\t\t%lu", volumesize);
    console_print(true, Display_Buffer);
    snprintf(Display_Buffer, sizeof(Display_Buffer), "\tVolume type is:\t\tFAT%d", volume.fatType());
    console_print(true, Display_Buffer);
    volumesize = volume.blocksPerCluster();     // clusters are collections of blocks
    volumesize *= volume.clusterCount() / 2048;   // blocks are always 512 bytes (2 blocks are 1KB)
    snprintf(Display_Buffer, sizeof(Display_Buffer), "\tVolume size:\t\t%lu (Mb)", volumesize);
    console_print(true, Display_Buffer);
    //    console_print(true,F(false,"Files found on the card (name, date and size in bytes): "));
    //    root.openRoot(volume);
    //    root.ls(LS_R | LS_DATE | LS_SIZE);    // list all files in the card with date and size
    console_print(true, F("SD Initialisation Complete"));
    console_print(true, F("Deleting any Existing Log File"));
    Check_Log_File();
    console_print(true, F("New Log Created"));
    console_print(true, F("Starting Ethernet Initialisation"));
    Ethernet.begin(mac, ip, my_dns, gateway, subnet);                         // Start Ethernet
    delay(1000);
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        console_print(true, F("Ethernet Shield not found"));
        Wait_for_Reset_Switch();
    }
    else {
        console_print(true, F("\tEthernet Shield Found"));
    }
    if (Ethernet.linkStatus() == LinkOFF) {
        console_print(true, F("\tEthernet cable not connected"));
    }
    else {
        console_print(true, F("\tEthernet cable connected"));
    }
    EthernetClient client = server.accept();
    console_print(true, F("Ethernet Initialisation Complete"));
    console_print(true, F("Date and Time Server Initialisation"));
    int trys = 0;
    do {
        if (!Get_Time_and_Date()) {
            snprintf(Display_Buffer, sizeof(Display_Buffer), "\tGet Time and Date, Attempt Number: %d", trys);
            console_print(true, Display_Buffer);
            trys++;
        }
    } while (trys < 10);
    if (trys >= 10) {
        bitWrite(Device_Status, 5, 0);                          // set the status bit Date and Time false
        Current_Date = __DATE__;      // Assign the compilation date
        Current_Time = __TIME__;      // Assign the compilation time
        // Concatenate the date and time into one string
        Current_Date_and_Time = Current_Date + " " + Current_Time;
        console_print(true, F("Failed to Get Date and Time")); // This line is for error handling, adjust as needed
        snprintf(Display_Buffer, sizeof(Display_Buffer), "\tDate and Time Set to :\%s", Current_Date_and_Time.c_str());
        console_print(true, Display_Buffer);
    }
    else {
        bitWrite(Device_Status, 5, 1);                          // set the status bit Date and Time true
        Clock_Display();                                        // and display the clock
    }
    console_print(true, F("Date and Time Server Initialised"));
    pinMode(Voltage_pin, INPUT);
    pinMode(Fan_pin, OUTPUT);                                       // specify the fan pin as an output
    console_print(true, F("Temperature and Humidity Sensor Initialisation"));
    sensors_event_t event;
    Ambient_Sensor.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        console_print(true, F("\tTemperature Invalid or Sensor not Connected"));
    }
    else {
        snprintf(Display_Buffer, sizeof(Display_Buffer), "\tTemperature Sensor:%.2f", (double)event.temperature);
        console_print(true, Display_Buffer);
    }
    Ambient_Sensor.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        console_print(true, F("\tRelative Humidity Invalid or Sensor not Connected"));
    }
    else {
        snprintf(Display_Buffer, sizeof(Display_Buffer), "\tHumidity Sensor:%.2f", (double)event.relative_humidity);
        console_print(true, Display_Buffer);
    }
    console_print(true, F("Sensor Initialisation Complete"));
    console_print(true, F("Serial Port Initialisation"));
    pinMode(Altitude_RX_pin, INPUT);
    if (digitalRead(Altitude_RX_pin)) {
        console_print(true, F("\tAltitude Communication Line Connected"));
    }
    else {
        console_print(true, F("\tAltitude Communication Line not Connected"));
    }
    Altitude_Port.begin(Altitude_baud, SERIAL_8N2);					// initialise the Altitude serial port    
    Altitude_Port.flush();                                          // clear the Altitude serial buffer
    pinMode(Azimuth_RX_pin, INPUT);
    if (digitalRead(Azimuth_RX_pin)) {
        console_print(true, F("\tAzimuth Communication Line Connected"));
    }
    else {
        console_print(true, F("\tAzimuth Communication Line not Connected"));
    }
    Azimuth_Port.begin(Azimuth_baud, SERIAL_8N2);					// initialise the Azimuth serial port
    Azimuth_Port.flush();											// clear the Azimuth serial buffer
    pinMode(Focuser_RX_pin, INPUT);
    if (digitalRead(Focuser_RX_pin)) {
        console_print(true, F("\tFocuser Communication Line Connected"));
    }
    else {
        console_print(true, F("\tFocuser Communication Line not Connected"));
    }
    Focuser_Port.begin(Focuser_baud, SERIAL_8N2);					// initialise the Focuser serial port
    Focuser_Port.flush();											// clear the Focuser serial buffer
    console_print(true, F("Serial Port Initialisation Complete"));
    console_print(true, F("Enabling WatchDog Timer"));
    wdt_enable(WDTO_4S);                                    // 4 second timeout
    if (getWdtTimeoutMs()) {
        snprintf(Display_Buffer, sizeof(Display_Buffer), "\tCurrent Watchdog Timeout: %d (mS)", getWdtTimeoutMs());
        console_print(true, Display_Buffer);
    }
    else {
        console_print(true, F("\tWatchdog Timer Initialisation failure"));
    }
    console_print(true, F("Watchdog Timer Initialisation Complete"));
#ifdef SIMULATE_CPU_INCOMING_PACKETS
    console_print(true, F("Simulating CPU Incoming Packets"));
#endif
#ifdef SIMULATE_ALT_INCOMING_PACKETS
    console_print(true, F("Simulating ALT Incoming Packets"));
#endif
#ifdef SIMULATE_AZI_INCOMING_PACKETS
    console_print(true, F("Simulating AZI Incoming Packets"));
#endif
#ifdef SIMULATE_FOC_INCOMING_PACKETS
    console_print(true, F("Simulating FOC Incoming Packets"));
#endif
    Led_Control(RUN_Active_led_pin, ON);
    console_print(true, F("Logging to SD Drive now Active"));
    console_print(true, F("Setup Complete"));
} // end setup
// Main -------------------------------------------------------------------------------------------
void loop() {
    wdt_reset();                                                            // keep watch dog timer active
    Maintain_Internet();
    Led_Control(RUN_Active_led_pin, ON);
    if (Check_CPU_Packet_Received()) {
        if (!Process_CPU_Packet()) {
            console_print(true, F("Bad Packet Received from CPU"));
        }
    }
    if (Check_Altitude_Packet_Received()) Copy_Packet_to_CPU(ALT);
    if (Check_Azimuth_Packet_Received()) Copy_Packet_to_CPU(AZI);
    if (Check_Focuser_Packet_Received()) Copy_Packet_to_CPU(FOC);
    Check_Lights();
    Update_Environmental_Sensors();
    Update_Time_and_Date();
}// end of main loop ------------------------------------------------------------------------------
void console_print(bool line_feed, const __FlashStringHelper* message) {
    Serial.print(millis(), DEC);  // Print the current timestamp
    Serial.print("\t");            // Tab for spacing
    Serial.print(message);       // Print the message
    if (line_feed) Serial.println();
}
void console_print(bool line_feed, const char* message) {
    Serial.print(millis(), DEC);
    Serial.print("\t");
    Serial.print(message);  // Use Serial.print to avoid a newline
    if (line_feed) Serial.println();
}
void Save_Packet_to_Disk(char target, char* data, char size) {
    console_print(true, F("\tWriting Packet to Disk"));
    SD.open("log.csv", FILE_WRITE);
    if (LogFile) {
        if (bitRead(Device_Status, 5)) {
            LogFile.print(Current_Date_and_Time);                        // save timestamp
        }
        else {
            LogFile.print(__DATE__); LogFile.print(__TIME__);
        }
        LogFile.print(",");
        for (int i = 0; i < size; i++) {                        // save message
            LogFile.print(data[i]);
        }
        LogFile.close();
        LogFile.flush();
    }
    else {
        console_print(true, F("Log File Write Failed"));
        Wait_for_Reset_Switch();
    }
}
// Received Character Handling --------------------------------------------------------------------
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
bool Check_CPU_Packet_Received(void) {
#ifdef SIMULATE_CPU_INCOMING_PACKETS
    if (millis() > CPU_Time_to_Send_Next_Packet) {
        CPU_Time_to_Send_Next_Packet = millis() + Time_Between_CPU_Packets;
        strcpy(Incoming_Packet_from_CPU, Standard_CPU_Packets[CPU_Simulation_Packet_Pointer]);
        CPU_packet_length = strlen(Incoming_Packet_from_CPU);
        CPU_Simulation_Packet_Pointer++;
        if (CPU_Simulation_Packet_Pointer > Number_of_Standard_CPU_Packets) CPU_Simulation_Packet_Pointer = 0;
        bitWrite(Device_Status, 1, 1);                                      // set CPU Active bit true
        CPU_Packet_Received_Count++;
        return true;
    }
    return false;
#else
#ifdef PRINT_CPU_INCOMING
    if (client.available()) {
        console_print(true, "\tIncoming Data from CPU:");
    }
#endif
    while (client && client.available() > 0) {
        uint8_t thisbyte = client.read();                               // read the character
        if (thisbyte == (uint8_t)SOH) {                                 // look for start character
#ifdef PRINT_CPU_INCOMING
            console.print(thisbyte, DEC); console.print(",");
#endif
            CPU_string_ptr = 0;                                         // start character received so zero the string pointer
            Incoming_Packet_from_CPU[CPU_string_ptr++] = (uint8_t)SOH;  // start of packet, store and increment the string pointer
        }
        else {
            if (thisbyte == (uint8_t)EOT) {                             // characters was not an SOH check for ETX
#ifdef PRINT_CPU_INCOMING
                console.println(thisbyte, DEC);
#endif
                Incoming_Packet_from_CPU[CPU_string_ptr++] = (uint8_t)EOT;   // save the EOT and increment the string pointer
                CPU_packet_length = CPU_string_ptr - 1;                 // record the received packet length
                CPU_string_ptr = 0;                                     // zero the string pointer
                bitWrite(Device_Status, 1, 1);                          // set the Hub status CPU Active bit
                CPU_Packet_Received_Count++;                            // increment CPU packets received count
                Print_Byte_to_server(ACK);                              // send a packet receipt to CPU
                return true;
            }
            else {
                Incoming_Packet_from_CPU[CPU_string_ptr++] = thisbyte;       // Not a control so save it and increment string pointer
#ifdef PRINT_CPU_INCOMING
                console.print(thisbyte, DEC); console.print(",");
#endif
            }
        }
    }
    return false;
#endif 
}
bool Check_Altitude_Packet_Received(void) {
#ifdef SIMULATE_ALT_INCOMING_PACKETS
    if (millis() > ALT_Time_to_Send_Next_Packet) {
        ALT_Time_to_Send_Next_Packet = millis() + Time_Between_ALT_Packets;
        for (int i = 0; i < sizeof(Standard_ALT_Packets[ALT_Simulation_Packet_Pointer]); i++) {
            Incoming_Packet_from_Altitude[i] = Standard_ALT_Packets[ALT_Simulation_Packet_Pointer][i];
        }
        Altitude_packet_length = strlen(Incoming_ALT_Packet);
        ALT_Simulation_Packet_Pointer++;
        if (ALT_Simulation_Packet_Pointer > Number_of_Standard_ALT_Packets) ALT_Simulation_Packet_Pointer = 0;
        ALT_Packet_Received_Count++;
        return true;
    }
#else
    while (Altitude_outptr != Altitude_inptr) {                                 // check Altitude serial buffer for data
        uint8_t thisbyte = Altitude_inbuffer[Altitude_outptr++];                // take a characters from the input buffer and increment pointer
        if (thisbyte == (uint8_t)SOH) {                                         // look for the SOH
            Incoming_Packet_from_Altitude[Altitude_string_ptr++] = (uint8_t)SOH; // store the SOH and increment the string pointer
        }
        else {
            if (thisbyte == (uint8_t)EOT) {                                         // characters was not an SOH check for ETX
                Incoming_Packet_from_Altitude[Altitude_string_ptr++] = (uint8_t)EOT; // save the EOT and increment the string pointer
                Altitude_packet_length = Altitude_string_ptr - 1;
                Altitude_string_ptr = 0;                                            // zero the string pointer
                ALT_Packet_Received_Count++;
                return true;
            }
            else {
                Incoming_Packet_from_Altitude[Altitude_string_ptr++] = thisbyte; // Not a control so save it and increment string pointer
            }
        }
    } // end of while Altitude
    return false;
#endif
}
bool Check_Azimuth_Packet_Received(void) {
#ifdef SIMULATE_AZI_INCOMING_PACKETS
    if (millis() > AZI_Time_to_Send_Next_Packet) {
        AZI_Time_to_Send_Next_Packet = millis() + Time_Between_AZI_Packets;
        for (int i = 0; i < sizeof(Standard_AZI_Packets[AZI_Simulation_Packet_Pointer]); i++) {
            Incoming_Packet_from_Azimuth[i] = Standard_AZI_Packets[AZI_Simulation_Packet_Pointer][i];
        }
        Azimuth_packet_length = strlen(Incoming_AZI_Packet);
        AZI_Simulation_Packet_Pointer++;
        if (AZI_Simulation_Packet_Pointer > Number_of_Standard_AZI_Packets) AZI_Simulation_Packet_Pointer = 0;
        AZI_Packet_Received_Count++;
        return true;
    }
#else
    while (Azimuth_outptr != Azimuth_inptr) {                                   // check Azimuth serial buffer for data
        uint8_t thisbyte = Azimuth_inbuffer[Azimuth_outptr++];                  // take a characters from the input buffer and increment pointer
        if (thisbyte == (uint8_t)SOH) {                                         // look for the SOH
            Incoming_Packet_from_Azimuth[Azimuth_string_ptr++] = (uint8_t)SOH;  // store the SOH and increment the string pointer
        }
        else {
            if (thisbyte == (uint8_t)EOT) {                                          // characters was not an STX check for ETX
                Incoming_Packet_from_Azimuth[Azimuth_string_ptr++] = (uint8_t)EOT;  // save the EOT and increment the string pointer
                Azimuth_packet_length = Azimuth_string_ptr - 1;
                Azimuth_string_ptr = 0;                                             // zero the string pointer
                AZI_Packet_Received_Count++;
                return true;
            }
            else {
                Incoming_Packet_from_Azimuth[Azimuth_string_ptr++] = thisbyte;  // Not a control so save it and increment string pointer
            }
        }
    } // end of while Azimuth
    return false;
#endif
}
bool Check_Focuser_Packet_Received(void) {
#ifdef SIMULATE_FOC_INCOMING_PACKETS
    if (millis() > FOC_Time_to_Send_Next_Packet) {
        FOC_Time_to_Send_Next_Packet = millis() + Time_Between_FOC_Packets;
        for (int i = 0; i < sizeof(Standard_FOC_Packets[FOC_Simulation_Packet_Pointer]); i++) {
            Incoming_Packet_from_Focuser[i] = Standard_FOC_Packets[FOC_Simulation_Packet_Pointer][i];
        }
        Focuser_packet_length = strlen(Incoming_FOC_Packet);
        FOC_Simulation_Packet_Pointer++;
        if (FOC_Simulation_Packet_Pointer > Number_of_Standard_FOC_Packets) FOC_Simulation_Packet_Pointer = 0;
        FOC_Packet_Received_Count++;
        return true;
    }
#else
    while (Focuser_outptr != Focuser_inptr) {                                   // check Focuser serial buffer for data
        uint8_t thisbyte = Focuser_inbuffer[Focuser_outptr++];                  // take a characters from the input buffer and increment pointer
        if (thisbyte == (char)SOH) {                                            // look for the SOH
            Incoming_Packet_from_Focuser[Focuser_string_ptr++] = (uint8_t)SOH;  // store the SOH and increment the string pointer
        }
        else {
            if (thisbyte == (char)EOT) {                                            // characters was not an STX check for ETX
                Incoming_Packet_from_Focuser[Focuser_string_ptr++] = (uint8_t)EOT;  // save the EOT and increment the string pointer
                Focuser_packet_length = Focuser_string_ptr - 1;
                Focuser_string_ptr = 0;                                             // zero the string pointer
                FOC_Packet_Received_Count++;
                return true;
            }
            else {
                Incoming_Packet_from_Focuser[Focuser_string_ptr++] = thisbyte; // Not a control so save it and increment string pointer
            }
        }
    } // end of while Focuser
    return false;
#endif
}
// Process Received Packets -----------------------------------------------------------------------
bool Decode_Fields(char* str, char Packet_Fields[][20]) {
    uint8_t count = 0;
    uint8_t packet_size = strlen(str);                              // determine the size of the input packet
    // 1. Create a temporary array
    char* temp_command_string = (char*)malloc(packet_size * sizeof(char)); // create a temporary array
    if (temp_command_string == NULL) {                              // Check if the memory allocation was successful
        console_print(true, F("Memory allocation failed!"));
        while (1);                                                  // Stop the program if memory allocation fails
    }
    // 2. Copy the coded message, from the STX to end into the temporary array
    char* STX_position = strchr(str, (int)STX);                     // determine the position of the STX
    if (!STX_position) {                                            // if not found return error
        return false;
    }
    else {
        for (int i = (int)STX_position + 1; i < packet_size; i++) { // copy the data part to temp_command_string
            temp_command_string[count++] = str[i];
        }
        // 3.Unpack the fields into character strings Packet_Fields[n]
        char* token = strtok(temp_command_string, (const char*)&FLD);   // separate the fields into Packet_Fields
        while (token != NULL) {
            strcpy(Packet_Fields[count++], token);
            token = strtok(NULL, (const char*)&FLD);
        }
    }
    free(temp_command_string);                                          // free up the allocated space
    return true;
}
int Obtain_Int_Parameter(int parameter_number) {
    return atoi(Packet_Field[parameter_number]);
}
double Obtain_Float_Parameter(int parameter_number) {
    return atof(Packet_Field[parameter_number]);
}
bool Obtain_Bool_Parameter(int parameter_number) {
    return (bool)Packet_Field[parameter_number];
}
long Obtain_Long_Parameter(int parameter_number) {
    return atol(Packet_Field[parameter_number]);
}
bool Process_CPU_Packet() {                                         // Process a packet from the CPU  
    console_print(true, F("Packet Received from CPU"));
    switch (Incoming_Packet_from_CPU[TARGET]) {                          // switch on the target
    case HUB: {
        if (!Decode_Fields(Incoming_Packet_from_CPU, Packet_Field)) {
            console_print(true, F("Corrupt Packet from CPU, target was HUB"));
            return false;
        }
        else {
            console_print(true, F("Packet Received from CPU, target was HUB"));
        }
        switch (Incoming_Packet_from_CPU[COMMAND]) {                      // switch on the Command Number
        case (Reset): {
#ifdef SIMULATION
            console_print(true, F("Restart Requested by CPU"));
#endif
            wdt_enable(WDTO_15MS);  // Enable the watchdog timer with a timeout of 15 ms
            while (true) {}         // Infinite loop to allow the watchdog to reset the microcontroller
            break;
        }
        case (Environment): {
            if (Incoming_Packet_from_CPU[4] == (uint8_t)GET) {
#ifdef SIMULATION
                console_print(true, F("Environment Get Received from CPU"));
#endif
                Send_Reply_to_CPU((int)Environment);
            }
            else if (Incoming_Packet_from_CPU[4] == (uint8_t)SET) {
#ifdef SIMULATION
                console_print(true, F("Environment Set Lights Received from CPU"));
#endif
                if (Obtain_Bool_Parameter(1)) {
                    bitWrite(Device_Status, 2, 1);
                }
                else {
                    bitWrite(Device_Status, 2, 0);
                }
            }
            break;
        }
        case (FirmwareVersion): {
#ifdef SIMULATION
            console_print(true, F("Firmware Version Get Received from CPU"));
#endif
            Send_Reply_to_CPU((int)Firmware_Version);
            break;
        }
        case (Statistics): {
#ifdef SIMULATION
            console_print(true, F("Statistics Get Received from CPU"));
#endif
            Send_Reply_to_CPU((int)Statistics);
            break;
        }
        case (Retrieve): {
            int character_count = 0;
            char field[25];
            int datafieldNo = 0;
            char datatemp;
#ifdef SIMULATION
            console_print(true, F("Retrieve Get Received from CPU"));
#endif
            LogFile = SD.open("log.csv", FILE_READ);                 // open the SD file
            console_print(true, F("\tProcessing Log.csv"));
            if (!LogFile) {                                                                    // oops - file not available!
                Serial.print("Error re-opening LogFile:");
                Wait_for_Reset_Switch();                                                             // Reset will restart the processor so no return
            }
            else {
                while (LogFile.available()) {                           // whilst there is data in the log file
                    datatemp = LogFile.read();                          // read character into datatemp
                    field[character_count++] = datatemp;                            // add it to the csvfield string
                    if (datatemp == '\n' || datatemp == ',') {          // end of field or line detected
                        field[character_count - 1] = '\0';              // insert termination character where the ',' or '\n' was
                        switch (datafieldNo) {                          // store the field into appropriate variable
                        case 0: {
                            Retrieved_Timestamp = field;
                            break;
                        }
                        case 1: {
                            Retrieved_Message = field;
                        }
                        }
                        datafieldNo++;
                        field[0] = '\0';
                        character_count = 0;
                    }
                    if (datatemp == '\n') {                             // at this point the obtained record has been retrieved from SD
                        Send_Reply_to_CPU(Retrieve);
                        delay(1000);                                    // delay 1 second before sending next record
                    }
                }
                LogFile.close();
                LogFile.flush();
                SD.remove("/Log.csv");                                  // delete the log file
            }
            break;
        }
        case (DateTime): {
#ifdef SIMULATION
            console_print(true, F("Date and Time Get Received from CPU"));
#endif
            Send_Reply_to_CPU(DateTime);
            break;
        }
        case (DeleteLog): {
#ifdef SIMULATION
            console_print(true, F("Delete Log File Received from CPU"));
#endif
            SD.remove("/Log.csv");                                  // delete the log file
            break;
        }
        default: {
#ifdef SIMULATION
            console_print(true, F("Unknown Command Received from CPU"));
#endif
            Send_Reply_to_CPU((int)Obtain_Int_Parameter(0));
            break;
        }
        }                                                  // end of switch on command number
    }
    case ALT: {                                                     // send the packet to the ALT
        console_print(true, F("Packet Destination Altitude"));
        Transmit_Packet_to_Target((char)ALT, Incoming_Packet_from_CPU, CPU_packet_length);
        Save_Packet_to_Disk((char)ALT, Incoming_Packet_from_CPU, CPU_packet_length);
        break;
    }
    case AZI: {
        console_print(true, F("Packet Destination Azimuth"));
        Transmit_Packet_to_Target((char)AZI, Incoming_Packet_from_CPU, CPU_packet_length);
        Save_Packet_to_Disk((char)AZI, Incoming_Packet_from_CPU, CPU_packet_length);
        break;
    }
    case BTH: {
        console_print(true, F("Packet Destination Both Motors"));
        Transmit_Packet_to_Target((char)ALT, Incoming_Packet_from_CPU, CPU_packet_length);
        Save_Packet_to_Disk((char)ALT, Incoming_Packet_from_CPU, CPU_packet_length);
        Transmit_Packet_to_Target((char)AZI, Incoming_Packet_from_CPU, CPU_packet_length);
        Save_Packet_to_Disk((char)AZI, Incoming_Packet_from_CPU, CPU_packet_length);
        break;
    }
    case FOC: {
        console_print(true, F("Packet Destination Focuser"));
        Transmit_Packet_to_Target((char)FOC, Incoming_Packet_from_CPU, CPU_packet_length);
        Save_Packet_to_Disk((char)FOC, Incoming_Packet_from_CPU, CPU_packet_length);
        break;
    }
    case ALL: {
        console_print(true, F("Packet Destination All Devices"));
        Transmit_Packet_to_Target((char)ALT, Incoming_Packet_from_CPU, CPU_packet_length);
        Save_Packet_to_Disk((char)ALT, Incoming_Packet_from_CPU, CPU_packet_length);
        Transmit_Packet_to_Target((char)AZI, Incoming_Packet_from_CPU, CPU_packet_length);
        Save_Packet_to_Disk((char)AZI, Incoming_Packet_from_CPU, CPU_packet_length);
        Transmit_Packet_to_Target((char)FOC, Incoming_Packet_from_CPU, CPU_packet_length);
        Save_Packet_to_Disk((char)FOC, Incoming_Packet_from_CPU, CPU_packet_length);
    }                                                   // end of switch target
    }
    return true;
}
void Copy_Packet_to_CPU(uint8_t target) {    // Send message received from ALT,AZI,FOC
    switch (target) {
    case ALT: {
#ifdef PRINT_CONSOLE_MESSAGES
        console_print(true, F("Packet Received from Altitude"));
#endif
        Transmit_Packet_to_Target((char)CPU, Incoming_Packet_from_Altitude, Altitude_packet_length);
        Save_Packet_to_Disk((char)CPU, Incoming_Packet_from_Altitude, Altitude_packet_length);
        break;
    }
    case AZI: {
#ifdef PRINT_CONSOLE_MESSAGES
        console_print(true, F("Packet Received from Azimuth"));
#endif
        Transmit_Packet_to_Target((char)CPU, Incoming_Packet_from_Azimuth, Azimuth_packet_length);
        Save_Packet_to_Disk((char)CPU, Incoming_Packet_from_Azimuth, Azimuth_packet_length);
        break;
    }
    case FOC: {
#ifdef PRINT_CONSOLE_MESSAGES
        console_print(true, F("Packet Received from Focuser"));
#endif
        Transmit_Packet_to_Target((char)CPU, Incoming_Packet_from_Focuser, Focuser_packet_length);
        Save_Packet_to_Disk((char)CPU, Incoming_Packet_from_Focuser, Focuser_packet_length);
        break;
    }
    }
}
void Send_Reply_to_CPU(int command) {
    char temp[20];
    Print_Byte_to_server(SOH);                          // Byte 0   SOH
    Print_Byte_to_server(CPU);                          // Byte 1   Target
    Print_Byte_to_server(HUB);                          // Byte 2   Source
    Print_Byte_to_server(REP);                          // Byte 3   Packet Type
    Print_Byte_to_server(command);                      // Byte 4   Command
    Print_Byte_to_server(STX);                          // Byte 5   STX
    sprintf(temp, "%d", Device_Status);
    Print_String_to_server(temp, strlen(temp));         // Byte 6 & 7
    Print_Byte_to_server(FLD);                          // Byte 8
    if (command == Environment) {
        sprintf(temp, "%.2f", Ambient_Temperature);
        Print_String_to_server(temp, strlen(temp));
        Print_Byte_to_server(FLD);
        sprintf(temp, "%.2f", Ambient_Humidity);
        Print_String_to_server(temp, strlen(temp));
        Print_Byte_to_server(FLD);
        sprintf(temp, "%.2f", Motor_Voltage);
        Print_String_to_server(temp, strlen(temp));
        Print_Byte_to_server(FLD);
        Free_Memory = freeMemory();
        itoa(Free_Memory, temp, 10);
        Print_String_to_server(temp, strlen(temp));
        Print_Byte_to_server(FLD);
    }
    else if (command == FirmwareVersion) {
        sprintf(temp, "%.2f", Firmware_Version);
        Print_String_to_server(temp, strlen(temp));
        Print_Byte_to_server(FLD);
        sprintf(temp, "%.2f", Commands_Version);
        Print_String_to_server(temp, strlen(temp));
        Print_Byte_to_server(FLD);
    }
    else if (command == Statistics) {
        sprintf(temp, "%lu", CPU_Packet_Received_Count);
        Print_String_to_server(temp, strlen(temp));
        Print_Byte_to_server(FLD);
        sprintf(temp, "%lu", CPU_Packet_Transmitted_Count);
        Print_String_to_server(temp, strlen(temp));
        Print_Byte_to_server(FLD);
        sprintf(temp, "%lu", ALT_Packet_Received_Count);
        Print_String_to_server(temp, strlen(temp));
        Print_Byte_to_server(FLD);
        sprintf(temp, "%lu", ALT_Packet_Transmitted_Count);
        Print_String_to_server(temp, strlen(temp));
        Print_Byte_to_server(FLD);
        sprintf(temp, "%lu", AZI_Packet_Received_Count);
        Print_String_to_server(temp, strlen(temp));
        Print_Byte_to_server(FLD);
        sprintf(temp, "%lu", AZI_Packet_Transmitted_Count);
        Print_String_to_server(temp, strlen(temp));
        Print_Byte_to_server(FLD);
        sprintf(temp, "%lu", FOC_Packet_Received_Count);
        Print_String_to_server(temp, strlen(temp));
        Print_Byte_to_server(FLD);
        sprintf(temp, "%lu", FOC_Packet_Transmitted_Count);
        Print_String_to_server(temp, strlen(temp));
        Print_Byte_to_server(FLD);
    }
    else if (command == Retrieve) {
        for (unsigned int i = 0; i < Retrieved_Timestamp.length(); i++) {
            client.print(Retrieved_Timestamp[i]);
        }
        Print_Byte_to_server(FLD);
        for (unsigned int i = 0; i < Retrieved_Message.length(); i++) {
            client.print(Retrieved_Message[i]);
        }
        Print_Byte_to_server(FLD);
    }
    else if (command == DateTime) {
        for (unsigned int i = 0; i < Current_Date.length(); i++) {
            client.print(Current_Date[i]);
        }
        Print_Byte_to_server(FLD);
        for (unsigned int i = 0; i < Current_Time.length(); i++) {
            client.print(Current_Time[i]);
        }
        Print_Byte_to_server(FLD);
    }
    Print_Byte_to_server(ETX);
    Print_Byte_to_server(EOT);
    CPU_Packet_Transmitted_Count++;
}
void Print_Byte_to_server(uint8_t data) {                         // used to send single byte ack to CPU
    client.println(data);
}
void Print_String_to_server(char* data, char size) {              // used to send character string to CPU
    for (int i = 0; i < size; i++) {
        client.println(data[i]);
    }
}
void Transmit_Packet_to_Target(char target, char* data, char size) {
    switch (target) {
    case CPU: {
        for (int i = 0; i < size; i++) {
            while (!server.availableForWrite()) {
                delay(10);
            }
            server.write(data[i]);
        }
#ifdef PRINT_CONSOLE_MESSAGES
        console.print("Packet sent to CPU");
#endif
        CPU_Packet_Transmitted_Count++;
        break;
    }
    case ALT: {
        for (int i = 0; i < size; i++) {
            while (!Altitude_Port.availableForWrite()) {
                delay(10);
            }
            Altitude_Port.write(data[i]);
        }
#ifdef PRINT_CONSOLE_MESSAGES
        console.print("Packet sent to Altitude");
#endif
        ALT_Packet_Transmitted_Count++;
        break;
    }
    case AZI: {
        for (int i = 0; i < size; i++) {
            while (!Azimuth_Port.availableForWrite()) {
                delay(10);
            }
            Azimuth_Port.write(data[i]);
        }
#ifdef PRINT_CONSOLE_MESSAGES
        console.print("Packet sent to Azimuth");
#endif
        AZI_Packet_Transmitted_Count++;
        break;
    }
    case BTH: {
        for (int i = 0; i < size; i++) {
            while (!Altitude_Port.availableForWrite()) {
                delay(10);
            }
            Altitude_Port.write(data[i]);
            while (!Azimuth_Port.availableForWrite()) {
                delay(10);
            }
            Azimuth_Port.write(data[i]);
        }
#ifdef PRINT_CONSOLE_MESSAGES
        console.print("Packet sent to Altitude and Azimuth");
#endif
        ALT_Packet_Transmitted_Count++;
        AZI_Packet_Transmitted_Count++;
        break;
    }
    case FOC: {
        for (int i = 0; i < size; i++) {
            while (!Focuser_Port.availableForWrite()) {
                delay(10);
            }
            Focuser_Port.write(data[i]);
        }
#ifdef PRINT_CONSOLE_MESSAGES
        console.print("Packet sent to Focuser");
#endif
        FOC_Packet_Received_Count++;
        break;
    }
    case ALL: {
        for (int i = 0; i < size; i++) {
            while (!Altitude_Port.availableForWrite()) {
                delay(10);
            }
            Altitude_Port.write(data[i]);
            while (!Azimuth_Port.availableForWrite()) {
                delay(10);
            }
            Azimuth_Port.write(data[i]);
            while (!Altitude_Port.availableForWrite()) {
                delay(10);
            }
            Azimuth_Port.write(data[i]);
            while (!Focuser_Port.availableForWrite()) {
                delay(10);
            }
            Focuser_Port.write(data[i]);
        }
#ifdef PRINT_CONSOLE_MESSAGES
        console.print("Packet sent to All Devices");
#endif
        ALT_Packet_Transmitted_Count++;
        AZI_Packet_Transmitted_Count++;
        FOC_Packet_Transmitted_Count++;
        break;
    }
    default: {
        break;
    }
    }
}
void Check_Log_File() {
#ifdef DISPLAY_FREE_MEMORY
    Serial.print("Free memory before SD operations: ");
    Serial.println(freeMemory());
#endif
    if (SD.begin(SD_CS)) {
        console_print(true, F("\tSD Drive Begin successful"));
    }
    else {
        console_print(true, F("SD Drive Begin failed"));
        Wait_for_Reset_Switch();
    }
#ifdef DISPLAY_FREE_MEMORY
    Serial.print("Free memory after SD.begin: ");
    Serial.println(freeMemory());
#endif
    if (SD.exists("datalog.csv")) {
        console_print(true, F("\tLog File exists, so delete it"));
        SD.remove("datalog.csv");
    }
#ifdef DISPLAY_FREE_MEMORY
    Serial.print("Free memory before opening file: ");
    Serial.println(freeMemory());
#endif
    LogFile = SD.open("datalog.csv", FILE_WRITE);
    if (!LogFile) {
        console_print(true, F("Error Creating Logfile"));
        Wait_for_Reset_Switch();
    }
    else {
        console_print(true, F("\tLog File Created Successfully"));
        LogFile.print(__DATE__);                                        // write the first record to file
        LogFile.print(" ");
        LogFile.println(__TIME__);
        LogFile.print(",File Stated");
        LogFile.close();
    }
#ifdef DISPLAY_FREE_MEMORY
    Serial.print("Free memory after closing file: ");
    Serial.println(freeMemory());
#endif
}
void Update_Environmental_Sensors() {
    sensors_event_t event;
    Ambient_Sensor.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        Ambient_Temperature = 0;
    }
    else {
        Ambient_Temperature = event.temperature;
    }
    Ambient_Sensor.humidity().getEvent(&event);			// Get humidity event and print its value.
    if (isnan(event.relative_humidity)) {
        Ambient_Humidity = 0;
    }
    else {
        Ambient_Humidity = event.relative_humidity;
    }
    Motor_Voltage = digitalRead(Voltage_pin);
    if (Ambient_Temperature > Fan_Switch_On_Temperature) {          // Turn the fan on if necessary
        bitWrite(Device_Status, 8, 1);
        digitalWrite(Fan_pin, ON);
    }
    else if (Ambient_Temperature < Fan_Switch_Off_Temperature) {     // Turn the fan off if necessary
        digitalWrite(Fan_pin, OFF);
        bitWrite(Device_Status, 8, 0);
    }
    Free_Memory = (freeMemory());
}
void Led_Control(uint8_t led, bool state) {
    switch (led) {
    case (RUN_Active_led_pin): {
        bitWrite(Device_Status, 0, state);
        break;
    }
    case (Shield_led_pin): {
        bitWrite(Device_Status, 4, state);
        break;
    }
    }
}
void Check_Lights() {
    if (bitRead(Device_Status, 0)) {                                            // are the lights enabled
        if (bitRead(Device_Status, 0)) {                                    // Run_Active led
            if (millis() >= RUN_Active_Led_Start_Time + Led_On_Time) {
                RUN_Active_Led_Start_Time = millis();
                digitalWrite(RUN_Active_led_pin, !digitalRead(RUN_Active_led_pin));       // toggle the CAM_Active led
            }
            else {
                digitalWrite(RUN_Active_led_pin, OFF);                              // turn the CAM_Active led off
            }
        }
        if (bitRead(Device_Status, 4)) {                                    // Run_Active led
            if (millis() >= Shield_Led_Start_Time + Led_On_Time) {
                Shield_Led_Start_Time = millis();
                digitalWrite(Shield_led_pin, !digitalRead(Shield_led_pin));       // toggle the CAM_Active led
            }
            else {
                digitalWrite(Shield_led_pin, OFF);                              // turn the CAM_Active led off
            }
        }
    }
}
const uint16_t wdtTimeouts[] = { 16, 32, 64, 125, 250, 500, 1000, 2000, 4000, 8000 };
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
void Wait_for_Reset_Switch() {
    while (1) {
        Reset_Switch.update();
        if (Reset_Switch.fell()) {
            Serial.println("Reset Switch Pressed");
            wdt_enable(WDTO_15MS);  // Enable the watchdog timer with a timeout of 15 ms
            while (true) {}         // Infinite loop to allow the watchdog to reset the microcontroller
        }
        delay(500);
    }
}
int Get_Time_and_Date() {
    int flag = 0;
    if (!UDP.begin(localPort)) {
        console_print(true, "Time Server not reached");
        return 0;
    }
    sendNTPpacket(timeServer);
    delay(1000);
    if (UDP.parsePacket()) {
        UDP.readBytes(NTP_Packet_Buffer, NTP_PACKET_SIZE);  // read the packet into the buffer
        unsigned long highWord, lowWord, epoch;
        highWord = word(NTP_Packet_Buffer[40], NTP_Packet_Buffer[41]);
        lowWord = word(NTP_Packet_Buffer[42], NTP_Packet_Buffer[43]);
        epoch = highWord << 16 | lowWord;
        epoch = epoch - 2208988800 + timeZoneOffset;
        flag = 1;
        setTime(epoch);
        ntpLastUpdate = now();
    }
    return flag;
}
void sendNTPpacket(IPAddress& address) {
    memset(NTP_Packet_Buffer, 0, NTP_PACKET_SIZE);
    NTP_Packet_Buffer[0] = 0b11100011;
    NTP_Packet_Buffer[1] = 0;
    NTP_Packet_Buffer[2] = 6;
    NTP_Packet_Buffer[3] = 0xEC;
    NTP_Packet_Buffer[12] = 49;
    NTP_Packet_Buffer[13] = 0x4E;
    NTP_Packet_Buffer[14] = 49;
    NTP_Packet_Buffer[15] = 52;
    UDP.beginPacket(address, 123);
    UDP.write(NTP_Packet_Buffer, NTP_PACKET_SIZE);
    UDP.endPacket();
}
void Format_Date_and_Time(bool format) {
    Current_Date_Time_Data.field.Year = year();
    Current_Date_Time_Data.field.Month = month();
    Current_Date_Time_Data.field.Day = day();
    Current_Date_Time_Data.field.Hour = hour();
    Current_Date_Time_Data.field.Minute = minute();
    Current_Date_Time_Data.field.Second = second();
    // ----------------------------------------------------------------------------------------------------------------
    Current_Date = String(Current_Date_Time_Data.field.Year);            //  1951
    if (format) {
        Current_Date += "/";                            //  1951/
    }
    if (Current_Date_Time_Data.field.Month < 10) {
        Current_Date += "0";                            //  1951/0
    }
    Current_Date += String(Current_Date_Time_Data.field.Month);          //  1951/11
    if (format) {
        Current_Date += "/";                            //  1951/11/
    }
    if (Current_Date_Time_Data.field.Day < 10) {
        Current_Date += "0";                            //  1951/11/0
    }
    Current_Date += String(Current_Date_Time_Data.field.Day);            //  1951/11/18
    // TIME -----------------------------------------------------------------------------------------------------------
    Current_Time = "";
    if (Current_Date_Time_Data.field.Hour < 10) {                            // if hours are less than 10 add a 0
        Current_Time = "0";
    }
    Current_Time += String(Current_Date_Time_Data.field.Hour);           //  add hours
    if (format) {                                           //  add a : if format true
        Current_Time += ":";                            //  23:
    }
    if (Current_Date_Time_Data.field.Minute < 10) {                          //  if minutes are less than 10 add a 0
        Current_Time += "0";                            //  23:0
    }
    Current_Time += String(Current_Date_Time_Data.field.Minute);         //  23:59
    if (format) {
        Current_Time += ":";                            //  23:59:
    }
    if (Current_Date_Time_Data.field.Second < 10) {
        Current_Time += "0";                            //  23:59:0
    }
    Current_Time += String(Current_Date_Time_Data.field.Second);         //  23:59:59
    Current_Date_and_Time = Current_Date + " " + Current_Time;
}
void Update_Time_and_Date() {
    if (!bitRead(Device_Status, 5)) {                            // try getting the date and time again
        if (now() - ntpLastUpdate > ntpSyncTime) {
            int trys = 0;
            while (!Get_Time_and_Date() && trys < 10) {
                trys++;
            }
            if (trys < 10) {
                console_print(true, F("ntp server update success"));
                console_print(true, F("\tSD Drive Begin successful"));
                bitWrite(Device_Status, 5, 1);
            }
            else {
                console_print(true, F("ntp server update failed"));
                bitWrite(Device_Status, 5, 0);
            }
        }
    }
    if (bitRead(Device_Status, 5)) {
        if (now() != prevDisplay) {                 // Display the time if it has changed by more than a second.
            prevDisplay = now();
            Clock_Display();
        }
    }
}
void Clock_Display() {                                   // Clock display of the time and date (Basic)
    Serial.print(millis(), DEC); Serial.print("\t");
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print(" ");
    Serial.print(month());
    Serial.print(" ");
    Serial.print(year());
    Serial.println();
}
void printDigits(int digits) {
    Serial.print(":");
    if (digits < 10)
        Serial.print('0');
    Serial.print(digits);
}
