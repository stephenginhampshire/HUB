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
07/11/2024  2.6     Day of Week calculation added
09/11/2024  2.7     Simulations Debugged and all working
*/
constexpr double Firmware_Version = (double)2.7;
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
//#define PRINT_FIELDS
//#define SIMULATE_INCOMING_PACKETS           // Simulate the receipt of packets from the CPU
#include <C:\Users\Stephen\Dropbox\Projects\Combined_Telescope\Common_Files\Telescope_Commands.h>
//#define PRINT_CONSOLE_MESSAGES
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
/*
0.uk.pool.ntp.org:  109.74.206.120, 176.58.109.199,     94.125.129.7,   5.77.45.219
1.uk.pool.ntp.org:  93.93.131.118,  185.53.93.157,      158.43.128.33,  134.0.16.1
2.uk.pool.ntp.org:  5.77.45.219,    82.219.4.30,        176.58.109.199, 85.119.80.232
3.uk.pool.ntp.org:  149.18.38.230,  176.126.242.239,    91.212.90.20,   188.114.116.1
*/
// IPAddress timeServer(216, 23, 247, 62);                     // NTP server from https://tf.nist.gov/tf-cgi/servers.cgi
IPAddress timeServers[] = {
    {109,74,206,120},                           // [0]
    {176,58,109,199},                           // [1]
    {94,125,129,7},                             // [2]
    {5,77,45,219},                              // [3]
    {93,93,131,118},                            // [4]
    {185,53,93,157},                            // [5]
    {158,43,128,33},                            // [6]
    {134,0,16,1},                               // [7]
    {5,77,45,219},                              // [8]
    {82,219,4,30},                              // [9]
    {176,58,109,199},                           // [10]
    {85,119,80,232},                            // [11]
    {149,18,38,230},                            // [12]
    {176,126,242,239},                          // [13]
    {91,212,90,20},                             // [14]
    {188,114,116,1}                             // [15]
};
int Number_of_TimeServers = 16;
const char* Weekdays[] = {
    "Bad",
    "Sunday",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday"
};
EthernetUDP ethernet_UDP;                                   // define Ethernet UDP object and local port 8888
unsigned int localPort = 8888;
unsigned int ntpSyncTime = 3600;
const long timeZoneOffset = -14400L;                        // offset (in seconds) to GMT - 4 */
const int NTP_PACKET_SIZE = 48;                             // NTP time stamp is in the first 48 bytes of the message
byte NTP_Packet_Buffer[NTP_PACKET_SIZE];                    // Buffer to hold incoming and outgoing packets
EthernetUDP UDP;                                            // A UDP instance to let us send and receive packets over UDP
unsigned long ntpLastUpdate = 0;                            // Keeps track of how long ago we updated the NTP server
int prevDisplay = 0;                                        // last minute the Date and Time were displayed
char Display_Buffer[100];                                   // space for formatted monitor display
const uint16_t wdtTimeouts[] = { 16, 32, 64, 125, 250, 500, 1000, 2000, 4000, 8000 };
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
unsigned long CPU_Packet_Received_Count = 0;
unsigned long CPU_Packet_Transmitted_Count = 0;
unsigned long ALT_Packet_Received_Count = 0;
unsigned long ALT_Packet_Transmitted_Count = 0;
unsigned long AZI_Packet_Received_Count = 0;
unsigned long AZI_Packet_Transmitted_Count = 0;
unsigned long FOC_Packet_Received_Count = 0;
unsigned long FOC_Packet_Transmitted_Count = 0;
uint8_t CPU_string_ptr;
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
uint16_t Device_Status = 0;
enum { OFF = 0, ON = 1 };
unsigned long RUN_Active_Led_Start_Time = 0;
unsigned long Shield_Led_Start_Time = 0;
// Date and Time Fields ---------------------------------------------------------------------------
struct Date_Time {
    uint8_t Second;                     // [0]
    uint8_t Minute;                     // [1]
    uint8_t Hour;                       // [2]
    uint8_t Day;                        // [3]
    uint8_t Month;                      // [4]
    uint16_t Year;                      // [5 - 6]
}__attribute__((packed));
constexpr int Date_Time_Record_Length = 32;
union Date_Time_Union {
    Date_Time field;
    unsigned char character[Date_Time_Record_Length + 1];
};
struct NTP_Date_Format {
    uint8_t Word_40;                    // [0]
    uint8_t Word_41;                    // [1]
    uint8_t Word_42;                    // [2]
    uint8_t Word_43;                    // [3]
}__attribute__((packed));
union NTP_seconds {
    uint32_t Seconds;
    uint8_t Words[4];
};
NTP_seconds NTP_Seconds;
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
        Wait_for_Reset_Switch(F("Initialisation Failed"));
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
        Wait_for_Reset_Switch(F("\tCould not find FAT16/FAT32 partition.\nMake sure the card is formatted (FAT16/FAT32)"));
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
    bitWrite(Device_Status, Disk_Status, 1);
    console_print(true, F("SD Initialisation Complete"));
    console_print(true, F("Deleting any Existing Log File"));
    Check_Log_File();
    console_print(true, F("New Log Created"));
    console_print(true, F("Starting Ethernet Initialisation"));
    Ethernet.begin(mac, ip, my_dns, gateway, subnet);                         // Start Ethernet
    delay(1000);
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Wait_for_Reset_Switch(F("Ethernet Shield not found"));
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
    //EthernetClient client = server.accept();
    bitWrite(Device_Status, Ethernet_Status, 1);
    console_print(true, F("Ethernet Initialisation Complete"));
    // Date and Time -----------------------------------------------------------------------------------
    console_print(true, F("Date and Time Server Initialisation"));
    int trys = 0;
    do {
        if (!Get_Time_and_Date(trys)) {
            snprintf(Display_Buffer, sizeof(Display_Buffer), "\tGet Time and Date, Attempt Number: %d", trys);
            console_print(true, Display_Buffer);
            trys++;
        }
        else {
            break;
        }
    } while (trys < Number_of_TimeServers);
    if (trys >= Number_of_TimeServers) {
        bitWrite(Device_Status, Date_Status, 0);                          // set the status bit Date and Time false
        Current_Date = __DATE__;      // Assign the compilation date
        Current_Time = __TIME__;      // Assign the compilation time
        // Concatenate the date and time into one string
        Current_Date_and_Time = Current_Date + " " + Current_Time;
        console_print(true, F("Failed to Get Date and Time")); // This line is for error handling, adjust as needed
        snprintf(Display_Buffer, sizeof(Display_Buffer), "\tDate and Time Set to :\%s", Current_Date_and_Time.c_str());
        console_print(true, Display_Buffer);
    }
    else {
        bitWrite(Device_Status, Date_Status, 1);                          // set the status bit Date and Time true
        Clock_Display();                                        // and display the clock
    }
    bitWrite(Device_Status, Date_Status, 1);
    console_print(true, F("Date and Time Server Initialised"));
    pinMode(Voltage_pin, INPUT);
    pinMode(Fan_pin, OUTPUT);                                       // specify the fan pin as an output
    console_print(true, F("Temperature and Humidity Sensor Initialisation"));
    sensors_event_t event;
    Ambient_Sensor.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        console_print(true, F("\tTemperature Invalid or Sensor not Connected"));
        bitWrite(Device_Status, Temperature_Status, 0);
    }
    else {
        snprintf(Display_Buffer, sizeof(Display_Buffer), "\tTemperature Sensor:%.2f", (double)event.temperature);
        console_print(true, Display_Buffer);
        bitWrite(Device_Status, Temperature_Status, 1);
    }
    Ambient_Sensor.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        console_print(true, F("\tRelative Humidity Invalid or Sensor not Connected"));
        bitWrite(Device_Status, Humidity_Status, 0);
    }
    else {
        snprintf(Display_Buffer, sizeof(Display_Buffer), "\tHumidity Sensor:%.2f", (double)event.relative_humidity);
        console_print(true, Display_Buffer);
        bitWrite(Device_Status, Humidity_Status, 1);
    }
    if (!digitalRead(Voltage_pin)) {
        bitWrite(Device_Status, Voltage_Status, 0);
    }
    else {
        bitWrite(Device_Status, Voltage_Status, 1);
    }
    console_print(true, F("Sensor Initialisation Complete"));
    console_print(true, F("Serial Port Initialisation"));
    pinMode(Altitude_RX_pin, INPUT);
    if (digitalRead(Altitude_RX_pin)) {
        console_print(true, F("\tAltitude Communication Line Connected"));
        bitWrite(Device_Status, ALT_Status, 1);
    }
    else {
        console_print(true, F("\tAltitude Communication Line not Connected"));
        bitWrite(Device_Status, ALT_Status, 0);
    }
    Altitude_Port.begin(Altitude_baud, SERIAL_8N2);					// initialise the Altitude serial port    
    Altitude_Port.flush();                                          // clear the Altitude serial buffer
    pinMode(Azimuth_RX_pin, INPUT);
    if (digitalRead(Azimuth_RX_pin)) {
        console_print(true, F("\tAzimuth Communication Line Connected"));
        bitWrite(Device_Status, AZI_Status, 1);
    }
    else {
        console_print(true, F("\tAzimuth Communication Line not Connected"));
        bitWrite(Device_Status, AZI_Status, 0);
    }
    Azimuth_Port.begin(Azimuth_baud, SERIAL_8N2);					// initialise the Azimuth serial port
    Azimuth_Port.flush();											// clear the Azimuth serial buffer
    pinMode(Focuser_RX_pin, INPUT);
    if (digitalRead(Focuser_RX_pin)) {
        console_print(true, F("\tFocuser Communication Line Connected"));
        bitWrite(Device_Status, FOC_Status, 1);
    }
    else {
        console_print(true, F("\tFocuser Communication Line not Connected"));
        bitWrite(Device_Status, FOC_Status, 0);
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
#ifdef SIMULATE_INCOMING_PACKETS
    console_print(true, F("Simulating CPU Incoming Packets"));
#endif
    Led_Control(RUN_Active_led_pin, ON);
    console_print(true, F("Logging to SD Drive now Active"));
    console_print(true, F("Setup Complete"));
    console_print(true, F("Starting Main Loop"));
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
    if (Check_Altitude_Packet_Received()) Transmit_Packet_to_Target(CPU, Incoming_Packet_from_Altitude);
    if (Check_Azimuth_Packet_Received()) Transmit_Packet_to_Target(CPU, Incoming_Packet_from_Azimuth);
    if (Check_Focuser_Packet_Received()) Transmit_Packet_to_Target(CPU, Incoming_Packet_from_Focuser);
    Check_Lights();
    Update_Environmental_Sensors();
    Update_Time_and_Date();
}// end of main loop ------------------------------------------------------------------------------
void Save_Packet_to_Log_File(char* data) {
    LogFile = SD.open("log.csv", FILE_WRITE);
    if (LogFile) {
        if (bitRead(Device_Status, Date_Status)) {
            Format_Date_and_Time(true);
            LogFile.print(Current_Date_and_Time);                        // save real timestamp
        }
        else {
            LogFile.print(__DATE__); LogFile.print(__TIME__);           // or compiler timestamp
        }
        LogFile.print(",");                                             // field delimiter
        LogFile.print(data);
        LogFile.println();                                              // terminate the line
        LogFile.close();
        LogFile.flush();
    }
    else {
        Wait_for_Reset_Switch(F("Log File Write Failed"));
    }
}
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
#ifdef SIMULATE_INCOMING_PACKETS
    if (millis() > CPU_Time_to_Send_Next_Packet) {
        CPU_Time_to_Send_Next_Packet = millis() + Time_Between_CPU_Packets;                     // update timing
        for (int i = 0; i < Standard_CPU_Packet_Length; i++) {
            Incoming_Packet_from_CPU[i] = Standard_CPU_Packets[CPU_Simulation_Packet_Pointer][i]; // copy sample packet to received packet
        }
        CPU_Simulation_Packet_Pointer++;                                                        // increment the packet pointer
        if (CPU_Simulation_Packet_Pointer > Number_of_Standard_CPU_Packets) CPU_Simulation_Packet_Pointer = 0;
        CPU_Packet_Received_Count++;
        Save_Packet_to_Log_File(Incoming_Packet_from_CPU);
        return true;
    }
    return false;
#else
    if (server.available()) {
        client = server.accept();
        console_print(true, "Client connected:");
    }
    while (client && client.connected() && client.available() > 0) {
#ifdef PRINT_CPU_INCOMING
        console_print(true, "\tIncoming Data from CPU:");
#endif
        uint8_t thisbyte = client.read();                               // read the character
        if (thisbyte == (uint8_t)SOH) {                                 // look for start character
#ifdef PRINT_CPU_INCOMING
            console.print(thisbyte, DEC); console.println(",");
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
                CPU_string_ptr = 0;                                     // zero the string pointer
                CPU_Packet_Received_Count++;                            // increment CPU packets received count
                Save_Packet_to_Log_File(Incoming_Packet_from_CPU);
                return true;
            }
            else {
                Incoming_Packet_from_CPU[CPU_string_ptr++] = thisbyte;       // Not a control so save it and increment string pointer
#ifdef PRINT_CPU_INCOMING
                console.print(thisbyte, DEC); console.println(",");
#endif
            }
        }
    }
    return false;
#endif 
}
bool Check_Altitude_Packet_Received(void) {
#ifdef SIMULATE_INCOMING_PACKETS
    if (millis() > ALT_Time_to_Send_Next_Packet) {
        ALT_Time_to_Send_Next_Packet = millis() + Time_Between_ALT_Packets;
        for (int i = 0; i < Standard_Packet_Length; i++) {
            Incoming_Packet_from_Altitude[i] = Standard_ALT_Packets[ALT_Simulation_Packet_Pointer][i];
        }
        ALT_Simulation_Packet_Pointer++;
        if (ALT_Simulation_Packet_Pointer > Number_of_Standard_ALT_Packets) ALT_Simulation_Packet_Pointer = 0;
        ALT_Packet_Received_Count++;
        Save_Packet_to_Log_File(Incoming_Packet_from_Altitude);
        return true;
    }
    return false;
#else
    while (Altitude_outptr != Altitude_inptr) {                                 // check Altitude serial buffer for data
        uint8_t thisbyte = Altitude_inbuffer[Altitude_outptr++];                // take a characters from the input buffer and increment pointer
        if (thisbyte == (uint8_t)SOH) {                                         // look for the SOH
            Incoming_Packet_from_Altitude[Altitude_string_ptr++] = (uint8_t)SOH; // store the SOH and increment the string pointer
        }
        else {
            if (thisbyte == (uint8_t)EOT) {                                         // characters was not an SOH check for ETX
                Incoming_Packet_from_Altitude[Altitude_string_ptr++] = (uint8_t)EOT; // save the EOT and increment the string pointer
                Altitude_string_ptr = 0;                                            // zero the string pointer
                ALT_Packet_Received_Count++;
                Save_Packet_to_Log_File(Incoming_Packet_from_Azimuth);
                return true;
            }
            else {
                Incoming_Packet_from_Altitude[Altitude_string_ptr++] = thisbyte; // Not a control so save it and increment string pointer
            }
        }
    }
    return false;
#endif
}
bool Check_Azimuth_Packet_Received(void) {
#ifdef SIMULATE_INCOMING_PACKETS
    if (millis() > AZI_Time_to_Send_Next_Packet) {
        AZI_Time_to_Send_Next_Packet = millis() + Time_Between_AZI_Packets;
        for (int i = 0; i < Standard_Packet_Length; i++) {
            Incoming_Packet_from_Azimuth[i] = Standard_AZI_Packets[AZI_Simulation_Packet_Pointer][i];
        }
        AZI_Simulation_Packet_Pointer++;
        if (AZI_Simulation_Packet_Pointer > Number_of_Standard_AZI_Packets) AZI_Simulation_Packet_Pointer = 0;
        AZI_Packet_Received_Count++;
        Save_Packet_to_Log_File(Incoming_Packet_from_Azimuth);
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
                Azimuth_string_ptr = 0;                                             // zero the string pointer
                AZI_Packet_Received_Count++;
                Save_Packet_to_Log_File(Incoming_Packet_from_Azimuth);
                return true;
            }
            else {
                Incoming_Packet_from_Azimuth[Azimuth_string_ptr++] = thisbyte;  // Not a control so save it and increment string pointer
            }
        }
    } // end of while Azimuth
#endif
    return false;
}
bool Check_Focuser_Packet_Received(void) {
#ifdef SIMULATE_INCOMING_PACKETS
    if (millis() > FOC_Time_to_Send_Next_Packet) {
        FOC_Time_to_Send_Next_Packet = millis() + Time_Between_FOC_Packets;
        for (int i = 0; i < Standard_Packet_Length; i++) {
            Incoming_Packet_from_Focuser[i] = Standard_FOC_Packets[FOC_Simulation_Packet_Pointer][i];
        }
        FOC_Simulation_Packet_Pointer++;
        if (FOC_Simulation_Packet_Pointer > Number_of_Standard_FOC_Packets) FOC_Simulation_Packet_Pointer = 0;
        FOC_Packet_Received_Count++;
        Save_Packet_to_Log_File(Incoming_Packet_from_Focuser);
        return true;
    }
    return false;
#else
    while (Focuser_outptr != Focuser_inptr) {                                   // check Focuser serial buffer for data
        uint8_t thisbyte = Focuser_inbuffer[Focuser_outptr++];                  // take a characters from the input buffer and increment pointer
        if (thisbyte == (char)SOH) {                                            // look for the SOH
            Incoming_Packet_from_Focuser[Focuser_string_ptr++] = (uint8_t)SOH;  // store the SOH and increment the string pointer
        }
        else {
            if (thisbyte == (char)EOT) {                                            // characters was not an STX check for ETX
                Incoming_Packet_from_Focuser[Focuser_string_ptr++] = (uint8_t)EOT;  // save the EOT and increment the string pointer
                Focuser_string_ptr = 0;                                             // zero the string pointer
                FOC_Packet_Received_Count++;
                Save_Packet_to_Log_File(Incoming_Packet_from_Focuser);
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
bool Process_CPU_Packet() {                                         // Process a packet from the CPU  
    console_print(true, F("Processing Packet Received from CPU"));
    //    console_print(false, F("Target:"));
    //    Serial.println(Incoming_Packet_from_CPU[TARGET], DEC);
    switch (Incoming_Packet_from_CPU[TARGET]) {                          // switch on the target
    case HUB: {
        console_print(true, F("Packet Destination HUB"));
        Save_Packet_to_Log_File(Incoming_Packet_from_CPU);
        unsigned long size_of_packet = 0;
        for (unsigned int i = 0; i < sizeof(Incoming_Packet_from_CPU); i++) {
            //            Serial.print("(");
            //            Serial.print(Incoming_Packet_from_CPU[i], DEC);
            //            Serial.print("),");
            size_of_packet++;
            if (Incoming_Packet_from_CPU[i] == EOT) {
                //                Serial.println();
                break;
            }
        }
        if (!parsePacket(Incoming_Packet_from_CPU, size_of_packet)) {
            console_print(true, F("Corrupt Packet from CPU, target was HUB, but unable to decode"));
            Print_FreeMemory();
            return false;
        }
        switch (Incoming_Packet_from_CPU[COMMAND]) {                      // switch on the Command Number
        case (Reset): {
            console_print(true, F("Reset Command Received from CPU"));
            if (Incoming_Packet_from_CPU[TYPE] == CMD) {
                Wait_for_Reset_Switch(F("Reset Requested by CPU"));
            }
            else {
                console_print(true, F("Illegal Reset Type Received"));
            }
            break;
        }
        case (Environment): {
            console_print(true, F("Environment Requested by CPU"));
            if (Incoming_Packet_from_CPU[TYPE] == (uint8_t)GET) {
                console_print(true, F("Environment Get Received from CPU"));
                Send_Reply_to_CPU((int)Environment);
            }
            else if (Incoming_Packet_from_CPU[TYPE] == (uint8_t)SET) {
                if (getFieldAsBool(1)) {
                    bitWrite(Device_Status, Lights_Status, 1);
                    console_print(true, F("Environment Set Turn Lights ON"));
                }
                else {
                    bitWrite(Device_Status, Lights_Status, 0);
                    console_print(true, F("Environment Set Turn Lights OFF"));
                }
            }
            else {
                console_print(true, F("Illegal Environment Type Received"));
            }
            break;
        }
        case (FirmwareVersion): {
            console_print(true, F("Firmware Version Get Received from CPU"));
            if (Incoming_Packet_from_CPU[TYPE] == GET) {
                Send_Reply_to_CPU((int)FirmwareVersion);
            }
            else {
                console_print(true, F("Illegal Firmware Version Type Received"));
            }
            break;
        }
        case (Statistics): {
            console_print(true, F("Statistics Get Received from CPU"));
            if (Incoming_Packet_from_CPU[TYPE] == GET) {
                Send_Reply_to_CPU((int)Statistics);
            }
            else {
                console_print(true, F("Illegal Statistics Type Received"));
            }
            break;
        }
        case (Retrieve): {
            int character_count = 0;
            char field[25];
            int datafieldNo = 0;
            char datatemp;
            console_print(true, F("Retrieve Get Received from CPU"));
            if (Incoming_Packet_from_CPU[TYPE] == GET) {
                LogFile = SD.open("log.csv", FILE_READ);                 // open the SD file
                console_print(true, F("\tProcessing Log.csv"));
                if (!LogFile) {                                                                    // oops - file not available!
                    Wait_for_Reset_Switch(F("Error re-opening Log File"));                                                             // Reset will restart the processor so no return
                }
                else {
                    while (LogFile.available()) {                           // whilst there  data are the log file
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
                            wdt_reset();                                                            // keep watch dog timer active
                            delay(1000);                                    // delay 1 second before sending next record
                        }
                    }
                    LogFile.close();
                    LogFile.flush();
                    SD.remove("/Log.csv");                                  // delete the log file
                }
            }
            else {
                console_print(true, F("Illegal Retrieve Type Received"));
            }
            break;
        }
        case (DateTime): {
            if (Incoming_Packet_from_CPU[TYPE] == GET) {
                console_print(true, F("Date and Time Get Received from CPU"));
                Send_Reply_to_CPU(DateTime);
            }
            else {
                console_print(true, F("Illegal Date and Time Type Received"));
            }
            break;
        }
        case (DeleteLog): {
            if (Incoming_Packet_from_CPU[TYPE] == CMD) {
                console_print(true, F("Delete Log File Received from CPU"));
                SD.remove("/Log.csv");                                  // delete the log file
            }
            else {
                console_print(true, F("Illegal Delete Log Type Received"));
            }
            break;
        }
        default: {
            console_print(true, F("Unknown Command Received from CPU"));
            break;
        }
        }                                                  // end of switch on command number
        break;
    }
    case ALT: {                                                     // send the packet to the ALT
        console_print(true, F("Packet Destination Altitude"));
        Transmit_Packet_to_Target((char)ALT, Incoming_Packet_from_CPU);
        break;
    }
    case AZI: {
        console_print(true, F("Packet Destination Azimuth"));
        Transmit_Packet_to_Target((char)AZI, Incoming_Packet_from_CPU);
        break;
    }
    case BTH: {
        console_print(true, F("Packet Destination Both Motors"));
        Transmit_Packet_to_Target((char)ALT, Incoming_Packet_from_CPU);
        Transmit_Packet_to_Target((char)AZI, Incoming_Packet_from_CPU);
        break;
    }
    case FOC: {
        console_print(true, F("Packet Destination Focuser"));
        Transmit_Packet_to_Target((char)FOC, Incoming_Packet_from_CPU);
        break;
    }
    case ALL: {
        console_print(true, F("Packet Destination All Devices"));
        Transmit_Packet_to_Target((char)ALT, Incoming_Packet_from_CPU);
        Transmit_Packet_to_Target((char)AZI, Incoming_Packet_from_CPU);
        Transmit_Packet_to_Target((char)FOC, Incoming_Packet_from_CPU);
    }
    default: {
        console_print(true, F("Unspecified Target Device"));
        break;// end of switch target
    }
    }
    return true;
}
void Send_Reply_to_CPU(int command) {
    char temp[20];
    console_print(false, F("Sending Reply to CPU: "));
    client.print(SOH);                            // Byte 0   SOH
    Serial.print(F("("));
    Serial.print(SOH, DEC);

    Serial.print(F("),("));
    client.print(CPU);                            // Byte 1   Target
    Serial.print(CPU, DEC);

    Serial.print(F("),("));
    client.print(HUB);                            // Byte 2   Source
    Serial.print(HUB, DEC);

    Serial.print(F("),("));
    client.print(REP);                            // Byte 3   Packet Type
    Serial.print(REP, DEC);

    Serial.print(F("),("));
    client.print(command);                        // Byte 4   Command
    Serial.print(command, DEC);

    Serial.print(F("),("));
    client.print(STX);                            // Byte 5   STX
    Serial.print(STX, DEC);

    Serial.print(F("),("));
    sprintf(temp, "%d", Device_Status);
    client.print(temp);                           // Byte 6 & 7
    Serial.print(temp);

    Serial.print(F("),("));
    client.print(FLD);                            // Byte 8
    Serial.print(FLD, DEC);
    switch (command) {
    case Environment: {
        Serial.print(F("),[Data]("));
        dtostrf(Ambient_Temperature, 4, 2, temp);
        client.print(temp);
        Serial.print(temp);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD, DEC);

        Serial.print(F("),[Data]("));
        dtostrf(Ambient_Humidity, 4, 2, temp);
        client.print(temp);
        Serial.print(temp);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD, DEC);

        Serial.print(F("),[Data]("));
        dtostrf(Motor_Voltage, 4, 2, temp);
        client.print(temp);
        Serial.print(temp);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD, DEC);

        Serial.print(F("),[Data]("));
        Free_Memory = freeMemory();
        itoa(Free_Memory, temp, 10);
        client.print(temp);
        Serial.print(temp);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD, DEC);
        break;
    }
    case FirmwareVersion: {
        Serial.print(F("),[Data]("));
        dtostrf(Firmware_Version, 4, 2, temp);
        client.print(temp);
        Serial.print(temp);

        Serial.print(F("),[FLD]("));
        client.print(FLD);
        Serial.print(FLD, DEC);

        Serial.print(F("),[Data]("));
        dtostrf(Protocol_Version, 4, 2, temp);
        client.print(temp);
        Serial.print(temp);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD, DEC);

        break;
    }
    case Statistics: {
        Serial.print(F("),[Data]("));
        sprintf(temp, "%lu", CPU_Packet_Received_Count);
        client.print(temp);
        Serial.print(temp);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD, DEC);

        Serial.print(F("),[Data]("));
        sprintf(temp, "%lu", CPU_Packet_Transmitted_Count);
        client.print(temp);
        Serial.print(temp);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD);

        Serial.print(F("),[Data]("));
        sprintf(temp, "%lu", ALT_Packet_Received_Count);
        client.print(temp);
        Serial.print(temp);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD);

        Serial.print(F("),[Data]("));
        sprintf(temp, "%lu", ALT_Packet_Transmitted_Count);
        client.print(temp);
        Serial.print(temp);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD, DEC);

        Serial.print(F("),[Data]("));
        sprintf(temp, "%lu", AZI_Packet_Received_Count);
        client.print(temp);
        Serial.print(temp);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD, DEC);

        Serial.print(F("),[Data]("));
        sprintf(temp, "%lu", AZI_Packet_Transmitted_Count);
        client.print(temp);
        Serial.print(temp);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD, DEC);

        Serial.print(F("),[Data]("));
        sprintf(temp, "%lu", FOC_Packet_Received_Count);
        client.print(temp);
        Serial.print(temp);

        Serial.print(F("),("));
        client.println(FLD);
        Serial.print(FLD, DEC);

        Serial.print(F("),[Data]("));
        sprintf(temp, "%lu", FOC_Packet_Transmitted_Count);
        client.print(temp);
        Serial.print(temp);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD, DEC);
        break;
    }
    case Retrieve: {
        Serial.print(F("),[Data]("));
        client.print(Retrieved_Timestamp);
        Serial.print(Retrieved_Timestamp);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD, DEC);

        Serial.print(F("),[Data]("));
        client.print(Retrieved_Message);
        client.print(Retrieved_Message);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD, DEC);
        break;
    }
    case DateTime: {
        Serial.print(F("),[Data]("));
        client.print(Current_Date);
        Serial.print(Current_Date);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD, DEC);

        Serial.print(F("),[Data]("));
        client.print(Current_Time);
        Serial.print(Current_Time);

        Serial.print(F("),("));
        client.print(FLD);
        Serial.print(FLD, DEC);
        break;
    }
    }
    Serial.print(F("),("));
    client.print(ETX);
    Serial.print(ETX, DEC);

    Serial.print(F("),("));
    client.print(EOT);
    Serial.print(EOT, DEC);

    Serial.println(F(")"));
    CPU_Packet_Transmitted_Count++;
}
void Transmit_Packet_to_Target(char target, char* data) {
    switch (target) {
    case CPU: {
        if (bitRead(Device_Status, CPU_Status)) {
            for (unsigned int i = 0; i < sizeof(data); i++) {
                while (!server.availableForWrite()) {
                    console_print(true, F("Waiting for server available"));
                    delay(10);
                }
                server.write(data[i]);
            }
            console_print(true, F("Packet sent to CPU"));
            CPU_Packet_Transmitted_Count++;
        }
        break;
    }
    case ALT: {
        for (unsigned int i = 0; i < sizeof(data); i++) {
            while (!Altitude_Port.availableForWrite()) {
                delay(10);
            }
            Altitude_Port.write(data[i]);
        }
        console_print(true, F("Packet sent to Altitude"));
        ALT_Packet_Transmitted_Count++;
        break;
    }
    case AZI: {
        for (unsigned int i = 0; i < sizeof(data); i++) {
            while (!Azimuth_Port.availableForWrite()) {
                delay(10);
            }
            Azimuth_Port.write(data[i]);
        }
        console_print(true, F("Packet sent to Azimuth"));
        AZI_Packet_Transmitted_Count++;
        break;
    }
    case BTH: {
        for (unsigned int i = 0; i < sizeof(data); i++) {
            while (!Altitude_Port.availableForWrite()) {
                delay(10);
            }
            Altitude_Port.write(data[i]);
            while (!Azimuth_Port.availableForWrite()) {
                delay(10);
            }
            Azimuth_Port.write(data[i]);
        }
        console_print(true, F("Packet sent to Altitude and Azimuth"));
        ALT_Packet_Transmitted_Count++;
        AZI_Packet_Transmitted_Count++;
        break;
    }
    case FOC: {
        for (unsigned int i = 0; i < sizeof(data); i++) {
            while (!Focuser_Port.availableForWrite()) {
                delay(10);
            }
            Focuser_Port.write(data[i]);
        }
        console_print(true, F("Packet sent to Focuser"));
        FOC_Packet_Received_Count++;
        break;
    }
    case ALL: {
        for (unsigned int i = 0; i < sizeof(data); i++) {
            while (!Altitude_Port.availableForWrite()) {
                delay(10);
            }
            Altitude_Port.write(data[i]);
            while (!Azimuth_Port.availableForWrite()) {
                delay(10);
            }
            Azimuth_Port.write(data[i]);
            while (!Focuser_Port.availableForWrite()) {
                delay(10);
            }
            Focuser_Port.write(data[i]);
            ALT_Packet_Transmitted_Count++;
            AZI_Packet_Transmitted_Count++;
            FOC_Packet_Transmitted_Count++;
            console_print(true, F("Packet sent to All Devices"));
        }
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
        Wait_for_Reset_Switch(F("SD Drive Begin failed"));
    }
    if (SD.exists("log.csv")) {
        console_print(true, F("\tLog File exists, so delete it"));
        SD.remove("log.csv");
    }
    LogFile = SD.open("log.csv", FILE_WRITE);
    if (!LogFile) {
        Wait_for_Reset_Switch(F("Error Creating Logfile"));
    }
    else {
        console_print(true, F("\tLog File Created Successfully"));
        LogFile.print(__DATE__);                                        // write the first record to file
        LogFile.print(" ");
        LogFile.print(__TIME__);
        LogFile.println(",File Started");
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
        bitWrite(Device_Status, Temperature_Status, 0);
    }
    else {
        Ambient_Temperature = event.temperature;
        bitWrite(Device_Status, Temperature_Status, 1);
    }
    Ambient_Sensor.humidity().getEvent(&event);			// Get humidity event and print its value.
    if (isnan(event.relative_humidity)) {
        Ambient_Humidity = 0;
        bitWrite(Device_Status, Humidity_Status, 0);
    }
    else {
        Ambient_Humidity = event.relative_humidity;
        bitWrite(Device_Status, Humidity_Status, 1);
    }
    Motor_Voltage = digitalRead(Voltage_pin);
    if (Ambient_Temperature > Fan_Switch_On_Temperature) {          // Turn the fan on if necessary
        bitWrite(Device_Status, Fan_Status, 1);
        digitalWrite(Fan_pin, ON);
    }
    else if (Ambient_Temperature < Fan_Switch_Off_Temperature) {     // Turn the fan off if necessary
        digitalWrite(Fan_pin, OFF);
        bitWrite(Device_Status, Fan_Status, 0);
    }
    Free_Memory = (freeMemory());
}
void Led_Control(uint8_t led, bool state) {
    switch (led) {
    case (RUN_Active_led_pin): {
        bitWrite(Device_Status, RUN_Status, state);
        break;
    }
    case (Shield_led_pin): {
        bitWrite(Device_Status, Ethernet_Status, state);
        break;
    }
    }
}
void Check_Lights() {
    if (bitRead(Device_Status, Lights_Status)) {                                // are the lights enabled
        if (bitRead(Device_Status, RUN_Status)) {                               // Run_Active led
            if (millis() >= RUN_Active_Led_Start_Time + Led_On_Time) {
                RUN_Active_Led_Start_Time = millis();
                digitalWrite(RUN_Active_led_pin, !digitalRead(RUN_Active_led_pin));  // toggle the RUN led
            }
            else {
                digitalWrite(RUN_Active_led_pin, OFF);                          // turn the RUN led off
            }
        }
        if (bitRead(Device_Status, Ethernet_Status)) {                          // Ethernet Active
            if (millis() >= Shield_Led_Start_Time + Led_On_Time) {
                Shield_Led_Start_Time = millis();
                digitalWrite(Shield_led_pin, !digitalRead(Shield_led_pin));     // toggle the Shield led
            }
            else {
                digitalWrite(Shield_led_pin, OFF);                              // turn the Shield led off
            }
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
void Wait_for_Reset_Switch(const __FlashStringHelper* message) {
    int trys = 0;
    console_print(false, F("Failure: "));
    Serial.print(message);
    console_print(true, F(", Press Reset"));
    do {
        delay(1000);
    } while (trys < 60);
    wdt_enable(WDTO_15MS);  // Enable the watchdog timer with a timeout of 15 ms
    while (true) {}         // Infinite loop to allow the watchdog to reset the microcontroller
}
bool Get_Time_and_Date(int timeIPNumber) {
    if (!UDP.begin(localPort)) {
        console_print(true, F("Opening UDP Port Failed"));
        return false;
    }
    console_print(true, F("UDP Port Opened"));
    console_print(false, F("Sending NTP Packet to: "));
    Serial.println(timeServers[timeIPNumber]);
    sendNTPpacket(timeServers[timeIPNumber]);
    delay(1000);
    if (UDP.parsePacket()) {
        console_print(true, F("Parsing UDP Packet"));
        uint32_t epoch = 0;
        UDP.readBytes(NTP_Packet_Buffer, NTP_PACKET_SIZE);  // read the packet into the buffer
        NTP_Seconds.Words[3] = NTP_Packet_Buffer[40];
        NTP_Seconds.Words[2] = NTP_Packet_Buffer[41];
        NTP_Seconds.Words[1] = NTP_Packet_Buffer[42];
        NTP_Seconds.Words[0] = NTP_Packet_Buffer[43];
        epoch = NTP_Seconds.Seconds - 2208988800; // +timeZoneOffset;
        setTime(epoch);
        ntpLastUpdate = minute();
        return true;
    }
    return false;
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
    if (!bitRead(Device_Status, Date_Status)) {                         // try getting the date and time again
        int trys = 0;
        while (!Get_Time_and_Date(trys) && trys < 10) {
            trys++;
        }
        if (trys >= Number_of_TimeServers) {
            console_print(true, F("ntp server update failed"));
            bitWrite(Device_Status, Date_Status, 0);
        }
        else {
            console_print(true, F("ntp server update success"));
            prevDisplay = minute();
            Format_Date_and_Time(true);
            Clock_Display();
            bitWrite(Device_Status, Date_Status, 1);
        }
    }
    else {
        if (minute() != prevDisplay) {                 // Display the time if it has changed by more than a second.
            prevDisplay = minute();
            Clock_Display();
        }
    }
}
void Clock_Display() {                                   // Clock display of the time and date (Basic)
    int weekday = calcDayOfWeek(year(), month(), day());
    Format_Date_and_Time(true);
    Serial.print(millis(), DEC); Serial.print("\t");
    Serial.print(Weekdays[weekday]);
    Serial.print(", ");
    printDigits(hour());
    Serial.print(":");
    printDigits(minute());
    Serial.print(":");
    printDigits(second());
    Serial.print(" ");
    printDigits(day());
    Serial.print("/");
    printDigits(month());
    Serial.print("/");
    Serial.print(year());
    Serial.println();
}
void printDigits(int digits) {
    if (digits < 10)
        Serial.print('0');
    Serial.print(digits);
}
byte calcDayOfWeek(int y, byte m, byte d) {
    // Old mental arithmetic method for calculating day of week
    // adapted for Arduino, for years 2000~2099
    // returns 1 for Sunday, 2 for Monday, etc., up to 7 for Saturday
    // for "bad" dates (like Feb. 30), it returns 0
    // Note: input year (y) should be a number from 0~99
    if (y > 2099) return 0; // we don't accept years after 2099
    // we take care of bad months later
    if (d < 1) return 0; // because there is no day 0
    byte w = 6; // this is a magic number (y2k fix for this method)
    // one ordinary year is 52 weeks + 1 day left over
    // a leap year has one more day than that
    // we add in these "leftover" days
    w += (y + (y >> 2));
    // correction for Jan. and Feb. of leap year
    if (((y & 3) == 0) && (m <= 2)) w--;
    // add in "magic number" for month
    switch (m) {
    case 1:  if (d > 31) return 0; w += 1; break;
    case 2:  if (d > ((y & 3) ? 28 : 29)) return 0; w += 4; break;
    case 3:  if (d > 31) return 0; w += 4; break;
    case 4:  if (d > 30) return 0; break;
    case 5:  if (d > 31) return 0; w += 2; break;
    case 6:  if (d > 30) return 0; w += 5; break;
    case 7:  if (d > 31) return 0; break;
    case 8:  if (d > 31) return 0; w += 3; break;
    case 9:  if (d > 30) return 0; w += 6; break;
    case 10: if (d > 31) return 0; w += 1; break;
    case 11: if (d > 30) return 0; w += 4; break;
    case 12: if (d > 31) return 0; w += 6; break;
    default: return 0;
    }
    // then add day of month
    w += d;
    // there are only 7 days in a week, so we "cast out" sevens
    while (w > 7) w = (w >> 3) + (w & 7);
    return w;
}
void Print_FreeMemory() {
    console_print(false, F("Free Memory; "));
    Serial.println(freeMemory());
}
/*
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
*/