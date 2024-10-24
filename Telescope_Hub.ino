/*
  Arduino Ethernet Telescope Hub
        Interfaces Steve and Jamie Gould's Telescope to a ASCOM compliant software driver
        Declination = Altitude = north/south = up down
        Right Ascension = Azimuth  = east/west = left right
        Communications to the motor controllers is made through this HUB.
    Functionality:
    1. Receive packets of information from the CPU (Windows PC)
        a)  If the target is the Hub execute the contained command
        b)  Otherwise forward the received packet to the indicated target
    2. Receive packets of information from the attached devices
        a)  Forward the received packets to the indicated target
*/
/* Version Control --------------------------------------------------------------------------------
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
01/10/2024  2.0     Development Restarted
24/10/2024  2.1     Functionality reduced to support only Hub, Altitude, Azimuth and Focuser communications
*/
constexpr double Firmware_Version = (double)2.1;
// Inclusions -------------------------------------------------------------------------------------
#include <avr/wdt.h>
#include <Bounce2.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Ethernet2.h>
#include <util.h>
#include <EthernetUdp2.h>
#include <EthernetServer.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>
#include <Hardwareserial.h>
#define SIMULATE_CPU_INCOMING_PACKETS           // Simulate the receipt of packets from the CPU
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
uint8_t mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);
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
constexpr uint8_t RUN_Active_led_pin = 3;  // RUN led
constexpr uint8_t Ambient_Sensor_pin = 4;	// ambient temperature and humidity pin
constexpr uint8_t Fan_pin = 5;             // fan (relay) pin
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
// Instantiations ---------------------------------------------------------------------------------
EthernetServer CPU_Port(80);                                // Create a server listening on port 80.
HardwareSerial Altitude_Port = Serial1;                     // Altitude Port
HardwareSerial Azimuth_Port = Serial2;                      // Azimuth Port
HardwareSerial Focuser_Port = Serial3;                      // Focuser Port
// Communications Variables -----------------------------------------------------------------------
char Incoming_CPU_Packet[0xFF];
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
uint8_t CPU_inptr;					    // must be 8 bit uint8_t so that it overflows at 256
uint8_t CPU_outptr;				        // must be 8 bit uint8_t so that it overflows at 256
uint8_t CPU_inbuffer[0xff];
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
uint16_t Device_status = 0;
enum { OFF = 0, ON = 1 };
unsigned long RUN_Active_Led_Start_Time = 0;
// Instantiations ---------------------------------------------------------------------------------
DHT_Unified Ambient_Sensor(Ambient_Sensor_pin, DHT22);
Bounce Reset_button = Bounce();
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
    console_print("Setup Commenced");
    pinMode(RUN_Active_led_pin, OUTPUT);
    Led_Control(RUN_Active_led_pin, ON);            // turn the run led on
    console_print("Starting Ethernet");
    CPU_Port.begin();                               // Start Ethernet
    console_print("Ethernet Started");
    pinMode(Voltage_pin, INPUT);
    pinMode(Fan_pin, OUTPUT);                                       // specify the fan pin as an output
    console_print("Temperature and Humidity Sensor Set Up");
    sensor_t sensor;
    Ambient_Sensor.temperature().getSensor(&sensor);
    console_print("Temperature Sensor: " + String(sensor.name));
    console_print("Driver Ver:         " + String(sensor.version));
    console_print("Unique ID:          " + String(sensor.sensor_id));
    console_print("Max Value:          " + String(sensor.max_value) + " *C");
    console_print("Min Value:          " + String(sensor.min_value) + " *C");
    console_print("Resolution:         " + String(sensor.resolution) + " *C");
    Ambient_Sensor.humidity().getSensor(&sensor);
    console_print("Humidity Sensor:    " + String(sensor.name));
    console_print("Driver Ver:         " + String(sensor.version));
    console_print("Unique ID:          " + String(sensor.sensor_id));
    console_print("Max Value:          " + String(sensor.max_value) + "%");
    console_print("Min Value:          " + String(sensor.min_value) + "%");
    console_print("Resolution:         " + String(sensor.resolution) + "%");
    console_print("Sensor Setup Complete");
    console_print("Setup Serial Ports");
    Altitude_Port.begin(Altitude_baud, SERIAL_8N2);					// initialise the Altitude serial port    
    Altitude_Port.flush();                                          // clear the Altitude serial buffer
    console_print("Altitude serial port started");
    Azimuth_Port.begin(Azimuth_baud, SERIAL_8N2);					// initialise the Azimuth serial port
    Azimuth_Port.flush();											// clear the Azimuth serial buffer
    console_print("Azimuth serial port started");
    Focuser_Port.begin(Focuser_baud, SERIAL_8N2);					// initialise the Focuser serial port
    Focuser_Port.flush();											// clear the Focuser serial buffer
    console_print("Focuser serial port started");
    console_print("Enabling WatchDog Timer");
    wdt_enable(WDTO_4S);                                    // 4 second timeout
    console_print("Setup Complete");
#ifdef SIMULATE_CPU_INCOMING_PACKETS
    console_print("Simulating CPU Incoming Packets");
#endif
#ifdef SIMULATE_ALT_INCOMING_PACKETS
    console_print("Simulating ALT Incoming Packets");
#endif
#ifdef SIMULATE_AZI_INCOMING_PACKETS
    console_print("Simulating AZI Incoming Packets");
#endif
#ifdef SIMULATE_FOC_INCOMING_PACKETS
    console_print("Simulating FOC Incoming Packets");
#endif
    Led_Control(RUN_Active_led_pin, ON);
} // end setup
void(*resetFunc) (void) = 0;                                // reset function
// Main -------------------------------------------------------------------------------------------
void loop() {
    wdt_reset();                                                            // keep watch dog timer active
    Led_Control(RUN_Active_led_pin, ON);
    if (Check_CPU_Packet_Received()) {
        if (!Process_CPU_Packet()) {
            console_print("Bad Packet Received from CPU");
        }
    }
    if (Check_Altitude_Packet_Received()) Copy_Packet_to_CPU(ALT);
    if (Check_Azimuth_Packet_Received()) Copy_Packet_to_CPU(AZI);
    if (Check_Focuser_Packet_Received()) Copy_Packet_to_CPU(FOC);
    Check_Lights();
    Update_Environmental_Sensors();
}// end of main loop ------------------------------------------------------------------------------
void console_print(String message) {
    console.print(millis(), DEC); console.print("\t"); console.println(message);
}
// Received Character Handling --------------------------------------------------------------------
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
bool Check_CPU_Packet_Received(void) {
#ifdef SIMULATE_CPU_INCOMING_PACKETS
    if (millis() > CPU_Time_to_Send_Next_Packet) {
        CPU_Time_to_Send_Next_Packet = millis() + Time_Between_CPU_Packets;
        strcpy(Incoming_CPU_Packet, Standard_CPU_Packets[CPU_Simulation_Packet_Pointer]);
        CPU_packet_length = strlen(Incoming_CPU_Packet);
        CPU_Simulation_Packet_Pointer++;
        if (CPU_Simulation_Packet_Pointer > Number_of_Standard_CPU_Packets) CPU_Simulation_Packet_Pointer = 0;
        bitWrite(Device_status, 1, 1);                          // set CPU Active bit true
        CPU_Packet_Received_Count++;
        return true;
    }
    return false;
#else
    Maintain_Internet();
    EthernetClient client = CPU_Port.available();               // Listen for incoming client requests.
    if (client) {
        if (client.available()) {
            uint8_t thisbyte = client.read();
            if (thisbyte == (uint8_t)SOH) {
                CPU_string_ptr = 0;
                Incoming_CPU_Packet[CPU_string_ptr++] = (uint8_t)SOH;   // store the SOH and increment the string pointer
            }
            else {
                if (thisbyte == (uint8_t)EOT) {                                 // characters was not an SOH check for ETX
                    Incoming_CPU_Packet[CPU_string_ptr++] = (uint8_t)EOT;  // save the EOT and increment the string pointer
                    CPU_packet_length = CPU_string_ptr - 1;
                    CPU_string_ptr = 0;                                         // zero the string pointer
                    bitWrite(Device_status, 1, 1);
                    CPU_Packet_Received_Count++;
                    return true;
                }
                else {
                    Incoming_CPU_Packet[CPU_string_ptr++] = thisbyte; // Not a control so save it and increment string pointer
                }
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
        console_print("Memory allocation failed!");
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
bool Process_CPU_Packet() {                                    // Process a packet from the CPU  
    console_print("Packet Received from CPU");
    switch (Incoming_CPU_Packet[1]) {
    case HUB: {
        if (!Decode_Fields(Incoming_CPU_Packet, Packet_Field)) {
            console_print("Corrupt Packet from CPU, target was HUB");
            return false;
        }
        else {
            console_print("Packet Received from CPU, target was HUB");
        }
        switch (Obtain_Int_Parameter(0)) {                              // switch on the Command Number
        case (Reset): {
#ifdef SIMULATION
            console_print("Restart Requested by CPU");
#endif
            wdt_enable(WDTO_15MS);  // Enable the watchdog timer with a timeout of 15 ms
            while (true) {}         // Infinite loop to allow the watchdog to reset the microcontroller
            break;
        }
        case (Environment): {
            if (Incoming_CPU_Packet[4] == (uint8_t)GET) {
#ifdef SIMULATION
                console_print("Environment Get Received from CPU");
#endif
                Send_Reply_to_CPU((int)Environment);
            }
            else if (Incoming_CPU_Packet[4] == (uint8_t)SET) {
#ifdef SIMULATION
                console_print("Environment Set Lights Received from CPU");
#endif
                if (Obtain_Bool_Parameter(1)) {
                    bitWrite(Device_status, 2, 1);
                }
                else {
                    bitWrite(Device_status, 2, 0);
                }
            }
            break;
        }
        case (FirmwareVersion): {
#ifdef SIMULATION
            console_print("Firmware Version Get Received from HUB");
#endif
            Send_Reply_to_CPU((int)Firmware_Version);
            break;
        }
        case (Statistics): {
#ifdef SIMULATION
            console_print("Statistics Get Received from HUB");
#endif
            Send_Reply_to_CPU((int)Statistics);
            break;
        }
        default: {
#ifdef SIMULATION
            console_print("Unknown Command Received from HUB");
#endif
            Send_Reply_to_CPU((int)Obtain_Int_Parameter(0));
            break;
        }
        }                                                  // end of switch on command number
    }
    case ALT: {
        console_print("Packet Destination Altitude");
        Transmit_Packet_to_Target((char)ALT, Incoming_Packet_from_Altitude, Altitude_packet_length);
        break;
    }
    case AZI: {
        console_print("Packet Destination Azimuth");
        Transmit_Packet_to_Target((char)AZI, Incoming_Packet_from_Azimuth, Azimuth_packet_length);
        break;
    }
    case BTH: {
        console_print("Packet Destination Both Motors");
        Transmit_Packet_to_Target((char)ALT, Incoming_Packet_from_Altitude, Altitude_packet_length);
        Transmit_Packet_to_Target((char)AZI, Incoming_Packet_from_Azimuth, Azimuth_packet_length);
        break;
    }
    case FOC: {
        console_print("Packet Destination Focuser");
        Transmit_Packet_to_Target((char)FOC, Incoming_Packet_from_Focuser, Focuser_packet_length);
        break;
    }
    case ALL: {
        console_print("Packet Destination All Devices");
        Transmit_Packet_to_Target((char)ALT, Incoming_Packet_from_Altitude, Altitude_packet_length);
        Transmit_Packet_to_Target((char)AZI, Incoming_Packet_from_Azimuth, Azimuth_packet_length);
        Transmit_Packet_to_Target((char)FOC, Incoming_Packet_from_Focuser, Focuser_packet_length);
    }                                                   // end of switch target
    }
    return true;
}
void Copy_Packet_to_CPU(uint8_t target) {    // Send message received from ALT,AZI,FOC
    switch (target) {
    case ALT: {
#ifdef PRINT_CONSOLE_MESSAGES
        console_print("Packet Received from Altitude");
#endif
        Transmit_Packet_to_Target(CPU, Incoming_Packet_from_Altitude, Altitude_packet_length);
        break;
    }
    case AZI: {
#ifdef PRINT_CONSOLE_MESSAGES
        console_print("Packet Received from Azimuth");
#endif
        Transmit_Packet_to_Target(CPU, Incoming_Packet_from_Azimuth, Azimuth_packet_length);
        break;
    }
    case FOC: {
#ifdef PRINT_CONSOLE_MESSAGES
        console_print("Packet Received from Focuser");
#endif
        Transmit_Packet_to_Target(CPU, Incoming_Packet_from_Focuser, Focuser_packet_length);
        break;
    }
    }
}
void Send_Reply_to_CPU(int command) {
    char temp[20];
    Print_Byte_to_Port(SOH);                        // Byte 0   SOH
    Print_Byte_to_Port(CPU);                        // Byte 1   Target
    Print_Byte_to_Port(HUB);                        // Byte 2   Source
    Print_Byte_to_Port(REP);                        // Byte 3   Packet Type
    Print_Byte_to_Port(command);                    // Byte 4   Command
    Print_Byte_to_Port(STX);                        // Byte 5   STX
    sprintf(temp, "%d", Device_status);
    Print_String_to_CPU_Port(temp, strlen(temp));   // Byte 6 & 7
    Print_Byte_to_Port(FLD);                        // Byte 8
    if (command == Environment) {
        sprintf(temp, "%.2f", Ambient_Temperature);
        Print_String_to_CPU_Port(temp, strlen(temp));
        Print_Byte_to_Port(FLD);
        sprintf(temp, "%.2f", Ambient_Humidity);
        Print_String_to_CPU_Port(temp, strlen(temp));
        Print_Byte_to_Port(FLD);
        sprintf(temp, "%.2f", Motor_Voltage);
        Print_String_to_CPU_Port(temp, strlen(temp));
        Print_Byte_to_Port(FLD);
    }
    else if (command == FirmwareVersion) {
        sprintf(temp, "%.2f", Firmware_Version);
        Print_String_to_CPU_Port(temp, strlen(temp));
        Print_Byte_to_Port(FLD);
    }
    else if (command == Statistics) {
        sprintf(temp, "%lu", CPU_Packet_Received_Count);
        Print_String_to_CPU_Port(temp, strlen(temp));
        Print_Byte_to_Port(FLD);
        sprintf(temp, "%lu", CPU_Packet_Transmitted_Count);
        Print_String_to_CPU_Port(temp, strlen(temp));
        Print_Byte_to_Port(FLD);
        sprintf(temp, "%lu", ALT_Packet_Received_Count);
        Print_String_to_CPU_Port(temp, strlen(temp));
        Print_Byte_to_Port(FLD);
        sprintf(temp, "%lu", ALT_Packet_Transmitted_Count);
        Print_String_to_CPU_Port(temp, strlen(temp));
        Print_Byte_to_Port(FLD);
        sprintf(temp, "%lu", AZI_Packet_Received_Count);
        Print_String_to_CPU_Port(temp, strlen(temp));
        Print_Byte_to_Port(FLD);
        sprintf(temp, "%lu", AZI_Packet_Transmitted_Count);
        Print_String_to_CPU_Port(temp, strlen(temp));
        Print_Byte_to_Port(FLD);
        sprintf(temp, "%lu", FOC_Packet_Received_Count);
        Print_String_to_CPU_Port(temp, strlen(temp));
        Print_Byte_to_Port(FLD);
        sprintf(temp, "%lu", FOC_Packet_Transmitted_Count);
        Print_String_to_CPU_Port(temp, strlen(temp));
        Print_Byte_to_Port(FLD);
    }
    Print_Byte_to_Port(ETX);
    Print_Byte_to_Port(EOT);
    CPU_Packet_Transmitted_Count++;
}
void Print_Byte_to_Port(uint8_t data) {
    while (!CPU_Port.availableForWrite()) {
        delay(10);
    }
    CPU_Port.print(data);
}
void Print_String_to_CPU_Port(char* data, char size) {
    for (int i = 0; i < size; i++) {
        while (!CPU_Port.availableForWrite()) {
            delay(10);
        }
        CPU_Port.print(data[i]);
    }
}
void Transmit_Packet_to_Target(char target, char* data, char size) {
    switch (target) {
    case CPU: {
        for (int i = 0; i < size; i++) {
            while (!CPU_Port.availableForWrite()) {
                delay(10);
            }
            CPU_Port.write(data[i]);
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
// -------------------------------------------------------------------------------------------------
void Console_Print(String message) {
    console.print(millis(), DEC);
    console.print("\t");
    console.println(message);
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
        bitWrite(Device_status, 8, 1);
        digitalWrite(Fan_pin, ON);
    }
    else if (Ambient_Temperature < Fan_Switch_Off_Temperature) {     // Turn the fan off if necessary
        digitalWrite(Fan_pin, OFF);
        bitWrite(Device_status, 8, 0);
    }
}
void Led_Control(uint8_t led, bool state) {
    switch (led) {
    case (RUN_Active_led_pin): {
        bitWrite(Device_status, 0, 1);
        break;
    }
    }
}
void Check_Lights() {
    if (bitRead(Device_status, 0)) {                                            // are the lights enabled
        if (bitRead(Device_status, 0)) {                                    // Run_Active led
            if (millis() >= RUN_Active_Led_Start_Time + Led_On_Time) {
                digitalWrite(RUN_Active_led_pin, !digitalRead(RUN_Active_led_pin));       // toggle the CAM_Active led
            }
            else {
                digitalWrite(RUN_Active_led_pin, OFF);                              // turn the CAM_Active led off
            }
        }
    }
}
// End of Programme--------------------------------------------------------------------------------
