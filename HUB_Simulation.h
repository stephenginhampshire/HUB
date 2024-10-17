#pragma once
/* Hub_Simulation.h
//
// Version History
// Date         Version     Change Implemented
// 05/10/2024   1.0         New base
//
// Communications Control Characters -------------------------------------------------------------
constexpr uint8_t CPU = 6;       // CPU Address
constexpr uint8_t ALT = 7;       // Altitude Address
constexpr uint8_t AZI = 8;       // Azimuth Address
constexpr uint8_t ALTAZI = 9;    // Both Motors Address
constexpr uint8_t FOC = 10;      // Focuser Address
constexpr uint8_t CAM = 11;      // Camera Address
constexpr uint8_t MON = 12;      // Monitor Address
constexpr uint8_t ALL = 13;      // All Devices Address
constexpr uint8_t HUB = 14;      // HUB Address
double Standard_Test_Packets[11][13] = {
    //srt,tar,src,sts,typ,com,stx,par,fld,etx,eot
    { SOH,ALT,CPU,100,STX,105,002,001,030,003,004},       // Halt
    { STX,5,1,100,11,0,0,0,0,0,0,0,ETX },       // Request Firmware Version
    { STX,5,1,105,11,0,0,0,0,0,0,0,ETX },       // Motor Move To
    { STX,5,1,106,11,0,0,0,0,0,0,0,ETX },       // Motor Move By
    { STX,5,1,110,11,0,0,0,0,0,0,0,ETX },       // Motor Find Home
    { STX,5,1,115,11,0,0,0,0,0,0,0,ETX },       // Heartbeat
    { STX,5,1,120,11,0,0,0,0,0,0,0,ETX },       // Request Reset
    { STX,5,1,125,11,0,0,0,0,0,0,0,ETX },       // Motor AccelStepper Parameters
    { STX,5,1,130,11,0,0,0,0,0,0,0,ETX },       // Motor Controller Parameters
    { STX,5,1,135,11,0,0,0,0,0,0,0,ETX },       // Fan Threshold
    { STX,5,1,140,11,0,0,0,0,0,0,0,ETX },       // Are You Connected
};
int Test_Packet_Number = 0;
