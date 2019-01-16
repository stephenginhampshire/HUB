/* 
 * File:   HUB_Commands.h
 * Author: Stephen
 *
 * Created on 14 February 2018, 13:19
 */

#ifndef HUB_COMMANDS_H
#define HUB_COMMANDS_H

int constexpr Focuser_Current_Focuser_Position = 0;
int constexpr Focuser_Motor_Moving_Status = 1;
int constexpr Focuser_Motor_Controller_Status = 2;
int constexpr Focuser_Firmware_Version_String = 3;
int constexpr Focuser_Firmware_Name_and_Version_String = 4;
int constexpr Focuser_New_Target_Position = 5;
int constexpr Focuser_Temperature = 6;
int constexpr Focuser_MaxStep = 7;
int constexpr Focuser_MaxIncrement = 10;
int constexpr Focuser_Coil_Powerting = 11;
int constexpr Focuser_Reverse_Directionting = 13;
int constexpr Focuser_MotorSpeed = 15;
int constexpr Focuser_Display_in_Celsius = 16;
int constexpr Focuser_Display_in_Fahrenheit = 17;
int constexpr Focuser_User_specified_StepSize = 18;
int constexpr Focuser_StepSize_value = 19;
int constexpr Focuser_Temperature_Coefficient = 22;
int constexpr Focuser_Temperature_Compensation = 23;
int constexpr Focuser_State_of_Temperature_Compensation = 24;
int constexpr Focuser_Temperature_Compensation_available = 25;
int constexpr Focuser_Home_Motor_Position_To_ZERO = 28;
int constexpr Focuser_Step_Mode = 29;
int constexpr Focuser_Current_Motor_Position = 31;
int constexpr Focuser_StepSize_is_Enabled = 32;
int constexpr Focuser_Step_Size = 33;
int constexpr Focuser_Display_Status = 36;
int constexpr Focuser_Temperature_mode = 38;
int constexpr Focuser_Target_Motor_Position = 39;
int constexpr Focuser_Reset_Arduino_Controller = 40;
int constexpr Focuser_Reset_Focuser_defaults = 42;
int constexpr Focuser_Motor_Speed = 43;
int constexpr Focuser_Motor_Speed_Threshold_When_Moving = 44;
int constexpr Focuser_Motor_Speed_Change_When_Moving = 46;
int constexpr Focuser_Motor_Speed_Change_Enabled = 47;
int constexpr Focuser_Save_Settings_To_EEPROM = 48;
int constexpr Focuser_Humidity = 50;
int constexpr Focuser_Longitude = 51;
int constexpr Focuser_Latitude = 52;
int constexpr Focuser_Altitude = 53;
int constexpr Focuser_BEMF = 60;
int constexpr Focuser_Update_of_Position_When_Moving = 61;
int constexpr Focuser_Status_of_Home_Position_Switch = 63;
int constexpr Focuser_Move_Steps = 64;
int constexpr Focuser_Jogging_State = 65;
int constexpr Focuser_Jogging_Direction = 67;
int constexpr Focuser_Delay_After_Move = 71;
int constexpr Focuser_Backlash_In = 73;
int constexpr Focuser_Backlash_Out = 75;
int constexpr Focuser_Backlash_In_Steps_In = 77;
int constexpr Focuser_Number_Of_Backlash_Steps_In = 78;
int constexpr Focuser_Number_Of_Backlash_Steps_Out = 80;
int constexpr Focuser_Temperature_Compensation_Direction = 87;

int constexpr HUB_Display = 90;

int constexpr Telescope_Azimuth = 100;
int constexpr Telescope_Declination = 102;
int constexpr Telescope_CanUnpark = 104;
int constexpr Telescope_CanSyncAltAz = 106;
int constexpr Telescope_CanSync = 108;
int constexpr Telescope_CanSlewAsync = 110;
int constexpr Telescope_CanSlewAltAzAsync = 112;
int constexpr Telescope_CanSlewAltAz = 114;
int constexpr Telescope_CanSlew = 116;
int constexpr Telescope_CanSetTracking = 118;
int constexpr Telescope_CanSetRightAscensionRate = 120;
int constexpr Telescope_CanSetPierSide = 122;
int constexpr Telescope_CanSetPark = 124;
int constexpr Telescope_CanSetGuideRates = 126;
int constexpr Telescope_CanSetDeclinationRate = 128;
int constexpr Telescope_CanPulseGuide = 130;
int constexpr Telescope_DeclinationRate = 132;
int constexpr Telescope_DoesRefraction = 134;
int constexpr Telescope_EquatorialSystem = 136;
int constexpr Telescope_FocalLength = 138;
int constexpr Telescope_TrackingRate = 140;
int constexpr Telescope_Tracking = 142;
int constexpr Telescope_TargetRightAscension = 144;
int constexpr Telescope_TargetDeclination = 146;
int constexpr Telescope_SlewSettleTime = 148;
int constexpr Telescope_Slewing = 150;
int constexpr Telescope_SiteLongitude = 152;
int constexpr Telescope_CanPark = 154;
int constexpr Telescope_SiteLatitude = 156;
int constexpr Telescope_SiderealTime = 158;
int constexpr Telescope_SideOfPier = 160;
int constexpr Telescope_RightAscensionRate = 162;
int constexpr Telescope_RightAscension = 164;
int constexpr Telescope_IsPulseGuiding = 166;
int constexpr Telescope_GuideRateRightAscension = 168;
int constexpr Telescope_GuideRateDeclination = 170;
int constexpr Telescope_SiteElevation = 172;
int constexpr Telescope_CanFindHome = 174;
int constexpr Telescope_UTCDATE = 177;
int constexpr Telescope_AtPark = 178;
int constexpr Telescope_AtHome = 180;
int constexpr Telescope_ApertureDiameter = 182;
int constexpr Telescope_ApertureArea = 184;
int constexpr Telescope_Altitude = 186;
int constexpr Telescope_AlignmentMode = 188;
int constexpr Telescope_InterfaceVersion = 194;
int constexpr Telescope_DriverVersion = 196;
int constexpr Telescope_Connected = 202;
int constexpr Telescope_TrackingRates = 204;
int constexpr Telescope_AbortSlew = 206;
int constexpr Telescope_Action = 208;
int constexpr Telescope_AxisRates = 210;
int constexpr Telescope_CanMoveAxis = 212;
int constexpr Telescope_CommandBlind = 214;
int constexpr Telescope_Command = 216;
int constexpr Telescope_DestinationSideOfPier = 220;
int constexpr Telescope_Dispose = 222;
int constexpr Telescope_FindHome = 224;
int constexpr Telescope_MoveAxis = 226;
int constexpr Telescope_Park = 228;
int constexpr Telescope_PulseGuide = 230;
int constexpr Telescope_SetPark = 232;
int constexpr Telescope_SetupDialog = 234;
int constexpr Telescope_SlewToAltAz = 236;
int constexpr Telescope_SlewToAltAzAsync = 238;
int constexpr Telescope_SlewToCoordinates = 240;
int constexpr Telescope_SlewToCoordinatesAsync = 242;
int constexpr Telescope_SlewToTarget = 244;
int constexpr Telescope_SlewToTargetAsync = 246;
int constexpr Telescope_SyncToAltAz = 248;
int constexpr Telescope_SyncToCoordinates = 250;
int constexpr Telescope_SyncToTarget = 252;
int constexpr Telescope_Unpark = 254;

#endif	/* HUB_COMMANDS_H */

