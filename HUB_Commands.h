/* 
 * File:   HUB_Commands.h
 * Author: Stephen Gould
 *
 * Created on 14 February 2018, 13:19
 */

#ifndef HUB_COMMANDS_H
#define HUB_COMMANDS_H

int constexpr Focuser_Current_Focuser_Position = 1;
int constexpr Focuser_Motor_Moving_Status = 2;
int constexpr Focuser_Motor_Controller_Status = 3;
int constexpr Focuser_Firmware_Version_String = 4;
int constexpr Focuser_Firmware_Name_and_Version_String = 5;
int constexpr Focuser_Target_Position = 6;
int constexpr Focuser_Temperature = 7;
int constexpr Focuser_Maximum_Step = 8;									// parameter_one == 0 GET, else SET
int constexpr Focuser_Maximum_Increment = 9;
int constexpr Focuser_Coil_Power = 10;									// parameter_one == 0 GET, else SET
int constexpr Focuser_Reverse_Direction = 11;
int constexpr Focuser_Motor_Speed = 12;
int constexpr Focuser_Display_Unit = 13;
int constexpr Focuser_User_Specified_Step_Size = 14;
int constexpr Focuser_Step_Size = 15;
int constexpr Focuser_Temperature_Coefficient = 16;
int constexpr Focuser_Temperature_Compensation = 17;
int constexpr Focuser_Temperature_Compensation_Available = 18;
int constexpr Focuser_Home_Motor_Position_To_ZERO = 19;
int constexpr Focuser_Step_Mode = 20;
int constexpr Focuser_Step_Size_is_Enabled = 21;
int constexpr Focuser_Display = 22;
int constexpr Focuser_Temperature_Mode = 23;
int constexpr Focuser_Reset_Arduino_Controller = 24;
int constexpr Focuser_Reset_Focuser_Defaults = 25;
int constexpr Focuser_Motor_Speed_Threshold_When_Moving = 26;
int constexpr Focuser_Motor_Speed_Change_When_Moving = 27;
int constexpr Focuser_Save_Settings_to_EEPROM = 28;
int constexpr Focuser_Humidity = 29;
int constexpr Focuser_Longitude = 30;
int constexpr Focuser_Latitude = 31;
int constexpr Focuser_Altitude = 32;
int constexpr Focuser_BEMF = 33;
int constexpr Focuser_Update_of_Position_When_Moving = 34;
int constexpr Focuser_Status_of_Home_Position_Switch = 35;
int constexpr Focuser_Jogging_Steps = 36;
int constexpr Focuser_Jogging_State = 37;
int constexpr Focuser_Jogging_Direction = 38;
int constexpr Focuser_Delay_After_Move = 39;
int constexpr Focuser_Backlash_In = 40;
int constexpr Focuser_Backlash_Out = 41;
int constexpr Focuser_Number_of_Backlash_Steps_In = 42;
int constexpr Focuser_Number_of_Backlash_Steps_Out = 43;
int constexpr Focuser_Temperature_Compensation_Direction = 44;

int constexpr Focuser_to_Controller_Heartbeat = 49;

int constexpr HUB_Display = 50;
int constexpr HUB_Get_Day_Month = 60;
int constexpr HUB_Get_Year_Hour = 61;
int constexpr HUB_Get_Minute_Second = 62;

int constexpr Telescope_Azimuth = 100;
int constexpr Telescope_Declination = 101;
int constexpr Telescope_Can_Unpark = 102;
int constexpr Telescope_Can_Sync_AltAz = 103;
int constexpr Telescope_Can_Sync = 104;
int constexpr Telescope_Can_Slew_Async = 105;
int constexpr Telescope_Can_Slew_AltAz_Async = 106;
int constexpr Telescope_Can_Slew_AltAz = 107;
int constexpr Telescope_Can_Slew = 108;
int constexpr Telescope_Can_Set_Tracking = 109;
int constexpr Telescope_Can_Set_Right_Ascension_Rate = 110;
int constexpr Telescope_Can_Set_Pier_Side = 111;
int constexpr Telescope_Can_Set_Park = 112;
int constexpr Telescope_Can_Set_Guide_Rates = 113;
int constexpr Telescope_Can_Set_Declination_Rate = 114;
int constexpr Telescope_Can_Pulse_Guide = 115;
int constexpr Telescope_Declination_Rate = 116;
int constexpr Telescope_Does_Refraction = 117;
int constexpr Telescope_Equatorial_System = 118;
int constexpr Telescope_Focal_Length = 119;
int constexpr Telescope_Tracking_Rate = 120;
int constexpr Telescope_Tracking = 121;
int constexpr Telescope_Target_Right_Ascension = 122;
int constexpr Telescope_Target_Declination = 123;
int constexpr Telescope_Slew_Settle_Time = 124;
int constexpr Telescope_Slewing = 125;
int constexpr Telescope_Site_Longitude = 126;
int constexpr Telescope_Can_Park = 127;
int constexpr Telescope_Site_Latitude = 128;
int constexpr Telescope_Sidereal_Time = 129;
int constexpr Telescope_Side_of_Pier = 130;
int constexpr Telescope_Right_Ascension_Rate = 131;
int constexpr Telescope_Right_Ascension = 132;
int constexpr Telescope_Is_Pulse_Guiding = 133;
int constexpr Telescope_Guide_Rate_Right_Ascension = 134;
int constexpr Telescope_Guide_Rate_Declination = 135;
int constexpr Telescope_Site_Elevation = 136;
int constexpr Telescope_Can_Find_Home = 137;
int constexpr Telescope_UTCDATE = 138;
int constexpr Telescope_At_Park = 139;
int constexpr Telescope_At_Home = 140;
int constexpr Telescope_Aperture_Diameter = 141;
int constexpr Telescope_Aperture_Area = 142;
int constexpr Telescope_Altitude = 143;
int constexpr Telescope_Alignment_Mode = 144;
int constexpr Telescope_Interface_Version = 145;
int constexpr Telescope_Driver_Version = 146;
int constexpr Telescope_Connected = 147;
int constexpr Telescope_Tracking_Rates = 148;
int constexpr Telescope_Abort_Slew = 149;
int constexpr Telescope_Action = 150;
int constexpr Telescope_Axis_Rates = 151;
int constexpr Telescope_Can_Move_Axis = 152;
int constexpr Telescope_Command_Blind = 153;
int constexpr Telescope_Command = 154;
int constexpr Telescope_Destination_Side_of_Pier = 155;
int constexpr Telescope_Dispose = 156;
int constexpr Telescope_Find_Home = 157;
int constexpr Telescope_Move_Axis = 158;
int constexpr Telescope_Park = 159;
int constexpr Telescope_Pulse_Guide = 160;
int constexpr Telescope_Set_Park = 161;
int constexpr Telescope_Setup_Dialog = 162;
int constexpr Telescope_Slew_to_AltAz = 163;
int constexpr Telescope_Slew_to_AltAz_Async = 164;
int constexpr Telescope_Slew_to_Coordinates = 165;
int constexpr Telescope_Slew_to_Coordinates_Async = 166;
int constexpr Telescope_Slew_to_Target = 167;
int constexpr Telescope_Slew_to_Target_Async = 168;
int constexpr Telescope_Sync_to_AltAz = 169;
int constexpr Telescope_Sync_to_Coordinates = 170;
int constexpr Telescope_Sync_to_Target = 171;
int constexpr Telescope_Unpark = 172;

int constexpr Telescope_Altitude_to_Controller_Heartbest = 188;
int constexpr Telescope_Azimuth_to_Controller_Heartbeat = 189;

// source definitions: --------------------------------------------------------------------------------------------------------------
int constexpr SOURCE_CONTROLLER = 0;
int constexpr SOURCE_ALTITUDE_MOTOR = 1;
int constexpr SOURCE_AZIMUTH_MOTOR = 2;
int constexpr SOURCE_FOCUSER = 3;
int constexpr SOURCE_BOTH_MOTORS = 4;
int constexpr SOURCE_HUB = 5;

#endif	/* HUB_COMMANDS_H */

