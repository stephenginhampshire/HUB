/* 
 * File:   HUB_Commands.h
 * Author: Stephen Gould
 *
 * Created on 14 February 2018, 13:19
 */

#ifndef HUB_COMMANDS_H
#define HUB_COMMANDS_H

int constexpr Focuser_Start = 10;
int constexpr Focuser_Current_Focuser_Position = 11;
int constexpr Focuser_Motor_Moving_Status = 12;
int constexpr Focuser_Is_Moving = 13;
int constexpr Focuser_Firmware_Version_String = 14;
int constexpr Focuser_Firmware_Name_and_Version_String = 15;
int constexpr Focuser_Target_Position = 16;
int constexpr Focuser_Temperature = 17;
int constexpr Focuser_Maximum_Step = 18;				
int constexpr Focuser_Maximum_Increment = 19;
int constexpr Focuser_Coil_Power = 20;									// parameter_one == 0 GET, else SET
int constexpr Focuser_Motor_Speed = 21;
int constexpr Focuser_Step_Size = 22;
int constexpr Focuser_Temperature_Coefficient = 23;
int constexpr Focuser_Temperature_Compensation = 24;
int constexpr Focuser_Move_to_Position = 25;
int constexpr Focuser_Step_Mode = 26;
int constexpr Focuser_Step_Size_Enabled = 27;
int constexpr Focuser_Display_Visible = 28;
int constexpr Focuser_Temperature_Mode = 29;
int constexpr Focuser_Reset_Arduino_Controller = 30;
int constexpr Focuser_Reset_Focuser_Defaults = 31;
int constexpr Focuser_Motor_Speed_Threshold_When_Moving = 32;
int constexpr Focuser_Motor_Speed_Change_Enabled_When_Moving = 33;
int constexpr Focuser_Save_Settings_to_EEPROM = 34;
int constexpr Focuser_Humidity = 35;
int constexpr Focuser_Longitude = 36;
int constexpr Focuser_Latitude = 37;
int constexpr Focuser_Altitude = 38;
int constexpr Focuser_Voltages = 39;
int constexpr Focuser_Controller_Current = 40;
int constexpr Focuser_Update_of_Position_When_Moving = 41;
int constexpr Focuser_Status_of_Home_Position_Switch = 42;
int constexpr Focuser_Jogging_Steps = 43;
int constexpr Focuser_Jogging_State = 44;
int constexpr Focuser_Jogging_Direction = 45;
int constexpr Focuser_Delay_After_Step = 46;
int constexpr Focuser_Backlash_In = 47;
int constexpr Focuser_Backlash_Out = 48;
int constexpr Focuser_Number_of_Backlash_Steps_In = 49;
int constexpr Focuser_Number_of_Backlash_Steps_Out = 50;
int constexpr Focuser_Temperature_Compensation_Direction = 51;
int constexpr Focuser_to_Controller_Heartbeat = 52;
int constexpr Focuser_Get_Year_Month = 53;
int constexpr Focuser_Get_Day_Hour = 54;
int constexpr Focuser_Get_Minute_Second = 55;
int constexpr Focuser_End = 56;

int constexpr HUB_Start = 60;
int constexpr HUB_Display_Visible = 61;
int constexpr HUB_Get_Year_Month = 62;
int constexpr HUB_Get_Day_Hour = 63;
int constexpr HUB_Get_Minute_Second = 64;
int constexpr HUB_Delete_Log_File = 65;
int constexpr HUB_End = 66;

int constexpr Telescope_Start = 100;
int constexpr Telescope_Azimuth = 101;
int constexpr Telescope_Declination = 102;
int constexpr Telescope_Can_Unpark = 103;
int constexpr Telescope_Can_Sync_AltAz = 104;
int constexpr Telescope_Can_Sync = 105;
int constexpr Telescope_Can_Slew_Async = 106;
int constexpr Telescope_Can_Slew_AltAz_Async = 107;
int constexpr Telescope_Can_Slew_AltAz = 108;
int constexpr Telescope_Can_Slew = 109;
int constexpr Telescope_Can_Set_Tracking = 110;
int constexpr Telescope_Can_Set_Right_Ascension_Rate = 111;
int constexpr Telescope_Can_Set_Pier_Side = 112;
int constexpr Telescope_Can_Set_Park = 113;
int constexpr Telescope_Can_Set_Guide_Rates = 114;
int constexpr Telescope_Can_Set_Declination_Rate = 115;
int constexpr Telescope_Can_Pulse_Guide = 116;
int constexpr Telescope_Declination_Rate = 117;
int constexpr Telescope_Does_Refraction = 118;
int constexpr Telescope_Equatorial_System = 119;
int constexpr Telescope_Focal_Length = 120;
int constexpr Telescope_Tracking_Rate = 121;
int constexpr Telescope_Tracking = 122;
int constexpr Telescope_Future_Target_Right_Ascension = 123;
int constexpr Telescope_Future_Target_Declination = 124;
int constexpr Telescope_Slew_Settle_Time = 125;
int constexpr Telescope_Slewing = 126;
int constexpr Telescope_Site_Longitude = 127;
int constexpr Telescope_Can_Park = 128;
int constexpr Telescope_Site_Latitude = 129;
int constexpr Telescope_Sidereal_Time_Hours = 130;
int constexpr Telescope_Sidereal_Time_Minutes_Seconds = 131;
int constexpr Telescope_Side_of_Pier = 132;
int constexpr Telescope_Right_Ascension_Rate = 133;
int constexpr Telescope_Right_Ascension = 134;
int constexpr Telescope_Is_Pulse_Guiding = 135;
int constexpr Telescope_Guide_Rate_Right_Ascension = 136;
int constexpr Telescope_Guide_Rate_Declination = 137;
int constexpr Telescope_Site_Elevation = 138;
int constexpr Telescope_Can_Find_Home = 139;
int constexpr Telescope_UTCDATE_Year_Month = 140;
int constexpr Telescope_UTCDATE_Day_Hour = 141;
int constexpr Telescope_UTCDATE_Minute_Second = 142;
int constexpr Telescope_At_Park = 143;
int constexpr Telescope_At_Home = 144;
int constexpr Telescope_Aperture_Diameter = 145;
int constexpr Telescope_Aperture_Area = 146;
int constexpr Telescope_Altitude = 147;
int constexpr Telescope_Alignment_Mode = 148;
int constexpr Telescope_Array_List = 149;
int constexpr Telescope_Name = 150;
int constexpr Telescope_Interface_Version = 151;
int constexpr Telescope_Driver_Version = 152;
int constexpr Telescope_Driver_Info = 153;
int constexpr Telescope_Description = 154;
int constexpr Telescope_Connected = 155;
int constexpr Telescope_Tracking_Rates_0_1 = 156;
int constexpr Telescope_Tracking_Rates_2_3 = 157;
int constexpr Telescope_Abort_Slew = 158;
int constexpr Telescope_Action = 159;
int constexpr Telescope_Axis_Rates = 160;
int constexpr Telescope_Can_Move_Axis = 161;
int constexpr Telescope_Command_Blind = 162;
int constexpr Telescope_Command_Bool = 163;
int constexpr Telescope_Command_String = 164;
int constexpr Telescope_Destination_Side_of_Pier = 165;
int constexpr Telescope_Dispose = 166;
int constexpr Telescope_Find_Home = 167;
int constexpr Telescope_Move_Axis = 168;
int constexpr Telescope_Park = 169;
int constexpr Telescope_Pulse_Guide = 170;
int constexpr Telescope_Set_Park = 171;
int constexpr Telescope_Setup_Dialog = 172;
int constexpr Telescope_Slew_to_AltAz = 173;
int constexpr Telescope_Slew_to_AltAz_Async = 174;
int constexpr Telescope_Slew_to_Coordinates = 175;
int constexpr Telescope_Slew_to_Coordinates_Async = 176;
int constexpr Telescope_Slew_to_Target = 177;
int constexpr Telescope_Slew_to_Target_Async = 178;
int constexpr Telescope_Sync_to_AltAz = 179;
int constexpr Telescope_Sync_to_Coordinates = 180;
int constexpr Telescope_Sync_to_Target = 181;
int constexpr Telescope_Unpark = 182;
int constexpr Telescope_Altitude_to_Controller_Heartbeat = 183;
int constexpr Telescope_Azimuth_to_Controller_Heartbeat = 184;
int constexpr Telescope_Get_Year_Month = 185;
int constexpr Telescope_Get_Day_Hour = 186;
int constexpr Telescope_Get_Minute_Second = 187;
int constexpr Telescope_Display_Visible = 188;
int constexpr Telescope_End = 189;
// Error Responses ------------------------------------------------------------------------------------------
int constexpr Telescope_Command_Error_Response = 200;
int constexpr Focuser_Command_Error_Response = 201;
// source definitions: --------------------------------------------------------------------------------------------------------------
int constexpr SOURCE_CONTROLLER = 0;
int constexpr SOURCE_ALTITUDE = 1;
int constexpr SOURCE_AZIMUTH = 2;
int constexpr SOURCE_FOCUSER = 3;
int constexpr SOURCE_ALTAZI = 4;
int constexpr SOURCE_HUB = 5;
int constexpr SOURCE_TERTIARY_MOTOR = 6;

#endif	/* HUB_COMMANDS_H */

