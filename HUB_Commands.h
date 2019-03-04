/* 
 * File:   HUB_Commands.h
 * Author: Stephen Gould
 *
 * Created on 14 February 2018, 13:19
 */

#ifndef HUB_COMMANDS_H
#define HUB_COMMANDS_H

int constexpr Focuser_Start = 10;								// start command number for Focuser commands
int constexpr Focuser_Current_Focuser_Position = Focuser_Start+1;
int constexpr Focuser_Motor_Moving_Status = Focuser_Current_Focuser_Position + 1;
int constexpr Focuser_Motor_Controller_Status = Focuser_Motor_Moving_Status + 1;
int constexpr Focuser_Firmware_Version_String = Focuser_Motor_Controller_Status + 1;
int constexpr Focuser_Firmware_Name_and_Version_String = Focuser_Firmware_Version_String + 1;
int constexpr Focuser_Target_Position = Focuser_Firmware_Name_and_Version_String + 1;
int constexpr Focuser_Temperature = Focuser_Target_Position + 1;
int constexpr Focuser_Maximum_Step = Focuser_Temperature + 1;									// parameter_one == 0 GET, else SET
int constexpr Focuser_Maximum_Increment = Focuser_Maximum_Step + 1;
int constexpr Focuser_Coil_Power = Focuser_Maximum_Increment + 1;									// parameter_one == 0 GET, else SET
int constexpr Focuser_Motor_Speed = Focuser_Coil_Power + 1;
int constexpr Focuser_Display_Unit = Focuser_Motor_Speed + 1;
int constexpr Focuser_User_Specified_Step_Size = Focuser_Display_Unit + 1;
int constexpr Focuser_Step_Size = Focuser_User_Specified_Step_Size + 1;
int constexpr Focuser_Temperature_Coefficient = Focuser_Step_Size + 1;
int constexpr Focuser_Temperature_Compensation = Focuser_Temperature_Coefficient + 1;
int constexpr Focuser_Temperature_Compensation_Enabled = Focuser_Temperature_Compensation + 1;
int constexpr Focuser_Move_to_Position = Focuser_Temperature_Compensation_Enabled + 1;
int constexpr Focuser_Step_Mode = Focuser_Move_to_Position + 1;
int constexpr Focuser_Step_Size_Enabled = Focuser_Step_Mode + 1;
int constexpr Focuser_Display = Focuser_Step_Size_Enabled + 1;
int constexpr Focuser_Temperature_Mode = Focuser_Display + 1;
int constexpr Focuser_Reset_Arduino_Controller = Focuser_Temperature_Mode + 1;
int constexpr Focuser_Reset_Focuser_Defaults = Focuser_Reset_Arduino_Controller + 1;
int constexpr Focuser_Motor_Speed_Threshold_When_Moving = Focuser_Reset_Focuser_Defaults + 1;
int constexpr Focuser_Motor_Speed_Change_When_Moving = Focuser_Motor_Speed_Threshold_When_Moving + 1;
int constexpr Focuser_Save_Settings_to_EEPROM = Focuser_Motor_Speed_Change_When_Moving + 1;
int constexpr Focuser_Humidity = Focuser_Save_Settings_to_EEPROM + 1;
int constexpr Focuser_Longitude = Focuser_Humidity + 1;
int constexpr Focuser_Latitude = Focuser_Longitude + 1;
int constexpr Focuser_Altitude = Focuser_Latitude + 1;
int constexpr Focuser_Voltages = Focuser_Altitude + 1;
int constexpr Focuser_Update_of_Position_When_Moving = Focuser_Voltages + 1;
int constexpr Focuser_Status_of_Home_Position_Switch = Focuser_Update_of_Position_When_Moving + 1;
int constexpr Focuser_Jogging_Steps = Focuser_Status_of_Home_Position_Switch + 1;
int constexpr Focuser_Jogging_State = Focuser_Jogging_Steps + 1;
int constexpr Focuser_Jogging_Direction = Focuser_Jogging_State + 1;
int constexpr Focuser_Delay_After_Move = Focuser_Jogging_Direction + 1;
int constexpr Focuser_Backlash_In = Focuser_Delay_After_Move + 1;
int constexpr Focuser_Backlash_Out = Focuser_Backlash_In + 1;
int constexpr Focuser_Number_of_Backlash_Steps_In = Focuser_Backlash_Out + 1;
int constexpr Focuser_Number_of_Backlash_Steps_Out = Focuser_Number_of_Backlash_Steps_In + 1;
int constexpr Focuser_Temperature_Compensation_Direction = Focuser_Number_of_Backlash_Steps_Out + 1;
int constexpr Focuser_to_Controller_Heartbeat = Focuser_Temperature_Compensation_Direction + 1;
int constexpr Focuser_Get_Year_Month = Focuser_to_Controller_Heartbeat + 1;
int constexpr Focuser_Get_Day_Hour = Focuser_Get_Year_Month + 1;
int constexpr Focuser_Get_Minute_Second = Focuser_Get_Day_Hour + 1;
int constexpr Focuser_End = Focuser_Get_Minute_Second + 1;

#if (Focuser_End < 60)
	int constexpr HUB_Start = 60;
#else
	int constexpr HUB_Start = 70;
#endif
int constexpr HUB_Display = HUB_Start + 1;
int constexpr HUB_Get_Year_Month = HUB_Display + 1;
int constexpr HUB_Get_Day_Hour = HUB_Get_Year_Month + 1;
int constexpr HUB_Get_Minute_Second = HUB_Get_Day_Hour + 1;
int constexpr HUB_Delete_Log_File = HUB_Get_Minute_Second + 1;
int constexpr HUB_End = HUB_Delete_Log_File + 1;

int constexpr Telescope_Start = 100;
int constexpr Telescope_Azimuth = Telescope_Start + 1;
int constexpr Telescope_Declination = Telescope_Azimuth + 1;
int constexpr Telescope_Can_Unpark = Telescope_Declination + 1;
int constexpr Telescope_Can_Sync_AltAz = Telescope_Can_Unpark + 1;
int constexpr Telescope_Can_Sync = Telescope_Can_Sync_AltAz + 1;
int constexpr Telescope_Can_Slew_Async = Telescope_Can_Sync + 1;
int constexpr Telescope_Can_Slew_AltAz_Async = Telescope_Can_Slew_Async + 1;
int constexpr Telescope_Can_Slew_AltAz = Telescope_Can_Slew_AltAz_Async + 1;
int constexpr Telescope_Can_Slew = Telescope_Can_Slew_AltAz + 1;
int constexpr Telescope_Can_Set_Tracking = Telescope_Can_Slew + 1;
int constexpr Telescope_Can_Set_Right_Ascension_Rate = Telescope_Can_Set_Tracking + 1;
int constexpr Telescope_Can_Set_Pier_Side = Telescope_Can_Set_Right_Ascension_Rate + 1;
int constexpr Telescope_Can_Set_Park = Telescope_Can_Set_Pier_Side + 1;
int constexpr Telescope_Can_Set_Guide_Rates = Telescope_Can_Set_Park + 1;
int constexpr Telescope_Can_Set_Declination_Rate = Telescope_Can_Set_Guide_Rates + 1;
int constexpr Telescope_Can_Pulse_Guide = Telescope_Can_Set_Declination_Rate + 1;
int constexpr Telescope_Declination_Rate = Telescope_Can_Pulse_Guide + 1;
int constexpr Telescope_Does_Refraction = Telescope_Declination_Rate + 1;
int constexpr Telescope_Equatorial_System = Telescope_Does_Refraction + 1;
int constexpr Telescope_Focal_Length = Telescope_Equatorial_System + 1;
int constexpr Telescope_Tracking_Rate = Telescope_Focal_Length + 1;
int constexpr Telescope_Tracking = Telescope_Tracking_Rate + 1;
int constexpr Telescope_Future_Target_Right_Ascension = Telescope_Tracking + 1;
int constexpr Telescope_Future_Target_Declination = Telescope_Future_Target_Right_Ascension + 1;
int constexpr Telescope_Slew_Settle_Time = Telescope_Future_Target_Declination + 1;
int constexpr Telescope_Slewing = Telescope_Slew_Settle_Time + 1;
int constexpr Telescope_Site_Longitude = Telescope_Slewing + 1;
int constexpr Telescope_Can_Park = Telescope_Site_Longitude + 1;
int constexpr Telescope_Site_Latitude = Telescope_Can_Park + 1;
int constexpr Telescope_Sidereal_Time_Hours = Telescope_Site_Latitude + 1;
int constexpr Telescope_Sidereal_Time_Minutes_Seconds = Telescope_Sidereal_Time_Hours + 1;
int constexpr Telescope_Side_of_Pier = Telescope_Sidereal_Time_Minutes_Seconds + 1;
int constexpr Telescope_Right_Ascension_Rate = Telescope_Side_of_Pier + 1;
int constexpr Telescope_Right_Ascension = Telescope_Right_Ascension_Rate + 1;
int constexpr Telescope_Is_Pulse_Guiding = Telescope_Right_Ascension + 1;
int constexpr Telescope_Guide_Rate_Right_Ascension = Telescope_Is_Pulse_Guiding + 1;
int constexpr Telescope_Guide_Rate_Declination = Telescope_Guide_Rate_Right_Ascension + 1;
int constexpr Telescope_Site_Elevation = Telescope_Guide_Rate_Declination + 1;
int constexpr Telescope_Can_Find_Home = Telescope_Site_Elevation + 1;
int constexpr Telescope_UTCDATE_Year_Month = Telescope_Can_Find_Home + 1;
int constexpr Telescope_UTCDATE_Day_Hour = Telescope_UTCDATE_Year_Month + 1;
int constexpr Telescope_UTCDATE_Minute_Second = Telescope_UTCDATE_Day_Hour + 1;
int constexpr Telescope_At_Park = Telescope_UTCDATE_Minute_Second + 1;
int constexpr Telescope_At_Home = Telescope_At_Park + 1;
int constexpr Telescope_Aperture_Diameter = Telescope_At_Home + 1;
int constexpr Telescope_Aperture_Area = Telescope_Aperture_Diameter + 1;
int constexpr Telescope_Altitude = Telescope_Aperture_Area + 1;
int constexpr Telescope_Alignment_Mode = Telescope_Altitude + 1;
int constexpr Telescope_Array_List = Telescope_Alignment_Mode + 1;
int constexpr Telescope_Name = Telescope_Array_List + 1;
int constexpr Telescope_Interface_Version = Telescope_Name + 1;
int constexpr Telescope_Driver_Version = Telescope_Interface_Version + 1;
int constexpr Telescope_Driver_Info = Telescope_Driver_Version + 1;
int constexpr Telescope_Description = Telescope_Driver_Info + 1;
int constexpr Telescope_Connected = Telescope_Description + 1;
int constexpr Telescope_Tracking_Rates_0_1 = Telescope_Connected + 1;
int constexpr Telescope_Tracking_Rates_2_3 = Telescope_Tracking_Rates_0_1 + 1;
int constexpr Telescope_Abort_Slew = Telescope_Tracking_Rates_2_3 + 1;
int constexpr Telescope_Action = Telescope_Abort_Slew + 1;
int constexpr Telescope_Axis_Rates = Telescope_Action + 1;
int constexpr Telescope_Can_Move_Axis = Telescope_Axis_Rates + 1;
int constexpr Telescope_Command_Blind = Telescope_Can_Move_Axis + 1;
int constexpr Telescope_Command_Bool = Telescope_Command_Blind + 1;
int constexpr Telescope_Command_String = Telescope_Command_Bool + 1;
int constexpr Telescope_Destination_Side_of_Pier = Telescope_Command_String + 1;
int constexpr Telescope_Dispose = Telescope_Destination_Side_of_Pier + 1;
int constexpr Telescope_Find_Home = Telescope_Dispose + 1;
int constexpr Telescope_Move_Axis = Telescope_Find_Home + 1;
int constexpr Telescope_Park = Telescope_Move_Axis + 1;
int constexpr Telescope_Pulse_Guide = Telescope_Park + 1;
int constexpr Telescope_Set_Park = Telescope_Pulse_Guide + 1;
int constexpr Telescope_Setup_Dialog = Telescope_Set_Park + 1;
int constexpr Telescope_Slew_to_AltAz = Telescope_Setup_Dialog + 1;
int constexpr Telescope_Slew_to_AltAz_Async = Telescope_Slew_to_AltAz + 1;
int constexpr Telescope_Slew_to_Coordinates = Telescope_Slew_to_AltAz_Async + 1;
int constexpr Telescope_Slew_to_Coordinates_Async = Telescope_Slew_to_Coordinates + 1;
int constexpr Telescope_Slew_to_Target = Telescope_Slew_to_Coordinates_Async + 1;
int constexpr Telescope_Slew_to_Target_Async = Telescope_Slew_to_Target + 1;
int constexpr Telescope_Sync_to_AltAz = Telescope_Slew_to_Target_Async + 1;
int constexpr Telescope_Sync_to_Coordinates = Telescope_Sync_to_AltAz + 1;
int constexpr Telescope_Sync_to_Target = Telescope_Sync_to_Coordinates + 1;
int constexpr Telescope_Unpark = Telescope_Sync_to_Target + 1;
int constexpr Telescope_Altitude_to_Controller_Heartbeat = Telescope_Unpark + 1;
int constexpr Telescope_Azimuth_to_Controller_Heartbeat = Telescope_Altitude_to_Controller_Heartbeat + 1;
int constexpr Telescope_Get_Year_Month = Telescope_Azimuth_to_Controller_Heartbeat + 1;
int constexpr Telescope_Get_Day_Hour = Telescope_Get_Year_Month + 1;
int constexpr Telescope_Get_Minute_Second = Telescope_Get_Day_Hour + 1;
int constexpr Telescope_End = Telescope_Get_Minute_Second + 1;
// Error Responses ------------------------------------------------------------------------------------------
int constexpr Telescope_Command_Error_Response = 200;
int constexpr Focuser_Command_Error_Response = 201;
// source definitions: --------------------------------------------------------------------------------------------------------------
int constexpr SOURCE_CONTROLLER = 0;
int constexpr SOURCE_ALTITUDE_MOTOR = 1;
int constexpr SOURCE_AZIMUTH_MOTOR = 2;
int constexpr SOURCE_FOCUSER = 3;
int constexpr SOURCE_TELESCOPE_MASTER = 4;
int constexpr SOURCE_HUB = 5;
int constexpr SOURCE_TERTIARY_MOTOR = 6;

#endif	/* HUB_COMMANDS_H */

