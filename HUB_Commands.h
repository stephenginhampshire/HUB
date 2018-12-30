/* 
 * File:   HUB_Commands.h
 * Author: Stephen
 *
 * Created on 14 February 2018, 13:19
 */

#ifndef HUB_COMMANDS_H
#define	HUB_COMMANDS_H

#define AZ_Home_command                 0x10        // i1   special HOME command from ALT buttob pad to AZ motor
#define AZ_Halt_command                 0x12        // i2   special HALT command from ALT button pad to AZ motor
#define Halt_command                    0x14        // i3   Motor emergency halt
#define Azimuth_Display_command         0x16        // i4   packet containing Azimuth Bearing 
#define Motor_update_command            0x05        // c1
#define TargetAltitude_command          0xA0        // c2
#define TargetAzimuth_command           0xA2        // c3
#define Tracking_command				0xA4        // c4
#define TrackingRates_command           0xA6        // c5
#define AbortSlew_command				0xB0        // c6
#define FindHome_command				0xB2        // c7
#define Park_command					0xB4        // c8
#define SlewToAltAz_command				0xC0        // c9
#define SlewToTarget_command			0xC8        // c10
#define SyncToAltAz_command				0xD0        // c11
#define PulseGuide_command              0xE0        // c12
#define GuideRateRightAscension_command 0xE2        // c13
#define GuideRateDeclination_command    0xE4        // c14
#define UnPark_command					0xB6        // c15

#define Hub_Display_On_command			0x30
#define Hub_Display_Off_command			0x32

#define Focuser_Update_command			0x05		// fc1
#define Focuser_Home_command			0x10		// fc2
#define Focuser_Move_command			0x20		// fc3
#define Focuser_Move_to_command			0x25		// fc4
#define Focuser_Display_On_command		0x30		// fc5
#define Focuser_Display_Off_command		0x35		// fc6
#define Focuser_Halt_command			0xFF		// fc7

#endif	/* HUB_COMMANDS_H */

