#ifdef REST
// Expose variables to rest --------------------------------------------------------------------------------------------------------------
  {
  rest.set_id("001");                 // Give ID to the device
  rest.set_name("Stephen&Jamie");
  rest.variable("Altitude", &Altitude_value);										// v1
  rest.variable("AtHome", &AtHome_value);											// v2
  rest.variable("AtPark", &AtPark_value);											// v3
  rest.variable("Azimuth", &Current_Azimuth);											// v4
  rest.variable("Slewing", &Slewing_value);											// v5
  rest.variable("TargetAltitude", &TargetAltitude_value);							// v6
  rest.variable("TargetAzimuth", &TargetCurrent_Azimuth);								// v7
  rest.variable("Tracking", &Tracking_value);										// v8
  rest.variable("TrackingRateAzimuth", &TrackingRateCurrent_Azimuth);					// v9
  rest.variable("TrackingRateAltitude", &TrackingRateAltitude_value);				// v10
  rest.variable("HubDisplayStatus", &hub_display_state);							// v11
  //rest.variable("FocuserDisplayStatus", &Focuser_display_state);					// v12
  //rest.variable("FocuserCurrentPosition", &Focuser_Current_Position_value);			// fv1
  //rest.variable("FocuserCurrentTemperature", &Focuser_Current_Temperature_value);	// fv2
  //rest.variable("FocuserCurrentHumidity", &Focuser_Current_Humidity_value);			// fv3
  //rest.variable("FocuserLatitude", &Focuser_Latitude_value);						// fv4
  //rest.variable("FocuserLongitude", &Focuser_Longitude_value);						// fv5
  //rest.variable("FocuserAltitude", &Focuser_Altitude_value);						// fv6
  //rest.variable("FocuserYear", &Focuser_Year_value);								// fv7
  //rest.variable("FocuserMonth", &Focuser_Month_value);								// fv8
  //rest.variable("FocuserDay", &Focuser_Day_value);									// fv9
  //rest.variable("FocuserHour", &Focuser_Hour_value);								// fv10
  //rest.variable("FocuserMinute", &Focuser_Minute_value);							// fv11
  //rest.variable("FocuserSecond", &Focuser_Second_value);							// fv12
  }
  // ASCOM functions exposed to rest
  {
	  rest.function("TargetAltitude", TargetAltitude_f);							// fa2
	  rest.function("TargetAzimuth", TargetAzimuth_f);							// fa3
	  rest.function("Tracking", Tracking_f);										// fa4
	  rest.function("TrackingRates", TrackingRates_f);							// fa5
	  rest.function("AbortSlew", AbortSlew_f);									// fa6
	  rest.function("FindHome", FindHome_f);										// fa7
	  rest.function("Park", Park_f);												// fa8
	  rest.function("SlewToAltAz", SlewToAltAz_f);								// fa9
	  rest.function("SlewToTarget", SlewToTarget_f);								// fa10
	  rest.function("SyncToAltAz", SyncToAltAz_f);								// fa11
	  rest.function("PulseGuide", PulseGuide_f);									// fa12
	  rest.function("GuideRateRightAscension", GuideRateRightAscension_f);		// fa13
	  rest.function("GuideRateDeclination", GuideRateDeclination_f);				// fa14
	  rest.function("UnPark", UnPark_f);											// fa15
	  rest.function("HubDisplay", Hub_Display_f);							// fa16
//	  rest.function("FocuserHome", Focuser_Home_f);								// ff1
//	  rest.function("FocuserMove", Focuser_Move_f);								// ff2
//	  rest.function("FocuserMove_to", Focuser_Move_to_f);							// ff3
//	  rest.function("FocuserDisplay_On", Focuser_Display_On_f);					// ff4
//	  rest.function("FocuserDisplay_Off", Focuser_Display_Off_f);					// ff5
//	  rest.function("FocuserHalt", Focuser_Halt_f);								// ff6
  }
  //--------------------------------------------------------------------------------------------------------------------------------------
#endif