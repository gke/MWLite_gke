#if defined(USE_GPS) && !defined(SPEKTRUM)

#define RAD_DEG10 (1800.0/PI)
#define RAD_DEG (180.0/PI)

#define GPS_M 0.011131953098f

float LatE, LonE; 
int16_t navHeadingE;
int16_t navHeading;

void gpsSetHomePosition(void) {  

  gpsHome[LAT] = gpsPosition[LAT];
  gpsHome[LON] = gpsPosition[LON];

  gpsHomeAltitude = gpsAltitude;
  gpsLonScale = 1.0f; //cos((float)(gpsPosition[LAT] / 1000000) * PI/1800.0); 

  navTakeoffHeading = Heading;

  f.GPS_FIX_HOME = true; 

} // gpsSetHomePosition

void initGPS(void) {

  serialOpen(GPS_SERIAL_PORT, 115200);
  //ubloxInitSerial();
  f.GPS_FIX_HOME = f.GPS_ACTIVE = f.GPS_FIX = false;

#if defined(FAKE_GPS)
  gpsPosition[LON] = FAKE_LON;
  gpsPosition[LAT] = FAKE_LAT;
#endif

} // initGPS

void updateGPS(void) {

  gpsPacketReceived = false;
  while (serialAvailable(GPS_SERIAL_PORT) && !gpsPacketReceived) 
    gpsPacketReceived = ubloxReceivePacket(serialRead(GPS_SERIAL_PORT));

} // updateGPS


uint32_t gpsRelPosition(void) { // 270uS

  if (f.GPS_FIX_HOME) {
    LatE = gpsHome[LAT] - gpsPosition[LAT];
    LonE = gpsHome[LON] - gpsPosition[LON];
    homeDistance = sqrt(LatE*LatE + LonE*LonE) * GPS_M;
    homeHeading = atan2(LonE, LatE) * RAD_DEG10; // ~196uS on 32u4
  }
  else LatE = LonE = homeDistance = homeHeading = 0;
} // gpsRelPosition

void Navigate(int32_t desiredLat, int32_t desiredLon) {

  LatE = desiredLat - gpsPosition[LAT];
  LonE = desiredLon - gpsPosition[LON];
  navHeading = atan2(LonE, LatE) * RAD_DEG10;

#if defined(MULTICOPTER)

  // go minimal and turn to home and pitch down - velocity control?
  navCorr[ROLL] = navCorr[PITCH] = navCorr[YAW] = 0;

#else
  navHeadingE = minimumTurnDeg10(navHeading - gpsGroundCourse);
  navCorr[ROLL] = ((int32_t)navHeadingE * conf.P8[PIDPOS]) / 256; 
#endif

} // Navigate

void updateNav(void) {
  static uint32_t lastUpdatemS = millis();
  static uint32_t gpsTimeout = millis() + 2000;

  updateGPS();

  if (gpsPacketReceived && f.GPS_FIX && (gpsSats >= 5) ) {
    gpsTimeout = millis() + 2000;
    if (f.ARMED) {    
      if (f.GPS_HOME_MODE)
        Navigate(gpsHome[LAT], gpsHome[LON]);
      else {
        if (f.GPS_HOLD_MODE) {
          if (newHold) {
            gpsHold[LAT] = gpsPosition[LAT];
            gpsHold[LON] = gpsPosition[LON];
            newHold = false;
          }
          Navigate(gpsHold[LAT], gpsHold[LON]);
        } 
        else 
          navCorr[ROLL] = navCorr[PITCH] = navCorr[YAW] = 0; // maybe add decay? 
      }       
    } 
    else {
      LED_BLUE_TOGGLE;
      gpsSetHomePosition();
    }
  }
  else
    if (millis() > gpsTimeout) {
      f.GPS_ACTIVE = f.GPS_FIX = false;
      navCorr[ROLL] = navCorr[PITCH] = navCorr[YAW] = 0; // maybe add decay? 
    }

#if defined(DEBUG_NAV)
  if (gpsPacketReceived) {
    debug[0] = navCorr[ROLL] / 10;
    debug[1] = navCorr[PITCH] / 10;
    debug[2] = navHeadingE / 10;
    debug[3] = millis() - lastUpdatemS;
    lastUpdatemS = millis();
  }  
#endif

  gpsPacketReceived = false;

} // updateNav

#else

void initGPS(void) {
  f.GPS_FIX_HOME = f.GPS_ACTIVE = false;
  navCorr[ROLL] = navCorr[PITCH] = navCorr[YAW] = 0;
} // initGPS

void updateNav(void) {
} // updateNav;

#endif












