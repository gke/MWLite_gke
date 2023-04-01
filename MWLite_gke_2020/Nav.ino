#if defined(USE_GPS) && !defined(SPEKTRUM)


#define GPS_M 0.011131953098f

float LatE, LonE; 
int16_t navHeadingE;
int16_t navHeading;

void gpsSetHomePosition(void) {  

  wp[HOME].lat = gpsPosition[LAT];
  wp[HOME].lon = gpsPosition[LON];
  wp[HOME].alt = gpsAltitude;
  wp[HOME].loitermS = 0;
  
  gpsLonScale = 1.0f; //cos((float)(gpsPosition[LAT] / 1000000) * PI/1800.0); 

  navTakeoffHeading = Heading;

  f.GPS_FIX_HOME = true; 

} // gpsSetHomePosition

void initGPS(void) {

  //serialOpen(GPS_SERIAL_PORT, 115200);
  ubloxInitSerial(115200UL);
  f.GPS_FIX_HOME = f.GPS_ACTIVE = f.GPS_FIX = false;

} // initGPS

void updateGPS(void) {

  gpsPacketReceived = false;
  while (serialAvailable(GPS_SERIAL_PORT) && !gpsPacketReceived) 
    gpsPacketReceived = ubloxReceivePacket(serialRead(GPS_SERIAL_PORT));
} // updateGPS


uint32_t gpsRelPosition(void) { 

  if (f.GPS_FIX_HOME) {
    LatE = wp[HOME].lat - gpsPosition[LAT];
    LonE = wp[HOME].lon - gpsPosition[LON];
    homeDistance = sqrt(sq(LatE) + sq(LonE)) * GPS_M;
    homeHeading = atan2(LonE, LatE) * RAD_DEG10; // ~196uS on 32u4
  }
  else LatE = LonE = homeDistance = homeHeading = 0;
} // gpsRelPosition

void Navigate(int32_t desiredLat, int32_t desiredLon) {
  float navDistance;

  LatE = desiredLat - gpsPosition[LAT];
  LonE = desiredLon - gpsPosition[LON];
  navHeading = atan2(LonE, LatE) * RAD_DEG10;
  navDistance = sqrt(sq(LatE) + sq(LonE)) * GPS_M;
  navHeadingE = minimumTurnDeg10(navHeading - gpsGroundCourse);

#if defined(MULTICOPTER)

  // go minimal and turn to home and pitch down - velocity control?
  navCorr[YAW] = constrain(navHeadingE / 20, 0, 20);  
  navCorr[PITCH] = constrain(navDistance * 5, 0, 100);

#else 
  navCorr[ROLL] = Limit1(((int32_t)navHeadingE * conf.pid[POSITION].Kp) / 128, MAX_NAV_BANK_ANGLE);
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
        Navigate(wp[HOME].lat, wp[HOME].lon);
      else {
        if (f.GPS_HOLD_MODE) {
          if (newHold) {
            wp[HOLD].lat = gpsPosition[LAT];
            wp[HOLD].lon = gpsPosition[LON];
            newHold = false;
          }
          Navigate(wp[HOLD].lat, wp[HOLD].lon);
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













