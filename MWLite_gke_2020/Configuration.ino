
// Stick Programming - arming, accelerometer trim etc.


int16_t Threshold(int16_t v, int16_t t) {

  return (abs(v) >= t ? v : 0);
} // Threshold

void blinkLED(uint8_t t, uint8_t repeat) {

  uint8_t r;

  if ((rcCommand[THROTTLE] < MIN_THR_US) && !f.ARMED)
    for (r = 0; r < repeat; r++) {
      LED_BLUE_TOGGLE;
      delay(t);
      LED_BLUE_TOGGLE;
    }

} // blinkLED


void zeroIntegrals(void) {
  memset(&RateIntE, 0, sizeof(RateIntE));
  memset(&AngleIntE, 0, sizeof(AngleIntE));
} // zeroIntegrals

inline void zeroAngles(void) {
  angle[ROLL] = angle[PITCH] = angle[YAW] = 0;
} // zeroAngles

void doArm(void) {

  calibratingG = 512;
  while ( calibratingG != 0 )
    getRatesAndAccelerations();

  if (f.BARO_ACTIVE) {
    calibratingB = 32;
    while ( calibratingB > 0 ) {
      slotFree = true;
      updateAltitude();
    }
  }

  f.ARMED = f.ACC_CALIBRATED && !f.ALARM;

  if ( f.ARMED ) {
    velZ = 0.0f;
    if (f.MAG_ACTIVE && f.MAG_CALIBRATED && f.HEAD_HOLD_MODE)
      holdHeading = angle[YAW];
    else
      yawAngle = 0.0f;
    zeroIntegrals();
  }
#if defined(USE_GPS) && !defined(SPEKTRUM)
  else
    blinkLED(200, 1);
#endif

} // doArm

void doDisarm(void) {

  if (f.ARMED) {
    LED_BLUE_OFF;
    f.ARMED = false;
#if defined(GPS_RESET_HOME_ON_DISARM)
    f.GPS_FIX_HOME = false;
#endif
  }
} // doDisarm

void doStickArming(uint8_t rcSticks, boolean stickArm) {

  if ( conf.activate[BOX_ARM] == 0 ) {
    if (stickArm) {
#if defined(ALLOW_ARM_DISARM_VIA_TX_YAW)
      if (rcSticks == (THR_LO | YAW_HI | PIT_CE | ROL_CE)) doArm();
      else if (rcSticks == (THR_LO | YAW_LO | PIT_CE | ROL_CE)) doDisarm();
#elif defined(ALLOW_ARM_DISARM_VIA_TX_ROLL)
      if (rcSticks == (THR_LO | YAW_CE | PIT_CE | ROL_HI)) doArm();
      else if (rcSticks == (THR_LO | YAW_CE | PIT_CE | ROL_LO)) doDisarm();
#endif
    }
  }
} // doStickArming

void doConfigUpdate(void) {
  static uint8_t rcDelayCommand = 0;
  static uint8_t rcSticks;
  boolean updateglobal;
  uint8_t newSticks = 0;
  uint16_t auxState = 0;
  uint8_t i;

  for (i = 0; i < 4; i++) {
    newSticks >>= 2;
    if (rcData[i] > MIN_CHECK)
      newSticks |= 0x80; // check for MIN
    if (rcData[i] < MAX_CHECK)
      newSticks |= 0x40; // check for MAX
  }

  if (newSticks == rcSticks) {
    if (rcDelayCommand < 250)
      rcDelayCommand++;
  }
  else
    rcDelayCommand = 0;
  rcSticks = newSticks;

  // perform actions
  if (rcData[THROTTLE] <= MIN_CHECK) {
    zeroIntegrals();
    if (conf.activate[BOX_ARM] > 0) // Arming via ARM BOX_
      if ( rcOptions[BOX_ARM] && f.OK_TO_ARM && !f.ARMED )
        doArm();
      else if (f.ARMED)
        doDisarm();
  }

if (rcDelayCommand == 20) 
  {
    if (f.ARMED)
      doStickArming(rcSticks, true); //false);
    else {

      doStickArming(rcSticks, true);

      if (!f.ARMED) {

        switch (rcSticks) {
          //case THR_HI + YAW_HI + PIT_LO + ROL_CE: f.CALIBRATE_MAG = true; break;
          case THR_HI | YAW_LO | PIT_LO | ROL_CE:
           calibratingA = 512;
            break;
          case THR_LO | YAW_LO | PIT_LO | ROL_CE:
            calibratingG = 512;
            break;
          default:
            break;
        } // switch

        updateglobal = true;
        switch (rcSticks) {
          case THR_HI | YAW_CE | PIT_HI | ROL_CE:
            global_conf.accZero[PITCH] += ACC_TRIM_STEP_SIZE;
            break;
          case THR_HI | YAW_CE | PIT_LO | ROL_CE:
            global_conf.accZero[PITCH] -= ACC_TRIM_STEP_SIZE;
            break;
          case THR_HI | YAW_CE | PIT_CE | ROL_HI:
            global_conf.accZero[ROLL] += ACC_TRIM_STEP_SIZE;
            break;
          case THR_HI | YAW_CE | PIT_CE | ROL_LO:
            global_conf.accZero[ROLL] -= ACC_TRIM_STEP_SIZE;
            break;
#if !defined(MULTICOPTER)
          case THR_LO | YAW_HI | PIT_HI | ROL_CE:
            setServoTrims();
            break;
#endif
          default:
            updateglobal = false;
            break;
        } // switch

        if ( updateglobal ) {
          LED_BLUE_TOGGLE;
          writeGlobalSet(0);
          rcDelayCommand = 0; // allow autorepetition
        }
      }
    }
  }

  for (i = 0; i < 4; i++)
    auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);

  for (i = 0; i < CHECKBOX_ITEMS; i++)
    rcOptions[i] = (auxState & conf.activate[i]) > 0;

  if ( rcOptions[BOX_ANGLE] || inFailsafe ) { // bumpless transfer to Level mode
    if (!f.ANGLE_MODE)
      zeroIntegrals();
    f.ANGLE_MODE = true;
  }
  else
    f.ANGLE_MODE = false;

#if defined(USE_MW)
  if ( rcOptions[BOX_HORIZON] ) {
    f.ANGLE_MODE = false;
    if (!f.HORIZON_MODE) {
      zeroIntegrals();
      f.HORIZON_MODE = true;
    }
  }
  else
    f.HORIZON_MODE = false;
#endif

#if !defined(MULTICOPTER)
  f.BYPASS_MODE = rcOptions[BOX_BYPASS];
#endif

  f.EXP = rcOptions[BOX_EXP];

#if defined(USE_ALT)
  if ((rcCommand[THROTTLE] > 1200) && rcOptions[BOX_ALT_HOLD] && (f.BARO_ACTIVE || f.SONAR_ACTIVE || f.GPS_FIX_HOME)) {
    if (!f.ALT_HOLD_MODE ) {
      desiredAltitude = relativeAltitude;
      hoverThrottle = rcCommand[THROTTLE];
      AltitudeIntE = ROC = altPID = 0;
    }
    f.ALT_HOLD_MODE = true;
  }
  else
    f.ALT_HOLD_MODE = false;
#endif

#if defined(USE_GPS)

  if (f.ARMED) {
    if (rcOptions[BOX_GPS_HOME]) {
      f.GPS_HOLD_MODE = false;
      f.GPS_HOME_MODE = f.GPS_FIX_HOME;
    }
    else if (rcOptions[BOX_GPS_HOLD]) {
      f.GPS_HOME_MODE = false;
      if (!f.GPS_HOLD_MODE) {
        newHold = true;
        f.GPS_HOLD_MODE = true;
      }
    }
    else
      f.GPS_HOME_MODE = f.GPS_HOLD_MODE = false;
  }
  else
    f.GPS_HOME_MODE = f.GPS_HOLD_MODE = false;

#endif

  if (!rcOptions[BOX_ARM])
    f.OK_TO_ARM = true;

} // doConfigUpdate
