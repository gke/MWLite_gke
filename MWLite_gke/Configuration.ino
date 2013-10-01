/*

 MWLite_gke
 May 2013
 
 MWLite_gke is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 
 MWLite_gke is distributed in the hope that it will be useful,but WITHOUT ANY 
 WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR 
 A PARTICULAR PURPOSE. 
 
 See the GNU General Public License for more details.
 
 Lite was based originally on MultiWiiCopter V2.2 by Alexandre Dubus
 www.multiwii.com, March  2013. The rewrite by Prof Greg Egan was renamed 
 so as not to confuse it with the original.
 
 It preserves the parameter specification and GUI interface with parameters
 scaled to familiar values. 
 
 Major changes include the control core which comes from UAVX with the 
 addition of MW modes.
 
 Lite supports only Atmel 32u4 processors using an MPU6050 and optionally 
 BMP085 and MS5611 barometers and HMC5883 magnetometer with 4 motors, 
 no servos and 8KHz PWM for brushed DC motors.
 
 */
// Stick Programming - arming, accelerometer trim etc.


void blinkLED(uint8_t t, uint8_t repeat) { 
  uint8_t r;

  if ((rcCommand[THROTTLE] < MINTHROTTLE) && !f.ARMED) 
    for (r = 0; r < repeat; r++) {
        LED_BLUE_TOGGLE; 
        delay(t);
        LED_BLUE_TOGGLE;
      }
} // blinkLED


void doArm(void) {

  calibratingG = 512;
  while ( calibratingG !=0 ) 
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
    throttleLVCScale = 1024;
    velZ = 0.0f;
    if (f.MAG_ACTIVE && f.MAG_CALIBRATED && f.HEAD_HOLD_MODE) 
      holdHeading = angle[YAW];
    else 
      YawAngle = 0.0f;
    zeroIntegrals();
  }
  else
    blinkLED(200, 1);
  
} // doArm

void doDisarm(void) {
  if (f.ARMED)
    LED_BLUE_OFF;
  f.ARMED = false; 
} // doDisarm

void doStickArming(uint8_t rcSticks, bool stickArm) {

  if ( conf.activate[BOX_ARM] == 0 ) {
    if (stickArm) {
#if defined(ALLOW_ARM_DISARM_VIA_TX_YAW)
      if (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) doArm(); 
#endif
#if defined(ALLOW_ARM_DISARM_VIA_TX_ROLL)
      if (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_HI) doArm(); 
#endif        
    } 
    else {
#if defined(ALLOW_ARM_DISARM_VIA_TX_YAW)
      if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) doDisarm();
#endif
#if defined(ALLOW_ARM_DISARM_VIA_TX_ROLL)
      if (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_LO) doDisarm(); 
#endif
    }
  } 
} // doStickArming


void doConfigUpdate(void) {
  static uint8_t rcDelayCommand = 0; 
  static uint8_t rcSticks; 
  bool updateglobal;
  uint8_t newSticks = 0;
  uint16_t auxState = 0;
  uint8_t i;

  for(i = 0; i < 4; i++) {
    newSticks >>= 2;
    if(rcData[i] > MINCHECK) 
      newSticks |= 0x80; // check for MIN
    if(rcData[i] < MAXCHECK) 
      newSticks |= 0x40; // check for MAX
  }

  if(newSticks == rcSticks) {
    if(rcDelayCommand < 250) 
      rcDelayCommand++;
  }
  else 
    rcDelayCommand = 0;
  rcSticks = newSticks;

  // perform actions    
  if (rcData[THROTTLE] <= MINCHECK) { 
    zeroIntegrals();
    if (conf.activate[BOX_ARM] > 0) // Arming via ARM BOX_
      if ( rcOptions[BOX_ARM] && f.OK_TO_ARM && !f.ARMED ) 
        doArm(); 
      else 
        if (f.ARMED) 
        doDisarm();
  }

  if(rcDelayCommand == 20) {
    if( f.ARMED) 
      doStickArming(rcSticks, false);
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
          //case THR_LO | YAW_HI | PIT_HI | ROL_CE: break; // Enter LCD config
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

  for(i = 0; i < 4; i++)
    auxState |= (rcData[AUX1+i]<1300)<<(3*i) | (1300<rcData[AUX1+i] && rcData[AUX1+i]<1700)<<(3*i+1) | (rcData[AUX1+i]>1700)<<(3*i+2);

  for(i = 0; i < CHECKBOX_ITEMS; i++)
    rcOptions[i] = (auxState & conf.activate[i]) > 0;

  if ( rcOptions[BOX_ANGLE] || inFailsafe ) { // bumpless transfer to Level mode
    if (!f.ANGLE_MODE)
      zeroIntegrals();
    f.ANGLE_MODE = true; 
  } 
  else 
    f.ANGLE_MODE = false;

  f.ENABLE_ACRO_TRAINER = rcOptions[BOX_ACRO_TRAINER];

  if ( rcOptions[BOX_HORIZON] ) {
    f.ANGLE_MODE = false;
    if (!f.HORIZON_MODE) {
      zeroIntegrals();
      f.HORIZON_MODE = true;
    }
  } 
  else 
    f.HORIZON_MODE = false;

  if ((rcCommand[THROTTLE] > 1200) && rcOptions[BOX_ALT_HOLD] && (f.BARO_ACTIVE || f.SONAR_ACTIVE)) {
    if (!f.ALT_HOLD_MODE ) {
      desiredAltitude = relativeAltitude;
      hoverThrottle = rcCommand[THROTTLE];
      AltitudeIntE = 0;
      ROC = 0;
      altPID = 0;
    }
    f.ALT_HOLD_MODE = true;
  } 
  else 
    f.ALT_HOLD_MODE = false;

  if (!rcOptions[BOX_ARM]) 
    f.OK_TO_ARM = true; 

} // doConfigUpdate

























