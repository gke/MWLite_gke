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
 
// Heading

void doHeadFree(void) {
  float relHeading, cosA, sinA;
  int16_t Temp;

  if (rcOptions[BOX_HEAD_FREE])
    if (!f.HEAD_FREE_MODE) headfreeHeading = angle[YAW];

  f.HEAD_FREE_MODE = rcOptions[BOX_HEAD_FREE];

  if(f.HEAD_FREE_MODE) { //to optimize
    relHeading = (angle[YAW] - headfreeHeading) * (PI/180.0);
    cosA = cos(relHeading);
    sinA = sin(relHeading);
    Temp = rcCommand[PITCH] * cosA + rcCommand[ROLL] * sinA;
    rcCommand[ROLL] =  rcCommand[ROLL] * cosA - rcCommand[PITCH] * sinA; 
    rcCommand[PITCH] = Temp;
  }
} // doHeadFree

void doMagHold(void) {
#if defined(USE_MAG)
  int16_t HE;

  if (f.ANGLE_MODE && rcOptions[BOX_MAG] && f.MAG_ACTIVE && f.MAG_CALIBRATED && f.SMALL_ANGLE_25DEG && (abs(rcCommand[YAW]) < 70)) {

    f.MAG_MODE = true;

    HE = angle[YAW] - holdHeading;
    if (HE > 180) 
      HE -= 360; 
    else
      if (HE <= -180) 
        HE += 360;

    rcCommand[YAW] -= (HE * conf.P8[PIDMAG]) >> 5;
  }
  else {
    f.MAG_MODE = false;
    HE = 0; 
    holdHeading = angle[YAW];
  }

#if defined(DEBUG_HEAD_HOLD)
  debug[0] = angle[YAW];
  debug[1] = holdHeading;
  debug[2] = HE;
  debug[3] = rcCommand[YAW];
#endif

#endif // USE_MAG
} // doMagHold


























