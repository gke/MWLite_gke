
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

// MW control routines tidied up by Prof Greg Egan aka gke 2013


#if defined(USE_MW)

int16_t Threshold(int16_t v, int16_t t) {

  if (v > t)
    v -= t;
  else if (v < -t)
    v += t;
  else
    v = 0;

  return (v);
} // Threshold

#define RATE_F_CUT   17.0f
#define RATE_TC      ((int32_t)(1000000 / ( 2.0 * PI * RATE_F_CUT))) 
#define RATE_MAX_I 256

int32_t inline SmoothRateDerivative(uint8_t axis, int32_t Rate) {
  // simple 3 sample moving average filter
  int32_t d, deltaSum; 
  static int32_t deltaSumP[3] = {
    0
  };
  static int32_t Ratep[3] = {
    0
  };
  static int32_t delta[2][3] = {
    0
  };

  d = Rate - Ratep[axis];
  Ratep[axis] = Rate;

  d = (d *(4194304L / dTuS)) >> 8; 

  deltaSum = delta[0][axis] + delta[1][axis] + d;
  delta[1][axis] = delta[0][axis];
  delta[0][axis] = d;

  /*
#if defined(USE_RATE_DELTA_LPF)
   //deltaSum = deltaSumP[axis] + (dTuS / (RC + dTuS)) * (deltaSum - deltaSumP[axis]);
   deltaSum = deltaSumP[axis] + ((int32_t)(deltaSum - deltaSumP[axis]) * CYCLETIME)/(RATE_TC + CYCLETIME);  
   deltaSumP[axis] = deltaSum;
   #endif
   */

  return (deltaSum);
} // SmoothRateDerivative  

void inline doRCRates(void) {
  int16_t Temp;
  uint8_t index, chan, axis;
  int16_t prop1, prop2;

  prop2 = 128; 
  if (rcData[THROTTLE] > 1500) // breakpoint is fix: 1500
      if (rcData[THROTTLE] < 2000) 
      prop2 -=  ((uint16_t)conf.dynThrPID * (rcData[THROTTLE] - 1500) >> 9); //  /512 instead of /500
    else 
      prop2 -=  conf.dynThrPID;

  for(axis = 0; axis < 3; axis++) {

    Temp = min(abs(rcData[axis] - MIDRC),500);
#if defined(DEADBAND)
    Temp = (Temp > DEADBAND) ? Temp - DEADBAND : 0; 
#endif

    if(axis == YAW) 
      rcCommand[axis] = Temp;
    else { 
      index = Temp >> 7; // 500/128 = 3.9  => range [0;3]
      rcCommand[axis] = rollpitchCurve[index] + ((Temp-(index << 7)) * (rollpitchCurve[index+1]-rollpitchCurve[index]) >> 7);
      prop1 = 128 - ((uint16_t)conf.rollPitchRate*Temp >> 9);
      prop1 = (uint16_t)prop1 * prop2 >> 7;
      dyn.P8[axis] = (uint16_t)conf.P8[axis] * prop1 >> 7; 
      dyn.D8[axis] = (uint16_t)conf.D8[axis] * prop1 >> 7;
    }
    if (rcData[axis] < MIDRC) rcCommand[axis] = -rcCommand[axis];
  }

  for (chan = YAW + 1; chan < RC_CHANS; chan++)
    if (chan != THROTTLE)
      rcCommand[chan] = Limit1(rcData[chan] - MIDRC, 500); 

  Temp = constrain(rcData[THROTTLE],MIN_CHECK,2000);
  Temp = (uint32_t)(Temp - MIN_CHECK) * 2559 / (2000 - MIN_CHECK); // [MINCHECK;2000] -> [0;2559]
  index = Temp/256; // range [0;9]
  rcCommand[THROTTLE] = thrCurve[index] + (Temp - index*256) * (thrCurve[index+1] - thrCurve[index]) / 256; // [0;2559] -> expo -> [conf.minthrottle;MAXTHROTTLE]

  rcCommand[THROTTLE] = ((((int32_t)(rcCommand[THROTTLE] - MIN_COMMAND)) * throttleLVCScale) >> 10) + MIN_COMMAND;

} // DoRCRates


#if defined(USE_MW_ALEXK_CONTROL)

void computeControl(void) { // Alex Khoroshko  http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671&start=10#p37387
  uint8_t axis;
  int32_t prop, RateP, RateI, RateD, DesiredRate, RateE, AngleE;

  prop = min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 500); // range [0;500]

  for(axis = ROLL; axis <= YAW; axis++) {

    if ((f.ANGLE_MODE || f.HORIZON_MODE) && (axis != YAW) )
      AngleE = (rcCommand[axis] * STICK_TO_ANGLE) - angle[axis];

    if (axis == YAW)
      DesiredRate = (((int32_t) (conf.yawRate + 27) * rcCommand[YAW]) >> 5);
    else 
      if (f.ANGLE_MODE) 
      DesiredRate = ((int32_t) AngleE * conf.P8[PIDLEVEL]) >> 4;
    else {
      DesiredRate = ((int32_t) (conf.rollPitchRate + 27) * rcCommand[axis]) >> 4;
      if (f.HORIZON_MODE)
        DesiredRate += ((int32_t) AngleE * conf.I8[PIDLEVEL]) >> 8;
    } 

    RateE = DesiredRate - (gyroData[axis] >> 2);

    RateP = ((int32_t) RateE * conf.P8[axis]) >> 7;

    RateD = (SmoothRateDerivative(axis, RateE) * conf.D8[axis]) >> 8;

    axisPID[axis] =  RateP - RateD;

  }
} // computeControl

#elif defined (USE_MW_LEGACY_CONTROL)

void computeControl(void) {
  uint8_t axis;
  int16_t P, I, D, RateP, RateI, RateE, AngleE;
  int32_t AngleP, AngleI;
  int16_t prop;

  prop = min(max(abs(rcCommand[PITCH]),abs(rcCommand[ROLL])), 500); // range [0;500]

  for(axis = ROLL; axis <= YAW; axis++) {

    if ((f.ANGLE_MODE || f.HORIZON_MODE) && (axis != YAW) ) { 

      AngleE = Limit1(rcCommand[axis] * STICK_TO_ANGLE, 500) - angle[axis];

      AngleP = ((int32_t)AngleE * conf.P8[PIDLEVEL]) >> 7; 
      AngleP = Limit1(AngleP, conf.D8[PIDLEVEL] * 5);

      AngleIntE[axis] = Limit1(AngleIntE[axis] + AngleE, 10000); 
      AngleI = ((int32_t)AngleIntE[axis] * conf.I8[PIDLEVEL]) >> 12; 
    }
    if ( !f.ANGLE_MODE || f.HORIZON_MODE || (axis == YAW) ) { 

      RateE = (((int32_t)rcCommand[axis] << 6) / conf.P8[axis]) - (gyroData[axis] >> 2); 

      RateP = rcCommand[axis];

      RateIntE[axis] =  (abs(gyroData[axis]) > 2260) ? 0 : Limit1(RateIntE[axis] + RateE, 16000);    
      RateI = ((RateIntE[axis] >> 7) * conf.I8[axis]) >> 6;     
    }
    if ( f.HORIZON_MODE && (axis != YAW)) {
      P = ((int32_t)AngleP * (512 - prop) + (int32_t)RateP * prop) >> 9; 
      I = ((int32_t)AngleI * (512 - prop) + (int32_t)RateI * prop) >> 9;
    } 
    else {
      if ( f.ANGLE_MODE && (axis != YAW)) {
        P = AngleP;
        I = AngleI;
      } 
      else {
        P = RateP;
        I = RateI;
      }
    }

    P -= ((int32_t)gyroData[axis] * dyn.P8[axis]) >> 8; // 32 bits is needed for calculation   

    D = (SmoothRateDerivative(axis, gyroData[axis]) * dyn.D8[axis]) >> 7;

    axisPID[axis] =  P + I - D;
  }

} // MWControl

#elif defined(USE_MW_2_3_CONTROL)

#define RATE_MAX_P 300
#define RATE_MAX_I 250

void computeControl(void) {
  static int16_t gyroDataP[3] = {
    0
  };
  uint8_t axis;
  int16_t P, D, RateE, AngleP, AngleE;
  int32_t I, AngleI;
  int16_t prop, rc;

  prop = f.HORIZON_MODE ? min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 512) : 0;

  for(axis = ROLL; axis < YAW; axis++) {

    rc = Limit1(rcCommand[axis] * STICK_TO_ANGLE, MAX_BANK_ANGLE);

    P = ((int32_t)rc*conf.P8[axis]) >> 6;

    if ( abs(gyroData[axis]) > 2260 )
      RateIntE[axis] = I = 0;
    else {
      RateE = rc - (gyroData[YAW] >> 2);
      RateIntE[axis] = Limit1(RateIntE[axis] + RateE, 16000);
      I = ((RateIntE[axis] >> 7) * conf.I8[axis]) >> 6;
    }

    if (f.ANGLE_MODE || f.HORIZON_MODE) {

      AngleE = rc - angle[axis];

      AngleP = ((int32_t)AngleE * conf.P8[PIDLEVEL]) >> 7; 
      AngleP = Limit1(AngleP, conf.D8[PIDLEVEL] * 5);

      AngleIntE[axis] = Limit1(AngleIntE[axis] + AngleE, 10000); 
      AngleI = ((int32_t)AngleIntE[axis] * conf.I8[PIDLEVEL]) >> 12;  

      if (f.HORIZON_MODE) {
        P = AngleP + (((P - AngleP) * prop) >> 9);
        I = AngleI + (((I - AngleI) * prop) >> 9);
      } 
      else
      {
        P = AngleP;
        I = AngleI;
      }
    }

    P -= ((int32_t)gyroData[axis] * dyn.P8[axis]) >> 8; 

    D = (SmoothRateDerivative(axis, gyroData[axis]) * dyn.D8[axis]) >> 7;

    axisPID[axis] =  P + I - D;
  }

  // yaw
  rc = ((int32_t)rcCommand[YAW] * (conf.yawRate * 2 + 30)) >> 5;

  P = (int32_t)RateE * conf.P8[YAW] >> 6;
  P = Limit1(P, RATE_MAX_P - conf.D8[YAW]);

  RateIntE[YAW]  += (int32_t)RateE * conf.I8[YAW];
  RateIntE[YAW]  = (abs(rc) > 50) ? 0 :
  Limit1(RateIntE[YAW], -2 + ((int32_t)1 << 28));
  I = Limit1((int16_t)(RateIntE[YAW] >> 13), RATE_MAX_I);

  axisPID[YAW] = P + I;

} // computeControl

#else

#error "Control scheme not specified"

#endif

#endif // USE_MW_XXX
























