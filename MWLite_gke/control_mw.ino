
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

static int maxRollPitchStick = 0;

int16_t Threshold(int16_t v, int16_t t) {

  if (v > t)
    v -= t;
  else if (v < -t)
    v += t;
  else
    v = 0;

  return (v);
} // Threshold

void zeroIntegrals(void) {
 memset(&RateIntE, 0, sizeof(RateIntE));
 memset(&AngleIntE, 0, sizeof(AngleIntE));
} // zeroIntegrals

uint32_t SmoothRateDerivative(uint8_t axis, int16_t Rate) {
  // simple 3 sample moving average filter
  uint32_t d, deltaSum; 
  static int16_t Ratep[3] = {
    0,0,0                                                                                                                                             };
  static int16_t delta[2][3] = {
    {
      0,0,0    
    }
    ,{
      0,0,0 
    }
  };

  d = Rate - Ratep[axis];
  Ratep[axis] = Rate;

  deltaSum = delta[0][axis] + delta[1][axis] + d;
  delta[1][axis] = delta[0][axis];
  delta[0][axis] = d;

  return ((int32_t)deltaSum * dynD8[axis]) >> 7;
} // SmoothDerivative  

void doRCRates(void) {
  int16_t Temp;
  uint8_t index;
  uint8_t axis, dynScale;

  for(axis = 0; axis <= YAW; axis++) {

    rcCommand[axis] = Threshold(min(abs(rcData[axis] - MIDRC), 500), DEADBAND);

    dynScale =  (axis == YAW) ?
    (100 - (((uint16_t)conf.yawRate * rcCommand[YAW]) >> 9))
      : ( 100 - (((uint16_t)conf.rollPitchRate * rcCommand[axis]) >> 9));

    dynP8[axis] = ((uint16_t)conf.P8[axis] * dynScale) / 100;
    dynD8[axis] = ((uint16_t)conf.D8[axis] * dynScale) / 100;

    if (rcData[axis] < MIDRC) 
      rcCommand[axis] = -rcCommand[axis];
  }
  maxRollPitchStick =  constrain(max(abs(rcCommand[PITCH]),abs(rcCommand[ROLL])), 0, 500);
 
  #if defined(USE_THROTTLE_CURVE)
  Temp = constrain(rcData[THROTTLE], MINCHECK, 2000);
  Temp = ((uint32_t)(Temp - MINCHECK) * 1000) / (2000 - MINCHECK); 
  index = Temp / 100;
  rcCommand[THROTTLE] = thrCurve[index] + (Temp - index * 100) * (thrCurve[index+1] - thrCurve[index]) / 100; 
#else
  rcCommand[THROTTLE] = constrain(rcData[THROTTLE], 0, MAXTHROTTLE);
#endif // USE_THROTTLE_CURVE
  rcCommand[THROTTLE] = ((((int32_t)(rcCommand[THROTTLE] - MINCOMMAND)) * throttleLVCScale) >> 10) + MINCOMMAND;

} // DoRCRates

#if defined(USE_MW_EXPANDED_CONTROL)

void computeControl(void) {
  uint8_t axis;
  int16_t P, I, D, RateP, RateI, RateE, AngleE;
  int32_t AngleP, AngleI;
  int16_t AngleProp, RateProp;

  f.ACRO_TRAINER_MODE = f.ENABLE_ACRO_TRAINER && (maxRollPitchStick >= ACROTRAINER_THRESHOLD);
  if ( f.ACRO_TRAINER_MODE )
    f.ANGLE_MODE = f.HORIZON_MODE = false;

  if (f.ANGLE_MODE) 
    for(axis = ROLL; axis < YAW; axis++) {

      AngleE = Limit1(rcCommand[axis] * STICK_TO_ANGLE, 500) - angle[axis];
      AngleIntE[axis] = Limit1(AngleIntE[axis] + AngleE, 10000); 
      I = ((int32_t)AngleIntE[axis] * conf.I8[PIDLEVEL]) >> 12; 

      P = ((int32_t)AngleE * conf.P8[PIDLEVEL]) >> 7; 
      P = Limit1(P, conf.D8[PIDLEVEL] * 5);
      P -= ((int32_t)gyroData[axis] * dynP8[axis]) >> 8; // 32 bits is needed for calculation   

      D = SmoothRateDerivative(axis, gyroData[axis]); // 32 bits is needed for calculation

      axisPID[axis] =  P + I - D;   
    }
  else   
    if (f.HORIZON_MODE) {
    RateProp = maxRollPitchStick; // [0;500]
    AngleProp = 512 - RateProp;

    for(axis = ROLL; axis < YAW; axis++) {

      AngleE = Limit1(rcCommand[axis] << 1, 500) - angle[axis];

      AngleP = ((int32_t)AngleE * conf.P8[PIDLEVEL]) >> 7; 
      AngleP = Limit1(AngleP, conf.D8[PIDLEVEL] * 5);

      AngleIntE[axis] = Limit1(AngleIntE[axis] + AngleE, 10000); 
      AngleI = ((int32_t)AngleIntE[axis] * conf.I8[PIDLEVEL]) >> 12; 

      RateP = rcCommand[axis];

      RateE = (((int32_t)rcCommand[axis] << 6) / conf.P8[axis]) - (gyroData[axis]>>2); 
      RateIntE[axis] = (abs(gyroData[axis]) > 2260) ?
      0 : Limit1(RateIntE[axis] + RateE, 16000); 

      RateI = ((RateIntE[axis] >> 7) * conf.I8[axis]) >> 6;  

      P = ((int32_t)AngleP * AngleProp + (int32_t)RateP * RateProp) >> 9; 
      I = ((int32_t)AngleI * AngleProp + (int32_t)RateI * RateProp) >> 9;

      P -= ((int32_t)gyroData[axis] * dynP8[axis]) >> 8;    

      D = SmoothRateDerivative(axis, gyroData[axis]); 

      axisPID[axis] =  P + I - D;
    }
  }
  else // ACRO
  for(axis = ROLL; axis < YAW; axis++) {

    RateP = rcCommand[axis] - ((int32_t)gyroData[axis] * dynP8[axis]) >> 8; // ??? 

    RateE = (((int32_t)rcCommand[axis] << 6) / conf.P8[axis]) - gyroData[axis]; 

    RateIntE[axis] = (abs(gyroData[axis]) > 2260) ? 0:  
    Limit1(RateIntE[axis] + RateE, 16000); 
    RateI = ((RateIntE[axis] >> 7) * conf.I8[axis]) >> 6;  

    D = SmoothRateDerivative(axis, gyroData[axis]); // 32 bits is needed for calculation

    axisPID[axis] =  RateP + RateI - D;
  }

  P = rcCommand[YAW] - (((int32_t)gyroData[YAW] * dynP8[YAW]) >> 8); // 32 bits is needed for calculation   

  RateE = (((int32_t)rcCommand[YAW] << 6) / conf.P8[YAW]) - (gyroData[YAW] >> 2); 
  RateIntE[YAW] = (abs(gyroData[YAW]) > 2260)? 0 : RateIntE[YAW] = Limit1(RateIntE[YAW] + RateE, 16000); 
  I = ((RateIntE[axis] >> 7) * conf.I8[YAW]) >> 6;  

  D = SmoothRateDerivative(axis, gyroData[YAW]); // 32 bits is needed for calculation

  axisPID[YAW] =  P + I - D;

} // MWControl

#elif defined(USE_MW_BASEFLIGHT_CONTROL)

#define F_CUT   17.0f
#define RC      0.5f / (PI * F_CUT)
#define GYRO_I_MAX 256

void computeControl(void) { // aka pidRewrite from baseflight
  int32_t AngleE = 0;
  uint8_t axis;
  int32_t delta, deltaSum;
  static int32_t delta1[3], delta2[3];
  static int16_t lastDTerm[3] = {
    0,0,0      };
  int32_t PTerm, ITerm, DTerm;
  static int32_t RateEp[3] = { 
    0, 0, 0
  };
  int32_t DesiredRate, RateE;

  static int32_t dT = conf.cycletimeuS;
  
  f.ACRO_TRAINER_MODE = f.ENABLE_ACRO_TRAINER && (maxRollPitchStick >= ACROTRAINER_THRESHOLD);
  if ( f.ACRO_TRAINER_MODE )
    f.ANGLE_MODE = f.HORIZON_MODE = false;

  for(axis = ROLL; axis <= YAW; axis++) {

    if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis != YAW ) 
      AngleE = Limit1(rcCommand[axis] << 1, 500) - angle[axis]; 

    if (axis == YAW) 
      DesiredRate = (((int32_t)(conf.yawRate + 27) * rcCommand[YAW]) >> 5);
    else 
      if (!f.ANGLE_MODE) { 
      DesiredRate = ((int32_t) (conf.rollPitchRate + 27) * rcCommand[axis]) >> 4;
      if (f.HORIZON_MODE) 
        DesiredRate += (AngleE * conf.I8[PIDLEVEL]) >> 8;
    } 
    else 
      DesiredRate = (AngleE * conf.P8[PIDLEVEL]) >> 4;      

    RateE = DesiredRate - (gyroData[axis] >> 2);

    PTerm = (RateE * conf.P8[axis]) >> 7;

    RateIntE[axis] = RateIntE[axis] + ((RateE * dT) >> 11) * conf.I8[axis];
    RateIntE[axis] = Limit1(RateIntE[axis], (int32_t)GYRO_I_MAX << 13);
    ITerm = RateIntE[axis] >> 13;

    delta = RateE - RateEp[axis]; 
    RateEp[axis] = RateE;

    delta = (delta * ((uint16_t)0xffff / (dT >> 4))) >> 6;
    deltaSum = delta1[axis] + delta2[axis] + delta;
    delta2[axis] = delta1[axis];
    delta1[axis] = delta;

    deltaSum = lastDTerm[axis] + (dT / (RC + dT)) * (deltaSum - lastDTerm[axis]);
    lastDTerm[axis] = deltaSum;

    DTerm = (deltaSum * conf.D8[axis]) >> 8;

    axisPID[axis] = PTerm + ITerm + DTerm;
  }
} // computeControl

#else // USE_MW_CONTROL

void computeControl(void) {
  uint8_t axis;
  int16_t P, I, D, RateP, RateI, RateE, AngleE;
  int32_t AngleP, AngleI;
  int16_t prop;

  f.ACRO_TRAINER_MODE = f.ENABLE_ACRO_TRAINER && (maxRollPitchStick >= ACROTRAINER_THRESHOLD);
  if ( f.ACRO_TRAINER_MODE )
    f.ANGLE_MODE = f.HORIZON_MODE = false;

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

    P -= ((int32_t)gyroData[axis] * dynP8[axis]) >> 8; // 32 bits is needed for calculation   

    D = SmoothRateDerivative(axis, gyroData[axis]); // 32 bits is needed for calculation

    axisPID[axis] =  P + I - D;
  }

} // MWControl

#endif // USE_ALT_MW_CONTROL

#endif // USE_MW_XXX

























































