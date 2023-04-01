

#if defined(USE_MW)

// Original code "tidied" a little for consistency with gke variable names etc.

inline int32_t SmoothRateDerivative(uint8_t axis, int32_t Rate) {
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

  deltaSum = delta[0][axis] + delta[1][axis] + d;
  delta[1][axis] = delta[0][axis];
  delta[0][axis] = d;

  return (deltaSum);
} // SmoothRateDerivative  

void inline doRCRates(void) {
  int16_t Temp;
  uint8_t index, chan, axis;
  int16_t prop1, prop2;

  prop2 = 128; 
  if (rcData[THROTTLE] > 1500) // breakpoint is fix: 1500
      if (rcData[THROTTLE] < 2000) 
      prop2 -=  ((uint16_t)conf.rates.dynThrPID * (rcData[THROTTLE] - 1500) >> 9); //  /512 instead of /500
    else 
      prop2 -=  conf.rates.dynThrPID;

  for (chan = 0; chan < RC_CHANS; chan++)
    rcCommand[chan] = Limit1(rcData[chan] - MID_RC_US, 500); // simpler to do throtte as well

  for(axis = ROLL; axis <= YAW; axis++) {

    Temp = Threshold(abs(rcCommand[axis]), STICK_DEADBAND_US);

    if(axis == YAW) 
      rcCommand[axis] = Temp;
    else { 
      index = Temp >> 7; // 500/128 = 3.9  => range [0;3]
      rcCommand[axis] = rollpitchCurve[index] + ((Temp-(index << 7)) * (rollpitchCurve[index+1]-rollpitchCurve[index]) >> 7);
      prop1 = 128 - ((uint16_t)conf.rates.rollPitchRate*Temp >> 9);
      prop1 = (uint16_t)prop1 * prop2 >> 7;
      dyn[axis].Kp = (uint16_t)conf.pid[axis].Kp * prop1 >> 7; 
      dyn[axis].Kd = (uint16_t)conf.pid[axis].Kd * prop1 >> 7;
    }
    if (rcData[axis] < MID_RC_US) rcCommand[axis] = -rcCommand[axis];
  }

  rcCommand[THROTTLE] = constrain(rcData[THROTTLE], MIN_RC_US, MAX_THR_US);
#if defined(LVC_LIMIT)
  rcCommand[THROTTLE] = ((((int32_t)(rcCommand[THROTTLE] - MIN_RC_US)) * throttleLVCScale) >> 10) + MIN_RC_US;
#endif

} // DoRCRates

//________________________________________

#if defined(USE_MW_ALEXK_CONTROL)

#error "MultiWii AlexK Control scheme NOT COMMISSIONED"

void computeControl(void) { // Alex Khoroshko  http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671&start=10#p37387
  uint8_t axis;
  int32_t prop, RateP, RateI, RateD, DesiredRate, RateE, AngleE;

  prop = min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 500); // range [0;500]

  for(axis = ROLL; axis <= YAW; axis++) {

    if ((f.ANGLE_MODE || f.HORIZON_MODE) && (axis != YAW) )
      AngleE = (rcCommand[axis] * STICK_TO_ANGLE) - angle[axis];

    if (axis == YAW)
      DesiredRate = (((int32_t) (conf.yawRate + 27) * rcCommand[YAW]) >> 5);
    else {
      if (f.ANGLE_MODE) 
        DesiredRate = ((int32_t) AngleE * conf.pid[LEVEL].Kp) >> 4;
      else {
        DesiredRate = ((int32_t) (conf.rollPitchRate + 27) * rcCommand[axis]) >> 4;
        if (f.HORIZON_MODE)
          DesiredRate += ((int32_t) AngleE * conf[LEVEL].Ki) >> 8;
      } 
    }

    RateE = DesiredRate - (gyroData[axis] >> 2);

    RateP = ((int32_t) RateE * conf.pid[axis].Kp) >> 7;

    RateD = (SmoothRateDerivative(axis, RateE) * conf.pid[axis].Kd) >> 5; // forget dt scaling

    axisPID[axis] =  RateP - RateD;

  }
} // computeControl

//________________________________________

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

      AngleP = ((int32_t)AngleE * conf.pid[LEVEL].Kp) >> 7; 
      AngleP = Limit1(AngleP, conf.pid[LEVEL].Kd * 5);

      AngleIntE[axis] = Limit1(AngleIntE[axis] + AngleE, 10000); 
      AngleI = ((int32_t)AngleIntE[axis] * conf.pid[LEVEL].Ki) >> 12; 
    }
    if ( !f.ANGLE_MODE || f.HORIZON_MODE || (axis == YAW) ) { 

      RateE = (((int32_t)rcCommand[axis] << 6) / conf.pid[axis].Kp) - (gyroData[axis] >> 2); 

      RateP = rcCommand[axis];

      RateIntE[axis] =  (abs(gyroData[axis]) > 2260) ? 0 : Limit1(RateIntE[axis] + RateE, 16000);    
      RateI = ((RateIntE[axis] >> 7) * conf.pid[axis].Ki) >> 6;     
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

    P -= ((int32_t)gyroData[axis] * dyn.pid[axis].Kp) >> 8; // 32 bits is needed for calculation   

    D = (SmoothRateDerivative(axis, gyroData[axis]) * dyn.pid[axis].Kd) >> 7;

    axisPID[axis] =  P + I - D;
  }

} // MWControl

//________________________________________

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
  int16_t prop, Desired;

  prop = f.HORIZON_MODE ? min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 512) : 0;

  for (axis = ROLL; axis < YAW; axis++) {

    Desired = Limit1(rcCommand[axis] * STICK_TO_ANGLE, MAX_BANK_ANGLE);

    P = ((int32_t)Desired * conf.pid[axis].Kp) >> 6;

    if ( abs(gyroData[axis]) > 2260 )
      RateIntE[axis] = I = 0;
    else {
      RateE = Desired - (gyroData[axis] >> 2);
      RateIntE[axis] = Limit1(RateIntE[axis] + RateE, 16000);
      I = ((RateIntE[axis] >> 7) * conf.pid[axis].Ki) >> 6;
    }

    if (f.ANGLE_MODE || f.HORIZON_MODE) {

      AngleE = Desired - angle[axis];

      AngleP = ((int32_t)AngleE * conf.pid[LEVEL].Kp) >> 7; 
      AngleP = Limit1(AngleP, conf.pid[LEVEL].Kd * 5);

      AngleIntE[axis] = Limit1(AngleIntE[axis] + AngleE, 10000); 
      AngleI = ((int32_t)AngleIntE[axis] * conf.pid[LEVEL].Ki) >> 12;  

      P = AngleP + (((P - AngleP) * prop) >> 9);
      I = AngleI + (((I - AngleI) * prop) >> 9); 
    }

    P -= ((int32_t)gyroData[axis] * dyn[axis].Kp) >> 8; 

    D = (SmoothRateDerivative(axis, gyroData[axis]) * dyn[axis].Kd) >> 7;

    axisPID[axis] =  P + I - D;
  }

  // Yaw
  Desired = ((int32_t)rcCommand[YAW] * (conf.rates.yawRate * 2 + 30)) >> 5;

  RateE = Desired - (gyroData[YAW] >> 2);

  P = (int32_t)RateE * conf.pid[YAW].Kp >> 6;
  P = Limit1(P, RATE_MAX_P - conf.pid[YAW].Kd);

  if (abs(Desired) > 50) 
    RateIntE[YAW] = I = 0;
  else {
    I = RateIntE[YAW]  = Limit1(RateIntE[YAW] + (int32_t)RateE * conf.pid[YAW].Ki, (int32_t)RATE_MAX_I << 13);
    I = RateIntE[YAW] >> 13;
  }

  axisPID[YAW] = P + I;

} // computeControl

#else

//________________________________________

#error "MultiWii Control scheme not specified"

#endif

#endif // USE_MW_XXX





























