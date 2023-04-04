#if !defined(USE_MW)

// MW control routines replaced by those from UAVX by Prof Greg Egan aka gke 2013

int32_t SmoothRateDerivative(uint8_t axis, int32_t Rate) {
  // simple 3 sample moving average filter
  int32_t d, deltaSum; 
  static int32_t deltaSumP[3] = {
    0
  };
  static int16_t RateP[3] = {
    0
  };
  static int16_t delta[2][3] = {
    0
  };

  d = Rate - RateP[axis];
  RateP[axis] = Rate;

  deltaSum = delta[0][axis] + delta[1][axis] + d;
  delta[1][axis] = delta[0][axis];
  delta[0][axis] = d;

  return (deltaSum);
} // SmoothRateDerivative  

void doRCRates(void) {
  uint8_t chan, index;
  int16_t Temp;

  for (chan = 0; chan < RC_CHANS; chan++)
    if (chan != THROTTLE)
      rcCommand[chan] = Limit1(rcData[chan] - MID_RC_US, 500); // simpler to inc throttle

  for (chan = ROLL; chan <= YAW; chan++)
    rcCommand[chan] = Threshold(rcCommand[chan], STICK_DEADBAND_US);

  rcCommand[THROTTLE] = constrain(rcData[THROTTLE], MIN_RC_US, MAX_THR_US);
#if defined(LVC_LIMIT)
  rcCommand[THROTTLE] = ((((int32_t)(rcCommand[THROTTLE] - MIN_RC_US)) * throttleLVCScale) >> 10) + MIN_RC_US;
#endif      

  f.STICKS_CENTRED = (abs(rcCommand[ROLL]) < STICK_NEUTRAL_DEADBAND_US) && (abs(rcCommand[PITCH]) < STICK_NEUTRAL_DEADBAND_US);

} // doRCRates

void computeControl(void) {
#define ANGLE_MAX_I 200L
#define RATE_MAX_D 300L
#define YAW_RATE_MAX_P 300L
#define YAW_RATE_MAX_I 250L

  uint8_t axis;
  int16_t AngleP, AngleI, AngleD, RateP, RateI, RateD, DesiredAngle, DesiredRate, P, I, D;
  int32_t AngleE, RateE;
  int16_t Temp;

  checkRelayTune(); 

  // Roll/Pitch
  for (axis = ROLL; axis < YAW; axis++)
#if defined(USE_TUNING)
    if (f.RELAY_MODE && tuningAxis[axis])
      axisPID[axis] = tuneStimulus[tuneaxis];
    else
#endif
    { 

      //________________________

#if defined(WOLFERL_CONTROL)

      // From UAVX (Wolferl modified rate scheme) - as simple as it gets?   
      DesiredAngle = Limit1(rcCommand[axis] * STICK_TO_ANGLE + navCorr[axis], MAX_BANK_ANGLE);

      P = -((int32_t)gyroData[axis] * conf.pid[axis].Kp) >> 8;
      I = -Limit1(((int32_t)angle[axis] * conf.pid[axis].Ki) >> 5, 200); 
      D = (SmoothRateDerivative(axis, gyroData[axis]) * conf.pid[axis].Kd) >> 5; //7;
      D = Limit1(D, RATE_MAX_D);

      axisPID[axis] = Limit1((P + I - D) + DesiredAngle, 500);
      axisPID[axis] = (P + I - D) + DesiredAngle;  

      //________________________

#else // PI-PD

      if (f.ANGLE_MODE || f.STICKS_CENTRED) {

        DesiredAngle = Limit1(rcCommand[axis] * STICK_TO_ANGLE + navCorr[axis], MAX_BANK_ANGLE);

        AngleE = DesiredAngle - angle[axis]; 
        AngleP = (AngleE * conf.pid[LEVEL].Kp) >> 3;

        AngleIntE[axis] = Limit1(AngleIntE[axis] + AngleE * conf.pid[axis].Ki, (int32_t)ANGLE_MAX_I << 11); 
        AngleI = AngleIntE[axis] >> 13; 

        DesiredRate = AngleP + AngleI;
      } 
      else {
        AngleIntE[axis] = 0; // for return to Angle Mode
        DesiredRate = ((int32_t)(rcCommand[axis]) << 4) / conf.pid[axis].Kp; // was 6
      }

      RateE = DesiredRate - (gyroData[axis] >> 2);

      RateP = (RateE * conf.pid[axis].Kp) >> 6;
      RateD = (SmoothRateDerivative(axis, RateE) * conf.pid[axis].Kd) >> 3; //5;
      RateD = Limit1(RateD, RATE_MAX_D);

      //axisPID[axis] = Limit1(RateP + RateD, 500);
      axisPID[axis] = RateP + RateD;  

#endif 
    }  

  axisPID[PITCH] += abs((int32_t)axisPID[ROLL] * ROLL_PITCH_COUPLING)/100; // feed forward

  // Yaw PI for rate only
  DesiredRate = ((int32_t)rcCommand[YAW] * (conf.rates.yawRate * 2 + 32)) >> 5;

  RateE = DesiredRate - (gyroData[YAW] >> 2);

  RateP = ((int32_t)RateE * conf.pid[YAW].Kp) >> 6;
  RateP = Limit1(RateP, YAW_RATE_MAX_P);

  if (abs(DesiredRate) > STICK_NEUTRAL_DEADBAND_US) 
    RateIntE[YAW] = RateI = 0;
  else
  {
    RateIntE[YAW] = Limit1(RateIntE[YAW] + (int32_t)RateE * conf.pid[YAW].Ki, (int32_t)YAW_RATE_MAX_I << 13);
    RateI = RateIntE[YAW] >> 13;
  }

  axisPID[YAW] = Limit1(RateP + RateI, abs(rcCommand[YAW]) + 100); // prevent "yaw jump" - slew limit?

} // computeControl

#endif
