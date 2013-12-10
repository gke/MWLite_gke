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

// MW control routines replaced by those from UAVX by Prof Greg Egan aka gke 2013

#if !defined(USE_MW)

#if defined(USE_TX_TUNING)
#define txTuneRate(n)  rateGain[n]/1000
#else
#define txTuneRate(n) 1
#endif

static int16_t rateGain[2];

int16_t Threshold(int16_t v, int16_t t) {

  if (v > t)
    v -= t;
  else if (v < -t)
    v += t;
  else
    v = 0;

  return (v);
} // Threshold

#define RATE_F_CUT   20
#define RATE_TC      ((int32_t)(1000000 / ( 2.0 * PI * RATE_F_CUT))) 

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

  d = (d *(4194304L / dTuS)) >> 8;

  deltaSum = delta[0][axis] + delta[1][axis] + d;
  delta[1][axis] = delta[0][axis];
  delta[0][axis] = d;

  //#define USE_RATE_DELTA_LPF
#if defined(USE_RATE_DELTA_LPF)
  //deltaSum = deltaSumP[axis] + (dTuS / (RC + dTuS)) * (deltaSum - deltaSumP[axis]);
  deltaSum = deltaSumP[axis] + ((deltaSum - deltaSumP[axis]) * CYCLETIME)/(RATE_TC + CYCLETIME);  
  deltaSumP[axis] = deltaSum;
#endif

  return (deltaSum);
} // SmoothRateDerivative  

int8_t ScaleParam(int16_t c, uint8_t r, uint8_t p) {
  uint16_t Scale;

  Scale = constrain(500 - (((int32_t)abs(c) * r) >> 6), 0, 500);
  return(((uint32_t)p * Scale) >> 9); 

} // ScaleParam

void doRCRates(void) {
  uint8_t chan, index;
  int16_t Temp;

  for (chan = 0; chan < RC_CHANS; chan++)
    if (chan != THROTTLE)
      rcCommand[chan] = Limit1(rcData[chan] - MIDRC, 500); 

  for (chan = ROLL; chan <= YAW; chan++)
    rcCommand[chan] = Threshold(rcCommand[chan], DEADBAND);

  f.STICKS_CENTRED = (abs(rcCommand[ROLL]) < STICK_NEUTRAL_DEADBAND) && (abs(rcCommand[PITCH]) < STICK_NEUTRAL_DEADBAND);

#if defined(USE_THROTTLE_CURVE)
  Temp = constrain(rcData[THROTTLE],MIN_CHECK,2000);
  Temp = (uint32_t)(Temp - MIN_CHECK) * 2559 / (2000 - MIN_CHECK); // [MINCHECK;2000] -> [0;2559]
  index = Temp / 256; // range [0;9]
  rcCommand[THROTTLE] = thrCurve[index] + (Temp - index*256) * (thrCurve[index+1] - thrCurve[index]) / 256; // [0;2559] -> expo -> [conf.minthrottle;MAXTHROTTLE]
#else
  rcCommand[THROTTLE] = constrain(rcData[THROTTLE], MIN_COMMAND, MAX_THROTTLE);
#endif // USE_THROTTLE_CURVE

    rcCommand[THROTTLE] = ((((int32_t)(rcCommand[THROTTLE] - MIN_COMMAND)) * throttleLVCScale) >> 10) + MIN_COMMAND;

#if defined(USE_TX_TUNING)
#if defined(ISMULTICOPTER)
  debug[0] = rateGain[ROLL] = rateGain[PITCH] = rcCommand[AUX3] + 500;
#else
  debug[0] = rateGain[ROLL] = rcCommand[AUX3] + 500;
  debug[1] = rateGain[PITCH] = rcCommand[AUX4] + 500;
#endif
#endif 

} // DoRCRates

void computeControl(void) {
#define ANGLE_MAX_I 200
#define YAW_RATE_MAX_P 300L
#define YAW_RATE_MAX_I 250L

  static uint8_t axis;
  int16_t AngleP, AngleI, AngleD, RateP, RateI, RateD, DesiredAngle, DesiredRate, P, I, D;
  int32_t AngleE, RateE;
  int16_t Temp;

  checkRelayTune(); 

  // Roll/Pitch
  for (axis = ROLL; axis < YAW; axis++)
    if (f.RELAY_MODE && tuningAxis[axis])
      axisPID[axis] = tuneStimulus[tuneaxis];
    else { 

      //________________________

#if defined(WOLFERL)

      // From UAVX (Wolferl modified rate scheme) - as simple as it gets?
      DesiredAngle = rcCommand[axis] * STICK_TO_ANGLE;
      DesiredAngle = Limit1(DesiredAngle, MAX_BANK_ANGLE);

      P = -((int32_t)gyroData[axis] * conf.P8[axis]) >> 8;
      I = -Limit1(((int32_t)angle[axis] * conf.I8[axis]) >> 5, 500); 
      D = (SmoothRateDerivative(axis, gyroData[axis]) * conf.D8[axis]) >> 7;

      axisPID[axis] = (P + I - D) + DesiredAngle;  

      //________________________

#elif defined(V20131112) // P-PD

      if (f.ANGLE_MODE || f.STICKS_CENTRED) {

        DesiredAngle = rcCommand[axis]; // * STICK_TO_ANGLE; // stick is [-500;500] or 50deg unscaled
        DesiredAngle = Limit1(DesiredAngle, MAX_BANK_ANGLE);

        AngleE = DesiredAngle - angle[axis]; 
        AngleP = (AngleE * conf.P8[PIDLEVEL] * txTuneRate(axis)) >> 3;

        AngleIntE[axis] = Limit1(AngleIntE[axis] + (int32_t)AngleE * conf.I8[axis], ANGLE_MAX_I << 13);
        AngleI = AngleIntE[axis] >> 13;

        DesiredRate = AngleP + AngleI;
      } 
      else
        DesiredRate = (rcCommand[axis] << 6) / conf.P8[axis];

      RateE = DesiredRate - (gyroData[axis] >> 2);

      RateP = (RateE * conf.P8[axis]) >> 6;
      RateD = (SmoothRateDerivative(axis, RateE) * conf.D8[axis]) >> 5;

      axisPID[axis] = RateP + RateD;

#else
#error "Control scheme not specified"
#endif 

    }

  // Yaw PI for rate only
  DesiredRate = ((int32_t)rcCommand[YAW] * (conf.yawRate * 2 + 32)) >> 5;

  RateE = DesiredRate - (gyroData[YAW] >> 2);

  RateP = ((int32_t)RateE * conf.P8[YAW]) >> 6;
  RateP = Limit1(RateP, YAW_RATE_MAX_P);

  if (abs(DesiredRate) > STICK_NEUTRAL_DEADBAND) 
    RateIntE[YAW] = RateI = 0;
  else
  {
    RateIntE[YAW] = Limit1(RateIntE[YAW] + (int32_t)RateE * conf.I8[YAW], YAW_RATE_MAX_I << 13);
    RateI = RateIntE[YAW] >> 13;
  }

  axisPID[YAW] = RateP + RateI;

  // axisPID[YAW] = Limit1(axisPID[YAW], abs(rcCommand[YAW]) + 100); // prevent "yaw jump" - slew limit?

} // computeControl

#endif
























