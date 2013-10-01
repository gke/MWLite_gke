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


int16_t Threshold(int16_t v, int16_t t) {

  if (v > t)
    v -= t;
  else if (v < -t)
    v += t;
  else
    v = 0;

  return (v);
} // Threshold

uint32_t inline SmoothRateDerivative(uint8_t axis, int16_t Rate) {
  // simple 3 sample moving average filter
  int32_t d, deltaSum; 
  static int16_t Ratep[3] = {
    0      };
  static int32_t delta[2][3] = {
    {
      0
    }
    ,
    {
      0
    }
  };

  d = Rate - Ratep[axis];
  Ratep[axis] = Rate;

  deltaSum = delta[0][axis] + delta[1][axis] + d;
  delta[1][axis] = delta[0][axis];
  delta[0][axis] = d;

  return ((int32_t)deltaSum*conf.D8[axis]) >> 7;
} // SmoothDerivative 

int8_t inline ScaleParam(int16_t c, uint8_t r, uint8_t p) {
  uint16_t Scale;
  
  Scale = constrain(500 - (((int32_t)abs(c) * r) >> 6), 0, 500);
  return(((uint32_t)p * Scale) >> 9); 

} // ScaleParam

void zeroIntegrals(void) {
 memset(&RateIntE, 0, sizeof(RateIntE));
 memset(&AngleIntE, 0, sizeof(AngleIntE));
} // zeroIntegrals

void inline doRCRates(void) {
  // generally reduce PID compensation with greater stick deflection
  static uint8_t axis;
  uint8_t index;
  uint16_t Temp, Scale;

  for (axis = ROLL; axis <= YAW; axis++) 
    rcCommand[axis] = Threshold(Limit1(rcData[axis] - MIDRC, 500), DEADBAND); 

  dynP8[YAW] = ScaleParam(rcCommand[YAW], conf.yawRate, conf.P8[YAW]); 

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

void computeControl(void) {
  static uint8_t axis;
  int16_t AngleE, AngleP, AngleI, AngleD, RateE, RateP, RateI, RateD, DesiredAngle, DesiredRate, P,I,D;
  int16_t Temp;

  checkRelayTune(); 
  checkQuickTune();

  // Roll/Pitch
  for (axis = ROLL; axis < YAW; axis++)  
    if (f.RELAY_MODE && tuningAxis[axis])
      axisPID[axis] = tuneStimulus[tuneaxis];
    else { 

#if defined(WOLFERL)

      // From UAVX (Wolferl modified rate scheme) - as simple as it gets?
      P = -((int32_t)gyroData[axis] * conf.P8[axis]) >> 8;
      I = -Limit1(((int32_t)angle[axis] * conf.I8[axis]) >> 5, 500); 
      D = SmoothRateDerivative(axis, gyroData[axis]);

      DesiredAngle = (f.TUNE_MODE && tuningAxis[axis]) ? tuneStimulus[axis] : (rcCommand[axis]<<1);

      axisPID[axis] = (P + I - D) + DesiredAngle;

#elif defined(V20130614b)

      // From UAVX (Wolferl modified rate scheme)
      P =  -((int32_t)gyroData[axis] * conf.P8[axis]) >> 8;

      if ((abs(rcCommand[axis]) >= ACROTRAINER_MODE) || (abs(gyroData[axis]) > 2360))// zero integral at high rotation rates - Acro
        I = 0;
      else
        if ( f.ANGLE_MODE || f.HORIZON_MODE ) {  
          I = -Limit1(((int32_t)angle[axis] * conf.I8[axis]) >> 5, 500); 
          if ( f.HORIZON_MODE ) 
            I = ((int32_t)I * (512 - abs(rcCommand[axis]))) >> 9;
        }
        else  
          I = 0;

      D = SmoothRateDerivative(axis, gyroData[axis]);

      DesiredAngle = (f.TUNE_MODE && tuningAxis[axis]) ? tuneStimulus[axis] : (rcCommand[axis]<<1);

      axisPID[axis] = (P + I - D) + DesiredAngle;

#elif defined(GKE_EXP)

      DesiredAngle = (f.TUNE_MODE && tuningAxis[axis]) ? tuneStimulus[axis] : rcCommand[axis] * STICK_TO_ANGLE;

      AngleE = DesiredAngle - angle[axis];

      AngleP = ((int32_t)AngleE * conf.P8[PIDLEVEL]) >> 6; //7

      AngleIntE[axis] = Limit1(AngleIntE[axis] + AngleE, 10000); 
      AngleI = ((int32_t)AngleIntE[axis] * conf.I8[PIDLEVEL]) >> 8; // 12;
      AngleI = Limit1(AngleI, 500);

      AngleD = SmoothRateDerivative(PIDLEVEL, gyroData[axis]);

      axisPID[axis] =  AngleP + AngleI - AngleD;

#else

      if ((f.ANGLE_MODE || f.HORIZON_MODE) && (abs(rcCommand[axis]) < ACROTRAINER_MODE) ) {

        DesiredAngle = (f.TUNE_MODE && tuningAxis[axis]) ? tuneStimulus[axis] : rcCommand[axis] * STICK_TO_ANGLE;

        AngleE = DesiredAngle - angle[axis];

        AngleP = ((int32_t)AngleE * conf.P8[PIDLEVEL]) >> 6; //7

        AngleIntE[axis] = Limit1(AngleIntE[axis] + AngleE, 10000); 
        AngleI = ((int32_t)AngleIntE[axis] * conf.I8[PIDLEVEL]) >> 8; // 12;
        AngleI = Limit1(AngleI, 500);

        DesiredRate = AngleP + AngleI;
      }
      else
      {
        AngleIntE[axis] = 0;
        DesiredRate = rcCommand[axis];
      }

      RateP = DesiredRate - (((int32_t)gyroData[axis] * conf.P8[axis]) >> 8);

      RateD = SmoothRateDerivative(axis, gyroData[axis]);

      axisPID[axis] =  RateP - RateD;

#endif 

    }

  // Yaw
  P = ((int32_t)gyroData[YAW] * dynP8[YAW]) >> 8;

  RateE = (((int32_t)rcCommand[YAW] << 6) / conf.P8[YAW]) - (gyroData[YAW] >> 2); 
  RateIntE[YAW] = (abs(gyroData[YAW]) > 2560)? 0 : RateIntE[YAW] = Limit1(RateIntE[YAW] + RateE, 16000); 
  I = ((RateIntE[YAW] >> 7) * conf.I8[YAW]) >> 6;

  Temp = Limit1(rcCommand[YAW] - (P + I), 500); 

  axisPID[YAW] = Limit1(Temp, abs(rcCommand[YAW]) + 100); // prevent "yaw jump" - slew limit?

} // computeControl

#endif







































































