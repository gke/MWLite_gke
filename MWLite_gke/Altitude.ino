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

// Altitude

void doAltitudeControl(void) {
  static int16_t altCorr = 0;
  int16_t throttleDiff;

  checkQuickAltTune();

  if (f.ARMED && f.ALT_HOLD_MODE && (f.BARO_ACTIVE || f.SONAR_ACTIVE)) {      
    if (f.ALT_TUNE_MODE && tuningAlt) {
      desiredAltitude = tuneAltStimulus;
    } else {
      if (abs(rcCommand[THROTTLE] - hoverThrottle) > ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
        throttleDiff = rcCommand[THROTTLE] - hoverThrottle;
#ifdef USE_PROP_ALT_HOLD
        desiredAltitude += throttleDiff/ALT_HOLD_THROTTLE_NEUTRAL_ZONE; 
#else
        desiredAltitude += Sign(throttleDiff) * ALT_HOLD_STEP;  // 40cm/S per step 
#endif
        desiredAltitude = constrain(desiredAltitude, 100, ALT_HOLD_LIMIT_M * 100);
        AltitudeIntE = 0;
      } 
    }
    rcCommand[THROTTLE] = hoverThrottle + altPID; // throttle is overriden by altitude hold
  }
} // doAltitudeControl

uint32_t inline SmoothAltDerivative(int16_t Rate) {
  // simple 4 sample moving average filter
  int32_t d, deltaSum; 
  static int16_t Ratep = 0;
  static int32_t delta[3] = {
    0,0
  };

  d = Rate - Ratep;
  Ratep = Rate;

  deltaSum = delta[0] + delta[1] + delta[2] + d;
  delta[2] = delta[1];
  delta[1] = delta[0];
  delta[0] = d;

  return ((int32_t)deltaSum >> 2);
} // SmoothAltDerivative

//#define ACC_VEL_SCALE (9.80665f / 10000.0f * GRAVITY_R) // zzz
#define ACC_VEL_SCALE (9.80665f * GRAVITY_R * 100.0f) // cm/S/S

int16_t filterROC(float BaroROC, float dT) {
  // Apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
  // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
  if (f.SMALL_ANGLE_25DEG) {
    calculateVerticalAcceleration();
    velZ += accZ * ACC_VEL_SCALE * dT;
    velZ = velZ * 0.985f + BaroROC * 0.015f;
  } 
  else
    velZ = BaroROC;

  return(velZ);
} // filterROC

#if defined(USE_MW_ALT_CONTROL)

void computeAltitudeControl(void) {

  static int32_t baroGroundPressure;
  static int32_t relativeAltitudep;
  static float vel = 0.0;
  static int16_t accZoffset = 0;
  int16_t vel_tmp;
  int16_t baroVel;
  int16_t AltE, P, I, D;
  static uint32_t prevUpdateuS = 0;
  float dT, dTR;

  if (updateAltitude()) {
    NowuS = micros();
    dT = (float)(NowuS - prevUpdateuS) * 0.000001f;
    dTR = 1.0 / dT;
    prevUpdateuS = NowuS; 

    AltE = Limit1(desiredAltitude - relativeAltitude, 300);
    AltE = Threshold(AltE, 10); //remove small P parameter to reduce noise near zero position
    P = Limit1((conf.P8[PIDALT] * AltE >> 7), 150);

    AltitudeIntE += conf.I8[PIDALT] * AltE >>6;
    AltitudeIntE = Limit1(AltitudeIntE, 30000);
    I = AltitudeIntE >> 9; //I in range +/-60

    baroVel = (relativeAltitude - relativeAltitudep) * dTR;
    relativeAltitudep = relativeAltitude;

    baroVel = Limit1(baroVel, 300); // constrain baro velocity +/- 300cm/s
    baroVel = Threshold(baroVel, 10); // to reduce noise near zero

    ROC = filterROC(baroVel, dT);  
    ROC = Threshold(ROC, 5);

    D = Limit1(conf.D8[PIDALT] * ROC >> 4, 150);

    altPID = constrain(P + I - D, -150, 200);

#if defined(DEBUG_ALT_HOLD)
    debug[0] = ROC;
    debug[1] = desiredAltitude; 
    debug[2] = altPID; 
    debug[3] = rcCommand[THROTTLE]; 
#endif
  }

} // computeAltitudeControl

#else

#define Kdd 10 // no idea yet!

void computeAltitudeControl(void) {
  int16_t AltE, P, I, D, DD, desiredROC, desiredAcc;
  static uint32_t prevUpdateuS = 0;
  float dT, dTR;

  if (updateAltitude()) {
    NowuS = micros();
    dT = (float)(NowuS - prevUpdateuS) * 0.000001f;
    dTR = 1.0 / dT;
    prevUpdateuS = NowuS;

    desiredROC = desiredAltitude - ((relativeAltitude * conf.P8[PIDALT]) >> 7);
    desiredROC = constrain(desiredROC, 100, 200);

    ROC = SmoothAltDerivative(relativeAltitude) * dTR;
    desiredAcc =  desiredROC - ((ROC * conf.D8[PIDALT]) >> 4);

    calculateVerticalAcceleration();
    altPID = desiredAcc - ((accZ * Kdd) >> 2); 

#if defined(DEBUG_ALT_HOLD)
    debug[0] = ROC;
    debug[1] = desiredAltitude; 
    debug[2] = altPID; 
    debug[3] = rcCommand[THROTTLE]; 
#endif
  }
} // computeAltitudeControl

#endif



























































