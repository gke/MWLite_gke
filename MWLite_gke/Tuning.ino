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

#include "config.h"

enum tuneStates {
  Tuning, startTuning, stopTuning, notTuning};

boolean TuningUsed = false;

// Quick Tune

// Copyright 2013 Brad Quick rewritten by Prof Greg Egan 2013

// The appplied target angle alternates from negative to positive QUICK_ANGLE_TARGET degrees.
// We set the I term to zero and work on only the P and D terms.
// When we apply the step we will overshoot or undershoot the target angle. If we 
// overshoot we should see a damped oscillation.

// We then have adjust P and D as follows:
// if (overshoot)
//      if (too much osc) decrease P
//      else increase D
//   else // undershoot
//      if (too much osc) decrease D
//      else increase P

// To adjust how agressive the tuning is, adjust the acceptable overshoot (MAX_OVERSHOOT_PERCENT). 
// A larger value will result in more aggressive tuning. A lower value will result in softer tuning.

#define MAX_OVERSHOOT_PERCENT 10

#define QUICK_ANGLE_TARGET 200 
#define MAX_OSC ((MAX_OVERSHOOT_PERCENT*2*QUICK_ANGLE_TARGET)/100)

#define SETTLING_TIME 250 // mS

#define increase(p, m) (constrain(p++, 0, m))
#define decrease(p, m) (constrain(p--, m, 200))

void QuickTune(uint8_t tuneState) {
  enum pStates {
    tuneI, tunePD
  };
  static uint8_t paramState;
  static int8_t Direction = 1;
  static uint32_t tuneTime;
  static int16_t maxPeak, minPeak;
  static uint8_t newI;
  static int16_t oscAmp;

  tuningAxis[tuneaxis] = tuneState == Tuning;

  if (f.ARMED) {

    switch (tuneState) {
    case startTuning:
      newI = conf.I8[tuneaxis];
      conf.I8[tuneaxis] = 0;

      Direction  = -1;
      tuneStimulus[tuneaxis] = Direction * QUICK_ANGLE_TARGET;
      maxPeak = minPeak = 0;

      TuningUsed = false;
      tuneTime = millis() + 100;
      paramState = tuneI; //Tuning;
      break; 
    case Tuning:
      switch (paramState) {
      case tuneI:
        if (Sign(angle[tuneaxis]) == Sign(tuneStimulus[tuneaxis])) {

          maxPeak = max(maxPeak, abs(angle[tuneaxis]));

          if ((abs(angle[tuneaxis]) < maxPeak) || (millis() > tuneTime)) {

            if ((maxPeak - tuneStimulus[tuneaxis]) < (MAX_OSC/2)) 
              newI = increase(newI, MAX_I);
            else
              newI = decrease(newI, MIN_I);

            minPeak = maxPeak;
            tuneTime = millis() + SETTLING_TIME;
            paramState = tunePD;
          } 
        }
        break;
      case tunePD:

        minPeak = min(minPeak, abs(angle[tuneaxis]));

        if (millis() > tuneTime){

          oscAmp = maxPeak - minPeak;

          if (maxPeak > tuneStimulus[tuneaxis]) {
            if (oscAmp > MAX_OSC) 
              conf.P8[tuneaxis] = decrease(conf.P8[tuneaxis], MIN_P);
            else 
              conf.D8[tuneaxis] = increase(conf.D8[tuneaxis],  MAX_D);
          }
          else {
            if (oscAmp > MAX_OSC) 
              conf.D8[tuneaxis] = decrease(conf.D8[tuneaxis], MIN_D);
            else 
              conf.P8[tuneaxis] = increase(conf.P8[tuneaxis], MAX_P);
          }

          Direction = (Direction < 0) ? 1 : -1;
          tuneStimulus[tuneaxis] = Direction * QUICK_ANGLE_TARGET;
          maxPeak = minPeak = 0;
          tuneTime = millis() + 100;
          paramState = tuneI;
        }
        break;    
      }
    case stopTuning:    
      conf.I8[tuneaxis] = newI;
      TuningUsed = true;
      break;
    }
  }
  else
    if (TuningUsed) {
      writeParams(0);
      TuningUsed = false;
    }

} // QuickTune

void checkQuickTune(void) {
#if defined(USE_QUICK_TUNE)

  if (f.ANGLE_MODE) {
    if (rcOptions[BOX_TUNE])
      if (!f.TUNE_MODE) {
#if defined(TUNE_BOTH_AXES)      
        tuneaxis = (tuneaxis == ROLL) ? PITCH : ROLL;
#endif
        QuickTune(startTuning); 
      }
      else
        QuickTune(Tuning);
    else 
      if (f.TUNE_MODE) 
      QuickTune(stopTuning);

    f.TUNE_MODE = rcOptions[BOX_TUNE];
  }
#else
  tuningAxis[0] = tuningAxis[1] = false;
#endif
} // checkTunePID


boolean AltTuningUsed = false;

#define QUICK_ALT_TARGET 200
#define MAX_ALT_OSC ((MAX_OVERSHOOT_PERCENT*2*QUICK_ALT_TARGET)/100)

#define ALT_SETTLING_TIME 5000 // mS

void QuickAltTune(uint8_t tuneState) {
  enum pStates {
    tuneI, tunePD
  };
  static int32_t holdTuneAltitude = desiredAltitude;
  static uint8_t paramState;
  static int8_t Direction = 1;
  static uint32_t tuneTime;
  static int16_t maxPeak, minPeak;
  static uint8_t newI;
  static int16_t oscAmp;
  int16_t altDiff;

  tuningAlt = tuneState == Tuning;

  if (f.ARMED) {
    
    altDiff = relativeAltitude - desiredAltitude; // +ve high

    switch (tuneState) {
    case startTuning:
      newI = conf.I8[PIDALT];
      conf.I8[PIDALT] = 0;

      Direction  = -1;
      holdTuneAltitude = relativeAltitude;
      tuneAltStimulus = holdTuneAltitude + Direction * QUICK_ALT_TARGET;
      maxPeak = minPeak = 0;

      TuningUsed = false;
      tuneTime = millis() + (ALT_SETTLING_TIME/2);
      paramState = tuneI; //Tuning;
      break; 
    case Tuning:
      switch (paramState) {
      case tuneI:
        if (Sign(altDiff) == Sign(tuneAltStimulus)) {

          maxPeak = max(maxPeak, abs(relativeAltitude - desiredAltitude));

          if ((abs(altDiff) < maxPeak) || (millis() > tuneTime)) {

            if ((maxPeak - tuneAltStimulus) < (MAX_OSC/2)) 
              newI = increase(newI, MAX_I);
            else
              newI = decrease(newI, MIN_ALT_I);

            minPeak = maxPeak;
            tuneTime = millis() + ALT_SETTLING_TIME;
            paramState = tunePD;
          } 
        }
        break;
      case tunePD:

        minPeak = min(minPeak, abs(altDiff));

        if (millis() > tuneTime){

          oscAmp = maxPeak - minPeak;

          if (maxPeak > tuneAltStimulus) {
            if (oscAmp > MAX_ALT_OSC) 
              conf.P8[PIDALT] = decrease(conf.P8[PIDALT], MIN_ALT_P);
            else 
              conf.D8[PIDALT] = increase(conf.D8[PIDALT],  MAX_ALT_D);
          }
          else {
            if (oscAmp > MAX_ALT_OSC) 
              conf.D8[PIDALT] = decrease(conf.D8[PIDALT], MIN_ALT_D);
            else 
              conf.P8[PIDALT] = increase(conf.P8[PIDALT], MAX_ALT_P);
          }

          Direction = (Direction < 0) ? 1 : -1;
          tuneAltStimulus = holdTuneAltitude + Direction * QUICK_ALT_TARGET;
          maxPeak = minPeak = 0;
          tuneTime = millis() + (ALT_SETTLING_TIME/2);
          paramState = tuneI;
        }
        break;    
      }
    case stopTuning:    
      conf.I8[PIDALT] = newI;
      desiredAltitude = holdTuneAltitude;
      AltTuningUsed = true;
      break;
    }
  }
  else
    if (AltTuningUsed) {
      writeParams(0);
      AltTuningUsed = false;
    }

} // QuickAltTune

void checkQuickAltTune(void) {
#if defined(USE_QUICK_ALT_TUNE)
    if (rcOptions[BOX_ALT_TUNE] && f.ALT_HOLD_MODE)
      if (!f.ALT_TUNE_MODE) 
        QuickAltTune(startTuning); 
      else
        QuickAltTune(Tuning);
    else 
      if (f.ALT_TUNE_MODE  && f.ALT_HOLD_MODE) 
      QuickAltTune(stopTuning);

    f.ALT_TUNE_MODE = rcOptions[BOX_ALT_TUNE] && f.ALT_HOLD_MODE;

#else
  tuningAlt = false;
#endif
} // checkQuickAltTune

// Relay Tune

// Copyright 2013  Prof Greg Egan
// Uses relay scheme to capture tuning parameters
// "Relay-based PID Tuning", D.I.Wilson, Automation & Control, pp 10-12, Feb/Mar 2005.

enum Scenarios {
  ZN, SmallOvershoot, NoOvershoot};
float K[3][3] = {
  { 
    0.6, 0.5, 0.125        }
  ,{
    0.33,0.5,0.33        }
  ,{
    0.2,0.5,0.33        }
};

#define RELAY_START_SAMPLES 8
#define RELAY_TUNE_SAMPLES 8

#define RELAY_TARGET 200


void RelayPID(uint8_t tuneaxis, uint8_t s) {
#define KPSCALE ((float)(1<<6))
#define KISCALE ((float)(1<<5))
#define KDSCALE ((float)(1<<5))

  float Kc, Ti, Td;

  Kc = K[s][1] * global_conf.RelayK[tuneaxis];
  Ti = K[s][2] * global_conf.RelayP[tuneaxis];
  Td = K[s][2] * global_conf.RelayP[tuneaxis];

  // NOT FINISHED
  //  conf.P8[tuneaxis] = (int8_t)constrain(Kc * KPSCALE, MIN_P, MAX_P);
  //  conf.I8[tuneaxis] = (int8_t)constrain((Kc / Ti) * KISCALE, MIN_I, MAX_I);
  //  conf.D8[tuneaxis] = (int8_t)constrain((Kc * Td) * KDSCALE, MIN_D, MAX_D);

#if defined(DEBUG_RELAY)
  debug[tuneaxis*2] = (int8_t)constrain(Kc * KPSCALE, MIN_P, MAX_P);
  debug[tuneaxis*2+1] = (int8_t)constrain((Kc / Ti) * KISCALE, MIN_I, MAX_I);
#endif
} // RelayPID

void RelayTune(uint8_t tuneState) {
  enum relayStates { 
    relayStart, relayCapture, relayFinished, relayFailed 
  };
  static uint8_t relayState = relayFinished;
  static int8_t Direction = 1;
  static uint32_t RelayTimeuS;
  static int16_t RelayA = 0;
  static uint32_t RelayTau;
  static uint8_t Samples;
  static int32_t RelayASum, RelayTauSum;
  static int16_t TimeOut;

  if (f.ARMED)
    switch (tuneState) {
    case startTuning:
      Samples = 0;
      Direction = (Sign(angle[tuneaxis]) < 0) ? 1 : -1;
      tuneStimulus[tuneaxis] = Direction * RELAY_TARGET;
      TuningUsed = false;
      TimeOut = millis() + 500;
      tuningAxis[tuneaxis] = true;
      relayState = relayStart;
      break;
    case Tuning:
      switch (relayState) {
      case relayStart:

        if (Sign(angle[tuneaxis]) == Sign(tuneStimulus[tuneaxis])) {
          RelayTimeuS = micros();
          TimeOut = millis() + 500;
          Direction = (Direction < 0) ? 1 : -1;
          tuneStimulus[tuneaxis] = Direction * RELAY_TARGET;
          if (++Samples > RELAY_START_SAMPLES) {
            RelayASum = RelayTauSum = RelayA = Samples = 0;
            relayState = relayCapture;
          }
        }
        else 
          if ((millis() > TimeOut) || (abs(angle[tuneaxis]) > 450)) 
          relayState = relayFailed;

        break;
      case relayCapture: 

        RelayA = max(RelayA, abs(angle[tuneaxis]));

        if (Sign(angle[tuneaxis]) == Sign(tuneStimulus[tuneaxis])) {

          RelayTau = micros() - RelayTimeuS;
          TimeOut = millis() + 500;
          RelayTauSum += RelayTau;
          RelayASum += RelayA;
          RelayA = 0;

          RelayTimeuS = micros();
          Direction = (Direction < 0) ? 1 : -1;
          tuneStimulus[tuneaxis] = Direction * RELAY_TARGET;
          if (++Samples > RELAY_TUNE_SAMPLES) 
            relayState = relayFinished;
        } 
        else 
          if ((millis() > TimeOut) || (abs(angle[tuneaxis]) > 450)) 
          relayState = relayFailed;

        break;
      case relayFinished:
        tuningAxis[tuneaxis] = false;
        global_conf.RelayK[tuneaxis] = ((4.0 * RELAY_TARGET * RELAY_TUNE_SAMPLES)/PI) / RelayASum;        
        global_conf.RelayP[tuneaxis] = (float)RelayTauSum / RELAY_TUNE_SAMPLES;
        RelayPID(tuneaxis, SmallOvershoot);
        TuningUsed = true;

        break;
      case relayFailed:
        tuningAxis[tuneaxis] = false;
        break;
      }
      break;
    case stopTuning:
      tuningAxis[tuneaxis] = false;
      break;
    }
  else
    if (TuningUsed) {
      writeGlobalSet(0);
      TuningUsed = false;
    } 

} // RelayTune

void checkRelayTune(void) {
#if defined(USE_RELAY_TUNE)
  uint8_t tuneState = notTuning;
  RelayPID(1,1);
  if (rcOptions[BOX_RELAY])
    if (!f.RELAY_MODE) {
#ifdef TUNE_BOTH_AXES
      tuneaxis = (tuneaxis == ROLL) ? PITCH : ROLL;
#endif 
      RelayTune(startTuning); 
    }
    else
      RelayTune(Tuning);
  else 
    if (f.RELAY_MODE) 
    RelayTune(stopTuning);

  f.RELAY_MODE = rcOptions[BOXRELAY];
#endif
} // checkRelayTune


































































