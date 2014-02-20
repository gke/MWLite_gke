
#include "config.h"

enum tuneStates {
  Tuning, startTuning, stopTuning, notTuning};

boolean TuningUsed = false;

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

  f.RELAY_MODE = rcOptions[BOX_RELAY];
#endif
} // checkRelayTune


































































