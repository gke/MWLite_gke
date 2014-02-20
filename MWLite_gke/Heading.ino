
// Heading

float makePi(float a) {
  
  while (a < PI) a += TWO_PI;
  while (a >= PI) a -= TWO_PI;
  
  return(a);
} // makePi

int16_t minimumTurnDeg10(int16_t a) {

  while (a < -1800)
    a += 3600;
  while (a >= 1800)
    a -= 3600;

  return a;
} // minimumTurnDeg10

void doHeadFree(void) {
#if defined(MULTICOPTER)
  float relHeading, cosA, sinA;
  int16_t Temp;

  if (rcOptions[BOX_HEAD_FREE])
    if (!f.HEAD_FREE_MODE) headfreeHeading = angle[YAW];

  f.HEAD_FREE_MODE = rcOptions[BOX_HEAD_FREE];

  if(f.HEAD_FREE_MODE) { //to optimize
    relHeading = (angle[YAW] - headfreeHeading) * (PI/1800.0);
    cosA = cos(relHeading);
    sinA = sin(relHeading);
    Temp = rcCommand[PITCH] * cosA + rcCommand[ROLL] * sinA;
    rcCommand[ROLL] =  rcCommand[ROLL] * cosA - rcCommand[PITCH] * sinA; 
    rcCommand[PITCH] = Temp;
  }
#endif
} // doHeadFree

void doMagHold(void) {
#if defined(USE_MAG)
  int32_t HE;

  if (f.ANGLE_MODE && rcOptions[BOX_HEAD_HOLD] && f.MAG_ACTIVE && f.MAG_CALIBRATED && f.SMALL_ANGLE_25DEG && (abs(rcCommand[YAW]) < 70)) {

    f.HEAD_HOLD_MODE = true;
    HE = minimumTurnDeg10(angle[YAW] - holdHeading);
    rcCommand[YAW] -= (HE * conf.P8[PIDMAG]) >> 8;
  }
  else {
    f.HEAD_HOLD_MODE = false;
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


