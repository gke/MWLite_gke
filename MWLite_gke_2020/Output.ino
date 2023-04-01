
// since we are uing the PWM generation in a direct way, the pin order is just to initialise the right pins 
// its not possible to change a PWM output pin just by changing the order

// set PWM frequency for DC motors

#define DC_SCALE 0 // 8KHz
//#define DC_SCALE 1 // 4KHz
//#define DC_SCALE 2 // 2KHz

uint8_t PWM_PIN[] = {
  9,10,5,6,11,13, 0, 0};   //for a quad+: rear,right,left,front

#if !defined(MULTICOPTER)
#include <Servo.h>
Servo pwmChan[PWM_OUTPUTS];
const int8_t  servoReverse[] = SERVO_DIR; // Inverted servos
static int16_t newtrims[PWM_OUTPUTS] = {
  0};

#if !defined(SERVO_MECH_TRIMS) // last resort if servos cannot be be machanically trimmed
#define SERVO_MECH_TRIMS {0,0,0,0,0}
#endif

const int16_t servoMechTrims[5] = SERVO_MECH_TRIMS;

void setServoTrims(void) {
  uint8_t i;

  for(i = 1; i < PWM_OUTPUTS; i++)
    global_conf.trim[i] = newtrims[i];

} // setServoTrims

inline void doDifferential(uint8_t R, uint8_t L) {
#if defined(AILERON_DIFF) && (AILERON_DIFF > 0)
  if(AILERON_DIFF != 0) {
    if(pwm[R] < 0)
      pwm[R] = ((int32_t)pwm[R] * (100 - AILERON_DIFF))/100; // use 128? >> 7

    if(pwm[L] < 0)
      pwm[L] = ((int32_t)pwm[L] * (100 - AILERON_DIFF))/100;
  }
#endif
} // doDifferential

#endif

void pwmWrite() { 
  uint8_t i;

#if defined(MULTICOPTER)

#if defined(DC_MOTORS)

  uint16_t Temp;

  // code due originally to Cesco 
  OCR1A = (pwm[0]-1000) << DC_SCALE; //  pin 9 (BR)
  OCR1B = (pwm[1]-1000) << DC_SCALE; //  pin 10 (FR)

  Temp = 2047 - (pwm[2]-1000); // 1023
  TC4H = Temp >> 8; 
  OCR4A = Temp & 0xFF; //  pin 5 (BL)

  Temp = pwm[3] - 1000;
  TC4H = Temp >> 8; 
  OCR4D = Temp & 0xFF; //  pin 6 (FL)

#else // !DC_MOTORS

  OCR1A = pwm[0] << 3; //  pin 9 (BR)
  OCR1B = pwm[1] << 3; //  pin 10 (FR)

  // to write values > 255 to timer 4 A/B we need to split the bytes
  TC4H = (2047-pwm[2]) >> 8; 
  OCR4A = ((2047-pwm[2])&0xFF); //  pin 5 (BL)

  TC4H = pwm[3] >> 8; 
  OCR4D = (pwm[3]&0xFF); //  pin 6 (FL) 

#endif // DC_MOTORS

#else // !MULTICOPTER

  pwmChan[0].writeMicroseconds(pwm[0]);
  for(i = 1; i < PWM_OUTPUTS; i++) {
    pwmP[i] = ((pwmP[i] * 3) + pwm[i] * servoReverse[i] + MID_RC_US + servoMechTrims[i]) >> 2;
    pwmChan[i].writeMicroseconds(pwmP[i]); // + global_conf.trim[i]);//
  }

#endif

} // pwmWrite

void pwmWriteAll(int16_t mc) { 

  for (uint8_t i = 0; i < PWM_OUTPUTS; i++)
    pwm[i]=mc;
  pwmWrite();

} // pwmWriteAll

void initOutput() {
  uint32_t TimeoutuS;
  uint8_t i;

  for (i = 0; i < PWM_OUTPUTS; i++) 
    pinMode(PWM_PIN[i], OUTPUT);

#if defined(MULTICOPTER)

#ifdef DC_MOTORS

  // code due originally to Cesco - 16 bit counters      
  TCCR1A |= (1 << WGM11); // phase correct mode & no prescaler
  TCCR1A &= ~(1 << WGM10);

  TCCR1B &= ~(1 << WGM12) &  ~(1 << CS11) & ~(1 << CS12);
  TCCR1B |= (1 << WGM13) | (1 << CS10);

  ICR1 = (0x03FF << DC_SCALE) | 0x0003; 

  TCCR1A |= _BV(COM1A1); // connect pin 9 (BR) to timer 1 channel A
  TCCR1A |= _BV(COM1B1); // connect pin 10 (FR) to timer 1 channel B

  TCCR4B &= 0xf8;
  TCCR4B |= (1 + DC_SCALE);

  TC4H = 0x3; 
  OCR4C = 0xFF; // phase and frequency correct mode 

  TCCR4D |= (1 << WGM40); 
  TCCR4A |= (1 << COM4A0)|(1 << PWM4A); // connect pin 5 (BL) to timer 4 channel A 
  TCCR4C |= (1 << COM4D1)|(1 << PWM4D); // connect pin 6 (FL) to timer 4 channel D

#else // !DC_MOTORS

  TCCR1A |= (1<<WGM11); // phase correct mode & no prescaler
  TCCR1A &= ~(1<<WGM10);
  TCCR1B &= ~(1<<WGM12) &  ~(1<<CS11) & ~(1<<CS12);
  TCCR1B |= (1<<WGM13) | (1<<CS10); 
  ICR1   |= 0x3FFF; // TOP to 16383; 

  TCCR1A |= _BV(COM1A1); // connect pin 9 (BR) to timer 1 channel A
  TCCR1A |= _BV(COM1B1); // connect pin 10 (FR) to timer 1 channel B

  // timer 4A
  TCCR4E |= (1<<ENHC4); // enhanced pwm mode
  TCCR4B &= ~(1<<CS41); 
  TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
  TCCR4D |= (1<<WGM40); 
  TC4H = 0x3; 
  OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
  TCCR4A |= (1<<COM4A0)|(1<<PWM4A); // connect pin 5 (BL) to timer 4 channel A   

    TCCR4C |= (1<<COM4D1)|(1<<PWM4D); // connect pin 6 (FL) to timer 4 channel D

#endif // DC_MOTORS

#if defined(ESC_CALIB_CANNOT_FLY)
  pwmWriteAll(ESC_CALIB_HIGH);
  LED_BLUE_ON;
  delay(4000);
  pwmWriteAll(ESC_CALIB_LOW);
  while (true) {
    delay(500);
    LED_BLUE_ON;
    delay(500);
    LED_BLUE_OFF;
  }
  exit; // statement never reached
#endif

  pwmWriteAll(MIN_PWM_US);

#else // FLYING_WING || AIRCRAFT

  for (i = 0; i < PWM_OUTPUTS; i++) {
    pwmChan[i].attach(PWM_PIN[i], MIN_PWM_US, MAX_PWM_US);
    pwm[i] = pwmP[i] = (i == THROTTLE) ? MIN_RC_US : MID_RC_US;
    pwmChan[i].writeMicroseconds(pwm[i]);
  }

#endif

  delay(300);

} // initOutput


void mixTable() {
  int16_t maxMotor;
  uint8_t i;

#if defined(MULTICOPTER)

#define PIDMIX(X,Y,Z)  rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + (YAW_DIR) * axisPID[YAW]*Z

#if defined(QUADP)

  pwm[0] = PIDMIX( 0,+1,-1); //REAR
  pwm[1] = PIDMIX(-1, 0,+1); //RIGHT
  pwm[2] = PIDMIX(+1, 0,+1); //LEFT
  pwm[3] = PIDMIX( 0,-1,-1); //FRONT

#elif defined(QUADX)

  pwm[0] = PIDMIX(-1,+1,-1); //REAR_R
  pwm[1] = PIDMIX(-1,-1,+1); //FRONT_R
  pwm[2] = PIDMIX(+1,+1,+1); //REAR_L
  pwm[3] = PIDMIX(+1,-1,-1); //FRONT_L

#endif

  for(i = 0; i < PWM_OUTPUTS; i++) {
    if (f.ARMED) {
      if (rcCommand[THROTTLE] < MIN_CHECK) 
        pwm[i] = MIN_EFF_THR_US;
      else
        pwm[i] = constrain(pwm[i], conf.min_throttleuS, MAX_THR_US);
    }
    else
      pwm[i] = MIN_RC_US;        
  }

#else // !MULTICOPTER

#if !defined(RUDDER_AILERON_COUPLING)
#define RUDDER_AILERON_COUPLING 0
#endif

  static boolean firstPWM = true;
  int16_t R;
  const int8_t servoLimit[5][2] = SERVO_LIMITS; // Rates in 0-100%

#if defined(FLYING_WING)

  if (f.BYPASS_MODE) 
    R = rcCommand[ROLL] + rcCommand[YAW];
  else 
    R = axisPID[ROLL] + axisPID[YAW];

  pwm[RightElevon]  = R;
  pwm[LeftElevon]  = -R;   

  doDifferential(RightElevon, LeftElevon); // only for correction removal of mechanical differential

  if (f.BYPASS_MODE) 
    R = rcCommand[PITCH];
  else 
    R = axisPID[PITCH];

  pwm[RightElevon] -= R;
  pwm[LeftElevon] -= R;

#elif defined(AIRPLANE) || defined(VTAIL) 

  flaperons = 0; //rcCommand[AUX4];

  if(f.BYPASS_MODE){
    R = rcCommand[ROLL] + ((int32_t)rcCommand[YAW]*(100 - RUDDER_AILERON_COUPLING))/100;
    pwm[RightAileron] = R;
    pwm[LeftAileron] = -R;
#if defined(VTAIL)
    pwm[RightRudder] = -rcCommand[YAW] + rcCommand[PITCH];
    pwm[LeftRudder] = rcCommand[YAW] + rcCommand[PITCH];
#else
    pwm[Rudder] = rcCommand[YAW];
    pwm[Elevator] = rcCommand[PITCH];
#endif
  }
  else {
    R = axisPID[ROLL] + ((int32_t)axisPID[YAW]*(100 - RUDDER_AILERON_COUPLING))/100;
    pwm[RightAileron] = R;
    pwm[LeftAileron]  = -R;
#if defined(VTAIL)
    pwm[RightRudder] = axisPID[YAW] + axisPID[PITCH];
    pwm[LeftRudder] = axisPID[YAW] + axisPID[PITCH];
#else
    pwm[Rudder] = axisPID[YAW]; 
    pwm[Elevator] = axisPID[PITCH];
#endif
  }

  doDifferential(RightAileron, LeftAileron);

  pwm[RightAileron] += flaperons;
  pwm[LeftAileron] += flaperons;

#endif // AIRPLANE

  pwm[0] = f.ARMED ? rcData[THROTTLE] : MIN_RC_US;

  for(i = 1; i < PWM_OUTPUTS; i++)
    pwm[i] = constrain(pwm[i], servoLimit[i][0] * 5, servoLimit[i][1] * 5);

  if (firstPWM && !firstRC) {
    for(i = 1; i < PWM_OUTPUTS; i++) {
      newtrims[i] = (pwm[i] - servoMechTrims[i]) * servoReverse[i];
#if defined(DEBUG_TRIMS) 
      debug[i] = newtrims[i];
#endif
    }
    firstPWM = false;
  }

#endif

} // mixTable






















































































