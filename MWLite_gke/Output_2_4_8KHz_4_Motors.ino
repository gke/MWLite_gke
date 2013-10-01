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
 
// Version Stripped for 4 motors and no servos

// set PWM frequency for DC motors

#define DC_SCALE 0 // 8KHz
//#define DC_SCALE 1 // 4KHz
//#define DC_SCALE 2 // 2KHz

// since we are uing the PWM generation in a direct way, the pin order is just to initialise the right pins 
// its not possible to change a PWM output pin just by changing the order

uint8_t PWM_PIN[] = {
  9,10,5,6};   //for a quadx: Rear right, front right, rear left & front left; quad+: rear, right, left & front

void writeMotors() { 

#if defined(DC_MOTORS)

  uint16_t Temp;

#if defined(CESCO_OFFSET)

  OCR1A = (motor[0]-1000) << 1; //  pin 9 (BR)
  OCR1B = (motor[1]-1000) << 1; //  pin 10 (FR)

  OCR3A = (motor[2]-1000) << 1; //  pin 5 (BL)
  Temp = motor[3] - 1000;
  TC4H = Temp >> 8;
  OCR4D = Temp & 0xFF; //  pin 6 (FL)

#else

  // code due originally to Cesco 
  OCR1A = (motor[0]-1000) << DC_SCALE; //  pin 9 (BR)
  OCR1B = (motor[1]-1000) << DC_SCALE; //  pin 10 (FR)

  Temp = 2047 - (motor[2]-1000); // 1023
  TC4H = Temp >> 8; 
  OCR4A = Temp & 0xFF; //  pin 5 (BL)

  Temp = motor[3] - 1000;
  TC4H = Temp >> 8; 
  OCR4D = Temp & 0xFF; //  pin 6 (FL)

#endif

#else // !DC_MOTORS

  OCR1A = motor[0] << 3; //  pin 9 (BR)
  OCR1B = motor[1] << 3; //  pin 10 (FR)
#if defined(HWPWM6)
  OCR3A = motor[2] << 3; //  pin 5 (BL)
#else
  // to write values > 255 to timer 4 A/B we need to split the bytes
  TC4H = (2047-motor[2]) >> 8; 
  OCR4A = ((2047-motor[2])&0xFF); //  pin 5 (BL)
#endif
  TC4H = motor[3] >> 8; 
  OCR4D = (motor[3]&0xFF); //  pin 6 (FL) 

#endif // DC_MOTORS

} // writeMotors

void writeAllMotors(int16_t mc) {  
  for (uint8_t i =0; i < NUMBER_MOTOR; i++)
    motor[i]=mc;
  writeMotors();
} // writeAllMotors

void initOutput() {
  uint32_t TimeoutuS;

  for (uint8_t i = 0; i < NUMBER_MOTOR; i++) 
    pinMode(PWM_PIN[i],OUTPUT);

#ifdef DC_MOTORS

#if defined(CESCO_OFFSET)

  TCCR1A = (1<<WGM11); 
  TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);

  delayMicroseconds(64);
  TCCR3A = (1<<WGM31); 
  TCCR3B = (1<<WGM33) | (1<<WGM32) | (1<<CS30); 

  ICR1   = 0x07FF; // TOP to 2046;     
  ICR3   = 0x07FF; // TOP to 2046; 

  delayMicroseconds(105);
  TC4H = 0x3;
  OCR4C = 0xFF; // phase and frequency correct mode
  TCCR4B = (1<<CS41); // prescaler to 2

  TCCR1A |= _BV(COM1A1); // connect pin 9 (BR) to timer 1 channel A
  TCCR1A |= _BV(COM1B1); // connect pin 10 (FR) to timer 1 channel B
  TCCR3A |= _BV(COM3A1); // connect pin 5 (BL) to timer 3 channel A    

  TCCR4D = 0;    
  TCCR4C |= (1<<COM4D1)|(1<<PWM4D); // connect pin 6 (FL) to timer 4 channel D

#else

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

#endif

#else // !DC_MOTORS

  TCCR1A |= (1<<WGM11); // phase correct mode & no prescaler
  TCCR1A &= ~(1<<WGM10);
  TCCR1B &= ~(1<<WGM12) &  ~(1<<CS11) & ~(1<<CS12);
  TCCR1B |= (1<<WGM13) | (1<<CS10); 
  ICR1   |= 0x3FFF; // TOP to 16383; 

  TCCR1A |= _BV(COM1A1); // connect pin 9 (BR) to timer 1 channel A
  TCCR1A |= _BV(COM1B1); // connect pin 10 (FR) to timer 1 channel B

#if defined(HWPWM6) // timer 3A
  TCCR3A |= (1<<WGM31); // phase correct mode & no prescaler
  TCCR3A &= ~(1<<WGM30);
  TCCR3B &= ~(1<<WGM32) &  ~(1<<CS31) & ~(1<<CS32);
  TCCR3B |= (1<<WGM33) | (1<<CS30); 
  ICR3   |= 0x3FFF; // TOP to 16383;     
  TCCR3A |= _BV(COM3A1); // connect pin 5 (BL) to timer 3 channel A   
#else
  // timer 4A
  TCCR4E |= (1<<ENHC4); // enhanced pwm mode
  TCCR4B &= ~(1<<CS41); 
  TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
  TCCR4D |= (1<<WGM40); 
  TC4H = 0x3; 
  OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
  TCCR4A |= (1<<COM4A0)|(1<<PWM4A); // connect pin 5 (BL) to timer 4 channel A   
#endif 

#if defined(HWPWM6) 
  TCCR4E |= (1<<ENHC4); // enhanced pwm mode
  TCCR4B &= ~(1<<CS41); 
  TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
  TCCR4D |= (1<<WGM40); 
  TC4H = 0x3; 
  OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
#endif
  TCCR4C |= (1<<COM4D1)|(1<<PWM4D); // connect pin 6 (FL) to timer 4 channel D

#endif // DC_MOTORS

  writeAllMotors(MINCOMMAND);
  delay(300);
} // initOutput


void mixTable() {
  int16_t maxMotor;
  uint8_t i;

#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + (YAW_DIRECTION) * axisPID[YAW]*Z

#if defined(QUADP)
  motor[0] = PIDMIX( 0,+1,-1); //REAR
  motor[1] = PIDMIX(-1, 0,+1); //RIGHT
  motor[2] = PIDMIX(+1, 0,+1); //LEFT
  motor[3] = PIDMIX( 0,-1,-1); //FRONT
#elif defined(QUADX)
  motor[0] = PIDMIX(-1,+1,-1); //REAR_R
  motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
  motor[2] = PIDMIX(+1,+1,+1); //REAR_L
  motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
#endif

// Preserve attitude control headroom

#if defined(USE_HEADROOM)
  maxMotor = motor[0];
  for (i = 1; i< NUMBER_MOTOR; i++)
    if (motor[i] > maxMotor) maxMotor = motor[i];

  for(i = 0; i< NUMBER_MOTOR; i++) {
    if (maxMotor > MAXTHROTTLE)
      motor[i] -= maxMotor - MAXTHROTTLE;

    motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE); 

    if (!f.ARMED)
      motor[i] = MINCOMMAND;
    else
      if (rcData[THROTTLE] < MINCHECK)
        motor[i] = EFF_MINCOMMAND;

  }
#else
  for(i = 0; i< NUMBER_MOTOR; i++)
    if (!f.ARMED)
      motor[i] = MINCOMMAND;
    else
      if (rcData[THROTTLE] < MINCHECK)
        motor[i] = EFF_MINCOMMAND;
#endif
} // mixTable

























