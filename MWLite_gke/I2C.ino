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
 
// I2C general functions

#define I2C_NACK 1
#define I2C_ACK 0

void i2cInit(void) {
#if defined(INTERNAL_I2C_PULLUPS)
  I2C_PULLUPS_ENABLE;
#else
  I2C_PULLUPS_DISABLE;
#endif
  TWSR = 0;                                  // no prescaler => prescaler = 1
  TWBR = ((F_CPU / 400000) - 16) / 2;       // change the I2C clock rate
  TWCR = 1 << TWEN;                         // enable twi module, no interrupt
} // i2cInit

void i2cRepStart(uint8_t addr) {
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) ; // send REPEAT START condition
  i2cWaitTransmission();                       // wait until transmission completed
  TWDR = addr;                              // send device address
  TWCR = (1 << TWINT) | (1 << TWEN);
  i2cWaitTransmission();                       // wail until transmission completed
} // i2cRepStart

void i2cStop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
} // i2cStop

void i2cWriteByte(uint8_t data ) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1 << TWINT) | (1 << TWEN);
  i2cWaitTransmission();
} // i2cWriteByte

uint8_t i2cReadByte(uint8_t ack) {
  uint8_t r;

  TWCR = (1 << TWINT) | (1 << TWEN) | (ack? 0 : (1 << TWEA));
  i2cWaitTransmission();
  r = TWDR;
  if (ack) i2cStop();
  return (r);
} // i2cReadByte


void i2cWaitTransmission(void) {
  uint16_t count = 255;

  while (!(TWCR & (1 << TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      i2cErrors++;
      break;
    }
  }
} // i2cWaitTransmission

boolean i2cResponse(uint8_t i2cx, uint8_t d) { 
  uint16_t i2cCount;
  uint8_t val;

  if (i2cx == I2C_SW) 
    return(softi2cResponse(d));
  else {
    i2cCount = i2cErrors;
    i2cReadToBuf(i2cx, d, 0, &val, 1);
    return(i2cErrors == i2cCount);
  }  
} // i2cResponse

void i2cReadToBuf(uint8_t i2cx, uint8_t d, uint8_t addr, uint8_t *b, uint8_t l) {
  uint8_t i;

  if (i2cx == I2C_SW) 
    softi2cReadToBuf(d, addr, b, l);
  else {  
    i2cRepStart(d);
    i2cWriteByte(addr);
    i2cRepStart(d | 1);
    for (i = 0; i < (l-1); i++)	
      b[i] = i2cReadByte(I2C_ACK);
    b[l-1] = i2cReadByte(I2C_NACK);
    i2cStop();
  } 
} // i2cReadToBuf

void i2cReadi16vAtAddr(uint8_t i2cx, uint8_t d, uint8_t addr, int16_t * v, uint8_t l, boolean h) {
  uint8_t b, c, S[32];

  i2cReadToBuf(i2cx, d, addr, S, l << 1); 

  c = 0;
  for ( b = 0; b < l; b++) {
    if ( h )
      v[b] = ((int16_t)(S[c]) << 8) | S[c+1];
    else
      v[b] = ((int16_t)(S[c+1]) << 8) | S[c];
    c += 2;
  }
} // i2cReadi16vAtAddr


void i2cWriteReg(uint8_t i2cx, uint8_t d, uint8_t addr, uint8_t val) {

  if (i2cx == I2C_SW) 
    softi2cWriteReg(d, addr, val);
  else {
    i2cRepStart(d); 
    i2cWriteByte(addr); 
    i2cWriteByte(val);
    i2cStop();
  }
} // i2cWriteReg

void i2cWrite(uint8_t i2cx, uint8_t d, uint8_t val) {

  if (i2cx == I2C_SW) 
    softi2cWriteReg(d, val);
  else { 
    i2cRepStart(d);  
    i2cWriteByte(val);
    i2cStop();
  }
} // i2cWriteReg


uint8_t i2cReadReg(uint8_t i2cx, uint8_t d, uint8_t addr) {
  uint8_t val;

  if (i2cx == I2C_SW) 
    softi2cReadToBuf(d, addr, &val, 1); 
  else
    i2cReadToBuf(i2cx, d, addr, &val, 1);
  return val;
} // i2cReadReg


//______________________________________________________________________________________________

// Soft I2C routines  from UAVX

#define I2C_SDA_PIN     3 // MISO
#define I2C_5V_PIN      2 // MOSI CAUTION 10mA @5V/5mA @3.3V
#define I2C_SCL_PIN     1 // SCK

#define I2C_SDA_SW	((PINB&(1<<I2C_SDA_PIN)) != 0)
#define I2C_SCL_SW	((PINB&(1<<I2C_SCL_PIN)) != 0)

#define I2C_DATA_LOW	{DDRB|=(1<<I2C_SDA_PIN);PORTB&=~(1<<I2C_SDA_PIN);} 
#define I2C_DATA_FLOAT	{DDRB&=~(1<<I2C_SDA_PIN);PORTB|=(1<<I2C_SDA_PIN);} 

#define I2C_CLK_LOW	{DDRB|=(1<<I2C_SCL_PIN);PORTB&=~(1<<I2C_SCL_PIN);} 
#define I2C_CLK_FLOAT	{DDRB&=~(1<<I2C_SCL_PIN);PORTB|=(1<<I2C_SCL_PIN);} 

#define T_LOW_STA		
#define T_HD_STA	{Delay1TCY();Delay1TCY();Delay1TCY();} // 4.0/0.6uS
#define T_HD_DAT	// 5.0/0.0uS
#define T_SU_DAT	// 250/100nS

#define T_HIGH_R	// 4.0/0.6uS	
#define T_LOW_R		// 4.7/1.3uS
#define T_HIGH_W	// 4.0/0.6uS
#define T_LOW_W		// 4.7/1.3uS

#define T_HIGH_ACK_R    //			
#define T_HIGH_ACK_W    //	
#define T_LOW_STP       //		
#define T_SU_STO	{Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY();}
#define T_BUF		Delay10TCYx(5)

#define Delay1TCY()     delayMicroseconds(2)
#define Delay10TCYx(n)  delayMicroseconds(20*n)

void softi2cPower(void) { // CAUTION - perhaps OK for one sensor
#if defined(USE_5V_ON_MOSI) & !defined(STANDARD_RX)
    DDRB|=(1<<I2C_5V_PIN);
    PORTB|=(1<<I2C_5V_PIN);
#endif
} // softi2cPower

boolean softi2cWaitClkHi(void) {
  uint8_t s;

  //  Delay1TCY(); // setup
  //  Delay1TCY();
  Delay1TCY();

  I2C_CLK_FLOAT;	// set SCL to input, output a high
  s = 1;
  while( !I2C_SCL_SW )	// timeout wraparound through 255 to 0 0.5mS @ 40MHz
    if( ++s == (uint8_t)0 ) {
      i2cErrors++;
      return (false);
    }
  return( true );
} // softi2cWaitClkHi

uint8_t softi2cRepStart(uint8_t d) {
  boolean r;

  I2C_DATA_FLOAT;
  r = softi2cWaitClkHi();
  I2C_DATA_LOW;
  T_HD_STA;
  I2C_CLK_LOW;
  T_LOW_STA;

  return(softi2cWriteByte(d));
} // softi2cStart

void softi2cStop(void) {
  boolean r;

  T_LOW_STP;
  I2C_DATA_LOW;
  r = softi2cWaitClkHi();
  T_SU_STO;
  I2C_DATA_FLOAT;
  T_BUF;

} // softi2cStop 

uint8_t softi2cReadByte(uint8_t ack) {
  uint8_t s, d;

  I2C_DATA_FLOAT;
  d = 0;
  s = 8;
  do {
    if( softi2cWaitClkHi() ) { 
      d <<= 1;
      if( I2C_SDA_SW ) d |= 1;
      T_HIGH_R;
      I2C_CLK_LOW;
      T_LOW_R;
    } 
    else {
      i2cErrors++;
      return(false);
    }
  } 
  while ( --s );

  if (ack)
    PORTB|=(1 << I2C_SDA_PIN);
  else
    PORTB&=~(1 << I2C_SDA_PIN);

  DDRB|=(1 << I2C_SDA_PIN);

  if( softi2cWaitClkHi() ) {
    T_HIGH_ACK_R;
    I2C_CLK_LOW;
    return(d);
  } 
  else {
    i2cErrors++;
    return(false);
  }

} // softi2cReadByte

uint8_t softi2cWriteByte(uint8_t d) {
  uint8_t s, dd;

  dd = d; 
  s = 8;
  do {
    if( dd & 0x80 )
      I2C_DATA_FLOAT
else
  I2C_DATA_LOW

    if( softi2cWaitClkHi() ) { 	
    T_HIGH_W;
    I2C_CLK_LOW;
    T_LOW_W;
    dd <<= 1;
  } 
  else {
    i2cErrors++;
    return(I2C_NACK);
  }
} 
while ( --s );

I2C_DATA_FLOAT;
if( softi2cWaitClkHi() )
s = I2C_SDA_SW;
else {
  i2cErrors++;
  return(I2C_NACK);
}	
T_HIGH_ACK_W;
I2C_CLK_LOW;

return(s);
} // softi2cWriteByte


uint8_t zzzsofti2cReadByteAtAddr(uint8_t d, uint8_t addr) {
  uint8_t r;

  softi2cRepStart(d);
  softi2cWriteByte(addr); 
  softi2cRepStart(d | 1);
  r = softi2cReadByte(I2C_NACK);
  softi2cStop();

  return (r);
} // softi2cReadByteAtAddr

void softi2cReadToBuf(uint8_t d,  uint8_t addr, uint8_t * b, uint8_t l) {
  uint8_t i;

  softi2cRepStart(d);
  softi2cWriteByte(addr);
  softi2cRepStart(d | 1); 
  for (i = 0; i<(l-1);i++)	
    b[i] = softi2cReadByte(I2C_ACK);
  b[l-1] = softi2cReadByte(I2C_NACK);
  softi2cStop();

} // softi2cReadToBuff


void softi2cWriteReg(uint8_t d, uint8_t addr, uint8_t v) {

  softi2cRepStart(d); 
  softi2cWriteByte(addr);
  softi2cWriteByte(v);
  softi2cStop();

} // softi2cWriteReg

void softi2cWriteReg(uint8_t d, uint8_t v) {

  softi2cRepStart(d); 
  softi2cWriteByte(v);
  softi2cStop();

} // softi2cWrite


boolean softi2cResponse(uint8_t d) {
  boolean r;

  r = softi2cRepStart(d) == I2C_ACK;
  softi2cStop();

  return (r);
} // softi2cResponse














