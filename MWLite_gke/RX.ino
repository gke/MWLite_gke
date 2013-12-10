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

// Rx


#define RC_GOOD_BUCKET_MAX 20
#define RC_GOOD_RATIO 4

#define WidthOK(w) ((w>=900) && (w<=2200))

static uint32_t rcTimeuS  = 0;
static boolean rcFrameOK = false;
static int16_t  rcGlitches = 0;
static int16_t failsafeEvents = 0;
static int16_t failsafeCnt = 0;

#if defined(SPEKTRUM)
#include <wiring.c>  //Auto-included by the Arduino core... but we need it sooner. 
#endif

#if defined(SBUS)
volatile uint16_t rcValue[RC_CHANS] = {
  1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#elif defined(SPEKTRUM)

volatile uint8_t  spekFrameFlags;
volatile uint32_t spekTimeLast;

volatile uint16_t rcValue[RC_CHANS] = {
  1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#elif defined(SERIAL_SUM_PPM)
volatile uint16_t rcValue[RC_CHANS]= {
  1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#elif defined(FLYSKY)
volatile uint16_t rcValue[RC_CHANS]= {
  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // interval [1000;2000]  
#else
volatile uint16_t rcValue[RC_CHANS] = {
  1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#endif

#if defined(SERIAL_SUM_PPM) //Channel order for PPM SUM RX Configs
static uint8_t rcChannel[RC_CHANS] = {
  SERIAL_SUM_PPM};
#elif defined(SBUS) //Channel order for SBUS RX Configs
// for 16 + 2 Channels SBUS. The 10 extra channels 8->17 are not used by MultiWii, but it should be easy to integrate them.
static uint8_t rcChannel[RC_CHANS] = {
  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11,12,13,14,15,16,17};
static uint16_t sbusIndex=0;
#elif defined(SPEKTRUM)
static uint8_t rcChannel[RC_CHANS] = {
  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11};
#elif defined(FLYSKY)
static uint8_t rcChannel[RC_CHANS] = {
  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4}; 
#else // Standard Channel order
static uint8_t rcChannel[RC_CHANS]  = {
  ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN};
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {
  PCINT_RX_BITS}; // if this slowes the PCINT readings we can switch to a define for each pcint bit
#endif

//_____________________________________________________________________________________________

// Configure Pins

void configureReceiver(void) {

#if defined(STANDARD_RX)
  // PCINT activation
  for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){ 
    PCINT_RX_PORT |= PCInt_RX_Pins[i];
    PCINT_RX_MASK |= PCInt_RX_Pins[i];
  }
  PCICR = PCIR_PORT_BIT;

  //atmega32u4's Specific RX Pin Setup
  //Throttle on pin 7
  DDRE &= ~(1 << 6); // pin 7 to input
  PORTE |= (1 << 6); // enable pullups
  EIMSK |= (1 << INT6); // enable interuppt
  EICRB |= (1 << ISC60);
  // Aux2 pin on PBO (D17/RXLED)
#if defined(RCAUX2PIND17)
  DDRB &= ~(1 << 0); // set D17 to input 
#endif
  // Aux2 pin on PD2 (RX0)
#if defined(RCAUX2PINRXO)
  DDRD &= ~(1 << 2); // RX to input
  PORTD |= (1 << 2); // enable pullups
  EIMSK |= (1 << INT2); // enable interuppt
  EICRA |= (1 << ISC20);
#endif

#endif // STANDARD_RX

#if defined(SERIAL_SUM_PPM)
  PPM_PIN_INTERRUPT; 
#endif

#if defined (SPEKTRUM)
  SerialOpen(SPEK_SERIAL_PORT,115200);
#elif defined(SBUS)
  SerialOpen(1,100000);
#elif defined(FLYSKY)
  initFlySky();
#endif

} // configureReceiver

void failsafeUpdate(void) {

  rcNewValues = rcFrameOK;

  if( rcFrameOK) 
    failsafeCnt++;
  else {
    rcGlitches++;
#if defined(DEBUG_RC)
    debug[0] = rcGlitches;
#endif
    failsafeCnt -= RC_GOOD_RATIO;
  }
  failsafeCnt = Limit1(failsafeCnt, RC_GOOD_BUCKET_MAX);
  inFailsafe = failsafeCnt <= 0;
  rcFrameOK = true;
} // failsafeUpdate

//_____________________________________________________________________________________________

// Standard Parallel PPM

#if defined(STANDARD_RX)

static uint8_t mask;
static uint8_t pin;
static int16_t Width; // can be negative on wraparound
static uint32_t edgeTime[8];


// predefined PC pin block (thanks to lianj)  - Version without failsafe
void inline rxPinCheck(uint8_t pin_pos, uint8_t rc_value_pos) {

  if (mask & PCInt_RX_Pins[pin_pos])                           
    if (!(pin & PCInt_RX_Pins[pin_pos])) 
    {
      if (pin_pos == 0) 
        failsafeUpdate();

      Width = NowuS - edgeTime[pin_pos]; 
      if (WidthOK(Width)) 
        rcValue[rc_value_pos] = Width;
      else 
        rcFrameOK &= false; 
    } 
    else 
      edgeTime[pin_pos] = NowuS; 
} // rxPinCheck

// port change Interrupt
ISR(RX_PC_INTERRUPT) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a RX input pin
  static uint8_t PCintLast;

  pin = RX_PCINT_PIN_PORT; // RX_PCINT_PIN_PORT indicates the state of each PIN for the arduino port dealing with Ports digital pins

  mask = pin ^ PCintLast;   // doing a ^ between the current interrupt and the last one indicates which pin changed
  NowuS = micros(); 
  sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical 
  // and can be interrupted safely
  PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]

#if (PCINT_PIN_COUNT > 0)
  rxPinCheck(0,2);
#endif
#if (PCINT_PIN_COUNT > 1)
  rxPinCheck(1,4);
#endif
#if (PCINT_PIN_COUNT > 2)
  rxPinCheck(2,5);
#endif
#if (PCINT_PIN_COUNT > 3)
  rxPinCheck(3,6);
#endif
#if (PCINT_PIN_COUNT > 4)
  rxPinCheck(4,7);
#endif
#if (PCINT_PIN_COUNT > 5)
  rxPinCheck(5,0);
#endif
#if (PCINT_PIN_COUNT > 6)
  rxPinCheck(6,1);
#endif
#if (PCINT_PIN_COUNT > 7)
  rxPinCheck(7,3);
#endif

} // RX_PC_INTERRUPT

// atmega32u4's Throttle & Aux2 Pin

// throttle
ISR(INT6_vect){ 
  static uint16_t NowuS, Width;
  static uint16_t PrevuS = 0;
  NowuS = micros();  
  if(!(PINE & (1<<6))){
    Width = NowuS - PrevuS;
    if(WidthOK(Width))
      rcValue[3] = Width;
    else 
      rcFrameOK &= false;
  }
  else PrevuS = NowuS; 
}

// Aux 2
#if defined(RCAUX2PINRXO)
ISR(INT2_vect){
  static uint16_t NowuS, Width;
  static uint16_t PrevuS = 0; 
  NowuS = micros();  
  if(!(PIND & (1<<2))){
    Width = NowuS - PrevuS;
    if(WidthOK(Width)) 
      rcValue[7] = Width;
    else 
      rcFrameOK &= false;
  }
  else PrevuS = NowuS;
}
#endif  
#endif // STANDARD_RX

//_____________________________________________________________________________________________

// Compound PPM (CPPM)

#if defined(SERIAL_SUM_PPM)

ISR(INT6_vect){
  rxInt();
} // INT6_vect

void rxInt(void) {
  uint16_t NowuS, Width;
  static uint16_t PrevuS = 0;
  static uint8_t chan = 0;

  NowuS = micros();
  sei();
  Width = NowuS - PrevuS;
  PrevuS = NowuS;
  if( Width > 3000) { 
    if (chan >= 4) 
      failsafeUpdate();
    chan = 0;
  }
  else
    if (chan < RC_CHANS) { 
      if (WidthOK(Width))
        rcValue[chan] = Width;
      else 
        rcFrameOK &= false;

      chan++;
    }
} // rxInt
#endif // SERIAL_SUM_PPM

//_____________________________________________________________________________________________

// SBus

#if defined(SBUS)
void  readSBus(void){
#define SBUS_SYNCBYTE 0x0F // Not 100% sure: at the beginning of coding it was 0xF0 !!!
  static uint16_t sbus[25]={
    0                                                                                                                                                                                                                };
  while(SerialAvailable(1)){
    int val = SerialRead(1);
    if( sbusIndex == 0 && val != SBUS_SYNCBYTE)
      continue;
    sbus[sbusIndex++] = val;
    if(sbusIndex==25){
      sbusIndex=0;
      rcValue[0]  = ((sbus[1]|sbus[2]<< 8) & 0x07FF)/2+976; // Perhaps you may change the term "/2+976" -> center will be 1486
      rcValue[1]  = ((sbus[2]>>3|sbus[3]<<5) & 0x07FF)/2+976; 
      rcValue[2]  = ((sbus[3]>>6|sbus[4]<<2|sbus[5]<<10) & 0x07FF)/2+976; 
      rcValue[3]  = ((sbus[5]>>1|sbus[6]<<7) & 0x07FF)/2+976; 
      rcValue[4]  = ((sbus[6]>>4|sbus[7]<<4) & 0x07FF)/2+976; 
      rcValue[5]  = ((sbus[7]>>7|sbus[8]<<1|sbus[9]<<9) & 0x07FF)/2+976;
      rcValue[6]  = ((sbus[9]>>2|sbus[10]<<6) & 0x07FF)/2+976; 
      rcValue[7]  = ((sbus[10]>>5|sbus[11]<<3) & 0x07FF)/2+976; // & the other 8 + 2 channels if you need them
      //The following lines: If you need more than 8 channels, max 16 analog + 2 digital. Must comment the not needed channels!
      rcValue[8]  = ((sbus[12]|sbus[13]<< 8) & 0x07FF)/2+976; 
      rcValue[9]  = ((sbus[13]>>3|sbus[14]<<5) & 0x07FF)/2+976; 
      rcValue[10] = ((sbus[14]>>6|sbus[15]<<2|sbus[16]<<10) & 0x07FF)/2+976; 
      rcValue[11] = ((sbus[16]>>1|sbus[17]<<7) & 0x07FF)/2+976; 
      rcValue[12] = ((sbus[17]>>4|sbus[18]<<4) & 0x07FF)/2+976; 
      rcValue[13] = ((sbus[18]>>7|sbus[19]<<1|sbus[20]<<9) & 0x07FF)/2+976; 
      rcValue[14] = ((sbus[20]>>2|sbus[21]<<6) & 0x07FF)/2+976; 
      rcValue[15] = ((sbus[21]>>5|sbus[22]<<3) & 0x07FF)/2+976; 
      // now the two Digital-Channels
      if ((sbus[23]) & 0x0001)       rcValue[16] = 2000; 
      else rcValue[16] = 1000;
      if ((sbus[23] >> 1) & 0x0001)  rcValue[17] = 2000; 
      else rcValue[17] = 1000;

      // Failsafe: there is one Bit in the SBUS-protocol (Byte 25, Bit 4) whitch is the failsafe-indicator-bit
      rcFrameOK = !((sbus[23] >> 3) & 0x0001);
      failsafeUpdate();
    }
  }        
} // readSBus
#endif // SBUS

//_____________________________________________________________________________________________

// Spektrum

#if defined(SPEKTRUM)
void readSpektrum(void) {
  uint32_t spekInterval;
  int16_t Temp;
  uint8_t i, b, bh, bl, spekChannel;

  if ((!f.ARMED) && ( SerialPeek(SPEK_SERIAL_PORT) == '$')) {
    while (SerialAvailable(SPEK_SERIAL_PORT)) {
      serialCom();
      delay (10);
    }
    return;
  } //End of: Is it the GUI?

  while (SerialAvailable(SPEK_SERIAL_PORT) > SPEK_FRAME_SIZE) // More than a frame?  More bytes implies we weren't called for multiple frame times.  
    // We do not want to process 'old' frames in the buffer.
    for (i = 0; i < SPEK_FRAME_SIZE; i++) 
      SerialRead(SPEK_SERIAL_PORT); //Toss one full frame of bytes.

  if (spekFrameFlags == 0x01) //The interrupt handler saw at least one valid frame start since we were last here. 
    if (SerialAvailable(SPEK_SERIAL_PORT) == SPEK_FRAME_SIZE) {  //A complete frame? If not, we'll catch it next time we are called. 
      SerialRead(SPEK_SERIAL_PORT); 
      SerialRead(SPEK_SERIAL_PORT); // Eat the header bytes 
      for (b = 2; b < SPEK_FRAME_SIZE; b += 2) {
        bh = SerialRead(SPEK_SERIAL_PORT);
        bl = SerialRead(SPEK_SERIAL_PORT);
        spekChannel = 0x0F & (bh >> SPEK_CHAN_SHIFT);
        if (spekChannel < RC_CHANS) {
          Temp =  ((((uint16_t)(bh & SPEK_CHAN_MASK) << 8) + bl) SPEK_DATA_SHIFT);
#if defined(USE_GKE_DM9_SPEKTRUM_SCALING) 
          rcValue[spekChannel] = (Temp * 1.18) + 913; // direct measurement
#elif defined (SPEKTRUM_RTF_SCALING)
          Temp = Temp + 988 - MIDRC;
          if ((rcChannel[spekChannel] == YAW) || (rcChannel[spekChannel] == ROLL))
            Temp = -Temp;
          Temp = Temp * 1.65 + MIDRC;
          rcValue[spekChannel] = constrain(Temp, 1000, 2000);
#else
          rcValue[spekChannel] = Temp + 988;
#endif
        }  
      }
      failsafeUpdate();
      spekFrameFlags = 0x00;
    } 
    else { //Start flag is on, but not enough bytes means there is an incomplete frame in buffer.  
      // This could be OK, if we happened to be called in the middle of a frame.  Or not, if it has 
      // been a while since the start flag was set.
      spekInterval = micros() - spekTimeLast;
      if (spekInterval > 2500) { 
        rcGlitches++;
        spekFrameFlags = 0; 
      } 
    }
} // readSpektrum

// Code-based Spektrum satellite receiver binding for the HobbyKing Pocket Quad
// Spektrum Binding code due to Andrew L.
// navigation07@gmail.com

// Merge idea due to davidea using standard bind link between GND and THROTTLE at startup

// Bind Mode Table:
// 2 low pulses: DSM2 1024/22ms 
// 3 low pulses: no result
// 4 low pulses: DSM2 2048/11ms
// 5 low pulses: no result
// 6 low pulses: DSMX 22ms
// 7 low pulses: no result
// 8 low pulses: DSMX 11ms

#if (SPEKTRUM == 1024)
#define SPEK_BIND_PULSES 2
#elif
#define SPEK_BIND_PULSES 4
#endif

void checkSpektrumBinding(void) { 
  uint8_t pulse;

  pinMode(7, INPUT);           // THR pin as input
  digitalWrite(7, HIGH);       // turn on pullup resistors

  if (!digitalRead(7)){

    pinMode(0, OUTPUT);       // Tx pin for satellite
    digitalWrite(0, HIGH);

    pinMode(0, OUTPUT);

    digitalWrite(0, HIGH);
    delayMicroseconds(116);

    for (pulse = 0; pulse < SPEK_BIND_PULSES; pulse++) { 
      digitalWrite(0, LOW); 
      delayMicroseconds(116);
      digitalWrite(0, HIGH);
      delayMicroseconds(116);
    }

    pinMode(0, INPUT);

    while(true) {
      LED_BLUE_TOGGLE;
      delay(500);    
    }
  } 
} // checkSpektrumBinding

#endif // SPEKTRUM


//_____________________________________________________________________________________________

// FlySky using XL7105 Transceiver

#if defined(FLYSKY)

NOT COMMISSIONED

// **********************************************************
// ******************   Flysky Rx Code   *******************
//               by midelic on RCgroups.com 
//   Thanks to PhracturedBlue,ThierryRC,Dave1993,and the team
//    of OpenLRS project.
//    Many thanks to Philip Cowzer(SadSack)for testing
//   Rewrite by Prof Greg Egan 2013
// **********************************************************
// Date:before my bet with Dave1993 expired(April-July 2013). 
// Hardware:mega328(Promini)+XL7105+mpu6050+MOSFET(brushed)
// Project Forum : Not yet dedicated project forum
// Google Code Page:Not yet,waiting for somebody to make one
// Info and questions at:
// http://www.rcgroups.com/forums/showthread.php?t=1710948
// **********************************************************

static const uint8_t XL7105Regs[] = {
  0xff, 0x42, 0x00, 0x14, 0x00, 0xff, 0xff ,0x00, 0x00, 0x00, 0x00, 0x01, 0x21, 0x05, 0x00, 0x50,
  0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00, 0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,
  0x13, 0xc3, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
  0x01, 0x0f, 0xff,
};
static const uint8_t flyskyChannels[16][16] = {
  {
    0x0a, 0x5a, 0x14, 0x64, 0x1e, 0x6e, 0x28, 0x78, 0x32, 0x82, 0x3c, 0x8c, 0x46, 0x96, 0x50, 0xa0                                                }
  ,
  {
    0xa0, 0x50, 0x96, 0x46, 0x8c, 0x3c, 0x82, 0x32, 0x78, 0x28, 0x6e, 0x1e, 0x64, 0x14, 0x5a, 0x0a                                                }
  ,
  {
    0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x46, 0x96, 0x1e, 0x6e, 0x3c, 0x8c, 0x28, 0x78, 0x32, 0x82                                                }
  ,
  {
    0x82, 0x32, 0x78, 0x28, 0x8c, 0x3c, 0x6e, 0x1e, 0x96, 0x46, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a                                                }
  ,
  {
    0x28, 0x78, 0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96                                                }
  ,
  {
    0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a, 0x78, 0x28                                                }
  ,
  {
    0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96, 0x14, 0x64                                                }
  ,
  {
    0x64, 0x14, 0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50                                                }
  ,
  {
    0x50, 0xa0, 0x46, 0x96, 0x3c, 0x8c, 0x28, 0x78, 0x0a, 0x5a, 0x32, 0x82, 0x1e, 0x6e, 0x14, 0x64                                                }
  ,
  {
    0x64, 0x14, 0x6e, 0x1e, 0x82, 0x32, 0x5a, 0x0a, 0x78, 0x28, 0x8c, 0x3c, 0x96, 0x46, 0xa0, 0x50                                                }
  ,
  {
    0x46, 0x96, 0x3c, 0x8c, 0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64                                                }
  ,
  {
    0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50, 0x8c, 0x3c, 0x96, 0x46                                                }
  ,
  {
    0x46, 0x96, 0x0a, 0x5a, 0x3c, 0x8c, 0x14, 0x64, 0x50, 0xa0, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82                                                }
  ,
  {
    0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0xa0, 0x50, 0x64, 0x14, 0x8c, 0x3c, 0x5a, 0x0a, 0x96, 0x46                                                }
  ,
  {
    0x46, 0x96, 0x0a, 0x5a, 0x50, 0xa0, 0x3c, 0x8c, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64                                                }
  ,
  {
    0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0x8c, 0x3c, 0xa0, 0x50, 0x5a, 0x0a, 0x96, 0x46                                                }
  ,
};

#define  GIO_HI PORTD |=0x40//D6 
#define  GIO_INP_HI (PIND & 0x40) == 0x40 //D6 input
#define  GIO_INP_LO (PIND & 0x40) == 0x00 //D6

#define NOP() __asm__ __volatile__("nop")

static uint32_t ID;
static uint8_t txID[4];
static uint8_t chanrow;
static uint8_t chancol;
static uint8_t chanoffset;
static uint8_t channel;
static word counter1=512;
static uint8_t currID[4];
static uint8_t XL7105Packet[21];
static uint16_t Width;

void initFlySky(void){
  uint8_t i;

  _spiInit(); 

  delay(10); // XL7105 wakeup
  resetXL7105();
  writeXL7105ID(0x5475c52a); 

  //readXL7105ID();
  //Serial.print(currID[0],HEX);
  //Serial.print(currID[1],HEX);
  //Serial.print(currID[2],HEX);
  //Serial.print(currID[3],HEX);

  for (i = 0; i < 0x33; i++)
    if(XL7105Regs[i] != 0xff)
      _spiWrite(i, XL7105Regs[i]);

  _spiWrite(0x02, 0x01);
  while(_spiRead(0x02)) 
    if(_spiRead(0x22) & 0x10){//do nothing
    }
  _spiWrite(0x24,0x13);
  _spiWrite(0x26,0x3b);
  _spiWrite(0x0f,0x00); // channel 0
  _spiWrite(0x02,0x02);
  while(_spiRead(0x02))
    if(_spiRead(0x25)&0x08){ // do nothing
    }

  _spiWrite(0x0F,0xa0);
  _spiWrite(0x02,0x02);
  while(_spiRead(0x02))
    if(_spiRead(0x25) & 0x08){//do nothing
    }
  _spiWrite(0x25,0x08);

  bindFlysky();
  //id=(XL7105Packet[1] | ((uint32_t)XL7105Packet[2]<<8) | ((uint32_t)XL7105Packet[3]<<16) | ((uint32_t)XL7105Packet[4]<<24));
  ID = (txID[0] | ((uint32_t)txID[1]<<8) | ((uint32_t)txID[2]<<16) | ((uint32_t)txID[3]<<24));
  chanrow = ID & 15;
  chanoffset = (ID & 0xff) >> 4;
  chancol = 0;
} // initFlySky

boolean syncXL7105(void) { // FlySky header match  
  return (XL7105Packet[1]==txID[0]) && (XL7105Packet[2]==txID[1])&& (XL7105Packet[3]==txID[2])&& (XL7105Packet[4]==txID[3]);  
} // syncXL7105

void readFlySky(void) {
  uint32_t NowuS;
  uint8_t v;
  uint8_t i;

  channel=flyskyChannels[chanrow][chancol]-1-chanoffset;
  _spiStrobe(0xa0);
  _spiStrobe(0xf0);
  _spiWrite(0x0f,channel);
  _spiStrobe(0xc0); 
  chancol = (chancol + 1) & 15;

  NowuS = micros();

  if((micros() - NowuS) > 2250){
    LED_BLUE_OFF;
    chancol = (chancol + 1) & 15;
    channel=flyskyChannels[chanrow][chancol]-1-chanoffset;
  }
  else
    if (GIO_INP_HI) {
      if (_spiRead(0x00) & 0b01100000) { // CRF & CRC

        readXL7105Packet();

        if (syncXL7105()) {
          LED_BLUE_ON;
          for (i = 0; i < RC_CHANS; i++){
            Width=((XL7105Packet[6 + (i<<1)] << 8) + XL7105Packet[5 + (i<<1)]);
            if (WidthOK(Width))
              rcValue[i] = Width;
          } 	  
        }
      }
    }
} // readFlySky

void bindFlysky(void) {
  uint8_t v;

  _spiStrobe(0xa0);
  _spiStrobe(0xf0);
  _spiWrite(0x0f,0x00); // listen on channel 0
  _spiStrobe(0xc0);

  while(counter1){ // 5 sec.
    delay(10);
    if (bitRead(counter1, 2) == 1)
      LED_BLUE_ON;
    if(bitRead(counter1, 2) == 0)
      LED_BLUE_OFF;

    if (GIO_INP_HI){
      if (_spiRead(0x00) & 0b01100000) { // CRF & CRC

        readXL7105Packet();

        txID[0] = XL7105Packet[1];
        txID[1] = XL7105Packet[2];
        txID[2] = XL7105Packet[3];
        txID[3] = XL7105Packet[4];
      }
      else
        bindFlysky();
      break;
    }
    else
      counter1--;
  }
  LED_BLUE_OFF;
} // bindFlySky

void readXL7105ID(void){
  _spiReadBytes(0x46, currID, 4);  
} // readXL7105ID

void readXL7105Packet(void) {
  _spiReadBytes(0x45, XL7105Packet, 21);
} // ReadPacket

void resetXL7105(void) {
  _spiWrite(0x00,0x00); 
} // resetXL7105

// SPI routines

//Dave/mwii pins configuration
#define GIO_pin 6 // GIO-D6
#define SDI_pin 5 // SDIO-D5 
#define SCLK_pin 4 // SCK-D4
#define CS_pin 2 // CS-D2
//------------------
#define  CS_HI PORTD |= 0x04 //D2
#define  CS_LO PORTD &= 0xFB //D2
#define  SCK_HI PORTD |= 0x10//D4
#define  SCK_LO PORTD &= 0xEF//D4
#define  SDI_HI PORTD |= 0x20 //D5
#define  SDI_LO PORTD &= 0xDF //D5

#define  SDI_INP (PIND & 0x20) == 0x20 //D5

void writeXL7105ID(uint32_t ID) {
  CS_LO;
  _spiWriteByte(0x06);
  _spiWriteByte((ID >> 24) & 0xff); 
  _spiWriteByte((ID >> 16) & 0xff);
  _spiWriteByte((ID >> 8) & 0xff);
  _spiWriteByte((ID >> 0) & 0xff);
  CS_HI;
} // writeXL7105ID

void _spiReadBytes(uint8_t a, uint8_t * v, uint8_t l) {
  uint8_t i;

  CS_LO;
  _spiWriteByte(a);
  for (i = 0; i < l; i++) 
    v[i] = _spiReadByte();
  CS_HI;
} // _spiReadBytes

void _spiWriteByte(uint8_t v) {  
  uint8_t n=8; 

  SCK_LO;
  SDI_LO;
  while(n--) {
    if(v & 0x80)
      SDI_HI;
    else 
      SDI_LO;
    SCK_HI;
    NOP();
    SCK_LO;
    v <<= 1;
  }
  SDI_HI;
} // _spiWriteByte

void _spiWrite(uint8_t a, uint8_t v) {
  CS_LO;
  _spiWriteByte(a); 
  NOP();
  _spiWriteByte(v);  
  CS_HI;
} // _spiWrite

uint8_t _spiReadByte(void) {
  uint8_t r;
  uint8_t i;
  r=0;
  pinMode(SDI_pin,INPUT); 
  //SDI_HI;
  for(i = 0; i < 8; i++) {                    
    if(SDI_INP)  ///if SDIO = 1 
      r=(r<<1)|0x01;
    else
      r <<= 1;
    SCK_HI;
    NOP();
    SCK_LO;
    NOP();
  }
  pinMode(SDI_pin, OUTPUT);
  return (r);
} // _spiRead

uint8_t _spiRead(uint8_t a) { 
  uint8_t r;

  CS_LO;
  a |=0x40;
  _spiWriteByte(a);
  r = _spiReadByte();  
  CS_HI;
  return(r); 
} // _spiRead

void _spiStrobe(uint8_t a) {
  CS_LO;
  _spiWriteByte(a);
  CS_HI;
} // _spiStrobe

void _spiInit(void) {

  pinMode(GIO_pin, INPUT_PULLUP);
  pinMode(SDI_pin, OUTPUT);
  pinMode(SCLK_pin, OUTPUT);
  pinMode(CS_pin, OUTPUT);
  CS_HI; 
  SDI_HI; 
  SCK_LO; 

} // _spiInit

#endif // FLYSKY


//_____________________________________________________________________________________________

// General 

uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
#if defined(SPEKTRUM) || defined(FLYSKYRC)
  readSpektrum();
  if (chan < RC_CHANS)
    data = rcValue[rcChannel[chan]];
  else data = MIDRC;
#else
  uint8_t oldSREG;
  oldSREG = SREG; 
  cli(); // disable interrupts
  data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
  SREG = oldSREG; // restore interrupt state
#endif
  return data; // We return the value correctly copied when the IRQ's where disabled
} // readRawRC

void getRCInput(void) {
  uint8_t chan;

#if defined(USE_RC_FILTER)

  static uint16_t rcDatap[RC_CHANS];
  static int16_t Temp;
  static uint8_t index;

  for (chan = 0; chan < RC_CHANS; chan++) { // simple average
    rcData[chan] = readRawRC(chan);
    if (firstRC)
      rcDatap[chan] = rcData[chan];
    rcData[chan] = rcDatap[chan] = (rcData[chan] + rcDatap[chan])>>1; 
  }
  firstRC = false;

#else
  for (chan = 0; chan < RC_CHANS; chan++) 
    rcData[chan] = readRawRC(chan);

#endif // USE_RC_FILTER

} // getRCInput

bool inline newRCValues(void) {
  static uint32_t rcLastUpdateuS = 0;
  static uint32_t rcIntervaluS = 0;
  bool r;

#if defined(SPEKTRUM)
  readSpektrum();   
#elif defined(SBUS)
  readSBus();
#elif defined(FLYSKY)
  readFlySky();
#endif

  rcIntervaluS = micros() - rcLastUpdateuS;

  if (rcNewValues) {
#if defined(DEBUG_RC)
    debug[1] = rcIntervaluS / 1000;
#endif

    rcLastUpdateuS = micros();
    rcNewValues = inFailsafe = false;
    r = true;
  } 
  else
    r = false;

  if (f.ARMED && (inFailsafe || (rcIntervaluS > (FAILSAFE_DELAY * 100000))))  {
#if defined(DEBUG_RC)
    debug[2] = inFailsafe;
#endif
    inFailsafe = true;

#if defined(FAILSAFE)
    f.ANGLE_MODE = true;  
#if defined(USE_MW_CONTROL)
    f.HORIZON_MODE = false;
#endif
    rcCommand[ROLL] = rcCommand[PITCH] = 0;

    if (rcIntervaluS > (FAILSAFE_OFF_DELAY * 100000)) {
      rcCommand[THROTTLE] = MIN_COMMAND;
      f.OK_TO_ARM = false;
      doDisarm();
    } 
    else
      rcCommand[THROTTLE] = FAILSAFE_THROTTLE;   
#endif
  }

  return (r);
} // newRCValues

/*

 * General Futaba SBus Encoding Scheme
 
 * The data is contained in 16 byte packets transmitted at 115200 baud.
 
 * The format of each frame, as know to date, is as follows
 
 *  byte1:  0x0f sentinel
 *  bytes1-24: packet with 11 bit channel values bit packed towards byte1.
 _______________________________________________________________________
 
 * General Spektrum Encoding Scheme
 
 * The bind function means that the satellite receivers believe they are
 * connected to a 9 channel JR-R921 24 receiver thus during the bind process
 * they try to get the transmitter to transmit at the highest resolution that
 * it can manage. The data is contained in 16 byte packets transmitted at
 * 115200 baud. Depending on the transmitter either 1 or 2 frames are required
 * to contain the data for all channels. These frames are either 11ms or 22ms
 * apart.
 
 * The format of each frame for the main receiver is as follows
 
 *  byte1:  frame loss data
 *  byte2:  transmitter information
 *  byte3:  and byte4:  channel data
 *  byte5:  and byte6:  channel data
 *  byte7:  and byte8:  channel data
 *  byte9:  and byte10: channel data
 *  byte11: and byte12: channel data
 *  byte13: and byte14: channel data
 *  byte15: and byte16: channel data
 
 * The format of each frame for the secondary receiver is as follows
 
 *  byte1:  frame loss data
 *  byte2:  frame loss data
 *  byte3:  and byte4:  channel data
 *  byte5:  and byte6:  channel data
 *  byte7:  and byte8:  channel data
 *  byte9:  and byte10: channel data
 *  byte11: and byte12: channel data
 *  byte13: and byte14: channel data
 *  byte15: and byte16: channel data
 
 * The frame loss data bytes starts out containing 0 as long as the
 * transmitter is switched on before the receivers. It then increments
 * whenever frames are dropped.
 
 * Three values for the transmitter information byte have been seen thus far
 
 * 0x01 From a Spektrum DX7eu which transmits a single frame containing all
 * channel data every 22ms with 10bit resolution.
 
 * 0x02 From a Spektrum DM9 module which transmits two frames to carry the
 * data for all channels 11ms apart with 10bit resolution.
 
 * 0x12 From a Spektrum DX7se which transmits two frames to carry the
 * data for all channels 11ms apart with 11bit resolution.
 
 * 0x12 From a JR X9503 which transmits two frames to carry the
 * data for all channels 11ms apart with 11bit resolution.
 
 * 0x01 From a Spektrum DX7 which transmits a single frame containing all
 * channel data every 22ms with 10bit resolution.
 
 * 0x12 From a JR DSX12 which transmits two frames to carry the
 * data for all channels 11ms apart with 11bit resolution.
 
 * 0x1 From a Spektrum DX5e which transmits a single frame containing all
 * channel data every 22ms with 10bit resolution.
 
 * 0x01 From a Spektrum DX6i which transmits a single frame containing all
 * channel data every 22ms with 10bit resolution.
 
 * Currently the assumption is that the data has the form :
 
 * [0 0 0 R 0 0 N1 N0]
 
 * where :
 
 * 0 means a '0' bit
 * R: 0 for 10 bit resolution 1 for 11 bit resolution channel data
 * N1 to N0 is the number of frames required to receive all channel
 * data.
 
 * Channels can have either 10bit or 11bit resolution. Data from a transmitter
 * with 10 bit resolution has the form:
 
 * [F 0 C3 C2 C1 C0 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0]
 
 * Data from a transmitter with 11 bit resolution has the form
 
 * [F C3 C2 C1 C0 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0]
 
 * where :
 
 * 0 means a '0' bit
 * F: Normally 0 but set to 1 for the first channel of the 2nd frame if a
 * second frame is transmitted.
 
 * C3 to C0 is the channel number, 4 bit, matching the numbers allocated in
 * the transmitter.
 
 * D9 to D0 is the channel data (10 bit) 0xaa..0x200..0x356 for
 * 100% transmitter-travel
 
 * D10 to D0 is the channel data (11 bit) 0x154..0x400..0x6AC for
 * 100% transmitter-travel
 
 * The order of the channels on a Spektrum is always as follows:
 *
 * Throttle   0
 * Aileron    1
 * Elevator   2
 * Rudder     3
 * Gear       4
 * Flap/Aux1  5
 * Aux2       6
 
 * Aux3       7
 * Aux4       8
 * Aux5       9
 * Aux6      10
 * Aux7      11
 
 */

















































































