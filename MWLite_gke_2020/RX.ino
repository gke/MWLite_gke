
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

volatile uint8_t  spekFrameSeen;
volatile uint32_t spekLastUpdateuS;

volatile uint16_t rcValue[RC_CHANS] = {
  1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#elif defined(SERIAL_SUM_PPM)
volatile uint16_t rcValue[RC_CHANS]= {
  1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000] 
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
  Serial.begin(SPEK_SERIAL_PORT,115200);
#elif defined(SBUS)
  serialOpen(1,100000);
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
  interrupts();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical 
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
  uint16_t NowuS, Width;
  static uint16_t PrevuS = 0;
  static uint8_t chan = 0;

  NowuS = micros();
  interrupts();
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
} // INT6_vect

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

  if ((!f.ARMED) && ( serialPeek(SPEK_SERIAL_PORT) == '$')) {
    while (serialAvailable(SPEK_SERIAL_PORT)) {
      serialCom();
      delay (10);
    }
    return;
  } //End of: Is it the GUI?

  while (serialAvailable(SPEK_SERIAL_PORT) > SPEK_FRAME_SIZE)
    for (i = 0; i < SPEK_FRAME_SIZE; i++) 
      serialRead(SPEK_SERIAL_PORT); //discard stale frame and use latest

  if (spekFrameSeen) 
    if (serialAvailable(SPEK_SERIAL_PORT) == SPEK_FRAME_SIZE) {  
      serialRead(SPEK_SERIAL_PORT); 
      serialRead(SPEK_SERIAL_PORT); // discard header
      for (b = 2; b < SPEK_FRAME_SIZE; b += 2) {
        bh = serialRead(SPEK_SERIAL_PORT);
        bl = serialRead(SPEK_SERIAL_PORT);
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
      spekFrameSeen = false;
    } 
    else { // check frame timeout
      spekInterval = micros() - spekLastUpdateuS;
      if (spekInterval > 2500) { 
        rcGlitches++;
        spekFrameSeen = false; 
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
#else
#define SPEK_BIND_PULSES 4
#endif

void doSpektrumBinding(void) { 
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
  } 
} // checkSpektrumBinding

#endif // SPEKTRUM

//_____________________________________________________________________________________________

// General 

uint16_t readRawRC(uint8_t chan) {
  uint16_t v;
#if defined(SPEKTRUM)
  readSpektrum();
  if (chan < RC_CHANS)
    v = rcValue[rcChannel[chan]];
  else v = MID_RC_US;
#else
  noInterrupts(); 
  v = rcValue[rcChannel[chan]]; 
  interrupts();
#endif
  return v; 
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

boolean inline newRCValues(void) {
  static uint32_t rcLastUpdateuS = 0;
  static uint32_t rcIntervaluS = 0;
  boolean r;

#if defined(SPEKTRUM)
  readSpektrum();   
#elif defined(SBUS)
  readSBus();
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
#if defined(USE_MW)
    f.HORIZON_MODE = false;
#endif
    rcCommand[ROLL] = rcCommand[PITCH] = 0;

    if (rcIntervaluS > (FAILSAFE_OFF_DELAY * 100000)) {
      rcCommand[THROTTLE] = MIN_RC_US;
      f.OK_TO_ARM = false;
      doDisarm();
    } 
    else
      rcCommand[THROTTLE] = FAILSAFE_THR_US;   
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
