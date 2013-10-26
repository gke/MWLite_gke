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

// Macros

#define Limit1(i,l) (((i) < -(l)) ? -(l) : (((i) > (l)) ? (l) : (i)))
#define Sign(n) ((n<0) ? -1 : 1)
#define PercentToStick(n) (n*5)

// Motors

#define NUMBER_MOTOR     4

#ifdef MOTOR_STOP
#define EFF_MINCOMMAND MINCOMMAND
#else
#define EFF_MINCOMMAND conf.minthrottle
#endif

#if (defined(USE_MW_CONTROL) || defined(USE_MW_BASEFLIGHT_CONTROL) || defined(USE_MW_EXPANDED_CONTROL))
#define USE_MW
#endif

#if defined(USE_QUICK_TUNE) || defined(USE_QUICK_ALT_TUNE) || defined(USE_RELAY_TUNE)
#define USE_TUNING
#endif

#if defined(USE_BOSCH_MAG)
#define USE_MAG
#endif

/// atmega32u4 

#define LED_BLUE_PINMODE             //
#define LED_BLUE_PIN                PORTD5
#define LED_BLUE_TOGGLE             PIND |= 1<<5     //switch LEDPIN state (Port D5)
#define LED_BLUE_OFF                PORTD |= (1<<5)
#define LED_BLUE_ON                 PORTD &= ~(1<<5)

#define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1   // PIN 2&3 (SDA&SCL)
#define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1)
#define PPM_PIN_INTERRUPT          DDRE &= ~(1 << 6);PORTE |= (1 << 6);EIMSK |= (1 << INT6);EICRB |= (1 << ISC61)|(1 << ISC60)
#if !defined(SPEK_SERIAL_PORT)
#define SPEK_SERIAL_PORT           1
#endif
#define USB_CDC_TX                 3
#define USB_CDC_RX                 2

// Standard RX
#define THROTTLEPIN                3
#if defined(A32U4ALLPINS)
#define ROLLPIN                    6
#define PITCHPIN                   2
#define YAWPIN                     4
#define AUX1PIN                    5
#else
#define ROLLPIN                    4
#define PITCHPIN                   5
#define YAWPIN                     2
#define AUX1PIN                    6
#endif
#define AUX2PIN                      7 
#define AUX3PIN                      1 // unused 
#define AUX4PIN                      0 // unused 

#if !defined(RCAUX2PIND17)
#define PCINT_PIN_COUNT          4
#define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4)
#else
#define PCINT_PIN_COUNT          5 // one more bit (PB0) is added in RX code
#define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4),(1<<0)
#endif
#define PCINT_RX_PORT                PORTB
#define PCINT_RX_MASK                PCMSK0
#define PCIR_PORT_BIT                (1<<0)
#define RX_PC_INTERRUPT              PCINT0_vect
#define RX_PCINT_PIN_PORT            PINB

// Board Orientations and Sensor definitions

#if defined(NANOWII)
#define QUADX
#define MPU6050
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  =  X; accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
#undef INTERNAL_I2C_PULLUPS

#define LED_BLUE_PINMODE             DDRD |= (1<<4)           //D4 to output
#define LED_BLUE_TOGGLE              PIND |= (1<<5)|(1<<4)     //switch LEDPIN state (Port D5) & pin D4
#define LED_BLUE_OFF                 PORTD |= (1<<5); PORTD &= ~(1<<4)
#define LED_BLUE_ON                  PORTD &= ~(1<<5); PORTD |= (1<<4)

#define VBAT_PIN A3
#define VOLTS_RTOP 58  // 56 Ratios as per HK NanoWii manual for 3S pack
#define VOLTS_RBOT 33  

#define SONAR_PIN A1 // ??????

#endif

#if defined(HK_PocketQuad)
#define QUADX
#define DC_MOTORS
#define CESCO_OFFSET // motor PWM phase offset to smooth battery load
#define MPU6050
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  =  -Y; accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
#undef INTERNAL_I2C_PULLUPS

#define VBAT_PIN A8
#define VOLTS_RTOP 0  // no divider for 1S pack
#define VOLTS_RBOT 1

#define SONAR_PIN A1 // ??????

#endif

//Multitype declaration for the GUI's 

#if defined(QUADP)
#define MULTITYPE 2
#elif defined(QUADX)
#define MULTITYPE 3
#endif

// Some unsorted "chain" defines 

#if !defined(SERIAL_SUM_PPM) && !defined(SPEKTRUM) && !defined(SBUS) && !defined(FLYSKY)
#define STANDARD_RX
#endif

#if defined(SPEKTRUM)
#define SPEK_FRAME_SIZE 16
#if (SPEKTRUM == 1024)
#define SPEK_CHAN_SHIFT  2       // Assumes 10 bit frames, that is 1024 mode.
#define SPEK_CHAN_MASK   0x03    // Assumes 10 bit frames, that is 1024 mode.
#define SPEK_DATA_SHIFT          // Assumes 10 bit frames, that is 1024 mode.
#define SPEK_BIND_PULSES 3
#endif
#if (SPEKTRUM == 2048)
#define SPEK_CHAN_SHIFT  3       // Assumes 11 bit frames, that is 2048 mode.
#define SPEK_CHAN_MASK   0x07    // Assumes 11 bit frames, that is 2048 mode.
#define SPEK_DATA_SHIFT >> 1     // Assumes 11 bit frames, that is 2048 mode.
#define SPEK_BIND_PULSES 5
#endif
#endif // SPEKTRUM

#if defined(SBUS)
#define RC_CHANS 18
#elif defined(SPEKTRUM) 
#define RC_CHANS 14
#elif defined(FLYSKY)
#define RC_CHANS 8
#elif defined(SERIAL_SUM_PPM)
#define RC_CHANS 12
#else
#define RC_CHANS 8
#endif // SBUS

#if !defined(ACROTRAINER_MODE)
#define ACROTRAINER_MODE 1000
#endif

#if !defined(ACC_TRIM_STEP_SIZE)
#define ACC_TRIM_STEP_SIZE 8
#endif

#if !defined(ALT_HOLD_THROTTLE_NEUTRAL_ZONE)
#define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 40
#endif

#if defined(USE_MW)
#define  VERSION  1000 
#else
#define  VERSION  1001 
#endif

/*********** RC alias *****************/
enum rc { // must be in this order
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  AUX1,
  AUX2,
  AUX3,
  AUX4
};

enum pid {
  PIDROLL,
  PIDPITCH,
  PIDYAW,
  PIDALT,
  PIDPOS,
  PIDPOSR,
  PIDNAVR,
  PIDLEVEL,
  PIDMAG,
  PIDVEL, 
  PIDITEMS
};

enum box {
  BOX_ARM,
  BOX_ANGLE,
  BOX_HORIZON,
  BOX_ACRO_TRAINER,
  BOX_ALT_HOLD,
  BOX_HEAD_FREE,
  BOX_HEAD_HOLD,
#if defined(USE_TUNING)
  BOX_TUNE,
  BOX_ALT_TUNE,
  BOX_RELAY,
#endif
  CHECKBOX_ITEMS
};


struct flags_struct {
uint8_t OK_TO_ARM :
  1 ;
uint8_t ARMED :
  1 ;
uint8_t ACC_CALIBRATED :
  1 ;
uint8_t ANGLE_MODE :
  1 ;
uint8_t HORIZON_MODE :
  1 ;
uint8_t ALT_HOLD_MODE :
  1 ;  
uint8_t ACRO_TRAINER_MODE :
  1 ; 
uint8_t HEAD_FREE_MODE :
  1 ;
uint8_t HEAD_HOLD_MODE :
  1 ;
uint8_t TUNE_MODE :
  1;
uint8_t ALT_TUNE_MODE :
  1;
uint8_t RELAY_MODE : 
  1;
uint8_t BARO_ACTIVE :
  1 ; 
uint8_t SMALL_ANGLE_25DEG :
  1;
uint8_t CALIBRATE_MAG :
  1 ;
uint8_t MAG_ACTIVE:
  1;
uint8_t MAG_CALIBRATED:
  1;
uint8_t ENABLE_ACRO_TRAINER:
  1;
uint8_t GPS_ACTIVE:
  1;
uint8_t SONAR_ACTIVE:
  1;
uint8_t OPTIC_ACTIVE:
  1;
uint8_t ALARM:
  1;
} 
f;

// RC

#define MINCHECK 1100
#define MAXCHECK 1900

#define ROL_LO  (1<<(2*ROLL))
#define ROL_CE  (3<<(2*ROLL))
#define ROL_HI  (2<<(2*ROLL))
#define PIT_LO  (1<<(2*PITCH))
#define PIT_CE  (3<<(2*PITCH))
#define PIT_HI  (2<<(2*PITCH))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))


const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
"ARM;"
"ANGLE;"
"HORIZ;"
"ACRO.T;"
"A.HOLD;"
"H.FREE;"
"H.HOLD;"
#if defined(USE_TUNING)
"TUNE;"
"A.TUNE;"
"RELAY;"
#endif
;

const char pidnames[] PROGMEM =
"ROLL;"
"PITCH;"
"YAW;"
"ALT;"
"Pos;"
"PosR;"
"NavR;"
"LEVEL;"
"MAG;"
"VEL;"
;

const uint8_t boxids[] PROGMEM = {
  1 << BOX_ARM, 
  1 << BOX_ANGLE, 
  1 << BOX_HORIZON,
  1 << BOX_ACRO_TRAINER,
  1 << BOX_ALT_HOLD,
  1 << BOX_HEAD_FREE, 
  1 << BOX_HEAD_HOLD, 
#if defined(USE_TUNING)
  1 << BOX_TUNE,
  1 << BOX_ALT_TUNE,
  1 << BOX_RELAY,
#endif
};


// Errors

#if !defined(I2CMPU) | !defined(I2CBARO)
#define I2CMPU 0
#define I2CBARO 0
#endif

#if !defined(MAG_ORIENTATION)
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = Z;} // GY86 upside down SW I2C
#endif

#if ((defined(USE_BOSCH_BARO) + defined(USE_MS_BARO) + defined(USE_SONAR)) > 1)
#error "Select only ONE of SONAR or MS5611 or BMP085 barometer"
#endif

#if !defined(NUMBER_MOTOR)
#error "NUMBER_MOTOR is not set, most likely you have not defined any type of multicopter"
#endif

#if (!defined(__AVR_ATmega32U4__) || ((NUMBER_MOTOR !=4) || defined(SERVO)))
#error "Implementation restriction: must be exactly 4 DC_MOTORS and no SERVOS and use Atmel 32u4 (Leonard)"
#endif

#if ((defined(USE_QUICK_TUNE) + defined(USE_QUICK_ALT_TUNE) +  defined(USE_RELAY_TUNE))>1)
#error "Only one tuning scheme permitted at a time (USE_QUICK_TUNE or USE_QUICK_ALT_TUNE)" // or USE_RELAY_TUNE)"
#endif




