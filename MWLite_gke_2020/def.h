

#define Limit1(i,l) (((i) < -(l)) ? -(l) : (((i) > (l)) ? (l) : (i)))
#define Sign(n) ((n<0) ? -1 : 1)
#define Sqr(n) (n*n)

#if (defined(USE_MW_ALEXK_CONTROL) || defined(USE_MW_LEGACY_CONTROL) || defined(USE_MW_2_3_CONTROL) )
#define USE_MW
#define STICK_TO_ANGLE 2  // [0:500] -> [0:100] degrees
#else
#define STICK_TO_ANGLE 1  // [0:500] -> [0:50] degrees
#endif

#define RAD_DEG10 (1800.0/PI)
#define RAD_DEG (180.0/PI)

#define MAX_BANK_ANGLE 500 // 0.1 Degrees - don't change

#define MAX_WAYPOINTS 2 // just home and hold for now (max 16)
#define HOME 0
#define HOLD (MAX_WAYPOINTS-1)

#if !defined(MAX_NAV_BANK_ANGLE)
#define MAX_NAV_BANK_ANGLE 300
#endif

#if !defined(ROLL_PITCH_COUPLING)
#define ROLL_PITCH_COUPLING 0 // %
#endif

#define MIN_RC_US 1000
#define MID_RC_US 1500
#define MAX_RC_US 2000

#if !defined(MAX_THR_US)
#define MAX_THR_US MAX_RC_US
#endif

#if !defined(MIN_THR_US)
#define MIN_THR_US MIN_RC_US
#endif

#if defined(MOTOR_STOP)
#define MIN_EFF_THR_US MIN_RC_US
#else
#define MIN_EFF_THR_US MIN_THR_US
#endif

#define MIN_PWM_US 1020 //  [1020;2000]
#define MAX_PWM_US 2000 //  [1020;2000]

/// introduce a deadband around the stick center 
// Must be greater than zero, comment if you dont want a deadband on roll, pitch and yaw 
#if !defined(STICK_DEADBAND_US)
#define STICK_DEADBAND_US 0
#endif

#if !defined(STICK_NEUTRAL_DEADBAND_US)
#define STICK_NEUTRAL_DEADBAND_US 50
#endif

#if !defined(YAW_DIR)
#define YAW_DIR 1
#endif

#if !defined(LVC_LIMIT)
#define LVC_LIMIT 0
#endif

// Software I2C - SCL is pin SCK/YAW, SDA is pin MISO/Pitch CAUTION VCC pin is 5V which will destroy some sensor boards 

#define I2C_SW 1  // software I2C bus
#define I2CMPU 0

#define UART_NUMBER 2
#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 128
#define INBUF_SIZE 64

#define GPS_SERIAL_PORT 1

/// atmega32u4 

#define BEEPER_ON {}
#define BEEPER_OFF {}
#define BEEPER_TOGGLE {}

#define LED_BLUE_PINMODE           DDRD |= (1<<4)           //D4 to output
#define LED_BLUE_TOGGLE            PIND |= (1<<5)|(1<<4)    //switch LEDPIN state (Port D5) & pin D4
#define LED_BLUE_OFF               PORTD |= (1<<5); PORTD &= ~(1<<4)
#define LED_BLUE_ON                PORTD &= ~(1<<5); PORTD |= (1<<4)

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
#define AUX2PIN                    7 
//#define AUX3PIN                  1 // unused 
//#define AUX4PIN                  0 // unused 

#if !defined(RCAUX2PIND17)
#define PCINT_PIN_COUNT            4
#define PCINT_RX_BITS              (1<<1),(1<<2),(1<<3),(1<<4)
#else
#define PCINT_PIN_COUNT            5 // one more bit (PB0) is added in RX code
#define PCINT_RX_BITS              (1<<1),(1<<2),(1<<3),(1<<4),(1<<0)
#endif

#define PCINT_RX_PORT               PORTB
#define PCINT_RX_MASK               PCMSK0
#define PCIR_PORT_BIT               (1<<0)
#define RX_PC_INTERRUPT             PCINT0_vect
#define RX_PCINT_PIN_PORT           PINB

// Board Orientations and Sensor definitions

//________________________________________

#if defined(NANOWII)

#define I2CBARO 0
#define I2CMAG 0

#define MPU6050

#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  =  X; accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  Y; magADC[PITCH]  =  -X; magADC[YAW]  = -Z;}

#undef INTERNAL_I2C_PULLUPS

#define VBAT_PIN A3
#define VOLTS_RTOP 58  // 56 Ratios as per HK NanoWii manual for 3S pack
#define VOLTS_RBOT 33  

#define SONAR_PIN A1 // ??????

//________________________________________

#elif defined(Multiwii_32U4_SE)

#define I2CBARO 0
#define I2CMAG 0

#define MPU6050
//#define HMC5883L
//#define MS5611

#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}

#undef INTERNAL_I2C_PULLUPS

#define VBAT_PIN A4
#define VOLTS_RTOP 58  // 56 Ratios as per HK NanoWii manual for 3S pack
#define VOLTS_RBOT 33  

#define SONAR_PIN A1

//________________________________________

#elif defined(HK_PocketQuad)

#if !defined(STANDARD_RX)
#define USE_5V_ON_MOSI
#define I2CBARO 1
#define I2CMAG 1
#else
#define I2CBARO 0
#define I2CMAG 0
#endif

#define DC_MOTORS

#define MPU6050

#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  =  -Y; accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}

#undef INTERNAL_I2C_PULLUPS

#define VBAT_PIN A8
#define VOLTS_RTOP 0  // no divider for 1S pack
#define VOLTS_RBOT 1

#define SONAR_PIN A1 // ??????

#endif

//________________________________________

//Multitype declaration for the GUI's 

#if defined(QUADP)
#define MULTITYPE 2
#elif defined(QUADX)
#define MULTITYPE 3
#elif defined(FLYING_WING)
#define MULTITYPE 8
#elif defined(AIRPLANE)
#define MULTITYPE 14
#elif defined(VTAIL)
#define MULTITYPE 17
#endif

enum AirplaneControl {
  RightAileron = 1, LeftAileron, Rudder, Elevator};
enum VTailControl {
  RightRudder = 3, LeftRudder};
enum WingControl {
  RightElevon = 1, LeftElevon};

#if !defined(SERIAL_SUM_PPM) && !defined(SPEKTRUM) && !defined(SBUS) && !defined(FLYSKY)
#define STANDARD_RX
#endif

#if defined(SPEKTRUM)
#define SPEK_FRAME_SIZE 16
#if (SPEKTRUM == 1024)
#define SPEK_CHAN_SHIFT  2 // Assumes 10 bit frames, that is 1024 mode.
#define SPEK_CHAN_MASK   0x03    
#define SPEK_DATA_SHIFT          
#define SPEK_BIND_PULSES 3
#endif
#if (SPEKTRUM == 2048)
#define SPEK_CHAN_SHIFT  3 // Assumes 11 bit frames, that is 2048 mode.
#define SPEK_CHAN_MASK   0x07  
#define SPEK_DATA_SHIFT >> 1
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

#if !defined(ACC_TRIM_STEP_SIZE)
#define ACC_TRIM_STEP_SIZE 8
#endif

#if !defined(ALT_HOLD_THROTTLE_NEUTRAL_ZONE)
#define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 40
#endif

#if defined(BMP085) || defined(MS5611) || defined(MAXBOTIX)
#define USE_ALT
#endif

#if defined(HMC5883L)
#define USE_MAG
#endif

#if (defined(USE_GPS) || defined(HMC5883L) || defined(MS5611) || defined(BMP085))
#define CYCLE_US 3000 
#else
#define CYCLE_US 2048 // PID calculations scaled for this - no point in going faster :)
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
  //PIDROLL,
  //PIDPITCH,
  //PIDYAW,
  ALTITUDE = 3,
  POSITION,
  POSITIONRATE,
  NAVRATE,
  LEVEL,
  HEADING,
  VELOCITY, 
  PID_ITEMS
};

enum box {
  BOX_ARM,
  BOX_ANGLE,
#if defined(USE_MW)
  BOX_HORIZON,
#endif
#if defined(MULTICOPTER)
  BOX_HEAD_FREE,
#endif
#if defined(USE_MAG)
  BOX_HEAD_HOLD,
#endif
#if defined(USE_ALT)
  BOX_ALT_HOLD,
#endif
#if defined(USE_GPS)
  BOX_GPS_HOLD,
  BOX_GPS_HOME,
#endif
#if !defined(MULTICOPTER)
  BOX_BYPASS,
#endif
#if defined(USE_TUNING)
  BOX_RELAY,
#endif
  BOX_EXP,
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
  1;
uint8_t ALT_HOLD_MODE :
  1 ;  
uint8_t HEAD_FREE_MODE :
  1 ;
uint8_t HEAD_HOLD_MODE :
  1 ;
uint8_t RELAY_MODE : 
  1;
uint8_t BARO_ACTIVE :
  1 ; 
uint8_t SMALL_ANGLE_25DEG :
  1;
uint8_t STICKS_CENTRED :
  1;
uint8_t CALIBRATE_MAG :
  1 ;
uint8_t MAG_ACTIVE:
  1;
uint8_t MAG_CALIBRATED:
  1;
uint8_t BYPASS_MODE:
  1;
uint8_t GPS_ACTIVE:
  1;
uint8_t SONAR_ACTIVE:
  1;
uint8_t OPTIC_ACTIVE:
  1;
uint8_t EXP:
  1;
uint8_t GPS_FIX:
  1;  
uint8_t GPS_FIX_HOME:
  1;
uint8_t GPS_HOLD_MODE:
  1;
uint8_t GPS_HOME_MODE:
  1;
uint8_t ALARM:
  1;
} 
f;

// RC

#define MIN_CHECK 1100
#define MAX_CHECK 1900

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
#if defined(USE_MW)
"HORIZON;"
#endif
#if defined(MULTICOPTER)
"H.FREE;"
#endif
#if defined(USE_MAG)
"H.HOLD;"
#endif
#if defined(USE_ALT)
"A.HOLD;"
#endif
#if defined(USE_GPS)
"P.HOLD;"
"RTH;"
#endif
#if !defined(MULTICOPTER)
"BYPASS;"
#endif
#if defined(USE_TUNING)
"RELAY;"
#endif
"EXP;"
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
#if defined(USE_MW)
  1 << BOX_HORIZON,
#endif 
#if defined(MULTICOPTER)
  1 << BOX_HEAD_FREE,
#endif
#if defined(USE_MAG) 
  1 << BOX_HEAD_HOLD,
#endif
#if defined(USE_ALT)
  1 << BOX_ALT_HOLD,
#endif
#if defined(USE_GPS)
  1 << BOX_GPS_HOLD,
  1 << BOX_GPS_HOME,
#endif
#if !defined(MULTICOPTER)
  1 << BOX_BYPASS,
#endif
#if defined(USE_TUNING)
  1 << BOX_RELAY,
#endif
  1 << BOX_EXP,
};


// Errors

#if !defined(MAG_ORIENTATION)
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = Z;} // GY86 upside down SW I2C
#endif

#if ((defined(QUADX) + defined(QUADP) + defined(FLYING_WING) + defined(AIRPLANE) + defined(VTAIL)) > 1)
#error "Select only QUADX, QUADP, FLYING_WING or AIRPLANE"
#endif

#if ((defined(BMP085) + defined(MS5611)) > 1)
#error "Select only ONE of MS5611 or BMP085 barometer"
#endif

#if !defined(PWM_OUTPUTS)
#error "PWM_OUTPUTS is not set, most likely you have not defined any type of aircraft"
#endif

#if (PWM_OUTPUTS > 5)
#error "PWM_OUTPUTS must be less than 6"
#endif

//#if (!defined(__AVR_ATmega32U4__) || ((PWM_OUTPUTS !=4) || defined(SERVO)))
//#error "Implementation restriction: must be exactly 4 DC_MOTORS and no SERVOS and use Atmel 32u4 (Leonardo)"
//#endif
