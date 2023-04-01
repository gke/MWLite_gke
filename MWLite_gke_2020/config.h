#define ESC_CALIB_LOW  MIN_PWM_US
#define ESC_CALIB_HIGH MAX_PWM_US
//#define ESC_CALIB_CANNOT_FLY  // uncomment to activate - YOU MUST REMOVE ALL PROPELLERS!!!!!

#define VERSION 230

//#define JESOLINS // default settings that should work for most quads

#if defined(JESOLINS)

#define MULTICOPTER

#define HK_PocketQuad
//#define BMP085  
//#define MS5611

//#define WOLFERL_CONTROL
//#define UAVX_CONTROL
//#define USE_MW_LEGACY_CONTROL
#define USE_MW_2_3_CONTROL

#define STICK_DEADBAND_US 10 // [0:500]
#define STICK_NEUTRAL_DEADBAND_US 50 // forces ANGLE MODE if sticks below this deflection [0:500]

#define MPU6050_DLPF_CFG MPU6050_LPF_42HZ 
//#define MPU6050_DLPF_CFG MPU6050_LPF_98HZ
//#define MPU6050_DLPF_CFG MPU6050_LPF_188HZ

#define SPEKTRUM 1024
//#define SPEKTRUM 2048
//#define SERIAL_SUM_PPM  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum
//#define SPEKTRUM_RTF_SCALING // rescales and reverses yaw and roll for Blade RTF MLP4DSM game style Tx
#define USE_GKE_DM9_SPEKTRUM_SCALING // dodgy DM9 does not appear to capture pulse widths correctly?

#define MOTOR_STOP // comment out for slow motor run after arming
#define MIN_THR_US 1050
#define MAX_THR_US 1850

//#define LVC_LIMIT 30 // x0.1V on PUMQ connect D8/AUX1 pin directly to battery raw

#define MAG_DECLINATION 12

#else

//_____________________________________________________________________________________________

// Configurations for gke's aircraft

#define MULTICOPTER
//#define FLYING_WING
//#define AIRPLANE
//#define VTAIL // MultiGUI does not support V-Tails

//________________________

#if defined(MULTICOPTER)

//#define ECKS
#define PUMQ
//#define WLTOYS

//#define WOLFERL_CONTROL
#define UAVX_CONTROL
//#define USE_MW_LEGACY_CONTROL
//#define USE_MW_2_3_CONTROL
//NOT COMMISSIONED #define USE_MW_ALEXK_CONTROL

#define QUADX
#define PWM_OUTPUTS   4
#define MIN_THR_US 1060 
#define MAX_THR_US 1860

//_______________

#if defined(ECKS)

//#define Multiwii_32U4_SE
#define NANOWII
//#define BMP085
//#define MS5611
//#define HMC5883L
//#define USE_GPS

#define SERIAL_SUM_PPM  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum

//#define LVC_LIMIT 33 // 0.1V

//_______________

#elif defined(PUMQ) 

#define HK_PocketQuad
//#define BMP085
//#define MS5611
//#define HMC5883L

#define SPEKTRUM 1024

#define LVC_LIMIT 30 // x0.1V on PUMQ connect D8/AUX1 pin directly to battery raw

//_______________

#elif defined(WLTOYS)

#define HK_PocketQuad
//#define BMP085
//#define MS5611
//#define HMC5883L

#define YAW_DIR (-1)

#define SPEKTRUM 1024

#define LVC_LIMIT 30 // x0.1V on PUMQ connect D8/AUX1 pin directly to battery raw

//_______________

#else

#error "Multicopter type not specified"

#endif

//________________________

#elif defined(FLYING_WING)


#define USE_GPS
//#define USE_MSP_WP
#define GPS_RESET_HOME_ON_DISARM
//#define MAXBOTIX
#define DEBUG_NAV

//#define PLANKS
//#define Z84
#define FX81
#define NANOWII

//#define Multiwii_32U4_SE
#define PWM_OUTPUTS     3
#define BMP085
//#define MS5611

#define SERIAL_SUM_PPM  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 // Graupner/Spektrum

//_______________

#if defined(PLANKS)

#define SERVO_LIMITS {{-100,100},{-70,70},{-70,70}}
#define SERVO_DIR {1, 1, -1}
#define AILERON_DIFF  0 // should be very small to correct any mechanical differential ONLY
#define MAX_NAV_BANK_ANGLE 150

//_______________

#elif defined(Z84)

#define SERVO_MECH_TRIMS {0, 0, 115}
#define SERVO_LIMITS {{-100,100},{-70,70},{-70,70}}
#define SERVO_DIR {1, -1, 1}
#define AILERON_DIFF  0 // should be very small to correct any mechanical differential ONLY
#define MAX_NAV_BANK_ANGLE 150

//_______________

#elif defined(FX81)

#define SERVO_MECH_TRIMS {0, 0, 100}
#define SERVO_LIMITS {{-100,100},{-100,100},{-100,100}}
#define SERVO_DIR {1, -1, 1}
#define AILERON_DIFF  0 // should be very small to correct any mechanical differential ONLY
#define ROLL_PITCH_COUPLING 40
#define MAX_NAV_BANK_ANGLE 350

//_______________

#else

#error "Flying wing type not specified"

#endif

//________________________

#elif defined(AIRPLANE)

//#define PUSHYCAT
//#define SPITFIRE_HK
//#define LANCASTER
#define RUDDERELEV

#define USE_GPS
#define NANOWII
#define PWM_OUTPUTS     5
//#define BMP085
#define MS5611

#define SERIAL_SUM_PPM  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum

//_______________

#if defined(PUSHYCAT)

#define SERVO_LIMITS {{-100,100},{-70,70},{-70,70},{-50,50},{-60,60}}
#define SERVO_DIR {1, 1, -1, 1, -1}
#define AILERON_DIFF  40
#define RUDDER_AILERON_COUPLING 25
#define MAX_NAV_BANK_ANGLE 150

//_______________

#elif defined(SPITFIRE_HK)

#define SERVO_MECH_TRIMS {0, 0, 0, 0, -87}
#define SERVO_LIMITS {{-100,100},{-70,70},{-70,70},{-50,50},{-60,60}}
#define SERVO_DIR {1, 1, -1, -1, 1}
#define AILERON_DIFF  40
#define MAX_NAV_BANK_ANGLE 150

//_______________

#elif defined(LANCASTER)

#define SERVO_LIMITS {{-100,100},{-70,70},{-70,70},{-50,50},{-60,60}}
#define SERVO_DIR {1, 1, -1, 1, -1}
#define AILERON_DIFF  40
#define MAX_NAV_BANK_ANGLE 100

//_______________

#elif defined(RUDDERELEV)

#define SERVO_LIMITS {{-100,100},{-50,50},{-60,60}}
#define SERVO_DIR {1, 1, -1}
#define RUDDER_AILERON_COUPLING 25
#define MAX_NAV_BANK_ANGLE 100

//_______________

#else

#error "Aircraft type not specified"

#endif

//________________________

#elif defined(VTAIL)

#define NANOWII
#define PWM_OUTPUTS     5

#define SERVO_LIMITS {{-100,100},{-70,70},{-70,70},{-60,60},{-60,60}}
#define SERVO_DIR {1, 1, -1, 1, -1}
#define AILERON_DIFF  40

#define SERIAL_SUM_PPM  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum

#endif // gke

#define STICK_DEADBAND_US 15 // uSec was 10
#define STICK_NEUTRAL_DEADBAND_US 70 // forces angle mode on this axis if sticks below this deflection [0:500] // was 50

#if defined(MULTICOPTER)
#define MPU6050_DLPF_CFG MPU6050_LPF_188HZ
#else
#define MPU6050_DLPF_CFG MPU6050_LPF_42HZ 
#endif

#define MAG_DECLINATION 12

#define USE_GKE_DM9_SPEKTRUM_SCALING // dodgy DM9 does not appear to capture pulse widths correctly?
//#define SPEKTRUM_RTF_SCALING // rescales and reverses yaw and roll for Blade RTF MLP4DSM game style Tx

//#define DEBUG_RC // debug1 = glitch count, debug2 = frame width, debug3 = failsafe
//#define DEBUG_RELAY // debug1 = RollKu*100, debug2 = Rollw*10 ...
//#define DEBUG_NAV // debug1 = navRoll, debug2 = navPitch, debug3 = navYaw ...
//#define DEBUG_ATTITUDE // debug1 = roll, debug2 = pitch, debug3 = yaw, debug4 = accOK
//#define DEBUG_BARO // debug1 = temp*100, debug1 = press*10, debug3 = groundalt, debug4 = alt
//#define DEBUG_ALT_HOLD // debug1 = rate of climb, debug2 = desired alt, debug3 = BaroPID, debug4 = throttle
//#define DEBUG_HEAD_HOLD // debug1 = heading, debug2 = desired heading, debug3 = error, debug4 = yaw command
//#define DEBUG_TRIMS // debug2 = aileron, debug3 = elevator, debug4 = rudder trims

#endif // gke configurations

//_________________________________________

// Best not to change these ;)

#if !defined(GPS_RESET_HOME_ON_DISARM)
#define GPS_RESET_HOME_ON_DISARM
#endif

#define GPS_SBAS SBAS_DISABLED
//#define GPS_SBAS SBAS_WAAS // Americas
//#define GPS_SBAS SBAS_EGNOS // Europe and Africa
//#define GPS_SBAS SBAS_MSAS // Asia north of the Equator (not Oz)

//#define USE_RELAY_TUNE
#define TUNE_BOTH_AXES // if uncommented alternates between pitch and roll otherwise just pitch

#define ALLOW_ARM_DISARM_VIA_TX_YAW
//#define ALLOW_ARM_DISARM_VIA_TX_ROLL

#define FAILSAFE

#define USE_RC_FILTER // simple average
#define USE_ACC_LPF // enables accelerometer moving average filter
#define USE_GYRO_AVERAGE // remove some jitter by averaging last two readings

//____________________________________________________________________________________________

// Altitude

#define USE_MW_ALT_CONTROL // original MW altitude hold scheme
#define USE_PROP_ALT_HOLD // rate of climb is proportional to throttle 
#define ALT_HOLD_LIMIT_M 120 // metres
#define ALT_HOLD_STEP 8 

//#define USE_MS5611_EXTENDED_TEMP  // optional if flying below 20C

// Magnetometer

//#define HMC5883L

// Choose the option which gives almost no compass heading change when you tilt the quad about 20deg in any DIR
// Package top upwards
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  Y; magADC[PITCH]  =  -X; magADC[YAW]  = -Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -X; magADC[PITCH]  =  -Y; magADC[YAW]  = -Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -Y; magADC[PITCH]  =  X; magADC[YAW]  = -Z;}

// Package top downwards
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  Y; magADC[PITCH]  =  X; magADC[YAW]  = Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  -Y; magADC[YAW]  = Z;} // GY86 upside down SW I2C
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -Y; magADC[PITCH]  =  -X; magADC[YAW]  = Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -X; magADC[PITCH]  =  Y; magADC[YAW]  = Z;}

// Throttle

//#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
//#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
//#define MINTHROTTLE 1064 // special ESC (simonk)
//#define MINTHROTTLE 1050 // for brushed ESCs like ladybird
//#define MINTHROTTLE 1150 // (*)

//#define SERIAL_SUM_PPM         PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum
//#define SERIAL_SUM_PPM         ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Robe/Hitec/Futaba
//#define SERIAL_SUM_PPM         ROLL,PITCH,YAW,THROTTLE,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Multiplex
//#define SERIAL_SUM_PPM         PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For some Hitec/Sanwa/Others

//#define SPEKTRUM 1024
//#define SPEKTRUM 2048
//#define SPEK_SERIAL_PORT 1    // Forced to 0 on Pro Mini and single serial boards

//#define SBUS

#define SERIAL0_COM_SPEED 115200
#define SERIAL1_COM_SPEED 115200

// Battery

// On HK PUMQ connect D8/AUX1 pin directly to battery raw
#define LVC_DELAY_TIME_S 2 // autoland after this delay in seconds
#define LVC_WARNING_PERCENT 90 // scales down desired throttle "suddenly" to this percentage when low volts reached
#define LVC_LANDING_TIME_S 5 // time for throttle to drop to zero
//#define USE_MINIMAL_LVC // reduces code size

// Failsafes

// Failsafe check pulses on four main control channels CH1-CH4. If the pulse is missing or below 985us (on any of these 
// four channels the failsafe procedure is initiated. After FAILSAFE_DELAY time from failsafe detection, the level mode,
// PITCH, ROLL and YAW is centered and THROTTLE is set to FAILSAFE_THR0TTLE value. You must set this value to descending 
// about 1m/s or so for best results. This value is depended from your configuration, AUW and some other params.  Next,  
// after FAILSAFE_OFF_DELAY the copter is disarmed, and motors is stopped. If RC pulse coming back before reached 
// FAILSAFE_OFF_DELAY time, after the small quard time the RC control is returned to normal. 

#define FAILSAFE_DELAY     10 // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
#define FAILSAFE_OFF_DELAY (FAILSAFE_DELAY*5) // Time for Landing before motors stop in 0.1sec. 
#if defined(MULTICOPTER)
#define FAILSAFE_THR_US  (MIN_THR_US + 200) // (*) Throttle level used for landing - may be relative to MINTHROTTLE 
#else
#define FAILSAFE_THR_US MIN_RC_US
#endif




