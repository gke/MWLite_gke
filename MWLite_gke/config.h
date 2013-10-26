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
 
// CONFIGURABLE PARAMETERS

#define GENERAL_USE // default settings that should work for most quads

#if defined(GENERAL_USE)

#define ALLOW_ARM_DISARM_VIA_TX_YAW
//#define ALLOW_ARM_DISARM_VIA_TX_ROLL

#define SPEKTRUM 1024
//#define SPEKTRUM 2048
//#define SERIAL_SUM_PPM  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum
//#define SPEKTRUM_RTF_SCALING // rescales and reverses yaw and roll for Blade RTF MLP4DSM game style Tx

#define USE_MW_CONTROL // original MW control scheme
//#define USE_MW_BASEFLIGHT_CONTROL // baseflight version of MW control scheme
//#define WOLFERL // angle mode only - use default parameters Brad's tuning scheme tested with this

//#define USE_QUICK_TUNE // based on Brad Quick's tuning scheme - change PID tuning limits at the bottom of this file.
//#define USE_QUICK_ALT_TUNE // tuning for altitude control - change PID tuning limits at the bottom of this file.
#define TUNE_BOTH_AXES // if uncommented alternates between pith and roll otherwise just pitch

#define ACROTRAINER_THRESHOLD 200 // inactive if commented out or > 500
#define DEADBAND 25 
#define ACC_TRIM_STEP_SIZE 16 // controls acc trim using stick calibration - increase if you are impatient 

#define USE_THROTTLE_CURVE // MW GUI throttle curve active - comment out if using baro, magnetometer and battery LVC

#define HK_PocketQuad
//#define MPU6050_DLPF_CFG MPU6050_LPF_42HZ 
//#define MPU6050_DLPF_CFG MPU6050_LPF_98HZ
#define MPU6050_DLPF_CFG MPU6050_LPF_188HZ

#define GYR_CMPF_FACTOR 1000 // faster angle estimate recovery after aerobatics but more jitters
//#define GYR_CMPF_FACTOR 4000 // better for crusing and less jitters

//#define MOTOR_STOP // comment out for slow motor run after arming
#define MINCOMMAND  1000
#define MINTHROTTLE 1050
#define MAXTHROTTLE 1850

// Battery

//#define LVC_LIMIT 32 // 0.1V
#define LVC_DELAY_TIME_S 2 // autoland after this delay in seconds
#define LVC_WARNING_PERCENT 90 // scales down desired throttle "suddenly" to this percentage when low volts reached
#define LVC_LANDING_TIME_S 3 // time for throttle to drop to zero
//#define USE_MINIMAL_LVC // reduces code size

// Altitude

//#define USE_BOSCH_BARO  
//#define USE_MS_BARO
//#define USE_MS5611_EXTENDED_TEMP  // optional if flying below 20C
#define USE_MW_ALT_CONTROL // original MW altitude hold scheme
//#define USE_SONAR

#define USE_PROP_ALT_HOLD // rate of climb is proportional to throttle 
#define ALT_HOLD_STEP 1
#define ALT_HOLD_LIMIT_M 20 // metres

// Magnetometer

//#define USE_BOSCH_MAG
#define MAG_DECLINATION (0) // use declination for your location

// Choose the option which gives almost no compass heading change when you tilt the quad about 20deg in any direction
// Package top upwards
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  Y; magADC[PITCH]  =  -X; magADC[YAW]  = -Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -X; magADC[PITCH]  =  -Y; magADC[YAW]  = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -Y; magADC[PITCH]  =  X; magADC[YAW]  = -Z;} // Goebish

// Package top downwards
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  Y; magADC[PITCH]  =  X; magADC[YAW]  = Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  -Y; magADC[YAW]  = Z;} // GY86 upside down SW I2C
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -Y; magADC[PITCH]  =  -X; magADC[YAW]  = Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -X; magADC[PITCH]  =  Y; magADC[YAW]  = Z;}

//_____________________________________________________________________________________________

// Configurations for gke's aircraft

#else

#define MW_WLTOYS
//#define MW_ECKS

//#define USE_QUICK_TUNE // based on Brad Quick's tuning scheme - change PID tuning limits at the bottom of this file.
//#define USE_QUICK_ALT_TUNE // tuning for altitude control - change PID tuning limits at the bottom of this file.
//DO NOT USE #define USE_RELAY_TUNE
#define TUNE_BOTH_AXES // if uncommented alternates between pitch and roll otherwise just pitch

//#define WOLFERL
//#define V20130614b
//#define GKE_EXP

#define USE_MW_CONTROL // original MW control scheme
//#define USE_MW_BASEFLIGHT_CONTROL // baseflight version of MW control scheme
//#define USE_MW_EXPANDED_CONTROL // original MW control scheme "flattened" to make code understandable

#define ALLOW_ARM_DISARM_VIA_TX_YAW
//#define ALLOW_ARM_DISARM_VIA_TX_ROLL

#define ACROTRAINER_THRESHOLD 200 // inactive if not defined or > 500
#define DEADBAND 25 // uSec  

#define USE_THROTTLE_CURVE // MW GUI throttle curve active

#define ALT_HOLD_LIMIT_M 20 // 120 // metres
#define ALT_HOLD_STEP 8

#define MAG_DECLINATION (-12) // use declination for your location

//#define DEBUG_RC // debug1 = glitch count, debug2 = frame width, debug3 = failsafe
//#define DEBUG_RELAY // debug1 = RollKu*100, debug2 = Rollw*10 ...
//#define DEBUG_ATTITUDE // debug1 = roll, debug2 = pitch, debug3 = yaw, debug4 = accOK
//#define DEBUG_BARO // debug1 = temp*100, debug1 = press*10, debug3 = groundalt, debug4 = alt
//#define DEBUG_ALT_HOLD // debug1 = rate of climb, debug2 = desired alt, debug3 = BaroPID, debug4 = throttle
//#define DEBUG_HEAD_HOLD // debug1 = heading, debug2 = desired heading, debug3 = error, debug4 = yaw command 

//________________________

#ifdef MW_ECKS

#define NANOWII
//#define MPU6050_DLPF_CFG MPU6050_LPF_42HZ
//#define MPU6050_DLPF_CFG MPU6050_LPF_98HZ
#define MPU6050_DLPF_CFG MPU6050_LPF_188HZ
//#define GYR_CMPF_FACTOR 1000 // faster angle estimate recovery after aerobatics but more jitters
#define GYR_CMPF_FACTOR 4000 // better for crusing and less jitters

#define MINCOMMAND  1000
#define MINTHROTTLE 1064 // special ESC (simonk)
#define MAXTHROTTLE 1850
#define SERIAL_SUM_PPM  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum

// Battery

#define LVC_LIMIT 33 // 0.1V
#define LVC_DELAY_TIME_S 2 // autoland after this delay in seconds
#define LVC_WARNING_PERCENT 90 // scales down desired throttle "suddenly" to this percentage when low volts reached
#define LVC_LANDING_TIME_S 10 // time for throttle to drop to zero
//#define USE_MINIMAL_LVC // reduces code size

// Altitude

//#define USE_BOSCH_BARO  
//#define USE_MS_BARO
//#define USE_MS5611_EXTENDED_TEMP  // optional if flying below 20C
#define USE_MW_ALT_CONTROL // original MW altitude hold scheme
//#define USE_SONAR

#define USE_PROP_ALT_HOLD // rate of climb is proportional to throttle 

// Magnetometer

//#define USE_BOSCH_MAG

// Choose the option which gives almost no compass heading change when you tilt the quad about 20deg in any direction
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

//________________________

#else // POCKET QUAD

#define SPEKTRUM 1024
//#define SERIAL_SUM_PPM  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum
//#define STANDARD_RX
//#define FLYSKY // not commissioned yet

#define USE_GKE_DM9_SPEKTRUM_SCALING // dodgy DM9 does not appear to capture pulse widths correctly?
//#define SPEKTRUM_RTF_SCALING // rescales and reverses yaw and roll for Blade RTF MLP4DSM game style Tx

#define HK_PocketQuad
//#define MPU6050_DLPF_CFG MPU6050_LPF_42HZ 
//#define MPU6050_DLPF_CFG MPU6050_LPF_98HZ
#define MPU6050_DLPF_CFG MPU6050_LPF_188HZ
//#define GYR_CMPF_FACTOR 1000 // faster angle estimate recovery after aerobatics but more jitters
#define GYR_CMPF_FACTOR 4000 // better for crusing and less jitters

#define MINCOMMAND  1000
#define MINTHROTTLE 1050
#define MAXTHROTTLE 1850

// Battery

//#define LVC_LIMIT 32 // 0.1V
#define LVC_DELAY_TIME_S 2 // autoland after this delay in seconds
#define LVC_WARNING_PERCENT 90 // scales down desired throttle "suddenly" to this percentage when low volts reached
#define LVC_LANDING_TIME_S 5 // time for throttle to drop to zero
//#define USE_MINIMAL_LVC // reduces code size

// Altitude

//#define USE_BOSCH_BARO  
//#define USE_MS_BARO
//#define USE_MS5611_EXTENDED_TEMP  // optional if flying below 20C
#define USE_MW_ALT_CONTROL // original MW altitude hold scheme
//#define USE_SONAR

#define USE_PROP_ALT_HOLD // rate of climb is proportional to throttle 

// Magnetometer

//#define USE_BOSCH_MAG

// Choose the option which gives almost no compass heading change when you tilt the quad about 20deg in any direction
// Package top upwards
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  Y; magADC[PITCH]  =  -X; magADC[YAW]  = -Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -X; magADC[PITCH]  =  -Y; magADC[YAW]  = -Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -Y; magADC[PITCH]  =  X; magADC[YAW]  = -Z;}

// Package top downwards
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  Y; magADC[PITCH]  =  X; magADC[YAW]  = Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  -Y; magADC[YAW]  = Z;} // GY86 upside down SW I2C
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -Y; magADC[PITCH]  =  -X; magADC[YAW]  = Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -X; magADC[PITCH]  =  Y; magADC[YAW]  = Z;}

#endif // MW_ECKS

#endif // GENERAL_USE

// Best not to change these ;)

#define FAILSAFE

//#define USE_MW_RC_FILTER // moving average RC jitter filter
#define USE_RC_FILTER // simple average
#define STICK_TO_ANGLE 2  // [0:500] -> [0:100] degrees
#define USE_ACC_LPF // enables accelerometer moving average filter
#define USE_GYRO_AVERAGE // remove some jitter by averaging last two readings
#define USE_HEADROOM

#if defined(MW_WLTOYS)
#define YAW_DIRECTION -1
#else
#define YAW_DIRECTION 1 // sets the sense of the yaw control
#endif

//____________________________________________________________________________________________

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
#define SERIAL2_COM_SPEED 115200
#define SERIAL3_COM_SPEED 115200

// Train Acro with auto recovery. Value set the point where ANGLE_MODE takes over.
// Remember to activate ANGLE_MODE first!...
// A Value on 200 will give a very distinct transfer */
#if !defined(ACROTRAINER_THRESHOLD)
#define ACROTRAINER_THRESHOLD 1000
#endif

// Failsafes

// Failsafe check pulses on four main control channels CH1-CH4. If the pulse is missing or below 985us (on any of these 
// four channels the failsafe procedure is initiated. After FAILSAFE_DELAY time from failsafe detection, the level mode,
// PITCH, ROLL and YAW is centered and THROTTLE is set to FAILSAFE_THR0TTLE value. You must set this value to descending 
// about 1m/s or so for best results. This value is depended from your configuration, AUW and some other params.  Next,  
// after FAILSAFE_OFF_DELAY the copter is disarmed, and motors is stopped. If RC pulse coming back before reached 
// FAILSAFE_OFF_DELAY time, after the small quard time the RC control is returned to normal. 

#define FAILSAFE_DELAY     10 // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
#define FAILSAFE_OFF_DELAY (FAILSAFE_DELAY*5) // Time for Landing before motors stop in 0.1sec. 
#define FAILSAFE_THROTTLE  (MINTHROTTLE + 200) // (*) Throttle level used for landing - may be relative to MINTHROTTLE 

// Tx

/// introduce a deadband around the stick center 
// Must be greater than zero, comment if you dont want a deadband on roll, pitch and yaw 
#if !defined(DEADBAND)
#define DEADBAND 0
#endif

#define MIDRC 1500

// Software I2C - SCL is pin SCK/YAW, SDA is pin MISO/Pitch CAUTION VCC pin is 5V which will destroy some sensor boards 

#define I2C_SW 1  // software I2C bus

#if !defined(STANDARD_RX)
  #define USE_5V_ON_MOSI  // external sensors use MOSI pin as 3V supply - CAUTION 10mA @5V/5mA @3.3V
  #define I2CBARO 1
  #define I2CMAG 1
  #define I2CMPU 0
#endif 

// Autotune

// Choose sensible limits - tuning is constrained to being with these values
// The limits MUST be positive and greater than 1

// Pitch/Roll
#define MIN_P 20
#define MAX_P 60

#define MIN_I 1
#define MAX_I 50

#define MIN_D 2
#define MAX_D 40

// Altitude
#define MIN_ALT_P 55
#define MAX_ALT_P 75

#define MIN_ALT_I 15
#define MAX_ALT_I 35

#define MIN_ALT_D 2
#define MAX_ALT_D 30

#if (defined(USE_BOSCH_MAG) || defined(USE_MS_BARO) || defined(USE_BOSCH_BARO))
#define CYCLETIME 3000 
#else
#define CYCLETIME 2048 // no point in going faster :)
#endif
