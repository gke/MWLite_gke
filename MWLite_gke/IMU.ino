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

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//

//  advanced users settings

// Set the Gyro Weight for Gyro/Acc complementary filter
// Increasing this value would reduce and delay Acc influence on the output of the filter
// originally the value was 600 rescaled to 4800 with removal of /8 on acc range. 
// 1000 gives good angle recovery with some jitters after full-on aerobatics.
// 5000 gives smooth cruising flight with minimal jitters.

#ifndef GYR_CMPF_FACTOR
#define GYR_CMPF_FACTOR 1000 // [1000..5000]
#endif

// Set the Gyro Weight for Gyro/Magnetometer complementary filter
// Increasing this value would reduce and delay Magnetometer influence on the output of the filter
#define GYR_CMPFM_FACTOR 250

//****** end of advanced users settings *************
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

//****** end of advanced users settings *************

//1998?
#define GYRO_SCALE ((1999.0f/32768.0f) * (PI/180.0f) * 0.000001f) //MPU6050
//#define GYRO_SCALE (0.001064225154f * 0.000001f)

typedef struct fp_vector {		
  float X,Y,Z;		
} 
t_fp_vector_def;

typedef union {		
  float A[3];		
  t_fp_vector_def V;		
} 
t_fp_vector;

typedef struct int32_t_vector {
  int32_t X,Y,Z;
} 
t_int32_t_vector_def;

typedef union {
  int32_t A[3];
  t_int32_t_vector_def V;
} 
t_int32_t_vector;

int16_t _atan2(int32_t y, int32_t x){ // -1800:1800
  float z;
  int16_t a;

  z = (float)y / x;
  if ( abs(y) < abs(x) ){
    a = 573 * z / (1.0f + 0.28f * sq(z));
    if (x < 0) 
      if (y < 0) a -= 1800;
      else a += 1800;
  } 
  else {
    a = 900 - 573 * z / (sq(z) + 0.28f);
    if (y<0) a -= 1800;
  }
  return a;
} // atan2

float invSqrt (float x){ 
  union{  
    int32_t i;  
    float   f; 
  } 
  conv; 
  conv.f = x; 
  conv.i = 0x5f3759df - (conv.i >> 1); 
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
} // invSqrt

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL] * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL] * v_tmp.Z - delta[YAW] * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW] * v_tmp.X;
} // rotateV

static t_fp_vector EstG;
static t_int32_t_vector EstG32 = {
  0,0,1};
static t_int32_t_vector EstM32;
static float invG;
static t_fp_vector EstM;
  
void getEstimatedAttitude(void){
  static uint32_t PrevuS;
  uint32_t TimeruS;
  int16_t Temp;
  uint8_t axis;
  int32_t sqGZ, sqGX, sqGY, sqGX_sqGZ, accMagSq; 
  float invmagXZ;  
  float gyroScale, deltaGyroAngle[3];
  boolean accOK;

  NowuS = micros();
  gyroScale = (NowuS - PrevuS) * GYRO_SCALE;
  PrevuS = NowuS;

  accMagSq = 0;
  for (axis = ROLL; axis <= YAW; axis++) {
    deltaGyroAngle[axis] = (float)gyroData[axis] * gyroScale; 
    accMagSq += sq((int32_t)accData[axis]);
  }
  accMagSq = (accMagSq * 100) / GRAVITY_SQ;

  rotateV(&EstG.V, deltaGyroAngle);

  accOK = (accMagSq > 72) && (accMagSq < 133); // >1.15G or <0.85G 
  if (accOK) 
    for (axis = ROLL; axis <= YAW; axis++) 
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + (float)accData[axis]) * INV_GYR_CMPF_FACTOR;

  for (axis = ROLL; axis <= YAW; axis++)
    EstG32.A[axis] = EstG.A[axis];

  // Attitude of the estimated vector
  sqGZ = sq(EstG32.V.Z);
  sqGX = sq(EstG32.V.X);
  sqGY = sq(EstG32.V.Y);
  sqGX_sqGZ = sqGX + sqGZ;
  invmagXZ  = invSqrt(sqGX_sqGZ);
  invG = invSqrt(sqGX_sqGZ + sq(EstG32.V.Y));
  angle[ROLL] = _atan2(EstG32.V.X , EstG32.V.Z);
  angle[PITCH] = _atan2(EstG32.V.Y , invmagXZ * sqGX_sqGZ);

  f.SMALL_ANGLE_25DEG = abs(angle[ROLL] < 250) && abs(angle[PITCH] < 250);

#if defined(USE_MAG)
  if (f.MAG_ACTIVE) {
    rotateV(&EstM.V,deltaGyroAngle);

    for (axis = ROLL; axis <= YAW; axis++) 
      EstM32.A[axis] = EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + magADC[axis]) * INV_GYR_CMPFM_FACTOR;

    angle[YAW] = _atan2(
    EstM32.V.Z * EstG32.V.X - EstM32.V.X * EstG32.V.Z,
    EstM32.V.Y * invG * sqGX_sqGZ  - (EstM32.V.X * EstG32.V.X + EstM32.V.Z * EstG32.V.Z) * invG * EstG32.V.Y ); 
    angle[YAW] = (angle[YAW] - MAG_DECLINATION * 10)/10;
  } 
  else 
#endif // USE_MAG
  {
    // use direct integration
    YawAngle += deltaGyroAngle[YAW]; 
    Temp = YawAngle * (180.0/PI);
    while (Temp < 0)
      Temp += 360;
    while (Temp >= 360)
      Temp-= 360;
    angle[YAW] = Temp;
  }

#if defined(DEBUG_ATTITUDE)
  for (axis = ROLL; axis <= YAW; axis++) 
    debug[axis] = angle[axis];
  debug[3] = accOK;
#endif
} // getEstimatedAttitude

#define ACC_Z_DEADBAND (GRAVITY >> 5) 

void calculateVerticalAcceleration(void) {
  static int16_t accZoffset;

  //#if defined(USE_MW_ALT_CONTROL)
  // projection of ACC vector to global Z, with 1G subtracted
  // Math: accZ = A * G / |G| - 1G
  //  accZ = (accData[ROLL] * EstG32.V.X + accData[PITCH] * EstG32.V.Y + accData[YAW] * EstG32.V.Z) * invG;
  //#else
  accZ = accData[YAW] - GRAVITY;
  //#endif

  if (f.SMALL_ANGLE_25DEG) {    
    if (!f.ARMED) {
      accZoffset -= accZoffset >> 3;
      accZoffset += accZ;
    }  
    accZ -= accZoffset >> 3;
    accZ = Threshold(accZ, ACC_Z_DEADBAND);
  } 
  else
    accZ = 0;
} // calculateVerticalAcceleration



































