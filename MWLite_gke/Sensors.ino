
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

enum MPU6050LPFs {
  MPU6050_LPF_256HZ, MPU6050_LPF_188HZ, MPU6050_LPF_98HZ, MPU6050_LPF_42HZ, MPU6050_LPF_20HZ, MPU6050_LPF_10HZ, MPU6050_LPF_5HZ};


#define MPU6050_ADDRESS (0x68<<1) // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board

#if !defined(MPU6050_DLPF_CFG)
#define MPU6050_DLPF_CFG   MPU6050_LPF_188HZ
#endif

void conditionGyro(void) {
  static int16_t gyroADCp[3] = {
    0,0,0
  };
  static int32_t g[3];
  static uint8_t axis;

  if (calibratingG > 0) {
    if ((calibratingG & 0x1f) == 0) LED_BLUE_TOGGLE; 
    delay(1);
    if (calibratingG == 512)
      for (axis = ROLL; axis <= YAW; axis++)
        global_conf.gyroZero[axis] = g[axis] = 0;

    for (axis = 0; axis < 3; axis++) 
      g[axis] += (int32_t)gyroADC[axis];

    if (calibratingG == 1) {
      for (axis = ROLL; axis <= YAW; axis++) {
        global_conf.gyroZero[axis] = (g[axis] + 256) >> 9;
        writeGlobalSet(0);
        blinkLED(50,1); 
      }
    }
    calibratingG--;
  }

  for (axis = ROLL; axis <= YAW; axis++)
#if defined(USE_GYRO_AVERAGE)
    if (axis != YAW)
      gyroData[axis] = (gyroData[axis] + (gyroADC[axis] - global_conf.gyroZero[axis]) + 1) >> 1; 
    else
      gyroData[axis] = (gyroADC[axis] - global_conf.gyroZero[axis]);
#else 
  gyroData[axis] = gyroADC[axis] - global_conf.gyroZero[axis]; 
#endif

} // conditionGyro

int16_t inline accFilter(uint8_t axis, int32_t a) {
#if defined (USE_ACC_LPF)
  static int32_t F[3] = {
    0, 0, GRAVITY
  };

  if (!(f.TUNE_MODE )) { // || f.RELAY_MODE)) {
    F[axis] -= F[axis] >> 4;
    F[axis] += a;
    return (F[axis] >> 4); 
  }
  else  
    return(a);
#else
  return (a);
#endif

} // AccFilter

void conditionAcc(void) {
  static int32_t a[3];
  static uint8_t axis;

  if (calibratingA > 0) {

    if ((calibratingA & 0x1f) == 0) LED_BLUE_TOGGLE; 
    delay(1);

    if (calibratingA == 512) {
      for (axis = 0; axis < 3; axis++)
        a[axis] = 0;
      f.ACC_CALIBRATED = false;
    }  

    for (axis = 0; axis < 3; axis++) 
      a[axis] += (int32_t)accADC[axis];

    if (calibratingA == 1) {
      for (axis = 0; axis < 3; axis++) 
        global_conf.accZero[axis] = (a[axis] + 256) >> 9;
      global_conf.accZero[YAW] -= GRAVITY; 
      global_conf.accCalibrated = f.ACC_CALIBRATED = true; 
      writeGlobalSet(1);              
    }
    calibratingA--;
  }

  for (axis=0; axis < 3; axis++)
    accData[axis] = accFilter(axis, accADC[axis] - global_conf.accZero[axis]);
} // conditionAcc


void initMPU6050(void) {

  delay(100); // let things settle

  i2cWriteReg(I2CMPU, MPU6050_ADDRESS, 0x6B, 0x80);
  delay(5);

  i2cWriteReg(I2CMPU, MPU6050_ADDRESS, 0x6B, 0x03);           
  i2cWriteReg(I2CMPU, MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG);
  i2cWriteReg(I2CMPU, MPU6050_ADDRESS, 0x1B, 0x18); // +/- 2000deg
  i2cWriteReg(I2CMPU, MPU6050_ADDRESS, 0x1C, 0x10); // +/-4096 1G

  delay(100);

} // initMPU6050

void getRatesAndAccelerations(void) {
  static uint8_t rawADC[14];

  i2cReadToBuf(I2CMPU, MPU6050_ADDRESS, 0x3B, rawADC, 14);

  ACC_ORIENTATION( ((rawADC[0] << 8) | rawADC[1]), 
  ((rawADC[2] << 8) | rawADC[3]),
  ((rawADC[4] << 8) | rawADC[5]));
  conditionAcc();

  // 6 & 7 is temperature

  GYRO_ORIENTATION( ((rawADC[8] << 8) | rawADC[9]),
  ((rawADC[10] << 8) | rawADC[11]),
  ((rawADC[12] << 8) | rawADC[13]));
  conditionGyro();

} // getRatesAndAccelerations


//______________________________________________________________________________________________

static float baroPressure;
static int16_t baroTemperature;

// BMP085 Barometer

#define BMP085_ADDRESS           0xee
#define BMP085_OSS               3 // 0 4.5mS, 1 7.5mS, 2 13.5mS, 3 25.5mS
#define BMP085_PRESS_US          25500 // 13500 make sum 25mS to reduce ROC jitter
#define BMP085_TEMP_US           4500  // 4500
#define BMP085_ID		0x55
#define BMP085_TEMP 	        0x2e
#define BMP085_PRESS 	        0x34	
#define BMP085_CTL 		0xf4
#define BMP085_ADC_MSB 		0xf6
#define BMP085_ADC_LSB  		0xf7
#define BMP085_ADC_XLSB 		0xf8
#define BMP085_TYPE 		0xd0
#define BMP085_PROM_START_ADDR	0xaa
#define BMP085_PROM_DATA__LEN	(22>>1)

static struct {
  int16_t  ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t  b1, b2, mb, mc, md;
  union {
    uint16_t val; 
    uint8_t raw[2]; 
  } 
  ut; //uncompensated T
  union {
    uint32_t val; 
    uint8_t raw[4]; 
  } 
  up; //uncompensated P
  boolean  state;
  uint32_t deadline;
} 
bmp085_ctx;  


void bmp085ReadCalibration(void){
  int16_t *p;
  uint8_t b[32];

  delay(20);
  i2cReadi16vAtAddr(I2CBARO, BMP085_ADDRESS, BMP085_PROM_START_ADDR, &bmp085_ctx.ac1, BMP085_PROM_DATA__LEN, true);

} // bmp085ReadCalibration

void  bmp085Init(void) {

  f.BARO_ACTIVE = i2cReadReg(I2CBARO, BMP085_ADDRESS, BMP085_TYPE) == BMP085_ID;
  if (f.BARO_ACTIVE) {
    delay(10);
    bmp085ReadCalibration();
    delay(5);
    bmp085Start(BMP085_TEMP); 
    bmp085_ctx.deadline = micros() + BMP085_TEMP_US;
    bmp085_ctx.state = true;
  }
} // bmp085Init

void bmp085Start(uint8_t c) {
  i2cWriteReg(I2CBARO, BMP085_ADDRESS, BMP085_CTL, c);
} // bmp085Start

void bmp085ReadPressure(void) {
  uint8_t b[3];

  slotFree = false;

  bmp085_ctx.up.val = 0;
  if (BMP085_OSS > 0) {
    i2cReadToBuf(I2CBARO, BMP085_ADDRESS, BMP085_ADC_MSB, b, 3);
    bmp085_ctx.up.raw[2] = b[0];
    bmp085_ctx.up.raw[1] = b[1];
    bmp085_ctx.up.raw[0] = b[2]; 
    bmp085_ctx.up.val >>= (8 - BMP085_OSS);
  }
  else
  {
    i2cReadToBuf(I2CBARO, BMP085_ADDRESS, BMP085_ADC_MSB, b, 2);
    bmp085_ctx.up.raw[1] = b[0];
    bmp085_ctx.up.raw[0] = b[1];  

  }
} // bmp085ReadPressure

void bmp085ReadTemperature(void) {
  uint8_t b[2];

  slotFree = false;

  i2cReadToBuf(I2CBARO, BMP085_ADDRESS, BMP085_ADC_MSB, b, 2);
  bmp085_ctx.ut.raw[1] = b[0];
  bmp085_ctx.ut.raw[0] = b[1];
} // bmp085ReadTemperature


void bmp085Compensate(void) {
  int32_t  x1, x2, x3, b3, b5, b6, p, tmp;
  uint32_t b4, b7;

  x1 = (((int32_t) bmp085_ctx.ut.val - (int32_t) bmp085_ctx.ac6) * (int32_t) bmp085_ctx.ac5) >> 15;
  x2 = ((int32_t) bmp085_ctx.mc << 11) / (x1 + bmp085_ctx.md);
  b5 = x1 + x2;
  baroTemperature = (b5 * 10 + 8) >> 4; // 0.01C

  b6 = b5 - 4000;

  x1 = (b6 * b6) >> 12;
  x1 *= bmp085_ctx.b2;
  x1 >>= 11;

  x2 = (bmp085_ctx.ac2 * b6);
  x2 >>= 11;

  x3 = x1 + x2;

  b3 = ((((int32_t) bmp085_ctx.ac1 * 4 + x3) << BMP085_OSS) + 2) >> 2;

  x1 = (bmp085_ctx.ac3 * b6) >> 13;
  x2 = (bmp085_ctx.b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (bmp085_ctx.ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = (uint32_t) ((bmp085_ctx.up.val) - b3) * (50000 >> BMP085_OSS);
  p = (b7 < 0x80000000) ? p = (b7 << 1) / b4 : p = (b7 / b4) * 2;

  x1 = p >> 8;
  x1 *= x1;
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;

  baroPressure = (float)(p + ((x1 + x2 + 3791) >> 4)); // pressure in Pa

} // bmp085Compensate

boolean bmp085Update(void) { 
  boolean r = false;

  if (f.BARO_ACTIVE && slotFree) {
    NowuS = micros();
    if (NowuS >= bmp085_ctx.deadline) {
      if (bmp085_ctx.state) {
        bmp085ReadTemperature(); 
        bmp085Start(BMP085_PRESS | (BMP085_OSS<<6)); 
        bmp085_ctx.deadline = NowuS + BMP085_PRESS_US;
        bmp085_ctx.state = false; 
      } 
      else
      {
        bmp085ReadPressure(); 
        bmp085Start(BMP085_TEMP);
        bmp085_ctx.deadline = NowuS + BMP085_TEMP_US; 

        bmp085Compensate();
        densityAltitude = CalculateDensityAltitude(baroPressure);
        conditionBaro();
        r = true;

        bmp085_ctx.state = true; 
      } 
    }
  }
  return(r);
} // bmp085Update

//______________________________________________________________________________________________

// MS5611 Barometer

#define MS5611_ADDRESS         0xee //CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND)
//#define MS5611_ADDRESS       0xec //CBR=1 0xEC I2C address when pin CSB is connected to HIGH (VCC)

#define MS5611_PROM 		0xA0
#define MS5611_PRESS 		0x40
#define MS5611_TEMP 		0x50
#define MS5611_RESET 		0x1E

// OSR (Over Sampling Ratio) constants
#define MS5611_OSR_256 		0x00 //0.065 mBar  0.6mS
#define MS5611_OSR_512 		0x02 //0.042 1.17mS
#define MS5611_OSR_1024 	0x04 //0.027 2.28mS
#define MS5611_OSR_2048 	0x06 //0.018 4.54mS
#define MS5611_OSR_4096 	0x08 //0.012 9.04mS
#define MS5611_OSR              MS5611_OSR_4096
#define MS5611_TIME_US		12500 // actually 9.04mS for OSR_4096 but increase to reduce jitter on ROC

static struct {
  uint16_t c[7];
  uint32_t utval; 
  uint32_t upval; 
  boolean  state;
  uint32_t deadline;
} 
ms5611_ctx;


void ms5611Reset(void){
  i2cWriteReg(I2CBARO, MS5611_ADDRESS, 0, MS5611_RESET);
  delay(200);
} // ms5611Reset

void ms5611ReadCalibration(void){
  uint8_t i, b[2];

  delay(20);
  for (i = 0; i < 6; i++) { // cannot do a block read - just one at a time
    i2cReadToBuf(I2CBARO, MS5611_ADDRESS, (MS5611_PROM + 2) +  i*2 , b, 2);
    ms5611_ctx.c[i] = (uint16_t)(b[0] << 8) | b[1];
  }

} // ms5611ReadCalibration

void ms5611Start(uint8_t c) {
  i2cWrite(I2CBARO, MS5611_ADDRESS, c + MS5611_OSR);
} // ms5611Start

uint32_t  ms5611Read(void) {
  union {
    uint32_t val; 
    uint8_t raw[4]; 
  } 
  b;  
  uint8_t raw[3];

  slotFree = false;

  i2cReadToBuf(I2CBARO, MS5611_ADDRESS, 0, raw, 3);

  b.raw[3] = 0;
  b.raw[2] = raw[0];
  b.raw[1] = raw[1];
  b.raw[0] = raw[2];

  return(b.val);
} // ms5611Read

void ms5611Init(void) {

  f.BARO_ACTIVE = i2cResponse(I2CBARO, MS5611_ADDRESS);
  if (f.BARO_ACTIVE) {
    delay(10);
    ms5611Reset();
    delay(100);
    ms5611ReadCalibration();
    delay(10);
    ms5611Start(MS5611_TEMP); 
    ms5611_ctx.state = true;
    ms5611_ctx.deadline = micros() + MS5611_TIME_US; 
  }
  else
    relativeAltitude = ROC = altPID = 0; 

} // ms5611Init

void ms5611Compensate(void) { 
  int64_t off, sens;
  int32_t off2, sens2, delt;

  //#define FABIO_IMU
#if defined(FABIO_IMU)
  int32_t ms5611dt;

  ms5611dt = (int32_t)(ms5611_ctx.utval - ((uint32_t)ms5611_ctx.c[4] << 8));
  baroTemperature = 2000 + ((ms5611dt * ms5611_ctx.c[5]) >> 23); // gives low temp - int truncation?

  off = ((uint32_t)ms5611_ctx.c[1] << 16) + (((int64_t)ms5611dt * ms5611_ctx.c[3]) >> 7); //ok
  sens = ((uint32_t)ms5611_ctx.c[0] << 15) + (((int64_t)ms5611dt * ms5611_ctx.c[2]) >> 8); //ok
  baroPressure = ((( (ms5611_ctx.upval * sens ) >> 21) - off) >> 11) * 0.0625; //ok

#else
  int64_t ms5611dt;

  ms5611dt = (int64_t)ms5611_ctx.utval - ((uint32_t) ms5611_ctx.c[4] << 8);
  baroTemperature = 2000 + ((ms5611dt * ms5611_ctx.c[5]) >> 23); // 0.01C

  off = ((int64_t)ms5611_ctx.c[1] << 16) + ((ms5611dt * ms5611_ctx.c[3]) >> 7);
  sens = ((int64_t)ms5611_ctx.c[0] << 15) + ((ms5611dt * ms5611_ctx.c[2]) >> 8);

#if defined(USE_MS5611_EXTENDED_TEMP)
  if (baroTemperature < 2000) { // < 20C
    delt = baroTemperature - 2000;
    delt  = 5 * sq(delt);
    off2  = delt >> 1;
    sens2 = delt >> 2;
    if (baroTemperature < -1500) { // < -15C
      delt  = baroTemperature + 1500;
      delt  = sq(delt);
      off2  += 7 * delt;
      sens2 += (11 * delt) >> 1;
    }
    off  -= off2; 
    sens -= sens2;
  }
#endif

  baroPressure = ((((ms5611_ctx.upval * sens) >> 21) - off) >> 11) * 0.0625;

#endif // FABIO_IMU

} // ms5611Compensate

boolean ms5611Update(void) { 
  boolean r = false;

  if (f.BARO_ACTIVE && slotFree) {
    NowuS = micros();
    if (NowuS >= ms5611_ctx.deadline) {
      ms5611_ctx.deadline = NowuS + MS5611_TIME_US; 
      if (ms5611_ctx.state) {
        ms5611_ctx.utval = ms5611Read(); 
        ms5611Start(MS5611_PRESS);
        ms5611_ctx.state = false;
      } 
      else {
        ms5611_ctx.upval = ms5611Read(); 
        ms5611Start(MS5611_TEMP); 
        ms5611Compensate(); 
        densityAltitude = CalculateDensityAltitude(baroPressure);
        conditionBaro();
        ms5611_ctx.state = true; 
        r = true;
      }
    }
  } 
  return(r);
} // ms5611Update

//______________________________________________________________________________________________

// Rangefinder/Sonar

#define MAXBOTIX_TIME_US 25000 // can go faster but PID tuning will be different
#define MAXBOTIX_CAL_CM 1 

boolean rangefinderUpdate(void) {
  static uint32_t nextUpdateuS = micros();
  boolean r = false;

  if (f.SONAR_ACTIVE && slotFree) {
    NowuS = micros();
    if (NowuS >= nextUpdateuS) {
      nextUpdateuS = NowuS + MAXBOTIX_TIME_US;
      relativeAltitude = analogRead(SONAR_PIN) * MAXBOTIX_CAL_CM; 
      r = true;
    }
  }

  return(r);
} // rangefinderUpdate

void rangefinderInit(void) {
  f.SONAR_ACTIVE = true;
} // rangefinderInit

//______________________________________________________________________________________________

// Altitude Common

boolean updateAltitude(void) {

#if defined(USE_BOSCH_BARO)
  return(bmp085Update());
#elif defined(USE_MS_BARO)
  return(ms5611Update());
#elif defined(USE_SONAR)
  return(rangefinderUpdate());
#else
  return(false);
#endif
} // updateBaro


int32_t CalculateDensityAltitude(float p) {
  int32_t a;
  //#define INC_BARO_FULL_MATH
#if defined(INC_BARO_FULL_MATH)
  a = (1.0f - pow(p / 101325.0f, 0.190295f)) * 4433000.0f;
#else
  a = (101325.0f - p) * 8.406f; // ~calibration to 200M
#endif
  // relativeAltitude = (relativeAltitudep * 6 + BaroAlt * 2) >> 3;
  return(a);
} // CalculateDensityAltitude

void conditionBaro(void) {
  static int16_t densityGroundAltitude = 0;
  static int32_t Average;

  if (calibratingB > 0) { 
    if (calibratingB == 32) {
      densityGroundAltitude = densityAltitude;     
      Average = 0;
    } 

    Average += densityAltitude;

    if (calibratingB == 1) 
      densityGroundAltitude = Average >> 5;

    calibratingB--;
  } 

  relativeAltitude = (int16_t)(densityAltitude - densityGroundAltitude);


#if defined(DEBUG_BARO)
  debug[0] = baroTemperature;
  debug[1] = baroPressure/10;
  debug[2] = densityGroundAltitude;
  debug[3] = densityAltitude;
#endif

} // conditionBaro

//______________________________________________________________________________________________

// HMC5883L Magnetometer

static float   magGain[3] = {
  1.0,1.0,1.0};  // gain for each axis, populated at sensor init
static boolean magInit = false;

#define HMC5883_R_CONFA 0
#define HMC5883_R_CONFB 1
#define HMC5883_R_MODE 2
#define HMC5883_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC5883_Y_SELF_TEST_GAUSS (+1.16)   //!< Y axis level when bias current is applied.
#define HMC5883_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

#define MAG_UPDATE_MS 100
#define MAG_UPDATE_US (MAG_UPDATE_MS*1000L)
#define MAG_CAL_US  30000000L

#define MAG_ADDRESS (0x1E*2)
#define MAG_DATA_REGISTER 0x03

boolean hmc5883Active(void) {
  return (i2cReadReg(I2CMAG, MAG_ADDRESS, 0x0a) == 'H');
} //  hmc5883Active

void hmc5883Init(void) {
#define MAG_SAMPLES 10
  int32_t magSum[3] = {
    0,0,0    
  };  
  uint8_t bias, axis, i;

  f.MAG_ACTIVE = hmc5883Active();
  if (f.MAG_ACTIVE) {

    delay(50); 

    for ( bias = HMC_POS_BIAS; bias <= HMC_NEG_BIAS; bias++) {

      i2cWriteReg(I2CMAG, MAG_ADDRESS,HMC5883_R_CONFA, 0x010 + bias); 
      i2cWriteReg(I2CMAG, MAG_ADDRESS, HMC5883_R_CONFB, 2 << 5);  //Set the Gain
      i2cWriteReg(I2CMAG, MAG_ADDRESS,HMC5883_R_MODE, 1);

      delay(MAG_UPDATE_MS);
      getHMC5883(); // discard first sample

      for (i = 0; i < MAG_SAMPLES; i++) { 
        i2cWriteReg(I2CMAG, MAG_ADDRESS,HMC5883_R_MODE, 1);      
        delay(MAG_UPDATE_MS);
        LED_BLUE_TOGGLE; 
        if (getHMC5883()) { 
          for (axis = 0; axis < 3; axis++)
            if (bias == HMC_POS_BIAS)
              magSum[axis] += magADC[axis];
            else
              magSum[axis] -= magADC[axis];
        } 
        else {
          magInit = f.MAG_ACTIVE = false;
          return;
        }
      }
    }

    for (axis = 0; axis < 3; axis++)
      magGain[axis] = abs((820.0 * HMC5883_X_SELF_TEST_GAUSS * 2.0 * MAG_SAMPLES) / magSum[axis]);

    i2cWriteReg(I2CMAG, MAG_ADDRESS ,HMC5883_R_CONFA ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    i2cWriteReg(I2CMAG, MAG_ADDRESS ,HMC5883_R_CONFB ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    i2cWriteReg(I2CMAG, MAG_ADDRESS ,HMC5883_R_MODE  ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
    delay(MAG_UPDATE_MS);

    magInit = true;
  }
} //  hmc5883Init

boolean getHMC5883(void) { // 460uS
  uint8_t rawADC[6];

  slotFree = false;

  i2cReadToBuf(I2CMAG, MAG_ADDRESS, MAG_DATA_REGISTER, rawADC, 6);
  MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]),
  ((rawADC[4]<<8) | rawADC[5]),
  ((rawADC[2]<<8) | rawADC[3]) );

  return((magADC[0] != -4096) && (magADC[1] != -4096) && (magADC[2] != -4096));
} // getHMC5883


boolean hmc5883Update(void) { // 530uS
  static uint32_t CalTimeruS = 0;
  static uint32_t hmc5883deadline = 0;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];
  boolean r = false;
  uint8_t axis;

  if (f.MAG_ACTIVE && slotFree ) {
    NowuS = micros();
    if (NowuS >= hmc5883deadline) { 
      hmc5883deadline = NowuS + MAG_UPDATE_US;

      if (getHMC5883()) {
        for(axis = 0; axis < 3; axis++)
          magADC[axis]  = magADC[axis]  * magGain[axis];

        if (f.CALIBRATE_MAG) {
          CalTimeruS = NowuS + MAG_CAL_US;
          for(axis = 0;axis < 3; axis++) {
            global_conf.magZero[axis] = 0;
            magZeroTempMin[axis] = magADC[axis];
            magZeroTempMax[axis] = magADC[axis];
          }
          f.CALIBRATE_MAG = f.MAG_CALIBRATED = false;
        }

        if (magInit) // we apply offset only once mag calibration is done
          for(axis = 0; axis < 3; axis++)
            magADC[axis]  -= global_conf.magZero[axis];

        if (CalTimeruS != 0) {
          if (NowuS < CalTimeruS) { // 30s: you have 30s to turn the multi in all directions
            for(axis = 0; axis < 3; axis++) {
              if (magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magADC[axis]; 
              if (magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magADC[axis];
            }
          } 
          else {
            CalTimeruS = 0;
            for(axis = 0; axis < 3; axis++)
              global_conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])>>1;
            f.MAG_CALIBRATED = global_conf.magCalibrated = true;
            writeGlobalSet(1);
          }
        } 
      }
    }
  }
  return (r);
} // hmc5883Update

boolean updateMagnetometer(void) {
#if defined(USE_BOSCH_MAG)
  return(hmc5883Update());
#else
  return(false);
#endif 
} // updateMagnetometer

//______________________________________________________________________________________________

// Battery 

#define VBAT_UPDATE_MS 25
#define VBAT_BUCKET_TRIG (2000/VBAT_UPDATE_MS)
#define LVC_DECAY (VBAT_UPDATE_MS/5)
#define LVC_FAST_DECAY (VBAT_UPDATE_MS/2)

void checkBattery(void) { // 28-32uS addition due to Goebish, bucket suggestion vlad_vy
#if defined(LOW_VOLTAGE_LIMIT)
  enum lvcStates {
    Monitor, Warning, Wait, Land                                      };
  static uint8_t lvcState = Monitor;
  uint32_t NowmS;
  static uint32_t vbatUpdatemS = millis();
  static uint32_t lvcTimeoutmS = millis();
  static int8_t bucket = VBAT_BUCKET_TRIG;
  static uint32_t v = 0;
  static uint16_t newv;

  NowmS = millis();
  if (NowmS > vbatUpdatemS ) {
    vbatUpdatemS = NowmS + VBAT_UPDATE_MS;

    newv = (((int32_t)analogRead(VBAT_PIN) * 200L * (VOLTS_RTOP+VOLTS_RBOT))/VOLTS_RBOT + 512) >> 10; 

    v = (v * 3 + newv + 4) >> 2; 
    analog.vbat = v >> 2;

#if defined(USE_MINIMAL_LVC)
    if ((analog.vbat > LOW_VOLTAGE_LIMIT) && !f.ALARM) {
      if (bucket < VBAT_BUCKET_TRIG) 
        bucket += 2;
    }
    else      
      if (bucket <= 0) {
      f.ALARM = true;
      if (throttleLVCScale >= LVC_FAST_DECAY)  
        throttleLVCScale -= LVC_FAST_DECAY;
      else 
        doDisarm(); 
    } 
    else bucket--; 
#else
    switch (lvcState) {
    case Monitor:
      f.ALARM = false;
      if (analog.vbat <= LOW_VOLTAGE_LIMIT )
        lvcState = Warning;
      else
        if (bucket < VBAT_BUCKET_TRIG) 
          bucket += 2;
      break;
    case Warning:
      f.ALARM = true;
      if (analog.vbat > LOW_VOLTAGE_LIMIT ) {
        throttleLVCScale = 1024;
        lvcState = Monitor;   
      } 
      else
        if (bucket <= 0) {
          throttleLVCScale = ((LOW_VOLTAGE_WARNING_PERCENT * 1024L)/100);
          lvcTimeoutmS = NowmS + (LOW_VOLTAGE_TIMEOUT_S * 1000);  
          lvcState = Wait;
        }
        else
          bucket--;
      break;
    case Wait:
      debug[3] = lvcTimeoutmS;
      if (NowmS > lvcTimeoutmS)
        lvcState = Land;    
      break;
    case Land:
      if (throttleLVCScale >= LVC_DECAY)  
        throttleLVCScale -= LVC_DECAY;
      else 
        doDisarm();  
      break;
    } // switch 
#endif 
  }
#endif
} // checkBattery
//______________________________________________________________________________________________


void initSensors(void) {
  delay(200);
  delay(100);
  i2cInit();
  delay(200);
  initMPU6050();
#if defined(USE_BOSCH_BARO)
  bmp085Init();
#elif defined(USE_MS_BARO)
  ms5611Init();
#elif defined(USE_SONAR)  
  rangefinderInit();
#endif
#if defined(USE_BOSCH_MAG)
  hmc5883Init();
#endif
} // initSensors













































































































































