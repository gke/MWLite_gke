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
 
 #include <avr/eeprom.h>


uint8_t calculate_sum(uint8_t *cb , uint8_t siz) {
  uint8_t sum = 0x55;  // checksum init
  while(--siz) sum += *cb++;  // calculate checksum (without checksum byte)
  return sum;
} // calculate_sum

void readGlobalSet() {
  eeprom_read_block((void*)&global_conf, (void*)0, sizeof(global_conf));

  f.ACC_CALIBRATED = global_conf.accCalibrated;
  f.MAG_CALIBRATED = global_conf.magCalibrated;
} // readGlobalSet

void readEEPROM() {
  uint8_t i;
  int16_t tmp, y;
  int16_t thrMid, thrExpo;

  eeprom_read_block((void*)&conf, (void*)(sizeof(global_conf)), sizeof(conf));  
  if((calculate_sum((uint8_t*)&conf, sizeof(conf)) != conf.checksum) || ( conf.thisVersion != VERSION)) {
    blinkLED(100,3);    
    LoadDefaults(); 

    memset(&global_conf, 0, sizeof(global_conf));
    writeGlobalSet(1);
    readGlobalSet(); 
  }

#if defined(USE_THROTTLE_CURVE)

  for(i = 0; i<11; i++ ) {
    tmp = 10 * i - conf.thrMid8;
    y = 1;
    if ( tmp > 0 ) y = 100 - conf.thrMid8;
    if ( tmp < 0 ) y = conf.thrMid8;

    thrCurve[i] = 10 * conf.thrMid8 + tmp*( 100 - conf.thrExpo8 + (int32_t)conf.thrExpo8 * sq(tmp)/sq(y) )/10; // [0;1000]
    thrCurve[i] = conf.minthrottle + (int32_t)(MAXTHROTTLE - conf.minthrottle) * thrCurve[i]/1000; // [0;1000] -> [conf.minthrottle;MAXTHROTTLE]
  }
#endif // USE_THROTTLE_CURVE 

} // readEEPROM

void writeGlobalSet(uint8_t b) {
  global_conf.checksum = calculate_sum((uint8_t*)&global_conf, sizeof(global_conf));
  eeprom_write_block((const void*)&global_conf, (void*)0, sizeof(global_conf));

  if (b == 1) blinkLED(50,1);
} // writeGlobalSet

void writeParams(uint8_t b) {

  conf.thisVersion = VERSION;
  conf.checksum = calculate_sum((uint8_t*)&conf, sizeof(conf));
  eeprom_write_block((const void*)&conf, (void*)(sizeof(global_conf)), sizeof(conf));
  readEEPROM();

  if (b == 1) blinkLED(50,1);
} // writeParams

void LoadDefaults(void) {

  memset(&conf, 0, sizeof(conf));

#if defined(USE_MW)

  conf.P8[ROLL] = 40;  
  conf.I8[ROLL] = 30; 
  conf.D8[ROLL] = 23;

  conf.P8[PITCH] = 40; 
  conf.I8[PITCH] = 30; 
  conf.D8[PITCH] = 23;
  
  conf.P8[YAW] = 80;  
  conf.I8[YAW] = 25;  

  conf.P8[PIDLEVEL] = 90; 
  conf.I8[PIDLEVEL] = 1; 
  conf.D8[PIDLEVEL] = 100;

  conf.P8[PIDMAG]   = 40;

  conf.rcRate8 = 80; 
  conf.rcExpo8 = 50;
  conf.rollPitchRate = 60;
  conf.yawRate = 100;

  conf.dynThrPID = 0;

#else

  conf.P8[ROLL] = 35;   
  conf.I8[ROLL] = 60; 
  conf.D8[ROLL] = 25;
  conf.P8[PITCH] = 35;  
  conf.I8[PITCH] = 60; 
  conf.D8[PITCH] = 25;
  conf.P8[YAW] = 80;  
  conf.I8[YAW] = 25; 

  conf.P8[PIDLEVEL] = 90; 
  conf.I8[PIDLEVEL] = 1; 
  conf.D8[PIDLEVEL] = 100;

  conf.P8[PIDMAG] = 40; 

  conf.rcRate8 = 100; 
  conf.yawRate = 100;

#endif // USE_MW_XXX

#ifdef USE_THROTTLE_CURVE
  conf.thrMid8 = 45; 
  conf.thrExpo8 = 40;
#else  
  conf.thrMid8 = 50; 
  conf.thrExpo8 = 0; 
#endif // USE_THROTTLE_CURVE

  conf.P8[PIDALT] = 64; 
  conf.I8[PIDALT] = 25; 
  conf.D8[PIDALT] = 24;

  for(uint8_t i = 0; i < CHECKBOX_ITEMS; i++)
    conf.activate[i] = 0;

  f.ANGLE_MODE = true; 
  conf.activate[BOX_ANGLE] = 7; // Aux1 all positions

  conf.cycletimeuS = CYCLETIME;
  conf.minthrottle = MINTHROTTLE;

  writeParams(0); // this will also (p)reset this Version with the current version number again.

  calibratingG = 512;

} // loadDefaults












