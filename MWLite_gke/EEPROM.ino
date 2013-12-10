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

void readGlobalSet(void) {
  eeprom_read_block((void*)&global_conf, (void*)0, sizeof(global_conf));

  f.ACC_CALIBRATED = global_conf.accCalibrated;
  f.MAG_CALIBRATED = global_conf.magCalibrated;
} // readGlobalSet

void readEEPROM(void) {
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

  // 500/128 = 3.90625    3.9062 * 3.9062 = 15.259   1526*100/128 = 1192
  for(i = 0; i < 5; i++) 
    rollpitchCurve[i] = (1526+conf.rcExpo8*(sq(i)-15))*i*(int32_t)conf.rcRate8/1192;

  for(i = 0; i < 11; i++) {
    int16_t tmp = 10*i-conf.thrMid8;
    uint8_t y = 1;
    if (tmp>0) y = 100-conf.thrMid8;
    if (tmp<0) y = conf.thrMid8;
    thrCurve[i] = 10*conf.thrMid8 + tmp*( 100-conf.thrExpo8+(int32_t)conf.thrExpo8*sq(tmp)/sq(y) )/10; // [0;1000]
    thrCurve[i] = conf.minthrottle + (int32_t)(MAX_THROTTLE-conf.minthrottle)* thrCurve[i]/1000;  // [0;1000] -> [conf.minthrottle;MAXTHROTTLE]
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
  uint8_t i;

  memset(&conf, 0, sizeof(conf));

#include "EEPROMParams.h"

  for(i = 0; i < CHECKBOX_ITEMS; i++)
    conf.activate[i] = 0;

#if defined(ISMULTICOPTER)  
  f.ANGLE_MODE = true; 
  conf.activate[BOX_ANGLE] = 3; // Aux1 all positions
#else
  f.ANGLE_MODE = true; 
  conf.activate[BOX_ANGLE] = 7; // Aux1 all positions
  f.BYPASS_MODE = true;
  conf.activate[BOX_BYPASS] = 6; // Aux1 
#endif

  conf.cycletimeuS = CYCLETIME;

  conf.minthrottle = MIN_THROTTLE;

  writeParams(0); // this will also (p)reset this Version with the current version number again.

  calibratingG = 512;

} // loadDefaults
















