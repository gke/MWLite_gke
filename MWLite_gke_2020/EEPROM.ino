
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

void readWPSet(void) {
  eeprom_read_block((void*)&wp, (void*)(sizeof(global_conf)+sizeof(conf)), sizeof(wp));
} // readWPSet

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

#if defined(USE_MW)
  for(i = 0; i < 5; i++) 
    rollpitchCurve[i] = (1526+conf.rates.rcExpo8*(sq(i)-15))*i*(int32_t)conf.rates.rcRate8/1192;
#endif

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

void writeWPSet(uint8_t b) {
  global_conf.checksum = calculate_sum((uint8_t*)&wp, sizeof(wp));
  eeprom_write_block((const void*)&wp, (void*)(sizeof(global_conf)+sizeof(conf)), sizeof(wp));

  if (b == 1) blinkLED(50,1);
} // writeWPSet

void LoadDefaults(void) {
  uint8_t i;

  memset(&conf, 0, sizeof(conf));

#include "EEPROMParams.h"

  for(i = 0; i < CHECKBOX_ITEMS; i++)
    conf.activate[i] = 0;

//A1=SG, A2=SC, A3=S1, A4=SF, A5=S2

#if !defined(MULTICOPTER) 
  f.BYPASS_MODE = true;
  conf.activate[BOX_BYPASS] = (4+0+0)<<9; // Aux4
#endif

  f.ANGLE_MODE = true; 
  conf.activate[BOX_ANGLE] = (4+2+0)<<3; // Aux2 all positions

#if defined(USE_ALT)
  f.ALT_HOLD_MODE = false;
  conf.activate[BOX_ALT_HOLD] = (4+2+0)<<6; // Aux3   
#endif

#if defined(USE_GPS)
  f.GPS_HOLD_MODE = false;
  conf.activate[BOX_GPS_HOLD] = (0+2+0)<<0; // Aux1 
  f.GPS_HOME_MODE = false;
  conf.activate[BOX_GPS_HOME] = (4+0+0)<<0; // Aux1 
#else
#if defined(MULTICOPTER)
  f.HEAD_FREE_MODE = false;
  conf.activate[BOX_HEAD_FREE] = (4+2+0)<<0; // Aux1
#endif 
#endif 

  conf.cycletimeuS = CYCLE_US;
  conf.min_throttleuS = MIN_EFF_THR_US;
  conf.failsafe_throttleuS = MIN_EFF_THR_US;
  conf.mag_declination = 125;
  
  conf.vbat.scale = 1;
  conf.vbat.warn1 = LVC_LIMIT;
  conf.vbat.warn2 = LVC_LIMIT;
  conf.vbat.critical = LVC_LIMIT;

  writeParams(0); // this will also (p)reset this Version with the current version number again.

  calibratingG = 512;

} // loadDefaults
