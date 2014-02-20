
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
    
#if defined(USE_MW)
    for(i = 0; i < 5; i++) 
      rollpitchCurve[i] = (1526+conf.rcExpo8*(sq(i)-15))*i*(int32_t)conf.rcRate8/1192;
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

void LoadDefaults(void) {
  uint8_t i;

  memset(&conf, 0, sizeof(conf));

#include "EEPROMParams.h"

  for(i = 0; i < CHECKBOX_ITEMS; i++)
    conf.activate[i] = 0;

#if defined(MULTICOPTER)  
  f.ANGLE_MODE = true; 
  conf.activate[BOX_ANGLE] = 3; // Aux1 all positions
#else
  f.ANGLE_MODE = true; 
  conf.activate[BOX_ANGLE] = 7; // Aux1 all positions
  f.BYPASS_MODE = true;
  conf.activate[BOX_BYPASS] = 6; // Aux1 
#endif

  conf.cycletimeuS = CYCLE_US;
  conf.minthrottleuS = MIN_EFF_THR_US;

  writeParams(0); // this will also (p)reset this Version with the current version number again.

  calibratingG = 512;

} // loadDefaults


















