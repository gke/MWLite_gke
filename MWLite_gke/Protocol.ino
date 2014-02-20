// MultiWii GUI Protocol

#if defined(AIRPLANE)
const uint8_t pwmMap[8] = {
  7,0,1,2,1,3,4,0};
#else
const uint8_t pwmMap[8] = {
  0,2,1,2,1,3,4,0};
#endif


// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4


static uint8_t csum;
static uint8_t cmdIndex;
static uint8_t cmdMSP;

void headSerialResponse(uint8_t err, uint8_t s) {
  tx8('$');
  tx8('M');
  tx8(err ? '!' : '>');
  csum = 0; // start calculating a new checksum
  tx8(s);
  tx8(cmdMSP);
}

void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

void inline headSerialError(uint8_t s) {
  headSerialResponse(1, s);
}

void tailSerialReply() {
  tx8(csum);
  uartSendData(0);
}

void txNames(PGM_P s) {
  for (PGM_P c = s; pgm_read_byte(c); c++) 
    tx8(pgm_read_byte(c));
} // txNames

enum Sensors {
  SENS_ACC, SENS_BARO, SENS_MAG, SENS_GPS, SENS_SONAR, SENS_OPTIC};

void evaluateCommand(void) { 
  static uint32_t homeUpdatemS = millis() + 1000;
  uint8_t len, i;
  int16_t Temp;

  cmdIndex = 0;

  switch(cmdMSP) {
  case MSP_SET_RAW_RC:
    for(i = 0; i < 8; i++) 
      rcData[i] = rx16();
    headSerialReply(0);
    break;
  case MSP_SET_PID:
    for(i = 0; i < PIDITEMS; i++) {
      conf.P8[i]=rx8();
      conf.I8[i]=rx8();
      conf.D8[i]=rx8();
    }
    headSerialReply(0);
    break;
  case MSP_SET_BOX:
    for(i = 0; i < CHECKBOX_ITEMS; i++)
      conf.activate[i]=rx16();
    headSerialReply(0);
    break;
  case MSP_SET_RC_TUNING:
    conf.rcRate8 = rx8();
    conf.rcExpo8 = rx8();
    conf.rollPitchRate = rx8();
    conf.yawRate = rx8();
    conf.dynThrPID = rx8();
    conf.thrMid8 = rx8();
    conf.thrExpo8 = rx8();
    headSerialReply(0);
    break;
  case MSP_SET_MISC:
    headSerialReply(0);
    break;
  case MSP_IDENT:
    headSerialReply(7);
    tx8(VERSION);  
    tx8(MULTITYPE); 
    tx8(MSP_VERSION); 
    tx32(0);//pgm_read_dword(&(capability))); // "capability"
    break;
  case MSP_STATUS:
    headSerialReply(11);
    tx16(cycleTimeuS);
    tx16(i2cErrors);

    tx16(f.ACC_CALIBRATED << SENS_ACC | 
      f.BARO_ACTIVE << SENS_BARO | 
      (f.MAG_ACTIVE && f.MAG_CALIBRATED) << SENS_MAG | 
      f.GPS_ACTIVE << SENS_GPS | 
      f.SONAR_ACTIVE << SENS_SONAR | 
      f.OPTIC_ACTIVE << SENS_OPTIC ); 

    tx32( 
    f.ARMED << BOX_ARM 
      |  f.ANGLE_MODE << BOX_ANGLE
#if defined(USE_MW)
      | f.HORIZON_MODE << BOX_HORIZON
#endif
#if defined(MULTICOPTER)
      | f.HEAD_FREE_MODE << BOX_HEAD_FREE
#endif
#if defined(USE_MAG)
      | f.HEAD_HOLD_MODE << BOX_HEAD_HOLD
#endif
#if !defined(MULTICOPTER)
      | f.BYPASS_MODE << BOX_BYPASS
#endif
#if defined(USE_ALT)
      | f.ALT_HOLD_MODE << BOX_ALT_HOLD
#endif
#if defined(USE_GPS)
      | f.GPS_HOME_MODE << BOX_GPS_HOME
      | f.GPS_HOLD_MODE << BOX_GPS_HOLD
#endif
#if defined(USE_TUNING)
      | f.RELAY_MODE << BOX_RELAY
#endif
      | f.EXP << BOX_EXP
      );
    tx8(0); // current parameter set
    break;
  case MSP_RAW_IMU:
    headSerialReply(18);
    // for(i = 0; i < 3; i++) tx16((accData[i] * (GRAVITY_R * 180.0)/PI));
    // for(i = 0; i < 3; i++) tx16(asin(constrain(accData[i] * GRAVITY_R, -1.0, 1.0)) * (180.0/PI));
    for(i = 0; i < 3; i++) tx16(((int32_t)accData[i] * 100) >> 12);
    for(i = 0; i < 3; i++) tx16(gyroData[i]);
    for(i = 0; i < 3; i++) tx16(magADC[i]);
    break;
  case MSP_SERVO:
    headSerialReply(16);
    for(i = 0; i < 8; i++) {
      Temp = pwm[pwmMap[i]];
      if (pwmMap[i] != 0)
        Temp += MID_RC_US; 
      tx16(Temp);
    }
    break;
  case MSP_MOTOR:
    headSerialReply(16);
    for(i = 0; i < 8; i++)
      tx16( (i < PWM_OUTPUTS) ? pwm[i] : 0 );
    break;
  case MSP_RC:
    headSerialReply(RC_CHANS * 2);
    for(i = 0; i < RC_CHANS; i++) tx16(rcData[i]);
    break;
  case MSP_ATTITUDE:
    headSerialReply(10);
    tx16(angle[ROLL]);
    tx16(angle[PITCH]);
    tx16(angle[YAW] / 10);
    tx16(0);
    tx16(0);
    break;
  case MSP_ALTITUDE:
    headSerialReply(6);
    tx32(relativeAltitude);
    tx16(ROC);
    break;
  case MSP_ANALOG:
    headSerialReply(4);
    tx8(analog.vbat);
    tx16(analog.intPowerMeterSum);
    tx8(analog.rssi);
    break;
  case MSP_RC_TUNING:
    headSerialReply(7);
    tx8(conf.rcRate8);
    tx8(conf.rcExpo8);
    tx8(conf.rollPitchRate);
    tx8(conf.yawRate);
    tx8(conf.dynThrPID);
    tx8(conf.thrMid8);
    tx8(conf.thrExpo8);
    break;
  case MSP_PID:
    headSerialReply(3*PIDITEMS);
    for(i = 0; i < PIDITEMS; i++) {
      tx8(conf.P8[i]);
      tx8(conf.I8[i]);
      tx8(conf.D8[i]);
    }
    break;
  case MSP_PIDNAMES:
    headSerialReply(strlen_P(pidnames));
    txNames(pidnames);
    break;
  case MSP_BOX:
    headSerialReply(2*CHECKBOX_ITEMS);
    for(i = 0; i < CHECKBOX_ITEMS; i++)
      tx16(conf.activate[i]);
    break;
  case MSP_BOXNAMES:
    headSerialReply(strlen_P(boxnames));
    txNames(boxnames);
    break;
  case MSP_BOXIDS:
    headSerialReply(CHECKBOX_ITEMS);
    for(i = 0; i < CHECKBOX_ITEMS; i++)
      tx8(pgm_read_byte(&(boxids[i])));
    break;
  case MSP_MOTOR_PINS:
    headSerialReply(8);
    for(i = 0; i < 8; i++)
      tx8(PWM_PIN[i]);
    break;
  case MSP_RESET_CONF:
    if(!f.ARMED) LoadDefaults();
    headSerialReply(0);
    break;
  case MSP_ACC_CALIBRATION:
    if(!f.ARMED) 
      calibratingA = 512;
    headSerialReply(0);
    break;
  case MSP_MAG_CALIBRATION:
    if(!f.ARMED) f.CALIBRATE_MAG = true;
    headSerialReply(0);
    break;
#if defined(USE_GPS)
  case MSP_RAW_GPS:
    headSerialReply(14);
    tx8(f.GPS_FIX);
    tx8(gpsSats);
    tx32(gpsPosition[LAT]); // deg x 10^7
    tx32(gpsPosition[LON]);
    tx16(gpsAltitude / 100);
    tx16(gpsSpeed); // cm/S
    break;
  case MSP_COMP_GPS: 
    if (millis() > homeUpdatemS) {    
      homeUpdatemS = millis() + 1000;
      gpsRelPosition(); 
    }
    headSerialReply(5);   
    tx16(homeDistance);
    tx16(homeHeading / 10);
    tx8(f.GPS_FIX_HOME);
    break;
#endif
  case MSP_EEPROM_WRITE:
    writeParams(0);
    headSerialReply(0);
    break;
  case MSP_DEBUG:
    headSerialReply(8);
    for(i = 0; i < 4; i++)
      tx16(debug[i]); // 4 variables are here for general monitoring purpose
    break;
  default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
    headSerialError(0);
    break;
  }
  tailSerialReply();
} // evaluatCommand

void serialCom(void) {
  uint8_t c,n;  
  static uint8_t len;
  static uint8_t txBuffBytes;
  static enum _serial_state {
    waitSentinel,
    waitHeader1,
    waitHeader2,
    waitSize,
    waitCmd,
    waitPayload,
  } 
  MSPState;// = waitSentinel;

  while (serialAvailable(0)) {
    txBuffBytes = ((uint8_t)(txHead[0]-txTail[0]))%TX_BUFFER_SIZE; // indicates the number of occupied bytes in TX buffer
    if (txBuffBytes < (TX_BUFFER_SIZE - 50 )) { // ensure there is enough free TX buffer to go further (50 bytes margin)
      c = serialRead(0);
      csum ^= c;
      switch (MSPState) {
      case waitSentinel:
        MSPState = (c=='$') ? waitHeader1 : waitSentinel;
        break;
      case waitHeader1:
        MSPState = (c=='M') ? waitHeader2 : waitSentinel;
        break;
      case waitHeader2:
        MSPState = (c=='<') ? waitSize : waitSentinel; 
        break;
      case waitSize:
        if (c > INBUF_SIZE)
          MSPState = waitSentinel;
        else {
          len = csum = c;
          cmdIndex = 0;         
          MSPState = waitCmd;  // the command is to follow
        }
        break;
      case waitCmd:
        cmdMSP = c;
        MSPState = waitPayload;
        break;
      case waitPayload:
        if (cmdIndex < len) 
          cmdBuf[cmdIndex++] = c;
        else if (MSPState == waitPayload && cmdIndex >= len) {
          if (csum == 0) 
            evaluateCommand();  
          MSPState = waitSentinel;
        }
        break;
      } // switch
    } 
  }
} // serialCom

