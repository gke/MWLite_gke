
// MultiWii GUI Protocol

#if defined(AIRPLANE)
const uint8_t pwmMap[8] = {
  7,0,1,2,1,3,4,0};
#else
const uint8_t pwmMap[8] = {
  0,2,1,2,1,3,4,0};
#endif

static uint8_t csum;
static uint8_t cmdIndex;
static uint8_t cmdMSP;

#if defined(SPEKTRUM)
#define BIND_CAPABLE 1
#else
#define BIND_CAPABLE 0
#endif
const uint32_t capability = 0+BIND_CAPABLE;

void telemetryHeadResponse(uint8_t err, uint8_t s) {
  tx8('$');
  tx8('M');
  tx8(err ? '!' : '>');
  csum = 0; // start calculating a new checksum
  tx8(s);
  tx8(cmdMSP);
} // telemetryHeadResponse

void telemetryHeadReply(uint8_t s) {
  telemetryHeadResponse(0, s);
} // telemetryHeadReply

void inline telemetryHeadError(uint8_t s) {
  telemetryHeadResponse(1, s);
} // telemetryHeadError

void telemetryTailReply() {
  tx8(csum);
  txUARTData(0);
} // telemetryTailReply

void telemetryWriteNames(PGM_P s) {
  telemetryHeadReply(strlen_P(s));
  for (PGM_P c = s; pgm_read_byte(c); c++) 
    tx8(pgm_read_byte(c));
} // telemetryWriteNames

enum Sensors {
  SENS_ACC, SENS_BARO, SENS_MAG, SENS_GPS, SENS_SONAR, SENS_OPTIC};

/************************************** MultiWii Serial Protocol *******************************************************/
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
#define MSP_SERVO_CONF           120   //out message         Servo settings

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
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4


#define MW_2_3_PROTOCOL

uint8_t rcSerialCount;

void  telemetryWrite(uint8_t *cb, uint8_t l) {
  telemetryHeadReply(l);
  while(l--) 
    tx8(*cb++);
} // telemetryWrite

void telemetryRead(uint8_t *cb, uint8_t l) {
  while(l--) 
    *cb++ = rx8();
  telemetryHeadReply(0);
} // telemetryRead

void evaluateCommand(void) { 
  static uint32_t homeUpdatemS = millis() + 1000;
  uint8_t len, i, w;
  int16_t Temp;

  cmdIndex = 0;

  switch(cmdMSP) {
  case MSP_SET_RAW_RC:
    for(i = 0; i < 8; i++) 
      rcData[i] = rx16();
    telemetryHeadReply(0);
    break;
  case MSP_SET_PID:
    for(i = 0; i < PID_ITEMS; i++) {
      conf.pid[i].Kp = rx8();
      conf.pid[i].Ki = rx8();
      conf.pid[i].Kd = rx8();
    }
    telemetryHeadReply(0);
    break;
  case MSP_SET_BOX:
    for(i = 0; i < CHECKBOX_ITEMS; i++)
      conf.activate[i]=rx16();
    telemetryHeadReply(0);
    break;
  case MSP_SET_RC_TUNING:
    telemetryRead((uint8_t*)&conf.rates,7);
    break;
  case MSP_SET_MISC:
#if defined(MW_2_3_PROTOCOL)
    struct {
      uint16_t a,b,c,d,e,f;
      uint32_t g;
      uint16_t h;
      uint8_t  i,j,k,l;
    } 
    set_misc;
    telemetryRead((uint8_t*)&set_misc, 22);
#if defined(USE_POWERMETER)
    conf.powerTrigger1 = set_misc.a / PLEVELSCALE;
#endif
    conf.min_throttleuS = set_misc.b;
#if defined(FAILSAFE)
    conf.failsafe_throttleuS = set_misc.e;
#endif  
#if defined(USE_MAG)
    conf.mag_declination = set_misc.h;
#endif
#if (LVC_LIMIT > 0)
#if defined(MW_2_3_PROTOCOL)
    conf.vbat.scale = set_misc.i;
    conf.vbat.warn1 = set_misc.j;
    conf.vbat.warn2 = set_misc.k;
    conf.vbat.critical = set_misc.l;
#endif
#endif
#else
    telemetryHeadReply(0);
#endif
    break;
  case MSP_IDENT:
    struct {
      uint8_t v, t, msp_v;
      uint32_t cap;
    } 
    id;
    id.v = VERSION;
    id.t = MULTITYPE;
    id.msp_v = MSP_VERSION;
    id.cap = capability;
    telemetryWrite((uint8_t*)&id, 7);
    break;   
  case MSP_STATUS:
    struct {
      uint16_t cycleTimeuS, i2cErrors, sensor;
      uint32_t flag;
      uint8_t set;
    } 
    st;
    st.cycleTimeuS = cycleTimeuS;
    st.i2cErrors = i2cErrors;
    st.sensor = f.ACC_CALIBRATED << SENS_ACC | 
      f.BARO_ACTIVE << SENS_BARO | 
      (f.MAG_ACTIVE && f.MAG_CALIBRATED) << SENS_MAG | 
      f.GPS_ACTIVE << SENS_GPS | 
      f.SONAR_ACTIVE << SENS_SONAR | 
      f.OPTIC_ACTIVE << SENS_OPTIC; 

    st.flag = 
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
      | f.EXP << BOX_EXP;

    st.set = 0; // zzz global_conf.currentSet;
    telemetryWrite((uint8_t*)&st, 11);
    break;
  case MSP_RAW_IMU:
    telemetryHeadReply(18);
    // for(i = 0; i < 3; i++) tx16((accData[i] * (GRAVITY_R * 180.0)/PI));
    // for(i = 0; i < 3; i++) tx16(asin(constrain(accData[i] * GRAVITY_R, -1.0, 1.0)) * (180.0/PI));
    for(i = 0; i < 3; i++) 
      tx16(((int32_t)accData[i] * 100) >> 12);
    for(i = 0; i < 3; i++) 
      tx16(gyroData[i]);
    for(i = 0; i < 3; i++) 
      tx16(magADC[i]);
    break;
  case MSP_SERVO:
    telemetryHeadReply(16);
    for(i = 0; i < 8; i++) {
      Temp = pwm[pwmMap[i]];
      if (pwmMap[i] == 0) // throttle
        tx16(Temp);
      else
        tx16(Temp + MID_RC_US); 
    }
    break;
  case MSP_MOTOR:
    telemetryHeadReply(16);
    for(i = 0; i < 8; i++)
      tx16( (i < PWM_OUTPUTS) ? pwm[i] : 0 );
    break;
  case MSP_RC:
    telemetryHeadReply(RC_CHANS * 2);
    for(i = 0; i < RC_CHANS; i++) tx16(rcData[i]);
    break;
  case MSP_ATTITUDE:
    telemetryHeadReply(10);
    tx16(angle[ROLL]);
    tx16(angle[PITCH]);
    tx16(angle[YAW] / 10);
    tx16(0);
    tx16(0);
    break;
  case MSP_ALTITUDE:
    telemetryHeadReply(6);
    tx32(relativeAltitude);
    tx16(ROC);
    break;
  case MSP_ANALOG:
#if defined(MW_2_3_PROTOCOL)
    telemetryWrite((uint8_t*)&analog, 7);
#else
    telemetryHeadReply(5);
    tx8(analog.vbat);
    tx16(analog.intPowerMeterSum);
    tx16(analog.rssi);
#endif
    break;
  case MSP_RC_TUNING:
    telemetryWrite((uint8_t*)&conf.rates, 7);
    break;
  case MSP_PID:
    telemetryWrite((uint8_t*)&conf.pid, 3 * PID_ITEMS);
    break;
  case MSP_PIDNAMES:
    telemetryWriteNames(pidnames);
    break;
  case MSP_BOX:
    telemetryWrite((uint8_t*)&conf.activate[0], 2 * CHECKBOX_ITEMS);
    break;
  case MSP_BOXNAMES:
    telemetryWriteNames(boxnames);
    break; 
  case MSP_BOXIDS:
    telemetryHeadReply(CHECKBOX_ITEMS);
    for(i = 0; i < CHECKBOX_ITEMS; i++)
      tx8(pgm_read_byte(&(boxids[i])));
    break; 
  case MSP_MOTOR_PINS:
    telemetryHeadReply(8);
    for(i = 0; i < 8; i++)
      tx8(PWM_PIN[i]);
    break;
  case MSP_RESET_CONF:
    if(!f.ARMED) 
      LoadDefaults();
    telemetryHeadReply(0);
    break;
  case MSP_ACC_CALIBRATION:
    if(!f.ARMED) 
      calibratingA = 512;
    telemetryHeadReply(0);
    break;
  case MSP_MAG_CALIBRATION:
    if(!f.ARMED) f.CALIBRATE_MAG = true;
    telemetryHeadReply(0);
    break;

#if defined(USE_GPS)

#if defined(MW_2_3_PROTOCOL)
  case MSP_RAW_GPS:
    telemetryHeadReply(16);
    tx8(f.GPS_FIX);
    tx8(gpsSats);
    tx32(gpsPosition[LAT]);
    tx32(gpsPosition[LON]);
    tx16(gpsAltitude / 100);
    tx16(gpsSpeed);
    tx16(gpsGroundCourse);   
    break;
  case MSP_COMP_GPS:
    /* 
     telemetryHeadReply(5);
     tx16(homeDistance);
     tx16(homeHeading);
     tx8(0);//zzzgpsupdate & 1);
     */
    if (millis() > homeUpdatemS) {    
      homeUpdatemS = millis() + 1000;
      gpsRelPosition(); 
    }
    telemetryHeadReply(5);   
    tx16(homeDistance);
    tx16(homeHeading / 10);
    tx8(f.GPS_FIX_HOME);
    break;

#if defined(USE_MSP_WP)

  case MSP_WP:
    w = constrain(rx8(), 0, MAX_WAYPOINTS); // wp number  
    telemetryHeadReply(18);
    tx8(w);
    tx32(wp[w].lat);
    tx32(wp[w].lon); 
    tx16(wp[w].alt); // cm
    tx32(wp[w].heading);  // deg*10
    tx16(wp[w].loitermS); // mS
    tx8(wp[w].flag);
    break;
  case MSP_SET_WP:    
    w = rx8(); // wp number
    if ((w>0) && (w < MAX_WAYPOINTS)) {
      wp[w].lat = rx32();
      wp[w].lon = rx32();
      wp[w].alt = rx32(); // cm
      wp[w].heading = rx16(); // deg*10
      wp[w].loitermS = rx16(); // mS
      wp[w].flag = rx8();
    }
    telemetryHeadReply(0);
    break;

#endif

#if defined(USE_GPS_SIM)
  case MSP_SET_HEAD:
    telemetryRead((uint8_t*)&holdHeading, 2);
    break;
  case MSP_SET_RAW_GPS:
    f.GPS_FIX = rx8();
    gpsSats = rx8();
    gpsPosition[LAT] = rx32();
    gpsPosition[LON] = rx32();
    gpsAltitude = rx16();
    gpsSpeed = rx16();
    //   gpsUpdate |= 2;              // New data signalisation to GPS functions
    telemetryHeadReply(0);
    break;
#endif

#else

  case MSP_RAW_GPS:
    telemetryHeadReply(14);
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
    telemetryHeadReply(5);   
    tx16(homeDistance);
    tx16(homeHeading / 10);
    tx8(f.GPS_FIX_HOME);
    break;

#endif // MW_2_3_PROTOCOL

#endif // USE_GPS

#if defined(SPEKTRUM)
  case MSP_BIND:
    doSpektrumBinding();  
    telemetryHeadReply(0);
    break;
#endif

#if defined(MW_2_3_PROTOCOL)
  case MSP_MISC:
    struct {
      uint16_t a,b,c,d,e,f;
      uint32_t g;
      uint16_t h;
      uint8_t  i,j,k,l;
    } 
    misc;

    misc.a = 0;//zzzintPowerTrigger1;
    misc.b = conf.min_throttleuS;
    misc.c = MAX_THR_US;
    misc.d = MIN_RC_US;
#if defined(FAILSAFE) 
    misc.e = conf.failsafe_throttleuS;
#else  
    misc.e = 0;
#endif
    misc.f = 0; 
    misc.g =0;
    //#if defined(USE_MAG)
    misc.h = conf.mag_declination;
    //#else
    //    misc.h = 0;
    //#endif
#if (LVC_LIMIT > 0)
    misc.i = conf.vbat.scale;
    misc.j = conf.vbat.warn1;
    misc.k = conf.vbat.warn2;
    misc.l = conf.vbat.critical;
#else
    misc.i = 0;
    misc.j = 0;
    misc.k = 0;
    misc.l = 0;
#endif
    telemetryWrite((uint8_t*)&misc,22);   
    break;
    /*
  case MSP_SERVO_CONF:
     telemetryWrite((uint8_t*)&conf.servoConf[0].min, 56); // struct servo_conf_ is 7 bytes length: min:2 / max:2 / middle:2 / rate:1    ----     8 servo =>  8x7 = 56
     break;
     case MSP_SET_SERVO_CONF:
     telemetryRead((uint8_t*)&conf.servoConf[0].min, 56);
     break; 
     */
#endif

  case MSP_EEPROM_WRITE:
    writeParams(0);
    telemetryHeadReply(0);
    break;
  case MSP_DEBUG:
    telemetryWrite((uint8_t*)&debug, 8);
    break;
  default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
    telemetryHeadError(0);
    break;
  }
  telemetryTailReply();
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
































