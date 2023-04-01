#if defined(USE_GPS) && !defined(SPEKTRUM)

struct ubx_header {
  uint8_t preamble1;
  uint8_t preamble2;
  uint8_t msg_class;
  uint8_t msg_id;
  uint16_t length;
};

struct ubx_cfg_nav_rate {
  uint16_t measure_rate_ms;
  uint16_t nav_rate;
  uint16_t timeref;
};

struct ubx_cfg_msg_rate {
  uint8_t msg_class;
  uint8_t msg_id;
  uint8_t rate;
};


struct ubx_cfg_nav_settings {
  uint16_t mask;
  uint8_t  dynModel;
  uint8_t  fixMode;
  int32_t  fixedAlt;
  uint32_t fixedAltVar;
  int8_t   minElev;
  uint8_t  drLimit;
  uint16_t pDop;
  uint16_t tDop;
  uint16_t pAcc;
  uint16_t tAcc;
  uint8_t  staticHoldThresh;
  uint8_t  res1;
  uint32_t res2;
  uint32_t res3;
  uint32_t res4;
};

struct ubx_cfg_sbas_settings {
  uint8_t mode;
  uint8_t usage;
  uint8_t maxSBAS;
  uint8_t scanmode2;
  uint32_t scanmode1;
};

struct ubx_nav_posllh {
  uint32_t time;  // mS
  int32_t longitude; // deg*1e-7
  int32_t latitude;
  int32_t altitude_ellipsoid; // mm
  int32_t altitude_msl; 
  uint32_t horizontal_accuracy; // mm
  uint32_t vertical_accuracy;
};

struct ubx_nav_solution {
  uint32_t time; 
  int32_t time_nsec;
  int16_t week;
  uint8_t fix_type;
  uint8_t fix_status;
  int32_t ecef_x; // cm
  int32_t ecef_y;
  int32_t ecef_z;
  uint32_t position_accuracy_3d; // cm
  int32_t ecef_x_velocity; // cm/S
  int32_t ecef_y_velocity;
  int32_t ecef_z_velocity;
  uint32_t speed_accuracy; // cm/S
  uint16_t position_DOP;
  uint8_t res;
  uint8_t satellites;
  uint32_t res2;
};

struct ubx_nav_velned {
  uint32_t time;  // GPS Millisecond Time of Week (ms)
  int32_t ned_north; // cm/S
  int32_t ned_east; // cm/S
  int32_t ned_down; // cm/S
  uint32_t speed_3d; // cm/S
  uint32_t speed_2d; // cm/S
  int32_t heading_2d; // deg*1e-5
  uint32_t speed_accuracy; // cm/S
  uint32_t heading_accuracy; // deg*1e-5
};

enum ubx_protocol_bytes {
  PREAMBLE1 = 0xb5,
  PREAMBLE2 = 0x62,
  CLASS_NAV = 0x01,
  CLASS_ACK = 0x05,
  CLASS_CFG = 0x06,
  MSG_ACK_NACK = 0x00,
  MSG_ACK_ACK = 0x01,
  MSG_POSLLH = 0x2,
  MSG_STATUS = 0x3,
  MSG_SOL = 0x6,
  MSG_VELNED = 0x12,
  MSG_CFG_PRT = 0x00,
  MSG_CFG_RATE = 0x08,
  MSG_CFG_SET_RATE = 0x01,
  MSG_CFG_NAV_SETTINGS = 0x24,
  MSG_CFG_SBAS_SETTINGS = 0x16 
};

enum ubx_nav_fix_type {
  FIX_NONE = 0,
  FIX_DEAD_RECKONING = 1,
  FIX_2D = 2,
  FIX_3D = 3,
  FIX_GPS_DEAD_RECKONING = 4,
  FIX_TIME = 5
};

enum ubx_nav_status_bits {
  NAV_STATUS_FIX_VALID = 1
};

enum ublox_engine_setting {
  GPS_ENGINE_NONE        = -1,
  GPS_ENGINE_PEDESTRIAN  = 3,
  GPS_ENGINE_AUTOMOTIVE  = 4,
  GPS_ENGINE_SEA         = 5,
  GPS_ENGINE_AIRBORNE_1G = 6,
  GPS_ENGINE_AIRBORNE_2G = 7,
  GPS_ENGINE_AIRBORNE_4G = 8
};

#define SBAS_DISABLED            0x00
#define SBAS_WAAS                0x01        // Americas
#define SBAS_EGNOS               0x02        // Europe and Africa
#define SBAS_MSAS                0x03        // ASIA north of the Equator (not Oz)

#define SBAS_MODE_DISABLE        0x00
#define SBAS_MODE_ENABLE         0x01        //Enabled 
#define SBAS_MODE_ENABLE_TEST    0x03        //some EGNOS sats is still in Test mode 

#define SBAS_USAGE               0x07        //integrity, differential correction and ranging
#define SBAS_SATS_WAAS           0x0004e004  //Satellite mask for WAAS
#define SBAS_SATS_EGNOS          0x00000851  //Satellite mask for EGNOS
#define SBAS_SATS_MSAS           0x00020200  //Satellite mssk for MSAS

static ublox_engine_setting _nav_setting = GPS_ENGINE_PEDESTRIAN; // EOSBandi uses AUTOMOTIVE

static uint8_t ck_a, ck_b;
static uint8_t ubx_state;
static uint8_t msg_id;
static uint8_t ubx_len;
static uint8_t ubx_cc;
static uint8_t ubx_class;

static union {
  ubx_nav_posllh posllh;
  //ubx_nav_status status;
  ubx_nav_solution solution;
  ubx_nav_velned velned;
  uint8_t bytes[];
} 
ubx_buff;

void ubloxCalculateCheckSum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b) {
  while (len--) {
    ck_a += *data;
    ck_b += ck_a;
    data++;
  }
} // ubloxCalculateCheckSum

void ubloxSendMessage(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size) {
  struct ubx_header header;
  uint8_t ck_a=0, ck_b=0;

  header.preamble1 = PREAMBLE1;
  header.preamble2 = PREAMBLE2;
  header.msg_class = msg_class;
  header.msg_id    = msg_id;
  header.length    = size;

  ubloxCalculateCheckSum((uint8_t *)&header.msg_class, sizeof(header)-2, ck_a, ck_b);
  ubloxCalculateCheckSum((uint8_t *)msg, size, ck_a, ck_b);

  ubloxWriteBuffer(GPS_SERIAL_PORT,(uint8_t *)&header, sizeof(header));
  ubloxWriteBuffer(GPS_SERIAL_PORT,(uint8_t *)msg, size);
  ubloxWriteBuffer(GPS_SERIAL_PORT,(uint8_t *)&ck_a, 1);
  ubloxWriteBuffer(GPS_SERIAL_PORT,(uint8_t *)&ck_b, 1);

  delay(100);
} // ubloxSendMessage

void ubloxSetMessageRate(uint8_t msg_class, uint8_t msg_id, uint8_t rate) {
  struct ubx_cfg_msg_rate msg;
  msg.msg_class = msg_class;
  msg.msg_id = msg_id;
  msg.rate = rate;
  ubloxSendMessage(CLASS_CFG, MSG_CFG_SET_RATE, &msg, sizeof(msg));
} // ubloxSetMessageRate

void ubloxConfigureSBAS(uint8_t sbas_system) {
  struct ubx_cfg_sbas_settings msg;

  switch(sbas_system) {
  case SBAS_DISABLED:
    msg.mode = SBAS_MODE_DISABLE;
    msg.usage = 0;
    msg.maxSBAS = 0;
    msg.scanmode2 = 0;
    msg.scanmode1 = 0;
    break;
  case SBAS_WAAS:
    msg.mode = SBAS_MODE_ENABLE;
    msg.usage = SBAS_USAGE;
    msg.maxSBAS = 3;
    msg.scanmode2 = 0;
    msg.scanmode1 = SBAS_SATS_WAAS;
    break;  
  case SBAS_EGNOS:
    msg.mode = SBAS_MODE_ENABLE_TEST;
    msg.usage = SBAS_USAGE;
    msg.maxSBAS = 3;
    msg.scanmode2 = 0;
    msg.scanmode1 = SBAS_SATS_EGNOS;
    break;  
  case SBAS_MSAS:
    msg.mode = SBAS_MODE_ENABLE;
    msg.usage = SBAS_USAGE;
    msg.maxSBAS = 3;
    msg.scanmode2 = 0;
    msg.scanmode1 = SBAS_SATS_MSAS;
    break;  
  }
  ubloxSendMessage(CLASS_CFG, MSG_CFG_SBAS_SETTINGS,&msg, sizeof(msg));
} // ubloxConfigureSBAS

void ubloxInitSerial(uint32_t b) {
  uint8_t i;

#define UBLOX_SET_BINARY   "$PUBX,41,1,0003,0001,57600,0*2D\r\n"

  struct ubx_cfg_nav_rate msg;
  const unsigned long baudrates[5] = {
    9600UL, 19200UL, 38400UL, 57600UL, 115200UL   };
  /*
  for (i = 0; i < 5; i++) {
   serialOpen(GPS_SERIAL_PORT, baudrates[i]);
   rxEnabled[GPS_SERIAL_PORT] = false;
   ubloxWriteBuffer(GPS_SERIAL_PORT, (uint8_t *)UBLOX_SET_BINARY, sizeof(UBLOX_SET_BINARY) - 1);
   delay(300);
   serialEnd(GPS_SERIAL_PORT);
   }
   */
  serialOpen(GPS_SERIAL_PORT, b);
  //  delay(1000);

  msg.measure_rate_ms = 200;
  msg.nav_rate = 1;
  msg.timeref = 1; // UTC time
  ubloxSendMessage(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));

  // select sentences
  ubloxSetMessageRate(CLASS_NAV, MSG_POSLLH, 1);
  //ubloxSetMessageRate(CLASS_NAV, MSG_STATUS, 1);
  ubloxSetMessageRate(CLASS_NAV, MSG_SOL, 1);
  ubloxSetMessageRate(CLASS_NAV, MSG_VELNED, 1);

  //  ubloxConfigureSBAS(GPS_SBAS);

  //  ubloxSendMessage(CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0);	

} // ubloxInitSerial

void ubloxWriteBuffer(uint8_t port, uint8_t *b, uint8_t l) {
  uint8_t i;
  uint8_t *ptr;

  ptr = b;

  for (i = 0; i < l; i++) {
    serialWrite(port,*ptr);
    delayMicroseconds(100); // slow down for ublox to process
    ptr++;
  }
} // ubloxWriteBuffer

boolean ubloxReceivePacket(uint8_t data){
  enum {
    waitSentinel,
    waitPreamble,
    waitClass, 
    waitID, 
    waitLength,
    waitPayload,
    waitPayload2,
    waitChecksum,
    waitChecksum2
  }; 
  boolean parsed = false;

  switch(ubx_state) {
  case waitSentinel:
    if(PREAMBLE1 == data) 
      ubx_state = waitPreamble;
    break;
  case waitPreamble:
    if (PREAMBLE2 == data)
      ubx_state = waitClass;
    else
      ubx_state = waitSentinel;
    break;
  case waitClass:
    ubx_class = data;
    ck_b = ck_a = data; 
    ubx_state = waitID;
    break;
  case waitID:
    ck_b += (ck_a += data);  
    msg_id = data;
    ubx_state = waitLength;
    break;
  case waitLength:
    ck_b += (ck_a += data);
    ubx_len = data; 
    ubx_state = waitPayload;
    break;
  case waitPayload:
    ck_b += (ck_a += data);
    ubx_len += (uint16_t)(data<<8);
    if (ubx_len > 512) {
      ubx_len = 0;
      ubx_state = waitSentinel;
    }
    else {
      ubx_cc = 0;  // prepare to receive payload
      ubx_state = waitPayload2;
    }
    break;
  case waitPayload2:
    ck_b += (ck_a += data); 
    if (ubx_cc < sizeof(ubx_buff))
      ubx_buff.bytes[ubx_cc] = data;
    if (++ubx_cc == ubx_len)
      ubx_state = waitChecksum;
    break;
  case waitChecksum:
    if (ck_a == data) 
      ubx_state = waitChecksum2;
    else
      ubx_state = waitSentinel; 
    break;
  case waitChecksum2:
    if (ck_b == data) {
      f.GPS_ACTIVE = true;
      if (ubloxParsePacket()) 
        parsed = true;
    }
    ubx_state = waitSentinel; 
  } //end switch
  return parsed;
} // ubloxReceivePacket

boolean ubloxParsePacket(void) {
  static boolean sol_ok = false;
  static boolean vel_ok = false;

  switch (msg_id) {
  case MSG_POSLLH:
    //gpsLastPosUpdatemS = ubx_buff.posllh.time;
    if(sol_ok) {
      gpsPosition[LON] = ubx_buff.posllh.longitude;
      gpsPosition[LAT] = ubx_buff.posllh.latitude;
      gpsAltitude = ubx_buff.posllh.altitude_msl / 10; // mm -> cm
    }
    f.GPS_FIX = sol_ok; // && vel_ok;
    sol_ok = vel_ok = false;
    return true; // POSLLH message received, allow blink GUI icon and LED
    break;
  case MSG_SOL:
    sol_ok = (ubx_buff.solution.fix_status & NAV_STATUS_FIX_VALID) && (ubx_buff.solution.fix_type == FIX_3D);
    gpsSats = ubx_buff.solution.satellites;
    break;
  case MSG_VELNED:
    gpsSpeed = ubx_buff.velned.speed_2d; // cm/s
    gpsGroundCourse = (int16_t)(ubx_buff.velned.heading_2d / 10000);  // deg * 10
    vel_ok = true;
    break;
  default:
    break;
  }
  return false;
} // ubloxParsePacket

#endif
















