#if defined(USE_GPS) && !defined(SPEKTRUM)

prog_char UBLOX_INIT[] PROGMEM = { // PROGMEM array must be outside any function !!!
  0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19, //disable all default NMEA messages
  0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15,
  0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11,
  0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x00,0xFA,0x0F,
  0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x00,0xFC,0x13,
  0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x00,0xFE,0x17,
  0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x02,0x01,0x0E,0x47, //set POSLLH MSG rate
  0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x03,0x01,0x0F,0x49, //set STATUS MSG rate
  0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x06,0x01,0x12,0x4F, //set SOL MSG rate
  0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x12,0x01,0x1E,0x67, //set VELNED MSG rate
  0xB5,0x62,0x06,0x16,0x08,0x00,0x03,0x07,0x03,0x00,0x51,0x08,0x00,0x00,0x8A,0x41, // set WAAS to EGNOS ???
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A // set rate to 5Hz
};

prog_char SERIAL_INIT[] PROGMEM = {
  "$PUBX,41,1,0003,0001,115200,0*1E\r\n"};

struct ubx_header {
  uint8_t preamble1;
  uint8_t preamble2;
  uint8_t msgclass;
  uint8_t msg_id;
  uint16_t length;
};
struct ubx_nav_posllh {
  uint32_t time;  // GPS msToW
  int32_t longitude;
  int32_t latitude;
  int32_t altitude_ellipsoid;
  int32_t altitude_msl;
  uint32_t horizontal_accuracy;
  uint32_t vertical_accuracy;
};
struct ubx_nav_solution {
  uint32_t time;
  int32_t time_nsec;
  int16_t week;
  uint8_t fix_type;
  uint8_t fix_status;
  int32_t ecef_x;
  int32_t ecef_y;
  int32_t ecef_z;
  uint32_t position_accuracy_3d;
  int32_t ecef_x_velocity;
  int32_t ecef_y_velocity;
  int32_t ecef_z_velocity;
  uint32_t speed_accuracy;
  uint16_t position_DOP;
  uint8_t res;
  uint8_t satellites;
  uint32_t res2;
};
struct ubx_nav_velned {
  uint32_t time;  // GPS msToW
  int32_t ned_north;
  int32_t ned_east;
  int32_t ned_down;
  uint32_t speed_3d;
  uint32_t speed_2d;
  int32_t heading_2d;
  uint32_t speed_accuracy;
  uint32_t heading_accuracy;
};

enum ubs_protocol_bytes {
  UBX_PREAMBLE_1 = 0xb5,
  UBX_PREAMBLE_2 = 0x62,
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
  MSG_CFG_NAV_SETTINGS = 0x24
};
enum ubs_nav_fix_type {
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

static uint8_t ck_a, ck_b;
static uint8_t ubx_state;
static uint8_t msg_id;
static uint8_t ubx_len;
static uint8_t ubx_cc;
static uint8_t ubx_class;
static uint8_t fix_ok;

// Receive buffer
static union {
  ubx_nav_posllh posllh;
  //    ubx_nav_status status;
  ubx_nav_solution solution;
  ubx_nav_velned velned;
  uint8_t bytes[];
} 
ubx_buff;

void ubloxInitSerial(void) {
  const int32_t DEF_BAUD[] = {
    4800, 9600, 19200, 38400, 57600, 115200
  };
  uint8_t i;

  serialOpen(GPS_SERIAL_PORT, 115200);  
  delay(1000);

  for(i = 0; i < sizeof(DEF_BAUD); i++){
    serialOpen(GPS_SERIAL_PORT, DEF_BAUD[i]); 
    for(i = 0; i < sizeof(SERIAL_INIT); i++) {
      serialWrite(GPS_SERIAL_PORT, pgm_read_byte(SERIAL_INIT+i));
      delay(5);
    }
    while(serialAvailable(GPS_SERIAL_PORT)) delay(10);
  }

  delay(200);
  serialOpen(GPS_SERIAL_PORT, 115200);  
  for(i = 0; i < sizeof(UBLOX_INIT); i++) {
    serialWrite(GPS_SERIAL_PORT, pgm_read_byte(UBLOX_INIT+i));
    delay(5); // slow to 38Kbaud pace
  }
} // ubloxInitSerial

boolean ubloxReceivePacket(uint8_t data){
  boolean parsed = false;
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

  switch(ubx_state) {
  case waitSentinel:
    if(UBX_PREAMBLE_1 == data) 
      ubx_state = waitPreamble;
    break;
  case waitPreamble:
    if (UBX_PREAMBLE_2 == data)
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
  switch (msg_id) {
  case MSG_POSLLH:
    //i2c_dataset.time                = ubx_buff.posllh.time;
    if(fix_ok) {
#if defined(FAKE_GPS)
      gpsPosition[LON] += 100;
      gpsPosition[LAT] += 100;          
#else
      gpsPosition[LON] = ubx_buff.posllh.longitude;
      gpsPosition[LAT] = ubx_buff.posllh.latitude;
#endif
      gpsAltitude = ubx_buff.posllh.altitude_msl / 10; // mm -> cm
    }
    f.GPS_FIX = fix_ok;
    return true; // POSLLH message received, allow blink GUI icon and LED
    break;
  case MSG_SOL:
    fix_ok = false;
    if((ubx_buff.solution.fix_status & NAV_STATUS_FIX_VALID) && (ubx_buff.solution.fix_type == FIX_3D || ubx_buff.solution.fix_type == FIX_2D)) fix_ok = true;
    gpsSats = ubx_buff.solution.satellites;
    break;
  case MSG_VELNED:
#if defined(FAKE_GPS)
    gpsSpeed = 600;  // cm/s
    gpsGroundCourse = 450;
#else
    gpsSpeed = ubx_buff.velned.speed_2d;  // cm/s
    gpsGroundCourse = (uint16_t)(ubx_buff.velned.heading_2d / 100000);  // Heading 2D deg * 10^6 -> degx10
#endif
    break;
  default:
    break;
  }
  return false;
} // ubloxParsePacket

#endif











