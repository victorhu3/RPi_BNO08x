#define BNO08x_I2CADDR_DEFAULT 0x4A ///< The default I2C address

class BNO08x {

private:
  int _i2c_addr;
  int _i2c_handler;
  int _i2c_channel;
  uint8_t _sequence_number[6];
  uint8_t _read_buffer[1024];

public:

  typedef enum {
    BNO_CHANNEL_SHTP_COMMAND = 0,
    BNO_CHANNEL_EXE = 1,
    _BNO_CHANNEL_CONTROL = (2),
    _BNO_CHANNEL_INPUT_SENSOR_REPORTS = (3),
    _BNO_CHANNEL_WAKE_INPUT_SENSOR_REPORTS = (4),
    _BNO_CHANNEL_GYRO_ROTATION_VECTOR = (5)
  } bno_channel_t;

  typedef enum {
    _GET_FEATURE_REQUEST = (0xFF),
    _SET_FEATURE_COMMAND = (0xFE),
    _GET_FEATURE_RESPONSE = (0xFD),
    _BASE_TIMESTAMP = (0xFC),

    _TIMESTAMP_REBASE = (0xFB),

    _SHTP_REPORT_PRODUCT_ID_RESPONSE = (0xF8),
    _SHTP_REPORT_PRODUCT_ID_REQUEST = (0xF9),

    _FRS_WRITE_REQUEST = (0xF8),
    _FRS_WRITE_DATA = (0xF7),
    _FRS_WRITE_RESPONSE = (0xF6),

    _FRS_READ_REQUEST = (0xF5),
    _FRS_READ_RESPONSE = (0xF4),

    _COMMAND_REQUEST = (0xF3),
    _COMMAND_RESPONSE = (0xF2)
  } bno_message_t;

  BNO08x(uint8_t i2c_addr);
  BNO08x() : BNO08x(BNO08x_I2CADDR_DEFAULT) {};

  bool write8(uint8_t value);
  int read8();
  bool write(uint8_t* buffer, int len);
  bool read(uint8_t* buffer, int len);
  bool read_packet();
  void send_packet(bno_channel_t channel, uint8_t* data, int len);

  void soft_reset();
  bool check_id();



};