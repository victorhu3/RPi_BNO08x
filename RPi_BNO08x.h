#include <map>
#include <math.h>
#include <vector>
using namespace std;

#define BNO08x_I2CADDR_DEFAULT 0x4A ///< The default I2C address

class BNO08x {

private:
  int _i2c_addr;
  int _i2c_handler;
  int _i2c_channel;
  uint8_t _sequence_number[6];
  uint8_t _read_buffer[1024];
  map<int, vector<float>> _readings;

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
    _GET_FEATURE_REQUEST = (0xFE),
    _SET_FEATURE_COMMAND = (0xFD),
    _GET_FEATURE_RESPONSE = (0xFC),
    _BASE_TIMESTAMP = (0xFB),

    _TIMESTAMP_REBASE = (0xFA),

    _SHTP_REPORT_PRODUCT_ID_RESPONSE = (0xF8),
    _SHTP_REPORT_PRODUCT_ID_REQUEST = (0xF9),

    _FRS_WRITE_REQUEST = (0xF7),
    _FRS_WRITE_DATA = (0xF6),
    _FRS_WRITE_RESPONSE = (0xF5),

    _FRS_READ_REQUEST = (0xF4),
    _FRS_READ_RESPONSE = (0xF3),

    _COMMAND_REQUEST = (0xF2),
    _COMMAND_RESPONSE = (0xF1)
  } bno_message_t;

  typedef enum {
    // Calibrated Acceleration (m/s2)
    BNO_REPORT_ACCELEROMETER = (0x01),
    // Calibrated gyroscope (rad/s).
    BNO_REPORT_GYROSCOPE = (0x02),
    // Magnetic field calibrated (in µTesla). The fully calibrated magnetic field measurement.
    BNO_REPORT_MAGNETOMETER = (0x03),
    // Linear acceleration (m/s2). Acceleration of the device with gravity removed
    BNO_REPORT_LINEAR_ACCELERATION = (0x04),
    // Rotation Vector
    BNO_REPORT_ROTATION_VECTOR = (0x05),
    // Gravity Vector (m/s2). Vector direction of gravity
    BNO_REPORT_GRAVITY = (0x06),
    // Game Rotation Vector
    BNO_REPORT_GAME_ROTATION_VECTOR = (0x08),

    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR = (0x09),

    BNO_REPORT_STEP_COUNTER = (0x11),

    BNO_REPORT_RAW_ACCELEROMETER = (0x14),
    BNO_REPORT_RAW_GYROSCOPE = (0x15),
    BNO_REPORT_RAW_MAGNETOMETER = (0x16),
    BNO_REPORT_SHAKE_DETECTOR = (0x19),

    BNO_REPORT_STABILITY_CLASSIFIER = (0x13),
    BNO_REPORT_ACTIVITY_CLASSIFIER = (0x1E),
    BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR = (0x2A)
  } bno_report_t;

  typedef struct {
    // The entire report is of "length"
    float scalar;
    int count, length;
  } sensor_data_t;

  const static map<int, int> _REPORT_LENGTHS;
  const static map<int, sensor_data_t> _SENSOR_REPORTS;

  const static int _DEFAULT_REPORT_INTERVAL = 50000;
  static constexpr float _Q_POINT_14_SCALAR = pow(2, 14 * -1);
  static constexpr float _Q_POINT_12_SCALAR = pow(2, 12 * -1);
  static constexpr float _Q_POINT_10_SCALAR = pow(2, 10 * -1);
  static constexpr float _Q_POINT_9_SCALAR = pow(2, 9 * -1);
  static constexpr float _Q_POINT_8_SCALAR = pow(2, 8 * -1);
  static constexpr float _Q_POINT_4_SCALAR = pow(2, 4 * -1);

  BNO08x(uint8_t i2c_addr);
  BNO08x() : BNO08x(BNO08x_I2CADDR_DEFAULT) {};

  bool write8(uint8_t value);
  int read8();
  bool write(uint8_t* buffer, int len);
  bool read(uint8_t* buffer, int len);
  bool read_packet();
  void split_reports();
  void send_packet(bno_channel_t channel, uint8_t* data, int len);
  void process_available_packets(int max_packets = 1);
  void process_reports();
  int get_report_length(uint8_t report_id);

  void soft_reset();
  bool check_id();
  void enable_feature(bno_report_t feature_id, int report_interval = _DEFAULT_REPORT_INTERVAL,
                    int sensor_specific_config = 0);


  vector<float> acceleration();
  vector<float> gyroscope();



};