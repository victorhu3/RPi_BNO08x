#include <pigpio.h>
#include <iostream>
#include <map>
#include <cstring>
using namespace std;

#include "RPi_BNO08x.h"

#define DEBUG

BNO08x::BNO08x(uint8_t i2c_addr) {

    #ifdef DEBUG
    cout << "Initializing BNO..." << endl;
    #endif

    _i2c_addr = i2c_addr;
    _i2c_channel = 1;
    memset(_sequence_number, 0, sizeof(_sequence_number));

    if(gpioInitialise() < 0) {
        cout << "pigpio init failed" << endl;
        exit(1);
    }

    _i2c_handler = i2cOpen(_i2c_channel, _i2c_addr, 0);
    if(_i2c_handler < 0) {
        cout << "Error opening i2c" << endl;
        exit(1);
    }

    soft_reset();

    if(!check_id()) {
        cout << "Read ID failed" << endl;
        exit(1);
    }
}

void BNO08x::enable_feature(bno_report_t feature_id, int report_interval, int sensor_specific_config) {
    uint8_t data[17] = {0};
    if (feature_id == BNO_REPORT_ACTIVITY_CLASSIFIER) {
        // something
    } else {
        data[0] = _SET_FEATURE_COMMAND;
        data[1] = feature_id;
        data[5] = report_interval & 0xFF;
        data[6] = (report_interval >> 8) & 0xFF;
        data[7] = (report_interval >> 16) & 0xFF;
        data[8] = (report_interval >> 24) & 0xFF;
        data[13] = sensor_specific_config & 0xFF;
        data[14] = (sensor_specific_config >> 8) & 0xFF;
        data[15] = (sensor_specific_config >> 16) & 0xFF;
        data[16] = (sensor_specific_config >> 24) & 0xFF;
    }
    send_packet(_BNO_CHANNEL_CONTROL, data, 17);
    gpioSleep(PI_TIME_RELATIVE, 0, 200000);
}

void BNO08x::process_available_packets(int max_packets) {

    for(int i = 0; i < max_packets; i++) {
        if (!read_packet()) {
            return;
        }
        int packet_length = (_read_buffer[1] << 8) + _read_buffer[0];
        packet_length &= ~0x8000;
        int channel_number = _read_buffer[2];
        int sequence_number = _read_buffer[3];
        if (channel_number == _BNO_CHANNEL_INPUT_SENSOR_REPORTS) {
            process_reports();
        }
    }
}

void BNO08x::process_reports() {
    int packet_length = (_read_buffer[1] << 8) + _read_buffer[0];
    packet_length &= ~0x8000;
    int channel_number = _read_buffer[2];
    int sequence_number = _read_buffer[3];
    int idx = 4;
    int report_id;
    while(idx < packet_length) {
        report_id = _read_buffer[idx];
        #ifdef DEBUG
        printf("report %x %d %d\n", report_id,idx, packet_length);
        #endif
        if(report_id == BNO_REPORT_ACCELEROMETER || report_id == BNO_REPORT_GYROSCOPE) {

            sensor_data_t info = _SENSOR_REPORTS.at(report_id);
            vector<float> reading;
            int data_start = 4 + idx;
            for(int i = data_start; i < data_start + info.count*2; i+=2) {
                short unscaled = (_read_buffer[i + 1] << 8) + _read_buffer[i];
                reading.push_back((float)unscaled * info.scalar);
            }

            _readings[report_id] = reading;
        }
        int report_length = get_report_length(report_id);
        idx += report_length;
    }
}

int BNO08x::get_report_length(uint8_t report_id) {
    if (report_id < 0xF0) {
        return _SENSOR_REPORTS.at(report_id).length;

    }
    return _REPORT_LENGTHS.at(report_id);
}

void BNO08x::split_reports() {


}

bool BNO08x::check_id() {
    uint8_t buf[] = {_SHTP_REPORT_PRODUCT_ID_REQUEST, 0};
    send_packet(_BNO_CHANNEL_CONTROL, buf, 2);
    gpioSleep(PI_TIME_RELATIVE, 0, 100000);
    read_packet();

    return _read_buffer[4] == _SHTP_REPORT_PRODUCT_ID_RESPONSE;
}

void BNO08x::send_packet(bno_channel_t channel, uint8_t* data, int len) {

    uint8_t buffer[len + 4] = {0};
    // packet size is little endian
    buffer[0] = (len + 4) & 0xFF;
    buffer[1] = ((len + 4) >> 8) & 0xFF;
    buffer[2] = channel;
    buffer[3] = _sequence_number[channel];
    memcpy(buffer + 4, data, len);

    #ifdef DEBUG
    printf("Sending packet\n");
    printf("%d %d %d\n", len+4, buffer[2], buffer[3]);
    for(int i = 0; i < len; i++) {
        printf("0x%x ", data[i]);
    }
    printf("\n");
    #endif

    write(buffer, len + 4);
    _sequence_number[channel] = (_sequence_number[channel] + 1) % 256;
}

bool BNO08x::read_packet() {
    // When reading a packet, first read 4 byte header. Then
    // read ENTIRE packet_length again, including 4 byte header again
    if (!read(_read_buffer, 4)) return false;
    int packet_length = (_read_buffer[1] << 8) + _read_buffer[0];
    packet_length &= ~0x8000;
    int channel_number = _read_buffer[2];
    int sequence_number = _read_buffer[3];
    read(_read_buffer, packet_length);

    #ifdef DEBUG
    printf("Reading packet\n");
    printf("%d %d %d ", packet_length, channel_number, sequence_number);
    for(int i = 4; i < packet_length; i++) {
        printf("0x%x ",(int)_read_buffer[i]);
    }
    printf("\n");
    #endif

    _sequence_number[channel_number] = sequence_number;
    return true;
}


bool BNO08x::write8(uint8_t value) {
    i2cWriteByte(_i2c_handler, value);
    return true;
}

int BNO08x::read8() {
    return i2cReadByte(_i2c_handler);
}

bool BNO08x::read(uint8_t* buffer, int len) {
    /*
    for(int i = 0; i < len; i++) {
        int r = read8();
        if(r < 0) return false;
        buffer[i] = (uint8_t) r;
    }*/
    return i2cReadDevice(_i2c_handler, (char*)buffer, len) >= 0;
    
}

bool BNO08x::write(uint8_t* buffer, int len) {
    /*for(int i = 0; i < len; i++) {
        write8(buffer[i]);
    }*/
    return i2cWriteDevice(_i2c_handler, (char*)buffer, len) >= 0;
}

void BNO08x::soft_reset() {
    uint8_t buf[] = {1};
    send_packet(BNO_CHANNEL_EXE, buf, 1);
    gpioSleep(PI_TIME_RELATIVE, 0, 500000);
    send_packet(BNO_CHANNEL_EXE, buf, 1);
    gpioSleep(PI_TIME_RELATIVE, 0, 500000);
    read_packet();
    read_packet();
    read_packet();
}

vector<float> BNO08x::acceleration() {
    process_available_packets(100);
    //gpioSleep(PI_TIME_RELATIVE, 0, 500000);
    return _readings[BNO_REPORT_ACCELEROMETER];
}

vector<float> BNO08x::gyroscope() {
    process_available_packets(100);
    //gpioSleep(PI_TIME_RELATIVE, 0, 500000);
    return _readings[BNO_REPORT_GYROSCOPE];
}

const map<int, int> BNO08x::_REPORT_LENGTHS = {
    {_SHTP_REPORT_PRODUCT_ID_RESPONSE, 16},
    {_GET_FEATURE_RESPONSE, 17},
    {_COMMAND_RESPONSE, 16},
    {_SHTP_REPORT_PRODUCT_ID_RESPONSE, 16},
    {_BASE_TIMESTAMP, 5},
    {_TIMESTAMP_REBASE, 5}
};

const map<int, BNO08x::sensor_data_t> BNO08x::_SENSOR_REPORTS = {
    {BNO_REPORT_ACCELEROMETER, sensor_data_t{ _Q_POINT_8_SCALAR, 3, 10}},
    {BNO_REPORT_GRAVITY, sensor_data_t{ _Q_POINT_8_SCALAR, 3, 10}},
    {BNO_REPORT_GYROSCOPE, sensor_data_t{ _Q_POINT_9_SCALAR, 3, 10}},
    {BNO_REPORT_MAGNETOMETER, sensor_data_t{ _Q_POINT_4_SCALAR, 3, 10}},
    {BNO_REPORT_LINEAR_ACCELERATION, sensor_data_t{ _Q_POINT_8_SCALAR, 3, 10}},
    {BNO_REPORT_ROTATION_VECTOR, sensor_data_t{ _Q_POINT_14_SCALAR, 4, 14}},
    {BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR, sensor_data_t{ _Q_POINT_12_SCALAR, 4, 14}},
    {BNO_REPORT_GAME_ROTATION_VECTOR, sensor_data_t{ _Q_POINT_14_SCALAR, 4, 12}},
    {BNO_REPORT_STEP_COUNTER, sensor_data_t{ 1, 1, 12}},
    {BNO_REPORT_SHAKE_DETECTOR, sensor_data_t{ 1, 1, 6}},
    {BNO_REPORT_STABILITY_CLASSIFIER, sensor_data_t{ 1, 1, 6}},
    {BNO_REPORT_ACTIVITY_CLASSIFIER, sensor_data_t{ 1, 1, 16}},
    {BNO_REPORT_RAW_ACCELEROMETER, sensor_data_t{ 1, 3, 16}},
    {BNO_REPORT_RAW_GYROSCOPE, sensor_data_t{ 1, 3, 16}},
    {BNO_REPORT_RAW_MAGNETOMETER, sensor_data_t{ 1, 3, 16}},
};
