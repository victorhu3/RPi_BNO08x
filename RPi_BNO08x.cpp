#include <pigpio.h>
#include <iostream>
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
    printf("%d %d %d %d\n", buffer[0], buffer[1], buffer[2], buffer[3]);
    #endif

    write(buffer, len + 4);
    _sequence_number[channel] = (_sequence_number[channel] + 1) % 256;
}

bool BNO08x::read_packet() {
    // When reading a packet, first read 4 byte header. Then
    // read ENTIRE packet_length again, including 4 byte header again
    read(_read_buffer, 4);
    int packet_length = (_read_buffer[1] << 8) + _read_buffer[0];
    packet_length &= ~0x8000;
    int channel_number = _read_buffer[2];
    int sequence_number = _read_buffer[3];
    read(_read_buffer, packet_length);

    #ifdef DEBUG
    printf("Reading packet\n");
    cout << packet_length << " " << channel_number << " " << sequence_number << " ";
    for(int i = 4; i < packet_length; i++) {
        cout <<  (int)_read_buffer[i] << " ";
    }
    cout << endl;
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
    i2cReadDevice(_i2c_handler, (char*)buffer, len);
    return true;
    
}

bool BNO08x::write(uint8_t* buffer, int len) {
    /*for(int i = 0; i < len; i++) {
        write8(buffer[i]);
    }*/
    i2cWriteDevice(_i2c_handler, (char*)buffer, len);
    return true;
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