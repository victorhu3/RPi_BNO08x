#include <iostream>
#include <pigpio.h>

#include "RPi_BNO08x.h"

using namespace std;

//g++ -Wall -pthread test.cpp RPi_BNO08x.cpp -lpigpio 
// /usr/local/lib/python3.9/dist-packages/adafruit_bno08x
int main() {
    
    cout << "Test Program" << endl;

    BNO08x bno;

    bno.enable_feature(bno.BNO_REPORT_ACCELEROMETER);
    bno.enable_feature(bno.BNO_REPORT_GYROSCOPE);

    vector<float> accel, gyro;
    while(true) {
        gpioSleep(PI_TIME_RELATIVE, 0, 300000);
        accel = bno.acceleration();
        gyro = bno.acceleration();
        if(accel.size() > 0) printf("Accel: %f %f %f\n", accel[0], accel[1], accel[2]);
        if(gyro.size() > 0) printf("Gyro: %f %f %f\n", gyro[0], gyro[1], gyro[2]);
    }

    cout << "End Test Program" << endl;

    return 0;

}