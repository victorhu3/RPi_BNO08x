#include <iostream>

#include "RPi_BNO08x.h"

using namespace std;

//g++ -Wall -pthread test.cpp RPi_BNO08x.cpp -lpigpio 
// /usr/local/lib/python3.9/dist-packages/adafruit_bno08x
int main() {
    
    cout << "Test Program" << endl;

    BNO08x bno;

    cout << "End Test Program" << endl;

    return 0;

}