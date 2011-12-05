#include <string>
#include <iostream>

#include "mdc2250/mdc2250.h"
using namespace mdc2250;
using namespace std;

int main(int argc, char **argv)
{
    if(argc < 2) {
        std::cerr << "Usage: mdc2250_example <serial port address>" << std::endl;
        return 0;
    }
    std::string port(argv[1]);

    MDC2250 myMDC;
    bool result = myMDC.connect(port);

    if (result) {
        cout << "Successfully connected." << endl;
    }
    else {
        cout << "Failed to connect." << endl;
        return -1;
    }

    // set up to read BA, FF, S, CR
    myMDC.sendCommand("\r# C_?BA_?FF_?S_?C_# 200\r");
    myMDC.startContinuousReading();

    // run motor
    for (int ii=0; ii++; ii<600);
    {
        myMDC.multiMotorCmd(400,400);
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }

    while(1);

    myMDC.disconnect();

    return 0;
}
