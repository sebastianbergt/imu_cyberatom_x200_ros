#include <iostream>
#include <iomanip> // setfill in cout
#include <array>
#include <exception>
#include <thread>
#include "CyberAtomUsb.h"
#include "TicToc.h"

using namespace std;
using namespace lib_cyberatom_x200;

uint16_t callback_counter = 0;

std::array<float, 4> quaternion;
std::array<float, 3> rotationRate;
std::array<float, 3> acceleration;
SysInfo x200sysinfo;

void imuCallback(unsigned char buffer[], int length)
{
    callback_counter++;
    // buffer can be at maximum CyberAtomUsb::MSG_SIZE
    cout << "userDefinedCallback callback_counter=" << callback_counter << ", responseId=" << X200DataConverter::decodeResponseID(buffer[0]) << endl;
    X200DataConverter conv;
    switch (buffer[0])
    {
    case Response::SYS_INFO:
        x200sysinfo = conv.sysInfo(buffer, length);
        cout << "Received device_type = " << x200sysinfo.device_type
             << "Received firmware_version = " << x200sysinfo.firmware_version << endl;
        return;
    case Response::QUAT_DATA:
        quaternion = conv.quaternion(buffer, length);
        cout << "Received quaternion = (" << quaternion[0] << ", " << quaternion[1] 
            << ", " << quaternion[2] << ", " << quaternion[3] << ")" << endl;
        return;
    case Response::ROT_RATE_DATA:
        rotationRate = conv.rotationRate(buffer, length);
        cout << "Received rotationRate = (" << rotationRate[0] << ", " << rotationRate[1] 
            << ", " << rotationRate[2] << ")" << endl;
        return;
    case Response::CALIB_ACC:
        acceleration = conv.acceleration(buffer, length);
        cout << "Received acceleration = (" << acceleration[0] << ", " << acceleration[1] 
            << ", " << acceleration[2] << ")" << endl;
        toc();
        tic();
        return;
    default:
        cout << "Received response = "
                << "0x" << setfill('0') << setw(2) << hex << (int)buffer[0] << endl;
    }
};

int main()
{
    try
    {
        CyberAtomUsb imu;
        cout << "init" << endl;
        tic();
        imu.usbInit(CYBERATOM_VENDOR_ID, CYBERATOM_PRODUCT_ID);
        cout << "triggered replies" << endl;
        imu.usbRegisterReceiveCallback(imuCallback);
        imu.usbSendMultishot(Request::GET_QUAT_DATA);
        imu.usbSendMultishot(Request::GET_ROT_RATE_DATA);
        imu.usbSendMultishot(Request::GET_CALIB_ACC);
        cout << "handling events" << endl;
        for (int i = 0; i < 2000; ++i)
        {
            imu.usbHandleEventsForMicroSec(1000);
            //this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        imu.usbCancelAllTransfers();
        this_thread::sleep_for(std::chrono::seconds(3));
    }
    catch (exception &e)
    {
        cout << e.what() << endl;
    }

    return 0;
}
