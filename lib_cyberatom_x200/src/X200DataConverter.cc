#include <iostream>
#include <iomanip> // setfill in cout
#include <array>
#include "X200DataConverter.h"

namespace lib_cyberatom_x200
{

using namespace std;

SysInfo X200DataConverter::sysInfo(unsigned char *data, int size)
{
    SysInfo sysInfo;
    sysInfo.device_type.assign((char *)&data[1], 8);
    sysInfo.firmware_version.assign((char *)&data[9], 24);
    return sysInfo;
}

std::array<float, 4> X200DataConverter::quaternion(unsigned char *data, int size)
{
    std::array<float, 4> quat;
    quat[0] = unpackFloat(data, 1);
    quat[1] = unpackFloat(data, 5);
    quat[2] = unpackFloat(data, 9);
    quat[3] = unpackFloat(data, 13);
    return quat;
};

std::array<float, 3> X200DataConverter::rotationRate(unsigned char *data, int size)
{
    std::array<float, 3> rotation_rate;
    rotation_rate[0] = unpackFloat(data, 1);
    rotation_rate[1] = unpackFloat(data, 5);
    rotation_rate[2] = unpackFloat(data, 9);
    return rotation_rate;
};

std::array<float, 3> X200DataConverter::acceleration(unsigned char *data, int size)
{
    std::array<float, 3> acc;
    acc[0] = unpackFloat(data, 1);
    acc[1] = unpackFloat(data, 5);
    acc[2] = unpackFloat(data, 9);
    return acc;
};

float X200DataConverter::temperature(unsigned char *data, int size)
{
    return unpackFloat(data, 1);
};

float X200DataConverter::unpackFloat(const unsigned char *buf, int i)
{
    typedef union {
        unsigned char b[4];
        float f;
    } conv_t;
    conv_t conv;
    for (int j = 0; j < 4; ++j)
    {
        conv.b[j] = buf[i + j];
    }
    return conv.f;
};

std::string X200DataConverter::decodeResponseID(int response)
{
    switch (response)
    {
    case Response::SYS_INFO:
        return ("SYS_INFO");
    case Response::QUAT_DATA:
        return ("QUAT_DATA");
    case Response::ROT_RATE_DATA:
        return ("ROT_RATE_DATA");
    case Response::TEMP:
        return ("TEMP");
    case Response::CONFIRM:
        return ("CONFIRM");
    case Response::CALIB_ACC:
        return ("CALIB_ACC");
    case Response::STATS:
        return ("STATS");
    default:
        cout << "Received response = "
             << "0x" << setfill('0') << setw(2) << hex << (int) response << endl;
        return ("Response not in list");
    }
};

}