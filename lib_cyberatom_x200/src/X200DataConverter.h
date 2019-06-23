#include <iostream>
#include <array>

namespace lib_cyberatom_x200
{

// partial according to cyberatom-x200 documentation:
//  https://cyberatom.eu/download/documents/xhtml/X-200/build/usb-protocol-request.html
enum Request
{
    GET_SYS_INFO = 0x01,
    GET_QUAT_DATA = 0x02,
    GET_ROT_RATE_DATA = 0x04,
    REBOOT = 0x05,
    GET_TEMP = 0x0F,
    SET_I2C_ADDR = 0x11,
    GET_CALIB_ACC = 0x26,
    REBOOT_BOOTLOADER = 0x29
};

// partial according to cyberatom-x200 documentation:
//  https://cyberatom.eu/download/documents/xhtml/X-200/build/usb-protocol-response.html
enum Response
{
    SYS_INFO = 0x81,
    QUAT_DATA = 0x82,
    ROT_RATE_DATA = 0x84,
    TEMP = 0x8F,
    I2C_ADDR = 0x91,
    CONFIRM = 0x92,
    CALIB_ACC = 0xA6,
    STATS = 0xC0
};

struct SysInfo
{
    std::string device_type;
    std::string firmware_version;
};

class X200DataConverter
{
    float unpackFloat(const unsigned char *buf, int i);

public:
    static std::string decodeResponseID(int response);
    SysInfo sysInfo(unsigned char *data, int size);
    std::array<float, 4> quaternion(unsigned char *data, int size);
    std::array<float, 3> rotationRate(unsigned char *data, int size);
    std::array<float, 3> acceleration(unsigned char *data, int size);
    float temperature(unsigned char *data, int size);
};

} // namespace lib_cyberatom_x200