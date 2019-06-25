#include <iostream>
#include <array>
#include <vector>
#ifndef UNDER_TEST
#include <libusb-1.0/libusb.h>
#endif
#include <inttypes.h>
#include "X200DataConverter.h"

namespace lib_cyberatom_x200
{

constexpr int MSG_SIZE = 1024;
constexpr unsigned char DEVICE_ENDPOINT = 1;
constexpr uint16_t CYBERATOM_VENDOR_ID = 0xFFFF;
constexpr uint16_t CYBERATOM_PRODUCT_ID = 0x0002;
constexpr unsigned int TIMEOUT = 100; //in milliseconds
constexpr char static EXCEPTION_MSG[] = "Unexpected response message id";
constexpr bool DEBUG = false;

class CyberAtomUsb
{
public:
    CyberAtomUsb(){};
    ~CyberAtomUsb();
    // user API
    void usbInit(uint16_t vendor_id, uint16_t product_id);
    void usbRegisterReceiveCallback(void userDefinedCallback(unsigned char buffer[], int length)); // updates data with received data from device
    void usbSendSingleshot(unsigned char request_id);                                              // fire transfer once
    void usbSendMultishot(unsigned char request_id);                                               // fire transfer again, when transfer completed, needs to be canceled actively
    void usbHandleEventsForMicroSec(int timeout_usec);
    void usbHandleEventsForSec(int timeout_sec);
    void usbCancelAllTransfers();
    // libusb callbacks
    static void usbSendCallbackSingleshot(struct libusb_transfer *transfer);
    static void usbSendCallbackMultishot(struct libusb_transfer *transfer);
    static void usbReceiveCallback(struct libusb_transfer *transfer);
    // libusb helper
    static std::string usbDecodeError(int error);
    static std::string usbDecodeTransferStatus(int transfer_status);

private:
    // libusb interaction
    libusb_device **deviceList_;             // pointer to pointer of device, used to retrieve a list of devices
    libusb_device_handle *deviceHandle_; // a device handle
    libusb_context *usbContext_ = NULL;       // a libusb session
    //unsigned char data[MSG_SIZE];     // buffer for messages
    int actual_length; // used to find out how many bytes were written
    int r;             // for return values
    // transfer data storage
    std::vector<std::array<unsigned char, MSG_SIZE>> transfer_data{};
    std::vector<struct libusb_transfer *> transfers{};
    // freeing libusb data structures
    bool initialized = false; // determines if cleanup is necessary
    void cleanup();           // releases c data structures created by libusb
};

} // namespace lib_cyberatom_x200