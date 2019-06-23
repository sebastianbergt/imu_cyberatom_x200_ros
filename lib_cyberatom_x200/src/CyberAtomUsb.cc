#include <algorithm>
#include <iostream>
#include <iterator>
#include <iomanip> // displaying hex for debugging
#include <array>
#include <inttypes.h>
#include <libusb-1.0/libusb.h>
#include <stdexcept>
#include <thread> // std::this_thread::sleep_for
#include "CyberAtomUsb.h"
//#include "TicToc.h"

namespace lib_cyberatom_x200
{
using namespace std;

CyberAtomUsb::~CyberAtomUsb()
{
    if (initialized)
    {
        cleanup();
    }
};

void CyberAtomUsb::usbInit(uint16_t vendor_id, uint16_t product_id)
{
    r = libusb_init(&usbContext_); //initialize the library for the session we just declared
    if (r != libusb_error::LIBUSB_SUCCESS)
    {
        cout << usbDecodeError(r) << endl;
        throw std::runtime_error("Cannot initialize libusb");
    }
    libusb_set_debug(usbContext_, 3);                                //set verbosity level to 3, as suggested in the documentation
    ssize_t cnt = libusb_get_device_list(usbContext_, &deviceList_); //get the list of devices
    if (cnt < 0)
    {
        cout << usbDecodeError(cnt) << endl;
        throw std::runtime_error("Get device error");
    }
    cout << cnt << " Devices in list." << endl;
    deviceHandle_ = libusb_open_device_with_vid_pid(usbContext_, vendor_id, product_id); //these are vendorID and productID I found for my usb device
    if (deviceHandle_ == NULL)
    {
        libusb_free_device_list(deviceList_, 1); //free the list, unref the devices in it
        throw std::runtime_error("Cannot open device");
    }
    else
    {
        cout << "Device Opened" << endl;
        libusb_free_device_list(deviceList_, 1); //free the list, unref the devices in it
    }
    if (libusb_kernel_driver_active(deviceHandle_, 0) == 1) //find out if kernel driver is attached
    {
        cout << "Kernel Driver Active" << endl;
        r = libusb_detach_kernel_driver(deviceHandle_, 0); //detach it
        if (r != libusb_error::LIBUSB_SUCCESS)
        {
            cout << usbDecodeError(cnt) << endl;
            throw std::runtime_error("Could not detach Kernel Driver");
        }
        cout << "Kernel Driver Detached!" << endl;
    }
    r = libusb_claim_interface(deviceHandle_, 0); //claim interface 0 (the first) of device (mine had just 1)
    if (r < 0)
    {
        cout << usbDecodeError(cnt) << endl;
        throw std::runtime_error("Cannot claim interface");
    }
    cout << "Claimed Interface" << endl;
    initialized = true;
}

void CyberAtomUsb::cleanup()
{
    // cleanup in flight transfers
    usbCancelAllTransfers();
    // cleanup deviceHandle_ and context
    r = libusb_release_interface(deviceHandle_, 0); //release the claimed interface
    if (r == libusb_error::LIBUSB_SUCCESS)
    {
        if (DEBUG)
        {
            cout << "Released Interface" << endl;
        }
    }
    else
    {
        if (DEBUG)
        {
            cout << "Cannot Release Interface" << endl;
            cout << usbDecodeError(r) << endl;
        }
    }
    libusb_close(deviceHandle_); //close the device we opened
    libusb_exit(usbContext_);    //needs to be called to end the
    if (DEBUG)
    {
        cout << "Returned device handle and exited libusb" << endl;
    }
}

void CyberAtomUsb::usbSendCallbackMultishot(struct libusb_transfer *transfer)
{
    if (transfer->status == libusb_transfer_status::LIBUSB_TRANSFER_COMPLETED)
    {
        transfer->buffer[0] = (unsigned char)(intptr_t)transfer->user_data;
        if (DEBUG)
        {
            cout << "usbSendCallbackMultishot executed for "
                 << "0x" << setfill('0') << setw(2)
                 << hex << (int)(transfer->buffer[0])
                 << dec << " length was " << transfer->actual_length << endl;
        }
        libusb_submit_transfer(transfer);
    }
    else // LIBUSB_TRANSFER_CANCELLED, LIBUSB_TRANSFER_NO_DEVICE LIBUSB_TRANSFER_TIMED_OUT, LIBUSB_TRANSFER_ERROR, LIBUSB_TRANSFER_STALL, LIBUSB_TRANSFER_OVERFLOW
    {
        if (DEBUG)
        {
            cout << "usbSendCallbackMultishot for "
                 << CyberAtomUsb::usbDecodeTransferStatus(transfer->status) << endl;
        }
        libusb_free_transfer(transfer);
        transfer = nullptr;
    }
}

void CyberAtomUsb::usbSendCallbackSingleshot(struct libusb_transfer *transfer)
{
    if (transfer->status == libusb_transfer_status::LIBUSB_TRANSFER_COMPLETED)
    {
        if (DEBUG)
        {
            cout << "usbSendCallbackSingleshot executed for "
                 << "0x" << setfill('0') << setw(2)
                 << hex << (int)(transfer->buffer[0])
                 << dec << " length was " << transfer->actual_length << endl;
        }
    }
    else // LIBUSB_TRANSFER_CANCELLED, LIBUSB_TRANSFER_NO_DEVICE LIBUSB_TRANSFER_TIMED_OUT, LIBUSB_TRANSFER_ERROR, LIBUSB_TRANSFER_STALL, LIBUSB_TRANSFER_OVERFLOW
    {
        if (DEBUG)
        {
            cout << "usbSendCallbackSingleshot for "
                 << CyberAtomUsb::usbDecodeTransferStatus(transfer->status) << endl;
        }
    }
    libusb_free_transfer(transfer);
    transfer = nullptr;
}

void CyberAtomUsb::usbReceiveCallback(struct libusb_transfer *transfer)
{
    if (transfer->status == libusb_transfer_status::LIBUSB_TRANSFER_COMPLETED)
    {
        if (transfer->buffer[0] != 0)
        {
            if (DEBUG)
            {
                cout << "usbReceiveCallback executed for "
                     << "0x" << setfill('0') << setw(2)
                     << hex << (int)transfer->buffer[0]
                     << " = " << X200DataConverter::decodeResponseID(transfer->buffer[0]) << endl;
            }
            if (transfer->user_data != nullptr)
            {
                void (*userDefinedCallback)(unsigned char[], int length) = reinterpret_cast<void (*)(unsigned char[], int length)>(transfer->user_data);
                userDefinedCallback(transfer->buffer, transfer->length);
            }
        }
    }
    else
    {
        // LIBUSB_TRANSFER_CANCELLED
        // LIBUSB_TRANSFER_NO_DEVICE
        // LIBUSB_TRANSFER_TIMED_OUT
        // LIBUSB_TRANSFER_ERROR
        // LIBUSB_TRANSFER_STALL
        // LIBUSB_TRANSFER_OVERFLOW
        libusb_free_transfer(transfer);
        transfer = nullptr;
        return;
    }
    libusb_submit_transfer(transfer);
}

void CyberAtomUsb::usbSendSingleshot(unsigned char request_id)
{
    // prep
    std::array<unsigned char, MSG_SIZE> t = {(unsigned char)request_id};
    transfer_data.push_back(t);
    unsigned char *data = transfer_data.back().data();
    void *user_data = (void *)(intptr_t)request_id;
    // fill
    struct libusb_transfer *transfer = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(transfer, deviceHandle_, (1 | LIBUSB_ENDPOINT_OUT),
                              data, MSG_SIZE, &usbSendCallbackSingleshot,
                              user_data, TIMEOUT);
    // submit
    r = libusb_submit_transfer(transfer);
    if (r != libusb_error::LIBUSB_SUCCESS)
    {
        libusb_free_transfer(transfer);
        transfer = nullptr;
        cout << usbDecodeError(r) << endl;
        throw std::runtime_error("");
    }
};

void CyberAtomUsb::usbSendMultishot(unsigned char request_id)
{
    // prep
    std::array<unsigned char, MSG_SIZE> t = {(unsigned char)request_id};
    transfer_data.push_back(t);
    unsigned char *data = transfer_data.back().data();
    void *user_data = (void *)(intptr_t)request_id;
    // fill
    struct libusb_transfer *transfer = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(transfer, deviceHandle_, (1 | LIBUSB_ENDPOINT_OUT),
                              data, MSG_SIZE, &usbSendCallbackMultishot,
                              user_data, TIMEOUT);
    // submit
    r = libusb_submit_transfer(transfer);
    if (r != libusb_error::LIBUSB_SUCCESS)
    {
        libusb_free_transfer(transfer);
        transfer = nullptr;
        cout << usbDecodeError(r) << endl;
        throw std::runtime_error("");
    }
    transfers.push_back(transfer);
};

void CyberAtomUsb::usbRegisterReceiveCallback(void userDefinedCallback(unsigned char buffer[], int length) = nullptr)
{
    std::array<unsigned char, MSG_SIZE> t{};
    transfer_data.push_back(t);

    struct libusb_transfer *transfer = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(transfer, deviceHandle_, (1 | LIBUSB_ENDPOINT_IN),
                              transfer_data.back().data(), MSG_SIZE, &usbReceiveCallback,
                              (void *)userDefinedCallback, TIMEOUT);
    r = libusb_submit_transfer(transfer);
    if (r != libusb_error::LIBUSB_SUCCESS)
    {
        libusb_free_transfer(transfer);
        transfer = nullptr;
        cout << usbDecodeError(r) << endl;
        throw std::runtime_error("");
    }
    transfers.push_back(transfer);
};

void CyberAtomUsb::usbCancelAllTransfers()
{
    bool couldNotCancel = false;
    // sometimes canceling does not work the first time.
    for (int i = 0; i < transfers.size(); i++)
    {
        if (transfers[i] != nullptr)
        {
            int ret = libusb_cancel_transfer(transfers[i]);
            // this will trigger LIBUSB_TRANSFER_CANCELLED,
            //  to use libusb_free_transfer in callback functions
            if (ret != libusb_error::LIBUSB_SUCCESS)
            {
                if (DEBUG)
                {
                    cout << "Could not cancel transfer because " << usbDecodeError(ret) << endl;
                }
            }
        }
    }
    // use a maximum of 2 second to cancel all transfers
    for (int i = 0; i < 1000; ++i)
    {
        usbHandleEventsForMicroSec(1000);
        this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // clear pointers and buffers no longer needed/available
    transfers.clear();
    transfer_data.clear(); // with the transfers gone, there is no need to store their data anymore
};

void CyberAtomUsb::usbHandleEventsForMicroSec(int timeout_usec)
{
    struct timeval usec_timeval
    {
        .tv_usec = timeout_usec
    };
    // wait a maximum of 5s to cancel transfers
    int r = libusb_handle_events_timeout_completed(usbContext_, &usec_timeval, NULL);
    if (r != LIBUSB_SUCCESS)
    {
        cout << usbDecodeError(r) << endl;
        throw "";
    }
}

void CyberAtomUsb::usbHandleEventsForSec(int timeout_sec)
{
    struct timeval sec_timeval
    {
        .tv_usec = timeout_sec
    };
    // wait a maximum of 5s to cancel transfers
    int r = libusb_handle_events_timeout_completed(usbContext_, &sec_timeval, NULL);
    if (r != LIBUSB_SUCCESS)
    {
        cout << usbDecodeError(r) << endl;
        throw "";
    }
}

std::string CyberAtomUsb::usbDecodeError(int error)
{
    switch (error)
    {
    case libusb_error::LIBUSB_ERROR_IO:
        return ("LibUSB: Input/output error");
    case libusb_error::LIBUSB_ERROR_INVALID_PARAM:
        return ("LibUSB: Invalid parameter");
    case libusb_error::LIBUSB_ERROR_ACCESS:
        return ("LibUSB: Access denied (insufficient permissions)");
    case libusb_error::LIBUSB_ERROR_NO_DEVICE:
        return ("LibUSB: No such device (it may have been disconnected)");
    case libusb_error::LIBUSB_ERROR_NOT_FOUND:
        return ("LibUSB: Entity not found");
    case libusb_error::LIBUSB_ERROR_BUSY:
        return ("LibUSB: Resource busy");
    case libusb_error::LIBUSB_ERROR_TIMEOUT:
        return ("LibUSB: Operation timed out");
    case libusb_error::LIBUSB_ERROR_OVERFLOW:
        return ("LibUSB: Overflow");
    case libusb_error::LIBUSB_ERROR_PIPE:
        return ("LibUSB: Pipe error");
    case libusb_error::LIBUSB_ERROR_INTERRUPTED:
        return ("LibUSB: System call interrupted (perhaps due to signal)");
    case libusb_error::LIBUSB_ERROR_NO_MEM:
        return ("LibUSB: Insufficient memory");
    case libusb_error::LIBUSB_ERROR_NOT_SUPPORTED:
        return ("LibUSB: Operation not supported or unimplemented on this platform");
    case libusb_error::LIBUSB_ERROR_OTHER:
        return ("LibUSB: Other error");
    default:
        return ("LibUSB: Error unclear");
    }
};

std::string CyberAtomUsb::usbDecodeTransferStatus(int transfer_status)
{
    switch (transfer_status)
    {
    case libusb_transfer_status::LIBUSB_TRANSFER_COMPLETED:
        return ("LibUSB: Transfer completed without error. Note that this does not indicate that the entire amount of requested data was transferred.");
    case libusb_transfer_status::LIBUSB_TRANSFER_ERROR:
        return ("LibUSB: Transfer failed");
    case libusb_transfer_status::LIBUSB_TRANSFER_TIMED_OUT:
        return ("LibUSB: Transfer timed out");
    case libusb_transfer_status::LIBUSB_TRANSFER_CANCELLED:
        return ("LibUSB: Transfer was cancelled");
    case libusb_transfer_status::LIBUSB_TRANSFER_STALL:
        return ("LibUSB: For bulk/interrupt endpoints: halt condition detected (endpoint stalled). For control endpoints: control request not supported.");
    case libusb_transfer_status::LIBUSB_TRANSFER_NO_DEVICE:
        return ("LibUSB: Device was disconnected");
    case libusb_transfer_status::LIBUSB_TRANSFER_OVERFLOW:
        return ("LibUSB: Device sent more data than requested");
    default:
        return ("LibUSB: Transfer status unclear");
    }
};

} // namespace lib_cyberatom_x200