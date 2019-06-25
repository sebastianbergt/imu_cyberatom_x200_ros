#include <gtest/gtest.h>
#include "gmock/gmock.h"
using ::testing::Return;
using ::testing::_;
using namespace testing;

struct libusb_context;
struct libusb_device;
struct libusb_device_handle;
typedef struct libusb_context libusb_context;
typedef struct libusb_device libusb_device;
typedef struct libusb_device_handle libusb_device_handle;

class libusb_MOCK{
public:
    virtual ~libusb_MOCK(){}

    MOCK_METHOD1(libusb_init, 
        int(libusb_context ** ctx));
    MOCK_METHOD2(libusb_set_debug, 
        void(libusb_context ** ctx, int level));
    MOCK_METHOD2(libusb_get_device_list, 
        ssize_t(libusb_context *ctx, libusb_device ***list) );
    MOCK_METHOD3(libusb_open_device_with_vid_pid, 
        libusb_device_handle*(libusb_context *ctx, uint16_t vendor_id, uint16_t product_id) );
    MOCK_METHOD2(libusb_free_device_list,
        void(libusb_device **list, int unref_devices) );
    MOCK_METHOD2(libusb_kernel_driver_active,
        int(libusb_device_handle *dev_handle, int interface_number) );
    MOCK_METHOD2(libusb_detach_kernel_driver,
        int(libusb_device_handle *dev_handle, int interface_number) );
    MOCK_METHOD2(libusb_claim_interface,
        int(libusb_device_handle *dev_handle, int interface_number) );
};

std::unique_ptr<libusb_MOCK> libusb_mock_;
int libusb_init(libusb_context ** ctx) 
{
    return libusb_mock_->libusb_init(ctx);
}
void libusb_set_debug(libusb_context ** ctx, int level) 
{libusb_mock_->libusb_set_debug(ctx, level);}
ssize_t libusb_get_device_list(libusb_context *ctx,	libusb_device ***list)
{return libusb_mock_->libusb_get_device_list(ctx, list);}
libusb_device_handle * libusb_open_device_with_vid_pid(libusb_context *ctx, uint16_t vendor_id, uint16_t product_id) 
{return libusb_mock_->libusb_open_device_with_vid_pid(ctx, vendor_id, product_id);}
void libusb_free_device_list(libusb_device **list, int unref_devices) 
{libusb_mock_->gmock_libusb_free_device_list(list, unref_devices);}
void libusb_kernel_driver_active(libusb_device_handle *dev_handle, int interface_number) 
{libusb_mock_->libusb_kernel_driver_active(dev_handle, interface_number);}
void libusb_detach_kernel_driver(libusb_device_handle *dev_handle, int interface_number) 
{libusb_mock_->libusb_detach_kernel_driver(dev_handle, interface_number);}
void libusb_claim_interface(libusb_device_handle *dev_handle, int interface_number) 
{libusb_mock_->libusb_claim_interface(dev_handle, interface_number);}

#define UNDER_TEST 1
#include "CyberAtomUsb.h"

using namespace lib_cyberatom_x200;

TEST(CyberAtomUsb, usbInit) {
    long long CONTEXT = 0xFEFE;
    long long DEVICE = 0xEFEF;
    long long SUCCESS = 42;
    libusb_context * context = (libusb_context *) &CONTEXT;
    libusb_device_handle * device_handle = (libusb_device_handle *) DEVICE;
    EXPECT_CALL(*libusb_mock_, libusb_init(_) )
        .Times(1)
        .WillOnce(Return(SUCCESS));
    EXPECT_CALL(*libusb_mock_, libusb_set_debug(_, 3) )
        .Times(1);
    EXPECT_CALL(*libusb_mock_, libusb_get_device_list(_, _))
        .Times(1)
        .WillOnce(Return(3));
    EXPECT_CALL(*libusb_mock_, libusb_open_device_with_vid_pid(_, CYBERATOM_VENDOR_ID, CYBERATOM_PRODUCT_ID))
        .Times(1)
        .WillOnce(Return(device_handle));
    EXPECT_CALL(*libusb_mock_, libusb_kernel_driver_active(device_handle, 0))
        .Times(1)
        .WillOnce(Return(1)); // is active
    EXPECT_CALL(*libusb_mock_, libusb_detach_kernel_driver(device_handle, 0))
        .Times(1)
        .WillOnce(Return(0)); // successfully detached
    EXPECT_CALL(*libusb_mock_, libusb_claim_interface(device_handle, 0))
        .Times(1)
        .WillOnce(Return(0));
    
    CyberAtomUsb imu;
    imu.usbInit(CYBERATOM_VENDOR_ID, CYBERATOM_PRODUCT_ID);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


