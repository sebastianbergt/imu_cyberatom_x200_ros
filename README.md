# imu_cyberatom_x200_ros
is a simple ROS node implementation publishing IMU data received from [CyberAtom X200](https://cyberatom.eu/ahrs/x-200). 
Under the hood there is a ROS-independent library which utilizes libusb to communicate via USB with the physical device.
It does NOT require any other third party dependencies, especially not the closed-source binaries available from the device vendor.

# Yet another interface library
You may ask why this library was created if device vendors libraries are available.
As of now CyberAtom only offers raspbian, linux x86_64 and windows binaries. 
However I needed arm64 binaries for using the X200 with a Nvidia Jetson Nano.

# Contribute
As of now, the library is only capable of reading the IMU relevant sensor information.
Improvements are always welcome. Please feel free to send me a pull request or contact me if you need help.
Independent code reviews are highly appreciated too.

# Performance
On my AMD Ryzen 3 2200G machine I was able to achieve asynchronously receiving the three messages QUAT_DATA, ROT_RATE_DATA and CALIB_ACC at roughly 180Hz. Before I tried with the synchronous API and scored significantly lower rates at around 40Hz.

# Tested with
clang 8.0.0 on ubuntu 18.04 x86_64

# Sidenote
Updating the CyberAtom X200 firmware from 1.2 to 1.3 did not work through CyberStudio when I tried with Windows 10 or Linux Ubuntu 16.04.
After some trial and error I was successful with Device Firmware recovery described in their excellent [manual](https://cyberatom.eu/download/documents/xhtml/X-200/topics/about.html) when using [UART](https://cyberatom.eu/download/documents/xhtml/X-200/topics/uart-interface.html) and starting in [bootloader mode](https://cyberatom.eu/download/documents/xhtml/X-200/tasks/recovering-device-firmware.html).