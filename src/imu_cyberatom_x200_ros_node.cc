#include <exception>
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/Imu.h"
#include "CyberAtomUsb.h"

sensor_msgs::Imu imu_msg_{};
ros::Publisher imu_pub;
unsigned int callbackCounter = 0;

using namespace lib_cyberatom_x200;

void imuCallback(unsigned char buffer[], int length)
{
  callbackCounter++;
  X200DataConverter conv;
  switch (buffer[0])
  {
  case Response::QUAT_DATA:
  {
    std::array<float, 4> q = conv.quaternion(buffer, length);
    imu_msg_.orientation.x = q[0];
    imu_msg_.orientation.y = q[1];
    imu_msg_.orientation.z = q[2];
    imu_msg_.orientation.w = q[3];
    ROS_INFO("Received quaternion = (%f, %f, %f, %f)", q[0], q[1], q[2], q[3]);
    break;
  }
  case Response::ROT_RATE_DATA:
  {
    std::array<float, 3> r = conv.rotationRate(buffer, length);
    imu_msg_.header.stamp = ros::Time::now();
    imu_msg_.angular_velocity.x = r[0];
    imu_msg_.angular_velocity.y = r[1];
    imu_msg_.angular_velocity.z = r[2];
    ROS_INFO("Received rotationRate = (%f, %f, %f)", r[0], r[1], r[2]);
    imu_pub.publish(imu_msg_);
    ROS_INFO("Published sensor_msgs::Imu");
    break;
  }
  case Response::CALIB_ACC:
  {
    std::array<float, 3> a = conv.acceleration(buffer, length);
    imu_msg_.linear_acceleration.x = a[0];
    imu_msg_.linear_acceleration.y = a[1];
    imu_msg_.linear_acceleration.z = a[2];
    ROS_INFO("Received acceleration = (%f, %f, %f)", a[0], a[1], a[2]);
    break;
  }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_cyberatom_x200_ros");
  ros::NodeHandle n;
  imu_pub = n.advertise<sensor_msgs::Imu>("imu", 100);
  
  while (ros::ok())
  {
    try {
      ROS_INFO("Initializing IMU.");
      CyberAtomUsb imu;
      imu.usbInit(CYBERATOM_VENDOR_ID, CYBERATOM_PRODUCT_ID);
      imu.usbRegisterReceiveCallback(imuCallback);
      imu.usbSendMultishot(Request::GET_QUAT_DATA);
      imu.usbSendMultishot(Request::GET_ROT_RATE_DATA);
      imu.usbSendMultishot(Request::GET_CALIB_ACC);
      while (ros::ok()) {
        imu.usbHandleEventsForSec(1);
      }
    }
    catch(std::exception &e)
    {
      ROS_ERROR("Exception occured: %s", e.what());
    }
  }
  return 0;
}