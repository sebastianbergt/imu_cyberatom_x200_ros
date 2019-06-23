#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"

sensor_msgs::MagneticField mag_msg_;
sensor_msgs::Imu imu_msg_;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cyberatom-x200");
  ros::NodeHandle n;
  // advertise
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1);
  ros::Publisher mag_pub = n.advertise<sensor_msgs::MagneticField>("mag", 1);
  ros::Publisher tmp_pub = n.advertise<sensor_msgs::Temperature>("tmp", 1);

  ros::Rate loop_rate(100); //Hz
  while (ros::ok())
  {
    sensor_msgs::Imu imu_msg_;
    sensor_msgs::MagneticField mag_msg_;
    sensor_msgs::Temperature tmp_msg_;

    ROS_INFO("Received new imu sample");
    ros::Time currentTime = ros::Time::now();
    imu_msg_.header.stamp = currentTime;
    imu_msg_.linear_acceleration.x = 0.0;
    imu_msg_.linear_acceleration.y = 0.0;
    imu_msg_.linear_acceleration.z = 0.0;
    imu_msg_.angular_velocity.x = 0.0;
    imu_msg_.angular_velocity.y = 0.0;
    imu_msg_.angular_velocity.z = 0.0;
    imu_msg_.orientation.x = 0.0;
    imu_msg_.orientation.y = 0.0;
    imu_msg_.orientation.z = 0.0;
    imu_msg_.orientation.w = 0.0;

    mag_msg_.header.stamp = current_time;
    mag_msg_.magnetic_field.x = 0.0;
    mag_msg_.magnetic_field.y = 0.0;
    mag_msg_.magnetic_field.z = 0.0;

    tmp_msg_.header.stamp = current_time;
    tmp_msg_.temperature = 0.0;
    tmp_msg_.variance = 0; //is interpreted as variance unknown

    imu_pub.publish(imu_msg_);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}