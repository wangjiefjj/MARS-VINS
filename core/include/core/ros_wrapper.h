#ifndef ROS_WRAPPER_H
#define ROS_WRAPPER_H

#include <vector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

namespace MARS {

struct Generic_Image {
  unsigned char * data;
  int height;
  int width;
  int stride;
  double timestamp;
};

struct Generic_IMU {
  double accelX;
  double accelY;
  double accelZ;

  double gyroX;
  double gyroY;
  double gyroZ;

  double timestamp;
};

class GenericDriver {
public:
  GenericDriver(const ros::NodeHandle& n);
  GenericDriver(const GenericDriver&) = delete;
  GenericDriver operator=(const GenericDriver&) = delete;

  // Interface required by the mars_core library.
  void startCamera(int cam_id);
  void getImage(struct Generic_Image* img, int cam_id);

  void startIMU();
  void getIMU(Generic_IMU* imu);


  //typedef boost::shared_ptr<GenericDriver> Ptr;
  //typedef boost::shared_ptr<const GenericDriver> ConstPtr;

private:

  // Imu message callback.
  void imuCallback(
      const sensor_msgs::ImuConstPtr& msg);

  // Stereo image callback.
  void stereoCallback(
      const sensor_msgs::ImageConstPtr& cam0_img,
      const sensor_msgs::ImageConstPtr& cam1_img);


  // Imu message buffer.
  std::vector<sensor_msgs::Imu> imu_msg_buffer;

  // Latest stereo images.
  cv_bridge::CvImageConstPtr cam0_img_ptr;
  cv_bridge::CvImageConstPtr cam1_img_ptr;

  // Ros node handle.
  ros::NodeHandle nh;

  // Subscribers
  message_filters::Subscriber<
    sensor_msgs::Image> cam0_img_sub;
  message_filters::Subscriber<
    sensor_msgs::Image> cam1_img_sub;
  message_filters::TimeSynchronizer<
    sensor_msgs::Image, sensor_msgs::Image> stereo_sub;
  ros::Subscriber imu_sub;

};

//typedef GenericDriver::Ptr GenericDriverPtr;
//typedef GenericDriver::ConstPtr GenericDriverConstPtr;

} // end namespace MARS

#endif
