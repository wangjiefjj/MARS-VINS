#include <core/ros_wrapper.h>

namespace MARS {

GenericDriver::GenericDriver(const ros::NodeHandle& n):
  nh(n), stereo_sub(10) {

  // Create subscribers.
  imu_sub = nh.subscribe("imu", 200, &GenericDriver::imuCallback, this);

  cam0_img_sub.subscribe(nh, "cam0_image", 10);
  cam1_img_sub.subscribe(nh, "cam1_image", 10);

  stereo_sub.connectInput(cam0_img_sub, cam1_img_sub);
  stereo_sub.registerCallback(&GenericDriver::stereoCallback, this);

  return;
}

void GenericDriver::startCamera(int cam_id) {
  return;
}

void GenericDriver::getImage(
    struct Generic_Image* img, int cam_id) {

  img->height = 800;
  img->width = 960;
  img->stride = img->width;

  free(img->data);
  img->data = (unsigned char*) malloc(
      img->width * img->height * sizeof(unsigned char));
  cv::Mat cv_img(img->height, img->width, CV_8UC1, img->data);

  if (cam_id == 0) {

    img->timestamp = cam0_img_ptr->header.stamp.toSec();
    cam0_img_ptr->image.copyTo(cv_img);

  } else if (cam_id == 1) {

    img->timestamp = cam1_img_ptr->header.stamp.toSec();
    cam1_img_ptr->image.copyTo(cv_img);

  } else {
    ROS_ERROR("Cannot find camera %d...", cam_id);
  }

  return;
}

void GenericDriver::startIMU() {
  return;
}

void GenericDriver::getIMU(Generic_IMU* imu) {

  // Get the oldest IMU msg.
  const sensor_msgs::Imu& imu_msg = imu_msg_buffer[0];

  // Set the imu data.
  imu->timestamp = imu_msg.header.stamp.toSec();

  imu->gyroX = imu_msg.angular_velocity.x;
  imu->gyroY = imu_msg.angular_velocity.y;
  imu->gyroZ = imu_msg.angular_velocity.z;

  imu->accelX = imu_msg.linear_acceleration.x;
  imu->accelY = imu_msg.linear_acceleration.y;
  imu->accelZ = imu_msg.linear_acceleration.z;

  // Remove the used imu msg.
  imu_msg_buffer.erase(imu_msg_buffer.begin());

  return;
}

void GenericDriver::imuCallback(
    const sensor_msgs::ImuConstPtr& msg) {

  // Save the imu msgs into a buffer.
  imu_msg_buffer.push_back(*msg);

  return;
}

void GenericDriver::stereoCallback(
    const sensor_msgs::ImageConstPtr& cam0_img,
    const sensor_msgs::ImageConstPtr& cam1_img) {

  // Save the latest stereo images.
  cam0_img_ptr = cv_bridge::toCvShare(cam0_img,
      sensor_msgs::image_encodings::MONO8);
  cam1_img_ptr = cv_bridge::toCvShare(cam1_img,
      sensor_msgs::image_encodings::MONO8);

  return;
}


} // end namespace MARS
