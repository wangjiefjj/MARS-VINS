#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <core/ros_wrapper.h>
#include <core/mars-vins-facade.h>

void stereoCallback(
  const sensor_msgs::ImageConstPtr& cam0_img,
  const sensor_msgs::ImageConstPtr& cam1_img,
  ros::Publisher* odom_pub,
  MARS::MARSVinsFacade* mars_vins) {

  //Run mars vins.
  ROS_INFO("Run single image...");
  mars_vins->RunSingleImage();

  // Get pose from vins.
  MARS::Pose pose;
  ROS_INFO("Get pose...");
  mars_vins->GetPose(&pose);

  // Set the odometry msg.
  nav_msgs::OdometryPtr odom_msg_ptr(new nav_msgs::Odometry);
  odom_msg_ptr->header = cam0_img->header;
  odom_msg_ptr->header.frame_id = "map";
  odom_msg_ptr->child_frame_id = "odom";

  odom_msg_ptr->pose.pose.position.x = pose.global_P_imu[0];
  odom_msg_ptr->pose.pose.position.y = pose.global_P_imu[1];
  odom_msg_ptr->pose.pose.position.z = pose.global_P_imu[2];

  odom_msg_ptr->pose.pose.orientation.x = pose.imu_q_global[0];
  odom_msg_ptr->pose.pose.orientation.y = pose.imu_q_global[1];
  odom_msg_ptr->pose.pose.orientation.z = pose.imu_q_global[2];
  odom_msg_ptr->pose.pose.orientation.w = pose.imu_q_global[3];

  odom_pub->publish(odom_msg_ptr);

  return;
}

int main(int argc, char** argv) {

  // Initialize ros.
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  // Get the path for calibration file and output file.
  std::string calib_file;
  std::string output_file;

  if (!nh.getParam("calib_file", calib_file))
    ROS_ERROR("Cannot get calibration file...");
  if (!nh.getParam("output_file", output_file))
    ROS_ERROR("Cannot get output file...");

  // Odometry publisher.
  ROS_INFO("Initialize odom publisher...");
  ros::Publisher odom_pub;
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 40);

  // Initialize the ros wrapper for vins.
  ROS_INFO("Initialize ros wrapper...");
  std::unique_ptr<MARS::GenericDriver> driver =
    std::unique_ptr<MARS::GenericDriver>(new MARS::GenericDriver(nh));

  // Create the vins interface.
  ROS_INFO("Initialize mars vins...");
  MARS::MARSVinsFacade mars_vins(
      std::move(driver), calib_file.c_str(), output_file.c_str());

  // Stereo subscriber.
  // The subscriber callback is used to trigger the vins algorithm.
  ROS_INFO("Initialize stereo subscriber...");
  message_filters::Subscriber<
    sensor_msgs::Image> cam0_img_sub;
  message_filters::Subscriber<
    sensor_msgs::Image> cam1_img_sub;
  message_filters::TimeSynchronizer<
    sensor_msgs::Image, sensor_msgs::Image> stereo_sub(10);

  cam0_img_sub.subscribe(nh, "dummy_cam0_image", 10);
  cam1_img_sub.subscribe(nh, "dummy_cam1_image", 10);
  stereo_sub.connectInput(cam0_img_sub, cam1_img_sub);
  stereo_sub.registerCallback(boost::bind(
        stereoCallback, _1, _2, &odom_pub, &mars_vins));

  ROS_INFO("Start ros spin...");
  ros::spin();

  return 0;
}
