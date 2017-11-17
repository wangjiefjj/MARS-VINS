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

  MARS::Pose pose;
  mars_vins->GetPose(&pose);

  // Set the odometry msg.
  nav_msgs::OdometryPtr odom_msg_ptr(new nav_msgs::Odometry);
  odometry->header = cam0_img->header;
  odometry->header.frame_id = "map";
  odometry->child_frame_id = "odom";

  odometry->pose.pose.position.x = pose.global_P_imu[0];
  odometry->pose.pose.position.y = pose.global_P_imu[1];
  odometry->pose.pose.position.z = pose.global_P_imu[2];

  odometry->pose.pose.orientation.x = pose.imu_q_global[0];
  odometry->pose.pose.orientation.y = pose.imu_q_global[1];
  odometry->pose.pose.orientation.z = pose.imu_q_global[2];
  odometry->pose.pose.orientation.w = pose.imu_q_global[3];

  odom_pub->publish(odom_msg_ptr);

  return;
}

int main(int argc, char** argv) {

  // Initialize ros.
  ros::init(argc, argv, "~");
  ros::NodeHandle nh;

  // Get the path for calibration file and output file.
  std::string calib_file;
  std::string output_file;

  if (!nh.getParam("calib_file", calib_file))
    ROS_ERROR("Cannot get calibration file...");
  if (!nh.getParam("output_file", output_file))
    ROS_ERROR("Cannot get output file...");

  // Odometry publisher.
  ros::Publisher odom_pub;
  odom_pub = nh.advertise("odom", 40);

  // Initialize the ros wrapper for vins.
  std::unique_ptr<MARS::GenericDriver> driver =
    std::unique_ptr<MARS::GenericDriver>(new MARS::GenericDriver(nh));

  // Create the vins interface.
  MARS::MARSVinsFacade mars_vins(
      std::move(driver), calib_file.c_str(), output_file.c_str());

  // Stereo subscriber.
  // The subscriber callback is used to trigger the vins algorithm.
  message_filters::Subscriber<
    sensor_msgs::Image> cam0_img_sub;
  message_filters::Subscriber<
    sensor_msgs::Image> cam1_img_sub;
  message_filters::TimeSynchronizer<
    sensor_msgs::Image, sensor_msgs::Image> stereo_sub(10);

  cam0_img_sub.subscribe(nh, "dummy_cam0_image", 10);
  cam1_img_sub.subscribe(nh, "dummy_cam1_image", 10);
  stereo_sub.connectInput(cam0_img_sub, cam1_img_sub);
  stereo_sub.registerCallback(
      boost::bind(&GenericDriver::stereoCallback,
        this, _1, _2, &odom_pub, &mars_vins));

  ros::spin();

  return 0;
}
