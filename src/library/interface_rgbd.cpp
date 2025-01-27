#include <cv_bridge/cv_bridge.h>

#include "orb_slam_2_ros/interface_rgbd.hpp"

namespace orb_slam_2_interface {

OrbSlam2InterfaceRGBD::OrbSlam2InterfaceRGBD(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private,
                                             const bool visualization)
    : OrbSlam2Interface(nh, nh_private) {
  // Getting data and params
  nh_private.param("camera_name", camera_name, std::string("camera"));
  subscribeToTopics();
  //advertiseTopics();
  //getParametersFromRos();
  slam_system_ = std::shared_ptr<ORB_SLAM2::System>(
      new ORB_SLAM2::System(vocabulary_file_path_, settings_file_path_,
                            ORB_SLAM2::System::RGBD, visualization));
}

void OrbSlam2InterfaceRGBD::subscribeToTopics() {
  // Subscribing to the stereo images
  rgb_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
      new message_filters::Subscriber<sensor_msgs::Image>(
          nh_, camera_name + "/rgb/image_raw", 1));
  depth_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
      new message_filters::Subscriber<sensor_msgs::Image>(
          nh_, camera_name + "/depth_registered/image_raw", 1));
  // Creating a synchronizer
  sync_ = std::shared_ptr<message_filters::Synchronizer<sync_pol>>(
      new message_filters::Synchronizer<sync_pol>(sync_pol(10), *rgb_sub_,
                                                  *depth_sub_));
  // Registering the synchronized image callback
  sync_->registerCallback(
      boost::bind(&OrbSlam2InterfaceRGBD::rgbdImageCallback, this, _1, _2));
}

void OrbSlam2InterfaceRGBD::rgbdImageCallback(
    const sensor_msgs::ImageConstPtr& msg_rgb,
    const sensor_msgs::ImageConstPtr& msg_depth) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr_rgb, cv_ptr_depth;
  try {
    cv_ptr_rgb = cv_bridge::toCvShare(msg_rgb);
    cv_ptr_depth = cv_bridge::toCvShare(msg_depth);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // Handing the image to ORB slam for tracking
  cv::Mat T_C_W_opencv =
      slam_system_->TrackRGBD(cv_ptr_rgb->image, cv_ptr_depth->image,
                                cv_ptr_rgb->header.stamp.toSec());
  // If tracking successfull
  if (!T_C_W_opencv.empty()) {
    // Converting to kindr transform and publishing
    Transformation T_C_W, T_W_C;
    convertOrbSlamPoseToKindr(T_C_W_opencv, &T_C_W);
    T_W_C = T_C_W.inverse();
    publishCurrentPose(T_W_C, msg_rgb->header);
    // Saving the transform to the member for publishing as a TF
    T_W_C_ = T_W_C;
  }

}

}  // namespace orb_slam_2_interface
