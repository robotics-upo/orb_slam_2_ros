#ifndef ORB_SLAM_2_INTERFACE_RGBD
#define ORB_SLAM_2_INTERFACE_RGBD

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "orb_slam_2_ros/interface.hpp"
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

namespace orb_slam_2_interface {
// The synchronization policy used by the interface to sync stereo images
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image>
    sync_pol;
// Class handling global alignment calculation and publishing
class OrbSlam2InterfaceRGBD : public OrbSlam2Interface {
 public:
  // Constructor
  OrbSlam2InterfaceRGBD(const ros::NodeHandle& nh,
                        const ros::NodeHandle& nh_private,
                        const bool visualization);

 protected:
  // Subscribes to the appropriate ROS topics
  void subscribeToTopics();

  // Subscribers
  // Callbacks
  void rgbdImageCallback(const sensor_msgs::ImageConstPtr& msg_rgb,
                           const sensor_msgs::ImageConstPtr& msg_depth);

  // Subscribers
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> rgb_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<sync_pol>> sync_;
  
  // Camera name
  std::string camera_name;
};

}  // namespace orb_slam_2_interface

#endif /* ORB_SLAM_2_INTERFACE_RGBD */
