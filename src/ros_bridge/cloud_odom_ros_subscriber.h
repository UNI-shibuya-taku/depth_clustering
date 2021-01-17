#ifndef SRC_ROS_BRIDGE_CLOUD_ODOM_ROS_SUBSCRIBER_H_
#define SRC_ROS_BRIDGE_CLOUD_ODOM_ROS_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <string>
#include <map>

#include "communication/abstract_sender.h"
#include "utils/pose.h"
#include "utils/cloud.h"
#include "utils/useful_typedefs.h"

namespace depth_clustering {

/**
 * @brief      Class for cloud odom ros subscriber.
 */
class CloudOdomRosSubscriber : public AbstractSender<Cloud> {
  using PointCloudT = sensor_msgs::PointCloud2;
  using OdometryT = nav_msgs::Odometry;
  using ApproximateTimePolicy =
      message_filters::sync_policies::ApproximateTime<PointCloudT, OdometryT>;

 public:
  CloudOdomRosSubscriber(ros::NodeHandle* node_handle,
                         const ProjectionParams& params,
                         const std::string& topic_clouds,
                         const std::string& topic_odom = "");
  virtual ~CloudOdomRosSubscriber() {
    delete _subscriber_odom;
    delete _subscriber_clouds;
    delete _sync;
  }

 // ros::Publisher pub_color_cloud = node_handle.advertise<sensor_msgs::PointCloud2>("/segment_color_cloud", 1); // new

  /**
   * @brief      Get synchronized odometry and cloud
   *
   * @param[in]  msg_cloud  The message cloud
   * @param[in]  msg_odom   The message odom
   */
  void Callback(const PointCloudT::ConstPtr& msg_cloud,
                const OdometryT::ConstPtr& msg_odom);

  /**
   * @brief      Get point cloud from ROS
   *
   * @param[in]  msg_cloud  The message cloud
   */
  void CallbackVelodyne(const sensor_msgs::PointCloud2::ConstPtr& msg_cloud);

  /**
   * @brief      Starts listening to ros.
   */
  void StartListeningToRos();

 protected:
  Pose RosOdomToPose(const OdometryT::ConstPtr& msg);
  Cloud::Ptr RosCloudToCloud(const PointCloudT::ConstPtr& msg);

  ros::NodeHandle* _node_handle;

  message_filters::Subscriber<PointCloudT>* _subscriber_clouds;
  // ros::Subscriber _subscriber_clouds = node_handle.subscribe("~", 1, callback);
  message_filters::Subscriber<OdometryT>* _subscriber_odom;
  message_filters::Synchronizer<ApproximateTimePolicy>* _sync;
  std::string _topic_clouds;
  std::string _topic_odom;

  ProjectionParams _params;

  int _msg_queue_size;
};

}  // namespace depth_clustering

#endif  // SRC_ROS_BRIDGE_CLOUD_ODOM_ROS_SUBSCRIBER_H_
