// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// GNU-GPL licence that follows one of libQGLViewer.

#ifndef SRC_VISUALIZATION_VISUALIZER_H_
#define SRC_VISUALIZATION_VISUALIZER_H_
#include "ros/ros.h"
#include <QGLViewer/qglviewer.h>

#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <math.h>

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

#include "communication/abstract_client.h"
#include "utils/cloud.h"
#include "utils/useful_typedefs.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_types_conversion.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/passthrough.h>
namespace depth_clustering {

class IUpdateListener {
 public:
  virtual void onUpdate() = 0;
};

class ObjectPtrStorer
    : public AbstractClient<std::unordered_map<uint16_t, Cloud>> {
 public:
  ObjectPtrStorer() : AbstractClient<std::unordered_map<uint16_t, Cloud>>() {}

  void OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds,
                           const int id) override;

  void SetUpdateListener(IUpdateListener* update_listener) {
    _update_listener = update_listener;
  }

  virtual ~ObjectPtrStorer() {}

  std::unordered_map<uint16_t, Cloud> object_clouds() const;

 private:
  std::unordered_map<uint16_t, Cloud> _obj_clouds;
  IUpdateListener* _update_listener;
  mutable std::mutex _cluster_mutex;
};

/**
 * @brief      An OpenGl visualizer that shows data that is subscribes to.
 */
class Visualizer : public QGLViewer,
                   public AbstractClient<Cloud>,
                   public IUpdateListener {
 public:
  explicit Visualizer(QWidget* parent = 0);
  virtual ~Visualizer();

  void OnNewObjectReceived(const Cloud& cloud, const int id) override;

  void ground_callback(const sensor_msgs::PointCloud2ConstPtr&);
  void map_callback(const nav_msgs::OccupancyGridConstPtr&);
  void true_map_callback(const nav_msgs::OccupancyGridConstPtr&);
  void onUpdate() override;
  int get_index_from_xy(const double, const double);
  int get_x_index_from_index(const int);
  int get_y_index_from_index(const int);
  double get_x_from_index(const int);
  double get_y_from_index(const int);
  bool is_valid_point(double, double);
  double calculate_thre(int, int);
  double calculate_thre_sig(int, int);

  void dcd_callback(const sensor_msgs::PointCloud2ConstPtr&);

  void semantic_dynamic_callback(const sensor_msgs::PointCloud2ConstPtr&);
  void semantic_static_callback(const sensor_msgs::PointCloud2ConstPtr&);
  void imu_callback(const sensor_msgs::ImuConstPtr&);

  void copy_header(void);
  typedef pcl::PointXYZRGB PointXYZRGB;
  typedef pcl::PointCloud<PointXYZRGB> CloudXYZRGB;
  typedef pcl::PointCloud<PointXYZRGB>::Ptr CloudXYZRGBPtr;

  typedef pcl::PointXYZINormal PointXYZIN;
  typedef pcl::PointCloud<PointXYZIN> CloudXYZIN;
  typedef pcl::PointCloud<PointXYZIN>::Ptr CloudXYZINPtr;

  ObjectPtrStorer* object_clouds_client() { return &_cloud_obj_storer; }

 protected:
  void draw() override;
  void init() override;

 private:
  void DrawCloud(const Cloud& cloud);
  void DrawCube(const Eigen::Vector3f& center, const Eigen::Vector3f& scale);


  bool _updated;
  ObjectPtrStorer _cloud_obj_storer;
  Cloud _cloud;
  mutable std::mutex _cloud_mutex;

  ros::NodeHandle nh;
  ros::Publisher pub_test_map; // new
  nav_msgs::OccupancyGrid map;//map情報取得用
  nav_msgs::OccupancyGrid true_map;//map情報取得用
  ros::Publisher pub_dynamic_cloud; // new
  ros::Publisher pub_static_cloud; // new
  ros::Publisher pub_depth_obstacles_cloud; // new
  ros::Publisher pub_time_depth; // new
  ros::Publisher pub_dcd; // new

  nav_msgs::Odometry time_depth;
  sensor_msgs::Imu imu_depth;

  ros::Subscriber sub_true_dynamic;
  ros::Subscriber sub_true_static;
  ros::Subscriber sub_true_map;
  ros::Subscriber sub_ground_cloud;
  ros::Subscriber sub_dcd_cloud;
  ros::Subscriber sub_grid_map;

  ros::Publisher pub_imu_depth; // new
  ros::Subscriber sub_imu_d_c_d;
  ros::Publisher pub_gravity_point_cloud; // new

/*  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dynamic_cloud; // new
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr static_cloud; // new
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacles_cloud; // new
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr true_dynamic_cloud; // new
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr true_static_cloud; // new*/

  CloudXYZRGBPtr dynamic_cloud; // new
  CloudXYZRGBPtr static_cloud; // new
  CloudXYZRGBPtr obstacles_cloud; // new
  CloudXYZINPtr ground_cloud; // new
 // pcl::PointCloud<pcl::PointXYZRGB>::Ptr dcd_cloud; // new
  CloudXYZINPtr dcd_cloud; // new
  CloudXYZRGBPtr center_of_gravity_cloud; // new

 // CloudXYZRGBPtr ground_cloud; // new

  double RESOLUTION;
  double WIDTH;
  double WIDTH_2;
  int GRID_WIDTH;
  int GRID_WIDTH_2;
  int GRID_NUM;
  double OCCUPANCY_THRESHOLD;
  int BEAM_NUM;
  double thre_dynamic_cloud;

  double count_true;
  double accuracy_dynamic;
  double accuracy_static;

  bool true_dynamic;
  bool true_static;
  double record_stamp; // これでtime stampを継承しておかないとlego loamが動かない
};

}  // namespace depth_clustering

#endif  // SRC_VISUALIZATION_VISUALIZER_H_
