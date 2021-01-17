#include "./visualizer.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <limits>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


// 一応
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "clusterers/image_based_clusterer.h"

namespace depth_clustering {

using std::array;
using std::string;
using std::to_string;
using std::vector;

using std::lock_guard;
using std::map;
using std::mutex;
using std::string;
using std::thread;
using std::unordered_map;
using std::vector;

std_msgs::Header cloudHeader;
int count = 0;
ros::Publisher pub_color_cloud; // new

static vector<array<int, 3>> COLORS;

Visualizer::Visualizer(QWidget* parent)
    : QGLViewer(parent), AbstractClient<Cloud>(), _updated{false} {
  _cloud_obj_storer.SetUpdateListener(this);
    pub_gravity_point_cloud = nh.advertise<sensor_msgs::PointCloud2>("/gravity_point", 1); // new
    std::cout << "all parameter is set!!!" << std::endl;
}

void Visualizer::draw(){
  ros::spinOnce();
  const std::string this_frame_id = "base_link"; // ikuta.bag用
 // const std::string this_frame_id = "vehicle"; // semantic kitti様
  center_of_gravity_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>()); // 重心
  center_of_gravity_cloud->header.frame_id = this_frame_id;
  lock_guard<mutex> guard(_cloud_mutex);
  DrawCloud(_cloud); // 点群を表示

  // クラスタ単位の話
  // .object_clouds:クラスタ
  // 地面除去した点群をクラスタごとに分類 地面が混じる時もある
  for (const auto& kv : _cloud_obj_storer.object_clouds()) {
    std::vector<int> pre_dynamic_index;
    std::vector<int> pre_static_index;
    const auto& cluster = kv.second;
    Eigen::Vector3f center = Eigen::Vector3f::Zero(); // 3*1の0ベクトル
    Eigen::Vector3f extent = Eigen::Vector3f::Zero();
    Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(),
                              std::numeric_limits<float>::lowest(),
                              std::numeric_limits<float>::lowest()); //[lowest() lowest() lowest()]の転置
    Eigen::Vector3f min_point(std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max());

    for (const auto& point : cluster.points()){
      center = center + point.AsEigenVector();
      // 小さすぎ厳禁
      min_point << std::min(min_point.x(), point.x()),
          std::min(min_point.y(), point.y()),
          std::min(min_point.z(), point.z());
      // 大きすぎ厳禁
      max_point << std::max(max_point.x(), point.x()),
          std::max(max_point.y(), point.y()),
          std::max(max_point.z(), point.z());
    } // for 終了 このクラスタの全点処理終了 以降計算と静動devide

    // 重心publish
    pcl::PointXYZRGB gravity_point;
    gravity_point.x = center.x();
    gravity_point.y = center.y();
    gravity_point.z = center.z();
    gravity_point.r = 157;
    gravity_point.g = 204;
    gravity_point.b = 224;
    center_of_gravity_cloud->push_back(gravity_point);

    // ここで一つのクラスタの長方形計算
    DrawCube(center, extent); // このファイル内に関数あり
    // 次のクラスタ処理へ
  } // all clusters check
  pub_gravity_point_cloud.publish(center_of_gravity_cloud); // 重心publish
  center_of_gravity_cloud->clear();

}
void Visualizer::init() {
  setSceneRadius(100.0);
  camera()->showEntireScene();
  glDisable(GL_LIGHTING);
}
void Visualizer::DrawCloud(const Cloud& cloud) {
  glPushMatrix();
  glBegin(GL_POINTS);
  glColor3f(1.0f, 1.0f, 1.0f);
  for (const auto& point : cloud.points()) {
    glVertex3f(point.x(), point.y(), point.z());
  }
  glEnd();
  glPopMatrix();
}

void Visualizer::DrawCube(const Eigen::Vector3f& center,
                          const Eigen::Vector3f& scale) {
  glPushMatrix();
  glTranslatef(center.x(), center.y(), center.z());
  glScalef(scale.x(), scale.y(), scale.z());
  float volume = scale.x() * scale.y() * scale.z(); // 体積
  // 有効ボクセルは各辺の長さで判定
 // if (volume < 30.0f && scale.x() < 6 && scale.y() < 6 && scale.z() < 6) {
  // 赤色cubw
  if (volume < 30.0f && scale.x() < 6 && scale.y() < 6 && scale.z() < 6) {
    glColor3f(0.9f, 0.2f, 0.0f); // 色の調整
    glLineWidth(4.0f); // 線の太さ
  }
  // 無効ボクセル
  // 黒色cube
  else {
    // 大きいクラスタのcube
    glColor3f(0.3f, 0.3f, 0.3f); // 色の調整
    glLineWidth(1.0f); // 線の太さ
  }
  glBegin(GL_LINE_STRIP);

  // Bottom of Box
  glVertex3f(-0.5, -0.5, -0.5);
  glVertex3f(-0.5, -0.5, 0.5);
  glVertex3f(0.5, -0.5, 0.5);
  glVertex3f(0.5, -0.5, -0.5);
  glVertex3f(-0.5, -0.5, -0.5);

  // Top of Box
  glVertex3f(-0.5, 0.5, -0.5);
  glVertex3f(-0.5, 0.5, 0.5);
  glVertex3f(0.5, 0.5, 0.5);
  glVertex3f(0.5, 0.5, -0.5);
  glVertex3f(-0.5, 0.5, -0.5);

  glEnd();

  glBegin(GL_LINES);
  // For the Sides of the Box

  glVertex3f(-0.5, 0.5, -0.5);
  glVertex3f(-0.5, -0.5, -0.5);

  glVertex3f(-0.5, -0.5, 0.5);
  glVertex3f(-0.5, 0.5, 0.5);

  glVertex3f(0.5, -0.5, 0.5);
  glVertex3f(0.5, 0.5, 0.5);

  glVertex3f(0.5, -0.5, -0.5);
  glVertex3f(0.5, 0.5, -0.5);

  glEnd();
  glPopMatrix();
}

Visualizer::~Visualizer() {
}

void Visualizer::OnNewObjectReceived(const Cloud& cloud, const int) {
  lock_guard<mutex> guard(_cloud_mutex);
  _cloud = cloud;
}

void Visualizer::onUpdate() { this->update(); }

unordered_map<uint16_t, Cloud> ObjectPtrStorer::object_clouds() const {
  lock_guard<mutex> guard(_cluster_mutex);
  return _obj_clouds;
}

void ObjectPtrStorer::OnNewObjectReceived(
    const unordered_map<uint16_t, Cloud>& clouds, const int) {
  lock_guard<mutex> guard(_cluster_mutex);
  _obj_clouds = clouds;

  if (_update_listener) {
    _update_listener->onUpdate();
  }
}

}  // namespace depth_clustering
