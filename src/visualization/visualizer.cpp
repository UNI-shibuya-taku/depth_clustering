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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_segment_cloud; // new
pcl::PointCloud<pcl::PointXYZRGB>::Ptr dynamic_or_static_cloud; // new
std_msgs::Header cloudHeader;
int count = 0;
ros::Publisher pub_color_cloud; // new

static vector<array<int, 3>> COLORS;

Visualizer::Visualizer(QWidget* parent)
    : QGLViewer(parent), AbstractClient<Cloud>(), _updated{false} {
  _cloud_obj_storer.SetUpdateListener(this);
    std::cout << "Visualizer::Visualizer!!!" << std::endl;
    sub_grid_map = nh.subscribe("/occupancy_grid", 10, &Visualizer::map_callback, this);
    sub_true_map = nh.subscribe("/true_grid", 10, &Visualizer::true_map_callback, this);
    sub_ground_cloud = nh.subscribe("/d_c_d_ground", 10, &Visualizer::ground_callback, this);
   // sub_ground_cloud = nh.subscribe("/velodyneo_clear", 10, &Visualizer::ground_callback, this);

    sub_imu_d_c_d = nh.subscribe("/d_c_d/imu/data", 10, &Visualizer::imu_callback, this);

    sub_dcd_cloud = nh.subscribe("/d_c_d_points", 10, &Visualizer::dcd_callback, this);

   // pub_test_map = nh.advertise<nav_msgs::OccupancyGrid>("/test_map", 1); // new
    pub_dynamic_cloud = nh.advertise<sensor_msgs::PointCloud2>("/depth_ver_dynamic_cloud", 1); // new
    pub_static_cloud = nh.advertise<sensor_msgs::PointCloud2>("/depth_ver_static_cloud", 1); // new
    pub_depth_obstacles_cloud = nh.advertise<sensor_msgs::PointCloud2>("/depth_ver_obstacles", 1);
    pub_time_depth = nh.advertise<nav_msgs::Odometry>("/time_depth", 1);
    pub_imu_depth = nh.advertise<sensor_msgs::Imu>("/imu_depth/data", 1);
    pub_gravity_point_cloud = nh.advertise<sensor_msgs::PointCloud2>("/gravity_point", 1); // new

    WIDTH = 40.0; // 幅 40.0
   // WIDTH = 50.0; // 幅 60.0
    RESOLUTION = 0.2; // 分能能 0.2
    GRID_WIDTH = WIDTH / RESOLUTION; // グリッドの幅
    GRID_NUM = GRID_WIDTH * GRID_WIDTH; // gridの個数
    WIDTH_2 = WIDTH / 2.0;
    GRID_WIDTH_2 = GRID_WIDTH / 2.0;
    thre_dynamic_cloud = 0.5; // 50%
    true_dynamic = false;
    true_static = false;
    std::cout << "all parameter is set!!!" << std::endl;
}
// d_c_dから来た物体点
void Visualizer::dcd_callback(const sensor_msgs::PointCloud2ConstPtr& msg_dcd){
   // ground_cloud.reset(new CloudXYZRGB);
   // dcd_cloud.reset(new CloudXYZRGB);
    dcd_cloud.reset(new CloudXYZIN);
    pcl::fromROSMsg(*msg_dcd, *dcd_cloud);
    record_stamp = dcd_cloud->header.stamp; // !!!!time_stamp受け取る用
   // std::cout << "dcd_cloud Header: " << dcd_cloud->header << std::endl;
   // std::cout << "ground_cloud_subscribe!!!" << std::endl;
}
void Visualizer::imu_callback(const sensor_msgs::ImuConstPtr& msg_imu){
    imu_depth = *msg_imu;
    imu_depth.header.frame_id = "map";
    pub_imu_depth.publish(imu_depth);
}
// d_c_dから来た地面点
void Visualizer::ground_callback(const sensor_msgs::PointCloud2ConstPtr& msg_ground){
   // ground_cloud.reset(new CloudXYZRGB);
    ground_cloud.reset(new CloudXYZIN);
    pcl::fromROSMsg(*msg_ground, *ground_cloud);
   // std::cout << "ground_cloud_subscribe!!!" << std::endl;
}
// 静動map獲得
void Visualizer::map_callback(const nav_msgs::OccupancyGridConstPtr& msg_map){
    map = *msg_map;
}
void Visualizer::true_map_callback(const nav_msgs::OccupancyGridConstPtr& msg_true_grid){
    true_map = *msg_true_grid;
}

void Visualizer::draw(){
  const double depth_start_time = ros::Time::now().toSec();
//  int argc;
//  char **argv;
  ros::spinOnce();
  const std::string this_frame_id = "base_link"; // ikuta.bag用
 // const std::string this_frame_id = "vehicle"; // semantic kitti様
  center_of_gravity_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>()); // 重心
  center_of_gravity_cloud->header.frame_id = this_frame_id;

  dynamic_or_static_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>()); // 一時おき
//  std::cout << "record_stamp_2:" << record_stamp << std::endl;
  dynamic_cloud.reset(new CloudXYZRGB);
  dynamic_cloud->header.frame_id = this_frame_id;
 // dynamic_cloud->header.frame_id = "vehicle"; // <- semantic kitti用
  dynamic_cloud->header.stamp = record_stamp; // dynamic_cloud_detectorのtime stampを受け継ぐ
  //  dynamic_cloud->header = dcd_cloud->header;

  static_cloud.reset(new CloudXYZRGB);
  static_cloud->header.frame_id = this_frame_id;
//  static_cloud->header.frame_id = "vehicle";
  static_cloud->header.stamp = record_stamp; // dynamic_cloud_detectorのtime stampを受け継ぐ
  //  static_cloud->header = dcd_cloud->header;

//    ground_cloud->header.frame_id = "vehicle";
  obstacles_cloud.reset(new CloudXYZRGB);
  obstacles_cloud->header.frame_id = this_frame_id;
 // obstacles_cloud->header.frame_id = "vehicle";
  obstacles_cloud->header.stamp = record_stamp; // dynamic_cloud_detectorのtime stampを受け継ぐ
  //  obstacles_cloud->header = dcd_cloud->header;
  //  CloudXYZINPtr lego_static_cloud(new CloudXYZIN);
  //  lego_static_cloud->header.frame_id = "vehicle";
  lock_guard<mutex> guard(_cloud_mutex);
  DrawCloud(_cloud); // 点群を表示
  int count_dynamic = 0;
  int count_static = 0;

  int count_dynamic_true = 0;
  int count_dynamic_false = 0;
  int count_static_true = 0;
  int count_static_false = 0;
  bool check_ground = false;

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

    // cluster内の点を随時取り出してpointへ
    // クラスタ内の点の話
    // このクラスタに該当するセルのカウント(ダブリがないようにする)
    // 格子地図内かどうかチェック
    for (const auto& point : cluster.points()){
      if(!is_valid_point(point.x(), point.y())){
          continue; // next
      }
      int index = get_index_from_xy(point.x(), point.y()); // いるセルを特定
      // indexが範囲内かどうかチェック
      if(index < 0 || GRID_NUM <= index){
          continue;
      }
      pcl::PointXYZRGB seg_point;
      seg_point.x = point.x();
      seg_point.y = point.y();
      seg_point.z = point.z();
      dynamic_or_static_cloud->push_back(seg_point); // 今探索しているクラスタの点群をここに保存

      // その点がどっちのセルに所属しているかそれぞれカウント
      // 同じセルを数え上げないように
      // indexチェック!!!
      bool check_d = false;
      bool check_s = false;
/*      if(map.data[index] < 20)
          pre_dynamic_index.push_back(index);
      else
          pre_static_index.push_back(index);*/
      if(map.data[index] < 20){ // map.dataに占有度が格納されてる
          // 被ったindexがないかどうかチェック
          for(int s = 0; s < pre_dynamic_index.size(); s++){
             // if(index == pre_dynamic_index[s]){ // もし既にそのindexを保存していたら
              if(pre_dynamic_index[s] == index){ // もし既にそのindexを保存していたら
                  check_d = true; // 今処理してるindexはダブリindex これ以上ダブリを探す必要なし
                  break;
              }
          }
          if(!check_d) // 全部のセルチェックしてダブリがなければ
               pre_dynamic_index.push_back(index); // そのクラスタの移動セルindex格納
      }
      else{
          // 被ったindexがないかどうかチェック
          for(int t = 0; t < pre_static_index.size(); t++){
            //  if(index == pre_static_index[t]){ // もし既にそのindexを保存していたら
              if(pre_static_index[t] == index){ // もし既にそのindexを保存していたら
                  check_s = true; // 今処理してるindexはダブリindex
                  break;
              }
          }
          if(!check_s)
              pre_static_index.push_back(index); // そのクラスタの静セルindex格納
      }
/*      if(dynamic_or_static_cloud->size() > 1000)
          continue;
*/
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
    // このクラスタの移動セルと静セルの個数
/*    if(dynamic_or_static_cloud -> size() < 30)
        continue;
*/

    double per_dynamic = 0.0;
    double thre_sig = 0.0;
    per_dynamic = calculate_thre(pre_dynamic_index.size(), pre_static_index.size()); // そのクラスタの移動セルの数の割合 移動セル/クラスタのセル
   //  std::cout <<" per_dynamic: " << per_dynamic << std::endl;
   // double thre_sig = calculate_thre_sig(count_dynamic, count_static); // 可変
    thre_sig = calculate_thre_sig(pre_dynamic_index.size(), pre_static_index.size()); // 可変閾値
  //  thre_sig = 0.3;
    int cell_size = pre_dynamic_index.size() + pre_static_index.size();
    if(isnan(per_dynamic)){
        per_dynamic = 0.0;
    }
    center /= cluster.size(); // クラスタの平均中心座標を算出
    if (min_point.x() < max_point.x()){
      extent = max_point - min_point;
    }
  //  float volume = extent.x() * extent.y() * extent.z(); // 体積

    if(per_dynamic > thre_sig){ // 移動クウウド決定 赤色点群 このクラスタのセル全て移動セル
       for(int i = 0; i < dynamic_or_static_cloud->size(); i++){
         int check_index = get_index_from_xy(dynamic_or_static_cloud->points[i].x, dynamic_or_static_cloud->points[i].y); // いるセルを特定
         pcl::PointXYZRGB moving_point;
         moving_point.x = dynamic_or_static_cloud->points[i].x;
         moving_point.y = dynamic_or_static_cloud->points[i].y;
         moving_point.z = dynamic_or_static_cloud->points[i].z;
         moving_point.r = 255;
         moving_point.g = 0;
         moving_point.b = 0;
        /* moving_point.r = 157;
         moving_point.g = 204;
         moving_point.b = 224;*/

         if(true_map.data[check_index] == 0)
             count_dynamic_true ++;
         else if(true_map.data[check_index] == 100)
             count_dynamic_false ++;
         dynamic_cloud->push_back(moving_point);
       }
  //      count_dynamic++; // 移動クラスタカウント
    }else if(per_dynamic <= thre_sig){ // 静クラウド決定 緑色点群 このクラスタのセル全て静セル
   // else if(per_dynamic <= thre_dynamic_cloud){ // 静クラウド決定 緑色点群 このクラスタのセル全て静セル
   // else{ // 実験用
       for(int j = 0; j < dynamic_or_static_cloud->size(); j++){
         int check_index = get_index_from_xy(dynamic_or_static_cloud->points[j].x, dynamic_or_static_cloud->points[j].y); // いるセルを特定
         pcl::PointXYZRGB static_point;
         static_point.x = dynamic_or_static_cloud->points[j].x;
         static_point.y = dynamic_or_static_cloud->points[j].y;
         static_point.z = dynamic_or_static_cloud->points[j].z;
         static_point.r = 0;
         static_point.g = 255;
         static_point.r = 0;

         // recall カウント
         if(true_map.data[check_index] == 100)
             count_static_true ++;
         else if(true_map.data[check_index] == 0)
             count_static_false ++;
         static_cloud->push_back(static_point);
       }
       check_ground = true; // !!!!!trueなら地面点群抜いてる！ slamのときに使うだけ
       if(!check_ground){
          for(int v = 0; v < ground_cloud->size(); v++){
             pcl::PointXYZRGB ground_point;
             PointXYZIN lego_ground_point;
             ground_point.x = ground_cloud->points[v].x;
             ground_point.y = ground_cloud->points[v].y;
             ground_point.z = ground_cloud->points[v].z;
             ground_point.r = 255;
             ground_point.g = 255;
             ground_point.b = 0;
             static_cloud->push_back(ground_point);
          }
          check_ground = true;
       }
//       count_static++; // 静止クラスタカウント
    }

    pre_static_index.clear();
    pre_dynamic_index.clear();
    dynamic_or_static_cloud->clear();

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
    pub_dynamic_cloud.publish(dynamic_cloud); // 赤
    pub_static_cloud.publish(static_cloud); // 緑
    static_cloud->clear();
    dynamic_cloud->clear();

    double recall_static = (double)count_static_true / ((double)count_static_true + (double)count_static_false);
    double recall_dynamic = (double)count_dynamic_true / ((double)count_dynamic_true + (double)count_dynamic_false);

    if(!isnan(recall_dynamic) && recall_dynamic != 0){
 //       std::cout <<" recall_dynamic: " << recall_dynamic * 100 << "%" << std::endl;
        time_depth.twist.twist.linear.x = recall_dynamic * 100;
    }
    if(isnan(recall_dynamic)){
        recall_dynamic = 0.0;
    }
    std::cout <<" recall_static : " << recall_static * 100  << "%" << std::endl;
    time_depth.twist.twist.linear.y = recall_static * 100;
    std::cout <<" recall_dynamic: " << recall_dynamic * 100 << "%" << std::endl;

/*    time_depth.pose.pose.position.y = dynamic_cloud->size();
    time_depth.pose.pose.position.z = static_cloud->size();
    time_depth.pose.pose.position.x = count_dynamic;
    time_depth.twist.twist.linear.z = ros::Time::now().toSec() - depth_start_time;*/

//    std::cout <<" depth_time: " << ros::Time::now().toSec() - depth_start_time << std::endl;
 //   std::cout <<" show time : : " << ros::Time::now().toSec() - depth_time << std::endl;

  pub_time_depth.publish(time_depth);
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

double Visualizer::calculate_thre(int count_dynamic, int count_static){
   double threshould = 0.0;
   threshould = (double)count_dynamic / ((double)count_dynamic + (double)count_static); // 移動セル/全体のセルの数
   return threshould;
}
double Visualizer::calculate_thre_sig(int count_dynamic, int count_static){
    double cell_sum = (double)count_dynamic + (double)count_static;
    double sig = 0.2 / (1.0 + exp(5.0 - 0.3*cell_sum));
  //  double sig = 0.1 / (1.0 + exp(5.0 - 0.3*cell_sum));
    double thre = 0.5 + sig;
    return thre;
}
int Visualizer::get_index_from_xy(const double x, const double y)
{
    int _x = floor(x / RESOLUTION + 0.5) + GRID_WIDTH_2;
    int _y = floor(y / RESOLUTION + 0.5) + GRID_WIDTH_2;
    return _y * GRID_WIDTH + _x;
}
int Visualizer::get_x_index_from_index(const int index)
{
    return index % GRID_WIDTH;
}
int Visualizer::get_y_index_from_index(const int index)
{
    return index / GRID_WIDTH;
}
double Visualizer::get_x_from_index(const int index)
{
    return (get_x_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}
double Visualizer::get_y_from_index(const int index)
{
    return (get_y_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}
bool Visualizer::is_valid_point(double x, double y)
{
    int index = get_index_from_xy(x, y);
    if(x < -WIDTH_2 || x > WIDTH_2 || y < -WIDTH_2 || y > WIDTH_2){
        return false;
    }else if(index < 0 || GRID_NUM <= index){
        return false;
    }else{
        return true;
    }
}
// 引数 クラスタの中心座標とextent(スケール?)
// クラスタ長方形の作成
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
