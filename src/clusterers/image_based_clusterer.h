// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#ifndef SRC_CLUSTERERS_IMAGE_BASED_CLUSTERER_H_
#define SRC_CLUSTERERS_IMAGE_BASED_CLUSTERER_H_

#include <chrono>
#include <ctime>
#include <map>
#include <opencv/cv.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "utils/cloud.h"
#include "utils/radians.h"
#include "utils/timer.h"
#include "utils/useful_typedefs.h"
#include <ros/ros.h>
#include "clusterers/abstract_clusterer.h"
#include "image_labelers/diff_helpers/diff_factory.h"
#include "image_labelers/linear_image_labeler.h"
#include "projections/cloud_projection.h"

//#include "clusterers/file.h"
#include <stdio.h>
#include <stdlib.h>
#include "nav_msgs/Odometry.h"
#include <pcl_ros/point_cloud.h> // 必要 これがないとpublishの部分でエラーになる
#include <iostream>
#include <fstream>
namespace depth_clustering {

/**
 * @brief      Class for image based clusterer.
 *
 * @tparam     LabelerT  A Labeler class to be used for labeling.
 */
template <typename LabelerT>
class ImageBasedClusterer : public AbstractClusterer {
 public:
    FILE *fp;
  using Receiver = AbstractClient<Cloud>;
  using Sender = AbstractSender<std::unordered_map<uint16_t, Cloud>>;

 //   ofs_2("ofs_2.csv");
  uint64_t time_pub;
//  ros::NodeHandle nh;
//  ros::Publisher pub_time;
 /* void ros_publish(int argc, char** argv){
    ros::init(argc,argv,"depth_clustering_shibuya");
    ros::NodeHandle nh;
    ros::Publisher pub_time;
    pub_time = nh.advertise<nav_msgs::Odometry>("/time_depth_2", 1);
   // std::cout << "pub_time: " << time_pub << std::endl;
  }*/
  /**
   * @brief      Construct an image-based clusterer.
   *
   * @param[in]  angle_tollerance  The angle tollerance to separate objects
   * @param[in]  min_cluster_size  The minimum cluster size to send
   * @param[in]  max_cluster_size  The maximum cluster size to send
   */
  explicit ImageBasedClusterer(Radians angle_tollerance = 8_deg,
                               uint16_t min_cluster_size = 100,
                               uint16_t max_cluster_size = 25000)
      : AbstractClusterer(0.0, min_cluster_size, max_cluster_size),
        _counter(0),
        _angle_tollerance(angle_tollerance),
        _label_client{nullptr} {}

  virtual ~ImageBasedClusterer() {}

  /**
   * @brief      Sets the difference type.
   *
   * @param[in]  diff_type  The difference type
   */
  void SetDiffType(DiffFactory::DiffType diff_type) { _diff_type = diff_type; }

  /**
   * @brief      Sets the label image client.
   *
   * @param      client  The client to receive color images with labels
   */
  void SetLabelImageClient(AbstractClient<cv::Mat>* client) {
    this->_label_client = client;
  }

  /**
   * @brief      Gets called when clusterer receives a cloud to cluster
   *
   * @param[in]  cloud      The cloud to cluster
   * @param[in]  sender_id  The sender identifier
   */
  void OnNewObjectReceived(const Cloud& cloud, int) override {
    // generate a projection from a point cloud
 //   fp = fopen("/src/test2.csv/depth_clustering", "w");
  int argc;
  char** argv;
//  std::ofstream ofs("ofs3.csv");
    uint64_t count_time;
  //  ros::Publisher pub_depth = nh.advertise<nav_msgs::Odometry>("/depth_time", 10);
;
 //   ros_publish(argc, argv);
    if (!cloud.projection_ptr()) {
      fprintf(stderr, "ERROR: projection not initialized in cloud.\n");
      fprintf(stderr, "INFO: cannot label this cloud.\n");
      return;
    }
    time_utils::Timer timer;
    LabelerT image_labeler(cloud.projection_ptr()->depth_image(),
                           cloud.projection_ptr()->params(), _angle_tollerance);
    image_labeler.ComputeLabels(_diff_type);
    const cv::Mat* labels_ptr = image_labeler.GetLabelImage();
    uint64_t count_image;
    fprintf(stderr, "INFO: image based labeling took: %lu us\n",
           count_image =  timer.measure());
  //  count_time += timer.measure();
 //   fprintf(fp, "%lu \n",count_image);

    // send image to whoever wants to get it
    if (_label_client) {
      _label_client->OnNewObjectReceived(*labels_ptr, this->id());
    }

    uint64_t count_sent;
    fprintf(stderr, "INFO: labels image sent to clients in: %lu us\n",
           count_sent =  timer.measure());
  //  fprintf(fp, "%lu \n",count_sent);
   // count_time += timer.measure();

    // create 3d clusters from image labels
    // 3次元点群を画像ラベルから生成
    std::unordered_map<uint16_t, Cloud> clusters;
    // 全ラベル探索
    for (int row = 0; row < labels_ptr->rows; ++row) {
      for (int col = 0; col < labels_ptr->cols; ++col) {
        const auto& point_container = cloud.projection_ptr()->at(row, col);
        // point_containerが空っぽ
        if (point_container.IsEmpty()) {
          // this is ok, just continue, nothing interesting here, no points.
          continue;
        }
        // そのピクセルのラベル
        uint16_t label = labels_ptr->at<uint16_t>(row, col);
        if (label < 1) {
          // this is a default label, skip
          // このラベルはノーカウント 次のラベルへ
          continue;
        }
        // 有効ラベル
        for (const auto& point_idx : point_container.points()) {
          const auto& point = cloud.points()[point_idx];
          clusters[label].push_back(point); // ここへ同類を集結
        }
      }
    }

    // filter out unfitting clusters
    std::vector<uint16_t> labels_to_erase;
    for (const auto& kv : clusters) {
      const auto& cluster = kv.second;
      // clusterサイズェェック
      if (cluster.size() < this->_min_cluster_size ||
          cluster.size() > this->_max_cluster_size) {
        labels_to_erase.push_back(kv.first); // サイズックククアウトはここへpush_back
      }
    }
    // labelは変可可能で、コンテナ内の要素は変更不可能
    for (auto label : labels_to_erase) {
      clusters.erase(label);
    }

    uint64_t count_clusters;
    fprintf(stderr, "INFO: prepared clusters in: %lu us\n", count_clusters = timer.measure());
  //  fprintf(fp, "%lu \n",count_clusters);
   // count_time += timer.measure();

    this->ShareDataWithAllClients(clusters);
    uint64_t count_share;
    fprintf(stderr, "INFO: clusters shared: %lu us\n", count_share = timer.measure());
  //  fprintf(fp, "%lu \n",count_share);
 //   count_time += timer.measure();

    count_time = count_image + count_sent + count_clusters + count_share;
    time_pub = count_time;
   // fprintf(stderr, "INFO: count_time: %lu us\n", count_time);
   std::cout << "without ground remove count_time: " << count_time << std::endl;
 //  ofs << count_time << std::endl;
//   ofs_2 << count_time << std::endl;

//   fprintf(fp, "%lu \n", count_time);
//   ros_publish(argc, argv);
  /* if((fp = fopen("save.csv", "w")) != NULL){
     fprintf(fp, "%lu \n", count_time);
   }
   i++;
   if(i > 50)
        fclose(fp);*/
  }

 private:
  int _counter;
  Radians _angle_tollerance;

  AbstractClient<cv::Mat>* _label_client;

  DiffFactory::DiffType _diff_type = DiffFactory::DiffType::NONE;
};

}  // namespace depth_clustering
#endif  // SRC_CLUSTERERS_IMAGE_BASED_CLUSTERER_H_
