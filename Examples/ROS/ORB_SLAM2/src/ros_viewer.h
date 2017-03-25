#ifndef ROS_VIEWER_H
#define ROS_VIEWER_H
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nodelet/nodelet.h>
#include <iostream>
#include <mutex>

namespace My_Viewer {

struct rawData{
  cv::Mat im;
  cv::Mat depth;
  cv::Mat mTcw;
};

class ros_viewer
{
public:
  ros_viewer(const std::string &strSettingPath);
  void addKfToQueue(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const cv::Mat mTcw);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr createPointCloud(const rawData rawimg, int step=1);

  // Main function
  void Run();

  std::mutex mMutexROSViewer;
private:
  //Calibration matrix
  cv::Mat mK;
  cv::Mat mDistCoef;
  float fx_;
  float fy_;
  float cx_;
  float cy_;
  float invfx;
  float invfy;
  cv::Mat mMapx;
  cv::Mat mMapy;
  cv::Mat mK_new;

  std::vector<rawData> rawImages;

  ros::Publisher pub_pointCloud;

};

} // namespace
#endif // ROS_VIEWER_H
