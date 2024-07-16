#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef std::pair<pcl::PointXYZ, pcl::PointXYZ> PointLine;

class Viewer {
 public:
  Viewer();
  ~Viewer();

  void visualize(const list<PointLine>& lines,
                 const PointCloud::ConstPtr& min_cloud,
                 const PointCloud::ConstPtr& ground_cloud,
                 const PointCloud::ConstPtr& obstacle_cloud);
 protected:
  // typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  // typedef std::pair<pcl::PointXYZ, pcl::PointXYZ> PointLine;

  void visualizeLines(const list<PointLine>& lines);
  void visualizePointCloud(const PointCloud::ConstPtr& cloud,
                           const string& id);
  void addEmptyPointCloud(const string& id);
  void drawThread();

  pcl::visualization::PCLVisualizer viewer_;
  thread view_thread_;
  mutex viewer_mutex_;
  atomic<bool> redraw_{true};

};