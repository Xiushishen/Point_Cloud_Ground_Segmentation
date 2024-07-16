#include "ground_segmentation/viewer.h"

#include <chrono>

using namespace std;
using namespace chrono_literals;

Viewer::Viewer() {
  lock_guard<mutex> lock(viewer_mutex_);

  viewer_.setBackgroundColor(0, 0, 0);
  viewer_.addCoordinateSystem(1.0);
  viewer_.initCameraParameters();
  viewer_.setCameraPosition(-2.0, 0, 2.0, 1.0, 0, 0);

  addEmptyPointCloud("min_cloud");
  addEmptyPointCloud("ground_cloud");
  addEmptyPointCloud("obstacle_cloud");

  viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                           0.0f, 1.0f, 0.0f,
                                           "min_cloud");
  viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                           2.0f,
                                           "min_cloud");
  viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                           1.0f, 0.0f, 0.0f,
                                           "ground_cloud");

  view_thread_ = thread(&Viewer::drawThread, this);
}

Viewer::~Viewer() {
  redraw_ = false;
  {
    lock_guard<mutex> lock(viewer_mutex_);
    viewer_.close();
  }
  view_thread_.join();
}

void Viewer::visualize(const list<PointLine>& lines,
                       const PointCloud::ConstPtr& min_cloud,
                       const PointCloud::ConstPtr& ground_cloud,
                       const PointCloud::ConstPtr& obstacle_cloud) {
  redraw_ = false;
  lock_guard<mutex> lock(viewer_mutex_);
  viewer_.removeAllShapes();
  visualizePointCloud(min_cloud, "min_cloud");
  visualizePointCloud(ground_cloud, "ground_cloud");
  visualizePointCloud(obstacle_cloud, "obstacle_cloud");
  visualizeLines(lines);
}

void Viewer::visualizeLines(const list<PointLine>& lines) {
  size_t counter = 0;
  for (auto it = lines.begin(); it != lines.end(); ++it) {
    viewer_.addLine<pcl::PointXYZ>(it->first, it->second, to_string(counter++));
  }    
}

void Viewer::visualizePointCloud(const PointCloud::ConstPtr& cloud,
                                 const string& id) {
  viewer_.updatePointCloud(cloud, id);
}

void Viewer::addEmptyPointCloud(const string& id) {
  viewer_.addPointCloud(PointCloud().makeShared(), id);    
}

void Viewer::drawThread() {
  bool stopped = false;
  while (!stopped) {
    {
      lock_guard<mutex> lock(viewer_mutex_);
      redraw_ = true;
      while (redraw_) {
        viewer_.spinOnce(1);    
      }
    }
    std::this_thread::sleep_for(1ms);
    lock_guard<mutex> lock(viewer_mutex_);
    stopped = viewer_.wasStopped();
  }
}



