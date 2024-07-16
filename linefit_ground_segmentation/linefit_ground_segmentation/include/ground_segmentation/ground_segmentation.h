#ifndef GROUND_SEGMENTATION_H_
#define GROUND_SEGMENTATION_H_

#include <mutex>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "ground_segmentation/segment.h"
#include "ground_segmentation/viewer.h"

using namespace std;

class GroundSegmentationParams {
 public:
  GroundSegmentationParams()
      : visualize(false),
        r_min_square(0.3 * 0.3),
        r_max_square(20 * 20),
        n_bins(30),
        n_segments(180),
        max_dist_to_line(0.15),
        min_slope(0),
        max_slope(1),
        n_threads(4),
        max_error_square(0.01),
        long_threshold(2.0),
        max_long_height(0.1),
        max_start_height(0.2),
        sensor_height(0.2),
        line_search_angle(0.2) {}

  // visualize estimated ground
  bool visualize;
  // minimum range of segmentation
  double r_min_square;
  // maximum range of segmentation
  double r_max_square;
  // number of radial bins
  int n_bins;
  // number of angular segments
  int n_segments;
  // maximum distance to be ground line to be classified as ground
  double max_dist_to_line;
  // min slope to be considered ground line
  double min_slope;
  // max slope to be considered ground line
  double max_slope;
  // max error for line fit
  double max_error_square;
  // distance at which points are considered far from each other
  double long_threshold;
  // maximum slope for
  double max_long_height;
  // maximum heigh of starting line to be labelled ground
  double max_start_height;
  // height of sensor above ground
  double sensor_height;
  // how far to search for a line in angular direction [rad].
  double line_search_angle;
  // number of threads
  int n_threads;
};

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pair<pcl::PointXYZ, pcl::PointXYZ> PointLine;

class GroundSegmentation {
 public:
  explicit GroundSegmentation(const GroundSegmentationParams& params = GroundSegmentationParams());

  void segment(const PointCloud& cloud, vector<int>* segmentation);
 
 private:
  void assignCluster(vector<int>* segmentation);
  void assignClusterThread(const uint& start_index,
                           const uint& end_index,
                           vector<int>* segmentation);
  void insertPoints(const PointCloud& cloud);
  void insertionThread(const PointCloud& cloud,
                       const size_t start_index,
                       const size_t end_index);
  void getMinZPoints(PointCloud* out_cloud);
  void getLines(list<PointLine>* lines);
  void lineFitThread(const uint start_index, const uint end_index,
                     list<PointLine>* lines, mutex* lines_mutex);
  pcl::PointXYZ minZPointTo3d(const Bin::MinZPoint& min_z_point, const double& angle);
  void getMinZPointCloud(PointCloud* cloud);
  void resetSegments();
  void visualizePointCloud(const PointCloud::ConstPtr& cloud,
                           const string& id = "point_cloud");
  void visualizeLines(const list<PointLine>& lines);
  void visualize(const list<PointLine>& lines,
                 const PointCloud::ConstPtr& cloud,
                 const PointCloud::ConstPtr& ground_cloud,
                 const PointCloud::ConstPtr& obstacle_cloud);

  const GroundSegmentationParams params_;
  vector<Segment> segments_;
  vector<pair<int, int>> bin_index_;
  vector<Bin::MinZPoint> segment_coordinates_;
  unique_ptr<Viewer> viewer_;

};

#endif // GROUND_SEGMENTATION_H_