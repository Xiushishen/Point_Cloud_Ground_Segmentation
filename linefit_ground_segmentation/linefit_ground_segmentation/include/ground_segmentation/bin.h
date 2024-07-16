#ifndef GROUND_SEGMENTATION_BIN_H_
#define GROUND_SEGMENTATION_BIN_H_

#include <atomic>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Bin {
 public:
  struct MinZPoint {
   MinZPoint() : z(0), d(0) {}
   MinZPoint(const double& d, const double& z) : z(z), d(d) {}
   bool operator==(const MinZPoint& comp) {return z == comp.z && comp.d;}

   double z;
   double d; 
   };

  Bin();
  Bin(const Bin& segment);

  void addPoint(const pcl::PointXYZ& point);
  void addPoint(const double& d, const double& z);

  MinZPoint getMinZPoint();

  inline bool hasPoint() { return has_point_; }

 private:
  std::atomic<bool> has_point_;
  std::atomic<double> min_z;
  std::atomic<double> min_z_range;
};

#endif // GROUND_SEGMENTATION_BIN_H_