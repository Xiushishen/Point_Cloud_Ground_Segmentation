#ifndef GROUND_SEGMENTATION_SEGMENT_H_
#define GROUND_SEGMENTATION_SEGMENT_H_

#include <list>
#include <map>
#include <vector>

#include "ground_segmentation/bin.h"

using namespace std;

class Segment {
 public:
  typedef pair<Bin::MinZPoint, Bin::MinZPoint> Line;
  typedef pair<double, double> LocalLine;
  
  explicit Segment(const uint& n_bins,
          const double& min_slope,
          const double& max_slope,
          const double& max_error,
          const double& long_threshold,
          const double& max_long_height,
          const double& max_start_height,
          const double& sensor_height);

  double verticalDistanceToLine(const double& d, const double& z);
  void fitSegmentLines();
  bool getLines(list<Line>* lines);

  inline Bin& operator[](const size_t& index) {
    return bins_[index];
  }
  inline vector<Bin>::iterator begin() {
    return bins_.begin();
  }
  inline vector<Bin>::iterator end() {
    return bins_.end();
  }

 private:
  LocalLine fitLocalLine(const list<Bin::MinZPoint>& points);
  Line localLineToLine(const LocalLine& local_line, const list<Bin::MinZPoint>& line_points);
  double getMeanError(const list<Bin::MinZPoint>& points, const LocalLine& line);
  double getMaxError(const list<Bin::MinZPoint>& points, const LocalLine& line);

  const double min_slope_;
  const double max_slope_;
  const double max_error_;
  const double long_threshold_;
  const double max_long_height_;
  const double max_start_height_;
  const double sensor_height_;

  vector<Bin> bins_;
  list<Line> lines_;

};

#endif // GROUND_SEGMENTATION_SEGMENT_H_