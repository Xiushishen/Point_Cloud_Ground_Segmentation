#include "ground_segmentation/segment.h"

using namespace std;

Segment::Segment(const uint& n_bins,
                 const double& min_slope,
                 const double& max_slope,
                 const double& max_error,
                 const double& long_threshold,
                 const double& max_long_height,
                 const double& max_start_height,
                 const double& sensor_height) :
                 bins_(n_bins),
                 min_slope_(min_slope),
                 max_slope_(max_slope),
                 max_error_(max_error),
                 long_threshold_(long_threshold),
                 max_long_height_(max_long_height),
                 max_start_height_(max_start_height),
                 sensor_height_(sensor_height) {}

void Segment::fitSegmentLines() {
  auto line_start = bins_.begin();
  while (!line_start->hasPoint()) {
    ++line_start;
    if (line_start == bins_.end()) return;
  }
  bool is_long_line = false;
  double cur_ground_height = -sensor_height_;
  list<Bin::MinZPoint> current_line_points(1, line_start->getMinZPoint());
  LocalLine cur_line = std::make_pair(0, 0);
  for (auto line_iter = line_start + 1; line_iter != bins_.end(); ++line_iter) {
    if (line_iter->hasPoint()) {
      Bin::MinZPoint cur_point = line_iter->getMinZPoint();
      if (cur_point.d - current_line_points.back().d > long_threshold_) { is_long_line = true; }
      if (current_line_points.size() >= 2) {
        double expected_z = std::numeric_limits<double>::max();
        if (is_long_line && current_line_points.size() > 2) {
          expected_z = cur_line.first * cur_point.d + cur_line.second;
        }
        current_line_points.push_back(cur_point);
        cur_line = fitLocalLine(current_line_points);
        const double error = getMaxError(current_line_points, cur_line);
        if (error > max_error_ || fabs(cur_line.first) > max_slope_ || is_long_line
                               && fabs(expected_z - cur_point.z) > max_long_height_) {
          current_line_points.pop_back();
          if (current_line_points.size() >= 3) {
            const LocalLine new_line = fitLocalLine(current_line_points);
            lines_.push_back(localLineToLine(new_line, current_line_points));
            cur_ground_height = new_line.first * current_line_points.back().d + new_line.second;
          }
          is_long_line = false;
          current_line_points.erase(current_line_points.begin(), --current_line_points.end());
          --line_iter;
        } else {}
      } else { 
          if (cur_point.d - current_line_points.back().d < long_threshold_ && // not long line
              fabs(current_line_points.back().z - cur_ground_height) < max_start_height_) { // point is close to ground
            current_line_points.push_back(cur_point);
          } else { // two points are not on the same plane
              current_line_points.clear();
              current_line_points.push_back(cur_point);
            }
        }
    }
  }
  if (current_line_points.size() > 2) {
    const LocalLine new_line = fitLocalLine(current_line_points);
    lines_.push_back(localLineToLine(new_line, current_line_points));
  }
}

Segment::Line Segment::localLineToLine(const LocalLine& local_line,
                                       const list<Bin::MinZPoint>& line_points) {
  Line line;
  const double first_d = line_points.front().d;
  const double second_d = line_points.back().d;
  const double first_z = local_line.first * first_d + local_line.second;
  const double second_z = local_line.first * second_d + local_line.second;
  line.first.z = first_z;
  line.first.d = first_d;
  line.second.z = second_z;
  line.second.d = second_d;
  return line;
}

/* vertial distance from point to line */
double Segment::verticalDistanceToLine(const double& d, const double& z) {
  static const double kMargin = 0.1;
  double distance = -1;
  for (auto it = lines_.begin(); it != lines_.end(); ++it) {
    if (it->first.d - kMargin < d && it->second.d + kMargin > d) {
      const double delta_z = it->second.z - it->first.z;
      const double delta_d = it->second.d - it->first.d;
      const double expected_z = (d - it->first.d) / delta_d * delta_z + it->first.z;
      distance = fabs(z - expected_z);
    }
  }
  return distance;
}


double Segment::getMeanError(const list<Bin::MinZPoint>& points, const LocalLine& line) {
  double error_sum = 0;
  for (auto it = points.begin(); it != points.end(); ++it) {
    const double residual = (line.first * it->d + line.second) - it->z;
    error_sum += residual * residual;
  }
  return error_sum / points.size();
}

double Segment::getMaxError(const list<Bin::MinZPoint>& points, const LocalLine& line) {
  double max_error = 0;
  for (auto it = points.begin(); it != points.end(); ++it) {
    const double residual = (line.first * it->d + line.second) - it->z;
    const double error = residual * residual;
    if (error > max_error) {
        max_error = error;
    }
  }
  return max_error;
}

Segment::LocalLine Segment::fitLocalLine(const list<Bin::MinZPoint>& points) {
  const uint n_points = points.size();
  Eigen::MatrixXd X(n_points, 2);
  Eigen::VectorXd Y(n_points);
  uint counter = 0;
  for (auto iter = points.begin(); iter != points.end(); ++iter) {
    X(counter, 0) = iter->d;
    X(counter, 1) = 1;
    Y(counter) = iter->z;
    ++counter;  
  }
  Eigen::VectorXd result = X.colPivHouseholderQr().solve(Y);
  LocalLine line_result;
  line_result.first = result(0);
  line_result.second = result(1);
  return line_result;
}

bool Segment::getLines(list<Line>* lines) {
  if (lines_.empty()) {
    return false;
  } else {
    *lines = lines_;
    return true;
  }
}