#ifndef BUNDLE_ADJUSTMENT_H_
#define BUNDLE_ADJUSTMENT_H_

#include <fstream>
#include <string>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "opencv2/core/core.hpp"

namespace sfm {

// The problem being solved here is known as a Bundle Adjustment
// problem in computer vision. Given a set of 3d points X_1, ..., X_n,
// a set of cameras P_1, ..., P_m. If the point X_i is visible in
// image j, then there is a 2D observation u_ij that is the expected
// projection of X_i using P_j. The aim of this optimization is to
// find values of X_i and P_j such that the reprojection error
//    E(X,P) =  sum_ij  |u_ij - P_j X_i|^2
// is minimized.

// Problem anatomy:
// Objective: minimize the reprojection error
// Unknown: configuration of 3D object points and 6-dof camera poses.
// Given: a set of 2D image points with their correspondence is known exactly,
// camera calibration matrix K, a optional initial value for 3D object points
// and 6-dof camera poses.
class BAProblem {
  using Vec9d = cv::Vec<double, 9>;

 public:
  explicit BAProblem() {}
  ~BAProblem();

  bool LoadImagePoints(const std::string& file_name);
  bool WriteToFile(const std::string& file_name);

  void InitObjectPointsAndCameras(std::vector<cv::Point3d>* points_,
                                  std::vector<cv::Vec6d>* cameras) {}

  void BuildBAProblem() {}

  void SolveBAProblem() {}

  int num_observations() { return static_cast<int>(observations_.size()); }
  int num_object_points() { return static_cast<int>(points_.size()); }
  int num_cameras() { return static_cast<int>(cameras_.size()); }
  int num_parameters() { return static_cast<int>(parameters_.size()); }

 private:
  const std::vector<std::vector<cv::Point2d>> observations_;
  std::vector<cv::Point3d> points_;
  std::vector<cv::Vec6d> cameras_;
  std::vector<Vec9d> parameters_;
};

struct ReprojectionError9DOF {
  ReprojectionError9DOF(const cv::Point2d& observed) : observed_(observed) {}

  template <typename T>
  bool operator()(const T* const camera, const T* const point, T* residuals);

 private:
  const cv::Point2d& observed_;
};

}  // End namespace sfm

// #include "bundle_adjustment.cpp"

#endif  // BUNDLE_ADJUSTMENT_H_
