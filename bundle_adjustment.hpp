#ifndef BUNDLE_ADJUSTMENT_
#define BUNDLE_ADJUSTMENT_

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "opencv2/core/core.hpp"



class BAProblem {
 public:
 private:
};

// Reprojection error for bundle adjustment.
// This error models the distance between the estimated image points and the
// obsered image points, where the observed image points are given in advance
// while the estimated image points are computed using finite perspective
// projection. The error (residual) hence is the sum of the Euclidean distance
// between the estimated and observed image points.
struct ReprojectionError {
  //@note Discussion: class or struct?
  //@ref https://stackoverflow.com/a/54596/11240780
  //! Conclusion: use struct when there're POD types only. There's one
  //! exception, in spirit of in accordance with STL, use struct when
  //! defining functors.
  //@ref https://stackoverflow.com/a/146454/11240780
  ReprojectionError(const cv::Point2d& observed, const double focal_length,
                    const cv::Point2d& principal_point)
      : observed_(observed),
        focal_length_(focal_length),
        principal_point_(principal_point) {}

  // Cost functor used in ceres::AutoDiffCostFunction
  template <typename T>
  bool operator()(const T* const camera, const T* const point,
                  T* residuals) const {
    // Steps to get an image point projected by an object point
    // 1) Rigid transformation: X' = RX + t, conversion from world to camera
    // coords.
    // 2) Perspective division: X'_norm = X' / X'.z, divide by the Z coord. to
    // get the normalized 3D point.
    // 3) Calibration: x' = K * X'_norm to get the normalized pixel coords.
    //@ref http://grail.cs.washington.edu/projects/bal/

    T X[3];
    // Step 1: rigid transformation
    // camera[0, 1, 2] is the 3-param (axis-angle representation) rotation
    // Use Rodrigues rotation formula to rotate the X point
    ceres::AngleAxisRotatePoint(camera, point, X);
    // camera[3, 4, 5] is the 3-param translation
    X[0] += camera[3];
    X[1] += camera[4];
    X[2] += camera[5];

    // Step 2: Perspective division
    X[0] /= X[2];
    X[1] /= X[2];
    X[2] /= X[2];  // X[2] = T(1)

    // Step 3: Calibration
    // Camera matrix K = [[f, 0, px], [0, f, py], [0, 0, 1]]
    // TODO Model the skew factor and geometric distortion
    //@ref http://ceres-solver.org/nnls_tutorial.html#bundle-adjustment
    T x_p = focal_length_ * X[0] + principal_point_.x;
    T y_p = focal_length_ * X[1] + principal_point_.y;

    // residual = estimated - observed
    residuals[0] = x_p - T(observed_.x);
    residuals[1] = y_p - T(observed_.y);

    return true;
  }

  //@note Discussion: factory or constructor?
  //@ref https://stackoverflow.com/a/10205120/11240780
  static ceres::CostFunction* Create(const cv::Point2d& observed,
                                     const double focal_length,
                                     const cv::Point2d& principal_point) {
    // Cost functor, dim of residuals, dim of camera params, dim of 3D point
    // params
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(
        new ReprojectionError(observed, focal_length, principal_point)));
  }

 private:
  // Measurements parameters
  const cv::Point2d observed_;         // observed 2D image point: (x, y)
  const double focal_length_;          // focal length: f
  const cv::Point2d principal_point_;  // principal point: (px, py)
};

#endif  // BUNDLE_ADJUSTMENT_