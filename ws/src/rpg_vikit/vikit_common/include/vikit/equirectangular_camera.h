/*
 * equirectangular_camera.h
 *
 *  Created on: January 2025
 *      Author: Auto-generated for equirectangular camera model
 *
 *  This class implements the equirectangular (equidistant cylindrical) projection model.
 *  Suitable for 360-degree cameras like Theta X and panoramic LiDAR cameras like Livox MID 360.
 *
 *  Equirectangular projection maps:
 *  - Longitude (azimuth) -> x coordinate (0 to width)
 *  - Latitude (elevation) -> y coordinate (0 to height)
 *
 *  For Theta X: Full 360x180 degree FOV
 *  For Livox MID 360: 360 degree horizontal, 59 degree vertical (-7 to 52 degrees)
 */

#ifndef EQUIRECTANGULAR_CAMERA_H_
#define EQUIRECTANGULAR_CAMERA_H_

#include <stdlib.h>
#include <string>
#include <Eigen/Eigen>
#include <vikit/abstract_camera.h>
#include <opencv2/opencv.hpp>

namespace vk {

using namespace std;
using namespace Eigen;

class EquirectangularCamera : public AbstractCamera {

private:
  // For equirectangular, these are derived from width/height
  // fx = width / (2*PI), fy = height / PI
  // cx = width / 2, cy = height / 2
  const double fx_, fy_;
  const double cx_, cy_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructor: width and height are the image dimensions
  // For equirectangular, intrinsics are derived from image size
  EquirectangularCamera(double width, double height, double scale = 1.0);

  ~EquirectangularCamera();

  // Project from pixel coordinates to world coordinates (bearing vector)
  virtual Vector3d
  cam2world(const double& x, const double& y) const;

  virtual Vector3d
  cam2world(const Vector2d& px) const;

  // Project from world coordinates (3D point) to pixel coordinates
  virtual Vector2d
  world2cam(const Vector3d& xyz_c) const;

  // Project from unit plane coordinates to pixel coordinates
  virtual Vector2d
  world2cam(const Vector2d& uv) const;

  const Vector2d focal_length() const
  {
    return Vector2d(fx_, fy_);
  }

  virtual double errorMultiplier2() const
  {
    // Use average of fx and fy for equirectangular
    return (fabs(fx_) + fabs(fy_)) / 2.0;
  }

  virtual double errorMultiplier() const
  {
    return fabs(4.0 * fx_ * fy_);
  }

  virtual double fx() const { return fx_; };
  virtual double fy() const { return fy_; };
  virtual double cx() const { return cx_; };
  virtual double cy() const { return cy_; };
};

} // end namespace vk

#endif /* EQUIRECTANGULAR_CAMERA_H_ */



