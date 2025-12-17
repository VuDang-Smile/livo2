/*
 * equirectangular_camera.cpp
 *
 *  Created on: January 2025
 *      Author: Auto-generated for equirectangular camera model
 *
 *  Implementation of equirectangular (equidistant cylindrical) projection.
 *
 *  Forward projection (world2cam):
 *    Given a 3D point (x, y, z) in camera frame:
 *    1. Normalize to get bearing vector
 *    2. Convert to spherical coordinates (latitude, longitude)
 *       lat = -asin(y)  [elevation angle]
 *       lon = atan2(x, z)  [azimuth angle]
 *    3. Convert to pixel coordinates:
 *       u = width * (0.5 + lon / (2*PI))
 *       v = height * (0.5 - lat / PI)
 *
 *  Inverse projection (cam2world):
 *    Given pixel coordinates (u, v):
 *    1. Convert to spherical coordinates:
 *       lon = (u / width - 0.5) * 2*PI
 *       lat = (0.5 - v / height) * PI
 *    2. Convert to 3D bearing vector:
 *       x = cos(lat) * sin(lon)
 *       y = -sin(lat)
 *       z = cos(lat) * cos(lon)
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <vikit/equirectangular_camera.h>
#include <vikit/math_utils.h>

namespace vk {

EquirectangularCamera::
EquirectangularCamera(double width, double height, double scale) :
  AbstractCamera(width * scale, height * scale, scale),
  // For equirectangular projection:
  // fx = width / (2*PI) - represents pixels per radian in horizontal direction
  // fy = height / PI - represents pixels per radian in vertical direction
  // cx = width / 2 - center of image horizontally
  // cy = height / 2 - center of image vertically
  fx_(width * scale / (2.0 * M_PI)),
  fy_(height * scale / M_PI),
  cx_(width * scale / 2.0),
  cy_(height * scale / 2.0)
{
  cout << "EquirectangularCamera: scale=" << scale 
       << ", width=" << width_ << ", height=" << height_
       << ", fx=" << fx_ << ", fy=" << fy_
       << ", cx=" << cx_ << ", cy=" << cy_ << endl;
}

EquirectangularCamera::
~EquirectangularCamera()
{}

Vector3d EquirectangularCamera::
cam2world(const double& u, const double& v) const
{
  // Convert pixel coordinates to spherical coordinates
  // Longitude (azimuth): ranges from -PI to PI
  double lon = (u / width_ - 0.5) * 2.0 * M_PI;
  
  // Latitude (elevation): ranges from -PI/2 to PI/2
  double lat = (0.5 - v / height_) * M_PI;
  
  // Convert spherical coordinates to 3D bearing vector
  // Standard spherical to Cartesian conversion:
  // x = cos(lat) * sin(lon)  [forward/right]
  // y = -sin(lat)            [up/down, negative because image y increases downward]
  // z = cos(lat) * cos(lon)  [forward]
  Vector3d xyz;
  double cos_lat = cos(lat);
  xyz[0] = cos_lat * sin(lon);
  xyz[1] = -sin(lat);
  xyz[2] = cos_lat * cos(lon);
  
  return xyz.normalized();
}

Vector3d EquirectangularCamera::
cam2world(const Vector2d& px) const
{
  return cam2world(px[0], px[1]);
}

Vector2d EquirectangularCamera::
world2cam(const Vector3d& xyz) const
{
  Vector2d px;
  
  // Handle degenerate case (point at origin)
  if (xyz.squaredNorm() < 1e-6)
  {
    // Return center of image
    px[0] = cx_;
    px[1] = cy_;
    return px;
  }
  
  // Normalize to get bearing vector
  Vector3d bearing = xyz.normalized();
  
  // Convert to spherical coordinates
  // Latitude (elevation): -asin(y) because y increases upward in camera frame
  // but decreases in image coordinates
  double lat = -asin(bearing[1]);
  
  // Longitude (azimuth): atan2(x, z) gives angle in x-z plane
  double lon = atan2(bearing[0], bearing[2]);
  
  // Convert to pixel coordinates
  // u = width * (0.5 + lon / (2*PI))
  px[0] = width_ * (0.5 + lon / (2.0 * M_PI));
  
  // v = height * (0.5 - lat / PI)
  px[1] = height_ * (0.5 - lat / M_PI);
  
  return px;
}

Vector2d EquirectangularCamera::
world2cam(const Vector2d& uv) const
{
  // For equirectangular, we need the full 3D point, not just unit plane coordinates
  // This function is called with unit plane coordinates (x/z, y/z)
  // We need to reconstruct the 3D direction vector
  // Unit plane coordinates represent (x/z, y/z), so we can reconstruct as (x/z, y/z, 1)
  Vector3d xyz_3d = unproject2d(uv);
  return world2cam(xyz_3d);
}

} // end namespace vk



