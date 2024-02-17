/**
 * This file is edited part from ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef CAMERAMODELS_GEOMETRICCAMERA_H
#define CAMERAMODELS_GEOMETRICCAMERA_H

#include <Eigen/Geometry>
#include <boost/serialization/access.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace RVWO {
// GeometricCamera class declaration
class GeometricCamera {
  friend class boost::serialization::access;

  // Serialization function for boost
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar & mnId;
    ar & mnType;
    ar & mvParameters;
  }

public:
  // Constructors
  GeometricCamera() {}
  GeometricCamera(const std::vector<float> &_vParameters)
      : mvParameters(_vParameters) {}
  // Destructor
  ~GeometricCamera() {}

  // functions for geometric camera operations

  // Project a 3D point onto the image plane
  virtual cv::Point2f project(const cv::Point3f &p3D) = 0;
  // Project a 3D point represented as a cv::mat onto the image plane
  virtual cv::Point2f project(const cv::Mat &m3D) = 0;
  // Project a 3D point onto the image plane and return as Eigen vector
  virtual Eigen::Vector2d project(const Eigen::Vector3d &v3D) = 0;
  // Project a 3D point onto the image plane and return as cv::mat
  virtual cv::Mat projectMat(const cv::Point3f &p3D) = 0;
  // Calculate uncertainty for a 2D point
  virtual float uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D) = 0;
  // Unproject a 2D point to 3D
  virtual cv::Point3f unproject(const cv::Point2f &p2D) = 0;
  // Unproject a 2D point to 3D and return as matrix
  virtual cv::Mat unprojectMat(const cv::Point2f &p2D) = 0;
  // Jacobian of projection for a 3D point
  virtual cv::Mat projectJac(const cv::Point3f &p3D) = 0;
  // Jacobian of projection for a 3D point using Eigen vector
  virtual Eigen::Matrix<double, 2, 3>
  projectJac(const Eigen::Vector3d &v3D) = 0;
  // Jacobian of unprojection for a 2D point
  virtual cv::Mat unprojectJac(const cv::Point2f &p2D) = 0;
  // Reconstruct 3D points from two views
  virtual bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint> &vKeys1,
                                       const std::vector<cv::KeyPoint> &vKeys2,
                                       const std::vector<int> &vMatches12,
                                       cv::Mat &R21, cv::Mat &t21,
                                       std::vector<cv::Point3f> &vP3D,
                                       std::vector<bool> &vbTriangulated) = 0;
  // Get camera intrinsic matrix
  virtual cv::Mat toK() = 0;
  // Apply epipolar constraint
  virtual bool epipolarConstrain(GeometricCamera *otherCamera,
                                 const cv::KeyPoint &kp1,
                                 const cv::KeyPoint &kp2, const cv::Mat &R12,
                                 const cv::Mat &t12, const float sigmaLevel,
                                 const float unc) = 0;

  // Get Parameter from the camera
  float getParameter(const int i) { return mvParameters[i]; }
  // Set paramter from the camera
  void setParameter(const float p, const size_t i) { mvParameters[i] = p; }

  // Get camera parameters size
  size_t size() { return mvParameters.size(); }

  // Match and triangulate points between two views
  virtual bool matchAndtriangulate(const cv::KeyPoint &kp1,
                                   const cv::KeyPoint &kp2,
                                   GeometricCamera *pOther, cv::Mat &Tcw1,
                                   cv::Mat &Tcw2, const float sigmaLevel1,
                                   const float sigmaLevel2,
                                   cv::Mat &x3Dtriangulated) = 0;

  // Getter for camera ID
  unsigned int GetId() { return mnId; }
  // Getter for camera type
  unsigned int GetType() { return mnType; }

  // Camera types
  const unsigned int CAM_PINHOLE = 0;
  const unsigned int CAM_FISHEYE = 1;

  // Static variable for the next camera ID
  static long unsigned int nNextId;

protected:
  // Camera parameters
  std::vector<float> mvParameters;
  // Camera ID
  unsigned int mnId;
  // Camera type
  unsigned int mnType;
};
} // namespace RVWO

#endif // CAMERAMODELS_GEOMETRICCAMERA_H
