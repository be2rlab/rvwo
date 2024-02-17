/**
 * This file is modified version of ORB-SLAM3
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
#ifndef CAMERAMODELS_PINHOLE_H
#define CAMERAMODELS_PINHOLE_H

#include <assert.h>
#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

#include "geometry/camera_models/geometric_camera.h"

#include "map/two_view_reconstruction.h"

namespace RVWO {
// Pinhole class declaration
class Pinhole : public GeometricCamera {

  friend class boost::serialization::access;

  // Serialization function for boost
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &boost::serialization::base_object<GeometricCamera>(*this);
  }

public:
  // Constructors
  Pinhole() {
    mvParameters.resize(4);
    mnId = nNextId++;
    mnType = CAM_PINHOLE;
  }

  Pinhole(const std::vector<float> _vParameters)
      : GeometricCamera(_vParameters), tvr(nullptr) {
    assert(mvParameters.size() == 4);
    mnId = nNextId++;
    mnType = CAM_PINHOLE;
  }

  Pinhole(Pinhole *pPinhole)
      : GeometricCamera(pPinhole->mvParameters), tvr(nullptr) {
    assert(mvParameters.size() == 4);
    mnId = nNextId++;
    mnType = CAM_PINHOLE;
  }

  // Destructor
  ~Pinhole() {
    if (tvr)
      delete tvr;
  }

  // Projection functions
  cv::Point2f project(const cv::Point3f &p3D);
  cv::Point2f project(const cv::Mat &m3D);
  Eigen::Vector2d project(const Eigen::Vector3d &v3D);
  cv::Mat projectMat(const cv::Point3f &p3D);

  // Uncertainty calculation
  float uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D);

  // Unprojection functions
  cv::Point3f unproject(const cv::Point2f &p2D);
  cv::Mat unprojectMat(const cv::Point2f &p2D);

  // Jacobian matrices for projection and unprojection
  cv::Mat projectJac(const cv::Point3f &p3D);
  Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d &v3D);
  cv::Mat unprojectJac(const cv::Point2f &p2D);

  // Two-view reconstruction function
  bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint> &vKeys1,
                               const std::vector<cv::KeyPoint> &vKeys2,
                               const std::vector<int> &vMatches12, cv::Mat &R21,
                               cv::Mat &t21, std::vector<cv::Point3f> &vP3D,
                               std::vector<bool> &vbTriangulated);

  // Intrinsic matrix function
  cv::Mat toK();

  // Epipolar constraint function
  bool epipolarConstrain(GeometricCamera *pCamera2, const cv::KeyPoint &kp1,
                         const cv::KeyPoint &kp2, const cv::Mat &R12,
                         const cv::Mat &t12, const float sigmaLevel,
                         const float unc);

  // Match and triangulate function
  bool matchAndtriangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                           GeometricCamera *pOther, cv::Mat &Tcw1,
                           cv::Mat &Tcw2, const float sigmaLevel1,
                           const float sigmaLevel2, cv::Mat &x3Dtriangulated) {
    return false;
  }

  // Overloaded insertion and extraction operators for serialization
  friend std::ostream &operator<<(std::ostream &os, const Pinhole &ph);
  friend std::istream &operator>>(std::istream &os, Pinhole &ph);

private:
  // Skew symmetric matrix function
  cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

  // Two View Reconstruction member object
  TwoViewReconstruction *tvr;
};
} // namespace RVWO

#endif // CAMERAMODELS_PINHOLE_H
