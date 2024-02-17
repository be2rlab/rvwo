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
#ifndef CAMERAMODELS_KANNALABRANDT8_H
#define CAMERAMODELS_KANNALABRANDT8_H

#include <assert.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

#include "geometry/camera_models/geometric_camera.h"
#include "map/two_view_reconstruction.h"

namespace RVWO {
// KannalaBrandt8 class declaration
class KannalaBrandt8 final : public GeometricCamera {

  friend class boost::serialization::access;

  // Serialization function for boost
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &boost::serialization::base_object<GeometricCamera>(*this);
    ar &const_cast<float &>(precision);
  }

public:
  // Constructors
  KannalaBrandt8() : precision(1e-6) {
    mvParameters.resize(8);
    mnId = nNextId++;
    mnType = CAM_FISHEYE;
  }
  KannalaBrandt8(const std::vector<float> _vParameters)
      : GeometricCamera(_vParameters), precision(1e-6), mvLappingArea(2, 0),
        tvr(nullptr) {
    assert(mvParameters.size() == 8);
    mnId = nNextId++;
    mnType = CAM_FISHEYE;
  }
  KannalaBrandt8(const std::vector<float> _vParameters, const float _precision)
      : GeometricCamera(_vParameters), precision(_precision),
        mvLappingArea(2, 0) {
    assert(mvParameters.size() == 8);
    mnId = nNextId++;
    mnType = CAM_FISHEYE;
  }
  KannalaBrandt8(KannalaBrandt8 *pKannala)
      : GeometricCamera(pKannala->mvParameters), precision(pKannala->precision),
        mvLappingArea(2, 0), tvr(nullptr) {
    assert(mvParameters.size() == 8);
    mnId = nNextId++;
    mnType = CAM_FISHEYE;
  }

  // GeometricCamera functions implementation

  // Project a 3D point onto the image plane
  cv::Point2f project(const cv::Point3f &p3D);
  cv::Point2f project(const cv::Mat &m3D);
  Eigen::Vector2d project(const Eigen::Vector3d &v3D);
  cv::Mat projectMat(const cv::Point3f &p3D);

  // Calculate uncertainty for a 2D point
  float uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D);

  // Unproject a 2D point to 3D
  cv::Point3f unproject(const cv::Point2f &p2D);
  cv::Mat unprojectMat(const cv::Point2f &p2D);

  // Jacobian of projection for a 3D point
  cv::Mat projectJac(const cv::Point3f &p3D);
  Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d &v3D);

  // Jacobian of unprojection for a 2D point
  cv::Mat unprojectJac(const cv::Point2f &p2D);

  // Reconstruct 3D points from two views
  bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint> &vKeys1,
                               const std::vector<cv::KeyPoint> &vKeys2,
                               const std::vector<int> &vMatches12, cv::Mat &R21,
                               cv::Mat &t21, std::vector<cv::Point3f> &vP3D,
                               std::vector<bool> &vbTriangulated);

  // Get camera intrinsic matrix
  cv::Mat toK();

  // Apply epipolar constraint
  bool epipolarConstrain(GeometricCamera *pCamera2, const cv::KeyPoint &kp1,
                         const cv::KeyPoint &kp2, const cv::Mat &R12,
                         const cv::Mat &t12, const float sigmaLevel,
                         const float unc);

  // Triangulate 3D points from two views
  float TriangulateMatches(GeometricCamera *pCamera2, const cv::KeyPoint &kp1,
                           const cv::KeyPoint &kp2, const cv::Mat &R12,
                           const cv::Mat &t12, const float sigmaLevel,
                           const float unc, cv::Mat &p3D);

  // Lapping area for optimization
  std::vector<int> mvLappingArea;

  // Match and triangulate points between two views
  bool matchAndtriangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                           GeometricCamera *pOther, cv::Mat &Tcw1,
                           cv::Mat &Tcw2, const float sigmaLevel1,
                           const float sigmaLevel2, cv::Mat &x3Dtriangulated);

  // Overloaded insertion and extraction operators for serialization
  friend std::ostream &operator<<(std::ostream &os, const KannalaBrandt8 &kb);
  friend std::istream &operator>>(std::istream &is, KannalaBrandt8 &kb);

private:
  const float precision;

  // Two View Reconstruction member object
  TwoViewReconstruction *tvr;

  // Triangulate 3D point from two views
  void Triangulate(const cv::Point2f &p1, const cv::Point2f &p2,
                   const cv::Mat &Tcw1, const cv::Mat &Tcw2, cv::Mat &x3D);
};
} // namespace RVWO

#endif // CAMERAMODELS_KANNALABRANDT8_H
