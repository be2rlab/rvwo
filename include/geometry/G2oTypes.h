/**
 * This file is modified part of ORB-SLAM3
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

#ifndef GEOMETRY_G2OTYPES_H
#define GEOMETRY_G2OTYPES_H

#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/types/types_sba.h"

#include <opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <Blocks/Frame.h>
#include <Blocks/KeyFrame.h>

#include "Utils/Converter.h"
#include <math.h>

namespace RVWO {

class KeyFrame;
class Frame;
class GeometricCamera;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 15, 1> Vector15d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 15, 15> Matrix15d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;

/** @brief ExpSO3 computes the exponential
 * @param x,y,z the rotation vector
 * @return the rotation matrix
 */
Eigen::Matrix3d ExpSO3(const double x, const double y, const double z);

/** @brief ExpSO3 computes the exponential
 * @param w the rotation vector
 * @return the rotation matrix
 */
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d &w);

/** @brief LogSO3 computes the logarithm
 * @param R the rotation matrix
 * @return the rotation vector
 */
Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R);

/** @brief Inverse Right Jacobian of SO3
 * @param v the rotation vector
 * @return the inverse right jacobian matrix
 */
Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d &v);

/** @brief Right Jacobian of SO3
 * @param v the rotation vector
 * @return the right jacobian matrix
 */
Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d &v);

/** @brief Right Jacobian of SO3
 * @param x,y,z the rotation vector
 * @return the right jacobian matrix
 */
Eigen::Matrix3d RightJacobianSO3(const double x, const double y,
                                 const double z);

/** @brief Skew symmetric matrix
 * @param w the vector
 * @return the skew symmetric matrix
 */
Eigen::Matrix3d Skew(const Eigen::Vector3d &w);

/** @brief Skew symmetric matrix
 * @param x,y,z the vector
 * @return the skew symmetric matrix
 */
Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y,
                                        const double z);

/** @brief Normalize the rotation
 * @param R the rotation matrix
 * @return the normalized rotation matrix
 */
Eigen::Matrix3d NormalizeRotation(const Eigen::Matrix3d &R);

/** @brief Inverse Depth Point Class
 * @param rho the inverse depth
 * @param u,v the pixel coordinates
 */
class InvDepthPoint {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  InvDepthPoint() {}
  InvDepthPoint(double _rho, double _u, double _v, KeyFrame *pHostKF);

  void Update(const double *pu);

  double rho;
  double u, v; // they are not variables, observation in the host frame

  double fx, fy, cx, cy, bf; // from host frame

  int its;
};

// scale vertex
class VertexScale : public g2o::BaseVertex<1, double> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexScale() { setEstimate(1.0); }
  VertexScale(double ps) { setEstimate(ps); }

  virtual bool read(std::istream &is) { return false; }
  virtual bool write(std::ostream &os) const { return false; }

  virtual void setToOriginImpl() { setEstimate(1.0); }

  virtual void oplusImpl(const double *update_) {
    setEstimate(estimate() * exp(*update_));
  }
};

/** Inverse depth point (just one parameter, inverse depth at the host frame) */
class VertexInvDepth : public g2o::BaseVertex<1, InvDepthPoint> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexInvDepth() {}
  VertexInvDepth(double invDepth, double u, double v, KeyFrame *pHostKF) {
    setEstimate(InvDepthPoint(invDepth, u, v, pHostKF));
  }

  virtual bool read(std::istream &is) { return false; }
  virtual bool write(std::ostream &os) const { return false; }

  virtual void setToOriginImpl() {}

  virtual void oplusImpl(const double *update_) {
    _estimate.Update(update_);
    updateCache();
  }
};

/** @brief EdgeMono: a binary edge that connects a 3D point and a camera pose
 * It is used to optimize the 3D point and the camera pose
 * The error is the difference between the projected 3D point and the observed
 * 2D point The linearization is done using the Jacobian of the projection
 */
class EdgeMono
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ,
                                 VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeMono(int cam_idx_ = 0) : cam_idx(cam_idx_) {}

  virtual bool read(std::istream &is) { return false; }
  virtual bool write(std::ostream &os) const { return false; }

  void computeError() {
    const g2o::VertexSBAPointXYZ *VPoint =
        static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
    const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[1]);
    const Eigen::Vector2d obs(_measurement);
    _error = obs - VPose->estimate().Project(VPoint->estimate(), cam_idx);
  }

  virtual void linearizeOplus();

  bool isDepthPositive() {
    const g2o::VertexSBAPointXYZ *VPoint =
        static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
    const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[1]);
    return VPose->estimate().isDepthPositive(VPoint->estimate(), cam_idx);
  }

  Eigen::Matrix<double, 2, 9> GetJacobian() {
    linearizeOplus();
    Eigen::Matrix<double, 2, 9> J;
    J.block<2, 3>(0, 0) = _jacobianOplusXi;
    J.block<2, 6>(0, 3) = _jacobianOplusXj;
    return J;
  }

  Eigen::Matrix<double, 9, 9> GetHessian() {
    linearizeOplus();
    Eigen::Matrix<double, 2, 9> J;
    J.block<2, 3>(0, 0) = _jacobianOplusXi;
    J.block<2, 6>(0, 3) = _jacobianOplusXj;
    return J.transpose() * information() * J;
  }

public:
  const int cam_idx;
};

/** @brief Edge Mono Only Pose: a unary edge that connects a camera pose
 * It is used to optimize the camera pose only
 * The error is the difference between the projected 3D point and the observed
 * 2D point The linearization is done using the Jacobian of the projection
 */
class EdgeMonoOnlyPose
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeMonoOnlyPose(const cv::Mat &Xw_, int cam_idx_ = 0)
      : Xw(Converter::toVector3d(Xw_)), cam_idx(cam_idx_) {}

  virtual bool read(std::istream &is) { return false; }
  virtual bool write(std::ostream &os) const { return false; }

  void computeError() {
    const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[0]);
    const Eigen::Vector2d obs(_measurement);
    _error = obs - VPose->estimate().Project(Xw, cam_idx);
  }

  virtual void linearizeOplus();

  bool isDepthPositive() {
    const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[0]);
    return VPose->estimate().isDepthPositive(Xw, cam_idx);
  }

  Eigen::Matrix<double, 6, 6> GetHessian() {
    linearizeOplus();
    return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
  }

public:
  const Eigen::Vector3d Xw;
  const int cam_idx;
};

/** @brief EdgeStereo: a binary edge that connects a 3D point and a camera pose
 * It is used to optimize the 3D point and the camera pose
 * The error is the difference between the projected 3D point and the observed
 * 2D point The linearization is done using the Jacobian of the projection
 */
class EdgeStereo
    : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ,
                                 VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeStereo(int cam_idx_ = 0) : cam_idx(cam_idx_) {}

  virtual bool read(std::istream &is) { return false; }
  virtual bool write(std::ostream &os) const { return false; }

  void computeError() {
    const g2o::VertexSBAPointXYZ *VPoint =
        static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
    const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[1]);
    const Eigen::Vector3d obs(_measurement);
    _error = obs - VPose->estimate().ProjectStereo(VPoint->estimate(), cam_idx);
  }

  virtual void linearizeOplus();

  Eigen::Matrix<double, 3, 9> GetJacobian() {
    linearizeOplus();
    Eigen::Matrix<double, 3, 9> J;
    J.block<3, 3>(0, 0) = _jacobianOplusXi;
    J.block<3, 6>(0, 3) = _jacobianOplusXj;
    return J;
  }

  Eigen::Matrix<double, 9, 9> GetHessian() {
    linearizeOplus();
    Eigen::Matrix<double, 3, 9> J;
    J.block<3, 3>(0, 0) = _jacobianOplusXi;
    J.block<3, 6>(0, 3) = _jacobianOplusXj;
    return J.transpose() * information() * J;
  }

public:
  const int cam_idx;
};

/** @brief EdgeStereoOnlyPose: a unary edge that connects a camera pose
 * It is used to optimize the camera pose only
 * The error is the difference between the projected 3D point and the observed
 * 2D point The linearization is done using the Jacobian of the projection
 */
class EdgeStereoOnlyPose
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeStereoOnlyPose(const cv::Mat &Xw_, int cam_idx_ = 0)
      : Xw(Converter::toVector3d(Xw_)), cam_idx(cam_idx_) {}

  virtual bool read(std::istream &is) { return false; }
  virtual bool write(std::ostream &os) const { return false; }

  void computeError() {
    const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[0]);
    const Eigen::Vector3d obs(_measurement);
    _error = obs - VPose->estimate().ProjectStereo(Xw, cam_idx);
  }

  virtual void linearizeOplus();

  Eigen::Matrix<double, 6, 6> GetHessian() {
    linearizeOplus();
    return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
  }

public:
  const Eigen::Vector3d Xw; // 3D point coordinates
  const int cam_idx;
};
} // namespace RVWO

#endif // GEOMETRY_G2OTYPES_H
