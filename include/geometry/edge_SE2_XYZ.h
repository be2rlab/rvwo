/**
 * This file is modified part of se2lam
 *
 * Copyright (C) Fan ZHENG (github.com/izhengfan), Hengbo TANG
 * (github.com/hbtang)
 *
 * Modified by Long Vuong (github.com/hellovuong) (2021)
 *
 * Added EdgeSE2XYZOnlyPose (unary edge to optimize only the camera pose)
 * Added EdgeSE2XYZ         (project using focal_length in x,y direction)
 */

#ifndef UTILS_EDGE_SE2_XYZ_H
#define UTILS_EDGE_SE2_XYZ_H

#pragma once

#include "third_party/g2o/g2o/core/base_binary_edge.h"
#include "third_party/g2o/g2o/core/base_vertex.h"
#include "third_party/g2o/g2o/core/eigen_types.h"
#include "third_party/g2o/g2o/types/se2.h"
#include "third_party/g2o/g2o/types/types_six_dof_expmap.h"
#include "third_party/g2o/g2o/types/vertex_se2.h"

#include "geometry/camera_models/geometric_camera.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define CUSTOMIZE_JACOBIAN_SE2XYZ

namespace g2o {
typedef Eigen::Matrix<double, 2, 3> Matrix23d;
typedef Eigen::Matrix<double, 3, 2> Matrix32d;

/** @brief
 * EdgeSE2XYZ is a binary edge that connects a VertexSE2 and a VertexSBAPointXYZ
 * It is used to optimize the camera pose and the 3D point
 * The error is the difference between the projected 3D point and the observed
 * 2D point The linearization is done using the Jacobian of the projection
 * function
 */
class EdgeSE2XYZ
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSE2,
                                 g2o::VertexSBAPointXYZ> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // Constructor
  EdgeSE2XYZ();

  // Destructor
  ~EdgeSE2XYZ();

  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;

  /** @brief Compute Error in the edge. */
  void computeError();

  /** @brief Check if the depth is positive.
   * @return bool
   */
  bool isDepthPositive();

  /** @brief calculate the jacobians for the optimization. */
  virtual void linearizeOplus();

  /** @brief project point
   * @param trans_xyz 3D point in local camera coordinate system
   * @return 2D pixel position of the 3D point.
   */
  Vector2d cam_project(const Vector3d &trans_xyz) const;

  RVWO::GeometricCamera *pCamera;
  double fx, fy, cx, cy;
  g2o::SE3Quat Tcb, Tbc;
};

/** @brief EdgeSE2XYZOnlyPose is a unary edge that connects a VertexSE2
 * It is used to optimize the camera pose only
 * The error is the difference between the projected 3D point and the observed
 * 2D point The linearization is done using the Jacobian of the projection
 * function
 */
class EdgeSE2XYZOnlyPose
    : public g2o::BaseUnaryEdge<2, Vector2d, g2o::VertexSE2> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  // Constructor
  EdgeSE2XYZOnlyPose();

  // Descructor
  ~EdgeSE2XYZOnlyPose();

  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;

  /** @brief Calcualte the error of the edge in optimization*/
  void computeError();

  /** @brief Differenetaite the error fucntion over the optimization parameters.
   */
  virtual void linearizeOplus();

  /** @brief Project 3D point from local camera frame to image plane.
   * @param trans_xyz 3D camera position in local camera frame
   * @return 2D pixel of point in image plane
   */
  Vector2d cam_project(const Vector3d &trans_xyz) const;

  /** @brief check valid depth
   * @return bool
   */
  bool isDepthPositive();

  Vector3d Xw;
  RVWO::GeometricCamera *pCamera;
  double fx, fy, cx, cy;
  g2o::SE3Quat Tcb, Tbc;
};

/** @brief PreEdgeSE2 is a binary edge that connects two VertexSE2
 * It is used to optimize the relative pose between two camera poses
 * The error is the difference between the relative pose and the observed
 * relative pose The linearization is done using the Jacobian of the relative
 * pose
 */
class PreEdgeSE2
    : public g2o::BaseBinaryEdge<3, Vector3D, VertexSE2, VertexSE2> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  PreEdgeSE2() {}

  /** @brief calculate the error of this edge in the optimization. */
  void computeError() {
    const VertexSE2 *v1 = static_cast<const VertexSE2 *>(_vertices[0]);
    const VertexSE2 *v2 = static_cast<const VertexSE2 *>(_vertices[1]);
    Matrix2D Ri = v1->estimate().rotation().toRotationMatrix();
    Vector2D ri = v1->estimate().translation();
    double ai = v1->estimate().rotation().angle();
    double aj = v2->estimate().rotation().angle();
    Vector2D rj = v2->estimate().translation();

    _error.head<2>() = Ri.transpose() * (rj - ri) - _measurement.head<2>();
    _error[2] = aj - ai - _measurement[2];
  }

  /** @brief Calculation of the Jacobians*/
  virtual void linearizeOplus() {
    const VertexSE2 *v1 = static_cast<const VertexSE2 *>(_vertices[0]);
    const VertexSE2 *v2 = static_cast<const VertexSE2 *>(_vertices[1]);
    g2o::Matrix2D Ri = v1->estimate().rotation().toRotationMatrix();
    Vector2D ri = v1->estimate().translation();
    Vector2D rj = v2->estimate().translation();
    Vector2D rij = rj - ri;
    Vector2D rij_x(-rij[1], rij[0]);

    _jacobianOplusXi.block<2, 2>(0, 0) = -Ri.transpose();
    _jacobianOplusXi.block<2, 1>(0, 2) = -Ri.transpose() * rij_x;
    _jacobianOplusXi.block<1, 2>(2, 0).setZero();
    _jacobianOplusXi(2, 2) = -1;

    _jacobianOplusXj.setIdentity();
    _jacobianOplusXj.block<2, 2>(0, 0) = Ri.transpose();
  }
  virtual bool read(std::istream &is) { return true; }
  virtual bool write(std::ostream &os) const { return true; }
};
} // namespace g2o

#endif
