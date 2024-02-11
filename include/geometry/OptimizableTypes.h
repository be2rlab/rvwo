/**
 * This file is modified part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
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

#ifndef RVWO_OPTIMIZABLETYPES_H
#define RVWO_OPTIMIZABLETYPES_H

#include <Thirdparty/g2o/g2o/types/sim3.h>
#include <Thirdparty/g2o/g2o/types/types_six_dof_expmap.h>
#include <include/CameraModels/GeometricCamera.h>

#include <Eigen/Geometry>

#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"

namespace RVWO {
  /** @brief EdgeSE3ProjectXYZOnlyPose class
   * This class is used to define the edge between a vertex of type g2o::VertexSE3Expmap and a 2D point
   */
class EdgeSE3ProjectXYZOnlyPose
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief Constructor of the class
   */
  EdgeSE3ProjectXYZOnlyPose() = default;
  /** @brief Read the edge from a stream
   * @param is: input stream
   * @return true if the edge is read successfully
   */
  bool read(std::istream& is);

  /** @brief Write the edge to a stream
   * @param os: output stream
   * @return true if the edge is written successfully
   */
  bool write(std::ostream& os) const;

  /** @brief Compute the error of the edge
   */
  void computeError() {
    const g2o::VertexSE3Expmap* v1 =
        static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    Eigen::Vector2d obs(_measurement);
    _error = obs - pCamera->project(v1->estimate().map(Xw));
  }
  /** @brief Linearize the edge
   */
  virtual void linearizeOplus();

  Eigen::Vector3d Xw;
  GeometricCamera* pCamera;
};

/** @brief EdgeSE3ProjectXYZOnlyPoseToBody class
 * This class is used to define the edge between a vertex of type g2o::VertexSE3Expmap and a 2D point
 */
class EdgeSE3ProjectXYZOnlyPoseToBody
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief Constructor of the class
   */
  EdgeSE3ProjectXYZOnlyPoseToBody() = default;
  /** @brief Read the edge from a stream
   * @param is: input stream
   * @return true if the edge is read successfully
   */
  bool read(std::istream& is);

  /** @brief Write the edge to a stream
   * @param os: output stream
   * @return true if the edge is written successfully
   */
  bool write(std::ostream& os) const;
  /** @brief Compute the error of the edge
   */
  void computeError() {
    const g2o::VertexSE3Expmap* v1 =
        static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    Eigen::Vector2d obs(_measurement);
    _error = obs - pCamera->project((mTrl * v1->estimate()).map(Xw));
  }
  /** @brief Linearize the edge
   */
  virtual void linearizeOplus();

  Eigen::Vector3d Xw;
  GeometricCamera* pCamera;

  g2o::SE3Quat mTrl;
};

/** @brief EdgeSE3ProjectXYZ class
 * This class is used to define the edge between a vertex of type g2o::VertexSE3Expmap and a 2D point
 */
class EdgeSE3ProjectXYZ
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ,
                                 g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief Constructor of the class
   */
  EdgeSE3ProjectXYZ();
  /** @brief Read the edge from a stream
   * @param is: input stream
   * @return true if the edge is read successfully
   */
  bool read(std::istream& is);
  /** @brief Write the edge to a stream
   * @param os: output stream
   * @return true if the edge is written successfully
   */
  bool write(std::ostream& os) const;
  /** @brief Compute the error of the edge
   */
  void computeError() {
    const g2o::VertexSE3Expmap* v1 =
        static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* v2 =
        static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector2d obs(_measurement);
    _error = obs - pCamera->project(v1->estimate().map(v2->estimate()));
  }
  /** @brief is Depth Positive
   * @return true if the depth is positive
   */
  bool isDepthPositive() {
    const g2o::VertexSE3Expmap* v1 =
        static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* v2 =
        static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
    return ((v1->estimate().map(v2->estimate()))(2) > 0.0);
  }
  /** @brief Linearize the edge
   */
  virtual void linearizeOplus();

  GeometricCamera* pCamera;
};

/** @brief EdgeSE3ProjectXYZToBody class
 * This class is used to define the edge between a vertex of type g2o::VertexSE3Expmap and a 2D point
 */
class EdgeSE3ProjectXYZToBody
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ,
                                 g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief Constructor of the class
   */
  EdgeSE3ProjectXYZToBody();
  /** @brief Read the edge from a stream
   * @param is: input stream
   * @return true if the edge is read successfully
   */
  bool read(std::istream& is);
  /** @brief Write the edge to a stream
   * @param os: output stream
   * @return true if the edge is written successfully
   */
  bool write(std::ostream& os) const;
  /** @brief Compute the error of the edge
   */
  void computeError() {
    const g2o::VertexSE3Expmap* v1 =
        static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* v2 =
        static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector2d obs(_measurement);
    _error =
        obs - pCamera->project((mTrl * v1->estimate()).map(v2->estimate()));
  }
  /** @brief is Depth Positive
   * @return true if the depth is positive
   */
  bool isDepthPositive() {
    const g2o::VertexSE3Expmap* v1 =
        static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* v2 =
        static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
    return ((mTrl * v1->estimate()).map(v2->estimate()))(2) > 0.0;
  }
  /** @brief Linearize the edge
   */
  virtual void linearizeOplus();

  GeometricCamera* pCamera;
  g2o::SE3Quat mTrl;
};

/** @brief VertexSim3Expmap class
 * This class is used to define the vertex of type g2o::Sim3
 */
class VertexSim3Expmap : public g2o::BaseVertex<7, g2o::Sim3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief Constructor of the class
   */
  VertexSim3Expmap();
  /** @brief Read the vertex from a stream
   * @param is: input stream
   * @return true if the vertex is read successfully
   */
  virtual bool read(std::istream& is);
  /** @brief Write the vertex to a stream
   * @param os: output stream
   * @return true if the vertex is written successfully
   */
  virtual bool write(std::ostream& os) const;

  /** @brief Set the vertex to the origin
   */
  virtual void setToOriginImpl() { _estimate = g2o::Sim3(); }

  /** @brief oplus operator
   * @param update_: update
   */
  virtual void oplusImpl(const double* update_) {
    Eigen::Map<g2o::Vector7d> update(const_cast<double*>(update_));

    if (_fix_scale) update[6] = 0;

    g2o::Sim3 s(update);
    setEstimate(s * estimate());
  }

  GeometricCamera *pCamera1, *pCamera2;

  bool _fix_scale;
};
/** @brief EdgeSim3ProjectXYZ class
 * This class is used to define the edge between a vertex of type g2o::VertexSBAPointXYZ and a vertex of type VertexSim3Expmap
 */
class EdgeSim3ProjectXYZ
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ,
                                 ORB_SLAM3::VertexSim3Expmap> {
 public:
  /** @brief Constructor of the class
   */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSim3ProjectXYZ();
  /** @brief Read the edge from a stream
   * @param is: input stream
   * @return true if the edge is read successfully
   */
  virtual bool read(std::istream& is);
  /** @brief Write the edge to a stream
   * @param os: output stream
   * @return true if the edge is written successfully
   */
  virtual bool write(std::ostream& os) const;
  /** @brief Compute the error of the edge
   */
  void computeError() {
    const ORB_SLAM3::VertexSim3Expmap* v1 =
        static_cast<const ORB_SLAM3::VertexSim3Expmap*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* v2 =
        static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

    Eigen::Vector2d obs(_measurement);
    _error = obs - v1->pCamera1->project(v1->estimate().map(v2->estimate()));
  }

  // virtual void linearizeOplus();
};

/** @brief EdgeInverseSim3ProjectXYZ class
 * This class is used to define the edge between a vertex of type g2o::VertexSBAPointXYZ and a vertex of type VertexSim3Expmap
 */
class EdgeInverseSim3ProjectXYZ
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ,
                                 VertexSim3Expmap> {
 public:
 /** @brief Constructor of the class
  */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeInverseSim3ProjectXYZ();
  /** @brief Read the edge from a stream
   * @param is: input stream
   * @return true if the edge is read successfully
   */
  virtual bool read(std::istream& is);
  /** @brief Write the edge to a stream
   * @param os: output stream
   * @return true if the edge is written successfully
   */
  virtual bool write(std::ostream& os) const;
  /** @brief Compute the error of the edge
   */
  void computeError() {
    const ORB_SLAM3::VertexSim3Expmap* v1 =
        static_cast<const ORB_SLAM3::VertexSim3Expmap*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* v2 =
        static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

    Eigen::Vector2d obs(_measurement);
    _error = obs - v1->pCamera2->project(
                       (v1->estimate().inverse().map(v2->estimate())));
  }

  // virtual void linearizeOplus();
};

}  // namespace RVWO

#endif  // GEOMETRY_OPTIMIZABLETYPES_H
