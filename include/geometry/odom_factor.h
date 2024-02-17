#ifndef GEOMETRY_ODOMFACTOR_H
#define GEOMETRY_ODOMFACTOR_H

#include <cmath>
#include <map/blocks/frame>
#include <map/blocks/key_frame.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

#include "geometry/g2o_types.h"
#include "utils/converter.h"

#include "third_party/g2o/g2o/core/base_binary_edge.h"
#include "third_party/g2o/g2o/core/base_multi_edge.h"
#include "third_party/g2o/g2o/core/base_unary_edge.h"
#include "third_party/g2o/g2o/core/base_vertex.h"
#include "third_party/g2o/g2o/types/types_sba.h"

namespace RVWO {
typedef Eigen::Matrix<double, 2, 3> Matrix23d;
const static Matrix23d lambd = (Matrix23d() << 1, 0, 0, 0, 1, 0).finished();
const static Eigen::Vector3d e3 = (Eigen::Vector3d() << 0, 0, 1).finished();

/** @brief
 * Vertex for the pose in the odometry factor
 */
class EdgeWOdometry
    : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexPose, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeWOdometry() = default;
  EdgeWOdometry(ODOM::Preintegrated *pInt);
  virtual void computeError();
  virtual void linearizeOplus();
  virtual bool read(std::istream &is) { return true; }
  virtual bool write(std::ostream &os) const { return true; }

  ODOM::Preintegrated *mpInt{};
  Sophus::SE3d Tbo;
};
} // namespace RVWO
#endif // GEOMETRY_ODOMFACTOR_H
