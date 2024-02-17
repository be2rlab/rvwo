#ifndef GEOMETRY_ODOMTYPES_H
#define GEOMETRY_ODOMTYPES_H

#include "sophus/se2.hpp"
#include "sophus/se3.hpp"
#include <Eigen/Core>

namespace RVWO {
namespace ODOM {
/** @brief
 * Measurement class
 */
class Meas {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  /** @brief Constructor
   * @param meas_: measurement
   * @param timestamp: timestamp
   */

  Meas(const Sophus::SE2f &meas_, const double &timestamp)
      : meas(meas_), t(timestamp) {}

public:
  Sophus::SE2f meas;
  double t;
};
/** @brief
 * Preintegrated class
 */
class Preintegrated {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief Constructor
   * @param vNoises: noise vector
   * @param Tbo_: transformation matrix from body to odometry
   */
  Preintegrated(Eigen::Vector3d &vNoises, Sophus::SE3f &Tbo_);
  /** @brief Integrate a new measurement
   * @param meas: measurement
   */
  Preintegrated(Preintegrated *pOdomPre);
  /** @brief Integrate a new measurement
   * @param meas: measurement
   */
  void IntegratedNewMeasurement(Sophus::SE2f &PrevMeas, Sophus::SE2f &CurrMeas);

public:
  Sophus::SE2f Meas;
  Eigen::Matrix3f Cov, Cov1;
  double NoiseX, NoiseY, NoiseRotZ;
  Sophus::SE3f Tbo;
  Eigen::Vector3f Delta;
  std::vector<Sophus::SE2f> mvMeasurements;
};
} // namespace ODOM
} // namespace RVWO
#endif // GEOMETRY_ODOMTYPES_H
