/**
 * This file modified from ORB-SLAM3
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

#ifndef UTILS_CONVERTER_H
#define UTILS_CONVERTER_H

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "Thirdparty/g2o/g2o/types/se2.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

namespace RVWO {
class Converter {
public:
  /**
   * @brief convert a cv::Mat Descriptor to a vector of cv::Mat Descriptors
   * @param Descriptors
   * @return std::vector<cv::Mat>
   */
  static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

  // ** SE2 Geometry functions ** //

  /**
   * @brief convert a cv::Mat to a g2o::SE2
   * @param cvT pose
   * @return g2o::SE2 pose
   */
  static g2o::SE2 toSE2(const cv::Mat &cvT);

  /** @brief normalize an angle
   * @param theta angle
   * @return double
   */
  static double normalize_angle(double theta);

  // ** SE3 Geometry functions ** //
  /**
   * @brief get skew matrix of a vector
   * @param v vector
   * @return Eigen::Matrix3d
   */
  static Eigen::Matrix3d skew(const Vector3d &v);

  /**
   * @brief convert cv::Mat to g2o::SE3Quat
   * @param cvT
   * @return g2o::SE3Quat
   */
  static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);

  /**
   * @brief convert g2o::Sim3 to g2o::SE3Quat
   * @param g2o::Sim3
   * @return g2o::SE3Quat
   */
  static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

  // ** to cv::Mat conversion functions ** //
  /** @brief convert to cv::Mat from g2o::SE3Quat
   * @param SE3
   * @return cv::Mat
   */
  static cv::Mat toCvMat(const g2o::SE3Quat &SE3);

  /** @brief convert to cv::Mat from g2o::Sim3
   * @param Sim3
   * @return cv::Mat
   */
  static cv::Mat toCvMat(const g2o::Sim3 &Sim3);

  /** @brief convert to cv::Mat from Eigen::Matrix4d
   * @param m
   * @return cv::Mat
   */
  static cv::Mat toCvMat(const Eigen::Matrix<double, 4, 4> &m);

  /** @brief convert to cv::Mat from Eigen::Matrix3d
   * @param m
   * @return cv::Mat
   */
  static cv::Mat toCvMat(const Eigen::Matrix3d &m);

  /** @brief convert to cv::Mat from Eigen::Matrix<double,3,1>
   * @param m
   * @return cv::Mat
   */
  static cv::Mat toCvMat(const Eigen::Matrix<double, 3, 1> &m);

  /** @brief convert to cv::Mat from Eigen::Matrix
   * @param m
   * @return cv::Mat
   */
  static cv::Mat toCvMat(const Eigen::MatrixXd &m);

  /** @brief convert to cv::Mat from R, t
   * @param R rotation matrix
   * @param t translation vector
   * @return cv::Mat
   */
  static cv::Mat toCvSE3(const Eigen::Matrix<double, 3, 3> &R,
                         const Eigen::Matrix<double, 3, 1> &t);

  /** @brief get skew symmetric matrix from a cv::vector
   * @param v
   * @return cv::Mat
   */
  static cv::Mat tocvSkewMatrix(const cv::Mat &v);

  /** @brief inverse of cv::Mat SE3
   * @param SE3
   * @return cv::Mat
   */
  static cv::Mat invSE3(const cv::Mat &SE3);
  // ** to Eigen conversion functions ** //
  /** @brief convert to Eigen::Matrix from cv::Mat
   * @param cvVector
   * @return Eigen::Matrix<double,3,1>
   */
  static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Mat &cvVector);

  /** @brief convert to Eigen::Matrix from cv::Point3f
   * @param cvPoint
   * @return Eigen::Matrix<double,3,1>
   */
  static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Point3f &cvPoint);

  /** @brief convert to Eigen::Matrix from cvMat3
   * @param cvMat3
   * @return Eigen::Matrix<double,3,3>
   */
  static Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat &cvMat3);

  /** @brief convert to Eigen::Matrix from cvMat4
   * @param cvMat4
   * @return Eigen::Matrix<double,4,4>
   */
  static Eigen::Matrix<double, 4, 4> toMatrix4d(const cv::Mat &cvMat4);

  // ** to Quaternion conversion functions ** //
  /** @brief convert to Quaternion from cv::Mat
   * @param M
   * @return std::vector<float>
   */
  static std::vector<float> toQuaternion(const cv::Mat &M);

  // ** to Euler conversion functions ** //
  /** @brief convert to Euler from cv::Mat
   * @param R
   * @return bool if the matrix is a rotation matrix
   */
  static bool isRotationMatrix(const cv::Mat &R);

  /** @brief convert to Euler from cv::Mat
   * @param R
   * @return std::vector<float>
   */
  static std::vector<float> toEuler(const cv::Mat &R);
};

} // namespace RVWO

#endif // UTILS_CONVERTER_H
