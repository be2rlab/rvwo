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

#ifndef GEOMETRY_GEOMETRIC_TOOLS_H
#define GEOMETRY_GEOMETRIC_TOOLS_H

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>

namespace RVWO {

class KeyFrame;

class GeometricTools {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief Compute the Fundamental matrix between KF1 and KF2
   * @param pKF1: pointer to the first keyframe
   * @param pKF2: pointer to the second keyframe
   * @return Fundamental matrix
   */
  static Eigen::Matrix3f ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);

  /** @brief Triangulate point with KF1 and KF2
   * @param x_c1: point in camera frame of KF1
   * @param x_c2: point in camera frame of KF2
   * @param Tc1w: transformation matrix from camera to world of KF1
   * @param Tc2w: transformation matrix from camera to world of KF2
   * @param x3D: 3D point
   * @return true if triangulation is successful
   */
  static bool Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2,
                          Eigen::Matrix<float, 3, 4> &Tc1w,
                          Eigen::Matrix<float, 3, 4> &Tc2w,
                          Eigen::Vector3f &x3D);

  /** @brief Check if two matrices are equal
   * @param cvMat: OpenCV matrix
   * @param eigMat: Eigen matrix
   * @return true if the matrices are equal
   */
  template <int rows, int cols>
  static bool CheckMatrices(const cv::Mat &cvMat,
                            const Eigen::Matrix<float, rows, cols> &eigMat) {
    const float epsilon = 1e-3;
    // std::cout << cvMat.cols - cols << cvMat.rows - rows << std::endl;
    if (rows != cvMat.rows || cols != cvMat.cols) {
      std::cout << "wrong cvmat size\n";
      return false;
    }
    for (int i = 0; i < rows; i++)
      for (int j = 0; j < cols; j++)
        if ((cvMat.at<float>(i, j) > (eigMat(i, j) + epsilon)) ||
            (cvMat.at<float>(i, j) < (eigMat(i, j) - epsilon))) {
          std::cout << "cv mat:\n" << cvMat << std::endl;
          std::cout << "eig mat:\n" << eigMat << std::endl;
          return false;
        }
    return true;
  }

  /** @brief Check if two matrices are equal
   * @param eigMat1: Eigen matrix 1
   * @param eigMat2: Eigen matrix 2
   * @return true if the matrices are equal
   */
  template <typename T, int rows, int cols>
  static bool CheckMatrices(const Eigen::Matrix<T, rows, cols> &eigMat1,
                            const Eigen::Matrix<T, rows, cols> &eigMat2) {
    const float epsilon = 1e-3;
    for (int i = 0; i < rows; i++)
      for (int j = 0; j < cols; j++)
        if ((eigMat1(i, j) > (eigMat2(i, j) + epsilon)) ||
            (eigMat1(i, j) < (eigMat2(i, j) - epsilon))) {
          std::cout << "eig mat 1:\n" << eigMat1 << std::endl;
          std::cout << "eig mat 2:\n" << eigMat2 << std::endl;
          return false;
        }
    return true;
  }
};

} // namespace RVWO

#endif // GEOMETRY_GEOMETRIC_TOOLS_H
