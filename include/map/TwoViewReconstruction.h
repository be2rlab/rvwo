/**
 * This file is part of ORB-SLAM3
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

#ifndef MAP_TwoViewReconstruction_H
#define MAP_TwoViewReconstruction_H

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <unordered_set>

namespace RVWO {
/** @brief Class for two-view reconstruction
 */
class TwoViewReconstruction {
  typedef std::pair<int, int> Match;

public:
  /** @brief Constructor
   * @param k: calibration matrix
   * @param sigma: standard deviation
   * @param iterations: number of iterations
   * @return
   * @details This function initializes the class with the calibration matrix,
   * the standard deviation and the number of iterations
   */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TwoViewReconstruction(const Eigen::Matrix3f &k, float sigma = 1.0,
                        int iterations = 200);

  /** @brief Computes in parallel a fundamental matrix and a homography Selects
   * a model and tries to recover the motion and the structure from motion
   * @param vKeys1: keypoints from the first frame
   * @param vKeys2: keypoints from the second frame
   * @param vMatches12: matches between the first and the second frame
   * @param T21: transformation matrix from the second frame to the first frame
   * @param vP3D: 3D points
   * @param vbTriangulated: vector of booleans indicating if the point is
   * triangulated
   * @return true if the reconstruction is successful
   * @details This function computes in parallel a fundamental matrix and a
   * homography. Then, it selects a model and tries to recover the motion and
   * the structure from motion
   */
  bool Reconstruct(const std::vector<cv::KeyPoint> &vKeys1,
                   const std::vector<cv::KeyPoint> &vKeys2,
                   const std::vector<int> &vMatches12, Sophus::SE3f &T21,
                   std::vector<cv::Point3f> &vP3D,
                   std::vector<bool> &vbTriangulated);

private:
  void FindHomography(std::vector<bool> &vbMatchesInliers, float &score,
                      Eigen::Matrix3f &H21);
  void FindFundamental(std::vector<bool> &vbInliers, float &score,
                       Eigen::Matrix3f &F21);

  Eigen::Matrix3f ComputeH21(const std::vector<cv::Point2f> &vP1,
                             const std::vector<cv::Point2f> &vP2);
  Eigen::Matrix3f ComputeF21(const std::vector<cv::Point2f> &vP1,
                             const std::vector<cv::Point2f> &vP2);

  float CheckHomography(const Eigen::Matrix3f &H21, const Eigen::Matrix3f &H12,
                        std::vector<bool> &vbMatchesInliers, float sigma);

  float CheckFundamental(const Eigen::Matrix3f &F21,
                         std::vector<bool> &vbMatchesInliers, float sigma);

  bool ReconstructF(std::vector<bool> &vbMatchesInliers, Eigen::Matrix3f &F21,
                    Eigen::Matrix3f &K, Sophus::SE3f &T21,
                    std::vector<cv::Point3f> &vP3D,
                    std::vector<bool> &vbTriangulated, float minParallax,
                    int minTriangulated);

  bool ReconstructH(std::vector<bool> &vbMatchesInliers, Eigen::Matrix3f &H21,
                    Eigen::Matrix3f &K, Sophus::SE3f &T21,
                    std::vector<cv::Point3f> &vP3D,
                    std::vector<bool> &vbTriangulated, float minParallax,
                    int minTriangulated);

  void Normalize(const std::vector<cv::KeyPoint> &vKeys,
                 std::vector<cv::Point2f> &vNormalizedPoints,
                 Eigen::Matrix3f &T);

  int CheckRT(const Eigen::Matrix3f &R, const Eigen::Vector3f &t,
              const std::vector<cv::KeyPoint> &vKeys1,
              const std::vector<cv::KeyPoint> &vKeys2,
              const std::vector<Match> &vMatches12,
              std::vector<bool> &vbMatchesInliers, const Eigen::Matrix3f &K,
              std::vector<cv::Point3f> &vP3D, float th2,
              std::vector<bool> &vbGood, float &parallax);

  void DecomposeE(const Eigen::Matrix3f &E, Eigen::Matrix3f &R1,
                  Eigen::Matrix3f &R2, Eigen::Vector3f &t);

  // Keypoints from Reference Frame (Frame 1)
  std::vector<cv::KeyPoint> mvKeys1;

  // Keypoints from Current Frame (Frame 2)
  std::vector<cv::KeyPoint> mvKeys2;

  // Current Matches from Reference to Current
  std::vector<Match> mvMatches12;
  std::vector<bool> mvbMatched1;

  // Calibration
  Eigen::Matrix3f mK;

  // Standard Deviation and Variance
  float mSigma, mSigma2;

  // Ransac max iterations
  int mMaxIterations;

  // Ransac sets
  std::vector<std::vector<size_t>> mvSets;
};

} // namespace RVWO

#endif // MAP_TwoViewReconstruction_H
