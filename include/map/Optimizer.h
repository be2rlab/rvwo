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

#ifndef MAP_OPTIMIZER_H
#define MAP_OPTIMIZER_H

#include <tbb/parallel_for.h>
#include <cmath>

#include "Frame.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapPoint.h"
#include "OdomFactor.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

namespace RVWO {

class LoopClosing;
/**
 * @brief This class is used to optimize the map
 */
class Optimizer {
 public:
  /** @brief Sort by value
   * @param a: pair of MapPoint and int
   * @param b: pair of MapPoint and int
   * @return true if a.second < b.second
   */
  static bool sortByVal(const pair<MapPoint*, int>& a, const pair<MapPoint*, int>& b) {
    return (a.second < b.second);
  }
  /** @brief Bundle Adjustment 
   * @param vpKF: vector of KeyFrame pointers
   * @param vpMP: vector of MapPoint pointers
   * @param nIterations: number of iterations
   * @param pbStopFlag: pointer to a flag to stop the optimization
   * @param nLoopKF: number of loop keyframes
   * @param bRobust: flag to indicate if the optimization is robust
   */
  void static BundleAdjustment(const std::vector<KeyFrame*>& vpKF,
                               const std::vector<MapPoint*>& vpMP,
                               int nIterations = 5, bool* pbStopFlag = nullptr,
                               unsigned long nLoopKF = 0, bool bRobust = true);

  /** @brief Global Bundle Adjustment
   * @param pMap: pointer to the map
   * @param nIterations: number of iterations
   * @param pbStopFlag: pointer to a flag to stop the optimization
   * @param nLoopKF: number of loop keyframes
   * @param bRobust: flag to indicate if the optimization is robust
   */     
  void static GlobalBundleAdjustemnt(Map* pMap, int nIterations = 5,
                                     bool* pbStopFlag = nullptr,
                                     unsigned long nLoopKF = 0,
                                     bool bRobust = true);

  /** @brief Local Bundle Adjustment
   * @param pKF: pointer to the keyframe
   * @param pbStopFlag: pointer to a flag to stop the optimization
   * @param pMap: pointer to the map
   * @param num_fixedKF: number of fixed keyframes
   * @param num_OptKF: number of optimized keyframes
   * @param num_MPs: number of map points
   * @param num_edges: number of edges
   */
  void static LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag, Map* pMap,
                                    int& num_fixedKF, int& num_OptKF,
                                    int& num_MPs, int& num_edges);
  /** @brief Pose Optimization
   * @param pFrame: pointer to the frame
  */
  int static PoseOptimization(Frame* pFrame);
  /** @brief getAlfa 
   * @param prob: probability
   * @param err: error
  */
  double static getAlfa(double prob, double err);

  float static Px(bool x_cur, bool u_cur, bool x_last);

  /** @brief Label to Prob 
   * @param kpUn: keypoint
   * @param kpSegVal: keypoint segment value
   * @param pMP: pointer to the map point
   * @param pF: pointer to the frame
   */
  void static LabelToProb(const cv::KeyPoint &kpUn, const float &kpSegVal, MapPoint *pMP, Frame *pF);

  /** @brief Label to Prob 
   * @param kpUn: keypoint
   * @param kpSegVal: keypoint segment value
   * @param pMP: pointer to the map point
   * @param pKF: pointer to the keyframe
   */
  void static LabelToProb(const cv::KeyPoint &kpUn, const float &kpSegVal, MapPoint *pMP, KeyFrame *pKF);

  // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise
  // (mono)
  /** @brief Optimize Essential Graph
   * @param pMap: pointer to the map
   * @param pLoopKF: pointer to the loop keyframe
   * @param pCurKF: pointer to the current keyframe
   * @param NonCorrectedSim3: non corrected Sim3
   * @param CorrectedSim3: corrected Sim3
   * @param LoopConnections: loop connections
   * @param bFixScale: flag to indicate if the scale is fixed
   */
  void static OptimizeEssentialGraph(
      Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
      const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
      const LoopClosing::KeyFrameAndPose& CorrectedSim3,
      const map<KeyFrame*, set<KeyFrame*> >& LoopConnections,
      const bool& bFixScale);

  /** @brief Optimize Essential Graph
   * @param pCurKF: pointer to the current keyframe
   * @param vpFixedKFs: vector of fixed keyframes
   * @param vpFixedCorrectedKFs: vector of fixed corrected keyframes
   * @param vpNonFixedKFs: vector of non fixed keyframes
   * @param vpNonCorrectedMPs: vector of non corrected map points
   */
  void static OptimizeEssentialGraph(KeyFrame* pCurKF,
                                     vector<KeyFrame*>& vpFixedKFs,
                                     vector<KeyFrame*>& vpFixedCorrectedKFs,
                                     vector<KeyFrame*>& vpNonFixedKFs,
                                     vector<MapPoint*>& vpNonCorrectedMPs);

  /** @brief Optimize Essential Graph 4DoF
   * @param pMap: pointer to the map
   * @param pLoopKF: pointer to the loop keyframe
   * @param pCurKF: pointer to the current keyframe
   * @param NonCorrectedSim3: non corrected Sim3
   * @param CorrectedSim3: corrected Sim3
   * @param LoopConnections: loop connections
   */
  void static OptimizeEssentialGraph4DoF(
      Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
      const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
      const LoopClosing::KeyFrameAndPose& CorrectedSim3,
      const map<KeyFrame*, set<KeyFrame*> >& LoopConnections);

  /** @brief Optimize Sim3
   * if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
   * @param pKF1: pointer to the keyframe 1
   * @param pKF2: pointer to the keyframe 2
   * @param vpMatches1: vector of matches
   * @param g2oS12: Sim3
   * @param th2: threshold
   * @param bFixScale: flag to indicate if the scale is fixed
   * @param mAcumHessian: accumulated Hessian
   * @param bAllPoints: flag to indicate if all points are optimized
   */
  static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2,
                          std::vector<MapPoint*>& vpMatches1, g2o::Sim3& g2oS12,
                          float th2, bool bFixScale,
                          Eigen::Matrix<double, 7, 7>& mAcumHessian,
                          bool bAllPoints = false);

  /** @brief Local BA in welding area when two maps are merged
   * @param pMainKF: pointer to the main keyframe
   * @param vpAdjustKF: vector of keyframes to adjust
   * @param vpFixedKF: vector of fixed keyframes
   * @param pbStopFlag: pointer to a flag to stop the optimization
   */
  void static LocalBundleAdjustment(KeyFrame* pMainKF,
                                    vector<KeyFrame*> vpAdjustKF,
                                    const vector<KeyFrame*>& vpFixedKF,
                                    bool* pbStopFlag);

  /** @brief  Marginalize block element (start:end,start:end). Perform Schur complement.
   * Marginalized elements are filled with zeros.
   * @param H: Hessian
   * @param start: start index
   * @param end: end index
   */
  static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd& H, const int& start,
                                     const int& end);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

}  // namespace RVWO

#endif  // MAP_OPTIMIZER_H
