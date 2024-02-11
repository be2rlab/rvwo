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

#ifndef MAP_LOOPCLOSING_H
#define MAP_LOOPCLOSING_H

#include <boost/algorithm/string.hpp>
#include <mutex>
#include <thread>

#include "Atlas.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "ORBVocabulary.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Tracking.h"

namespace RVWO {

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;

class LoopClosing {
public:
  typedef pair<set<KeyFrame *>, int> ConsistentGroup;
  typedef map<KeyFrame *, g2o::Sim3, std::less<KeyFrame *>,
              Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3>>>
      KeyFrameAndPose;

public:
  /** @brief Constructor
   * @param pAtlas: pointer to the atlas
   * @param pDB: pointer to the keyframe database
   * @param pVoc: pointer to the ORB vocabulary
   * @param kernel: kernel type
   * @param bFixScale: flag to indicate if the scale is fixed
   * @param bActiveLC: flag to indicate if the loop closing is active
   */
  LoopClosing(Atlas *pAtlas, KeyFrameDatabase *pDB, ORBVocabulary *pVoc,
              const int kernel, const bool bFixScale, const bool bActiveLC);

  /** @brief Set the tracker
   * @param pTracker: pointer to the tracker
   */
  void SetTracker(Tracking *pTracker);

  /** @brief Set the local mapper
   * @param pLocalMapper: pointer to the local mapper
   * */
  void SetLocalMapper(LocalMapping *pLocalMapper);

  // ** Main function ** //
  /** @brief Main function of the loop closing thread
   */
  void Run();
  /** @brief Insert Key frame in the loop closing thread
   * @param pKF: pointer to the keyframe
   */
  void InsertKeyFrame(KeyFrame *pKF);

  /** @brief Request Reset of the loop closing thread */
  void RequestReset();
  /** @brief Request Reset of the loop closing thread of a specific map
   * @param pMap: pointer to the map
   */
  void RequestResetActiveMap(Map *pMap);

  /** @brief This function will run global bundle adjustment in a separate
   * thread
   * @param pActiveMap: pointer to the active map
   * @param nLoopKF: number of keyframes
   */
  void RunGlobalBundleAdjustment(Map *pActiveMap, unsigned long nLoopKF);

  /** @brief check if GBA is running
   * @return true if GBA is running
   */
  bool isRunningGBA() {
    unique_lock<std::mutex> lock(mMutexGBA);
    return mbRunningGBA;
  }
  /** Check if GBA is finished
   * @return true if GBA is finished
   */
  bool isFinishedGBA() {
    unique_lock<std::mutex> lock(mMutexGBA);
    return mbFinishedGBA;
  }
  /** @brief request finish */
  void RequestFinish();
  /** @brief check if finished
   * @return true if finished
   */
  bool isFinished();

  Viewer *mpViewer;

#ifdef REGISTER_TIMES

  vector<double> vdDataQuery_ms;
  vector<double> vdEstSim3_ms;
  vector<double> vdPRTotal_ms;

  vector<double> vdMergeMaps_ms;
  vector<double> vdWeldingBA_ms;
  vector<double> vdMergeOptEss_ms;
  vector<double> vdMergeTotal_ms;
  vector<int> vnMergeKFs;
  vector<int> vnMergeMPs;
  int nMerges;

  vector<double> vdLoopFusion_ms;
  vector<double> vdLoopOptEss_ms;
  vector<double> vdLoopTotal_ms;
  vector<int> vnLoopKFs;
  int nLoop;

  vector<double> vdGBA_ms;
  vector<double> vdUpdateMap_ms;
  vector<double> vdFGBATotal_ms;
  vector<int> vnGBAKFs;
  vector<int> vnGBAMPs;
  int nFGBA_exec;
  int nFGBA_abort;

#endif

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  bool CheckNewKeyFrames();

  // Methods to implement the new place recognition algorithm
  bool NewDetectCommonRegions();
  bool DetectAndReffineSim3FromLastKF(KeyFrame *pCurrentKF,
                                      KeyFrame *pMatchedKF, g2o::Sim3 &gScw,
                                      int &nNumProjMatches,
                                      std::vector<MapPoint *> &vpMPs,
                                      std::vector<MapPoint *> &vpMatchedMPs);
  bool DetectCommonRegionsFromBoW(std::vector<KeyFrame *> &vpBowCand,
                                  KeyFrame *&pMatchedKF,
                                  KeyFrame *&pLastCurrentKF, g2o::Sim3 &g2oScw,
                                  int &nNumCoincidences,
                                  std::vector<MapPoint *> &vpMPs,
                                  std::vector<MapPoint *> &vpMatchedMPs);
  bool DetectCommonRegionsFromLastKF(KeyFrame *pCurrentKF, KeyFrame *pMatchedKF,
                                     g2o::Sim3 &gScw, int &nNumProjMatches,
                                     std::vector<MapPoint *> &vpMPs,
                                     std::vector<MapPoint *> &vpMatchedMPs);
  int FindMatchesByProjection(KeyFrame *pCurrentKF, KeyFrame *pMatchedKFw,
                              g2o::Sim3 &g2oScw,
                              set<MapPoint *> &spMatchedMPinOrigin,
                              vector<MapPoint *> &vpMapPoints,
                              vector<MapPoint *> &vpMatchedMapPoints);

  void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap,
                     vector<MapPoint *> &vpMapPoints);
  void SearchAndFuse(const vector<KeyFrame *> &vConectedKFs,
                     vector<MapPoint *> &vpMapPoints);

  void CorrectLoop();

  void MergeLocal();
  void MergeLocal2();

  void CheckObservations(set<KeyFrame *> &spKFsMap1,
                         set<KeyFrame *> &spKFsMap2);

  void ResetIfRequested();
  bool mbResetRequested;
  bool mbResetActiveMapRequested;
  Map *mpMapToReset;
  std::mutex mMutexReset;

  bool CheckFinish();
  void SetFinish();
  bool mbFinishRequested;
  bool mbFinished;
  std::mutex mMutexFinish;

  Atlas *mpAtlas;
  Tracking *mpTracker;

  KeyFrameDatabase *mpKeyFrameDB;
  ORBVocabulary *mpORBVocabulary;
  const int mKernel;

  LocalMapping *mpLocalMapper;

  std::list<KeyFrame *> mlpLoopKeyFrameQueue;

  std::mutex mMutexLoopQueue;

  // Loop detector parameters
  float mnCovisibilityConsistencyTh;

  // Loop detector variables
  KeyFrame *mpCurrentKF;
  KeyFrame *mpLastCurrentKF;
  KeyFrame *mpMatchedKF;
  std::vector<ConsistentGroup> mvConsistentGroups;
  std::vector<KeyFrame *> mvpEnoughConsistentCandidates;
  std::vector<KeyFrame *> mvpCurrentConnectedKFs;
  std::vector<MapPoint *> mvpCurrentMatchedPoints;
  std::vector<MapPoint *> mvpLoopMapPoints;
  cv::Mat mScw;
  g2o::Sim3 mg2oScw;

  //-------
  Map *mpLastMap;

  bool mbLoopDetected;
  int mnLoopNumCoincidences;
  int mnLoopNumNotFound;
  KeyFrame *mpLoopLastCurrentKF;
  g2o::Sim3 mg2oLoopSlw;
  g2o::Sim3 mg2oLoopScw;
  KeyFrame *mpLoopMatchedKF;
  std::vector<MapPoint *> mvpLoopMPs;
  std::vector<MapPoint *> mvpLoopMatchedMPs;
  bool mbMergeDetected;
  int mnMergeNumCoincidences;
  int mnMergeNumNotFound;
  KeyFrame *mpMergeLastCurrentKF;
  g2o::Sim3 mg2oMergeSlw;
  g2o::Sim3 mg2oMergeSmw;
  g2o::Sim3 mg2oMergeScw;
  KeyFrame *mpMergeMatchedKF;
  std::vector<MapPoint *> mvpMergeMPs;
  std::vector<MapPoint *> mvpMergeMatchedMPs;
  std::vector<KeyFrame *> mvpMergeConnectedKFs;

  g2o::Sim3 mSold_new;
  //-------

  long unsigned int mLastLoopKFid;

  // Variables related to Global Bundle Adjustment
  bool mbRunningGBA;
  bool mbFinishedGBA;
  bool mbStopGBA;
  std::mutex mMutexGBA;
  std::thread *mpThreadGBA;

  // Fix scale in the stereo/RGB-D case
  bool mbFixScale;

  bool mnFullBAIdx;

  vector<double> vdPR_CurrentTime;
  vector<double> vdPR_MatchedTime;
  vector<int> vnPR_TypeRecogn;

  // DEBUG
  string mstrFolderSubTraj;
  int mnNumCorrection;
  int mnCorrectionGBA;

  // To (de)activate LC
  bool mbActiveLC = true;

#ifdef REGISTER_LOOP
  string mstrFolderLoop;
#endif
};

} // namespace RVWO

#endif // MAP_LOOPCLOSING_H
