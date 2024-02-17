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

#ifndef MAP_LOCALMAPPING_H
#define MAP_LOCALMAPPING_H

#include "map/blocks/atlas.h"
// #include "DynamicDetector.h"
#include "map/blocks/key_frame.h"
#include "map/blocks/key_frame_database.h"
#include "map/loop_closing.h"
#include "map/tracking.h"
#include "utils/settings.h"

#include <mutex>
#define REGISTER_TIMES

namespace RVWO {

class System;
class Tracking;
class LoopClosing;
class Atlas;
// class DynamicDetector;

class LocalMapping {
public:
  /** @brief Constructor of the local mapping thread
   * @param pSys: pointer to the system
   * @param pAtlas: pointer to the atlas
   * @param bMonocular: flag to indicate if the system is monocular
   * @param kernel: kernel type
   * @param _strSeqName: sequence name
   */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LocalMapping(System *pSys, Atlas *pAtlas, const float bMonocular,
               const int kernel, const string &_strSeqName = std::string());

  /** @brief Set the loop closer
   * @param pLoopCloser: pointer to the loop closer
   */
  void SetLoopCloser(LoopClosing *pLoopCloser);
  /** @brief Set the tracker
   * @param pTracker: pointer to the tracker
   */
  void SetTracker(Tracking *pTracker);

  //  void SetDynamicDetector(DynamicDetector* pDynamicDetector);

  // ** Main function ** //
  /** @brief Main function of the local mapping thread
   */
  void Run();

  // ** KeyFrame Management ** //
  /** @brief Insert a keyframe in the queue
   * @param pKF: pointer to the keyframe
   */
  void InsertKeyFrame(KeyFrame *pKF);
  /** @brief Empty the Queue: this function is called when the thread is
   * stopped
   */
  void EmptyQueue();

  /** @brief Get the keyframe to publish
   * @return pointer to the keyframe
   */
  KeyFrame *GetKeyFrameToPublish();

  // ** Thread Sync ** //
  /** @brief Request the stop of the thread
   */
  void RequestStop();
  /** @brief Request the reset of the thread
   */
  void RequestReset();
  /** @brief Request the reset of the active map
   * @param pMap: pointer to the map
   */
  void RequestResetActiveMap(Map *pMap);
  /** @brief Check if the thread has been requested to stop
   * @return true if the thread has been requested to stop
   */
  bool Stop();
  /** @brief Release */
  void Release();
  /** @brief check if local mapping thread is stopped
   * @return true if the local mapping thread is stopped
   */
  bool isStopped();
  /** @brief check if the thread has been requested to stop
   * @return true if the thread has been requested to stop
   */
  bool stopRequested();
  /** @brief is still accepting new Key Frames*/

  bool AcceptKeyFrames();
  /** @brief Set the flag to accept keyframes
   * @param flag: flag to accept keyframes
   */
  void SetAcceptKeyFrames(bool flag);
  /** @brief Set the flag to stop the thread
   * @param flag: flag to stop the thread
   */
  bool SetNotStop(bool flag);

  /** @brief Set the flag to abort the BA. */
  void InterruptBA();

  /** @brief Request finish*/
  void RequestFinish();
  /** @brief is finished?
   * @return true if the thread has finished
   */
  bool isFinished();
  /** @brief number of keyframes in the queue.
   * @return number of keyframes in the queue
   */
  int KeyframesInQueue() {
    unique_lock<std::mutex> lock(mMutexNewKFs);
    return mlNewKeyFrames.size();
  }
  /** @brief is the local mapping intializing?
   * @return true if the local mapping is initializing
   */
  bool IsInitializing() const;

  /** @brief Get the current key frame time
   * @return the current key frame time
   */
  double GetCurrKFTime();
  /** @brief Get the current Keyframe
   * @return pointer to the current keyframe
   */
  KeyFrame *GetCurrKF();

  Eigen::Matrix3d mRwg;
  Eigen::Vector3d mbg;
  Eigen::Vector3d mba;
  double mScale;
  double mInitTime{};
  double mCostTime{};

  unsigned int mInitSect;
  unsigned int mIdxInit;
  unsigned int mnKFs{};
  double mFirstTs{};
  int mnMatchesInliers;

  // For debugging (erase in normal mode)
  int mInitFr{};
  int mIdxIteration;
  string strSequence;

  bool mbNotBA1;
  bool mbNotBA2;

  bool mbWriteStats{};

  // not consider far points (clouds)
  bool mbFarPoints{};
  float mThFarPoints{};

  // List for Public
  std::list<KeyFrame *> mlProcessdKFs;

#ifdef REGISTER_TIMES
  vector<double> vdKFInsert_ms;
  vector<double> vdMPCulling_ms;
  vector<double> vdMPCreation_ms;
  vector<double> vdLBA_ms;
  vector<double> vdKFCulling_ms;
  vector<double> vdLMTotal_ms;

  vector<double> vdLBASync_ms;
  vector<double> vdKFCullingSync_ms;
  vector<int> vnLBA_edges;
  vector<int> vnLBA_KFopt;
  vector<int> vnLBA_KFfixed;
  vector<int> vnLBA_MPs;
  int nLBA_exec;
  int nLBA_abort;
#endif
protected:
  bool CheckNewKeyFrames();
  void ProcessNewKeyFrame();
  void CreateNewMapPoints();

  void MapPointCulling();
  void SearchInNeighbors();
  void KeyFrameCulling();

  System *mpSystem;

  bool mbMonocular;
  const int mKernel;

  void ResetIfRequested();
  bool mbResetRequested;
  bool mbResetRequestedActiveMap;
  Map *mpMapToReset{};
  std::mutex mMutexReset;

  bool CheckFinish();
  void SetFinish();
  bool mbFinishRequested;
  bool mbFinished;
  std::mutex mMutexFinish;

  Atlas *mpAtlas;

  LoopClosing *mpLoopCloser{};
  Tracking *mpTracker{};

  //  DynamicDetector* mpDynamicsDetectors;

  std::list<KeyFrame *> mlNewKeyFrames;

  KeyFrame *mpCurrentKeyFrame{};

  std::list<MapPoint *> mlpRecentAddedMapPoints;

  std::mutex mMutexNewKFs;

  bool mbAbortBA;

  bool mbStopped;
  bool mbStopRequested;
  bool mbNotStop;
  std::mutex mMutexStop;

  bool mbAcceptKeyFrames;
  std::mutex mMutexAccept;

  void ScaleRefinement();
  void FuseSemanticLabels(KeyFrame *pKF, bool *pbStopFlag);
  bool bInitializing;

  int mNumLM;
  int mNumKFCulling;

  float mTinit;

  int countRefinement{};

  // DEBUG
  ofstream f_lm;
};

} // namespace RVWO

#endif // MAP_LOCALMAPPING_H
