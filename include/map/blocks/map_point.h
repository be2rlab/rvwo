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

#ifndef BLOCKS_MAPPOINT_H
#define BLOCKS_MAPPOINT_H

#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/serialization.hpp>
#include <mutex>
#include <opencv2/core/core.hpp>

#include "map/blocks/frame.h"
#include "map/blocks/key_frame.h"
#include "map/blocks/map.h"
#include "utils/converter.h"
#include "utils/serialization_utils.h"

namespace RVWO {

class KeyFrame;
class Map;
class Frame;
/** @brief Map Point Class */
class MapPoint {
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar & mnId;
    ar & mnFirstKFid;
    ar & mnFirstFrame;
    ar & nObs;

    // Protected variables
    ar &boost::serialization::make_array(mWorldPos.data(), mWorldPos.size());
    ar &boost::serialization::make_array(mNormalVector.data(),
                                         mNormalVector.size());
    // ar & BOOST_SERIALIZATION_NVP(mBackupObservationsId);
    // ar & mObservations;
    ar & mBackupObservationsId1;
    ar & mBackupObservationsId2;
    serializeMatrix(ar, mDescriptor, version);
    ar & mBackupRefKFId;
    // ar & mnVisible;
    // ar & mnFound;

    ar & mbBad;
    ar & mBackupReplacedId;

    ar & mfMinDistance;
    ar & mfMaxDistance;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // ** constructors ** //
  MapPoint();
  /** @brief constructor
   * @param Pos: position of the map point
   * @param pRefKF: pointer to the reference keyframe
   * @param pMap: pointer to the map
   */
  MapPoint(const Eigen::Vector3f &Pos, KeyFrame *pRefKF, Map *pMap);

  /** @brief constructor
   * @param invDepth: inverse depth
   * @param uv_init: initial uv coordinates
   * @param pRefKF: pointer to the reference keyframe
   * @param pHostKF: pointer to the host keyframe
   * @param pMap: pointer to the map
   */
  MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame *pRefKF,
           KeyFrame *pHostKF, Map *pMap);
  /** @brief constructor
   * @param Pos: position of the map point
   * @param pRefKF: pointer to the reference keyframe
   * @param pMap: pointer to the map
   * @param idxF: index of the frame
   */
  MapPoint(const Eigen::Vector3f &Pos, Map *pMap, Frame *pFrame,
           const int &idxF);

  /** @brief Set World position
   * @param Pos: position of the map point
   */
  void SetWorldPos(const Eigen::Vector3f &Pos);
  /** @brief Set World position (deprecated)
   * @param Pos: position of the map point
   */
  void SetWorldPosPrev(const Eigen::Vector3f &Pos);

  /** @brief Get world position
   * @return world position
   */

  Eigen::Vector3f GetWorldPos();

  /** @brief Get world Position (deprcated)
   * @return world position
   */
  Eigen::Vector3f GetWorldPosPrev();

  /** @brief Get Normal of the Point
   * @return normal vector
   */
  Eigen::Vector3f GetNormal();
  /** @brief Get Normal of the Point (deprecated)
   * @return normal vector
   */
  Eigen::Vector3f GetNormalPrev();
  /** @brief Set Normal of the Point
   * @param normal: normal vector
   */
  void SetNormalVector(const Eigen::Vector3f &normal);
  /** @brief Set Normal of the Point (deprecated)
   * @param normal: normal vector
   */
  void SetNormalVectorPrev(const Eigen::Vector3f &normal);
  /** @brief Get Reference Keyframe
   * @return pointer to the reference keyframe
   */
  KeyFrame *GetReferenceKeyFrame();

  /** @brief Get Observations
   * @return map of observations
   */
  std::map<KeyFrame *, std::tuple<int, int>> GetObservations();
  /** @brief Get number of observations that see this point
   * @return number of observations
   */
  int Observations();
  /** @brief Add Observation
   * @param pKF: pointer to the keyframe
   * @param idx: index
   */
  void AddObservation(KeyFrame *pKF, int idx);
  /** @brief Erase Observation
   * @param pKF: pointer to the keyframe
   */
  void EraseObservation(KeyFrame *pKF);
  /** @brief Get Index in Keyframe
   * @param pKF: pointer to the keyframe
   * @return index in the keyframe
   */
  std::tuple<int, int> GetIndexInKeyFrame(KeyFrame *pKF);

  /** @brief Is in Keyframe
   * @param pKF: pointer to the keyframe
   * @return true if the point is in the keyframe
   */
  bool IsInKeyFrame(KeyFrame *pKF);

  /** @brief Set Bad Flag for this point */
  void SetBadFlag();
  /** @brief Is Bad
   * @return true if the point is bad
   */
  bool isBad();
  /** @brief Replace Map Point
   * @param pMP: pointer to the map point
   */
  void Replace(MapPoint *pMP);
  /** @brief Get Replaced Map Point
   * @return pointer to the map point
   */
  MapPoint *GetReplaced();
  /** @brief Increase Visible
   * @param n: number of visible
   */
  void IncreaseVisible(int n = 1);

  /** @brief Increase Found
   * @param n: number of found
   */
  void IncreaseFound(int n = 1);
  /** @brief Get Found Ratio
   * @return found ratio
   */
  float GetFoundRatio();

  /** @brief get found
   * @return number of found
   */
  inline int GetFound() { return mnFound; }
  /** @brief compute distinctive descriptors
   */
  void ComputeDistinctiveDescriptors();
  /** @brief Get Descriptor
   * @return descriptor
   */
  cv::Mat GetDescriptor();
  /** @brief Get Min Distance Invariance
   * @return min distance invariance
   */
  void UpdateNormalAndDepth();
  /** @brief Get Min Distance Invariance
   * @return min distance invariance
   */
  float GetMinDistanceInvariance();
  /** @brief Get Max Distance Invariance
   * @return max distance invariance
   */
  float GetMaxDistanceInvariance();
  /** @brief Predict Scale
   * @param currentDist: current distance
   * @param pKF: pointer to the keyframe
   * @return scale
   */
  int PredictScale(const float &currentDist, KeyFrame *pKF);
  /** @brief Predict Scale
   * @param currentDist: current distance
   * @param pF: pointer to the frame
   * @return scale
   */
  int PredictScale(const float &currentDist, Frame *pF);
  /** @brief Get Map
   * @return pointer to the map
   */
  Map *GetMap();

  /** @brief Update the map
   * @param pMap: pointer to the map
   */
  void UpdateMap(Map *pMap);
  /** @brief Print this point's observations*/
  void PrintObservations();
  /** @brief presave
   * @param spKF: set of keyframes
   * @param spMP: set of mappoints
   */
  void PreSave(set<KeyFrame *> &spKF, set<MapPoint *> &spMP);
  /** @brief Post Load
   * @param mpKFid: map of keyframes
   * @param mpMPid: map of mappoints
   */
  void PostLoad(map<long unsigned int, KeyFrame *> &mpKFid,
                map<long unsigned int, MapPoint *> &mpMPid);

public:
  long unsigned int mnId;
  static long unsigned int nNextId;
  long int mnFirstKFid;
  long int mnFirstFrame;
  int nObs;

  // Probability of movement
  double mProbMov;

  // Inverse Reliability
  double mRelInv;

  // Reprojection error for ARK
  double mReprErr;

  // Seen before
  bool mbFirstTime;

  // Variables used by the tracking
  float mTrackProjX;
  float mTrackProjY;
  float mTrackDepth;
  float mTrackDepthR;
  float mTrackProjXR;
  float mTrackProjYR;
  bool mbTrackInView, mbTrackInViewR;
  int mnTrackScaleLevel, mnTrackScaleLevelR;
  float mTrackViewCos, mTrackViewCosR;
  long unsigned int mnTrackReferenceForFrame;
  long unsigned int mnLastFrameSeen;

  // Variables used by local mapping
  long unsigned int mnBALocalForKF;
  long unsigned int mnFuseCandidateForKF;

  // Variables used by loop closing
  long unsigned int mnLoopPointForKF;
  long unsigned int mnCorrectedByKF;
  long unsigned int mnCorrectedReference;
  Eigen::Vector3f mPosGBA;
  long unsigned int mnBAGlobalForKF;
  long unsigned int mnBALocalForMerge;

  // Variable used by merging
  Eigen::Vector3f mPosMerge;
  Eigen::Vector3f mNormalVectorMerge;

  // Fopr inverse depth optimization
  double mInvDepth;
  double mInitU;
  double mInitV;
  KeyFrame *mpHostKF;

  static std::mutex mGlobalMutex;

  unsigned int mnOriginMapId;

protected:
  // Position in absolute coordinates
  Eigen::Vector3f mWorldPos;

  // Position in absolute coordinates (previous)
  Eigen::Vector3f mWorldPosPrev;

  // Keyframes observing the point and associated index in keyframe
  std::map<KeyFrame *, std::tuple<int, int>> mObservations;
  // For save relation without pointer, this is necessary for save/load function
  std::map<long unsigned int, int> mBackupObservationsId1;
  std::map<long unsigned int, int> mBackupObservationsId2;

  // Mean viewing direction
  Eigen::Vector3f mNormalVector, mNormalVectorPrev;

  // Best descriptor to fast matching
  cv::Mat mDescriptor;

  // Reference KeyFrame
  KeyFrame *mpRefKF;
  long unsigned int mBackupRefKFId;

  // Tracking counters
  int mnVisible;
  int mnFound;

  // Bad flag (we do not currently erase MapPoint from memory)
  bool mbBad;
  MapPoint *mpReplaced;
  // For save relation without pointer, this is necessary for save/load function
  long long int mBackupReplacedId;

  // Scale invariance distances
  float mfMinDistance;
  float mfMaxDistance;

  Map *mpMap;

  // Mutex
  std::mutex mMutexPos;
  std::mutex mMutexFeatures;
  std::mutex mMutexMap;
};

} // namespace RVWO

#endif // MAP_MAPPOINT_H
