/**
 * This file is modified part of ORB-SLAM3
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

#ifndef BLOCKS_KEYFRAME_H
#define BLOCKS_KEYFRAME_H

#include "map/blocks/frame.h"
#include "map/blocks/key_frame_database.h"
#include "map/blocks/map_point.h"
#include "third_party/DBoW2/DBoW2/BowVector.h"
#include "third_party/DBoW2/DBoW2/FeatureVector.h"
#include "utils/orb_extractor.h"
#include "utils/orb_vocabulary.h"

#include "geometry/camera_models/geometric_camera.h"

#include <memory>
#include <mutex>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

namespace RVWO {

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class GeometricCamera;

class KeyFrame {
  /** @brief serialize Matrix
   * @param ar Archive
   * @param mat Matrix to serialize
   * @param version Version of the serialization
   */
  template <class Archive>
  void serializeMatrix(Archive &ar, cv::Mat &mat, const unsigned int version) {
    int cols, rows, type;
    bool continuous;

    if (Archive::is_saving::value) {
      cols = mat.cols;
      rows = mat.rows;
      type = mat.type();
      continuous = mat.isContinuous();
    }

    ar & cols & rows & type & continuous;

    if (Archive::is_loading::value)
      mat.create(rows, cols, type);

    if (continuous) {
      const unsigned int data_size = rows * cols * mat.elemSize();
      ar &boost::serialization::make_array(mat.ptr(), data_size);
    } else {
      const unsigned int row_size = cols * mat.elemSize();
      for (int i = 0; i < rows; i++) {
        ar &boost::serialization::make_array(mat.ptr(i), row_size);
      }
    }
  }

  template <class Archive>
  void serializeMatrix(Archive &ar, const cv::Mat &mat,
                       const unsigned int version) {
    cv::Mat matAux = mat;

    serializeMatrix(ar, matAux, version);

    if (Archive::is_loading::value) {
      cv::Mat *ptr;
      ptr = (cv::Mat *)(&mat);
      *ptr = matAux;
    }
  }

  friend class boost::serialization::access;
  template <class Archive>
  void serializeVectorKeyPoints(Archive &ar, const vector<cv::KeyPoint> &vKP,
                                const unsigned int version) {
    int NumEl;

    if (Archive::is_saving::value) {
      NumEl = vKP.size();
    }

    ar & NumEl;

    vector<cv::KeyPoint> vKPaux = vKP;
    if (Archive::is_loading::value)
      vKPaux.reserve(NumEl);

    for (int i = 0; i < NumEl; ++i) {
      cv::KeyPoint KPi;

      if (Archive::is_loading::value)
        KPi = cv::KeyPoint();

      if (Archive::is_saving::value)
        KPi = vKPaux[i];

      ar & KPi.angle;
      ar & KPi.response;
      ar & KPi.size;
      ar & KPi.pt.x;
      ar & KPi.pt.y;
      ar & KPi.class_id;
      ar & KPi.octave;

      if (Archive::is_loading::value)
        vKPaux.push_back(KPi);
    }

    if (Archive::is_loading::value) {
      vector<cv::KeyPoint> *ptr;
      ptr = (vector<cv::KeyPoint> *)(&vKP);
      *ptr = vKPaux;
    }
  }

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar & mnId;
    ar &const_cast<long unsigned int &>(mnFrameId);
    ar &const_cast<double &>(mTimeStamp);
    // Grid
    ar &const_cast<int &>(mnGridCols);
    ar &const_cast<int &>(mnGridRows);
    ar &const_cast<float &>(mfGridElementWidthInv);
    ar &const_cast<float &>(mfGridElementHeightInv);
    // Variables of tracking
    ar & mnTrackReferenceForFrame;
    ar & mnFuseTargetForKF;
    // Variables of local mapping
    ar & mnBALocalForKF;
    ar & mnBAFixedForKF;
    ar & mnNumberOfOpt;
    // Variables used by KeyFrameDatabase
    ar & mnLoopQuery;
    ar & mnLoopWords;
    ar & mLoopScore;
    ar & mnRelocQuery;
    ar & mnRelocWords;
    ar & mRelocScore;
    ar & mnMergeQuery;
    ar & mnMergeWords;
    ar & mMergeScore;
    ar & mnPlaceRecognitionQuery;
    ar & mnPlaceRecognitionWords;
    ar & mPlaceRecognitionScore;
    ar & mbCurrentPlaceRecognition;
    // Variables of loop closing
    serializeMatrix(ar, mTcwGBA, version);
    serializeMatrix(ar, mTcwBefGBA, version);
    serializeMatrix(ar, mVwbGBA, version);
    serializeMatrix(ar, mVwbBefGBA, version);
    ar & mBiasGBA;
    ar & mnBAGlobalForKF;
    // Variables of Merging
    serializeMatrix(ar, mTcwMerge, version);
    serializeMatrix(ar, mTcwBefMerge, version);
    serializeMatrix(ar, mTwcBefMerge, version);
    serializeMatrix(ar, mVwbMerge, version);
    serializeMatrix(ar, mVwbBefMerge, version);
    ar & mBiasMerge;
    ar & mnMergeCorrectedForKF;
    ar & mnMergeForKF;
    ar & mfScaleMerge;
    ar & mnBALocalForMerge;
    // Scale
    ar & mfScale;
    // Calibration parameters
    ar &const_cast<float &>(fx);
    ar &const_cast<float &>(fy);
    ar &const_cast<float &>(invfx);
    ar &const_cast<float &>(invfy);
    ar &const_cast<float &>(cx);
    ar &const_cast<float &>(cy);
    ar &const_cast<float &>(mbf);
    ar &const_cast<float &>(mb);
    ar &const_cast<float &>(mThDepth);
    serializeMatrix(ar, mDistCoef, version);
    // Number of Keypoints
    ar &const_cast<int &>(N);
    // KeyPoints
    serializeVectorKeyPoints(ar, mvKeys, version);
    serializeVectorKeyPoints(ar, mvKeysUn, version);
    ar &const_cast<vector<float> &>(mvuRight);
    ar &const_cast<vector<float> &>(mvDepth);
    serializeMatrix(ar, mDescriptors, version);
    // BOW
    ar & mBowVec;
    ar & mFeatVec;
    // Pose relative to parent
    serializeMatrix(ar, mTcp, version);
    // Scale
    ar &const_cast<int &>(mnScaleLevels);
    ar &const_cast<float &>(mfScaleFactor);
    ar &const_cast<float &>(mfLogScaleFactor);
    ar &const_cast<vector<float> &>(mvScaleFactors);
    ar &const_cast<vector<float> &>(mvLevelSigma2);
    ar &const_cast<vector<float> &>(mvInvLevelSigma2);
    // Image bounds and calibration
    ar &const_cast<int &>(mnMinX);
    ar &const_cast<int &>(mnMinY);
    ar &const_cast<int &>(mnMaxX);
    ar &const_cast<int &>(mnMaxY);
    serializeMatrix(ar, mK, version);
    // Pose
    serializeMatrix(ar, Tcw, version);
    // MapPointsId associated to keypoints
    ar & mvBackupMapPointsId;
    // Grid
    ar & mGrid;
    // Connected KeyFrameWeight
    ar & mBackupConnectedKeyFrameIdWeights;
    // Spanning Tree and Loop Edges
    ar & mbFirstConnection;
    ar & mBackupParentId;
    ar & mvBackupChildrensId;
    ar & mvBackupLoopEdgesId;
    ar & mvBackupMergeEdgesId;
    // Bad flags
    ar & mbNotErase;
    ar & mbToBeErased;
    ar & mbBad;

    ar & mHalfBaseline;

    // Camera variables
    ar & mnBackupIdCamera;
    ar & mnBackupIdCamera2;
  }

public:
  // ** Constructor ** //
  /** @brief Constructor */
  KeyFrame();
  /** @brief Constructor
   * @param F Frame
   * @param pMap Map
   * @param pKFDB KeyFrameDatabase
   */
  KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB);

  // ** Pose functions ** //
  /** @brief Set Pose
   * @param Tcw Transformation matrix
   */
  void SetPose(const cv::Mat &Tcw);
  /** @brief Set Velocity
   * @param Vw_ Velocity
   */
  void SetVelocity(const cv::Mat &Vw_);

  /** @brief Get Pose
   * @return Transformation matrix cv::Mat
   * */
  cv::Mat GetPose();
  /** @brief Get Pose Inverse
   * @return Transformation matrix cv::Mat
   * */
  cv::Mat GetPoseInverse();
  /** @brief Get Camera Center
   * @return Camera center cv::Mat
   */
  cv::Mat GetCameraCenter();
  /** @brief Get Stereo Center
   * @return Stereo center cv::Mat
   */
  cv::Mat GetStereoCenter();
  /** @brief Get Rotation
   * @return Rotation matrix cv::Mat
   */
  cv::Mat GetRotation();

  /** @brief Get Translation
   * @return Translation matrix cv::Mat
   */
  cv::Mat GetTranslation();

  /** @brief Get Velocity
   * @return Velocity matrix cv::Mat
   */
  cv::Mat GetVelocity();

  /** @brief Get Odom Pose
   * @return Odom Pose matrix cv::Mat
   */
  cv::Mat GetOdomPose();

  /** @brief Bag of Words Representation */
  void ComputeBoW();

  // ** Covisibility graph functions ** //
  /** @brief Add Connection
   * @param pKF KeyFrame
   * @param weight Weight
   */
  void AddConnection(KeyFrame *pKF, const int &weight);
  /** @brief Erase Connection
   * @param pKF KeyFrame
   */
  void EraseConnection(KeyFrame *pKF);
  /** @brief Update Connections
   * @param upParent Update Parent
   * */
  void UpdateConnections(bool upParent = true);
  /** @brief Update Best Covisibles */
  void UpdateBestCovisibles();
  /** @brief Get Connected KeyFrames
   * @return Set of KeyFrames
   */
  std::set<KeyFrame *> GetConnectedKeyFrames();
  /** @brief Get Vector Covisible KeyFrames
   * @return Vector of KeyFrames
   */
  std::vector<KeyFrame *> GetVectorCovisibleKeyFrames();
  /** @brief Get Best Covisible KeyFrames
   * @param N Number of KeyFrames
   * @return Vector of KeyFrames
   */
  std::vector<KeyFrame *> GetBestCovisibilityKeyFrames(const int &N);
  /** @brief Get Covisible KeyFrames
   * @param w Weight
   * @return Vector of KeyFrames
   */
  std::vector<KeyFrame *> GetCovisiblesByWeight(const int &w);
  /** @brief Get Weight
   * @param pKF KeyFrame
   * @return Weight
   */
  int GetWeight(KeyFrame *pKF);

  // ** Spanning tree functions ** //
  /** @brief Add Child
   * @param pKF KeyFrame
   */
  void AddChild(KeyFrame *pKF);
  /** @brief Erase Child
   * @param pKF KeyFrame
   */
  void EraseChild(KeyFrame *pKF);
  /** @brief Change Parent
   * @param pKF KeyFrame
   */
  void ChangeParent(KeyFrame *pKF);
  /** @brief Get Childs
   * @return Set of KeyFrames
   */
  std::set<KeyFrame *> GetChilds();
  /** @brief Get Parent
   * @return KeyFrame
   */
  KeyFrame *GetParent();
  /** @brief has Child
   * @param pKF KeyFrame
   * @return True if has child
   */
  bool hasChild(KeyFrame *pKF);
  /** @brief Set First Connection
   * @param bFirst First Connection
   */
  void SetFirstConnection(bool bFirst);

  // ** Loop Edges ** //
  /** @brief Add Loop Edge
   * @param pKF KeyFrame
   */
  void AddLoopEdge(KeyFrame *pKF);
  /** @brief Get Loop Edges
   * @return Set of KeyFrames
   */
  std::set<KeyFrame *> GetLoopEdges();

  // ** Merge Edges ** //
  /** @brief Add Merge Edge
   * @param pKF KeyFrame
   */
  void AddMergeEdge(KeyFrame *pKF);
  /** @brief Get Merge Edges
   * @return Set of KeyFrames
   */
  set<KeyFrame *> GetMergeEdges();

  // ** MapPoint observation functions ** //
  /** @brief Get Number of MapPoints
   * @return Number of MapPoints
   */
  int GetNumberMPs();

  /** @brief Add Map Point
   * @param pMP MapPoint
   * @param idx Index
   */
  void AddMapPoint(MapPoint *pMP, const size_t &idx);
  /** @brief Erase MapPoint Match
   * @param idx Index
   */
  void EraseMapPointMatch(const int &idx);
  /** @brief Erase MapPoint Match
   * @param pMP MapPoint
   */
  void EraseMapPointMatch(MapPoint *pMP);
  /** @brief Replace MapPoint Match
   * @param idx Index
   * @param pMP MapPoint
   */
  void ReplaceMapPointMatch(const int &idx, MapPoint *pMP);
  /** @brief Get MapPoints
   * @return Set of MapPoints
   */
  std::set<MapPoint *> GetMapPoints();
  /** @brief Get MapPoint Matches
   * @return Vector of MapPoints
   */
  std::vector<MapPoint *> GetMapPointMatches();
  /** @brief Tracked MapPoints
   * @param minObs Minimum Observations
   * @return Number of MapPoints
   */
  int TrackedMapPoints(const int &minObs);
  /** @brief Get MapPoint
   * @param idx Index
   * @return MapPoint
   */
  MapPoint *GetMapPoint(const size_t &idx);

  // ** KeyPoint functions ** //
  /** @brief Get Features in Area
   * @param x X coordinate
   * @param y Y coordinate
   * @param r Radius
   * @param bRight Right
   * @return Vector of size_t
   */
  std::vector<size_t> GetFeaturesInArea(const float &x, const float &y,
                                        const float &r,
                                        const bool bRight = false) const;
  /** @brief Unproject Stereo
   * @param i Index
   * @return cv::Mat
   */
  cv::Mat UnprojectStereo(int i);

  // ** Image ** //
  /** @brief Is in Image
   * @param x X coordinate
   * @param y Y coordinate
   * @return True if is in image
   */
  bool IsInImage(const float &x, const float &y) const;

  // ** Enable/Disable bad flag changes ** //
  /** @brief Set Not Erase */
  void SetNotErase();
  /** @brief Set Erase */
  void SetErase();

  // ** Set/check bad flag ** //
  /** @brief Set Bad Flag */
  void SetBadFlag();
  /** @brief is Bad */
  bool isBad();

  // ** Compute Scene Depth (q=2 median). Used in monocular. ** //
  /** @brief Compute Scene Median Depth
   * @param q Q
   * @return Float
   */
  float ComputeSceneMedianDepth(const int q);

  /** @brief weightComp
   * @param a int
   * @param b int
   * @return True if a > b
   */
  static bool weightComp(int a, int b) { return a > b; }

  /** @brief lId
   * @param pKF1 KeyFrame
   * @param pKF2 KeyFrame
   * @return True if pKF1->mnId < pKF2->mnId
   */
  static bool lId(KeyFrame *pKF1, KeyFrame *pKF2) {
    return pKF1->mnId < pKF2->mnId;
  }

  /** @brief Get Map
   * @return Map
   */
  Map *GetMap();
  /** @brief Update Map
   * @param pMap Map
   */
  void UpdateMap(Map *pMap);
  /** @brief Project Point Distort
   * @param pMP MapPoint
   * @param kp cv::Point2f
   * @param u float
   * @param v float
   * @return True if success
   * */
  bool ProjectPointDistort(MapPoint *pMP, cv::Point2f &kp, float &u, float &v);
  /** @brief Project Point UnDistort
   * @param pMP MapPoint
   * @param kp cv::Point2f
   * @param u float
   * @param v float
   * @return True if success
   */
  bool ProjectPointUnDistort(MapPoint *pMP, cv::Point2f &kp, float &u,
                             float &v);
  /** @brief Pre Save
   * @param spKF set of KeyFrames
   * @param spMP set of MapPoints
   * @param spCam set of GeometricCameras
   * */
  void PreSave(set<KeyFrame *> &spKF, set<MapPoint *> &spMP,
               set<GeometricCamera *> &spCam);
  /** @brief Post Load
   * @param mpKFid map of KeyFrames
   * @param mpMPid map of MapPoints
   * @param mpCamId map of GeometricCameras
   */
  void PostLoad(map<long unsigned int, KeyFrame *> &mpKFid,
                map<long unsigned int, MapPoint *> &mpMPid,
                map<unsigned int, GeometricCamera *> &mpCamId);

  /** @brief Set ORB Vocabulary
   * @param pORBVoc ORBVocabulary
   */
  void SetORBVocabulary(ORBVocabulary *pORBVoc);
  /** @brief Set KeyFrame Database
   * @param pKFDB KeyFrameDatabase
   */
  void SetKeyFrameDatabase(KeyFrameDatabase *pKFDB);

  // The following variables are accesed from only 1 thread or never change (no
  // mutex needed).
public:
  std::vector<g2o::SE2> odom_LastKF;
  std::pair<KeyFrame *, PreSE2> odomToThis;
  std::pair<KeyFrame *, PreSE2> odomFromThis;
  g2o::SE2 odom;
  cv::Mat Tbc;
  cv::Mat Tcb;
  static long unsigned int nNextId;
  long unsigned int mnId;
  const long unsigned int mnFrameId;

  const double mTimeStamp;

  // Grid (to speed up feature matching)
  const int mnGridCols;
  const int mnGridRows;
  const float mfGridElementWidthInv;
  const float mfGridElementHeightInv;

  // Variables used by the tracking
  long unsigned int mnTrackReferenceForFrame;
  long unsigned int mnFuseTargetForKF;

  // Variables used by the local mapping
  long unsigned int mnBALocalForKF;
  long unsigned int mnBAFixedForKF;

  // Number of optimizations by BA(amount of iterations in BA)
  long unsigned int mnNumberOfOpt;

  // Variables used by the keyframe database
  long unsigned int mnLoopQuery;
  int mnLoopWords;
  float mLoopScore;
  long unsigned int mnRelocQuery;
  int mnRelocWords;
  float mRelocScore;
  long unsigned int mnMergeQuery;
  int mnMergeWords;
  float mMergeScore;
  long unsigned int mnPlaceRecognitionQuery;
  int mnPlaceRecognitionWords;
  float mPlaceRecognitionScore;

  bool mbCurrentPlaceRecognition;

  // Variables used by loop closing
  cv::Mat mTcwGBA;
  cv::Mat mTcwBefGBA;
  cv::Mat mVwbGBA;
  cv::Mat mVwbBefGBA;
  long unsigned int mnBAGlobalForKF;

  // Variables used by merging
  cv::Mat mTcwMerge;
  cv::Mat mTcwBefMerge;
  cv::Mat mTwcBefMerge;
  cv::Mat mVwbMerge;
  cv::Mat mVwbBefMerge;
  long unsigned int mnMergeCorrectedForKF;
  long unsigned int mnMergeForKF;
  float mfScaleMerge;
  long unsigned int mnBALocalForMerge;

  float mfScale;

  // Calibration parameters
  const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
  cv::Mat mDistCoef;

  // Number of KeyPoints
  const int N;

  // KeyPoints, stereo coordinate and descriptors (all associated by an index)
  const std::vector<cv::KeyPoint> mvKeys;
  const std::vector<cv::KeyPoint> mvKeysUn;
  const std::vector<float> mvuRight; // negative value for monocular points
  const std::vector<float> mvDepth;  // negative value for monocular points
  const cv::Mat mDescriptors;

  // BoW
  DBoW2::BowVector mBowVec;
  DBoW2::FeatureVector mFeatVec;

  // Pose relative to parent (this is computed when bad flag is activated)
  cv::Mat mTcp;

  // Scale
  const int mnScaleLevels;
  const float mfScaleFactor;
  const float mfLogScaleFactor;
  const std::vector<float> mvScaleFactors;
  const std::vector<float> mvLevelSigma2;
  const std::vector<float> mvInvLevelSigma2;

  // Image bounds and calibration
  const int mnMinX;
  const int mnMinY;
  const int mnMaxX;
  const int mnMaxY;
  const cv::Mat mK;

  KeyFrame *mpLastKF;
  void SetPoseByOdomTo(KeyFrame *refKF); // OUR

  KeyFrame *mPrevKF;
  KeyFrame *mNextKF;

  // Preintegrated WOdometry measurements from previous keyframe
  ODOM::Preintegrated *mpOdomPreintegrated;

  unsigned int mnOriginMapId;

  string mNameFile;

  int mnDataset;

  std::vector<KeyFrame *> mvpLoopCandKFs;
  std::vector<KeyFrame *> mvpMergeCandKFs;

  bool mbHasHessian;
  cv::Mat mHessianPose;

  // The following variables need to be accessed trough a mutex to be thread
  // safe.
protected:
  // SE3 Pose and camera center
  cv::Mat Tcw;
  cv::Mat Twc;
  cv::Mat Ow;
  cv::Mat Cw; // Stereo middel point. Only for visualization

  // Body position
  cv::Mat Owb;

  // Velocity (Only used for wheel odometry SLAM)
  cv::Mat Vw;

  // MapPoints associated to keypoints
  std::vector<MapPoint *> mvpMapPoints;
  // For save relation without pointer, this is necessary for save/load function
  std::vector<long long int> mvBackupMapPointsId;

  // BoW
  KeyFrameDatabase *mpKeyFrameDB;
  ORBVocabulary *mpORBvocabulary;

  // Grid over the image to speed up feature matching
  std::vector<std::vector<std::vector<size_t>>> mGrid;

  std::map<KeyFrame *, int> mConnectedKeyFrameWeights;
  std::vector<KeyFrame *> mvpOrderedConnectedKeyFrames;
  std::vector<int> mvOrderedWeights;
  // For save relation without pointer, this is necessary for save/load function
  std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;

  // Spanning Tree and Loop Edges
  bool mbFirstConnection;
  KeyFrame *mpParent;
  std::set<KeyFrame *> mspChildrens;
  std::set<KeyFrame *> mspLoopEdges;
  std::set<KeyFrame *> mspMergeEdges;
  // For save relation without pointer, this is necessary for save/load function
  long long int mBackupParentId;
  std::vector<long unsigned int> mvBackupChildrensId;
  std::vector<long unsigned int> mvBackupLoopEdgesId;
  std::vector<long unsigned int> mvBackupMergeEdgesId;

  // Bad flags
  bool mbNotErase;
  bool mbToBeErased;
  bool mbBad;

  float mHalfBaseline; // Only for visualization

  Map *mpMap;

  std::mutex mMutexPose; // for pose, velocity and biases
  std::mutex mMutexConnections;
  std::mutex mMutexFeatures;
  std::mutex mMutexMap;

  // Backup variables for inertial
  long long int mBackupPrevKFId;
  long long int mBackupNextKFId;

  // Backup for Cameras
  unsigned int mnBackupIdCamera, mnBackupIdCamera2;

public:
  GeometricCamera *mpCamera, *mpCamera2;

  // Indexes of stereo observations correspondences
  std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

  // Transformation matrix between cameras in stereo fisheye
  cv::Mat mTlr;
  cv::Mat mTrl;

  // KeyPoints in the right image (for stereo fisheye, coordinates are needed)
  const std::vector<cv::KeyPoint> mvKeysRight;

  const int NLeft, NRight;

  std::vector<std::vector<std::vector<size_t>>> mGridRight;

  cv::Mat GetRightPose();
  cv::Mat GetRightPoseInverse();
  cv::Mat GetRightPoseInverseH();
  cv::Mat GetRightCameraCenter();
  cv::Mat GetRightRotation();
  cv::Mat GetRightTranslation();

  cv::Mat imgLeft, imgRight; // TODO Backup??

  void PrintPointDistribution() {
    int left = 0, right = 0;
    int Nlim = (NLeft != -1) ? NLeft : N;
    for (int i = 0; i < N; i++) {
      if (mvpMapPoints[i]) {
        if (i < Nlim)
          left++;
        else
          right++;
      }
    }
    cout << "Point distribution in KeyFrame: left-> " << left << " --- right-> "
         << right << endl;
  }
};

} // namespace RVWO

#endif // KEYFRAME_H
