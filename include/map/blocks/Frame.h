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

#ifndef BLOCKS_FRAME_H
#define BLOCKS_FRAME_H

#include <vector>

#include "ORBVocabulary.h"
#include "OdomTypes.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "Thirdparty/g2o/g2o/types/se2.h"
#include "Thirdparty/g2o/g2o/types/se3quat.h"

#include <mutex>
#include <opencv2/opencv.hpp>

namespace RVWO {
/**
 * @brief PreSE2 is a struct that contains the preintegrated measurement of the
 * wheel odometry
 */
struct PreSE2 {
public:
  double meas[3];
  double cov[9]; // 3*3, RowMajor
  PreSE2() {
    memset(meas, 0, sizeof meas);
    memset(cov, 0, sizeof cov);
  }
  ~PreSE2() {}
};

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;
class ConstraintPoseImu;
class GeometricCamera;
class ORBextractor;
class System;

class Frame {
public:
  Frame();

  /** @brief Copy constructor.
   * @param frame Frame to be copied.
   */
  Frame(const Frame &frame);

  /** @brief Constructor for stereo cameras with wheel odometry.
   * @param imLeft Left image.
   * @param imRight Right image.
   * @param odo Wheel odometry measurement.
   * @param timeStamp Timestamp.
   * @param extractorLeft ORB extractor for the left image.
   * @param extractorRight ORB extractor for the right image.
   * @param voc ORB vocabulary.
   * @param K Calibration matrix.
   * @param distCoef OpenCV distortion parameters.
   * @param bf Stereo baseline multiplied by fx.
   * @param thDepth Threshold close/far points.
   * @param pCamera Geometric camera model.
   * @param pPrevF Pointer to the previous frame.
   */
  Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const g2o::SE2 &odo,
        const double &timeStamp, ORBextractor *extractorLeft,
        ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K,
        cv::Mat &distCoef, const float &bf, const float &thDepth,
        GeometricCamera *pCamera, Frame *pPrevF = static_cast<Frame *>(NULL));

  /** @brief Constructor for stereo cameras without wheel odometry.
   * @param imLeft Left image.
   * @param imRight Right image.
   * @param timeStamp Timestamp.
   * @param extractorLeft ORB extractor for the left image.
   * @param extractorRight ORB extractor for the right image.
   * @param voc ORB vocabulary.
   * @param K Calibration matrix.
   * @param distCoef OpenCV distortion parameters.
   * @param bf Stereo baseline multiplied by fx.
   * @param thDepth Threshold close/far points.
   * @param pCamera Geometric camera model.
   * @param pPrevF Pointer to the previous frame.
   */
  Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
        ORBextractor *extractorLeft, ORBextractor *extractorRight,
        ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf,
        const float &thDepth, GeometricCamera *pCamera,
        Frame *pPrevF = static_cast<Frame *>(NULL));

  /** @brief Constructor for Monocular cameras without wheel odometry.
   * @param imGray Image.
   * @param timeStamp Timestamp.
   * @param extractor ORB extractor.
   * @param voc ORB vocabulary.
   * @param pCamera Geometric camera model.
   * @param distCoef OpenCV distortion parameters.
   * @param bf Stereo baseline multiplied by fx.
   * @param thDepth Threshold close/far points.
   * @param pPrevF Pointer to the previous frame.
   */
  Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor,
        ORBVocabulary *voc, GeometricCamera *pCamera, cv::Mat &distCoef,
        const float &bf, const float &thDepth,
        Frame *pPrevF = static_cast<Frame *>(NULL));

  /** @brief Constructor for Monocular cameras with wheel odometry.
   * @param imGray Image.
   * @param odo Wheel odometry measurement.
   * @param timeStamp Timestamp.
   * @param extractor ORB extractor.
   * @param voc ORB vocabulary.
   * @param pCamera Geometric camera model.
   * @param distCoef OpenCV distortion parameters.
   * @param bf Stereo baseline multiplied by fx.
   * @param thDepth Threshold close/far points.
   * @param pPrevF Pointer to the previous frame.
   */
  Frame(const cv::Mat &imGray, const g2o::SE2 &odo, const double timeStamp,
        ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &extParaBc,
        GeometricCamera *pCamera, cv::Mat &distCoef, const float &bf,
        const float &thDepth, Frame *pPrevF = static_cast<Frame *>(NULL));

  // Destructor
  // ~Frame();

  /** @brief Extract ORB on the image. 0 for left image and 1 for right image.
   * @param flag 0 for left image and 1 for right image.
   * @param im Image.
   * @param x0 Minimum x coordinate.
   * @param x1 Maximum x coordinate.
   */
  void ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1);

  /** @brief Compute Bag of Words representation. */
  void ComputeBoW();

  /** @brief Set the camera pose. (Imu pose is not modified!)
   * @param Tcw Camera pose.
   */
  void SetPose(cv::Mat Tcw);

  /** @brief Get the camera pose.
   * @param Tcw Camera pose.
   */
  void GetPose(cv::Mat &Tcw);

  /** @brief Computes rotation, translation and camera center matrices from the
   * camera pose. */
  void UpdatePoseMatrices();

  /** @brief Returns the camera center.
   * @return Camera center.
   */
  inline cv::Mat GetCameraCenter() { return mOw.clone(); }

  /**  Returns inverse of rotation matrix.
   * @return Inverse of rotation matrix.
   */
  inline cv::Mat GetRotationInverse() { return mRwc.clone(); }

  /** Get pose from wheel odometry
   * @return Pose from wheel odometry.
   */
  cv::Mat GetOdomPose();

  /** @brief Check if a MapPoint is in the frustum of the camera
   * and fill variables of the MapPoint to be used by the tracking
   * @param pMP MapPoint to be checked.
   * @param viewingCosLimit Cosine of the angle of the frustum.
   * @return True if the MapPoint is in the frustum, false otherwise.
   */
  bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

  /** @brief Project point undistorting it with the camera distortion model.
   * @param pMP MapPoint to be projected.
   * @param kp Keypoint to be projected.
   * @param u Horizontal coordinate of the projection.
   * @param v Vertical coordinate of the projection.
   */
  bool ProjectPointDistort(MapPoint *pMP, cv::Point2f &kp, float &u, float &v);

  /** @brief inRefCoordinates
   * @param pCw position of the camera
   * @return position of the camera in the reference frame
   */
  cv::Mat inRefCoordinates(cv::Mat pCw);

  /** @brief Compute the cell of a keypoint (return false if outside the grid)
   * @param kp Keypoint to be projected.
   * @param posX Horizontal coordinate of the cell.
   * @param posY Vertical coordinate of the cell.
   * @return True if the keypoint is inside the grid, false otherwise.
   */
  bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

  /** @brief Get the keypoints in a certain area of the image.
   * @param x Horizontal coordinate of the center of the area.
   * @param y Vertical coordinate of the center of the area.
   * @param r Radius of the area.
   * @param minLevel Minimum level of the pyramid.
   * @param maxLevel Maximum level of the pyramid.
   * @param bRight True if the right image is used, false otherwise.
   * @return Vector of the indexes of the keypoints in the area.
   */
  vector<size_t> GetFeaturesInArea(const float &x, const float &y,
                                   const float &r, const int minLevel = -1,
                                   const int maxLevel = -1,
                                   const bool bRight = false) const;

  /** @brief Search a match for each keypoint in the left image to a keypoint in
   * the right image. If there is a match, depth is computed and the right
   * coordinate associated to the left keypoint is stored.
   */
  void ComputeStereoMatches();

  /** Backprojects a keypoint (if stereo) into 3D world coordinates.
   * @param i Index of the keypoint.
   * @return 3D world coordinates.
   */
  cv::Mat UnprojectStereo(const int &i);

  // @todo(@JaafarMahmoud1): We need to delete it
  /** @brief set integrated */
  void setIntegrated();

  cv::Mat mRwc;
  cv::Mat mOw;

public:
  // Wheel Odometry measurement
  g2o::SE2 odom;
  g2o::SE3Quat mOdom;
  // Odometry preintegration from last Keyframe
  ODOM::Preintegrated *mpOdomPreintegrated;

  // int mSensor;
  //  Extrinsic parameter
  cv::Mat Tbc;
  cv::Mat Tcb;

  // Vocabulary used for relocalization.
  ORBVocabulary *mpORBvocabulary;

  // Feature extractor. The right is used only in the stereo case.
  ORBextractor *mpORBextractorLeft, *mpORBextractorRight;

  // Frame timestamp.
  double mTimeStamp;

  // Calibration matrix and OpenCV distortion parameters.
  cv::Mat mK;
  static float fx;
  static float fy;
  static float cx;
  static float cy;
  static float invfx;
  static float invfy;
  cv::Mat mDistCoef;

  // Stereo baseline multiplied by fx.
  float mbf;

  // Stereo baseline in meters.
  float mb;

  // Threshold close/far points. Close points are inserted from 1 view.
  // Far points are inserted as in the monocular case from 2 views.
  float mThDepth;

  // Number of KeyPoints.
  int N;

  // Vector of keypoints (original for visualization) and undistorted (actually
  // used by the system). In the stereo case, mvKeysUn is redundant as images
  // must be rectified. In the RGB-D case, RGB images can be distorted.
  std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
  std::vector<cv::KeyPoint> mvKeysUn;

  // Corresponding stereo coordinate and depth for each keypoint.
  std::vector<MapPoint *> mvpMapPoints;
  // "Monocular" keypoints have a negative value.
  std::vector<float> mvuRight;
  std::vector<float> mvDepth;

  // Bag of Words Vector structures.
  DBoW2::BowVector mBowVec;
  DBoW2::FeatureVector mFeatVec;

  // ORB descriptor, each row associated to a keypoint.
  cv::Mat mDescriptors, mDescriptorsRight;

  // MapPoints associated to keypoints, NULL pointer if no association.
  // Flag to identify outlier associations.
  std::vector<bool> mvbOutlier;
  int mnCloseMPs;

  // Keypoints are assigned to cells in a grid to reduce matching complexity
  // when projecting MapPoints.
  static float mfGridElementWidthInv;
  static float mfGridElementHeightInv;
  std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

  // Camera pose.
  cv::Mat mTcw;

  // IMU linear velocity
  cv::Mat mVw;

  KeyFrame *mpLastKeyFrame;

  // Pointer to previous frame
  Frame *mpPrevFrame;

  // Current and Next Frame id.
  static long unsigned int nNextId;
  long unsigned int mnId;

  // Reference Keyframe.
  KeyFrame *mpReferenceKF;

  // Scale pyramid info.
  int mnScaleLevels;
  float mfScaleFactor;
  float mfLogScaleFactor;
  vector<float> mvScaleFactors;
  vector<float> mvInvScaleFactors;
  vector<float> mvLevelSigma2;
  vector<float> mvInvLevelSigma2;

  // Undistorted Image Bounds (computed once).
  static float mnMinX;
  static float mnMaxX;
  static float mnMinY;
  static float mnMaxY;

  static bool mbInitialComputations;

  map<long unsigned int, cv::Point2f> mmProjectPoints;
  map<long unsigned int, cv::Point2f> mmMatchedInImage;

  string mNameFile;

  int mnDataset;

  double mTimeStereoMatch;
  double mTimeORB_Ext;

private:
  // Undistort keypoints given OpenCV distortion parameters.
  // Only for the RGB-D case. Stereo must be already rectified!
  // (called in the constructor).
  void UndistortKeyPoints();

  // Computes image bounds for the undistorted image (called in the
  // constructor).
  void ComputeImageBounds(const cv::Mat &imLeft);

  // Assign keypoints to the grid for speed up feature matching (called in the
  // constructor).
  void AssignFeaturesToGrid();

  // Rotation, translation and camera center
  cv::Mat mRcw;
  cv::Mat mtcw;
  //==mtwc

  bool mbImuPreintegrated;

  std::mutex *mpMutexImu;

public:
  GeometricCamera *mpCamera, *mpCamera2;

  // Number of KeyPoints extracted in the left and right images
  int Nleft, Nright;
  // Number of Non Lapping Keypoints
  int monoLeft, monoRight;

  // For stereo matching
  std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

  // For stereo fisheye matching
  static cv::BFMatcher BFmatcher;

  // Triangulated stereo observations using as reference the left camera. These
  // are computed during ComputeStereoFishEyeMatches
  std::vector<cv::Mat> mvStereo3Dpoints;

  // Grid for the right image
  std::vector<std::size_t> mGridRight[FRAME_GRID_COLS][FRAME_GRID_ROWS];

  cv::Mat mTlr, mRlr, mtlr, mTrl;

  Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
        ORBextractor *extractorLeft, ORBextractor *extractorRight,
        ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf,
        const float &thDepth, GeometricCamera *pCamera,
        GeometricCamera *pCamera2, cv::Mat &Tlr,
        Frame *pPrevF = static_cast<Frame *>(NULL),
        const IMU::Calib &ImuCalib = IMU::Calib());

  // Stereo fisheye
  void ComputeStereoFishEyeMatches();

  bool isInFrustumChecks(MapPoint *pMP, float viewingCosLimit,
                         bool bRight = false);

  cv::Mat UnprojectStereoFishEye(const int &i);

  cv::Mat imgLeft, imgRight;

  void PrintPointDistribution() {
    int left = 0, right = 0;
    int Nlim = (Nleft != -1) ? Nleft : N;
    for (int i = 0; i < N; i++) {
      if (mvpMapPoints[i] && !mvbOutlier[i]) {
        if (i < Nlim)
          left++;
        else
          right++;
      }
    }
    cout << "Point distribution in Frame: left-> " << left << " --- right-> "
         << right << endl;
  }
};

} // namespace RVWO

#endif // BLOCKS_FRAME_H
