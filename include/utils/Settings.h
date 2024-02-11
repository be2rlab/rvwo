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

#ifndef UTILS_SETTINGS_H
#define UTILS_SETTINGS_H

// Flag to activate the measurement of time in each process (track,localmap,
// place recognition).
// #define REGISTER_TIMES

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

#include "CameraModels/GeometricCamera.h"

namespace RVWO {

class System;

class Settings {
public:
  // ** Enum for the different camera types implemented ** //
  enum CameraType { PinHole = 0, Rectified = 1, KannalaBrandt = 2 };

  // ** Delete default constructor ** //
  Settings() = delete;

  // ** Constructor from file ** //
  /** @brief Constructor from file
   * @param configFile: configuration file path
   * @param sensor: sensor type
   */
  Settings(const std::string &configFile, const int &sensor);

  /*
   * Ostream operator overloading to dump settings to the terminal
   */
  friend std::ostream &operator<<(std::ostream &output, const Settings &s);

  // ** Getter methods ** //
  /** @brief Get the camera type
   * @return camera type
   */
  CameraType cameraType() { return cameraType_; }
  /** @brief Get Camera 1
   * @return pointer to the camera
   */
  GeometricCamera *camera1() { return calibration1_; }

  /** @brief Get Camera 2
   * @return pointer to the camera
   */
  GeometricCamera *camera2() { return calibration2_; }

  /** @brief Camera 1 Distortion coefficents
   * @return distortion coefficients
   */
  cv::Mat camera1DistortionCoef() {
    return cv::Mat(vPinHoleDistorsion1_.size(), 1, CV_32F,
                   vPinHoleDistorsion1_.data());
  }
  /** @brief Camera 2 Distortion coefficents
   * @return distortion coefficients
   */
  cv::Mat camera2DistortionCoef() {
    return cv::Mat(vPinHoleDistorsion2_.size(), 1, CV_32F,
                   vPinHoleDistorsion1_.data());
  }
  /** @brief Get the transformation from left to right camera
   * @return transformation
   */
  Sophus::SE3f Tlr() { return Tlr_; }
  /** @brief Get the baseline times the focal length
   * @return baseline times the focal length
   */
  float bf() { return bf_; }
  /** @brief Get the baseline
   * @return baseline
   */
  float b() { return b_; }
  /** @brief Get the depth threshold
   * @return depth threshold
   */
  float thDepth() { return thDepth_; }
  /** @brief need to undistort
   * @return true if need to undistort
   */
  bool needToUndistort() { return bNeedToUndistort_; }
  /** @brief Get the new image size
   * @return new image size
   */
  cv::Size newImSize() { return newImSize_; }
  /** @brief get the fps of the camera
   * @return fps
   */
  float fps() { return fps_; }
  /** @brief check if rgb
   *  @return true if rgb
   */
  bool rgb() { return bRGB_; }
  /** @brief check if need to resize
   *  @return true if need to resize
   */
  bool needToResize() { return bNeedToResize1_; }
  /** @brief check if need to rectify
   *  @return true if need to rectify
   */
  bool needToRectify() { return bNeedToRectify_; }

  /** @brief Get transformation between camera and device
   * @return transformation
   */
  Sophus::SE3f Tbc() { return Tbc_; }
  /** boolean if we insert keyframe when lost
   * @return true if we insert keyframe when lost
   */
  bool insertKFsWhenLost() const { return insertKFsWhenLost_; }

  /** @brief return the depth map factor
   * @return return the depth map factor
   */
  float depthMapFactor() const { return depthMapFactor_; }

  /** @brief get the transformation between the device and the wheel odometry
   * @return transformation
   */
  Sophus::SE3f Tbo() { return Tbo_; }
  // ** Encoder of the wheels ** //
  /** @brief get the noise of the encoder in x
   * @return noise of the encoder in x
   */
  float noiseX() const { return NoiseX_; }
  /** @brief get the noise of the encoder in y
   * @return noise of the encoder in y
   */

  float noiseY() const { return NoiseY_; }
  /** @brief get the noise of the encoder in z
   * @return noise of the encoder in z
   */
  float noiseRotZ_() const { return NoiseRotZ_; }

  /** @brief get the number of features
   * @return number of features
   */
  int nFeatures() const { return nFeatures_; }
  /** @brief get the number of levels
   * @return number of levels
   */
  int nLevels() const { return nLevels_; }
  /** @brief get the initial threshold for FAST
   * @return initial threshold for FAST
   */
  float initThFAST() const { return initThFAST_; }
  /** @brief get the minimum threshold for FAST
   * @return minimum threshold for FAST
   */
  float minThFAST() const { return minThFAST_; }
  /** @brief get the scale factor
   * @return scale factor
   */
  float scaleFactor() const { return scaleFactor_; }

  /** @brief get the keyframe size
   * @return keyframe size
   */
  float keyFrameSize() const { return keyFrameSize_; }
  /** @brief get the keyframe line width
   * @return keyframe line width
   */
  float keyFrameLineWidth() const { return keyFrameLineWidth_; }
  /** @brief get the graph line width
   * @return graph line width
   */
  float graphLineWidth() const { return graphLineWidth_; }
  /** @brief get the point size
   * @return point size
   */
  float pointSize() const { return pointSize_; }
  /** @brief get the camera size
   * @return camera size
   */
  float cameraSize() const { return cameraSize_; }
  /** @brief get the camera line width
   * @return camera line width
   */
  float cameraLineWidth() const { return cameraLineWidth_; }
  /** @brief get the view point x
   * @return view point x
   */
  float viewPointX() const { return viewPointX_; }
  /** @brief get the view point y
   * @return view point y
   */
  float viewPointY() const { return viewPointY_; }
  /** @brief get the view point z
   * @return view point z
   */
  float viewPointZ() const { return viewPointZ_; }
  /** @brief get the view point f
   * @return view point f
   */
  float viewPointF() const { return viewPointF_; }
  /** @brief get the image viewer scale
   * @return image viewer scale
   */
  float imageViewerScale() const { return imageViewerScale_; }

  /** @brief get the file to load the atlas
   * @return file to load the atlas
   */
  std::string atlasLoadFile() { return sLoadFrom_; }
  /** @brief get the file to save the atlas
   * @return file to save the atlas
   */
  std::string atlasSaveFile() { return sSaveto_; }
  /** @brief get the threshold for far points
   * @return threshold for far points
   */
  float thFarPoints() const { return thFarPoints_; }
  / cv::Mat M1l() { return M1l_; }
  cv::Mat M2l() { return M2l_; }
  cv::Mat M1r() { return M1r_; }
  cv::Mat M2r() { return M2r_; }

private:
  template <typename T>
  T readParameter(cv::FileStorage &fSettings, const std::string &name,
                  bool &found, const bool required = true) {
    cv::FileNode node = fSettings[name];
    if (node.empty()) {
      if (required) {
        std::cerr << name << " required parameter does not exist, aborting..."
                  << std::endl;
        exit(-1);
      } else {
        std::cerr << name << " optional parameter does not exist..."
                  << std::endl;
        found = false;
        return T();
      }

    } else {
      found = true;
      return (T)node;
    }
  }

  void readCamera1(cv::FileStorage &fSettings);
  void readCamera2(cv::FileStorage &fSettings);
  void readImageInfo(cv::FileStorage &fSettings);
  void readIMU(cv::FileStorage &fSettings);
  void readRGBD(cv::FileStorage &fSettings);
  void readOdom(cv::FileStorage &fSettings);
  void readORB(cv::FileStorage &fSettings);
  void readViewer(cv::FileStorage &fSettings);
  void readLoadAndSave(cv::FileStorage &fSettings);
  void readOtherParameters(cv::FileStorage &fSettings);

  void precomputeRectificationMaps();

  int sensor_;
  CameraType cameraType_; // Camera type

  /*
   * Visual stuff
   */
  GeometricCamera *calibration1_, *calibration2_; // Camera calibration
  GeometricCamera *originalCalib1_, *originalCalib2_;
  std::vector<float> vPinHoleDistorsion1_, vPinHoleDistorsion2_;

  cv::Size originalImSize_, newImSize_;
  float fps_;
  bool bRGB_;

  bool bNeedToUndistort_;
  bool bNeedToRectify_;
  bool bNeedToResize1_, bNeedToResize2_;

  Sophus::SE3f Tlr_;
  float thDepth_;
  float bf_, b_;

  /*
   * Rectification stuff
   */
  cv::Mat M1l_, M2l_;
  cv::Mat M1r_, M2r_;

  /*
   * Inertial stuff
   */
  float noiseGyro_, noiseAcc_;
  float gyroWalk_, accWalk_;
  float imuFrequency_;
  Sophus::SE3f Tbc_;
  bool insertKFsWhenLost_;

  /*
   * RGBD stuff
   */
  float depthMapFactor_;

  /*
   * WOdometry stuff
   */
  float NoiseX_, NoiseY_, NoiseRotZ_;
  Sophus::SE3f Tbo_;

  /*
   * ORB stuff
   */
  int nFeatures_;
  float scaleFactor_;
  int nLevels_;
  int initThFAST_, minThFAST_;

  /*
   * Viewer stuff
   */
  float keyFrameSize_;
  float keyFrameLineWidth_;
  float graphLineWidth_;
  float pointSize_;
  float cameraSize_;
  float cameraLineWidth_;
  float viewPointX_, viewPointY_, viewPointZ_, viewPointF_;
  float imageViewerScale_;

  /*
   * Save & load maps
   */
  std::string sLoadFrom_, sSaveto_;

  /*
   * Other stuff
   */
  float thFarPoints_;
};
}; // namespace RVWO

#endif // UTILS_SETTINGS_H
