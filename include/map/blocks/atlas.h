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

#ifndef MAP_ATLAS_H
#define MAP_ATLAS_H

#include "geometry/camera_models/geometric_camera.h"
#include "geometry/camera_models/kannala_brandt8.h"
#include "geometry/camera_models/pinhole.h"
#include "map/blocks/key_frame.h"
#include "map/blocks/map.h"
#include "map/blocks/map_point.h"

#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <mutex>
#include <set>

namespace RVWO {

class Viewer;
class Map;
class MapPoint;
class KeyFrame;
class KeyFrameDatabase;
class Frame;
class KannalaBrandt8;
class Pinhole;

class Atlas {
  friend class boost::serialization::access;

  /**
   * @brief This method is used to serialize the Atlas object
   */
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar & mvpBackupMaps;
    ar & mvpCameras;
    ar &Map::nNextId;
    ar &Frame::nNextId;
    ar &KeyFrame::nNextId;
    ar &MapPoint::nNextId;
    ar &GeometricCamera::nNextId;
    ar & mnLastInitKFidMap;
  }

public:
  // ** Constructors ** //
  Atlas();
  Atlas(int initKFid); // When its initialization the first map is created
  ~Atlas();

  /** @brief create new map in Atlas */
  void CreateNewMap();
  /** @brief change the map*/
  void ChangeMap(Map *pMap);

  /** @brief get ID of the last Key frame in initialization */
  unsigned long int GetLastInitKFid();

  /** @brief Set the Viewer
   * @param pViewer: pointer to the viewer
   */
  void SetViewer(Viewer *pViewer);

  // ** Method for change components in the current map  ** //
  /** @brief Add new key frame to the map
   * @param pKF: pointer to the key frame
   */
  void AddKeyFrame(KeyFrame *pKF);
  /** @brief Add new map point to the map
   * @param pMP: pointer to the map point
   */
  void AddMapPoint(MapPoint *pMP);
  /** @brief Add new camera to the map
   * @param pCam: pointer to the camera
   */
  void AddCamera(GeometricCamera *pCam);

  // ** All methods without Map pointer work on current map ** //

  /** @brief Set Reference Map points in current map
   * @param vpMPs: vector of pointers to the map points
   */
  void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);
  /** @brief Inform new big change in current map */
  void InformNewBigChange();
  /** @brief Get the last big change index in current map
   * @return index of the last big change
   */
  int GetLastBigChangeIdx();

  /** @brief get number of map points in the map
   * @return number of map points
   */
  long unsigned int MapPointsInMap();

  /** @brief get number of key frames in the map
   * @return number of key frames
   */
  long unsigned KeyFramesInMap();

  // ** Method for get data in current map ** //

  /** @brief Get all key frames in current map
   * @return vector of pointers to the key frames
   */
  std::vector<KeyFrame *> GetAllKeyFrames();

  /** @brief Get all map points in current map
   * @return vector of pointers to the map points
   */
  std::vector<MapPoint *> GetAllMapPoints();

  /** @brief Get reference map points in current map
   * @return vector of pointers to the map points
   */
  std::vector<MapPoint *> GetReferenceMapPoints();

  /** @brief Get all the maps
   * @return vector of pointers to the maps
   */
  vector<Map *> GetAllMaps();

  /** @brief Get the number of maps
   * @return number of maps
   */

  /** @brief count of the maps
   * @return number of maps
   */
  int CountMaps();

  /** @brief clear map */
  void clearMap();

  /** @brief clear Atlas */
  void clearAtlas();

  /** @brief get current map
   * @return pointer to the current map
   */
  Map *GetCurrentMap();

  /** @brief set Map as bad
   * @param pMap: pointer to the map
   */
  void SetMapBad(Map *pMap);
  /** @brief remove all bad maps */
  void RemoveBadMaps();

  // ** Function for garantee the correction of serialization of this object **
  // //
  /** @brief save the map*/
  void PreSave();
  /** @brief load the map*/
  void PostLoad();

  /** @brief Set the key frame database
   * @param pKFDB: pointer to the key frame database
   */
  void SetKeyFrameDababase(KeyFrameDatabase *pKFDB);

  /** @brief Get Key Frame DataBase
   * @return pointer to the key frame database
   */
  KeyFrameDatabase *GetKeyFrameDatabase();

  /** @brief Set the ORB Vocabulary
   * @param pORBVoc: pointer to the ORB vocabulary
   */
  void SetORBVocabulary(ORBVocabulary *pORBVoc);

  /** @brief Get ORB Vocabulary
   * @return pointer to the ORB vocabulary
   */
  ORBVocabulary *GetORBVocabulary();

  /** @brief number of all live Key frames
   * @return number of key frames
   */
  long unsigned int GetNumLivedKF();

  /** @brief number of all live Map points
   * @return number of map points
   */
  long unsigned int GetNumLivedMP();

protected:
  std::set<Map *> mspMaps;
  std::set<Map *> mspBadMaps;
  std::vector<Map *> mvpBackupMaps;
  Map *mpCurrentMap;

  std::vector<GeometricCamera *> mvpCameras;
  std::vector<KannalaBrandt8 *> mvpBackupCamKan;
  std::vector<Pinhole *> mvpBackupCamPin;

  std::mutex mMutexAtlas;

  unsigned long int mnLastInitKFidMap;

  Viewer *mpViewer;
  bool mHasViewer;

  // ** Class references for the map reconstruction from the save file ** //
  KeyFrameDatabase *mpKeyFrameDB;
  ORBVocabulary *mpORBVocabulary;

}; // class Atlas

} // namespace RVWO

#endif // MAP_ATLAS_H
