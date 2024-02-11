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

#ifndef BLOCKS_KEYFRAMEDATABASE_H
#define BLOCKS_KEYFRAMEDATABASE_H

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <list>
#include <mutex>
#include <set>
#include <vector>

#include "Blocks/Frame.h"
#include "Blocks/KeyFrame.h"
#include "Blocks/Map.h"
#include "ORBVocabulary.h"

namespace RVWO {

class KeyFrame;
class Frame;
class Map;

/** @brief
 * This class is used to store and retrieve keyframes efficiently.
 * It uses the bag of words representation of the images.
 * It is based on the DBoW2 library by Dorian Galvez-Lopez.
 */
class KeyFrameDatabase {
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar & mvBackupInvertedFileId;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // ** Constructors ** //
  KeyFrameDatabase() {}

  /** @brief constructor
   * @param voc ORB Vocabulary
   */
  KeyFrameDatabase(const ORBVocabulary &voc);

  /** @brief add keyframe to database
   * @param pKF KeyFrame to add
   */
  void add(KeyFrame *pKF);

  /** @brief erase keyframe from database
   * @param pKF KeyFrame to erase
   */
  void erase(KeyFrame *pKF);

  /** @brief clear the database*/
  void clear();
  /** @brief clear specific map
   * @param pMap Map to clear
   */
  void clearMap(Map *pMap);

  /** @brief Loop Detection(DEPRECATED)
   * @param pKF KeyFrame to detect loop
   * @param minScore minimum score to consider a candidate
   * @return vector of KeyFrame candidates
   * */
  std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame *pKF, float minScore);

  // ** Loop and Merge Detection ** //
  /** @brief Detect Loop and Merge Candidates
   * @param pKF KeyFrame to detect loop
   * @param minScore minimum score to consider a candidate
   * @param vpLoopCand vector of KeyFrame loop candidates
   * @param vpMergeCand vector of KeyFrame merge candidates
   * */
  void DetectCandidates(KeyFrame *pKF, float minScore,
                        vector<KeyFrame *> &vpLoopCand,
                        vector<KeyFrame *> &vpMergeCand);
  /** @brief Detect Best Candidates
   * @param pKF KeyFrame to detect loop
   * @param vpLoopCand vector of KeyFrame loop candidates
   * @param vpMergeCand vector of KeyFrame merge candidates
   * @param nMinWords minimum number of words to consider a candidate
   */
  void DetectBestCandidates(KeyFrame *pKF, vector<KeyFrame *> &vpLoopCand,
                            vector<KeyFrame *> &vpMergeCand, int nMinWords);

  /** @brief Detect N Best candidtates
   * @param pKF KeyFrame to detect loop
   * @param vpLoopCand vector of KeyFrame loop candidates
   * @param vpMergeCand vector of KeyFrame merge candidates
   * @param nNumCandidates number of candidates to detect
   */
  void DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame *> &vpLoopCand,
                             vector<KeyFrame *> &vpMergeCand,
                             int nNumCandidates);

  // ** Relocalization ** //
  /** @brief Detect Relocalization Candidates
   * @param F Frame to detect relocalization
   * @param pMap Map to detect relocalization
   * @return vector of KeyFrame candidates
   */
  std::vector<KeyFrame *> DetectRelocalizationCandidates(Frame *F, Map *pMap);

  // ** Save and Load ** //
  void PreSave();
  void PostLoad(map<long unsigned int, KeyFrame *> mpKFid);

  /** @brief Set ORB Vocabulary
   * @param pORBVoc ORB Vocabulary
   */
  void SetORBVocabulary(ORBVocabulary *pORBVoc);

protected:
  // Associated vocabulary
  const ORBVocabulary *mpVoc;

  // Inverted file
  std::vector<list<KeyFrame *>> mvInvertedFile;

  // For save relation without pointer, this is necessary for save/load function
  std::vector<list<long unsigned int>> mvBackupInvertedFileId;

  // Mutex
  std::mutex mMutex;
};

} // namespace RVWO

#endif
