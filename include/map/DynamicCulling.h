#ifndef MAP_DYNAMICCULLING_H
#define MAP_DYNAMICCULLING_H

#include "DynamicCulling.h"
#include "DynamicDetector.h"
#include "map/blocks/key_frame.h"
#include "map/local_mapping.h"
#include "map/tracking.h"

namespace RVWO {
class DynamicDetector;
class LocalMapping;
class KeyFrame;
class Tracking;

/** @brief class for Dynamic culling of map elements
 */
class DynamicCulling {
public:
  /**  Constructor */
  DynamicCulling();
  /** @brief Run the dynamic culling thread.
   * This function is called when the thread is launched.
   */
  void Run();

  /** @brief Set the pointers to the local mapper.
   * @param pLocalMapper: pointer to the local mapper
   */
  void SetLocalMapper(LocalMapping *pLocalMapper);
  /** @brief Set the pointers to the tracker.
   * @param pTracking: pointer to the tracker
   */
  void SetTracker(Tracking *pTracking);
  /** @brief Set the pointers to the dynamic detector
   * @param pDynamicDetector: pointer to the dynamic detector
   */
  void SetDynamicDetector(DynamicDetector *pDynamicDetector);

  /** @brief Insert a keyframe in the queue.
   * @param pKF: pointer to the keyframe
   */
  void InsertKeyFrame(KeyFrame *pKF);

  //  ** Thread Sync ** //
  /** @brief Set the flag to accept keyframes.
   * @param flag: flag to accept keyframes
   */
  void SetAcceptKeyFrames(bool flag);
  /** @brief Request the finish of the thread.
   */
  void RequestFinish();
  /** @brief Check if the thread can accept keyframes.
   * @return true if the thread can accept keyframes
   */
  bool AcceptKeyFrames();
  /** @brief Check if there are new keyframes in the queue.
   * @return true if there are new keyframes in the queue
   */
  bool CheckNewKeyFrames();
  /** @brief Check if the thread has been requested to finish.
   * @return true if the thread has been requested to finish
   */
  bool CheckFinish();
  /** @brief Check if the thread has finished.
   * @return true if the thread has finished
   */
  bool isFinished();

  // ** Getters ** //
  /** @brief Get the new keyframe from the queue.
   * @return pointer to the new keyframe
   */
  KeyFrame *GetNewKeyFrame();
  /** @brief Get the number of keyframes in the queue.
   * @return number of keyframes in the queue
   */
  int KeyframesInQueue() {
    unique_lock<std::mutex> lock(mMutexNewKFs);
    return mlNewKeyFrames.size();
  }

private:
  // ** pointers ** //
  LocalMapping *mpLocalMapper;
  Tracking *mpTracker;
  Map *Mapper;
  DynamicDetector *mpDynamicDetector;

  // thread.
  std::mutex mMutexNewKFs;
  std::list<KeyFrame *> mlNewKeyFrames;

  bool mbAcceptKeyFrames;
  std::mutex mMutexAccept;

  void SetFinish();
  bool mbFinishRequested;
  bool mbFinished;
  std::mutex mMutexFinish;

  // debug
  // Vector for tracking time statistics
  vector<float> vTimesDetectKmeans;
};
} // namespace RVWO

#endif