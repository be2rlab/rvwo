#ifndef DYNAMICCULLING_H
#define DYNAMICCULLING_H

#include "DynamicCulling.h"
#include "DynamicDetector.h"
#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Tracking.h"

namespace ORB_SLAM3
{
class DynamicDetector;
class LocalMapping;
class KeyFrame;
class Tracking;

class DynamicCulling
{
public:
    DynamicCulling();
    void Run();
    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetTracker(Tracking* pTracking);
    void SetDynamicDetector(DynamicDetector* pDynamicDetector);
    void InsertKeyFrame(KeyFrame* pKF);
    // Thread Synch
    void SetAcceptKeyFrames(bool flag);
    void RequestFinish();
    bool AcceptKeyFrames();
    bool CheckNewKeyFrames();
    bool CheckFinish();
    bool isFinished();

    KeyFrame* GetNewKeyFrame();
    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }
private:
    // pointers
    LocalMapping* mpLocalMapper;
    Tracking* mpTracker;
    Map* Mapper;
	DynamicDetector* mpDynamicDetector;
    
    // thread.
	std::mutex mMutexNewKFs;
	std::list<KeyFrame*> mlNewKeyFrames; 
	
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
}

#endif