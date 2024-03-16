#include <iostream>
#include <fstream>
#include "DynamicCulling.h"

namespace ORB_SLAM3
{
DynamicCulling::DynamicCulling(): mbAcceptKeyFrames(true), mbFinishRequested(false), mbFinished(true)
                                 
{}

void DynamicCulling::SetLocalMapper(LocalMapping* pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
} // SetLocalMapper

void DynamicCulling::SetTracker(Tracking* pTracker)
{
    mpTracker = pTracker;
} // SetTracker

void DynamicCulling::SetDynamicDetector(DynamicDetector* pDynamicDetector)
{
    mpDynamicDetector = pDynamicDetector;
}

void DynamicCulling::Run()
{
    int skip_kf = 1;
    int num_pro_kf = 0;
    while(1)
    {
        // Tracking will see that DynamicCulling is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Get KF from newlKeyFrames 
            KeyFrame* pKF = GetNewKeyFrame();

            if(num_pro_kf < skip_kf)
            {
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
                mpDynamicDetector->removeDynamicPixels(pKF);
                // num_pro_kf++;
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

                double tdetectKmeans= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
                vTimesDetectKmeans.push_back(tdetectKmeans);
                
            }
            else 
                num_pro_kf = 0;
            // Send KeyFrame to LocalMapping
            // Local Mapping accept keyframes?
            bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();
            if(bLocalMappingIdle)
                mpLocalMapper->InsertKeyFrame(pKF);
            else 
            {
                mpLocalMapper->InterruptBA();
                if(mpLocalMapper->KeyframesInQueue() < 3)   
                    mpLocalMapper->InsertKeyFrame(pKF);
            }    
        }// if there are keyframe
        if(CheckFinish())
            break;
        // Tracking will see that DynamicCulling is free
        SetAcceptKeyFrames(true);
        usleep(3000);
    }
    
    // K-Means time statistics
    sort(vTimesDetectKmeans.begin(),vTimesDetectKmeans.end());
    float totaltime = 0;
    for(int ni=0; ni<vTimesDetectKmeans.size(); ni++)
    {
        totaltime+=vTimesDetectKmeans[ni];
    }
    ofstream tStatics;
    cout << "-------" << endl << endl;
    cout << "Total KFs K-Means processed: " << vTimesDetectKmeans.size() << endl;
    cout << "Median K-Means time: " << vTimesDetectKmeans[vTimesDetectKmeans.size()/2] << endl;
    cout << "Mean K-Means time: " << totaltime/vTimesDetectKmeans.size() << endl;
    tStatics.open("time_statistic_K-Means.txt");
    tStatics << "Total KFs K-Means processed: " << vTimesDetectKmeans.size() << endl;
    tStatics << "Median K-Means time: " << vTimesDetectKmeans[vTimesDetectKmeans.size()/2] << endl;
    tStatics << "Mean K-Means time: " << totaltime/vTimesDetectKmeans.size() << endl;
    tStatics.close();
    SetFinish();

} // run

void DynamicCulling::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
} // SetAcceptKeyFrames

bool DynamicCulling::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void DynamicCulling::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
} // InsertKeyFrame

bool DynamicCulling::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
} // CheckNewKeyFrames

KeyFrame* DynamicCulling::GetNewKeyFrame()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    KeyFrame* NewKeyFrame = mlNewKeyFrames.front();
    mlNewKeyFrames.pop_front();
    return NewKeyFrame;
} // GetNewKeyFrame

void DynamicCulling::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool DynamicCulling::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void DynamicCulling::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
}

bool DynamicCulling::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //ORB_SLAM
