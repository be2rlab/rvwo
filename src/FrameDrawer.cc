/**
 * This file is part of ORB-SLAM3
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

#include "FrameDrawer.h"

#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Tracking.h"

namespace ORB_SLAM3 {

FrameDrawer::FrameDrawer(Atlas* pAtlas) : both(false), mpAtlas(pAtlas) {
  mState = Tracking::SYSTEM_NOT_READY;
  mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  mImRight = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
}

cv::Mat FrameDrawer::DrawFrame(float imageScale) {
  cv::Mat im;
  vector<cv::KeyPoint>
      vIniKeys;  // Initialization: KeyPoints in reference frame
  vector<int>
      vMatches;  // Initialization: correspondeces with reference keypoints
  vector<cv::KeyPoint> vCurrentKeys;  // KeyPoints in current frame
  vector<cv::KeyPoint> vCurrentMovKeys; // Moving KeyPoints in current frame
  vector<bool> vbVO, vbMap;           // Tracked MapPoints in current frame
  vector<pair<cv::Point2f, cv::Point2f> > vTracks;
  int state;  // Tracking state
  vector<float> vCurrentDepth;
  float thDepth;

  Frame currentFrame;
  vector<MapPoint*> vpLocalMap;
  vector<cv::KeyPoint> vMatchesKeys;
  vector<MapPoint*> vpMatchedMPs;
  vector<cv::KeyPoint> vOutlierKeys;
  vector<MapPoint*> vpOutlierMPs;
  map<long unsigned int, cv::Point2f> mProjectPoints;
  map<long unsigned int, cv::Point2f> mMatchedInImage;

  cv::Scalar standardColor(0, 255, 0);
  cv::Scalar odometryColor(255, 0, 0);
  cv::Scalar matchingColor(0, 0, 255);

  // Copy variables within scoped mutex
  {
    unique_lock<mutex> lock(mMutex);
    state = mState;
    if (mState == Tracking::SYSTEM_NOT_READY) mState = Tracking::NO_IMAGES_YET;

    mIm.copyTo(im);

    if (mState == Tracking::NOT_INITIALIZED) {
      vCurrentKeys = mvCurrentKeys;
      vCurrentMovKeys = mvCurrentMovKeys;
      vIniKeys = mvIniKeys;
      vMatches = mvIniMatches;
      vTracks = mvTracks;
    } else if (mState == Tracking::OK) {
      vCurrentKeys = mvCurrentKeys;
      vCurrentMovKeys = mvCurrentMovKeys;
      vbVO = mvbVO;
      vbMap = mvbMap;

      currentFrame = mCurrentFrame;
      vpLocalMap = mvpLocalMap;
      vMatchesKeys = mvMatchedKeys;
      vpMatchedMPs = mvpMatchedMPs;
      vOutlierKeys = mvOutlierKeys;
      vpOutlierMPs = mvpOutlierMPs;
      mProjectPoints = mmProjectPoints;
      mMatchedInImage = mmMatchedInImage;

      vCurrentDepth = mvCurrentDepth;
      thDepth = mThDepth;

    } else if (mState == Tracking::LOST) {
      vCurrentKeys = mvCurrentKeys;
      vCurrentMovKeys = mvCurrentMovKeys;
    }
  }

  if (imageScale != 1.f) {
    int imWidth = im.cols / imageScale;
    int imHeight = im.rows / imageScale;
    cv::resize(im, im, cv::Size(imWidth, imHeight));
  }

  if (im.channels() < 3)  // this should be always true
    cvtColor(im, im, cv::COLOR_GRAY2BGR);

  // Draw
  if (state == Tracking::NOT_INITIALIZED) {
    for (unsigned int i = 0; i < vMatches.size(); i++) {
      if (vMatches[i] >= 0) {
        cv::Point2f pt1, pt2;
        if (imageScale != 1.f) {
          pt1 = vIniKeys[i].pt / imageScale;
          pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
        } else {
          pt1 = vIniKeys[i].pt;
          pt2 = vCurrentKeys[vMatches[i]].pt;
        }
        cv::line(im, pt1, pt2, standardColor);
      }
    }
    for (auto& vTrack : vTracks) {
      cv::Point2f pt1, pt2;
      if (imageScale != 1.f) {
        pt1 = vTrack.first / imageScale;
        pt2 = vTrack.second / imageScale;
      } else {
        pt1 = vTrack.first;
        pt2 = vTrack.second;
      }
      cv::line(im, pt1, pt2, standardColor, 5);
    }

  } else if (state == Tracking::OK)  // TRACKING
  {
    mnTracked = 0;
    mnTrackedVO = 0;
    const float r = 5;
    size_t n = vCurrentKeys.size();
    for (int i = 0; i < n; i++) {
      if (vbVO[i] || vbMap[i]) {
        cv::Point2f pt1, pt2;
        cv::Point2f point;
        if (imageScale != 1.f) {
          point = vCurrentKeys[i].pt / imageScale;
          float px = vCurrentKeys[i].pt.x / imageScale;
          float py = vCurrentKeys[i].pt.y / imageScale;
          pt1.x = px - r;
          pt1.y = py - r;
          pt2.x = px + r;
          pt2.y = py + r;
        } else {
          point = vCurrentKeys[i].pt;
          pt1.x = vCurrentKeys[i].pt.x - r;
          pt1.y = vCurrentKeys[i].pt.y - r;
          pt2.x = vCurrentKeys[i].pt.x + r;
          pt2.y = vCurrentKeys[i].pt.y + r;
        }

        // This is a match to a MapPoint in the map
        if (vbMap[i]) {
          cv::rectangle(im, pt1, pt2, standardColor);
          cv::circle(im, point, 2, standardColor, -1);
          mnTracked++;
        } else  // This is match to a "visual odometry" MapPoint created in the
                // last frame
        {
          cv::rectangle(im, pt1, pt2, odometryColor);
          cv::circle(im, point, 2, odometryColor, -1);
          mnTrackedVO++;
        }
      }
    }
    if (!mvCurrentKeys.empty()) {
            int nm = mvCurrentMovKeys.size();
            
            for(int i=0;i<nm;i++)
            {
                if(vbVO[i] || vbMap[i])
                {
                    cv::Point2f pt1,pt2;
                    cv::Point2f point;
                    if(imageScale != 1.f)
                    {
                        point = mvCurrentMovKeys[i].pt / imageScale;
                        float px = mvCurrentMovKeys[i].pt.x / imageScale;
                        float py = mvCurrentMovKeys[i].pt.y / imageScale;
                        pt1.x=px-r;
                        pt1.y=py-r;
                        pt2.x=px+r;
                        pt2.y=py+r;
                    }
                    else
                    {
                        point = mvCurrentMovKeys[i].pt;
                        pt1.x=mvCurrentMovKeys[i].pt.x-r;
                        pt1.y=mvCurrentMovKeys[i].pt.y-r;
                        pt2.x=mvCurrentMovKeys[i].pt.x+r;
                        pt2.y=mvCurrentMovKeys[i].pt.y+r;
                    }
    
                    // This is a match to a MapPoint in the map
                    // if(vbMap[i])
                    // {
                    //     cv::rectangle(im,pt1,pt2,matchingColor);
                    //     cv::circle(im,vCurrentMovKeys[i].pt,2,matchingColor,-1);
                    //     mnTracked++;
                    // }
                    // else // This is match to a "visual odometry" MapPoint created in the last frame
                    // {
                        cv::rectangle(im,pt1,pt2,odometryColor);
                        cv::circle(im,vCurrentMovKeys[i].pt,2,odometryColor,-1);
                        mnTrackedVO++;
                    // }
                }
            }
        }
  }

  cv::Mat imWithInfo;
  DrawTextInfo(im, state, imWithInfo);

  return imWithInfo;
}

cv::Mat FrameDrawer::DrawKeyFrame(float imageScale)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<cv::KeyPoint> vCurrentMovKeys; // Moving KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    vector<pair<cv::Point2f, cv::Point2f> > vTracks;
    int state; // Tracking state
    vector<float> vCurrentDepth;
    float thDepth;

    Frame currentFrame;
    vector<MapPoint*> vpLocalMap;
    vector<cv::KeyPoint> vMatchesKeys;
    vector<MapPoint*> vpMatchedMPs;
    vector<cv::KeyPoint> vOutlierKeys;
    vector<MapPoint*> vpOutlierMPs;
    map<long unsigned int, cv::Point2f> mProjectPoints;
    map<long unsigned int, cv::Point2f> mMatchedInImage;

    cv::Scalar standardColor(0,255,0);
    cv::Scalar odometryColor(255,0,0);
    cv::Scalar matchingColor(0,0,255);

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;
        
        mImKF.copyTo(im);
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            // vCurrentKeys = mvCurrentKeys;
            // vCurrentMovKeys = mvCurrentMovKeys;
            // vIniKeys = mvIniKeys;
            // vMatches = mvIniMatches;
            // vTracks = mvTracks;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vCurrentMovKeys = mvKFMovKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;

            currentFrame = mCurrentFrame;
            vpLocalMap = mvpLocalMap;
            vMatchesKeys = mvMatchedKeys;
            vpMatchedMPs = mvpMatchedMPs;
            vOutlierKeys = mvOutlierKeys;
            vpOutlierMPs = mvpOutlierMPs;
            mProjectPoints = mmProjectPoints;
            mMatchedInImage = mmMatchedInImage;

            vCurrentDepth = mvCurrentDepth;
            thDepth = mThDepth;

        }
        else if(mState==Tracking::LOST)
        {
            // vCurrentKeys = mvCurrentKeys;
            // vCurrentMovKeys = mvKFMovKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        // for(unsigned int i=0; i<vMatches.size(); i++)
        // {
        //     if(vMatches[i]>=0)
        //     {
        //         cv::Point2f pt1,pt2;
        //         if(imageScale != 1.f)
        //         {
        //             pt1 = vIniKeys[i].pt / imageScale;
        //             pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
        //         }
        //         else
        //         {
        //             pt1 = vIniKeys[i].pt;
        //             pt2 = vCurrentKeys[vMatches[i]].pt;
        //         }
        //         cv::line(im,pt1,pt2,standardColor);
        //      }
        // }        
        // for(vector<pair<cv::Point2f, cv::Point2f> >::iterator it=vTracks.begin(); it!=vTracks.end(); it++)
        // {
        //     cv::Point2f pt1,pt2;
        //     if(imageScale != 1.f)
        //     {
        //         pt1 = (*it).first / imageScale;
        //         pt2 = (*it).second / imageScale;
        //     }
        //     else
        //     {
        //         pt1 = (*it).first;
        //         pt2 = (*it).second;
        //     }
        //     cv::line(im,pt1,pt2, standardColor,5);
        // }

    }
    else if(state==Tracking::OK) //TRACKING
    {
        // mnTracked=0;
        // mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                cv::Point2f point;
                if(imageScale != 1.f)
                {
                    point = vCurrentKeys[i].pt / imageScale;
                    float px = vCurrentKeys[i].pt.x / imageScale;
                    float py = vCurrentKeys[i].pt.y / imageScale;
                    pt1.x=px-r;
                    pt1.y=py-r;
                    pt2.x=px+r;
                    pt2.y=py+r;
                }
                else
                {
                    point = vCurrentKeys[i].pt;
                    pt1.x=vCurrentKeys[i].pt.x-r;
                    pt1.y=vCurrentKeys[i].pt.y-r;
                    pt2.x=vCurrentKeys[i].pt.x+r;
                    pt2.y=vCurrentKeys[i].pt.y+r;
                }

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,standardColor);
                    cv::circle(im,point,2,standardColor,-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    // cv::rectangle(im,pt1,pt2,odometryColor);
                    // cv::circle(im,point,2,odometryColor,-1);
                    // mnTrackedVO++;
                }
            }
        }   // KeyPoint
        if(!mvCurrentMovKeys.empty())
        {
            int nm = mvKFMovKeys.size();

            for(int i=0;i<nm;i++)
            {

                cv::Point2f pt1,pt2;
                cv::Point2f point;
                if(imageScale != 1.f)
                {
                    point = mvKFMovKeys[i].pt / imageScale;
                    float px = mvKFMovKeys[i].pt.x / imageScale;
                    float py = mvKFMovKeys[i].pt.y / imageScale;
                    pt1.x=px-r;
                    pt1.y=py-r;
                    pt2.x=px+r;
                    pt2.y=py+r;
                }
                else
                {
                    point = mvKFMovKeys[i].pt;
                    pt1.x=mvKFMovKeys[i].pt.x-r;
                    pt1.y=mvKFMovKeys[i].pt.y-r;
                    pt2.x=mvKFMovKeys[i].pt.x+r;
                    pt2.y=mvKFMovKeys[i].pt.y+r;
                }

                // This is a moving point in the KF
                cv::rectangle(im,pt1,pt2,matchingColor);
                cv::circle(im,mvKFMovKeys[i].pt,2,matchingColor,-1);
            }
        }   // Moving KeyPoint by K-Means
        if(!mvKFSegKeys.empty())
        {
            int nm = mvKFSegKeys.size();
            for(int i=0;i<nm;i++)
            {

                cv::Point2f pt1,pt2;
                cv::Point2f point;
                if(imageScale != 1.f)
                {
                    point = mvKFSegKeys[i].pt / imageScale;
                    float px = mvKFSegKeys[i].pt.x / imageScale;
                    float py = mvKFSegKeys[i].pt.y / imageScale;
                    pt1.x=px-r;
                    pt1.y=py-r;
                    pt2.x=px+r;
                    pt2.y=py+r;
                }
                else
                {
                    point = mvKFSegKeys[i].pt;
                    pt1.x=mvKFSegKeys[i].pt.x-r;
                    pt1.y=mvKFSegKeys[i].pt.y-r;
                    pt2.x=mvKFSegKeys[i].pt.x+r;
                    pt2.y=mvKFSegKeys[i].pt.y+r;
                }
                // This is a moving point in the KF
                cv::rectangle(im,pt1,pt2,odometryColor);
                cv::circle(im,mvKFSegKeys[i].pt,2,odometryColor,-1);
            }
        }   // Moving KeyPoint by SegNet
    }

    // cv::Mat imWithInfo;
    // DrawTextInfo(im,state, imWithInfo);

    return im;
}

cv::Mat FrameDrawer::DrawRightFrame(float imageScale) {
  cv::Mat im;
  vector<cv::KeyPoint>
      vIniKeys;  // Initialization: KeyPoints in reference frame
  vector<int>
      vMatches;  // Initialization: correspondeces with reference keypoints
  vector<cv::KeyPoint> vCurrentKeys;  // KeyPoints in current frame
  vector<bool> vbVO, vbMap;           // Tracked MapPoints in current frame
  int state;                          // Tracking state

  // Copy variables within scoped mutex
  {
    unique_lock<mutex> lock(mMutex);
    state = mState;
    if (mState == Tracking::SYSTEM_NOT_READY) mState = Tracking::NO_IMAGES_YET;

    mImRight.copyTo(im);

    if (mState == Tracking::NOT_INITIALIZED) {
      vCurrentKeys = mvCurrentKeysRight;
      vIniKeys = mvIniKeys;
      vMatches = mvIniMatches;
    } else if (mState == Tracking::OK) {
      vCurrentKeys = mvCurrentKeysRight;
      vbVO = mvbVO;
      vbMap = mvbMap;
    } else if (mState == Tracking::LOST) {
      vCurrentKeys = mvCurrentKeysRight;
    }
  }  // destroy scoped mutex -> release mutex

  if (imageScale != 1.f) {
    int imWidth = im.cols / imageScale;
    int imHeight = im.rows / imageScale;
    cv::resize(im, im, cv::Size(imWidth, imHeight));
  }

  if (im.channels() < 3)  // this should be always true
    cvtColor(im, im, cv::COLOR_GRAY2BGR);

  // Draw
  if (state == Tracking::NOT_INITIALIZED)  // INITIALIZING
  {
    for (unsigned int i = 0; i < vMatches.size(); i++) {
      if (vMatches[i] >= 0) {
        cv::Point2f pt1, pt2;
        if (imageScale != 1.f) {
          pt1 = vIniKeys[i].pt / imageScale;
          pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
        } else {
          pt1 = vIniKeys[i].pt;
          pt2 = vCurrentKeys[vMatches[i]].pt;
        }

        cv::line(im, pt1, pt2, cv::Scalar(0, 255, 0));
      }
    }
  } else if (state == Tracking::OK)  // TRACKING
  {
    mnTracked = 0;
    mnTrackedVO = 0;
    const float r = 5;
    const size_t n = mvCurrentKeysRight.size();
    const size_t Nleft = mvCurrentKeys.size();

    for (int i = 0; i < n; i++) {
      if (vbVO[i + Nleft] || vbMap[i + Nleft]) {
        cv::Point2f pt1, pt2;
        cv::Point2f point;
        if (imageScale != 1.f) {
          point = mvCurrentKeysRight[i].pt / imageScale;
          float px = mvCurrentKeysRight[i].pt.x / imageScale;
          float py = mvCurrentKeysRight[i].pt.y / imageScale;
          pt1.x = px - r;
          pt1.y = py - r;
          pt2.x = px + r;
          pt2.y = py + r;
        } else {
          point = mvCurrentKeysRight[i].pt;
          pt1.x = mvCurrentKeysRight[i].pt.x - r;
          pt1.y = mvCurrentKeysRight[i].pt.y - r;
          pt2.x = mvCurrentKeysRight[i].pt.x + r;
          pt2.y = mvCurrentKeysRight[i].pt.y + r;
        }

        // This is a match to a MapPoint in the map
        if (vbMap[i + Nleft]) {
          cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
          cv::circle(im, point, 2, cv::Scalar(0, 255, 0), -1);
          mnTracked++;
        } else  // This is match to a "visual odometry" MapPoint created in the
                // last frame
        {
          cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 0));
          cv::circle(im, point, 2, cv::Scalar(255, 0, 0), -1);
          mnTrackedVO++;
        }
      }
    }
  }

  cv::Mat imWithInfo;
  DrawTextInfo(im, state, imWithInfo);

  return imWithInfo;
}

void FrameDrawer::DrawTextInfo(cv::Mat& im, int nState, cv::Mat& imText) {
  stringstream s;
  if (nState == Tracking::NO_IMAGES_YET)
    s << " WAITING FOR IMAGES";
  else if (nState == Tracking::NOT_INITIALIZED)
    s << " TRYING TO INITIALIZE ";
  else if (nState == Tracking::OK) {
    if (!mbOnlyTracking)
      s << "SLAM MODE |  ";
    else
      s << "LOCALIZATION | ";
    int nMaps = mpAtlas->CountMaps();
    int nKFs = mpAtlas->KeyFramesInMap();
    int nMPs = mpAtlas->MapPointsInMap();
    s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs
      << ", Matches: " << mnTracked;
    if (mnTrackedVO > 0) s << ", + VO matches: " << mnTrackedVO;
  } else if (nState == Tracking::LOST) {
    s << " TRACK LOST. TRYING TO RELOCALIZE ";
  } else if (nState == Tracking::SYSTEM_NOT_READY) {
    s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
  }

  int baseline = 0;
  cv::Size textSize =
      cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

  imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
  im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
  imText.rowRange(im.rows, imText.rows) =
      cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
  cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
}

void FrameDrawer::Update(Tracking* pTracker) {
  unique_lock<mutex> lock(mMutex);
  pTracker->mImGray.copyTo(mIm);
  pTracker->mCurrentFrame.mpReferenceKF->gray_.copyTo(mImKF);
  mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
  mvCurrentMovKeys=pTracker->mCurrentFrame.mvDynKeys;
  mvKFMovKeys=pTracker->mCurrentFrame.mpReferenceKF->mvDynKeysUn;
  mvKFSegKeys=pTracker->mCurrentFrame.mpReferenceKF->mvSegDynKeys;
  mThDepth = pTracker->mCurrentFrame.mThDepth;
  mvCurrentDepth = pTracker->mCurrentFrame.mvDepth;

  if (both) {
    mvCurrentKeysRight = pTracker->mCurrentFrame.mvKeysRight;
    pTracker->mImRight.copyTo(mImRight);
    N = mvCurrentKeys.size() + mvCurrentKeysRight.size();
  } else {
    N = mvCurrentKeys.size();
  }

  mvbVO = vector<bool>(N, false);
  mvbMap = vector<bool>(N, false);
  mbOnlyTracking = pTracker->mbOnlyTracking;

  // Variables for the new visualization
  mCurrentFrame = pTracker->mCurrentFrame;
  mmProjectPoints = mCurrentFrame.mmProjectPoints;
  mmMatchedInImage.clear();

  mvpLocalMap = pTracker->GetLocalMapMPS();
  mvMatchedKeys.clear();
  mvMatchedKeys.reserve(N);
  mvpMatchedMPs.clear();
  mvpMatchedMPs.reserve(N);
  mvOutlierKeys.clear();
  mvOutlierKeys.reserve(N);
  mvpOutlierMPs.clear();
  mvpOutlierMPs.reserve(N);

  if (pTracker->mLastProcessedState == Tracking::NOT_INITIALIZED) {
    mvIniKeys = pTracker->mInitialFrame.mvKeys;
    mvIniMatches = pTracker->mvIniMatches;
  } else if (pTracker->mLastProcessedState == Tracking::OK) {
    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
      if (pMP) {
        if (!pTracker->mCurrentFrame.mvbOutlier[i]) {
          if (pMP->Observations() > 0)
            mvbMap[i] = true;
          else
            mvbVO[i] = true;

          mmMatchedInImage[pMP->mnId] = mvCurrentKeys[i].pt;
        } else {
          mvpOutlierMPs.push_back(pMP);
          mvOutlierKeys.push_back(mvCurrentKeys[i]);
        }
      }
    }
  }
  mState = static_cast<int>(pTracker->mLastProcessedState);
}

}  // namespace ORB_SLAM3