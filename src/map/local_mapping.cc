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

#include "map/local_mapping.h"

#include <chrono>
#include <mutex>

#include "geometry/geometric_tools.h"
#include "map/loop_closing.h"
#include "map/optimizer.h"
#include "utils/converter.h"
#include "utils/orb_matcher.h"

namespace RVWO {

    LocalMapping::LocalMapping(System *pSys, Atlas *pAtlas, const float bMonocular,
                               const int kernel, const string &_strSeqName)
            : mpSystem(pSys),
              mbMonocular(bMonocular),
              mKernel(kernel),
              mbResetRequested(false),
              mbResetRequestedActiveMap(false),
              mbFinishRequested(false),
              mbFinished(true),
              mpAtlas(pAtlas),
              mbAbortBA(false),
              mbStopped(false),
              mbStopRequested(false),
              mbNotStop(false),
              mbAcceptKeyFrames(true),
              bInitializing(false)) {
      mnMatchesInliers = 0;

      mTinit = 0.f;

      mNumLM = 0;
      mNumKFCulling = 0;

#ifdef REGISTER_TIMES
      nLBA_exec = 0;
      nLBA_abort = 0;
#endif
    }

    void LocalMapping::SetLoopCloser(LoopClosing *pLoopCloser) {
      mpLoopCloser = pLoopCloser;
    }

    void LocalMapping::SetTracker(Tracking *pTracker) { mpTracker = pTracker; }

    // void LocalMapping::SetDynamicDetector(DynamicDetector* pDynamicDetector)
    // { mpDynamicsDetectors = pDynamicDetector; }

    void LocalMapping::Run() {
      mbFinished = false;

      while (true) {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if (CheckNewKeyFrames()) {
#ifdef REGISTER_TIMES
          double timeLBA_ms = 0;
          double timeKFCulling_ms = 0;

          std::chrono::steady_clock::time_point time_StartProcessKF =
              std::chrono::steady_clock::now();
#endif
          // BoW conversion and insertion in Map
          ProcessNewKeyFrame();
#ifdef REGISTER_TIMES
          std::chrono::steady_clock::time_point time_EndProcessKF =
              std::chrono::steady_clock::now();

          double timeProcessKF = std::chrono::duration_cast<
                                     std::chrono::duration<double, std::milli>>(
                                     time_EndProcessKF - time_StartProcessKF)
                                     .count();
          vdKFInsert_ms.push_back(timeProcessKF);
#endif

          // Check recent MapPoints
          MapPointCulling();
#ifdef REGISTER_TIMES
          std::chrono::steady_clock::time_point time_EndMPCulling =
              std::chrono::steady_clock::now();

          double timeMPCulling = std::chrono::duration_cast<
                                     std::chrono::duration<double, std::milli>>(
                                     time_EndMPCulling - time_EndProcessKF)
                                     .count();
          vdMPCulling_ms.push_back(timeMPCulling);
#endif

          // Triangulate new MapPoints
          CreateNewMapPoints();

          mbAbortBA = false;

          if (!CheckNewKeyFrames()) {
            // Find more matches in neighbor keyframes and fuse point
            // duplications
            SearchInNeighbors();
          }

#ifdef REGISTER_TIMES
          std::chrono::steady_clock::time_point time_EndMPCreation =
              std::chrono::steady_clock::now();

          double timeMPCreation =
              std::chrono::duration_cast<
                  std::chrono::duration<double, std::milli>>(
                  time_EndMPCreation - time_EndMPCulling)
                  .count();
          vdMPCreation_ms.push_back(timeMPCreation);
#endif

          bool b_doneLBA = false;
          int num_FixedKF_BA = 0;
          int num_OptKF_BA = 0;
          int num_MPs_BA = 0;
          int num_edges_BA = 0;

          if (!CheckNewKeyFrames() && !stopRequested()) {

            //        FuseSemanticLabels(mpCurrentKeyFrame, &mbAbortBA);

            if (mpAtlas->KeyFramesInMap() > 2) {

              Optimizer::LocalBundleAdjustment(
                  mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),
                  num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA);
              b_doneLBA = true;
            }
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndLBA =
                std::chrono::steady_clock::now();

            if (b_doneLBA) {
              timeLBA_ms = std::chrono::duration_cast<
                               std::chrono::duration<double, std::milli>>(
                               time_EndLBA - time_EndMPCreation)
                               .count();
              vdLBA_ms.push_back(timeLBA_ms);

              nLBA_exec += 1;
              if (mbAbortBA) {
                nLBA_abort += 1;
              }
              vnLBA_edges.push_back(num_edges_BA);
              vnLBA_KFopt.push_back(num_OptKF_BA);
              vnLBA_KFfixed.push_back(num_FixedKF_BA);
              vnLBA_MPs.push_back(num_MPs_BA);
            }

#endif

            // Check redundant local Keyframes
            KeyFrameCulling();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndKFCulling =
                std::chrono::steady_clock::now();

            timeKFCulling_ms = std::chrono::duration_cast<
                                   std::chrono::duration<double, std::milli>>(
                                   time_EndKFCulling - time_EndLBA)
                                   .count();
            vdKFCulling_ms.push_back(timeKFCulling_ms);
#endif
          }

#ifdef REGISTER_TIMES
          vdLBASync_ms.push_back(timeKFCulling_ms);
          vdKFCullingSync_ms.push_back(timeKFCulling_ms);
#endif

          mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);

          mlProcessdKFs.push_back(mpCurrentKeyFrame);
          //      if (!mpCurrentKeyFrame->gray_.empty()) {
          cv::Mat imgKFDebug = mpCurrentKeyFrame->gray_.clone();
          // cv::imwrite("/home/wfram/Datasets/rgbd_dataset_freiburg3_walking_xyz/TestKF/"
          // +
          //             std::to_string(mpCurrentKeyFrame->mnId) + ".png",
          //             imgKFDebug);

#ifdef REGISTER_TIMES
          std::chrono::steady_clock::time_point time_EndLocalMap =
              std::chrono::steady_clock::now();

          double timeLocalMap = std::chrono::duration_cast<
                                    std::chrono::duration<double, std::milli>>(
                                    time_EndLocalMap - time_StartProcessKF)
                                    .count();
          vdLMTotal_ms.push_back(timeLocalMap);
#endif
        } else if (Stop()) {
          // Safe area to stop
          while (isStopped() && !CheckFinish()) {
            usleep(3000);
          }
          if (CheckFinish())
            break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if (CheckFinish())
          break;

        usleep(3000);
      }

      SetFinish();
    }

    void LocalMapping::InsertKeyFrame(KeyFrame *pKF) {
      unique_lock<mutex> lock(mMutexNewKFs);
      mlNewKeyFrames.push_back(pKF);
      mbAbortBA = true;
    }

    KeyFrame *LocalMapping::GetKeyFrameToPublish() {
      unique_lock<mutex> lock(mMutexNewKFs);
      KeyFrame *pKFpub = mlProcessdKFs.front();
      //    cv::imwrite("/home/wfram/Datasets/rgbd_dataset_freiburg3_walking_xyz/TestKF/"+std::to_string(pKFpub->mnId)+".png",
      //                pKFpub->gray_);
      mlProcessdKFs.pop_front();
      return pKFpub;
    }

    bool LocalMapping::CheckNewKeyFrames() {
      unique_lock<mutex> lock(mMutexNewKFs);
      return (!mlNewKeyFrames.empty());
    }

    void LocalMapping::ProcessNewKeyFrame() {
      {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
      }
      // mpCurrentKeyFrame->selectStaticFeatures();

      // Compute Bags of Words structures
      mpCurrentKeyFrame->ComputeBoW();

      // Associate MapPoints to the new keyframe and update normal and
      // descriptor
      const vector<MapPoint *> vpMapPointMatches =
          mpCurrentKeyFrame->GetMapPointMatches();

      for (size_t i = 0; i < vpMapPointMatches.size(); i++) {
        MapPoint *pMP = vpMapPointMatches[i];
        if (pMP) {
          if (!pMP->isBad()) {
            if (!pMP->IsInKeyFrame(mpCurrentKeyFrame)) {
              pMP->AddObservation(mpCurrentKeyFrame, (int)i);
              pMP->UpdateNormalAndDepth();
              pMP->ComputeDistinctiveDescriptors();
            } else // this can only happen for new stereo points inserted by the
                   // Tracking
            {
              mlpRecentAddedMapPoints.push_back(pMP);
            }
          }
        }
      }

      // Update links in the Covisibility Graph
      mpCurrentKeyFrame->UpdateConnections();

      // Insert Keyframe in Map
      mpAtlas->AddKeyFrame(mpCurrentKeyFrame);
    }

    void LocalMapping::EmptyQueue() {
      while (CheckNewKeyFrames())
        ProcessNewKeyFrame();
    }

    void LocalMapping::MapPointCulling() {
      // Check Recent Added MapPoints
      auto lit = mlpRecentAddedMapPoints.begin();
      const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

      int nThObs;
      if (mbMonocular)
        nThObs = 2;
      else
        nThObs = 3;
      const int cnThObs = nThObs;

      size_t borrar = mlpRecentAddedMapPoints.size();
      while (lit != mlpRecentAddedMapPoints.end()) {
        MapPoint *pMP = *lit;

        if (pMP->isBad())
          lit = mlpRecentAddedMapPoints.erase(lit);
        else if (pMP->GetFoundRatio() < 0.25f) {
          pMP->SetBadFlag();
          lit = mlpRecentAddedMapPoints.erase(lit);
        } else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 2 &&
                   pMP->Observations() <= cnThObs) {
          pMP->SetBadFlag();
          lit = mlpRecentAddedMapPoints.erase(lit);
        } else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 3)
          lit = mlpRecentAddedMapPoints.erase(lit);
        else {
          lit++;
          borrar--;
        }
      }
    }

    void LocalMapping::CreateNewMapPoints() {
      // Retrieve neighbor keyframes in covisibility graph
      int nn = 10;

      if (mbMonocular)
        nn = 30;
      vector<KeyFrame *> vpNeighKFs =
          mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

      float th = 0.6f;

      ORBmatcher matcher(th, false);

      Sophus::SE3<float> sophTcw1 = mpCurrentKeyFrame->GetPose();
      Eigen::Matrix<float, 3, 4> eigTcw1 = sophTcw1.matrix3x4();
      Eigen::Matrix<float, 3, 3> Rcw1 = eigTcw1.block<3, 3>(0, 0);
      Eigen::Matrix<float, 3, 3> Rwc1 = Rcw1.transpose();
      Eigen::Vector3f tcw1 = sophTcw1.translation();
      Eigen::Vector3f Ow1 = mpCurrentKeyFrame->GetCameraCenter();

      const float &fx1 = mpCurrentKeyFrame->fx;
      const float &fy1 = mpCurrentKeyFrame->fy;
      const float &cx1 = mpCurrentKeyFrame->cx;
      const float &cy1 = mpCurrentKeyFrame->cy;
      const float &invfx1 = mpCurrentKeyFrame->invfx;
      const float &invfy1 = mpCurrentKeyFrame->invfy;

      const float ratioFactor = 1.5f * mpCurrentKeyFrame->mfScaleFactor;
      int countStereo = 0;
      int countStereoGoodProj = 0;
      int countStereoAttempt = 0;
      int totalStereoPts = 0;
      // Search matches with epipolar restriction and triangulate

      for (size_t i = 0; i < vpNeighKFs.size(); i++) {
        if (i > 0 && CheckNewKeyFrames())
          return;

        KeyFrame *pKF2 = vpNeighKFs[i];

        GeometricCamera *pCamera1 = mpCurrentKeyFrame->mpCamera,
                        *pCamera2 = pKF2->mpCamera;

        // Check first that baseline is not too short
        Eigen::Vector3f Ow2 = pKF2->GetCameraCenter();
        Eigen::Vector3f vBaseline = Ow2 - Ow1;
        const float baseline = vBaseline.norm();

        if (!mbMonocular) {
          if (baseline < pKF2->mb)
            continue;
        } else {
          const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
          const float ratioBaselineDepth = baseline / medianDepthKF2;

          if (ratioBaselineDepth < 0.01)
            continue;
        }

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t, size_t>> vMatchedIndices;

        matcher.SearchForTriangulation(mpCurrentKeyFrame, pKF2, vMatchedIndices,
                                       false, false);

        Sophus::SE3<float> sophTcw2 = pKF2->GetPose();
        Eigen::Matrix<float, 3, 4> eigTcw2 = sophTcw2.matrix3x4();
        Eigen::Matrix<float, 3, 3> Rcw2 = eigTcw2.block<3, 3>(0, 0);
        Eigen::Matrix<float, 3, 3> Rwc2 = Rcw2.transpose();
        Eigen::Vector3f tcw2 = sophTcw2.translation();

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for (int ikp = 0; ikp < nmatches; ikp++) {
          const int &idx1 = vMatchedIndices[ikp].first;
          const int &idx2 = vMatchedIndices[ikp].second;

          const cv::KeyPoint &kp1 =
              (mpCurrentKeyFrame->NLeft == -1)
                  ? mpCurrentKeyFrame->mvKeysUn[idx1]
              : (idx1 < mpCurrentKeyFrame->NLeft)
                  ? mpCurrentKeyFrame->mvKeys[idx1]
                  : mpCurrentKeyFrame
                        ->mvKeysRight[idx1 - mpCurrentKeyFrame->NLeft];
          const float kp1_ur = mpCurrentKeyFrame->mvuRight[idx1];
          bool bStereo1 = (!mpCurrentKeyFrame->mpCamera2 && kp1_ur >= 0);
          const bool bRight1 = !(mpCurrentKeyFrame->NLeft == -1 ||
                                 idx1 < mpCurrentKeyFrame->NLeft);

          const cv::KeyPoint &kp2 = (pKF2->NLeft == -1) ? pKF2->mvKeysUn[idx2]
                                    : (idx2 < pKF2->NLeft)
                                        ? pKF2->mvKeys[idx2]
                                        : pKF2->mvKeysRight[idx2 - pKF2->NLeft];

          const float kp2_ur = pKF2->mvuRight[idx2];
          bool bStereo2 = (!pKF2->mpCamera2 && kp2_ur >= 0);
          const bool bRight2 = !(pKF2->NLeft == -1 || idx2 < pKF2->NLeft);

          if (mpCurrentKeyFrame->mpCamera2 && pKF2->mpCamera2) {
            if (bRight1 && bRight2) {
              sophTcw1 = mpCurrentKeyFrame->GetRightPose();
              Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

              sophTcw2 = pKF2->GetRightPose();
              Ow2 = pKF2->GetRightCameraCenter();

              pCamera1 = mpCurrentKeyFrame->mpCamera2;
              pCamera2 = pKF2->mpCamera2;
            } else if (bRight1 && !bRight2) {
              sophTcw1 = mpCurrentKeyFrame->GetRightPose();
              Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

              sophTcw2 = pKF2->GetPose();
              Ow2 = pKF2->GetCameraCenter();

              pCamera1 = mpCurrentKeyFrame->mpCamera2;
              pCamera2 = pKF2->mpCamera;
            } else if (!bRight1 && bRight2) {
              sophTcw1 = mpCurrentKeyFrame->GetPose();
              Ow1 = mpCurrentKeyFrame->GetCameraCenter();

              sophTcw2 = pKF2->GetRightPose();
              Ow2 = pKF2->GetRightCameraCenter();

              pCamera1 = mpCurrentKeyFrame->mpCamera;
              pCamera2 = pKF2->mpCamera2;
            } else {
              sophTcw1 = mpCurrentKeyFrame->GetPose();
              Ow1 = mpCurrentKeyFrame->GetCameraCenter();

              sophTcw2 = pKF2->GetPose();
              Ow2 = pKF2->GetCameraCenter();

              pCamera1 = mpCurrentKeyFrame->mpCamera;
              pCamera2 = pKF2->mpCamera;
            }
            eigTcw1 = sophTcw1.matrix3x4();
            Rcw1 = eigTcw1.block<3, 3>(0, 0);
            Rwc1 = Rcw1.transpose();
            tcw1 = sophTcw1.translation();

            eigTcw2 = sophTcw2.matrix3x4();
            Rcw2 = eigTcw2.block<3, 3>(0, 0);
            Rwc2 = Rcw2.transpose();
            tcw2 = sophTcw2.translation();
          }

          // Check parallax between rays
          Eigen::Vector3f xn1 = pCamera1->unprojectEig(kp1.pt);
          Eigen::Vector3f xn2 = pCamera2->unprojectEig(kp2.pt);

          Eigen::Vector3f ray1 = Rwc1 * xn1;
          Eigen::Vector3f ray2 = Rwc2 * xn2;
          const float cosParallaxRays =
              ray1.dot(ray2) / (ray1.norm() * ray2.norm());

          float cosParallaxStereo = cosParallaxRays + 1;
          float cosParallaxStereo1 = cosParallaxStereo;
          float cosParallaxStereo2 = cosParallaxStereo;

          if (bStereo1)
            cosParallaxStereo1 =
                cos(2 * atan2(mpCurrentKeyFrame->mb / 2,
                              mpCurrentKeyFrame->mvDepth[idx1]));
          else if (bStereo2)
            cosParallaxStereo2 =
                cos(2 * atan2(pKF2->mb / 2, pKF2->mvDepth[idx2]));

          if (bStereo1 || bStereo2)
            totalStereoPts++;

          cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

          Eigen::Vector3f x3D;

          bool goodProj = false;
          bool bPointStereo = false;
          if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 &&
              (bStereo1 || bStereo2 || (cosParallaxRays < 0.9998))) {
            goodProj =
                GeometricTools::Triangulate(xn1, xn2, eigTcw1, eigTcw2, x3D);
            if (!goodProj)
              continue;
          } else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2) {
            countStereoAttempt++;
            bPointStereo = true;
            goodProj = mpCurrentKeyFrame->UnprojectStereo(idx1, x3D);
          } else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1) {
            countStereoAttempt++;
            bPointStereo = true;
            goodProj = pKF2->UnprojectStereo(idx2, x3D);
          } else {
            continue; // No stereo and very low parallax
          }

          if (goodProj && bPointStereo)
            countStereoGoodProj++;

          if (!goodProj)
            continue;

          // Check triangulation in front of cameras
          float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
          if (z1 <= 0)
            continue;

          float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
          if (z2 <= 0)
            continue;

          // Check reprojection error in first keyframe
          const float &sigmaSquare1 =
              mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
          const float x1 = Rcw1.row(0).dot(x3D) + tcw1(0);
          const float y1 = Rcw1.row(1).dot(x3D) + tcw1(1);
          const float invz1 = 1.0 / z1;

          if (!bStereo1) {
            cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1, y1, z1));
            float errX1 = uv1.x - kp1.pt.x;
            float errY1 = uv1.y - kp1.pt.y;

            if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1)
              continue;

          } else {
            float u1 = fx1 * x1 * invz1 + cx1;
            float u1_r = u1 - mpCurrentKeyFrame->mbf * invz1;
            float v1 = fy1 * y1 * invz1 + cy1;
            float errX1 = u1 - kp1.pt.x;
            float errY1 = v1 - kp1.pt.y;
            float errX1_r = u1_r - kp1_ur;
            if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) >
                7.8 * sigmaSquare1)
              continue;
          }

          // Check reprojection error in second keyframe
          const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
          const float x2 = Rcw2.row(0).dot(x3D) + tcw2(0);
          const float y2 = Rcw2.row(1).dot(x3D) + tcw2(1);
          const float invz2 = 1.0 / z2;
          if (!bStereo2) {
            cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2, y2, z2));
            float errX2 = uv2.x - kp2.pt.x;
            float errY2 = uv2.y - kp2.pt.y;
            if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2)
              continue;
          } else {
            float u2 = fx2 * x2 * invz2 + cx2;
            float u2_r = u2 - mpCurrentKeyFrame->mbf * invz2;
            float v2 = fy2 * y2 * invz2 + cy2;
            float errX2 = u2 - kp2.pt.x;
            float errY2 = v2 - kp2.pt.y;
            float errX2_r = u2_r - kp2_ur;
            if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) >
                7.8 * sigmaSquare2)
              continue;
          }

          // Check scale consistency
          Eigen::Vector3f normal1 = x3D - Ow1;
          float dist1 = normal1.norm();

          Eigen::Vector3f normal2 = x3D - Ow2;
          float dist2 = normal2.norm();

          if (dist1 == 0 || dist2 == 0)
            continue;

          if (mbFarPoints &&
              (dist1 >= mThFarPoints || dist2 >= mThFarPoints)) // MODIFICATION
            continue;

          const float ratioDist = dist2 / dist1;
          const float ratioOctave =
              mpCurrentKeyFrame->mvScaleFactors[kp1.octave] /
              pKF2->mvScaleFactors[kp2.octave];

          if (ratioDist * ratioFactor < ratioOctave ||
              ratioDist > ratioOctave * ratioFactor)
            continue;

          // Triangulation is succesfull
          auto *pMP =
              new MapPoint(x3D, mpCurrentKeyFrame, mpAtlas->GetCurrentMap());
          if (bPointStereo)
            countStereo++;

          pMP->AddObservation(mpCurrentKeyFrame, idx1);
          pMP->AddObservation(pKF2, idx2);

          mpCurrentKeyFrame->AddMapPoint(pMP, idx1);
          pKF2->AddMapPoint(pMP, idx2);

          pMP->ComputeDistinctiveDescriptors();

          pMP->UpdateNormalAndDepth();

          mpAtlas->AddMapPoint(pMP);
          mlpRecentAddedMapPoints.push_back(pMP);
        }
      }
    }

    void LocalMapping::SearchInNeighbors() {
      // Retrieve neighbor keyframes
      int nn = 10;
      if (mbMonocular)
        nn = 30;
      const vector<KeyFrame *> vpNeighKFs =
          mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
      vector<KeyFrame *> vpTargetKFs;
      for (vector<KeyFrame *>::const_iterator vit = vpNeighKFs.begin(),
                                              vend = vpNeighKFs.end();
           vit != vend; vit++) {
        KeyFrame *pKFi = *vit;
        if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
          continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
      }

      // Add some covisible of covisible
      // Extend to some second neighbors if abort is not requested
      for (int i = 0, imax = vpTargetKFs.size(); i < imax; i++) {
        const vector<KeyFrame *> vpSecondNeighKFs =
            vpTargetKFs[i]->GetBestCovisibilityKeyFrames(20);
        for (vector<KeyFrame *>::const_iterator vit2 = vpSecondNeighKFs.begin(),
                                                vend2 = vpSecondNeighKFs.end();
             vit2 != vend2; vit2++) {
          KeyFrame *pKFi2 = *vit2;
          if (pKFi2->isBad() ||
              pKFi2->mnFuseTargetForKF == mpCurrentKeyFrame->mnId ||
              pKFi2->mnId == mpCurrentKeyFrame->mnId)
            continue;
          vpTargetKFs.push_back(pKFi2);
          pKFi2->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
        }
        if (mbAbortBA)
          break;
      }

      // Search matches by projection from current KF in target KFs
      ORBmatcher matcher;
      vector<MapPoint *> vpMapPointMatches =
          mpCurrentKeyFrame->GetMapPointMatches();
      for (auto pKFi : vpTargetKFs) {
        matcher.Fuse(pKFi, vpMapPointMatches);
        if (pKFi->NLeft != -1)
          matcher.Fuse(pKFi, vpMapPointMatches, true);
      }

      if (mbAbortBA)
        return;

      // Search matches by projection from target KFs in current KF
      vector<MapPoint *> vpFuseCandidates;
      vpFuseCandidates.reserve(vpTargetKFs.size() * vpMapPointMatches.size());

      for (auto pKFi : vpTargetKFs) {
        vector<MapPoint *> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for (auto pMP : vpMapPointsKFi) {
          if (!pMP)
            continue;
          if (pMP->isBad() ||
              pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
            continue;
          pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
          vpFuseCandidates.push_back(pMP);
        }
      }

      matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates);
      if (mpCurrentKeyFrame->NLeft != -1)
        matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates, true);

      // Update points
      vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
      for (auto pMP : vpMapPointMatches) {
        if (pMP) {
          if (!pMP->isBad()) {
            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();
          }
        }
      }

      // Update connections in covisibility graph
      mpCurrentKeyFrame->UpdateConnections();
    }

    void LocalMapping::RequestStop() {
      unique_lock<mutex> lock(mMutexStop);
      mbStopRequested = true;
      unique_lock<mutex> lock2(mMutexNewKFs);
      mbAbortBA = true;
    }

    bool LocalMapping::Stop() {
      unique_lock<mutex> lock(mMutexStop);
      if (mbStopRequested && !mbNotStop) {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
      }

      return false;
    }

    bool LocalMapping::isStopped() {
      unique_lock<mutex> lock(mMutexStop);
      return mbStopped;
    }

    bool LocalMapping::stopRequested() {
      unique_lock<mutex> lock(mMutexStop);
      return mbStopRequested;
    }

    void LocalMapping::Release() {
      unique_lock<mutex> lock(mMutexStop);
      unique_lock<mutex> lock2(mMutexFinish);
      if (mbFinished)
        return;
      mbStopped = false;
      mbStopRequested = false;
      for (auto &mlNewKeyFrame : mlNewKeyFrames)
        delete mlNewKeyFrame;
      mlNewKeyFrames.clear();

      cout << "Local Mapping RELEASE" << endl;
    }

    bool LocalMapping::AcceptKeyFrames() {
      unique_lock<mutex> lock(mMutexAccept);
      return mbAcceptKeyFrames;
    }

    void LocalMapping::SetAcceptKeyFrames(bool flag) {
      unique_lock<mutex> lock(mMutexAccept);
      mbAcceptKeyFrames = flag;
    }

    bool LocalMapping::SetNotStop(bool flag) {
      unique_lock<mutex> lock(mMutexStop);

      if (flag && mbStopped)
        return false;

      mbNotStop = flag;

      return true;
    }

    void LocalMapping::InterruptBA() { mbAbortBA = true; }

    void LocalMapping::KeyFrameCulling() {
      // Check redundant keyframes (only local keyframes)
      // A keyframe is considered redundant if the 90% of the MapPoints it sees,
      // are seen in at least other 3 keyframes (in the same or finer scale) We
      // only consider close stereo points
      const int Nd = 21;
      mpCurrentKeyFrame->UpdateBestCovisibles();
      vector<KeyFrame *> vpLocalKeyFrames =
          mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

      float redundant_th;
      redundant_th = 0.9;

      else if (mbMonocular) redundant_th = 0.9;
      else redundant_th = 0.5;

      int count = 0;

      // Compoute last KF from optimizable window:
      unsigned int last_ID;

      for (auto pKF : vpLocalKeyFrames) {
        count++;
        if ((pKF->mnId == pKF->GetMap()->GetInitKFid()) || pKF->isBad())
          continue;
        const vector<MapPoint *> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs = nObs;
        int nRedundantObservations = 0;
        int nMPs = 0;
        for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++) {
          MapPoint *pMP = vpMapPoints[i];
          if (pMP) {
            if (!pMP->isBad()) {
              if (!mbMonocular) {
                if (pKF->mvDepth[i] > pKF->mThDepth || pKF->mvDepth[i] < 0)
                  continue;
              }

              nMPs++;
              if (pMP->Observations() > thObs) {
                const int &scaleLevel =
                    (pKF->NLeft == -1) ? pKF->mvKeysUn[i].octave
                    : (i < pKF->NLeft) ? pKF->mvKeys[i].octave
                                       : pKF->mvKeysRight[i].octave;
                const map<KeyFrame *, tuple<int, int>> observations =
                    pMP->GetObservations();
                int nObs = 0;
                for (map<KeyFrame *, tuple<int, int>>::const_iterator
                         mit = observations.begin(),
                         mend = observations.end();
                     mit != mend; mit++) {
                  KeyFrame *pKFi = mit->first;
                  if (pKFi == pKF)
                    continue;
                  tuple<int, int> indexes = mit->second;
                  int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
                  int scaleLeveli = -1;
                  if (pKFi->NLeft == -1)
                    scaleLeveli = pKFi->mvKeysUn[leftIndex].octave;
                  else {
                    if (leftIndex != -1) {
                      scaleLeveli = pKFi->mvKeys[leftIndex].octave;
                    }
                    if (rightIndex != -1) {
                      int rightLevel =
                          pKFi->mvKeysRight[rightIndex - pKFi->NLeft].octave;
                      scaleLeveli =
                          (scaleLeveli == -1 || scaleLeveli > rightLevel)
                              ? rightLevel
                              : scaleLeveli;
                    }
                  }

                  if (scaleLeveli <= scaleLevel + 1) {
                    nObs++;
                    if (nObs > thObs)
                      break;
                  }
                }
                if (nObs > thObs) {
                  nRedundantObservations++;
                }
              }
            }
          }
        }

        if (nRedundantObservations > (redundant_th * nMPs)) {

          pKF->SetBadFlag();
        }
        if ((count > 20 && mbAbortBA) || count > 100) {
          break;
        }
      }
    }

    void LocalMapping::RequestReset() {
      {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Map reset recieved" << endl;
        mbResetRequested = true;
      }
      cout << "LM: Map reset, waiting..." << endl;

      while (true) {
        {
          unique_lock<mutex> lock2(mMutexReset);
          if (!mbResetRequested)
            break;
        }
        usleep(3000);
      }
      cout << "LM: Map reset, Done!!!" << endl;
    }

    void LocalMapping::RequestResetActiveMap(Map *pMap) {
      {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Active map reset recieved" << endl;
        mbResetRequestedActiveMap = true;
        mpMapToReset = pMap;
      }
      cout << "LM: Active map reset, waiting..." << endl;

      while (1) {
        {
          unique_lock<mutex> lock2(mMutexReset);
          if (!mbResetRequestedActiveMap)
            break;
        }
        usleep(3000);
      }
      cout << "LM: Active map reset, Done!!!" << endl;
    }

    void LocalMapping::ResetIfRequested() {
      bool executed_reset = false;
      {
        unique_lock<mutex> lock(mMutexReset);
        if (mbResetRequested) {
          executed_reset = true;

          cout << "LM: Reseting Atlas in Local Mapping..." << endl;
          mlNewKeyFrames.clear();
          mlpRecentAddedMapPoints.clear();
          mbResetRequested = false;
          mbResetRequestedActiveMap = false;

          mIdxInit = 0;

          cout << "LM: End reseting Local Mapping..." << endl;
        }

        if (mbResetRequestedActiveMap) {
          executed_reset = true;
          cout << "LM: Reseting current map in Local Mapping..." << endl;
          mlNewKeyFrames.clear();
          mlpRecentAddedMapPoints.clear();

          mbResetRequested = false;
          mbResetRequestedActiveMap = false;
          cout << "LM: End reseting Local Mapping..." << endl;
        }
      }
      if (executed_reset)
        cout << "LM: Reset free the mutex" << endl;
    }

    void LocalMapping::RequestFinish() {
      unique_lock<mutex> lock(mMutexFinish);
      mbFinishRequested = true;
    }

    bool LocalMapping::CheckFinish() {
      unique_lock<mutex> lock(mMutexFinish);
      return mbFinishRequested;
    }

    void LocalMapping::SetFinish() {
      unique_lock<mutex> lock(mMutexFinish);
      mbFinished = true;
      unique_lock<mutex> lock2(mMutexStop);
      mbStopped = true;
    }

    bool LocalMapping::isFinished() {
      unique_lock<mutex> lock(mMutexFinish);
      return mbFinished;
    }

    bool LocalMapping::IsInitializing() const { return bInitializing; }

    double LocalMapping::GetCurrKFTime() {
      if (mpCurrentKeyFrame) {
        return mpCurrentKeyFrame->mTimeStamp;
      } else
        return 0.0;
    }

    KeyFrame *LocalMapping::GetCurrKF() { return mpCurrentKeyFrame; }

    void LocalMapping::FuseSemanticLabels(KeyFrame *pKF, bool *pbStopFlag) {
      // This is a test for fusing the sematic label effect on the probability
      // of movement Criteria of movement right now just if the object is person
      // and has big projection error
      Map *pCurrentMap = pKF->GetMap();
      cv::Mat img = pKF->gray_.clone();
      //  cv::cvtColor(img, img, cv::COLOR_GRAY2RGB);
      vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();
      int index = 0;
      cv::Vec3b PEOPLE_COLOR(61, 5, 150);
      float Px[3][3][3];
      memset(Px, 0, sizeof Px);
      Px[0][1][0] = 1.0f; // P( person, high, was_person )
      Px[1][1][0] = 0.0f; // P( not_person, high, was_person )
      Px[0][1][1] = 0.0f; // P( person, high, was_not_person ) 0.7
      Px[1][1][1] = 1.0f; // P( not_person, high, was_not_person ) 0.3

      Px[0][0][0] = 1.0f; // P( person, low, was_person ) 0.3
      Px[1][0][0] = 0.0f; // P( not_person, low, was_person ) 0.7
      Px[0][0][1] = 0.0f; // P( person, low, was_not_person ) 0.2
      Px[1][0][1] = 1.0f; // P( not_person, low, was_not_person ) 0.8

      float Pz[3][3];
      memset(Pz, 0, sizeof Pz);
      Pz[0][0] = 1.0f; // P( person, person) 0.7
      Pz[1][0] = 0.0f; // P( not_person, person) 0.3
      Pz[0][1] = 0.0f; // P( person, not_person)
      Pz[1][1] = 1.0f; // P( not_person, not_person)

      //        FILE *fp;
      //        std::string label_log_dir =
      //        "/home/wfram/r_viwo_ark_ws/src/R-VIWO-ARK/label_log.txt"; fp =
      //        fopen(label_log_dir.c_str(), "a");

      // bool ut = false, zt = true; // Temporal one, see above
      for (auto pMP : vpMPs) {
        if (pMP) {
          if (!pMP->isBad()) {
            Eigen::Vector3f pos3D = pMP->GetWorldPos();
            const cv::KeyPoint &kpUn = pKF->mvKeysUn[index];

            float u, v;
            cv::Point2f proj;
            pKF->ProjectPointUnDistort(pMP, proj, u, v);

            const float &kpSegVal = pKF->mvSegVal[index];
            float err = cv::norm(proj - kpUn.pt);
            float projectionTh = 5;
            bool ut = (err > projectionTh); // ? true : false;
            bool zt = (kpSegVal != 1.0f);   // not person
            float belPrev_0, belPrev_1;

            //                    fprintf(fp, "%d \n", zt);

            if (kpSegVal == 1.0f) {
              pMP->mProbMov = 1.0f;
            } else {
              pMP->mProbMov = 0.0f;
            }

            //                    if (zt)
            //                        pMP->mProbMov = 0.0f;
            //                    else
            //                        pMP->mProbMov = 1.0f;

            //       if (pMP->mbFirstTime) {
            //         pMP->mProbMov = zt ? 0 : 1.0;
            //         pMP->mbFirstTime = false;
            //       } else {
            //         belPrev_0 = pMP->mProbMov;
            //         belPrev_1 = 1.0f - belPrev_0;
            //
            //         float belCurP_0 =
            //             Px[0][ut][0] * belPrev_0 + Px[0][ut][1] * belPrev_1;
            //             // Person
            //         float belCurP_1 = Px[1][ut][0] * belPrev_0 +
            //                           Px[1][ut][1] * belPrev_1;  // Not a
            //                           person
            //         float belCurC_0 = Pz[zt][0] * belCurP_0;
            //         float belCurC_1 = Pz[zt][1] * belCurP_1;
            //         if (belCurC_0 + belCurC_1 == 0) {
            //           continue;
            //         }
            //         float etta = 1.0f / (belCurC_0 + belCurC_1);
            //
            //         belCurC_0 *= etta;
            //         belCurC_1 *= etta;
            //         pMP->mProbMov = belCurC_0;
            //       }

            // if(!zt) {
            //       if(pMP->mProbMov > 0.45) {
            //       // pMP->SetBadFlag();
            //         cv::line(img, kpUn.pt, proj, cv::Scalar(0,127,255),2);
            //         cv::circle(img, kpUn.pt, 3, cv::Scalar(0,255,0));
            //         cv::circle(img, proj, 2, cv::Scalar(255,0,0));
            //       }
            //       else {
            //         cv::circle(img, kpUn.pt, 6, cv::Scalar(0,0,255));
            //       }
          }
        }
        index++;
      }
      //        fclose(fp);

      //  cv::imshow("projection Error", img);
      //  cv::waitKey(1);
    //   cv::imwrite(
    //       "/home/wfram/Datasets/rgbd_dataset_freiburg3_walking_xyz/Test/" +
    //           std::to_string(pKF->mnId) + ".png",
    //       img);

      return;
    }
    } // namespace RVWO
