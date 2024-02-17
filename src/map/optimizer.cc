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

#include "map/optimizer.h"

#include <Eigen/StdVector>
#include <complex>
#include <mutex>
#include <unsupported/Eigen/MatrixFunctions>

#include "geometry/g2o_types.h"
#include "geometry/optimizable_types.h"
#include "utils/converter.h"

#define HT false

namespace RVWO {
double Optimizer::getAlfa(double prob, double err) {
  if (prob > 1.0f)
    prob = 1.0f;
  //    std::cout << prob << std::endl;
  //        FILE *fp;
  //        string prob_log_dir =
  //        "/home/wfram/r_viwo_ark_ws/src/R-VIWO-ARK/prob_log.txt"; fp =
  //        fopen(prob_log_dir.c_str(), "a"); fprintf(fp, "%0.3f \n", prob);
  //        fclose(fp);
  if (prob <= 0.25) {
    return (-8.0f * prob * prob - 2.0f * prob + 1.0f);
  } else {
    return (-10020.0f);
  }
}

float Optimizer::Px(bool x_cur, bool u_cur, bool x_last) {
  if (x_cur == 0 && u_cur == 1.0f && x_last == 0)
    return 1.00f; // P(person, high, was_person)
  else if (x_cur == 1 && u_cur == 1.0f && x_last == 0)
    return 0.00f; // P(not_person, high, was_person)
  else if (x_cur == 0 && u_cur == 1.0f && x_last == 1)
    return 1.00f; // P(person, high, was_not_person)
  else if (x_cur == 1 && u_cur == 1.0f && x_last == 1)
    return 0.00f; // P(not_person, high, was_not_person)
  else if (x_cur == 0 && u_cur == 0.0f && x_last == 0)
    return 0.00f; // P(person, low, was_person)
  else if (x_cur == 1 && u_cur == 0.0f && x_last == 0)
    return 1.00f; // P(not_person, low, was_person)
  else if (x_cur == 0 && u_cur == 0.0f && x_last == 1)
    return 0.00f; // P(person, low, was_not_person)
  else if (x_cur == 1 && u_cur == 0.0f && x_last == 1)
    return 1.00f; // P(not_person, low, was_not_person)
}

void Optimizer::LabelToProb(const cv::KeyPoint &kpUn, const float &kpSegVal,
                            MapPoint *pMP, Frame *pF) {
  // This is a test for fusing the semantic label effect on the probability of
  // movement Criteria of movement right now just if the object is person and
  // has big projection error
  cv::Vec3b PEOPLE_COLOR(61, 5, 150);
  //        float Px[3][3][3];
  //        memset(Px, 0, sizeof Px);
  //        Px[0][1][0] = 0.95f; // P( person, high, was_person ) should be
  //        quite high Px[1][1][0] = 0.05f; // P( not_person, high, was_person )
  //        should NOT be high Px[0][1][1] = 0.7f; // P( person, high,
  //        was_not_person ) 0.7 should be pretty high, not too much Px[1][1][1]
  //        = 0.3f; // P( not_person, high, was_not_person ) 0.3 should be low
  // TODO: Check 3, 4, 5, 6 - they are of interest

  //        Px[0][0][0] = 0.4f; // P( person, low, was_person ) 0.3 should be
  //        somehow in a middle Px[1][0][0] = 0.6f; // P( not_person, low,
  //        was_person ) 0.7 Px[0][0][1] = 0.05f; // P( person, low,
  //        was_not_person ) 0.2 Px[1][0][1] = 0.95f; // P( not_person, low,
  //        was_not_person ) 0.8 should be quite high

  float Pz[3][3];
  memset(Pz, 0, sizeof Pz);
  // Prediction says that the object is ... What about semantics?
  Pz[0][0] = 1.00f; // P(person, person) 0.7
  Pz[1][0] = 0.50f; // P(not_person, person) 0.3
  Pz[0][1] = 0.50f; // P(person, not_person)
  Pz[1][1] = 1.00f; // P(not_person, not_person)

  //        FILE *fp;
  //        std::string label_log_dir =
  //        "/home/wfram/r_viwo_ark_ws/src/R-VIWO-ARK/label_log.txt"; fp =
  //        fopen(label_log_dir.c_str(), "a");

  float u, v;
  cv::Point2f proj;
  pF->ProjectPointUnDistort(pMP, proj, u, v);
  //        double point_z = pMP->GetWorldPos().z();

  //        bool ut = false, zt = true; // Temporal one, see above
  float err = cv::norm(proj - kpUn.pt);
  float projectionTh = 5.0;
  float err_max = 40.0;
  bool ut = (err > projectionTh);
  bool zt = (kpSegVal != 1.0f); // not person
  float belPrev_0, belPrev_1;

  //        fprintf(fp, "%d \n", zt);

  pMP->mReprErr = err;

  float dist = fabs(err - projectionTh);
  float dist_max = fabs(err_max - projectionTh);
  float Pth = (dist <= dist_max) ? (dist / dist_max) : 1.0f;

  //        if (kpSegVal == PEOPLE_COLOR) {
  //            pMP->mProbMov = 1.0f;
  //        } else {
  //            pMP->mProbMov = 0.0f;
  //        }

  //                    if (zt)
  //                        pMP->mProbMov = 0.0f;
  //                    else
  //                        pMP->mProbMov = 1.0f;

  if (pMP->mbFirstTime) {
    pMP->mProbMov = zt ? 0 : 1.0;
    pMP->mRelInv = pMP->mProbMov;
    pMP->mbFirstTime = false;
    //            pMP->SetWorldPosPrev(pMP->GetWorldPos());
  } else {
    belPrev_0 = pMP->mProbMov;
    belPrev_1 = 1.0f - belPrev_0;

    //            float belCurP_0 =
    //                     Px(0, ut, 0) * belPrev_0 + Px(0, ut, 1) * belPrev_1;
    //                     // Person
    //            float belCurP_1 = Px(1, ut, 0) * belPrev_0 +
    //                              Px(1, ut, 1) * belPrev_1;  // Not a person
    float belCurC_0 =
        Pz[zt][0] * belPrev_0; // TODO: Discrete story again, let's work out
    float belCurC_1 = Pz[zt][1] * belPrev_1;

    //            fprintf(fp,
    //                    "%ld \t %d \t %0.3f \t %0.3f \n",
    //                    pMP->mnId,
    //                    zt,
    //                    pMP->GetWorldPosPrev().x() +
    //                    pMP->GetWorldPosPrev().y() +
    //                    pMP->GetWorldPosPrev().z(), pMP->GetWorldPos().x() +
    //                    pMP->GetWorldPos().y() + pMP->GetWorldPos().z());
    //            pMP->SetWorldPosPrev(pMP->GetWorldPos());

    if (belCurC_0 + belCurC_1 == 0) {
      return;
    }
    float etta = 1.0f / (belCurC_0 + belCurC_1);

    belCurC_0 *= etta;
    belCurC_1 *= etta;

    float weight_1 = 0.5, weight_2 = 0.5;
    pMP->mProbMov = belCurC_0;
    pMP->mRelInv = weight_1 * belCurC_0 + weight_2 * Pth;
    //            pMP->mRelInv = pMP->mProbMov;
  }

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
  //        fclose(fp);
  return;
}

void Optimizer::LabelToProb(const cv::KeyPoint &kpUn, const float &kpSegVal,
                            MapPoint *pMP, KeyFrame *pKF) {
  // This is a test for fusing the sematic label effect on the probability of
  // movement Criteria of movement right now just if the object is person and
  // has big projection error
  cv::Vec3b PEOPLE_COLOR(61, 5, 150);
  //        float Px[3][3][3];
  //        memset(Px, 0, sizeof Px);
  //        Px[0][1][0] = 0.95f; // P( person, high, was_person ) should be
  //        quite high Px[1][1][0] = 0.05f; // P( not_person, high, was_person )
  //        should NOT be high Px[0][1][1] = 0.7f; // P( person, high,
  //        was_not_person ) 0.7 should be pretty high, not too much Px[1][1][1]
  //        = 0.3f; // P( not_person, high, was_not_person ) 0.3 should be low
  // TODO: Check 3, 4, 5, 6 - they are of interest

  //        Px[0][0][0] = 0.4f; // P( person, low, was_person ) 0.3 should be
  //        somehow in a middle Px[1][0][0] = 0.6f; // P( not_person, low,
  //        was_person ) 0.7 Px[0][0][1] = 0.05f; // P( person, low,
  //        was_not_person ) 0.2 Px[1][0][1] = 0.95f; // P( not_person, low,
  //        was_not_person ) 0.8 should be quite high

  float Pz[3][3];
  memset(Pz, 0, sizeof Pz);
  // They should depend on a NN itself
  Pz[0][0] = 1.00f; // P( person, person) 0.7
  Pz[1][0] = 0.50f; // P( not_person, person) 0.3
  Pz[0][1] = 0.50f; // P( person, not_person)
  Pz[1][1] = 1.00f; // P( not_person, not_person)

  FILE *fp;
  std::string label_log_dir =
      "/home/wfram/r_viwo_ark_ws/src/R-VIWO-ARK/label_log.txt";
  fp = fopen(label_log_dir.c_str(), "a");

  float u, v;
  cv::Point2f proj;
  pKF->ProjectPointUnDistort(pMP, proj, u, v);

  //        bool ut = false, zt = true; // Temporal one, see above
  float err = cv::norm(proj - kpUn.pt);
  float projectionTh = 5.0;
  float err_max = 40.0;
  float radii = 50.0f;
  bool ut = (err > projectionTh);
  bool zt = (kpSegVal != 1.0f); // not person
  float belPrev_0, belPrev_1;

  pMP->mReprErr = err;

  fprintf(fp, "%0.3f \n", err);

  float dist = fabs(err - projectionTh);
  float dist_max = fabs(err_max - projectionTh);
  float Pth = (dist <= dist_max) ? (dist / dist_max) : 1.0f;

  if (pMP->mbFirstTime) {
    pMP->mProbMov = zt ? 0 : 1.0;
    pMP->mRelInv = pMP->mProbMov;
    pMP->mbFirstTime = false;
  } else {
    belPrev_0 = pMP->mProbMov;
    belPrev_1 = 1.0f - belPrev_0;

    // TODO: Implement an algorithm for probability of a MapPoint propagation
    // 1) Search in a local region for pMP
    //  1.1) Take a world position
    //  1.2) Search for closest positions
    // 2) Take corresponding 3D points for those neighbours
    // 3) Change the probability for them

    //            float belCurP_0 =
    //                    Px(0, ut, 0) * belPrev_0 + Px(0, ut, 1) * belPrev_1;
    //                    // Person
    //            float belCurP_1 = Px(1, ut, 0) * belPrev_0 +
    //                              Px(1, ut, 1) * belPrev_1;  // Not a person
    float belCurC_0 = Pz[zt][0] * belPrev_0;
    float belCurC_1 = Pz[zt][1] * belPrev_1;

    //            fprintf(fp,
    //                    "%ld \t %d \t %0.3f \t %0.3f \n",
    //                    pMP->mnId,
    //                    zt,
    //                    pMP->GetWorldPosPrev().x() +
    //                    pMP->GetWorldPosPrev().y() +
    //                    pMP->GetWorldPosPrev().z(), pMP->GetWorldPos().x() +
    //                    pMP->GetWorldPos().y() + pMP->GetWorldPos().z());

    if (belCurC_0 + belCurC_1 == 0) {
      return;
    }
    float etta = 1.0f / (belCurC_0 + belCurC_1);

    belCurC_0 *= etta;
    belCurC_1 *= etta;

    float weight_1 = 0.5, weight_2 = 0.5;
    pMP->mProbMov = belCurC_0;
    pMP->mRelInv = weight_1 * belCurC_0 + weight_2 * Pth;
    //            pMP->mRelInv = pMP->mProbMov;
  }

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
  fclose(fp);
  return;
}

void Optimizer::GlobalBundleAdjustemnt(Map *pMap, int nIterations,
                                       bool *pbStopFlag,
                                       const unsigned long nLoopKF,
                                       const bool bRobust) {
  vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
  vector<MapPoint *> vpMP = pMap->GetAllMapPoints();
  BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
}

void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs,
                                 const vector<MapPoint *> &vpMP,
                                 int nIterations, bool *pbStopFlag,
                                 const unsigned long nLoopKF,
                                 const bool bRobust) {
  vector<bool> vbNotIncludedMP;
  vbNotIncludedMP.resize(vpMP.size());

  Map *pMap = vpKFs[0]->GetMap();

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  auto *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  if (pbStopFlag)
    optimizer.setForceStopFlag(pbStopFlag);

  long unsigned int maxKFid = 0;

  const int nExpectedSize = (int)(vpKFs.size() * vpMP.size());

  vector<RVWO::EdgeSE3ProjectXYZ *> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  vector<RVWO::EdgeSE3ProjectXYZToBody *> vpEdgesBody;
  vpEdgesBody.reserve(nExpectedSize);

  vector<KeyFrame *> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  vector<KeyFrame *> vpEdgeKFBody;
  vpEdgeKFBody.reserve(nExpectedSize);

  vector<MapPoint *> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  vector<MapPoint *> vpMapPointEdgeBody;
  vpMapPointEdgeBody.reserve(nExpectedSize);

  vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  vector<KeyFrame *> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<MapPoint *> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  // Set KeyFrame vertices

  for (auto pKF : vpKFs) {
    if (pKF->isBad())
      continue;
    auto *vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKF->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId((int)(pKF->mnId));
    vSE3->setFixed(pKF->mnId == pMap->GetInitKFid());
    optimizer.addVertex(vSE3);
    if (pKF->mnId > maxKFid)
      maxKFid = pKF->mnId;
  }

  // TODO: Write a function to choose alfa parameter instead of delta threshold
  // Since alpha parameter is being chosen for the whole edge we need to find a
  // way of choosing something in a middle for two KFs
  const double thHuber2D = sqrt(5.99);
  const double thHuber3D = sqrt(7.815);

  // Set MapPoint vertices
  for (size_t i = 0; i < vpMP.size(); i++) {
    MapPoint *pMP = vpMP[i];
    if (pMP->isBad())
      continue;
    auto *vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    const int id = (int)(pMP->mnId + maxKFid + 1);
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const map<KeyFrame *, tuple<int, int>> observations =
        pMP->GetObservations();

    int nEdges = 0;
    // SET EDGES
    for (const auto &observation : observations) {
      KeyFrame *pKF = observation.first;
      if (pKF->isBad() || pKF->mnId > maxKFid)
        continue;
      if (optimizer.vertex(id) == nullptr ||
          optimizer.vertex((int)(pKF->mnId)) == nullptr)
        continue;
      nEdges++;

      const int leftIndex = get<0>(observation.second);

      if (leftIndex != -1 && pKF->mvuRight[get<0>(observation.second)] < 0) {
        const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];
        const float &kpSegVal = pKF->mvSegVal[leftIndex];

        Eigen::Matrix<double, 2, 1> obs;
        obs << kpUn.pt.x, kpUn.pt.y;

        auto *e = new RVWO::EdgeSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int)(pKF->mnId))));
        e->setMeasurement(obs);
        const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

        // ARK Bayes
        if (bRobust) {
          //          auto* rk = new g2o::RobustKernelHuber;
          //          e->setRobustKernel(rk);
          //          rk->setDelta(thHuber2D);

#if HT
          auto *ark = new g2o::RobustKernelAdaptive;
          double alphaBA = getAlfa(pMP->mRelInv);
          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
          e->setRobustKernel(ark);
          if (kpSegVal == PEOPLE_COLOR) {
            alphaBA = -10020.0f;
          } else {
            alphaBA = -2.0f;
          }
          ark->setAlpha(alphaBA);
#else
          LabelToProb(kpUn, kpSegVal, pMP, pKF);
          auto *ark = new g2o::RobustKernelAdaptive;
          double alphaBA = getAlfa(pMP->mRelInv, pMP->mReprErr);
          e->setRobustKernel(ark);
          ark->setAlpha(alphaBA);
#endif
          //          switch (mKernel) {
          //            case 0:
          //              auto* rk = new g2o::RobustKernelHuber;
          //              e->setRobustKernel(rk);
          //              rk->setDelta(thHuber2D);
          //              break;
          //            case 2:
          //              auto* ark = new g2o::RobustKernelAdaptive;
          //              double alphaBA;
          //              cv::Vec3b PEOPLE_COLOR(61, 5, 150);
          //              e->setRobustKernel(ark);
          //              if (kpSegVal == PEOPLE_COLOR) {
          //                alphaBA = -10020.0f;
          //              }
          //              else {
          //                alphaBA = 1.0f;
          //              }
          //              ark->setAlpha(alphaBA);
          //          }
        }

        e->pCamera = pKF->mpCamera;

        optimizer.addEdge(e);

        vpEdgesMono.push_back(e);
        vpEdgeKFMono.push_back(pKF);
        vpMapPointEdgeMono.push_back(pMP);
      } else if (leftIndex != -1 &&
                 pKF->mvuRight[leftIndex] >= 0) // Stereo observation
      {
        const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];
        const float &kpSegVal = pKF->mvSegVal[leftIndex];

        Eigen::Matrix<double, 3, 1> obs;
        const float kp_ur = pKF->mvuRight[get<0>(observation.second)];
        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

        auto *e = new g2o::EdgeStereoSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int)(pKF->mnId))));
        e->setMeasurement(obs);
        const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
        e->setInformation(Info);

        // ARK Bayes
        if (bRobust) {
//          auto* rk = new g2o::RobustKernelHuber;
//          e->setRobustKernel(rk);
//          rk->setDelta(thHuber3D);
#if HT
          auto *ark = new g2o::RobustKernelAdaptive;
          double alphaBA = getAlfa(pMP->mRelInv);
          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
          e->setRobustKernel(ark);
          if (kpSegVal == PEOPLE_COLOR) {
            alphaBA = -10020.0f;
          } else {
            alphaBA = -2.0f;
          }
          ark->setAlpha(alphaBA);
#else
          LabelToProb(kpUn, kpSegVal, pMP, pKF);
          auto *ark = new g2o::RobustKernelAdaptive;
          double alphaBA = getAlfa(pMP->mRelInv, pMP->mReprErr);
          e->setRobustKernel(ark);
          ark->setAlpha(alphaBA);
#endif
          //          switch (mKernel) {
          //            case 0:
          //              auto* rk = new g2o::RobustKernelHuber;
          //              e->setRobustKernel(rk);
          //              rk->setDelta(thHuber3D);
          //              break;
          //            case 2:
          //              auto* ark = new g2o::RobustKernelAdaptive;
          //              double alphaBA;
          //              cv::Vec3b PEOPLE_COLOR(61, 5, 150);
          //              e->setRobustKernel(ark);
          //              if (kpSegVal == PEOPLE_COLOR) {
          //                alphaBA = -10020.0f;
          //              }
          //              else {
          //                alphaBA = 1.0f;
          //              }
          //              ark->setAlpha(alphaBA);
          //              break;
          //          }
        }

        e->fx = pKF->fx;
        e->fy = pKF->fy;
        e->cx = pKF->cx;
        e->cy = pKF->cy;
        e->bf = pKF->mbf;

        optimizer.addEdge(e);

        vpEdgesStereo.push_back(e);
        vpEdgeKFStereo.push_back(pKF);
        vpMapPointEdgeStereo.push_back(pMP);
      }

      if (pKF->mpCamera2) {
        int rightIndex = get<1>(observation.second);

        if (rightIndex != -1 && rightIndex < pKF->mvKeysRight.size()) {
          rightIndex -= pKF->NLeft;

          //          const cv::KeyPoint& kpUn = pKF->mvKeysUn[leftIndex];
          //          const cv::Vec3b& kpSegVal = pKF->mvSegVal[leftIndex];

          Eigen::Matrix<double, 2, 1> obs;
          cv::KeyPoint kp = pKF->mvKeysRight[rightIndex];
          float kpSegValRight = pKF->mvSegVal[rightIndex];
          obs << kp.pt.x, kp.pt.y;

          auto *e = new RVWO::EdgeSE3ProjectXYZToBody();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              optimizer.vertex((int)(pKF->mnId))));
          e->setMeasurement(obs);
          const float &invSigma2 = pKF->mvInvLevelSigma2[kp.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          //          auto* rk = new g2o::RobustKernelHuber;
          //          e->setRobustKernel(rk);
          //          rk->setDelta(thHuber2D);

          //          switch (mKernel) {
          //            case 0:
          //              auto* rk = new g2o::RobustKernelHuber;
          //              e->setRobustKernel(rk);
          //              rk->setDelta(thHuber2D);
          //              break;
          //            case 2:
          //              auto* ark = new g2o::RobustKernelAdaptive;
          //              double alphaBA;
          //              cv::Vec3b PEOPLE_COLOR(61, 5, 150);
          //              e->setRobustKernel(ark);
          //              if (kpSegVal == PEOPLE_COLOR) {
          //                alphaBA = -10020.0f;
          //              }
          //              else {
          //                alphaBA = 1.0f;
          //              }
          //              ark->setAlpha(alphaBA);
          //              break;
          //          }

          // ARK Bayes
#if HT
          auto *rk = new g2o::RobustKernelAdaptive;
          double alphaBA = getAlfa(pMP->mRelInv);
          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
          e->setRobustKernel(rk);
          if (kpSegValRight == PEOPLE_COLOR) {
            alphaBA = -10020.0f;
          } else {
            alphaBA = -2.0f;
          }
          rk->setAlpha(alphaBA);
#else
          LabelToProb(kp, kpSegValRight, pMP, pKF);
          auto *rk = new g2o::RobustKernelAdaptive;
          double alphaBA = getAlfa(pMP->mRelInv, pMP->mReprErr);
          e->setRobustKernel(rk);
          rk->setAlpha(alphaBA);
#endif
          Sophus::SE3f Trl = pKF->GetRelativePoseTrl();
          e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(),
                                 Trl.translation().cast<double>());

          e->pCamera = pKF->mpCamera2;

          optimizer.addEdge(e);
          vpEdgesBody.push_back(e);
          vpEdgeKFBody.push_back(pKF);
          vpMapPointEdgeBody.push_back(pMP);
        }
      }
    }

    if (nEdges == 0) {
      optimizer.removeVertex(vPoint);
      vbNotIncludedMP[i] = true;
    } else {
      vbNotIncludedMP[i] = false;
    }
  }

  // Optimize!
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(nIterations);
  Verbose::PrintMess("BA: End of the optimization", Verbose::VERBOSITY_NORMAL);

  // Recover optimized data
  // Keyframes
  for (auto pKF : vpKFs) {
    if (pKF->isBad())
      continue;
    auto *vSE3 = dynamic_cast<g2o::VertexSE3Expmap *>(
        optimizer.vertex((int)(pKF->mnId)));

    g2o::SE3Quat SE3quat = vSE3->estimate();
    if (nLoopKF == pMap->GetOriginKF()->mnId) {
      pKF->SetPose(Sophus::SE3f(SE3quat.rotation().cast<float>(),
                                SE3quat.translation().cast<float>()));
    } else {
      pKF->mTcwGBA =
          Sophus::SE3d(SE3quat.rotation(), SE3quat.translation()).cast<float>();
      pKF->mnBAGlobalForKF = nLoopKF;

      Sophus::SE3f mTwc = pKF->GetPoseInverse();
      Sophus::SE3f mTcGBA_c = pKF->mTcwGBA * mTwc;
      Eigen::Vector3f vector_dist = mTcGBA_c.translation();
      double dist = vector_dist.norm();
      if (dist > 1) {
        int numMonoBadPoints = 0, numMonoOptPoints = 0;
        int numStereoBadPoints = 0, numStereoOptPoints = 0;
        vector<MapPoint *> vpMonoMPsOpt, vpStereoMPsOpt;

        for (size_t i2 = 0, iend = vpEdgesMono.size(); i2 < iend; i2++) {
          RVWO::EdgeSE3ProjectXYZ *e = vpEdgesMono[i2];
          MapPoint *pMP = vpMapPointEdgeMono[i2];
          KeyFrame *pKFedge = vpEdgeKFMono[i2];

          if (pKF != pKFedge) {
            continue;
          }

          if (pMP->isBad())
            continue;

          if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            numMonoBadPoints++;

          } else {
            numMonoOptPoints++;
            vpMonoMPsOpt.push_back(pMP);
          }
        }

        for (size_t i2 = 0, iend = vpEdgesStereo.size(); i2 < iend; i2++) {
          g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i2];
          MapPoint *pMP = vpMapPointEdgeStereo[i2];
          KeyFrame *pKFedge = vpEdgeKFMono[i2];

          if (pKF != pKFedge) {
            continue;
          }

          if (pMP->isBad())
            continue;

          if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            numStereoBadPoints++;
          } else {
            numStereoOptPoints++;
            vpStereoMPsOpt.push_back(pMP);
          }
        }
      }
    }
  }

  // Points
  for (size_t i = 0; i < vpMP.size(); i++) {
    if (vbNotIncludedMP[i])
      continue;

    MapPoint *pMP = vpMP[i];

    if (pMP->isBad())
      continue;
    auto *vPoint = dynamic_cast<g2o::VertexSBAPointXYZ *>(
        optimizer.vertex((int)(pMP->mnId + maxKFid + 1)));

    if (nLoopKF == pMap->GetOriginKF()->mnId) {
      pMP->SetWorldPos(vPoint->estimate().cast<float>());
      pMP->UpdateNormalAndDepth();
    } else {
      pMP->mPosGBA = vPoint->estimate().cast<float>();
      pMP->mnBAGlobalForKF = nLoopKF;
    }
  }
}

int Optimizer::PoseOptimization(Frame *pFrame) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

  auto *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  int nInitialCorrespondences = 0;

  // Set Frame vertex
  auto *vSE3 = new g2o::VertexSE3Expmap();
  Sophus::SE3<float> Tcw = pFrame->GetPose();
  vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                 Tcw.translation().cast<double>()));
  vSE3->setId(0);
  vSE3->setFixed(false);
  optimizer.addVertex(vSE3);

  // Set MapPoint vertices
  const int N = pFrame->N;

  vector<RVWO::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;
  vector<RVWO::EdgeSE3ProjectXYZOnlyPoseToBody *> vpEdgesMono_FHR;
  vector<size_t> vnIndexEdgeMono, vnIndexEdgeRight;
  vpEdgesMono.reserve(N);
  vpEdgesMono_FHR.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeRight.reserve(N);

  vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesStereo.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const double deltaMono = sqrt(5.991);
  const double deltaStereo = sqrt(7.815);
  //  std::cout << "Transverse Map Point in PO" << std::endl;
  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    //    tbb::parallel_for(
    //        tbb::blocked_range<size_t>(0, N), [&](tbb::blocked_range<size_t>
    //        rN) {
    for (int i = 0; i < N; i++) {
      MapPoint *pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        // Conventional SLAM
        if (!pFrame->mpCamera2) {
          // Monocular observation
          if (pFrame->mvuRight[i] < 0) {
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 2, 1> obs;
            const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
            const float &kpSegVal = pFrame->mvSegVal[i];
            obs << kpUn.pt.x, kpUn.pt.y;

            auto *e = new RVWO::EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            //            auto* rk = new g2o::RobustKernelHuber; // HERE
            //            e->setRobustKernel(rk);
            //            rk->setDelta(deltaMono);

            // ARK Bayes
#if HT
            auto *ark = new g2o::RobustKernelAdaptive;
            double alphaBA = getAlfa(pMP->mRelInv);
            cv::Vec3b PEOPLE_COLOR(61, 5, 150);
            e->setRobustKernel(ark);
            if (kpSegVal == PEOPLE_COLOR) {
              alphaBA = -10020.0f;
            } else {
              alphaBA = -2.0f;
            }
            ark->setAlpha(alphaBA);
#else
            LabelToProb(kpUn, kpSegVal, pMP, pFrame);
            auto *ark = new g2o::RobustKernelAdaptive;
            double alphaBA = getAlfa(pMP->mRelInv, pMP->mReprErr);
            e->setRobustKernel(ark);
            ark->setAlpha(alphaBA);
#endif
            e->pCamera = pFrame->mpCamera;
            e->Xw = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
          } else // Stereo observation
          {
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 3, 1> obs;
            const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
            const float &kpSegVal = pFrame->mvSegVal[i];
            const float &kp_ur = pFrame->mvuRight[i];
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

            auto *e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            //            auto* rk = new g2o::RobustKernelHuber;
            //            e->setRobustKernel(rk);
            //            rk->setDelta(deltaStereo);

            // ARK Bayes
#if HT
            auto *ark = new g2o::RobustKernelAdaptive;
            double alphaBA = getAlfa(pMP->mRelInv);
            cv::Vec3b PEOPLE_COLOR(61, 5, 150);
            e->setRobustKernel(ark);
            if (kpSegVal == PEOPLE_COLOR) {
              alphaBA = -10020.0f;
            } else {
              alphaBA = -2.0f;
            }
            ark->setAlpha(alphaBA);
#else
            LabelToProb(kpUn, kpSegVal, pMP, pFrame);
            auto *ark = new g2o::RobustKernelAdaptive;
            double alphaBA = getAlfa(pMP->mRelInv, pMP->mReprErr);
            e->setRobustKernel(ark);
            ark->setAlpha(alphaBA);
#endif
            e->fx = RVWO::Frame::fx;
            e->fy = RVWO::Frame::fy;
            e->cx = RVWO::Frame::cx;
            e->cy = RVWO::Frame::cy;
            e->bf = pFrame->mbf;
            e->Xw = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesStereo.push_back(e);
            vnIndexEdgeStereo.push_back(i);
          }
        }
        // SLAM with respect a rigid body
        else {
          nInitialCorrespondences++;

          cv::KeyPoint kpUn;
          float kpSegVal;

          if (i < pFrame->Nleft) { // Left camera observation
            kpUn = pFrame->mvKeys[i];
            kpSegVal = pFrame->mvSegVal[i];

            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            auto *e = new RVWO::EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            //            auto* rk = new g2o::RobustKernelHuber;
            //            e->setRobustKernel(rk);
            //            rk->setDelta(deltaMono);

            // ARK Bayes
#if HT
            auto *ark = new g2o::RobustKernelAdaptive;
            double alphaBA = getAlfa(pMP->mRelInv);
            cv::Vec3b PEOPLE_COLOR(61, 5, 150);
            e->setRobustKernel(ark);
            if (kpSegVal == PEOPLE_COLOR) {
              alphaBA = -10020.0f;
            } else {
              alphaBA = -2.0f;
            }
            ark->setAlpha(alphaBA);
#else
            LabelToProb(kpUn, kpSegVal, pMP, pFrame);
            auto *ark = new g2o::RobustKernelAdaptive;
            double alphaBA = getAlfa(pMP->mRelInv, pMP->mReprErr);
            e->setRobustKernel(ark);
            ark->setAlpha(alphaBA);
#endif
            e->pCamera = pFrame->mpCamera;
            e->Xw = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
          } else {
            kpUn = pFrame->mvKeysRight[i - pFrame->Nleft];
            kpSegVal = pFrame->mvSegVal[i - pFrame->Nleft];

            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            pFrame->mvbOutlier[i] = false;

            auto *e = new RVWO::EdgeSE3ProjectXYZOnlyPoseToBody();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            //            auto* rk = new g2o::RobustKernelHuber;
            //            e->setRobustKernel(rk);
            //            rk->setDelta(deltaMono);

            // ARK Bayes
#if HT
            auto *ark = new g2o::RobustKernelAdaptive;
            double alphaBA = getAlfa(pMP->mRelInv);
            cv::Vec3b PEOPLE_COLOR(61, 5, 150);
            e->setRobustKernel(ark);
            if (kpSegVal == PEOPLE_COLOR) {
              alphaBA = -10020.0f;
            } else {
              alphaBA = -2.0f;
            }
            ark->setAlpha(alphaBA);
#else
            LabelToProb(kpUn, kpSegVal, pMP, pFrame);
            auto *ark = new g2o::RobustKernelAdaptive;
            double alphaBA = getAlfa(pMP->mRelInv, pMP->mReprErr);
            e->setRobustKernel(ark);
            ark->setAlpha(alphaBA);
#endif
            e->pCamera = pFrame->mpCamera2;
            e->Xw = pMP->GetWorldPos().cast<double>();

            e->mTrl = g2o::SE3Quat(
                pFrame->GetRelativePoseTrl().unit_quaternion().cast<double>(),
                pFrame->GetRelativePoseTrl().translation().cast<double>());

            optimizer.addEdge(e);

            vpEdgesMono_FHR.push_back(e);
            vnIndexEdgeRight.push_back(i);
          }
        }
      }
    }
    //        });
  }

  if (nInitialCorrespondences < 3)
    return 0;

  // We perform 4 optimizations, after each optimization we classify observation
  // as inlier/outlier At the next optimization, outliers are not included, but
  // at the end they can be classified as inliers again.
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  for (size_t it = 0; it < 4; it++) {
    Tcw = pFrame->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));

    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    // Mono
    //    std::cout << "Transverse Mono edges in PO" << std::endl;
    //    tbb::parallel_for(
    //        tbb::blocked_range<size_t>(0, vpEdgesMono.size()),
    //        [&](tbb::blocked_range<size_t> rvpEdgesMono) {
    //    for (size_t i = rvpEdgesMono.begin(), iend = rvpEdgesMono.end(); i <
    //    iend;
    //         i++) {
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      //      std::cout << "Mono edge in PO" << std::endl;
      RVWO::EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const double chi2 = e->chi2();

      if (chi2 > chi2Mono[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
      }

      if (it == 2)
        e->setRobustKernel(nullptr);
    }
    //        });
    // Right Fisheye
    //    tbb::parallel_for(tbb::blocked_range<size_t>(0,
    //    vpEdgesMono_FHR.size()),
    //                      [&](tbb::blocked_range<size_t> rvpEdgesMono_FHR) {
    //    for (size_t i = rvpEdgesMono_FHR.begin(), iend =
    //    rvpEdgesMono_FHR.end();
    //         i < iend;
    //         i++) {
    for (size_t i = 0, iend = vpEdgesMono_FHR.size(); i < iend; i++) {
      //      std::cout << "Right fisheyse in PO" << std::endl;
      RVWO::EdgeSE3ProjectXYZOnlyPoseToBody *e = vpEdgesMono_FHR[i];

      const size_t idx = vnIndexEdgeRight[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const double chi2 = e->chi2();

      if (chi2 > chi2Mono[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
      }

      if (it == 2)
        e->setRobustKernel(nullptr);
    }
    //                      });
    // Stereo
    //    std::cout << "Transverse Stereo edges in PO" << std::endl;
    //    tbb::parallel_for(
    //        tbb::blocked_range<size_t>(0, vpEdgesStereo.size()),
    //        [&](tbb::blocked_range<size_t> rvpEdgesStereo) {
    //          for (size_t i = rvpEdgesStereo.begin(), iend =
    //          rvpEdgesStereo.end();
    //               i < iend;
    //               i++) {
    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      //      std::cout << "Stereo edge for PO" << std::endl;
      g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const double chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        e->setLevel(0);
        pFrame->mvbOutlier[idx] = false;
      }

      if (it == 2)
        e->setRobustKernel(nullptr);
    }
    //        });

    if (optimizer.edges().size() < 10)
      break;
  }

  // Recover optimized pose and return number of inliers
  auto *vSE3_recov = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
  g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
  Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
                          SE3quat_recov.translation().cast<float>());
  pFrame->SetPose(pose);

  return nInitialCorrespondences - nBad;
}

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag,
                                      Map *pMap, int &num_fixedKF,
                                      int &num_OptKF, int &num_MPs,
                                      int &num_edges) {
  // Local KeyFrames: First Breath Search from Current Keyframe
  list<KeyFrame *> lLocalKeyFrames;

  lLocalKeyFrames.push_back(pKF);
  pKF->mnBALocalForKF = pKF->mnId;
  Map *pCurrentMap = pKF->GetMap();

  const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
  for (auto pKFi : vNeighKFs) {
    pKFi->mnBALocalForKF = pKF->mnId;
    if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
      lLocalKeyFrames.push_back(pKFi);
  }

  // Local MapPoints seen in Local KeyFrames
  num_fixedKF = 0;
  list<MapPoint *> lLocalMapPoints;
  set<MapPoint *> sNumObsMP;
  for (auto pKFi : lLocalKeyFrames) {
    if (pKFi->mnId == pMap->GetInitKFid()) {
      num_fixedKF = 1;
    }
    vector<MapPoint *> vpMPs = pKFi->GetMapPointMatches();
    for (auto pMP : vpMPs) {
      if (pMP)
        if (!pMP->isBad() && pMP->GetMap() == pCurrentMap) {
          if (pMP->mnBALocalForKF != pKF->mnId) {
            lLocalMapPoints.push_back(pMP);
            pMP->mnBALocalForKF = pKF->mnId;
          }
        }
    }
  }

  // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local
  // Keyframes
  list<KeyFrame *> lFixedCameras;
  for (auto &lLocalMapPoint : lLocalMapPoints) {
    map<KeyFrame *, tuple<int, int>> observations =
        lLocalMapPoint->GetObservations();
    for (auto &observation : observations) {
      KeyFrame *pKFi = observation.first;

      if (pKFi->mnBALocalForKF != pKF->mnId &&
          pKFi->mnBAFixedForKF != pKF->mnId) {
        pKFi->mnBAFixedForKF = pKF->mnId;
        if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
          lFixedCameras.push_back(pKFi);
      }
    }
  }
  num_fixedKF = (int)(lFixedCameras.size() + num_fixedKF);

  if (num_fixedKF == 0) {
    Verbose::PrintMess(
        "LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted",
        Verbose::VERBOSITY_NORMAL);
    return;
  }

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  auto *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  if (pbStopFlag)
    optimizer.setForceStopFlag(pbStopFlag);

  unsigned long maxKFid = 0;

  // DEBUG LBA
  pCurrentMap->msOptKFs.clear();
  pCurrentMap->msFixedKFs.clear();

  // Set Local KeyFrame vertices
  for (auto pKFi : lLocalKeyFrames) {
    auto *vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId((int)(pKFi->mnId));
    vSE3->setFixed(pKFi->mnId == pMap->GetInitKFid());
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid)
      maxKFid = pKFi->mnId;
    // DEBUG LBA
    pCurrentMap->msOptKFs.insert(pKFi->mnId);
  }
  num_OptKF = (int)(lLocalKeyFrames.size());

  // Set Fixed KeyFrame vertices
  for (auto pKFi : lFixedCameras) {
    auto *vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId((int)(pKFi->mnId));
    vSE3->setFixed(true);
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid)
      maxKFid = pKFi->mnId;
    // DEBUG LBA
    pCurrentMap->msFixedKFs.insert(pKFi->mnId);
  }

  // Set MapPoint vertices
  const size_t nExpectedSize =
      (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

  vector<RVWO::EdgeSE3ProjectXYZ *> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  vector<RVWO::EdgeSE3ProjectXYZToBody *> vpEdgesBody;
  vpEdgesBody.reserve(nExpectedSize);

  vector<KeyFrame *> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  vector<KeyFrame *> vpEdgeKFBody;
  vpEdgeKFBody.reserve(nExpectedSize);

  vector<MapPoint *> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  vector<MapPoint *> vpMapPointEdgeBody;
  vpMapPointEdgeBody.reserve(nExpectedSize);

  vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  vector<KeyFrame *> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<MapPoint *> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const double thHuberMono = sqrt(5.991);
  const double thHuberStereo = sqrt(7.815);

  int nPoints = 0;

  int nEdges = 0;

  for (auto pMP : lLocalMapPoints) {
    auto *vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    int id = (int)(pMP->mnId + maxKFid + 1);

    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);
    nPoints++;

    const map<KeyFrame *, tuple<int, int>> observations =
        pMP->GetObservations();

    // Set edges
    for (const auto &observation : observations) {
      KeyFrame *pKFi = observation.first;

      if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
        const int leftIndex = get<0>(observation.second);

        // Monocular observation
        if (leftIndex != -1 && pKFi->mvuRight[get<0>(observation.second)] < 0) {
          const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
          const float &kpSegVal = pKFi->mvSegVal[leftIndex];
          // Do values with the same index correspond to each other?
          // const cv::Vec3b& kpSegVal = pKFi->mvSegVal[leftIndex];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          auto *e = new RVWO::EdgeSE3ProjectXYZ();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              optimizer.vertex((int)(pKFi->mnId))));
          e->setMeasurement(obs);
          const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          //          auto* rk = new g2o::RobustKernelHuber;
          //          e->setRobustKernel(rk);
          //          rk->setDelta(thHuberMono);

          // ARK Bayes
#if HT
          auto *ark = new g2o::RobustKernelAdaptive;
          double alphaBA = getAlfa(pMP->mRelInv);
          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
          e->setRobustKernel(ark);
          if (kpSegVal == PEOPLE_COLOR) {
            // Let's make a function for calculating the PDF
            alphaBA = -10020.0f;
          } else {
            alphaBA = -2.0f;
          }
          ark->setAlpha(alphaBA);
#else
          LabelToProb(kpUn, kpSegVal, pMP, pKFi);
          auto *ark = new g2o::RobustKernelAdaptive;
          double alphaBA = getAlfa(pMP->mRelInv, pMP->mReprErr);
          e->setRobustKernel(ark);
          ark->setAlpha(alphaBA);
#endif
          e->pCamera = pKFi->mpCamera;

          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(pKFi);
          vpMapPointEdgeMono.push_back(pMP);

          nEdges++;
        } else if (leftIndex != -1 &&
                   pKFi->mvuRight[get<0>(observation.second)] >=
                       0) // Stereo observation
        {
          const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex]; // This one?
          const float &kpSegVal = pKFi->mvSegVal[leftIndex];
          Eigen::Matrix<double, 3, 1> obs;
          const float kp_ur = pKFi->mvuRight[get<0>(
              observation.second)]; // How to find the left point?
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          auto *e = new g2o::EdgeStereoSE3ProjectXYZ();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              optimizer.vertex((int)(pKFi->mnId))));
          e->setMeasurement(obs);
          const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
          e->setInformation(Info);

          //          auto* rk = new g2o::RobustKernelHuber;
          //          e->setRobustKernel(rk);
          //          rk->setDelta(thHuberStereo);

          // ARK Bayes
#if HT
          auto *rk = new g2o::RobustKernelAdaptive;
          double alphaBA = getAlfa(pMP->mRelInv);
          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
          e->setRobustKernel(rk);
          if (kpSegVal == PEOPLE_COLOR) {
            alphaBA = -10020.0f;
          } else {
            alphaBA = -2.0f;
          }
          rk->setAlpha(alphaBA);
#else
          LabelToProb(kpUn, kpSegVal, pMP, pKFi);
          auto *rk = new g2o::RobustKernelAdaptive;
          double alphaBA = getAlfa(pMP->mRelInv, pMP->mReprErr);
          e->setRobustKernel(rk);
          rk->setAlpha(alphaBA);
#endif
          e->fx = pKFi->fx;
          e->fy = pKFi->fy;
          e->cx = pKFi->cx;
          e->cy = pKFi->cy;
          e->bf = pKFi->mbf;

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(pKFi);
          vpMapPointEdgeStereo.push_back(pMP);

          nEdges++;
        }

        if (pKFi->mpCamera2) {
          int rightIndex = get<1>(observation.second);

          //          const cv::KeyPoint& kpUn = pKFi->mvKeysUn[leftIndex]; //
          //          This one? const cv::Vec3b& kpSegVal =
          //          pKFi->mvSegVal[leftIndex];

          if (rightIndex != -1) {
            rightIndex -= pKFi->NLeft;

            Eigen::Matrix<double, 2, 1> obs;
            cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
            float kpSegValRight = pKFi->mvSegVal[rightIndex];
            obs << kp.pt.x, kp.pt.y;

            auto *e = new RVWO::EdgeSE3ProjectXYZToBody();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int)(pKFi->mnId))));
            e->setMeasurement(obs);
            const float &invSigma2 = pKFi->mvInvLevelSigma2[kp.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            //            auto* rk = new g2o::RobustKernelHuber;
            //            e->setRobustKernel(rk);
            //            rk->setDelta(thHuberMono);

            // ARK Bayes
#if HT
            auto *rk = new g2o::RobustKernelAdaptive;
            double alphaBA = getAlfa(pMP->mRelInv);
            cv::Vec3b PEOPLE_COLOR(61, 5, 150);
            e->setRobustKernel(rk);
            if (kpSegValRight == PEOPLE_COLOR) {
              alphaBA = -10020.0f;
            } else {
              alphaBA = -2.0f;
            }
            rk->setAlpha(alphaBA);
#else
            LabelToProb(kp, kpSegValRight, pMP, pKFi);
            auto *rk = new g2o::RobustKernelAdaptive;
            double alphaBA = getAlfa(pMP->mRelInv, pMP->mReprErr);
            e->setRobustKernel(rk);
            rk->setAlpha(alphaBA);
#endif
            Sophus::SE3f Trl = pKFi->GetRelativePoseTrl();
            e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(),
                                   Trl.translation().cast<double>());

            e->pCamera = pKFi->mpCamera2;

            optimizer.addEdge(e);
            vpEdgesBody.push_back(e);
            vpEdgeKFBody.push_back(pKFi);
            vpMapPointEdgeBody.push_back(pMP);

            nEdges++;
          }
        }
      }
    }
  }
  num_edges = nEdges;

  if (pbStopFlag)
    if (*pbStopFlag)
      return;

  optimizer.initializeOptimization();
  optimizer.optimize(10);

  vector<pair<KeyFrame *, MapPoint *>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesBody.size() +
                   vpEdgesStereo.size());

  // Check inlier observations
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    RVWO::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
    MapPoint *pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad())
      continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      KeyFrame *pKFi = vpEdgeKFMono[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++) {
    RVWO::EdgeSE3ProjectXYZToBody *e = vpEdgesBody[i];
    MapPoint *pMP = vpMapPointEdgeBody[i];

    if (pMP->isBad())
      continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      KeyFrame *pKFi = vpEdgeKFBody[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
    MapPoint *pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad())
      continue;

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      KeyFrame *pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // Get Map Mutex
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  if (!vToErase.empty()) {
    for (auto &i : vToErase) {
      KeyFrame *pKFi = i.first;
      MapPoint *pMPi = i.second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  // Recover optimized data
  // Keyframes
  for (auto pKFi : lLocalKeyFrames) {
    auto *vSE3 = dynamic_cast<g2o::VertexSE3Expmap *>(
        optimizer.vertex((int)(pKFi->mnId)));
    g2o::SE3Quat SE3quat = vSE3->estimate();
    Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                     SE3quat.translation().cast<float>());
    pKFi->SetPose(Tiw);
  }

  // Points
  for (auto pMP : lLocalMapPoints) {
    auto *vPoint = dynamic_cast<g2o::VertexSBAPointXYZ *>(
        optimizer.vertex((int)(pMP->mnId + maxKFid + 1)));
    pMP->SetWorldPos(vPoint->estimate().cast<float>());
    pMP->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();
}

void Optimizer::OptimizeEssentialGraph(
    Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
    const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
    const LoopClosing::KeyFrameAndPose &CorrectedSim3,
    const map<KeyFrame *, set<KeyFrame *>> &LoopConnections,
    const bool &bFixScale) {
  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::BlockSolver_7_3::LinearSolverType *linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
  auto *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
  auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  solver->setUserLambdaInit(1e-16);
  optimizer.setAlgorithm(solver);

  const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
  const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
      nMaxKFid + 1);
  vector<g2o::VertexSim3Expmap *> vpVertices(nMaxKFid + 1);

  vector<Eigen::Vector3d> vZvectors(nMaxKFid + 1); // For debugging
  Eigen::Vector3d z_vec;
  z_vec << 0.0, 0.0, 1.0;

  const int minFeat = 100;

  // Set KeyFrame vertices
  for (auto pKF : vpKFs) {
    if (pKF->isBad())
      continue;
    auto *VSim3 = new g2o::VertexSim3Expmap();

    const size_t nIDi = pKF->mnId;

    auto it = CorrectedSim3.find(pKF);

    if (it != CorrectedSim3.end()) {
      vScw[nIDi] = it->second;
      VSim3->setEstimate(it->second);
    } else {
      Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
      g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);
      vScw[nIDi] = Siw;
      VSim3->setEstimate(Siw);
    }

    if (pKF->mnId == pMap->GetInitKFid())
      VSim3->setFixed(true);

    VSim3->setId((int)(nIDi));
    VSim3->setMarginalized(false);
    VSim3->_fix_scale = bFixScale;

    optimizer.addVertex(VSim3);
    vZvectors[nIDi] = vScw[nIDi].rotation() * z_vec; // For debugging

    vpVertices[nIDi] = VSim3;
  }

  set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

  const Eigen::Matrix<double, 7, 7> matLambda =
      Eigen::Matrix<double, 7, 7>::Identity();

  // Set Loop edges
  int count_loop = 0;
  for (const auto &LoopConnection : LoopConnections) {
    KeyFrame *pKF = LoopConnection.first;
    const long unsigned int nIDi = pKF->mnId;
    const set<KeyFrame *> &spConnections = LoopConnection.second;
    const g2o::Sim3 Siw = vScw[nIDi];
    const g2o::Sim3 Swi = Siw.inverse();

    for (auto spConnection : spConnections) {
      const long unsigned int nIDj = spConnection->mnId;
      if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
          pKF->GetWeight(spConnection) < minFeat)
        continue;

      const g2o::Sim3 Sjw = vScw[nIDj];
      const g2o::Sim3 Sji = Sjw * Swi;

      auto *e = new g2o::EdgeSim3();
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex((int)nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex((int)nIDi)));
      e->setMeasurement(Sji);

      e->information() = matLambda;

      optimizer.addEdge(e);
      count_loop++;
      sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
    }
  }

  // Set normal edges
  for (auto pKF : vpKFs) {
    const size_t nIDi = pKF->mnId;

    g2o::Sim3 Swi;

    auto iti = NonCorrectedSim3.find(pKF);

    if (iti != NonCorrectedSim3.end())
      Swi = (iti->second).inverse();
    else
      Swi = vScw[nIDi].inverse();

    KeyFrame *pParentKF = pKF->GetParent();

    // Spanning tree edge
    if (pParentKF) {
      size_t nIDj = pParentKF->mnId;

      g2o::Sim3 Sjw;

      auto itj = NonCorrectedSim3.find(pParentKF);

      if (itj != NonCorrectedSim3.end())
        Sjw = itj->second;
      else
        Sjw = vScw[nIDj];

      g2o::Sim3 Sji = Sjw * Swi;

      auto *e = new g2o::EdgeSim3();
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex((int)nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex((int)nIDi)));
      e->setMeasurement(Sji);
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // Loop edges
    const set<KeyFrame *> sLoopEdges = pKF->GetLoopEdges();
    for (auto pLKF : sLoopEdges) {
      if (pLKF->mnId < pKF->mnId) {
        g2o::Sim3 Slw;

        auto itl = NonCorrectedSim3.find(pLKF);

        if (itl != NonCorrectedSim3.end())
          Slw = itl->second;
        else
          Slw = vScw[pLKF->mnId];

        g2o::Sim3 Sli = Slw * Swi;
        auto *el = new g2o::EdgeSim3();
        el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex((int)pLKF->mnId)));
        el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex((int)nIDi)));
        el->setMeasurement(Sli);
        el->information() = matLambda;
        optimizer.addEdge(el);
      }
    }

    // Covisibility graph edges
    const vector<KeyFrame *> vpConnectedKFs =
        pKF->GetCovisiblesByWeight(minFeat);
    for (auto pKFn : vpConnectedKFs) {
      if (pKFn && pKFn != pParentKF &&
          !pKF->hasChild(pKFn) /*&& !sLoopEdges.count(pKFn)*/) {
        if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
          if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId),
                                             max(pKF->mnId, pKFn->mnId))))
            continue;

          g2o::Sim3 Snw;

          auto itn = NonCorrectedSim3.find(pKFn);

          if (itn != NonCorrectedSim3.end())
            Snw = itn->second;
          else
            Snw = vScw[pKFn->mnId];

          g2o::Sim3 Sni = Snw * Swi;

          auto *en = new g2o::EdgeSim3();
          en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                               optimizer.vertex((int)pKFn->mnId)));
          en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                               optimizer.vertex((int)nIDi)));
          en->setMeasurement(Sni);
          en->information() = matLambda;
          optimizer.addEdge(en);
        }
      }
    }
  }

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  optimizer.optimize(20);
  optimizer.computeActiveErrors();
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  for (auto pKFi : vpKFs) {
    const size_t nIDi = pKFi->mnId;

    auto *VSim3 =
        dynamic_cast<g2o::VertexSim3Expmap *>(optimizer.vertex((int)nIDi));
    g2o::Sim3 CorrectedSiw = VSim3->estimate();
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
    double s = CorrectedSiw.scale();

    Sophus::SE3f Tiw(CorrectedSiw.rotation().cast<float>(),
                     CorrectedSiw.translation().cast<float>() / s);
    pKFi->SetPose(Tiw);
  }

  // Correct points. Transform to "non-optimized" reference keyframe pose and
  // transform back with optimized pose
  for (auto pMP : vpMPs) {
    if (pMP->isBad())
      continue;

    size_t nIDr;
    if (pMP->mnCorrectedByKF == pCurKF->mnId) {
      nIDr = pMP->mnCorrectedReference;
    } else {
      KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
      nIDr = pRefKF->mnId;
    }

    g2o::Sim3 Srw = vScw[nIDr];
    g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

    Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->GetWorldPos().cast<double>();
    Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
        correctedSwr.map(Srw.map(eigP3Dw));
    pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

    pMP->UpdateNormalAndDepth();
  }

  // TODO Check this changeindex
  pMap->IncreaseChangeIndex();
}

void Optimizer::OptimizeEssentialGraph(KeyFrame *pCurKF,
                                       vector<KeyFrame *> &vpFixedKFs,
                                       vector<KeyFrame *> &vpFixedCorrectedKFs,
                                       vector<KeyFrame *> &vpNonFixedKFs,
                                       vector<MapPoint *> &vpNonCorrectedMPs) {
  Verbose::PrintMess("Opt_Essential: There are " +
                         to_string(vpFixedKFs.size()) +
                         " KFs fixed in the merged map",
                     Verbose::VERBOSITY_DEBUG);
  Verbose::PrintMess("Opt_Essential: There are " +
                         to_string(vpFixedCorrectedKFs.size()) +
                         " KFs fixed in the old map",
                     Verbose::VERBOSITY_DEBUG);
  Verbose::PrintMess("Opt_Essential: There are " +
                         to_string(vpNonFixedKFs.size()) +
                         " KFs non-fixed in the merged map",
                     Verbose::VERBOSITY_DEBUG);
  Verbose::PrintMess("Opt_Essential: There are " +
                         to_string(vpNonCorrectedMPs.size()) +
                         " MPs non-corrected in the merged map",
                     Verbose::VERBOSITY_DEBUG);

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::BlockSolver_7_3::LinearSolverType *linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
  auto *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
  auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  solver->setUserLambdaInit(1e-16);
  optimizer.setAlgorithm(solver);

  Map *pMap = pCurKF->GetMap();
  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
      nMaxKFid + 1);
  vector<g2o::VertexSim3Expmap *> vpVertices(nMaxKFid + 1);

  vector<bool> vpGoodPose(nMaxKFid + 1);
  vector<bool> vpBadPose(nMaxKFid + 1);

  const int minFeat = 100;

  for (KeyFrame *pKFi : vpFixedKFs) {
    if (pKFi->isBad())
      continue;

    auto *VSim3 = new g2o::VertexSim3Expmap();

    const size_t nIDi = pKFi->mnId;

    Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
    g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    vCorrectedSwc[nIDi] = Siw.inverse();
    VSim3->setEstimate(Siw);

    VSim3->setFixed(true);

    VSim3->setId((int)nIDi);
    VSim3->setMarginalized(false);
    VSim3->_fix_scale = true;

    optimizer.addVertex(VSim3);

    vpVertices[nIDi] = VSim3;

    vpGoodPose[nIDi] = true;
    vpBadPose[nIDi] = false;
  }
  Verbose::PrintMess("Opt_Essential: vpFixedKFs loaded",
                     Verbose::VERBOSITY_DEBUG);

  set<unsigned long> sIdKF;
  for (KeyFrame *pKFi : vpFixedCorrectedKFs) {
    if (pKFi->isBad())
      continue;

    auto *VSim3 = new g2o::VertexSim3Expmap();

    const size_t nIDi = pKFi->mnId;

    Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
    g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    vCorrectedSwc[nIDi] = Siw.inverse();
    VSim3->setEstimate(Siw);

    Sophus::SE3d Tcw_bef = pKFi->mTcwBefMerge.cast<double>();
    vScw[nIDi] =
        g2o::Sim3(Tcw_bef.unit_quaternion(), Tcw_bef.translation(), 1.0);

    VSim3->setFixed(true);

    VSim3->setId((int)nIDi);
    VSim3->setMarginalized(false);

    optimizer.addVertex(VSim3);

    vpVertices[nIDi] = VSim3;

    sIdKF.insert(nIDi);

    vpGoodPose[nIDi] = true;
    vpBadPose[nIDi] = true;
  }

  for (KeyFrame *pKFi : vpNonFixedKFs) {
    if (pKFi->isBad())
      continue;

    const size_t nIDi = pKFi->mnId;

    if (sIdKF.count(nIDi)) // It has already added in the corrected merge KFs
      continue;

    auto *VSim3 = new g2o::VertexSim3Expmap();

    Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
    g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    vScw[nIDi] = Siw;
    VSim3->setEstimate(Siw);

    VSim3->setFixed(false);

    VSim3->setId((int)nIDi);
    VSim3->setMarginalized(false);

    optimizer.addVertex(VSim3);

    vpVertices[nIDi] = VSim3;

    sIdKF.insert(nIDi);

    vpGoodPose[nIDi] = false;
    vpBadPose[nIDi] = true;
  }

  vector<KeyFrame *> vpKFs;
  vpKFs.reserve(vpFixedKFs.size() + vpFixedCorrectedKFs.size() +
                vpNonFixedKFs.size());
  vpKFs.insert(vpKFs.end(), vpFixedKFs.begin(), vpFixedKFs.end());
  vpKFs.insert(vpKFs.end(), vpFixedCorrectedKFs.begin(),
               vpFixedCorrectedKFs.end());
  vpKFs.insert(vpKFs.end(), vpNonFixedKFs.begin(), vpNonFixedKFs.end());
  set<KeyFrame *> spKFs(vpKFs.begin(), vpKFs.end());

  const Eigen::Matrix<double, 7, 7> matLambda =
      Eigen::Matrix<double, 7, 7>::Identity();

  for (KeyFrame *pKFi : vpKFs) {
    int num_connections = 0;
    const size_t nIDi = pKFi->mnId;

    g2o::Sim3 correctedSwi;
    g2o::Sim3 Swi;

    if (vpGoodPose[nIDi])
      correctedSwi = vCorrectedSwc[nIDi];
    if (vpBadPose[nIDi])
      Swi = vScw[nIDi].inverse();

    KeyFrame *pParentKFi = pKFi->GetParent();

    // Spanning tree edge
    if (pParentKFi && spKFs.find(pParentKFi) != spKFs.end()) {
      size_t nIDj = pParentKFi->mnId;

      g2o::Sim3 Sjw;
      bool bHasRelation = false;

      if (vpGoodPose[nIDi] && vpGoodPose[nIDj]) {
        Sjw = vCorrectedSwc[nIDj].inverse();
        bHasRelation = true;
      } else if (vpBadPose[nIDi] && vpBadPose[nIDj]) {
        Sjw = vScw[nIDj];
        bHasRelation = true;
      }

      if (bHasRelation) {
        g2o::Sim3 Sji = Sjw * Swi;

        auto *e = new g2o::EdgeSim3();
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int)nIDj)));
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int)nIDi)));
        e->setMeasurement(Sji);

        e->information() = matLambda;
        optimizer.addEdge(e);
        num_connections++;
      }
    }

    // Loop edges
    const set<KeyFrame *> sLoopEdges = pKFi->GetLoopEdges();
    for (auto pLKF : sLoopEdges) {
      if (spKFs.find(pLKF) != spKFs.end() && pLKF->mnId < pKFi->mnId) {
        g2o::Sim3 Slw;
        bool bHasRelation = false;

        if (vpGoodPose[nIDi] && vpGoodPose[pLKF->mnId]) {
          Slw = vCorrectedSwc[pLKF->mnId].inverse();
          bHasRelation = true;
        } else if (vpBadPose[nIDi] && vpBadPose[pLKF->mnId]) {
          Slw = vScw[pLKF->mnId];
          bHasRelation = true;
        }

        if (bHasRelation) {
          g2o::Sim3 Sli = Slw * Swi;
          auto *el = new g2o::EdgeSim3();
          el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                               optimizer.vertex((int)pLKF->mnId)));
          el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                               optimizer.vertex((int)nIDi)));
          el->setMeasurement(Sli);
          el->information() = matLambda;
          optimizer.addEdge(el);
          num_connections++;
        }
      }
    }

    // Covisibility graph edges
    const vector<KeyFrame *> vpConnectedKFs =
        pKFi->GetCovisiblesByWeight(minFeat);
    for (auto pKFn : vpConnectedKFs) {
      if (pKFn && pKFn != pParentKFi && !pKFi->hasChild(pKFn) &&
          !sLoopEdges.count(pKFn) && spKFs.find(pKFn) != spKFs.end()) {
        if (!pKFn->isBad() && pKFn->mnId < pKFi->mnId) {
          g2o::Sim3 Snw = vScw[pKFn->mnId];
          bool bHasRelation = false;

          if (vpGoodPose[nIDi] && vpGoodPose[pKFn->mnId]) {
            Snw = vCorrectedSwc[pKFn->mnId].inverse();
            bHasRelation = true;
          } else if (vpBadPose[nIDi] && vpBadPose[pKFn->mnId]) {
            Snw = vScw[pKFn->mnId];
            bHasRelation = true;
          }

          if (bHasRelation) {
            g2o::Sim3 Sni = Snw * Swi;

            auto *en = new g2o::EdgeSim3();
            en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex((int)pKFn->mnId)));
            en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex((int)nIDi)));
            en->setMeasurement(Sni);
            en->information() = matLambda;
            optimizer.addEdge(en);
            num_connections++;
          }
        }
      }
    }

    if (num_connections == 0) {
      Verbose::PrintMess("Opt_Essential: KF " + to_string(pKFi->mnId) +
                             " has 0 connections",
                         Verbose::VERBOSITY_DEBUG);
    }
  }

  // Optimize!
  optimizer.initializeOptimization();
  optimizer.optimize(20);

  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  for (KeyFrame *pKFi : vpNonFixedKFs) {
    if (pKFi->isBad())
      continue;

    const size_t nIDi = pKFi->mnId;

    auto *VSim3 =
        dynamic_cast<g2o::VertexSim3Expmap *>(optimizer.vertex((int)nIDi));
    g2o::Sim3 CorrectedSiw = VSim3->estimate();
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
    double s = CorrectedSiw.scale();
    Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation() / s);

    pKFi->mTcwBefMerge = pKFi->GetPose();
    pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
    pKFi->SetPose(Tiw.cast<float>());
  }

  // Correct points. Transform to "non-optimized" reference keyframe pose and
  // transform back with optimized pose
  for (MapPoint *pMPi : vpNonCorrectedMPs) {
    if (pMPi->isBad())
      continue;

    KeyFrame *pRefKF = pMPi->GetReferenceKeyFrame();
    while (pRefKF->isBad()) {
      if (!pRefKF) {
        Verbose::PrintMess("MP " + to_string(pMPi->mnId) +
                               " without a valid reference KF",
                           Verbose::VERBOSITY_DEBUG);
        break;
      }

      pMPi->EraseObservation(pRefKF);
      pRefKF = pMPi->GetReferenceKeyFrame();
    }

    if (vpBadPose[pRefKF->mnId]) {
      Sophus::SE3f TNonCorrectedwr = pRefKF->mTwcBefMerge;
      Sophus::SE3f Twr = pRefKF->GetPoseInverse();

      Eigen::Vector3f eigCorrectedP3Dw =
          Twr * TNonCorrectedwr.inverse() * pMPi->GetWorldPos();
      pMPi->SetWorldPos(eigCorrectedP3Dw);

      pMPi->UpdateNormalAndDepth();
    } else {
      cout << "ERROR: MapPoint has a reference KF from another map" << endl;
    }
  }
}

int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2,
                            vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12,
                            const float th2, const bool bFixScale,
                            Eigen::Matrix<double, 7, 7> &mAcumHessian,
                            const bool bAllPoints) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType *linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  auto *solver_ptr = new g2o::BlockSolverX(linearSolver);

  auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  // Camera poses
  const Eigen::Matrix3f R1w = pKF1->GetRotation();
  const Eigen::Vector3f t1w = pKF1->GetTranslation();
  const Eigen::Matrix3f R2w = pKF2->GetRotation();
  const Eigen::Vector3f t2w = pKF2->GetTranslation();

  // Set Sim3 vertex
  auto *vSim3 = new RVWO::VertexSim3Expmap();
  vSim3->_fix_scale = bFixScale;
  vSim3->setEstimate(g2oS12);
  vSim3->setId(0);
  vSim3->setFixed(false);
  vSim3->pCamera1 = pKF1->mpCamera;
  vSim3->pCamera2 = pKF2->mpCamera;
  optimizer.addVertex(vSim3);

  // Set MapPoint vertices
  const size_t N = vpMatches1.size();
  const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
  vector<RVWO::EdgeSim3ProjectXYZ *> vpEdges12;
  vector<RVWO::EdgeInverseSim3ProjectXYZ *> vpEdges21;
  vector<size_t> vnIndexEdge;
  vector<bool> vbIsInKF2;

  vnIndexEdge.reserve(2 * N);
  vpEdges12.reserve(2 * N);
  vpEdges21.reserve(2 * N);
  vbIsInKF2.reserve(2 * N);

  const float deltaHuber = sqrt(th2);

  int nCorrespondences = 0;
  int nBadMPs = 0;
  int nInKF2 = 0;
  int nOutKF2 = 0;
  int nMatchWithoutMP = 0;

  vector<int> vIdsOnlyInKF2;

  for (int i = 0; i < N; i++) {
    if (!vpMatches1[i])
      continue;

    MapPoint *pMP1 = vpMapPoints1[i];
    MapPoint *pMP2 = vpMatches1[i];

    const int id1 = 2 * i + 1;
    const int id2 = 2 * (i + 1);

    const int i2 = get<0>(pMP2->GetIndexInKeyFrame(pKF2));

    Eigen::Vector3f P3D1c;
    Eigen::Vector3f P3D2c;

    if (pMP1 && pMP2) {
      if (!pMP1->isBad() && !pMP2->isBad()) {
        auto *vPoint1 = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f P3D1w = pMP1->GetWorldPos();
        P3D1c = R1w * P3D1w + t1w;
        vPoint1->setEstimate(P3D1c.cast<double>());
        vPoint1->setId(id1);
        vPoint1->setFixed(true);
        optimizer.addVertex(vPoint1);

        auto *vPoint2 = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
        P3D2c = R2w * P3D2w + t2w;
        vPoint2->setEstimate(P3D2c.cast<double>());
        vPoint2->setId(id2);
        vPoint2->setFixed(true);
        optimizer.addVertex(vPoint2);
      } else {
        nBadMPs++;
        continue;
      }
    } else {
      nMatchWithoutMP++;

      // TODO The 3D position in KF1 doesn't exist
      if (!pMP2->isBad()) {
        auto *vPoint2 = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
        P3D2c = R2w * P3D2w + t2w;
        vPoint2->setEstimate(P3D2c.cast<double>());
        vPoint2->setId(id2);
        vPoint2->setFixed(true);
        optimizer.addVertex(vPoint2);

        vIdsOnlyInKF2.push_back(id2);
      }
      continue;
    }

    if (i2 < 0 && !bAllPoints) {
      Verbose::PrintMess("    Remove point -> i2: " + to_string(i2) +
                             "; bAllPoints: " + to_string(bAllPoints),
                         Verbose::VERBOSITY_DEBUG);
      continue;
    }

    if (P3D2c(2) < 0) {
      Verbose::PrintMess("Sim3: Z coordinate is negative",
                         Verbose::VERBOSITY_DEBUG);
      continue;
    }

    nCorrespondences++;

    // Set edge x1 = S12*X2
    Eigen::Matrix<double, 2, 1> obs1;
    const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
    const float &kpSegVal1 = pKF1->mvSegVal[i];
    obs1 << kpUn1.pt.x, kpUn1.pt.y;

    auto *e12 = new RVWO::EdgeSim3ProjectXYZ();

    e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex(id2)));
    e12->setVertex(
        1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
    e12->setMeasurement(obs1);
    const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
    e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

    auto *rk1 = new g2o::RobustKernelHuber;
    e12->setRobustKernel(rk1);
    rk1->setDelta(deltaHuber);
    // ARK
    //    auto* ark1 = new g2o::RobustKernelAdaptive;
    //    double alphaBA1;
    //    cv::Vec3b PEOPLE_COLOR(61, 5, 150);
    //    e12->setRobustKernel(ark1);
    //    if (kpSegVal1 == PEOPLE_COLOR) {
    //      alphaBA1 = -10020.0f;
    //    }
    //    else {
    //      alphaBA1 = 2.0f;
    //    }
    //    ark1->setAlpha(alphaBA1);
    optimizer.addEdge(e12);

    // Set edge x2 = S21*X1
    Eigen::Matrix<double, 2, 1> obs2;
    cv::KeyPoint kpUn2;
    float kpSegVal2;
    bool inKF2;
    if (i2 >= 0) {
      kpUn2 = pKF2->mvKeysUn[i2];
      kpSegVal2 = pKF2->mvSegVal[i2];
      obs2 << kpUn2.pt.x, kpUn2.pt.y;
      inKF2 = true;

      nInKF2++;
    } else {
      float invz = 1 / P3D2c(2);
      float x = P3D2c(0) * invz;
      float y = P3D2c(1) * invz;

      obs2 << x, y;
      kpUn2 = cv::KeyPoint(cv::Point2f(x, y), (float)pMP2->mnTrackScaleLevel);

      inKF2 = false;
      nOutKF2++;
    }

    auto *e21 = new RVWO::EdgeInverseSim3ProjectXYZ();

    e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex(id1)));
    e21->setVertex(
        1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
    e21->setMeasurement(obs2);
    float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
    e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

    auto *rk2 = new g2o::RobustKernelHuber;
    e21->setRobustKernel(rk2);
    rk2->setDelta(deltaHuber);
    // ARK
    //    auto* ark2 = new g2o::RobustKernelAdaptive;
    //    double alphaBA2;
    ////    cv::Vec3b PEOPLE_COLOR(61, 5, 150);
    //    e21->setRobustKernel(ark2);
    //    if (kpSegVal2 == PEOPLE_COLOR) {
    //      alphaBA2 = -10020.0f;
    //    }
    //    else {
    //      alphaBA2 = 2.0f;
    //    }
    //    ark2->setAlpha(alphaBA2);
    optimizer.addEdge(e21);

    vpEdges12.push_back(e12);
    vpEdges21.push_back(e21);
    vnIndexEdge.push_back(i);

    vbIsInKF2.push_back(inKF2);
  }

  // Optimize!
  optimizer.initializeOptimization();
  optimizer.optimize(5);

  // Check inliers
  int nBad = 0;
  int nBadOutKF2 = 0;
  for (size_t i = 0; i < vpEdges12.size(); i++) {
    RVWO::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
    RVWO::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
    if (!e12 || !e21)
      continue;

    if (e12->chi2() > th2 || e21->chi2() > th2) {
      size_t idx = vnIndexEdge[i];
      vpMatches1[idx] = static_cast<MapPoint *>(nullptr);
      optimizer.removeEdge(e12);
      optimizer.removeEdge(e21);
      vpEdges12[i] = static_cast<RVWO::EdgeSim3ProjectXYZ *>(nullptr);
      vpEdges21[i] = static_cast<RVWO::EdgeInverseSim3ProjectXYZ *>(nullptr);
      nBad++;

      if (!vbIsInKF2[i]) {
        nBadOutKF2++;
      }
      continue;
    }

    // Check if remove the robust adjustment improve the result
    e12->setRobustKernel(nullptr);
    e21->setRobustKernel(nullptr);
  }

  int nMoreIterations;
  if (nBad > 0)
    nMoreIterations = 10;
  else
    nMoreIterations = 5;

  if (nCorrespondences - nBad < 10)
    return 0;

  // Optimize again only with inliers
  optimizer.initializeOptimization();
  optimizer.optimize(nMoreIterations);

  int nIn = 0;
  mAcumHessian = Eigen::MatrixXd::Zero(7, 7);
  for (size_t i = 0; i < vpEdges12.size(); i++) {
    RVWO::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
    RVWO::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
    if (!e12 || !e21)
      continue;

    e12->computeError();
    e21->computeError();

    if (e12->chi2() > th2 || e21->chi2() > th2) {
      size_t idx = vnIndexEdge[i];
      vpMatches1[idx] = static_cast<MapPoint *>(nullptr);
    } else {
      nIn++;
    }
  }

  // Recover optimized Sim3
  auto *vSim3_recov =
      dynamic_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
  g2oS12 = vSim3_recov->estimate();

  return nIn;
}

Eigen::MatrixXd Optimizer::Marginalize(const Eigen::MatrixXd &H,
                                       const int &start, const int &end) {
  // Goal
  // a  | ab | ac       a*  | 0 | ac*
  // ba | b  | bc  -->  0   | 0 | 0
  // ca | cb | c        ca* | 0 | c*

  // Size of block before block to marginalize
  const int a = start;
  // Size of block to marginalize
  const int b = end - start + 1;
  // Size of block after block to marginalize
  const long c = H.cols() - (end + 1);

  // Reorder as follows:
  // a  | ab | ac       a  | ac | ab
  // ba | b  | bc  -->  ca | c  | cb
  // ca | cb | c        ba | bc | b

  Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());
  if (a > 0) {
    Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
    Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
    Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
  }
  if (a > 0 && c > 0) {
    Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
    Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
  }
  if (c > 0) {
    Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
    Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
    Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
  }
  Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

  // Perform marginalization (Schur complement)
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      Hn.block(a + c, a + c, b, b), Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv =
      svd.singularValues();
  for (int i = 0; i < b; ++i) {
    if (singularValues_inv(i) > 1e-6)
      singularValues_inv(i) = 1.0 / singularValues_inv(i);
    else
      singularValues_inv(i) = 0;
  }
  Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() *
                          svd.matrixU().transpose();
  Hn.block(0, 0, a + c, a + c) =
      Hn.block(0, 0, a + c, a + c) -
      Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
  Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
  Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
  Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

  // Inverse reorder
  // a*  | ac* | 0       a*  | 0 | ac*
  // ca* | c*  | 0  -->  0   | 0 | 0
  // 0   | 0   | 0       ca* | 0 | c*
  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());
  if (a > 0) {
    res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
    res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
    res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
  }
  if (a > 0 && c > 0) {
    res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
    res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
  }
  if (c > 0) {
    res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
    res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
    res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
  }

  res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

  return res;
}

void Optimizer::LocalBundleAdjustment(KeyFrame *pMainKF,
                                      vector<KeyFrame *> vpAdjustKF,
                                      const vector<KeyFrame *> &vpFixedKF,
                                      bool *pbStopFlag) {
  vector<MapPoint *> vpMPs;

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  auto *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  optimizer.setVerbose(false);

  if (pbStopFlag)
    optimizer.setForceStopFlag(pbStopFlag);

  long unsigned int maxKFid = 0;
  set<KeyFrame *> spKeyFrameBA;

  Map *pCurrentMap = pMainKF->GetMap();

  // Set fixed KeyFrame vertices
  int numInsertedPoints = 0;
  for (KeyFrame *pKFi : vpFixedKF) {
    if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap) {
      Verbose::PrintMess("ERROR LBA: KF is bad or is not in the current map",
                         Verbose::VERBOSITY_NORMAL);
      continue;
    }

    pKFi->mnBALocalForMerge = pMainKF->mnId;

    auto *vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId((int)(pKFi->mnId));
    vSE3->setFixed(true);
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid)
      maxKFid = pKFi->mnId;

    set<MapPoint *> spViewMPs = pKFi->GetMapPoints();
    for (MapPoint *pMPi : spViewMPs) {
      if (pMPi)
        if (!pMPi->isBad() && pMPi->GetMap() == pCurrentMap)

          if (pMPi->mnBALocalForMerge != pMainKF->mnId) {
            vpMPs.push_back(pMPi);
            pMPi->mnBALocalForMerge = pMainKF->mnId;
            numInsertedPoints++;
          }
    }

    spKeyFrameBA.insert(pKFi);
  }

  // Set non fixed Keyframe vertices
  set<KeyFrame *> spAdjustKF(vpAdjustKF.begin(), vpAdjustKF.end());
  numInsertedPoints = 0;
  for (KeyFrame *pKFi : vpAdjustKF) {
    if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
      continue;

    pKFi->mnBALocalForMerge = pMainKF->mnId;

    auto *vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId((int)(pKFi->mnId));
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid)
      maxKFid = pKFi->mnId;

    set<MapPoint *> spViewMPs = pKFi->GetMapPoints();
    for (MapPoint *pMPi : spViewMPs) {
      if (pMPi) {
        if (!pMPi->isBad() && pMPi->GetMap() == pCurrentMap) {
          if (pMPi->mnBALocalForMerge != pMainKF->mnId) {
            vpMPs.push_back(pMPi);
            pMPi->mnBALocalForMerge = pMainKF->mnId;
            numInsertedPoints++;
          }
        }
      }
    }

    spKeyFrameBA.insert(pKFi);
  }

  const size_t nExpectedSize =
      (vpAdjustKF.size() + vpFixedKF.size()) * vpMPs.size();

  vector<RVWO::EdgeSE3ProjectXYZ *> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  vector<KeyFrame *> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  vector<MapPoint *> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  vector<KeyFrame *> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<MapPoint *> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const double thHuber2D = sqrt(5.99);
  const double thHuber3D = sqrt(7.815);

  // Set MapPoint vertices
  map<KeyFrame *, int> mpObsKFs;
  map<KeyFrame *, int> mpObsFinalKFs;
  map<MapPoint *, int> mpObsMPs;
  for (auto pMPi : vpMPs) {
    if (pMPi->isBad())
      continue;

    auto *vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMPi->GetWorldPos().cast<double>());
    const int id = (int)(pMPi->mnId + maxKFid + 1);
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const map<KeyFrame *, tuple<int, int>> observations =
        pMPi->GetObservations();
    int nEdges = 0;
    // SET EDGES
    for (const auto &observation : observations) {
      KeyFrame *pKF = observation.first;
      if (pKF->isBad() || pKF->mnId > maxKFid ||
          pKF->mnBALocalForMerge != pMainKF->mnId ||
          !pKF->GetMapPoint(get<0>(observation.second)))
        continue;

      nEdges++;

      const int leftIndex = get<0>(observation.second);
      const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];
      const float &kpSegVal = pKF->mvSegVal[leftIndex];

      if (pKF->mvuRight[get<0>(observation.second)] < 0) // Monocular
      {
        mpObsMPs[pMPi]++;
        Eigen::Matrix<double, 2, 1> obs;
        obs << kpUn.pt.x, kpUn.pt.y;

        auto *e = new RVWO::EdgeSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int)(id))));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int)(pKF->mnId))));
        e->setMeasurement(obs);
        const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

        //        auto* rk = new g2o::RobustKernelHuber;
        //        e->setRobustKernel(rk);
        //        rk->setDelta(thHuber2D);

        // ARK Bayes
#if HT
        auto *ark = new g2o::RobustKernelAdaptive;
        double alphaBA = getAlfa(pMPi->mRelInv);
        cv::Vec3b PEOPLE_COLOR(61, 5, 150);
        e->setRobustKernel(ark);
        if (kpSegVal == PEOPLE_COLOR) {
          alphaBA = -10020.0f;
        } else {
          alphaBA = -2.0f;
        }
        ark->setAlpha(alphaBA);
#else
        LabelToProb(kpUn, kpSegVal, pMPi, pKF);
        auto *ark = new g2o::RobustKernelAdaptive;
        double alphaBA = getAlfa(pMPi->mRelInv, pMPi->mReprErr);
        e->setRobustKernel(ark);
        ark->setAlpha(alphaBA);
#endif
        e->pCamera = pKF->mpCamera;

        optimizer.addEdge(e);

        vpEdgesMono.push_back(e);
        vpEdgeKFMono.push_back(pKF);
        vpMapPointEdgeMono.push_back(pMPi);

        mpObsKFs[pKF]++;
      } else // RGBD or Stereo
      {
        mpObsMPs[pMPi] += 2;
        Eigen::Matrix<double, 3, 1> obs;
        const float kp_ur = pKF->mvuRight[get<0>(observation.second)];
        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

        auto *e = new g2o::EdgeStereoSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int)(id))));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int)(pKF->mnId))));
        e->setMeasurement(obs);
        const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
        e->setInformation(Info);

        //        auto* rk = new g2o::RobustKernelHuber;
        //        e->setRobustKernel(rk);
        //        rk->setDelta(thHuber3D);

        // ARK Bayes
#if HT
        auto *rk = new g2o::RobustKernelAdaptive;
        double alphaBA = getAlfa(pMPi->mRelInv);
        cv::Vec3b PEOPLE_COLOR(61, 5, 150);
        e->setRobustKernel(rk);
        if (kpSegVal == PEOPLE_COLOR) {
          alphaBA = -10020.0f;
        } else {
          alphaBA = -2.0f;
        }
        rk->setAlpha(alphaBA);
#else
        LabelToProb(kpUn, kpSegVal, pMPi, pKF);
        auto *rk = new g2o::RobustKernelAdaptive;
        double alphaBA = getAlfa(pMPi->mRelInv, pMPi->mReprErr);
        e->setRobustKernel(rk);
        rk->setAlpha(alphaBA);
#endif
        e->fx = pKF->fx;
        e->fy = pKF->fy;
        e->cx = pKF->cx;
        e->cy = pKF->cy;
        e->bf = pKF->mbf;

        optimizer.addEdge(e);

        vpEdgesStereo.push_back(e);
        vpEdgeKFStereo.push_back(pKF);
        vpMapPointEdgeStereo.push_back(pMPi);

        mpObsKFs[pKF]++;
      }
    }
  }

  if (pbStopFlag)
    if (*pbStopFlag)
      return;

  optimizer.initializeOptimization();
  optimizer.optimize(5);

  bool bDoMore = true;

  if (pbStopFlag)
    if (*pbStopFlag)
      bDoMore = false;

  map<unsigned long int, int> mWrongObsKF;
  if (bDoMore) {
    // Check inlier observations
    int badMonoMP = 0, badStereoMP = 0;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      RVWO::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
      MapPoint *pMP = vpMapPointEdgeMono[i];

      if (pMP->isBad())
        continue;

      if (e->chi2() > 5.991 || !e->isDepthPositive()) {
        e->setLevel(1);
        badMonoMP++;
      }
      e->setRobustKernel(nullptr);
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
      MapPoint *pMP = vpMapPointEdgeStereo[i];

      if (pMP->isBad())
        continue;

      if (e->chi2() > 7.815 || !e->isDepthPositive()) {
        e->setLevel(1);
        badStereoMP++;
      }

      e->setRobustKernel(nullptr);
    }
    Verbose::PrintMess("[BA]: First optimization(Huber), there are " +
                           to_string(badMonoMP) + " monocular and " +
                           to_string(badStereoMP) + " stereo bad edges",
                       Verbose::VERBOSITY_DEBUG);

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);
  }

  vector<pair<KeyFrame *, MapPoint *>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());
  set<MapPoint *> spErasedMPs;
  set<KeyFrame *> spErasedKFs;

  // Check inlier observations
  int badMonoMP = 0, badStereoMP = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    RVWO::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
    MapPoint *pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad())
      continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      KeyFrame *pKFi = vpEdgeKFMono[i];
      vToErase.push_back(make_pair(pKFi, pMP));
      mWrongObsKF[pKFi->mnId]++;
      badMonoMP++;

      spErasedMPs.insert(pMP);
      spErasedKFs.insert(pKFi);
    }
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
    MapPoint *pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad())
      continue;

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      KeyFrame *pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(make_pair(pKFi, pMP));
      mWrongObsKF[pKFi->mnId]++;
      badStereoMP++;

      spErasedMPs.insert(pMP);
      spErasedKFs.insert(pKFi);
    }
  }

  Verbose::PrintMess("[BA]: Second optimization, there are " +
                         to_string(badMonoMP) + " monocular and " +
                         to_string(badStereoMP) + " sterero bad edges",
                     Verbose::VERBOSITY_DEBUG);

  // Get Map Mutex
  unique_lock<mutex> lock(pMainKF->GetMap()->mMutexMapUpdate);

  if (!vToErase.empty()) {
    for (auto &i : vToErase) {
      KeyFrame *pKFi = i.first;
      MapPoint *pMPi = i.second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }
  for (auto pMPi : vpMPs) {
    if (pMPi->isBad())
      continue;

    const map<KeyFrame *, tuple<int, int>> observations =
        pMPi->GetObservations();
    for (const auto &observation : observations) {
      KeyFrame *pKF = observation.first;
      if (pKF->isBad() || pKF->mnId > maxKFid ||
          pKF->mnBALocalForKF != pMainKF->mnId ||
          !pKF->GetMapPoint(get<0>(observation.second)))
        continue;

      if (pKF->mvuRight[get<0>(observation.second)] < 0) // Monocular
      {
        mpObsFinalKFs[pKF]++;
      } else // RGBD or Stereo
      {
        mpObsFinalKFs[pKF]++;
      }
    }
  }

  // Recover optimized data
  // Keyframes
  for (KeyFrame *pKFi : vpAdjustKF) {
    if (pKFi->isBad())
      continue;

    auto *vSE3 = dynamic_cast<g2o::VertexSE3Expmap *>(
        optimizer.vertex((int)(pKFi->mnId)));
    g2o::SE3Quat SE3quat = vSE3->estimate();
    Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                     SE3quat.translation().cast<float>());

    int numMonoBadPoints = 0, numMonoOptPoints = 0;
    int numStereoBadPoints = 0, numStereoOptPoints = 0;
    vector<MapPoint *> vpMonoMPsOpt, vpStereoMPsOpt;
    vector<MapPoint *> vpMonoMPsBad, vpStereoMPsBad;

    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      RVWO::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
      MapPoint *pMP = vpMapPointEdgeMono[i];
      KeyFrame *pKFedge = vpEdgeKFMono[i];

      if (pKFi != pKFedge) {
        continue;
      }

      if (pMP->isBad())
        continue;

      if (e->chi2() > 5.991 || !e->isDepthPositive()) {
        numMonoBadPoints++;
        vpMonoMPsBad.push_back(pMP);

      } else {
        numMonoOptPoints++;
        vpMonoMPsOpt.push_back(pMP);
      }
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
      MapPoint *pMP = vpMapPointEdgeStereo[i];
      KeyFrame *pKFedge = vpEdgeKFMono[i];

      if (pKFi != pKFedge) {
        continue;
      }

      if (pMP->isBad())
        continue;

      if (e->chi2() > 7.815 || !e->isDepthPositive()) {
        numStereoBadPoints++;
        vpStereoMPsBad.push_back(pMP);
      } else {
        numStereoOptPoints++;
        vpStereoMPsOpt.push_back(pMP);
      }
    }

    pKFi->SetPose(Tiw);
  }

  // Points
  for (MapPoint *pMPi : vpMPs) {
    if (pMPi->isBad())
      continue;

    auto *vPoint = dynamic_cast<g2o::VertexSBAPointXYZ *>(
        optimizer.vertex((int)(pMPi->mnId + maxKFid + 1)));
    pMPi->SetWorldPos(vPoint->estimate().cast<float>());
    pMPi->UpdateNormalAndDepth();
  }
}

void Optimizer::OptimizeEssentialGraph4DoF(
    Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
    const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
    const LoopClosing::KeyFrameAndPose &CorrectedSim3,
    const map<KeyFrame *, set<KeyFrame *>> &LoopConnections) {

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::BlockSolverX::LinearSolverType *linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
  auto *solver_ptr = new g2o::BlockSolverX(linearSolver);

  auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  optimizer.setAlgorithm(solver);

  const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
  const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
      nMaxKFid + 1);

  vector<VertexPose4DoF *> vpVertices(nMaxKFid + 1);

  const int minFeat = 100;
  // Set KeyFrame vertices
  for (auto pKF : vpKFs) {
    if (pKF->isBad())
      continue;

    VertexPose4DoF *V4DoF;

    const size_t nIDi = pKF->mnId;

    auto it = CorrectedSim3.find(pKF);

    if (it != CorrectedSim3.end()) {
      vScw[nIDi] = it->second;
      const g2o::Sim3 Swc = it->second.inverse();
      Eigen::Matrix3d Rwc = Swc.rotation().toRotationMatrix();
      Eigen::Vector3d twc = Swc.translation();
      V4DoF = new VertexPose4DoF(Rwc, twc, pKF);
    } else {
      Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
      g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

      vScw[nIDi] = Siw;
      V4DoF = new VertexPose4DoF(pKF);
    }

    if (pKF == pLoopKF)
      V4DoF->setFixed(true);

    V4DoF->setId((int)(nIDi));
    V4DoF->setMarginalized(false);

    optimizer.addVertex(V4DoF);
    vpVertices[nIDi] = V4DoF;
  }
  set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

  // Edge used in posegraph has still 6Dof, even if updates of camera poses are
  // just in 4DoF
  Eigen::Matrix<double, 6, 6> matLambda =
      Eigen::Matrix<double, 6, 6>::Identity();
  matLambda(0, 0) = 1e3;
  matLambda(1, 1) = 1e3;
  matLambda(0, 0) = 1e3;

  // Set Loop edges
  for (const auto &LoopConnection : LoopConnections) {
    KeyFrame *pKF = LoopConnection.first;
    const long unsigned int nIDi = pKF->mnId;
    const set<KeyFrame *> &spConnections = LoopConnection.second;
    const g2o::Sim3 Siw = vScw[nIDi];

    for (auto spConnection : spConnections) {
      const long unsigned int nIDj = spConnection->mnId;
      if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
          pKF->GetWeight(spConnection) < minFeat)
        continue;

      const g2o::Sim3 Sjw = vScw[nIDj];
      const g2o::Sim3 Sij = Siw * Sjw.inverse();
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3) = 1.;

      auto *e = new Edge4DoF(Tij);
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex((int)(nIDj))));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex((int)(nIDi))));

      e->information() = matLambda;
      optimizer.addEdge(e);

      sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
    }
  }

  // 1. Set normal edges
  for (auto pKF : vpKFs) {
    const size_t nIDi = pKF->mnId;

    g2o::Sim3 Siw;

    // Use noncorrected poses for posegraph edges
    auto iti = NonCorrectedSim3.find(pKF);

    if (iti != NonCorrectedSim3.end())
      Siw = iti->second;
    else
      Siw = vScw[nIDi];

    // 1.1.0 Spanning tree edge
    auto *pParentKF = static_cast<KeyFrame *>(nullptr);
    if (pParentKF) {
      int nIDj = (int)(pParentKF->mnId);

      g2o::Sim3 Swj;

      auto itj = NonCorrectedSim3.find(pParentKF);

      if (itj != NonCorrectedSim3.end())
        Swj = (itj->second).inverse();
      else
        Swj = vScw[nIDj].inverse();

      g2o::Sim3 Sij = Siw * Swj;
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3) = 1.;

      auto *e = new Edge4DoF(Tij);
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex((int)(nIDi))));
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex(nIDj)));
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // 1.1.1 Inertial edges
    KeyFrame *prevKF = pKF->mPrevKF;
    if (prevKF) {
      size_t nIDj = prevKF->mnId;

      g2o::Sim3 Swj;

      auto itj = NonCorrectedSim3.find(prevKF);

      if (itj != NonCorrectedSim3.end())
        Swj = (itj->second).inverse();
      else
        Swj = vScw[nIDj].inverse();

      g2o::Sim3 Sij = Siw * Swj;
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3) = 1.;

      auto *e = new Edge4DoF(Tij);
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex((int)(nIDi))));
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex((int)(nIDj))));
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // 1.2 Loop edges
    const set<KeyFrame *> sLoopEdges = pKF->GetLoopEdges();
    for (auto pLKF : sLoopEdges) {
      if (pLKF->mnId < pKF->mnId) {
        g2o::Sim3 Swl;

        auto itl = NonCorrectedSim3.find(pLKF);

        if (itl != NonCorrectedSim3.end())
          Swl = itl->second.inverse();
        else
          Swl = vScw[pLKF->mnId].inverse();

        g2o::Sim3 Sil = Siw * Swl;
        Eigen::Matrix4d Til;
        Til.block<3, 3>(0, 0) = Sil.rotation().toRotationMatrix();
        Til.block<3, 1>(0, 3) = Sil.translation();
        Til(3, 3) = 1.;

        auto *e = new Edge4DoF(Til);
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int)(nIDi))));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int)(pLKF->mnId))));
        e->information() = matLambda;
        optimizer.addEdge(e);
      }
    }

    // 1.3 Covisibility graph edges
    const vector<KeyFrame *> vpConnectedKFs =
        pKF->GetCovisiblesByWeight(minFeat);
    for (auto pKFn : vpConnectedKFs) {
      if (pKFn && pKFn != pParentKF && pKFn != prevKF && pKFn != pKF->mNextKF &&
          !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn)) {
        if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
          if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId),
                                             max(pKF->mnId, pKFn->mnId))))
            continue;

          g2o::Sim3 Swn;

          auto itn = NonCorrectedSim3.find(pKFn);

          if (itn != NonCorrectedSim3.end())
            Swn = itn->second.inverse();
          else
            Swn = vScw[pKFn->mnId].inverse();

          g2o::Sim3 Sin = Siw * Swn;
          Eigen::Matrix4d Tin;
          Tin.block<3, 3>(0, 0) = Sin.rotation().toRotationMatrix();
          Tin.block<3, 1>(0, 3) = Sin.translation();
          Tin(3, 3) = 1.;
          auto *e = new Edge4DoF(Tin);
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              optimizer.vertex((int)(nIDi))));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              optimizer.vertex((int)(pKFn->mnId))));
          e->information() = matLambda;
          optimizer.addEdge(e);
        }
      }
    }
  }

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  optimizer.optimize(20);

  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  for (auto pKFi : vpKFs) {
    const size_t nIDi = pKFi->mnId;

    auto *Vi = dynamic_cast<VertexPose4DoF *>(optimizer.vertex((int)(nIDi)));
    Eigen::Matrix3d Ri = Vi->estimate().Rcw[0];
    Eigen::Vector3d ti = Vi->estimate().tcw[0];

    g2o::Sim3 CorrectedSiw = g2o::Sim3(Ri, ti, 1.);
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();

    Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation());
    pKFi->SetPose(Tiw.cast<float>());
  }

  // Correct points. Transform to "non-optimized" reference keyframe pose and
  // transform back with optimized pose
  for (auto pMP : vpMPs) {
    if (pMP->isBad())
      continue;

    KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
    size_t nIDr = pRefKF->mnId;

    g2o::Sim3 Srw = vScw[nIDr];
    g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

    Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->GetWorldPos().cast<double>();
    Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
        correctedSwr.map(Srw.map(eigP3Dw));
    pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

    pMP->UpdateNormalAndDepth();
  }
  pMap->IncreaseChangeIndex();
}

} // namespace RVWO
