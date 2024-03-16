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

#include "Optimizer.h"

#include <Eigen/StdVector>
#include <complex>
#include <mutex>
#include <unsupported/Eigen/MatrixFunctions>

#include "Converter.h"
#include "G2oTypes.h"
#include "OptimizableTypes.h"

#define HT false

namespace ORB_SLAM3 {
    double Optimizer::getAlfa(double prob, double err) {
        if (prob > 1.0f) prob = 1.0f;
//    std::cout << prob << std::endl;
//        FILE *fp;
//        string prob_log_dir = "/home/wfram/r_viwo_ark_ws/src/R-VIWO-ARK/prob_log.txt";
//        fp = fopen(prob_log_dir.c_str(), "a");
//        fprintf(fp, "%0.3f \n", prob);
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

    void Optimizer::LabelToProb(const cv::KeyPoint &kpUn, const float &kpSegVal, MapPoint *pMP, Frame *pF) {
        // This is a test for fusing the semantic label effect on the probability of movement
        // Criteria of movement right now just if the object is person and has big projection error
        cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//        float Px[3][3][3];
//        memset(Px, 0, sizeof Px);
//        Px[0][1][0] = 0.95f; // P( person, high, was_person ) should be quite high
//        Px[1][1][0] = 0.05f; // P( not_person, high, was_person ) should NOT be high
//        Px[0][1][1] = 0.7f; // P( person, high, was_not_person ) 0.7 should be pretty high, not too much
//        Px[1][1][1] = 0.3f; // P( not_person, high, was_not_person ) 0.3 should be low
        // TODO: Check 3, 4, 5, 6 - they are of interest

//        Px[0][0][0] = 0.4f; // P( person, low, was_person ) 0.3 should be somehow in a middle
//        Px[1][0][0] = 0.6f; // P( not_person, low, was_person ) 0.7
//        Px[0][0][1] = 0.05f; // P( person, low, was_not_person ) 0.2
//        Px[1][0][1] = 0.95f; // P( not_person, low, was_not_person ) 0.8 should be quite high

        float Pz[3][3];
        memset(Pz, 0, sizeof Pz);
        // Prediction says that the object is ... What about semantics?
        Pz[0][0] = 1.00f; // P(person, person) 0.7
        Pz[1][0] = 0.50f; // P(not_person, person) 0.3
        Pz[0][1] = 0.50f; // P(person, not_person)
        Pz[1][1] = 1.00f; // P(not_person, not_person)

//        FILE *fp;
//        std::string label_log_dir = "/home/wfram/r_viwo_ark_ws/src/R-VIWO-ARK/label_log.txt";
//        fp = fopen(label_log_dir.c_str(), "a");

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
//                     Px(0, ut, 0) * belPrev_0 + Px(0, ut, 1) * belPrev_1;  // Person
//            float belCurP_1 = Px(1, ut, 0) * belPrev_0 +
//                              Px(1, ut, 1) * belPrev_1;  // Not a person
            float belCurC_0 = Pz[zt][0] * belPrev_0; // TODO: Discrete story again, let's work out
            float belCurC_1 = Pz[zt][1] * belPrev_1;

//            fprintf(fp,
//                    "%ld \t %d \t %0.3f \t %0.3f \n",
//                    pMP->mnId,
//                    zt,
//                    pMP->GetWorldPosPrev().x() + pMP->GetWorldPosPrev().y() + pMP->GetWorldPosPrev().z(),
//                    pMP->GetWorldPos().x() + pMP->GetWorldPos().y() + pMP->GetWorldPos().z());
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

    void Optimizer::LabelToProb(const cv::KeyPoint &kpUn, const float &kpSegVal, MapPoint *pMP, KeyFrame *pKF) {
        // This is a test for fusing the sematic label effect on the probability of movement
        // Criteria of movement right now just if the object is person and has big projection error
        cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//        float Px[3][3][3];
//        memset(Px, 0, sizeof Px);
//        Px[0][1][0] = 0.95f; // P( person, high, was_person ) should be quite high
//        Px[1][1][0] = 0.05f; // P( not_person, high, was_person ) should NOT be high
//        Px[0][1][1] = 0.7f; // P( person, high, was_not_person ) 0.7 should be pretty high, not too much
//        Px[1][1][1] = 0.3f; // P( not_person, high, was_not_person ) 0.3 should be low
        // TODO: Check 3, 4, 5, 6 - they are of interest

//        Px[0][0][0] = 0.4f; // P( person, low, was_person ) 0.3 should be somehow in a middle
//        Px[1][0][0] = 0.6f; // P( not_person, low, was_person ) 0.7
//        Px[0][0][1] = 0.05f; // P( person, low, was_not_person ) 0.2
//        Px[1][0][1] = 0.95f; // P( not_person, low, was_not_person ) 0.8 should be quite high

        float Pz[3][3];
        memset(Pz, 0, sizeof Pz);
        // They should depend on a NN itself
        Pz[0][0] = 1.00f; // P( person, person) 0.7
        Pz[1][0] = 0.50f; // P( not_person, person) 0.3
        Pz[0][1] = 0.50f; // P( person, not_person)
        Pz[1][1] = 1.00f; // P( not_person, not_person)

        FILE *fp;
        std::string label_log_dir = "/home/wfram/r_viwo_ark_ws/src/R-VIWO-ARK/label_log.txt";
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

        fprintf(fp,
                "%0.3f \n",
                err);

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
//                    Px(0, ut, 0) * belPrev_0 + Px(0, ut, 1) * belPrev_1;  // Person
//            float belCurP_1 = Px(1, ut, 0) * belPrev_0 +
//                              Px(1, ut, 1) * belPrev_1;  // Not a person
            float belCurC_0 = Pz[zt][0] * belPrev_0;
            float belCurC_1 = Pz[zt][1] * belPrev_1;

//            fprintf(fp,
//                    "%ld \t %d \t %0.3f \t %0.3f \n",
//                    pMP->mnId,
//                    zt,
//                    pMP->GetWorldPosPrev().x() + pMP->GetWorldPosPrev().y() + pMP->GetWorldPosPrev().z(),
//                    pMP->GetWorldPos().x() + pMP->GetWorldPos().y() + pMP->GetWorldPos().z());

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
                                     const vector<MapPoint *> &vpMP, int nIterations,
                                     bool *pbStopFlag, const unsigned long nLoopKF,
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

        if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

        long unsigned int maxKFid = 0;

        const int nExpectedSize = (int) (vpKFs.size() * vpMP.size());

        vector<ORB_SLAM3::EdgeSE3ProjectXYZ *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody *> vpEdgesBody;
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

        for (auto pKF: vpKFs) {
            if (pKF->isBad()) continue;
            auto *vSE3 = new g2o::VertexSE3Expmap();
            Sophus::SE3<float> Tcw = pKF->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                           Tcw.translation().cast<double>()));
            vSE3->setId((int) (pKF->mnId));
            vSE3->setFixed(pKF->mnId == pMap->GetInitKFid());
            optimizer.addVertex(vSE3);
            if (pKF->mnId > maxKFid) maxKFid = pKF->mnId;
        }

        // TODO: Write a function to choose alfa parameter instead of delta threshold
        // Since alpha parameter is being chosen for the whole edge we need to find a way of
        // choosing something in a middle for two KFs
        const double thHuber2D = sqrt(5.99);
        const double thHuber3D = sqrt(7.815);

        // Set MapPoint vertices
        for (size_t i = 0; i < vpMP.size(); i++) {
            MapPoint *pMP = vpMP[i];
            if (pMP->isBad()) continue;
            auto *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
            const int id = (int) (pMP->mnId + maxKFid + 1);
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

            int nEdges = 0;
            // SET EDGES
            for (const auto &observation: observations) {
                KeyFrame *pKF = observation.first;
                if (pKF->isBad() || pKF->mnId > maxKFid) continue;
                if (optimizer.vertex(id) == nullptr ||
                    optimizer.vertex((int) (pKF->mnId)) == nullptr)
                    continue;
                nEdges++;

                const int leftIndex = get<0>(observation.second);

                if (leftIndex != -1 && pKF->mvuRight[get<0>(observation.second)] < 0) {
                    const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];
                    const float &kpSegVal = pKF->mvSegVal[leftIndex];

                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    auto *e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int) (pKF->mnId))));
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
                           pKF->mvuRight[leftIndex] >= 0)  // Stereo observation
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
                            optimizer.vertex((int) (pKF->mnId))));
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

                        auto *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) (pKF->mnId))));
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
        for (auto pKF: vpKFs) {
            if (pKF->isBad()) continue;
            auto *vSE3 =
                    dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex((int) (pKF->mnId)));

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
                        ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i2];
                        MapPoint *pMP = vpMapPointEdgeMono[i2];
                        KeyFrame *pKFedge = vpEdgeKFMono[i2];

                        if (pKF != pKFedge) {
                            continue;
                        }

                        if (pMP->isBad()) continue;

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

                        if (pMP->isBad()) continue;

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
            if (vbNotIncludedMP[i]) continue;

            MapPoint *pMP = vpMP[i];

            if (pMP->isBad()) continue;
            auto *vPoint = dynamic_cast<g2o::VertexSBAPointXYZ *>(
                    optimizer.vertex((int) (pMP->mnId + maxKFid + 1)));

            if (nLoopKF == pMap->GetOriginKF()->mnId) {
                pMP->SetWorldPos(vPoint->estimate().cast<float>());
                pMP->UpdateNormalAndDepth();
            } else {
                pMP->mPosGBA = vPoint->estimate().cast<float>();
                pMP->mnBAGlobalForKF = nLoopKF;
            }
        }
    }

    void Optimizer::FullInertialBA(Map *pMap, int its, const bool bFixLocal,
                                   const long unsigned int nLoopId,
                                   bool *pbStopFlag, bool bInit, float priorG,
                                   float priorA, Eigen::VectorXd *vSingVal,
                                   bool *bHess) {
        long unsigned int maxKFid = pMap->GetMaxKFid();
        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver =
                new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        auto *solver_ptr = new g2o::BlockSolverX(linearSolver);

        auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(1e-5);
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

        int nNonFixed = 0;

        // Set KeyFrame vertices
        KeyFrame *pIncKF;
        for (auto pKFi: vpKFs) {
            if (pKFi->mnId > maxKFid) continue;
            auto *VP = new VertexPose(pKFi);
            VP->setId((int) (pKFi->mnId));
            pIncKF = pKFi;
            bool bFixed = false;
            if (bFixLocal) {
                bFixed = (pKFi->mnBALocalForKF >= (maxKFid - 1)) ||
                         (pKFi->mnBAFixedForKF >= (maxKFid - 1));
                if (!bFixed) nNonFixed++;
                VP->setFixed(bFixed);
            }
            optimizer.addVertex(VP);

            if (pKFi->bImu) {
                auto *VV = new VertexVelocity(pKFi);
                VV->setId((int) (maxKFid + 3 * (pKFi->mnId) + 1));
                VV->setFixed(bFixed);
                optimizer.addVertex(VV);
                if (!bInit) {
                    auto *VG = new VertexGyroBias(pKFi);
                    VG->setId((int) (maxKFid + 3 * (pKFi->mnId) + 2));
                    VG->setFixed(bFixed);
                    optimizer.addVertex(VG);
                    auto *VA = new VertexAccBias(pKFi);
                    VA->setId((int) (maxKFid + 3 * (pKFi->mnId) + 3));
                    VA->setFixed(bFixed);
                    optimizer.addVertex(VA);
                }
            }
        }

        if (bInit) {
            auto *VG = new VertexGyroBias(pIncKF);
            VG->setId((int) (4 * maxKFid + 2));
            VG->setFixed(false);
            optimizer.addVertex(VG);
            auto *VA = new VertexAccBias(pIncKF);
            VA->setId((int) (4 * maxKFid + 3));
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }

        if (bFixLocal) {
            if (nNonFixed < 3) return;
        }

        // IMU links
        for (auto pKFi: vpKFs) {
            if (!pKFi->mPrevKF) {
                Verbose::PrintMess("NOT INERTIAL LINK TO PREVIOUS FRAME!",
                                   Verbose::VERBOSITY_NORMAL);
                continue;
            }

            if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
                if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;
                if (pKFi->bImu && pKFi->mPrevKF->bImu) {
                    pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
                    g2o::HyperGraph::Vertex *VP1 =
                            optimizer.vertex((int) (pKFi->mPrevKF->mnId));
                    g2o::HyperGraph::Vertex *VV1 =
                            optimizer.vertex((int) (maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1));

                    g2o::HyperGraph::Vertex *VG1;
                    g2o::HyperGraph::Vertex *VA1;
                    g2o::HyperGraph::Vertex *VG2;
                    g2o::HyperGraph::Vertex *VA2;
                    if (!bInit) {
                        VG1 =
                                optimizer.vertex((int) (maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2));
                        VA1 =
                                optimizer.vertex((int) (maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3));
                        VG2 = optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 2));
                        VA2 = optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 3));
                    } else {
                        VG1 = optimizer.vertex((int) (4 * maxKFid + 2));
                        VA1 = optimizer.vertex((int) (4 * maxKFid + 3));
                    }

                    g2o::HyperGraph::Vertex *VP2 = optimizer.vertex((int) (pKFi->mnId));
                    g2o::HyperGraph::Vertex *VV2 =
                            optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 1));

                    if (!bInit) {
                        if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
                            cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
                                 << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2
                                 << endl;
                            continue;
                        }
                    } else {
                        if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2) {
                            cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
                                 << ", " << VP2 << ", " << VV2 << endl;
                            continue;
                        }
                    }

                    auto *ei = new EdgeInertial(pKFi->mpImuPreintegrated);
                    ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                    ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
                    ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG1));
                    ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA1));
                    ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                    ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));

                    auto *rki = new g2o::RobustKernelHuber;
                    ei->setRobustKernel(rki);
                    rki->setDelta(sqrt(16.92));

                    optimizer.addEdge(ei);

                    if (!bInit) {
                        auto *egr = new EdgeGyroRW();
                        egr->setVertex(0, VG1);
                        egr->setVertex(1, VG2);
                        Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                                .cast<double>()
                                .inverse();
                        egr->setInformation(InfoG);
                        egr->computeError();
                        optimizer.addEdge(egr);

                        auto *ear = new EdgeAccRW();
                        ear->setVertex(0, VA1);
                        ear->setVertex(1, VA2);
                        Eigen::Matrix3d InfoA =
                                pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                                        .cast<double>()
                                        .inverse();
                        ear->setInformation(InfoA);
                        ear->computeError();
                        optimizer.addEdge(ear);
                    }
                    if (pKFi->mpOdomPreintegrated) {
                        auto *eo = new EdgeWOdometry(pKFi->mpOdomPreintegrated);
                        eo->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                        eo->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                        optimizer.addEdge(eo);
                    }
                } else
                    cout << pKFi->mnId << " or " << pKFi->mPrevKF->mnId << " no imu"
                         << endl;
            }
        }

        if (bInit) {
            g2o::HyperGraph::Vertex *VG = optimizer.vertex((int) (4 * maxKFid + 2));
            g2o::HyperGraph::Vertex *VA = optimizer.vertex((int) (4 * maxKFid + 3));

            // Add prior to comon biases
            Eigen::Vector3f bprior;
            bprior.setZero();

            auto *epa = new EdgePriorAcc(bprior);
            epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
            double infoPriorA = priorA;  //
            epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
            optimizer.addEdge(epa);

            auto *epg = new EdgePriorGyro(bprior);
            epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
            double infoPriorG = priorG;  //
            epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
            optimizer.addEdge(epg);
        }

        const double thHuberMono = sqrt(5.991);
        const double thHuberStereo = sqrt(7.815);

        const unsigned long iniMPid = maxKFid * 5;

        vector<bool> vbNotIncludedMP(vpMPs.size(), false);

        for (size_t i = 0; i < vpMPs.size(); i++) {
            MapPoint *pMP = vpMPs[i];
            auto *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
            unsigned long id = pMP->mnId + iniMPid + 1;
            vPoint->setId((int) (id));
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

            bool bAllFixed = true;

            // Set edges
            for (const auto &observation: observations) {
                KeyFrame *pKFi = observation.first;

                if (pKFi->mnId > maxKFid) continue;

                if (!pKFi->isBad()) {
                    const int leftIndex = get<0>(observation.second);
                    cv::KeyPoint kpUn;
                    float kpSegVal, kpSegValRight;

                    if (leftIndex != -1 && pKFi->mvuRight[get<0>(observation.second)] <
                                           0)  // Monocular observation
                    {
                        kpUn = pKFi->mvKeysUn[leftIndex];
                        kpSegVal = pKFi->mvSegVal[leftIndex];
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        auto *e = new EdgeMono(0);

                        auto *VP = dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) (pKFi->mnId)));
                        if (bAllFixed)
                            if (!VP->fixed()) bAllFixed = false;

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) (id))));
                        e->setVertex(1, VP);
                        e->setMeasurement(obs);
                        const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        auto *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        // ARK
//          auto* ark = new g2o::RobustKernelAdaptive;
//          double alphaBA;
//          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//          e->setRobustKernel(ark);
//          if (kpSegVal == PEOPLE_COLOR) {
//            alphaBA = -10020.0f;
//          }
//          else {
//            alphaBA = 1.0f;
//          }
//          ark->setAlpha(alphaBA);

                        optimizer.addEdge(e);
                    } else if (leftIndex != -1 &&
                               pKFi->mvuRight[leftIndex] >= 0)  // stereo observation
                    {
                        kpUn = pKFi->mvKeysUn[leftIndex];
                        kpSegVal = pKFi->mvSegVal[leftIndex];
                        const float kp_ur = pKFi->mvuRight[leftIndex];
                        Eigen::Matrix<double, 3, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        auto *e = new EdgeStereo(0);

                        auto *VP = dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) (pKFi->mnId)));
                        if (bAllFixed)
                            if (!VP->fixed()) bAllFixed = false;

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) (id))));
                        e->setVertex(1, VP);
                        e->setMeasurement(obs);
                        const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                        e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                        auto *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        // ARK
//          auto* ark = new g2o::RobustKernelAdaptive;
//          double alphaBA;
//          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//          e->setRobustKernel(ark);
//          if (kpSegVal == PEOPLE_COLOR) {
//            alphaBA = -10020.0f;
//          }
//          else {
//            alphaBA = 1.0f;
//          }
//          ark->setAlpha(alphaBA);

                        optimizer.addEdge(e);
                    }

                    if (pKFi->mpCamera2) {  // Monocular right observation
                        int rightIndex = get<1>(observation.second);

                        if (rightIndex != -1 && rightIndex < pKFi->mvKeysRight.size()) {
                            rightIndex -= pKFi->NLeft;

                            Eigen::Matrix<double, 2, 1> obs;
                            kpUn = pKFi->mvKeysRight[rightIndex];
                            kpSegValRight = pKFi->mvSegVal[rightIndex];
                            obs << kpUn.pt.x, kpUn.pt.y;

                            auto *e = new EdgeMono(1);

                            auto *VP = dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                    optimizer.vertex((int) (pKFi->mnId)));
                            if (bAllFixed)
                                if (!VP->fixed()) bAllFixed = false;

                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                    optimizer.vertex((int) (id))));
                            e->setVertex(1, VP);
                            e->setMeasurement(obs);
                            const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                            auto *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(thHuberMono);

                            // ARK
//            auto* ark = new g2o::RobustKernelAdaptive;
//            double alphaBA;
//            cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//            e->setRobustKernel(ark);
//            if (kpSegValRight == PEOPLE_COLOR) {
//              alphaBA = -10020.0f;
//            }
//            else {
//              alphaBA = 1.0f;
//            }
//            ark->setAlpha(alphaBA);

                            optimizer.addEdge(e);
                        }
                    }
                }
            }

            if (bAllFixed) {
                optimizer.removeVertex(vPoint);
                vbNotIncludedMP[i] = true;
            }
        }

        if (pbStopFlag)
            if (*pbStopFlag) return;

        optimizer.initializeOptimization();
        optimizer.optimize(its);

        // Recover optimized data
        // Keyframes
        for (auto pKFi: vpKFs) {
            if (pKFi->mnId > maxKFid) continue;
            auto *VP = dynamic_cast<VertexPose *>(optimizer.vertex((int) (pKFi->mnId)));
            if (nLoopId == 0) {
                Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                                 VP->estimate().tcw[0].cast<float>());
                pKFi->SetPose(Tcw);
            } else {
                pKFi->mTcwGBA = Sophus::SE3f(VP->estimate().Rcw[0].cast<float>(),
                                             VP->estimate().tcw[0].cast<float>());
                pKFi->mnBAGlobalForKF = nLoopId;
            }
            if (pKFi->bImu) {
                auto *VV = dynamic_cast<VertexVelocity *>(
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 1)));
                if (nLoopId == 0) {
                    pKFi->SetVelocity(VV->estimate().cast<float>());
                } else {
                    pKFi->mVwbGBA = VV->estimate().cast<float>();
                }

                VertexGyroBias *VG;
                VertexAccBias *VA;
                if (!bInit) {
                    VG = dynamic_cast<VertexGyroBias *>(
                            optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 2)));
                    VA = dynamic_cast<VertexAccBias *>(
                            optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 3)));
                } else {
                    VG = dynamic_cast<VertexGyroBias *>(
                            optimizer.vertex((int) (4 * maxKFid + 2)));
                    VA = dynamic_cast<VertexAccBias *>(
                            optimizer.vertex((int) (4 * maxKFid + 3)));
                }

                Vector6d vb;
                vb << VG->estimate(), VA->estimate();
                IMU::Bias b((float) vb[3], (float) vb[4], (float) vb[5], (float) vb[0],
                            (float) vb[1], (float) vb[2]);
                if (nLoopId == 0) {
                    pKFi->SetNewBias(b);
                } else {
                    pKFi->mBiasGBA = b;
                }
            }
        }

        // Points
        for (size_t i = 0; i < vpMPs.size(); i++) {
            if (vbNotIncludedMP[i]) continue;

            MapPoint *pMP = vpMPs[i];
            auto *vPoint = dynamic_cast<g2o::VertexSBAPointXYZ *>(
                    optimizer.vertex((int) (pMP->mnId + iniMPid + 1)));

            if (nLoopId == 0) {
                pMP->SetWorldPos(vPoint->estimate().cast<float>());
                pMP->UpdateNormalAndDepth();
            } else {
                pMP->mPosGBA = vPoint->estimate().cast<float>();
                pMP->mnBAGlobalForKF = nLoopId;
            }
        }

        pMap->IncreaseChangeIndex();
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

        vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;
        vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody *> vpEdgesMono_FHR;
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

                            auto *e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

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
                        } else  // Stereo observation
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
                            e->fx = ORB_SLAM3::Frame::fx;
                            e->fy = ORB_SLAM3::Frame::fy;
                            e->cx = ORB_SLAM3::Frame::cx;
                            e->cy = ORB_SLAM3::Frame::cy;
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

                        if (i < pFrame->Nleft) {  // Left camera observation
                            kpUn = pFrame->mvKeys[i];
                            kpSegVal = pFrame->mvSegVal[i];

                            pFrame->mvbOutlier[i] = false;

                            Eigen::Matrix<double, 2, 1> obs;
                            obs << kpUn.pt.x, kpUn.pt.y;

                            auto *e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

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

                            auto *e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody();

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

        if (nInitialCorrespondences < 3) return 0;

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
                ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];

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

                if (it == 2) e->setRobustKernel(nullptr);
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
                ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody *e = vpEdgesMono_FHR[i];

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

                if (it == 2) e->setRobustKernel(nullptr);
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

                if (it == 2) e->setRobustKernel(nullptr);
            }
            //        });

            if (optimizer.edges().size() < 10) break;
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
        for (auto pKFi: vNeighKFs) {
            pKFi->mnBALocalForKF = pKF->mnId;
            if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                lLocalKeyFrames.push_back(pKFi);
        }

        // Local MapPoints seen in Local KeyFrames
        num_fixedKF = 0;
        list<MapPoint *> lLocalMapPoints;
        set<MapPoint *> sNumObsMP;
        for (auto pKFi: lLocalKeyFrames) {
            if (pKFi->mnId == pMap->GetInitKFid()) {
                num_fixedKF = 1;
            }
            vector<MapPoint *> vpMPs = pKFi->GetMapPointMatches();
            for (auto pMP: vpMPs) {
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
        for (auto &lLocalMapPoint: lLocalMapPoints) {
            map<KeyFrame *, tuple<int, int>> observations =
                    lLocalMapPoint->GetObservations();
            for (auto &observation: observations) {
                KeyFrame *pKFi = observation.first;

                if (pKFi->mnBALocalForKF != pKF->mnId &&
                    pKFi->mnBAFixedForKF != pKF->mnId) {
                    pKFi->mnBAFixedForKF = pKF->mnId;
                    if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                        lFixedCameras.push_back(pKFi);
                }
            }
        }
        num_fixedKF = (int) (lFixedCameras.size() + num_fixedKF);

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
        if (pMap->IsInertial()) solver->setUserLambdaInit(100.0);

        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

        unsigned long maxKFid = 0;

        // DEBUG LBA
        pCurrentMap->msOptKFs.clear();
        pCurrentMap->msFixedKFs.clear();

        // Set Local KeyFrame vertices
        for (auto pKFi: lLocalKeyFrames) {
            auto *vSE3 = new g2o::VertexSE3Expmap();
            Sophus::SE3<float> Tcw = pKFi->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                           Tcw.translation().cast<double>()));
            vSE3->setId((int) (pKFi->mnId));
            vSE3->setFixed(pKFi->mnId == pMap->GetInitKFid());
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
            // DEBUG LBA
            pCurrentMap->msOptKFs.insert(pKFi->mnId);
        }
        num_OptKF = (int) (lLocalKeyFrames.size());

        // Set Fixed KeyFrame vertices
        for (auto pKFi: lFixedCameras) {
            auto *vSE3 = new g2o::VertexSE3Expmap();
            Sophus::SE3<float> Tcw = pKFi->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                           Tcw.translation().cast<double>()));
            vSE3->setId((int) (pKFi->mnId));
            vSE3->setFixed(true);
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
            // DEBUG LBA
            pCurrentMap->msFixedKFs.insert(pKFi->mnId);
        }

        // Set MapPoint vertices
        const size_t nExpectedSize =
                (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

        vector<ORB_SLAM3::EdgeSE3ProjectXYZ *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody *> vpEdgesBody;
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

        for (auto pMP: lLocalMapPoints) {
            auto *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
            int id = (int) (pMP->mnId + maxKFid + 1);

            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);
            nPoints++;

            const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

            // Set edges
            for (const auto &observation: observations) {
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

                        auto *e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) (pKFi->mnId))));
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
                               0)  // Stereo observation
                    {
                        const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex]; // This one?
                        const float &kpSegVal = pKFi->mvSegVal[leftIndex];
                        Eigen::Matrix<double, 3, 1> obs;
                        const float kp_ur = pKFi->mvuRight[get<0>(observation.second)]; // How to find the left point?
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        auto *e = new g2o::EdgeStereoSE3ProjectXYZ();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) (pKFi->mnId))));
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

//          const cv::KeyPoint& kpUn = pKFi->mvKeysUn[leftIndex]; // This one?
//          const cv::Vec3b& kpSegVal = pKFi->mvSegVal[leftIndex];

                        if (rightIndex != -1) {
                            rightIndex -= pKFi->NLeft;

                            Eigen::Matrix<double, 2, 1> obs;
                            cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                            float kpSegValRight = pKFi->mvSegVal[rightIndex];
                            obs << kp.pt.x, kp.pt.y;

                            auto *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                    optimizer.vertex(id)));
                            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                    optimizer.vertex((int) (pKFi->mnId))));
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
            if (*pbStopFlag) return;

        optimizer.initializeOptimization();
        optimizer.optimize(10);

        vector<pair<KeyFrame *, MapPoint *>> vToErase;
        vToErase.reserve(vpEdgesMono.size() + vpEdgesBody.size() +
                         vpEdgesStereo.size());

        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad()) continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                KeyFrame *pKFi = vpEdgeKFMono[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++) {
            ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = vpEdgesBody[i];
            MapPoint *pMP = vpMapPointEdgeBody[i];

            if (pMP->isBad()) continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                KeyFrame *pKFi = vpEdgeKFBody[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad()) continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive()) {
                KeyFrame *pKFi = vpEdgeKFStereo[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        if (!vToErase.empty()) {
            for (auto &i: vToErase) {
                KeyFrame *pKFi = i.first;
                MapPoint *pMPi = i.second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
            }
        }

        // Recover optimized data
        // Keyframes
        for (auto pKFi: lLocalKeyFrames) {
            auto *vSE3 = dynamic_cast<g2o::VertexSE3Expmap *>(
                    optimizer.vertex((int) (pKFi->mnId)));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                             SE3quat.translation().cast<float>());
            pKFi->SetPose(Tiw);
        }

        // Points
        for (auto pMP: lLocalMapPoints) {
            auto *vPoint = dynamic_cast<g2o::VertexSBAPointXYZ *>(
                    optimizer.vertex((int) (pMP->mnId + maxKFid + 1)));
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

        vector<Eigen::Vector3d> vZvectors(nMaxKFid + 1);  // For debugging
        Eigen::Vector3d z_vec;
        z_vec << 0.0, 0.0, 1.0;

        const int minFeat = 100;

        // Set KeyFrame vertices
        for (auto pKF: vpKFs) {
            if (pKF->isBad()) continue;
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

            if (pKF->mnId == pMap->GetInitKFid()) VSim3->setFixed(true);

            VSim3->setId((int) (nIDi));
            VSim3->setMarginalized(false);
            VSim3->_fix_scale = bFixScale;

            optimizer.addVertex(VSim3);
            vZvectors[nIDi] = vScw[nIDi].rotation() * z_vec;  // For debugging

            vpVertices[nIDi] = VSim3;
        }

        set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

        const Eigen::Matrix<double, 7, 7> matLambda =
                Eigen::Matrix<double, 7, 7>::Identity();

        // Set Loop edges
        int count_loop = 0;
        for (const auto &LoopConnection: LoopConnections) {
            KeyFrame *pKF = LoopConnection.first;
            const long unsigned int nIDi = pKF->mnId;
            const set<KeyFrame *> &spConnections = LoopConnection.second;
            const g2o::Sim3 Siw = vScw[nIDi];
            const g2o::Sim3 Swi = Siw.inverse();

            for (auto spConnection: spConnections) {
                const long unsigned int nIDj = spConnection->mnId;
                if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
                    pKF->GetWeight(spConnection) < minFeat)
                    continue;

                const g2o::Sim3 Sjw = vScw[nIDj];
                const g2o::Sim3 Sji = Sjw * Swi;

                auto *e = new g2o::EdgeSim3();
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                        optimizer.vertex((int) nIDj)));
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                        optimizer.vertex((int) nIDi)));
                e->setMeasurement(Sji);

                e->information() = matLambda;

                optimizer.addEdge(e);
                count_loop++;
                sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
            }
        }

        // Set normal edges
        for (auto pKF: vpKFs) {
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
                        optimizer.vertex((int) nIDj)));
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                        optimizer.vertex((int) nIDi)));
                e->setMeasurement(Sji);
                e->information() = matLambda;
                optimizer.addEdge(e);
            }

            // Loop edges
            const set<KeyFrame *> sLoopEdges = pKF->GetLoopEdges();
            for (auto pLKF: sLoopEdges) {
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
                            optimizer.vertex((int) pLKF->mnId)));
                    el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int) nIDi)));
                    el->setMeasurement(Sli);
                    el->information() = matLambda;
                    optimizer.addEdge(el);
                }
            }

            // Covisibility graph edges
            const vector<KeyFrame *> vpConnectedKFs =
                    pKF->GetCovisiblesByWeight(minFeat);
            for (auto pKFn: vpConnectedKFs) {
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
                                optimizer.vertex((int) pKFn->mnId)));
                        en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) nIDi)));
                        en->setMeasurement(Sni);
                        en->information() = matLambda;
                        optimizer.addEdge(en);
                    }
                }
            }

            // Inertial edges if inertial
            if (pKF->bImu && pKF->mPrevKF) {
                g2o::Sim3 Spw;
                auto itp = NonCorrectedSim3.find(pKF->mPrevKF);
                if (itp != NonCorrectedSim3.end())
                    Spw = itp->second;
                else
                    Spw = vScw[pKF->mPrevKF->mnId];

                g2o::Sim3 Spi = Spw * Swi;
                auto *ep = new g2o::EdgeSim3();
                ep->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                        optimizer.vertex((int) pKF->mPrevKF->mnId)));
                ep->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                        optimizer.vertex((int) nIDi)));
                ep->setMeasurement(Spi);
                ep->information() = matLambda;
                optimizer.addEdge(ep);
            }
        }

        optimizer.initializeOptimization();
        optimizer.computeActiveErrors();
        optimizer.optimize(20);
        optimizer.computeActiveErrors();
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
        for (auto pKFi: vpKFs) {
            const size_t nIDi = pKFi->mnId;

            auto *VSim3 =
                    dynamic_cast<g2o::VertexSim3Expmap *>(optimizer.vertex((int) nIDi));
            g2o::Sim3 CorrectedSiw = VSim3->estimate();
            vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
            double s = CorrectedSiw.scale();

            Sophus::SE3f Tiw(CorrectedSiw.rotation().cast<float>(),
                             CorrectedSiw.translation().cast<float>() / s);
            pKFi->SetPose(Tiw);
        }

        // Correct points. Transform to "non-optimized" reference keyframe pose and
        // transform back with optimized pose
        for (auto pMP: vpMPs) {
            if (pMP->isBad()) continue;

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

        for (KeyFrame *pKFi: vpFixedKFs) {
            if (pKFi->isBad()) continue;

            auto *VSim3 = new g2o::VertexSim3Expmap();

            const size_t nIDi = pKFi->mnId;

            Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
            g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

            vCorrectedSwc[nIDi] = Siw.inverse();
            VSim3->setEstimate(Siw);

            VSim3->setFixed(true);

            VSim3->setId((int) nIDi);
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
        for (KeyFrame *pKFi: vpFixedCorrectedKFs) {
            if (pKFi->isBad()) continue;

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

            VSim3->setId((int) nIDi);
            VSim3->setMarginalized(false);

            optimizer.addVertex(VSim3);

            vpVertices[nIDi] = VSim3;

            sIdKF.insert(nIDi);

            vpGoodPose[nIDi] = true;
            vpBadPose[nIDi] = true;
        }

        for (KeyFrame *pKFi: vpNonFixedKFs) {
            if (pKFi->isBad()) continue;

            const size_t nIDi = pKFi->mnId;

            if (sIdKF.count(nIDi))  // It has already added in the corrected merge KFs
                continue;

            auto *VSim3 = new g2o::VertexSim3Expmap();

            Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
            g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);

            VSim3->setFixed(false);

            VSim3->setId((int) nIDi);
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

        for (KeyFrame *pKFi: vpKFs) {
            int num_connections = 0;
            const size_t nIDi = pKFi->mnId;

            g2o::Sim3 correctedSwi;
            g2o::Sim3 Swi;

            if (vpGoodPose[nIDi]) correctedSwi = vCorrectedSwc[nIDi];
            if (vpBadPose[nIDi]) Swi = vScw[nIDi].inverse();

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
                            optimizer.vertex((int) nIDj)));
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int) nIDi)));
                    e->setMeasurement(Sji);

                    e->information() = matLambda;
                    optimizer.addEdge(e);
                    num_connections++;
                }
            }

            // Loop edges
            const set<KeyFrame *> sLoopEdges = pKFi->GetLoopEdges();
            for (auto pLKF: sLoopEdges) {
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
                                optimizer.vertex((int) pLKF->mnId)));
                        el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) nIDi)));
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
            for (auto pKFn: vpConnectedKFs) {
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
                                    optimizer.vertex((int) pKFn->mnId)));
                            en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                    optimizer.vertex((int) nIDi)));
                            en->setMeasurement(Sni);
                            en->information() = matLambda;
                            optimizer.addEdge(en);
                            num_connections++;
                        }
                    }
                }
            }

            if (num_connections == 0) {
                Verbose::PrintMess(
                        "Opt_Essential: KF " + to_string(pKFi->mnId) + " has 0 connections",
                        Verbose::VERBOSITY_DEBUG);
            }
        }

        // Optimize!
        optimizer.initializeOptimization();
        optimizer.optimize(20);

        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
        for (KeyFrame *pKFi: vpNonFixedKFs) {
            if (pKFi->isBad()) continue;

            const size_t nIDi = pKFi->mnId;

            auto *VSim3 =
                    dynamic_cast<g2o::VertexSim3Expmap *>(optimizer.vertex((int) nIDi));
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
        for (MapPoint *pMPi: vpNonCorrectedMPs) {
            if (pMPi->isBad()) continue;

            KeyFrame *pRefKF = pMPi->GetReferenceKeyFrame();
            while (pRefKF->isBad()) {
                if (!pRefKF) {
                    Verbose::PrintMess(
                            "MP " + to_string(pMPi->mnId) + " without a valid reference KF",
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
        auto *vSim3 = new ORB_SLAM3::VertexSim3Expmap();
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
        vector<ORB_SLAM3::EdgeSim3ProjectXYZ *> vpEdges12;
        vector<ORB_SLAM3::EdgeInverseSim3ProjectXYZ *> vpEdges21;
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
            if (!vpMatches1[i]) continue;

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

            auto *e12 = new ORB_SLAM3::EdgeSim3ProjectXYZ();

            e12->setVertex(
                    0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id2)));
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
                kpUn2 = cv::KeyPoint(cv::Point2f(x, y), (float) pMP2->mnTrackScaleLevel);

                inKF2 = false;
                nOutKF2++;
            }

            auto *e21 = new ORB_SLAM3::EdgeInverseSim3ProjectXYZ();

            e21->setVertex(
                    0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
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
            ORB_SLAM3::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            ORB_SLAM3::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if (!e12 || !e21) continue;

            if (e12->chi2() > th2 || e21->chi2() > th2) {
                size_t idx = vnIndexEdge[i];
                vpMatches1[idx] = static_cast<MapPoint *>(nullptr);
                optimizer.removeEdge(e12);
                optimizer.removeEdge(e21);
                vpEdges12[i] = static_cast<ORB_SLAM3::EdgeSim3ProjectXYZ *>(nullptr);
                vpEdges21[i] =
                        static_cast<ORB_SLAM3::EdgeInverseSim3ProjectXYZ *>(nullptr);
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

        if (nCorrespondences - nBad < 10) return 0;

        // Optimize again only with inliers
        optimizer.initializeOptimization();
        optimizer.optimize(nMoreIterations);

        int nIn = 0;
        mAcumHessian = Eigen::MatrixXd::Zero(7, 7);
        for (size_t i = 0; i < vpEdges12.size(); i++) {
            ORB_SLAM3::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            ORB_SLAM3::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if (!e12 || !e21) continue;

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
        auto *vSim3_recov = dynamic_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
        g2oS12 = vSim3_recov->estimate();

        return nIn;
    }

    void Optimizer::LocalInertialBA(KeyFrame *pKF, bool *pbStopFlag, Map *pMap,
                                    int &num_fixedKF, int &num_OptKF, int &num_MPs,
                                    int &num_edges, bool bLarge, bool bRecInit) {
        Map *pCurrentMap = pKF->GetMap();

        int maxOpt = 10;
        int opt_it = 10;
        if (bLarge) {
            maxOpt = 25;
            opt_it = 4;
        }
        const int Nd = std::min((int) pCurrentMap->KeyFramesInMap() - 2, maxOpt);
        const unsigned long maxKFid = pKF->mnId;

        vector<KeyFrame *> vpOptimizableKFs;
        const vector<KeyFrame *> vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
        list<KeyFrame *> lpOptVisKFs;

        vpOptimizableKFs.reserve(Nd);
        vpOptimizableKFs.push_back(pKF);
        pKF->mnBALocalForKF = pKF->mnId;
        for (int i = 1; i < Nd; i++) {
            if (vpOptimizableKFs.back()->mPrevKF) {
                vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
                vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
            } else
                break;
        }

        size_t N = vpOptimizableKFs.size();

        // Optimizable points seen by temporal optimizable keyframes
        list<MapPoint *> lLocalMapPoints;
        for (size_t i = 0; i < N; i++) {
            vector<MapPoint *> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
            for (auto pMP: vpMPs) {
                if (pMP)
                    if (!pMP->isBad())
                        if (pMP->mnBALocalForKF != pKF->mnId) {
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF = pKF->mnId;
                        }
            }
        }

        // Fixed Keyframe: First frame previous KF to optimization window)
        list<KeyFrame *> lFixedKeyFrames;
        if (vpOptimizableKFs.back()->mPrevKF) {
            lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
        } else {
            vpOptimizableKFs.back()->mnBALocalForKF = 0;
            vpOptimizableKFs.back()->mnBAFixedForKF = pKF->mnId;
            lFixedKeyFrames.push_back(vpOptimizableKFs.back());
            vpOptimizableKFs.pop_back();
        }

        // Optimizable visual KFs
        const int maxCovKF = 0;
        for (auto pKFi: vpNeighsKFs) {
            if (lpOptVisKFs.size() >= maxCovKF) break;

            if (pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId)
                continue;
            pKFi->mnBALocalForKF = pKF->mnId;
            if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
                lpOptVisKFs.push_back(pKFi);

                vector<MapPoint *> vpMPs = pKFi->GetMapPointMatches();
                for (auto pMP: vpMPs) {
                    if (pMP)
                        if (!pMP->isBad())
                            if (pMP->mnBALocalForKF != pKF->mnId) {
                                lLocalMapPoints.push_back(pMP);
                                pMP->mnBALocalForKF = pKF->mnId;
                            }
                }
            }
        }

        // Fixed KFs which are not covisible optimizable
        const int maxFixKF = 200;

        for (auto &lLocalMapPoint: lLocalMapPoints) {
            map<KeyFrame *, tuple<int, int>> observations =
                    lLocalMapPoint->GetObservations();
            for (auto &observation: observations) {
                KeyFrame *pKFi = observation.first;

                if (pKFi->mnBALocalForKF != pKF->mnId &&
                    pKFi->mnBAFixedForKF != pKF->mnId) {
                    pKFi->mnBAFixedForKF = pKF->mnId;
                    if (!pKFi->isBad()) {
                        lFixedKeyFrames.push_back(pKFi);
                        break;
                    }
                }
            }
            if (lFixedKeyFrames.size() >= maxFixKF) break;
        }

        bool bNonFixed = (lFixedKeyFrames.empty());

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;
        linearSolver =
                new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        auto *solver_ptr = new g2o::BlockSolverX(linearSolver);

        if (bLarge) {
            auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
            solver->setUserLambdaInit(
                    1e-2);  // to avoid iterating for finding optimal lambda
            optimizer.setAlgorithm(solver);
        } else {
            auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
            solver->setUserLambdaInit(1e0);
            optimizer.setAlgorithm(solver);
        }

        // Set Local temporal KeyFrame vertices
        N = vpOptimizableKFs.size();
        for (int i = 0; i < N; i++) {
            KeyFrame *pKFi = vpOptimizableKFs[i];

            auto *VP = new VertexPose(pKFi);
            VP->setId((int) (pKFi->mnId));
            VP->setFixed(false);
            optimizer.addVertex(VP);

            if (pKFi->bImu) {
                auto *VV = new VertexVelocity(pKFi);
                VV->setId((int) (maxKFid + 3 * (pKFi->mnId) + 1));
                VV->setFixed(false);
                optimizer.addVertex(VV);
                auto *VG = new VertexGyroBias(pKFi);
                VG->setId((int) (maxKFid + 3 * (pKFi->mnId) + 2));
                VG->setFixed(false);
                optimizer.addVertex(VG);
                auto *VA = new VertexAccBias(pKFi);
                VA->setId((int) (maxKFid + 3 * (pKFi->mnId) + 3));
                VA->setFixed(false);
                optimizer.addVertex(VA);
            }
        }

        // Set Local visual KeyFrame vertices
        for (auto pKFi: lpOptVisKFs) {
            auto *VP = new VertexPose(pKFi);
            VP->setId((int) (pKFi->mnId));
            VP->setFixed(false);
            optimizer.addVertex(VP);
        }

        // Set Fixed KeyFrame vertices
        for (auto pKFi: lFixedKeyFrames) {
            auto *VP = new VertexPose(pKFi);
            VP->setId((int) pKFi->mnId);
            VP->setFixed(true);
            optimizer.addVertex(VP);

            if (pKFi->bImu)  // This should be done only for keyframe just before
                // temporal window
            {
                auto *VV = new VertexVelocity(pKFi);
                VV->setId((int) (maxKFid + 3 * (pKFi->mnId) + 1));
                VV->setFixed(true);
                optimizer.addVertex(VV);
                auto *VG = new VertexGyroBias(pKFi);
                VG->setId((int) (maxKFid + 3 * (pKFi->mnId) + 2));
                VG->setFixed(true);
                optimizer.addVertex(VG);
                auto *VA = new VertexAccBias(pKFi);
                VA->setId((int) (maxKFid + 3 * (pKFi->mnId) + 3));
                VA->setFixed(true);
                optimizer.addVertex(VA);
            }
        }

        // Create intertial constraints
        vector<EdgeInertial *> vei(N, (EdgeInertial *) nullptr);
        vector<EdgeGyroRW *> vegr(N, (EdgeGyroRW *) nullptr);
        vector<EdgeAccRW *> vear(N, (EdgeAccRW *) nullptr);
        vector<EdgeWOdometry *> veo(N, (EdgeWOdometry *) nullptr);

        for (int i = 0; i < N; i++) {
            KeyFrame *pKFi = vpOptimizableKFs[i];

            if (!pKFi->mPrevKF) {
                cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << endl;
                continue;
            }
            if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
                pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
                g2o::HyperGraph::Vertex *VP1 =
                        optimizer.vertex((int) (pKFi->mPrevKF->mnId));
                g2o::HyperGraph::Vertex *VV1 =
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1));
                g2o::HyperGraph::Vertex *VG1 =
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2));
                g2o::HyperGraph::Vertex *VA1 =
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3));
                g2o::HyperGraph::Vertex *VP2 = optimizer.vertex((int) (pKFi->mnId));
                g2o::HyperGraph::Vertex *VV2 =
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 1));
                g2o::HyperGraph::Vertex *VG2 =
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 2));
                g2o::HyperGraph::Vertex *VA2 =
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 3));

                if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
                    cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
                         << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2
                         << endl;
                    continue;
                }

                vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

                vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
                vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG1));
                vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA1));
                vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));

                if (i == N - 1 || bRecInit) {
                    // All inertial residuals are included without robust cost function, but
                    // not that one linking the last optimizable keyframe inside of the
                    // local window and the first fixed keyframe out. The information matrix
                    // for this measurement is also downweighted. This is done to avoid
                    // accumulating error due to fixing variables.
                    auto *rki = new g2o::RobustKernelHuber;
                    vei[i]->setRobustKernel(rki);
                    if (i == N - 1) vei[i]->setInformation(vei[i]->information() * 1e-2);
                    rki->setDelta(sqrt(16.92));
                }
                optimizer.addEdge(vei[i]);

                vegr[i] = new EdgeGyroRW();
                vegr[i]->setVertex(0, VG1);
                vegr[i]->setVertex(1, VG2);
                Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                        .cast<double>()
                        .inverse();
                vegr[i]->setInformation(InfoG);
                optimizer.addEdge(vegr[i]);

                vear[i] = new EdgeAccRW();
                vear[i]->setVertex(0, VA1);
                vear[i]->setVertex(1, VA2);
                Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                        .cast<double>()
                        .inverse();
                vear[i]->setInformation(InfoA);

                if (pKFi->mpOdomPreintegrated) {
                    optimizer.addEdge(vear[i]);

                    veo[i] = new EdgeWOdometry(pKFi->mpOdomPreintegrated);
                    veo[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                    veo[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                    optimizer.addEdge(veo[i]);
                }
            } else
                cout << "ERROR building inertial edge" << endl;
        }

        // Set MapPoint vertices
        const size_t nExpectedSize =
                (N + lFixedKeyFrames.size()) * lLocalMapPoints.size();

        // Mono
        vector<EdgeMono *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        // Stereo
        vector<EdgeStereo *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);

        const double thHuberMono = sqrt(5.991);
        const float chi2Mono2 = 5.991;
        const double thHuberStereo = sqrt(7.815);
        const float chi2Stereo2 = 7.815;

        const unsigned long iniMPid = maxKFid * 5;

        map<int, int> mVisEdges;
        for (int i = 0; i < N; i++) {
            KeyFrame *pKFi = vpOptimizableKFs[i];
            mVisEdges[(int) pKFi->mnId] = 0;
        }
        for (auto &lFixedKeyFrame: lFixedKeyFrames) {
            mVisEdges[(int) lFixedKeyFrame->mnId] = 0;
        }

        for (auto pMP: lLocalMapPoints) {
            auto *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

            unsigned long id = pMP->mnId + iniMPid + 1;
            vPoint->setId((int) id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);
            const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

            // Create visual constraints
            for (const auto &observation: observations) {
                KeyFrame *pKFi = observation.first;

                if (pKFi->mnBALocalForKF != pKF->mnId &&
                    pKFi->mnBAFixedForKF != pKF->mnId)
                    continue;

                if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
                    const int leftIndex = get<0>(observation.second);

                    cv::KeyPoint kpUn;
                    float kpSegVal;

                    // Monocular left observation
                    if (leftIndex != -1 && pKFi->mvuRight[leftIndex] < 0) {
                        mVisEdges[(int) pKFi->mnId]++;

                        kpUn = pKFi->mvKeysUn[leftIndex];
                        kpSegVal = pKFi->mvSegVal[leftIndex];
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        auto *e = new EdgeMono(0);

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) pKFi->mnId)));
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        auto *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        // ARK
//          auto* ark = new g2o::RobustKernelAdaptive;
//          double alphaBA;
//          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//          e->setRobustKernel(ark);
//          if (kpSegVal == PEOPLE_COLOR) {
//            alphaBA = -10020.0f;
//          }
//          else {
//            alphaBA = 1.0f;
//          }
//          ark->setAlpha(alphaBA);

                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);
                    }
                        // Stereo-observation
                    else if (leftIndex != -1)  // Stereo observation
                    {
                        kpUn = pKFi->mvKeysUn[leftIndex];
                        kpSegVal = pKFi->mvSegVal[leftIndex];
                        mVisEdges[(int) pKFi->mnId]++;

                        const float kp_ur = pKFi->mvuRight[leftIndex];
                        Eigen::Matrix<double, 3, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        auto *e = new EdgeStereo(0);

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) pKFi->mnId)));
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pKFi->mpCamera->uncertainty2(obs.head(2));

                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                        auto *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        // ARK
//          auto* ark = new g2o::RobustKernelAdaptive;
//          double alphaBA;
//          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//          e->setRobustKernel(ark);
//          if (kpSegVal == PEOPLE_COLOR) {
//            alphaBA = -10020.0f;
//          }
//          else {
//            alphaBA = 1.0f;
//          }
//          ark->setAlpha(alphaBA);

                        optimizer.addEdge(e);
                        vpEdgesStereo.push_back(e);
                        vpEdgeKFStereo.push_back(pKFi);
                        vpMapPointEdgeStereo.push_back(pMP);
                    }

                    // Monocular right observation
                    if (pKFi->mpCamera2) {
                        int rightIndex = get<1>(observation.second);

                        if (rightIndex != -1) {
                            rightIndex -= pKFi->NLeft;
                            mVisEdges[(int) pKFi->mnId]++;

                            Eigen::Matrix<double, 2, 1> obs;
                            cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                            float kpSegValRight = pKFi->mvSegVal[rightIndex];
                            obs << kp.pt.x, kp.pt.y;

                            auto *e = new EdgeMono(1);

                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                    optimizer.vertex((int) id)));
                            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                    optimizer.vertex((int) pKFi->mnId)));
                            e->setMeasurement(obs);

                            // Add here uncerteinty
                            const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
                            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                            auto *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(thHuberMono);

                            // ARK
//            auto* ark = new g2o::RobustKernelAdaptive;
//            double alphaBA;
//            cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//            e->setRobustKernel(ark);
//            if (kpSegValRight == PEOPLE_COLOR) {
//              alphaBA = -10020.0f;
//            }
//            else {
//              alphaBA = 1.0f;
//            }
//            ark->setAlpha(alphaBA);

                            optimizer.addEdge(e);
                            vpEdgesMono.push_back(e);
                            vpEdgeKFMono.push_back(pKFi);
                            vpMapPointEdgeMono.push_back(pMP);
                        }
                    }
                }
            }
        }

        // cout << "Total map points: " << lLocalMapPoints.size() << endl;
        for (auto mit = mVisEdges.begin(), mend = mVisEdges.end(); mit != mend;
             mit++) {
            //        std::cout << "Why we get inside this assert?" << std::endl;
            //        assert(mit->second>=3);
        }

        optimizer.initializeOptimization();
        optimizer.computeActiveErrors();
        double err = optimizer.activeRobustChi2();
        optimizer.optimize(opt_it);  // Originally to 2
        double err_end = optimizer.activeRobustChi2();
        if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

        vector<pair<KeyFrame *, MapPoint *>> vToErase;
        vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

        // Check inlier observations
        // Mono
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            EdgeMono *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];
            bool bClose = pMP->mTrackDepth < 10.f;

            if (pMP->isBad()) continue;

            if ((e->chi2() > chi2Mono2 && !bClose) ||
                (e->chi2() > 1.5f * chi2Mono2 && bClose) || !e->isDepthPositive()) {
                KeyFrame *pKFi = vpEdgeKFMono[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // Stereo
        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
            EdgeStereo *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad()) continue;

            if (e->chi2() > chi2Stereo2) {
                KeyFrame *pKFi = vpEdgeKFStereo[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // Get Map Mutex and erase outliers
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        // TODO: Some convergence problems have been detected here
        if ((2 * err < err_end || isnan(err) || isnan(err_end)) && !bLarge)  // bGN)
        {
            cout << "FAIL LOCAL-INERTIAL BA!!!!" << endl;
            return;
        }

        if (!vToErase.empty()) {
            for (auto &i: vToErase) {
                KeyFrame *pKFi = i.first;
                MapPoint *pMPi = i.second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
            }
        }

        for (auto &lFixedKeyFrame: lFixedKeyFrames)
            lFixedKeyFrame->mnBAFixedForKF = 0;

        // Recover optimized data
        // Local temporal Keyframes
        N = vpOptimizableKFs.size();
        for (int i = 0; i < N; i++) {
            KeyFrame *pKFi = vpOptimizableKFs[i];

            auto *VP = dynamic_cast<VertexPose *>(optimizer.vertex((int) pKFi->mnId));
            Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                             VP->estimate().tcw[0].cast<float>());
            pKFi->SetPose(Tcw);
            pKFi->mnBALocalForKF = 0;

            if (pKFi->bImu) {
                auto *VV = dynamic_cast<VertexVelocity *>(
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 1)));
                pKFi->SetVelocity(VV->estimate().cast<float>());
                auto *VG = dynamic_cast<VertexGyroBias *>(
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 2)));
                auto *VA = dynamic_cast<VertexAccBias *>(
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 3)));
                Vector6d b;
                b << VG->estimate(), VA->estimate();
                pKFi->SetNewBias(IMU::Bias((float) b[3], (float) b[4], (float) b[5],
                                           (float) b[0], (float) b[1], (float) b[2]));
            }
        }

        // Local visual KeyFrame
        for (auto pKFi: lpOptVisKFs) {
            auto *VP = dynamic_cast<VertexPose *>(optimizer.vertex((int) pKFi->mnId));
            Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                             VP->estimate().tcw[0].cast<float>());
            pKFi->SetPose(Tcw);
            pKFi->mnBALocalForKF = 0;
        }

        // Points
        for (auto pMP: lLocalMapPoints) {
            auto *vPoint = dynamic_cast<g2o::VertexSBAPointXYZ *>(
                    optimizer.vertex((int) (pMP->mnId + iniMPid + 1)));
            pMP->SetWorldPos(vPoint->estimate().cast<float>());
            pMP->UpdateNormalAndDepth();
        }

        pMap->IncreaseChangeIndex();
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

    void Optimizer::InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg,
                                         double &scale, Eigen::Vector3d &bg,
                                         Eigen::Vector3d &ba, bool bMono,
                                         Eigen::MatrixXd &covInertial,
                                         bool bFixedVel, bool bGauss, float priorG,
                                         float priorA) {
        Verbose::PrintMess("inertial optimization", Verbose::VERBOSITY_NORMAL);
        int its = 200;
        long unsigned int maxKFid = pMap->GetMaxKFid();
        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver =
                new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        auto *solver_ptr = new g2o::BlockSolverX(linearSolver);

        auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

        if (priorG != 0.f) solver->setUserLambdaInit(1e3);

        optimizer.setAlgorithm(solver);

        // Set KeyFrame vertices (fixed poses and optimizable velocities)
        for (auto pKFi: vpKFs) {
            if (pKFi->mnId > maxKFid) continue;
            auto *VP = new VertexPose(pKFi);
            VP->setId((int) (pKFi->mnId));
            VP->setFixed(true);
            optimizer.addVertex(VP);

            auto *VV = new VertexVelocity(pKFi);
            VV->setId((int) (maxKFid + (pKFi->mnId) + 1));
            if (bFixedVel)
                VV->setFixed(true);
            else
                VV->setFixed(false);

            optimizer.addVertex(VV);
        }

        // Biases
        auto *VG = new VertexGyroBias(vpKFs.front());
        VG->setId((int) (maxKFid * 2 + 2));
        if (bFixedVel)
            VG->setFixed(true);
        else
            VG->setFixed(false);
        optimizer.addVertex(VG);
        auto *VA = new VertexAccBias(vpKFs.front());
        VA->setId((int) (maxKFid * 2 + 3));
        if (bFixedVel)
            VA->setFixed(true);
        else
            VA->setFixed(false);

        optimizer.addVertex(VA);
        // prior acc bias
        Eigen::Vector3f bprior;
        bprior.setZero();

        auto *epa = new EdgePriorAcc(bprior);
        epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
        double infoPriorA = priorA;
        epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
        optimizer.addEdge(epa);
        auto *epg = new EdgePriorGyro(bprior);
        epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
        double infoPriorG = priorG;
        epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
        optimizer.addEdge(epg);

        // Gravity and scale
        auto *VGDir = new VertexGDir(Rwg);
        VGDir->setId((int) (maxKFid * 2 + 4));
        VGDir->setFixed(false);
        optimizer.addVertex(VGDir);
        auto *VS = new VertexScale(scale);
        VS->setId((int) (maxKFid * 2 + 5));
        VS->setFixed(!bMono);  // Fixed for stereo case
        optimizer.addVertex(VS);

        // Graph edges
        // IMU links with gravity and scale
        vector<EdgeInertialGS *> vpei;
        vpei.reserve(vpKFs.size());
        vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
        vppUsedKF.reserve(vpKFs.size());
        // std::cout << "build optimization graph" << std::endl;

        for (auto pKFi: vpKFs) {
            if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
                if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;
                if (!pKFi->mpImuPreintegrated)
                    std::cout << "Not preintegrated measurement" << std::endl;

                pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
                g2o::HyperGraph::Vertex *VP1 =
                        optimizer.vertex((int) (pKFi->mPrevKF->mnId));
                g2o::HyperGraph::Vertex *VV1 =
                        optimizer.vertex((int) (maxKFid + (pKFi->mPrevKF->mnId) + 1));
                g2o::HyperGraph::Vertex *VP2 = optimizer.vertex((int) (pKFi->mnId));
                g2o::HyperGraph::Vertex *VV2 =
                        optimizer.vertex((int) (maxKFid + (pKFi->mnId) + 1));
                g2o::HyperGraph::Vertex *VGk = optimizer.vertex((int) (maxKFid * 2 + 2));
                g2o::HyperGraph::Vertex *VAk = optimizer.vertex((int) (maxKFid * 2 + 3));
                g2o::HyperGraph::Vertex *VGDirk =
                        optimizer.vertex((int) (maxKFid * 2 + 4));
                g2o::HyperGraph::Vertex *VSk = optimizer.vertex((int) (maxKFid * 2 + 5));
                if (!VP1 || !VV1 || !VGk || !VAk || !VP2 || !VV2 || !VGDirk || !VSk) {
                    cout << "Error" << VP1 << ", " << VV1 << ", " << VGk << ", " << VAk
                         << ", " << VP2 << ", " << VV2 << ", " << VGDirk << ", " << VSk
                         << endl;

                    continue;
                }
                auto *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
                ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
                ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VGk));
                ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VAk));
                ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
                ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VGDirk));
                ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VSk));

                vpei.push_back(ei);

                vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
                optimizer.addEdge(ei);
            }
        }

        // Compute error for different scales
        std::set<g2o::HyperGraph::Edge *> setEdges = optimizer.edges();

        optimizer.setVerbose(false);
        optimizer.initializeOptimization();
        optimizer.optimize(its);

        scale = VS->estimate();

        // Recover optimized data
        // Biases
        VG = dynamic_cast<VertexGyroBias *>(optimizer.vertex((int) (maxKFid * 2 + 2)));
        VA = dynamic_cast<VertexAccBias *>(optimizer.vertex((int) (maxKFid * 2 + 3)));
        Vector6d vb;
        vb << VG->estimate(), VA->estimate();
        bg << VG->estimate();
        ba << VA->estimate();
        scale = VS->estimate();

        IMU::Bias b((float) vb[3], (float) vb[4], (float) vb[5], (float) vb[0],
                    (float) vb[1], (float) vb[2]);
        Rwg = VGDir->estimate().Rwg;

        // Keyframes velocities and biases
        const size_t N = vpKFs.size();
        for (size_t i = 0; i < N; i++) {
            KeyFrame *pKFi = vpKFs[i];
            if (pKFi->mnId > maxKFid) continue;

            auto *VV = dynamic_cast<VertexVelocity *>(
                    optimizer.vertex((int) (maxKFid + (pKFi->mnId) + 1)));
            Eigen::Vector3d Vw = VV->estimate();  // Velocity is scaled after
            pKFi->SetVelocity(Vw.cast<float>());

            if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01) {
                pKFi->SetNewBias(b);
                if (pKFi->mpImuPreintegrated) pKFi->mpImuPreintegrated->Reintegrate();
            } else
                pKFi->SetNewBias(b);
        }
    }

    void Optimizer::InertialOptimization(Map *pMap, Eigen::Vector3d &bg,
                                         Eigen::Vector3d &ba, float priorG,
                                         float priorA) {
        int its = 200;  // Check number of iterations
        long unsigned int maxKFid = pMap->GetMaxKFid();
        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver =
                new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        auto *solver_ptr = new g2o::BlockSolverX(linearSolver);

        auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(1e3);

        optimizer.setAlgorithm(solver);

        // Set KeyFrame vertices (fixed poses and optimizable velocities)
        for (auto pKFi: vpKFs) {
            if (pKFi->mnId > maxKFid) continue;
            auto *VP = new VertexPose(pKFi);
            VP->setId((int) pKFi->mnId);
            VP->setFixed(true);
            optimizer.addVertex(VP);

            auto *VV = new VertexVelocity(pKFi);
            VV->setId((int) (maxKFid + (pKFi->mnId) + 1));
            VV->setFixed(false);

            optimizer.addVertex(VV);
        }

        // Biases
        auto *VG = new VertexGyroBias(vpKFs.front());
        VG->setId((int) (maxKFid * 2 + 2));
        VG->setFixed(false);
        optimizer.addVertex(VG);

        auto *VA = new VertexAccBias(vpKFs.front());
        VA->setId((int) (maxKFid * 2 + 3));
        VA->setFixed(false);

        optimizer.addVertex(VA);
        // prior acc bias
        Eigen::Vector3f bprior;
        bprior.setZero();

        auto *epa = new EdgePriorAcc(bprior);
        epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
        double infoPriorA = priorA;
        epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
        optimizer.addEdge(epa);
        auto *epg = new EdgePriorGyro(bprior);
        epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
        double infoPriorG = priorG;
        epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
        optimizer.addEdge(epg);

        // Gravity and scale
        auto *VGDir = new VertexGDir(Eigen::Matrix3d::Identity());
        VGDir->setId((int) (maxKFid * 2 + 4));
        VGDir->setFixed(true);
        optimizer.addVertex(VGDir);
        auto *VS = new VertexScale(1.0);
        VS->setId((int) (maxKFid * 2 + 5));
        VS->setFixed(
                true);  // Fixed since scale is obtained from already well initialized map
        optimizer.addVertex(VS);

        // Graph edges
        // IMU links with gravity and scale
        vector<EdgeInertialGS *> vpei;
        vpei.reserve(vpKFs.size());
        vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
        vppUsedKF.reserve(vpKFs.size());

        for (auto pKFi: vpKFs) {
            if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
                if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;

                pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
                g2o::HyperGraph::Vertex *VP1 =
                        optimizer.vertex((int) (pKFi->mPrevKF->mnId));
                g2o::HyperGraph::Vertex *VV1 =
                        optimizer.vertex((int) (maxKFid + (pKFi->mPrevKF->mnId) + 1));
                g2o::HyperGraph::Vertex *VP2 = optimizer.vertex((int) (pKFi->mnId));
                g2o::HyperGraph::Vertex *VV2 =
                        optimizer.vertex((int) (maxKFid + (pKFi->mnId) + 1));
                g2o::HyperGraph::Vertex *VGk = optimizer.vertex((int) (maxKFid * 2 + 2));
                g2o::HyperGraph::Vertex *VAk = optimizer.vertex((int) (maxKFid * 2 + 3));
                g2o::HyperGraph::Vertex *VGDirk =
                        optimizer.vertex((int) (maxKFid * 2 + 4));
                g2o::HyperGraph::Vertex *VSk = optimizer.vertex((int) (maxKFid * 2 + 5));
                if (!VP1 || !VV1 || !VGk || !VAk || !VP2 || !VV2 || !VGDirk || !VSk) {
                    cout << "Error" << VP1 << ", " << VV1 << ", " << VGk << ", " << VAk
                         << ", " << VP2 << ", " << VV2 << ", " << VGDirk << ", " << VSk
                         << endl;

                    continue;
                }
                auto *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
                ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
                ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VGk));
                ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VAk));
                ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
                ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VGDirk));
                ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VSk));

                vpei.push_back(ei);

                vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
                optimizer.addEdge(ei);
            }
        }

        // Compute error for different scales
        optimizer.setVerbose(false);
        optimizer.initializeOptimization();
        optimizer.optimize(its);

        // Recover optimized data
        // Biases
        VG = dynamic_cast<VertexGyroBias *>(optimizer.vertex((int) (maxKFid * 2 + 2)));
        VA = dynamic_cast<VertexAccBias *>(optimizer.vertex((int) (maxKFid * 2 + 3)));
        Vector6d vb;
        vb << VG->estimate(), VA->estimate();
        bg << VG->estimate();
        ba << VA->estimate();

        IMU::Bias b((float) vb[3], (float) vb[4], (float) vb[5], (float) vb[0],
                    (float) vb[1], (float) vb[2]);

        // Keyframes velocities and biases
        const size_t N = vpKFs.size();
        for (size_t i = 0; i < N; i++) {
            KeyFrame *pKFi = vpKFs[i];
            if (pKFi->mnId > maxKFid) continue;

            auto *VV = dynamic_cast<VertexVelocity *>(
                    optimizer.vertex((int) (maxKFid + (pKFi->mnId) + 1)));
            Eigen::Vector3d Vw = VV->estimate();
            pKFi->SetVelocity(Vw.cast<float>());

            if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01) {
                pKFi->SetNewBias(b);
                if (pKFi->mpImuPreintegrated) pKFi->mpImuPreintegrated->Reintegrate();
            } else
                pKFi->SetNewBias(b);
        }
    }

    void Optimizer::InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg,
                                         double &scale) {
        int its = 10;
        long unsigned int maxKFid = pMap->GetMaxKFid();
        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver =
                new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        auto *solver_ptr = new g2o::BlockSolverX(linearSolver);

        auto *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
        optimizer.setAlgorithm(solver);

        // Set KeyFrame vertices (all variables are fixed)
        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKFi = vpKFs[i];
            if (pKFi->mnId > maxKFid) continue;
            auto *VP = new VertexPose(pKFi);
            VP->setId((int) (pKFi->mnId));
            VP->setFixed(true);
            optimizer.addVertex(VP);

            auto *VV = new VertexVelocity(pKFi);
            VV->setId((int) (maxKFid + 1 + (pKFi->mnId)));
            VV->setFixed(true);
            optimizer.addVertex(VV);

            // Vertex of fixed biases
            auto *VG = new VertexGyroBias(vpKFs.front());
            VG->setId((int) (2 * (maxKFid + 1) + (pKFi->mnId)));
            VG->setFixed(true);
            optimizer.addVertex(VG);
            auto *VA = new VertexAccBias(vpKFs.front());
            VA->setId((int) (3 * (maxKFid + 1) + (pKFi->mnId)));
            VA->setFixed(true);
            optimizer.addVertex(VA);
        }

        // Gravity and scale
        auto *VGDir = new VertexGDir(Rwg);
        VGDir->setId((int) (4 * (maxKFid + 1)));
        VGDir->setFixed(false);
        optimizer.addVertex(VGDir);
        auto *VS = new VertexScale(scale);
        VS->setId((int) (4 * (maxKFid + 1) + 1));
        VS->setFixed(false);
        optimizer.addVertex(VS);

        // Graph edges
        int count_edges = 0;
        for (auto pKFi: vpKFs) {
            if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
                if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;

                g2o::HyperGraph::Vertex *VP1 =
                        optimizer.vertex(((int) (pKFi->mPrevKF->mnId)));
                g2o::HyperGraph::Vertex *VV1 =
                        optimizer.vertex(((int) ((maxKFid + 1) + pKFi->mPrevKF->mnId)));
                g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(((int) (pKFi->mnId)));
                g2o::HyperGraph::Vertex *VV2 =
                        optimizer.vertex(((int) ((maxKFid + 1) + pKFi->mnId)));
                g2o::HyperGraph::Vertex *VG =
                        optimizer.vertex(((int) (2 * (maxKFid + 1) + pKFi->mPrevKF->mnId)));
                g2o::HyperGraph::Vertex *VA =
                        optimizer.vertex(((int) (3 * (maxKFid + 1) + pKFi->mPrevKF->mnId)));
                g2o::HyperGraph::Vertex *VGDirk =
                        optimizer.vertex(((int) (4 * (maxKFid + 1))));
                g2o::HyperGraph::Vertex *VSk =
                        optimizer.vertex(((int) (4 * (maxKFid + 1) + 1)));
                if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDirk || !VSk) {
                    Verbose::PrintMess(
                            "Error" + to_string(VP1->id()) + ", " + to_string(VV1->id()) +
                            ", " + to_string(VG->id()) + ", " + to_string(VA->id()) + ", " +
                            to_string(VP2->id()) + ", " + to_string(VV2->id()) + ", " +
                            to_string(VGDirk->id()) + ", " + to_string(VSk->id()),
                            Verbose::VERBOSITY_NORMAL);

                    continue;
                }
                count_edges++;
                auto *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
                ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
                ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
                ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
                ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
                ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VGDirk));
                ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VSk));
                auto *rk = new g2o::RobustKernelHuber;
                ei->setRobustKernel(rk);
                rk->setDelta(1.f);
                optimizer.addEdge(ei);
            }
        }

        // Compute error for different scales
        optimizer.setVerbose(false);
        optimizer.initializeOptimization();
        optimizer.computeActiveErrors();
        double err = optimizer.activeRobustChi2();
        optimizer.optimize(its);
        optimizer.computeActiveErrors();
        double err_end = optimizer.activeRobustChi2();
        // Recover optimized data
        scale = VS->estimate();
        Rwg = VGDir->estimate().Rwg;
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

        if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

        long unsigned int maxKFid = 0;
        set<KeyFrame *> spKeyFrameBA;

        Map *pCurrentMap = pMainKF->GetMap();

        // Set fixed KeyFrame vertices
        int numInsertedPoints = 0;
        for (KeyFrame *pKFi: vpFixedKF) {
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
            vSE3->setId((int) (pKFi->mnId));
            vSE3->setFixed(true);
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;

            set<MapPoint *> spViewMPs = pKFi->GetMapPoints();
            for (MapPoint *pMPi: spViewMPs) {
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
        for (KeyFrame *pKFi: vpAdjustKF) {
            if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap) continue;

            pKFi->mnBALocalForMerge = pMainKF->mnId;

            auto *vSE3 = new g2o::VertexSE3Expmap();
            Sophus::SE3<float> Tcw = pKFi->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                           Tcw.translation().cast<double>()));
            vSE3->setId((int) (pKFi->mnId));
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;

            set<MapPoint *> spViewMPs = pKFi->GetMapPoints();
            for (MapPoint *pMPi: spViewMPs) {
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

        vector<ORB_SLAM3::EdgeSE3ProjectXYZ *> vpEdgesMono;
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
        for (auto pMPi: vpMPs) {
            if (pMPi->isBad()) continue;

            auto *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMPi->GetWorldPos().cast<double>());
            const int id = (int) (pMPi->mnId + maxKFid + 1);
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, tuple<int, int>> observations =
                    pMPi->GetObservations();
            int nEdges = 0;
            // SET EDGES
            for (const auto &observation: observations) {
                KeyFrame *pKF = observation.first;
                if (pKF->isBad() || pKF->mnId > maxKFid ||
                    pKF->mnBALocalForMerge != pMainKF->mnId ||
                    !pKF->GetMapPoint(get<0>(observation.second)))
                    continue;

                nEdges++;

                const int leftIndex = get<0>(observation.second);
                const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];
                const float &kpSegVal = pKF->mvSegVal[leftIndex];

                if (pKF->mvuRight[get<0>(observation.second)] < 0)  // Monocular
                {
                    mpObsMPs[pMPi]++;
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    auto *e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int) (id))));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int) (pKF->mnId))));
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
                } else  // RGBD or Stereo
                {
                    mpObsMPs[pMPi] += 2;
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKF->mvuRight[get<0>(observation.second)];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    auto *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int) (id))));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int) (pKF->mnId))));
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
            if (*pbStopFlag) return;

        optimizer.initializeOptimization();
        optimizer.optimize(5);

        bool bDoMore = true;

        if (pbStopFlag)
            if (*pbStopFlag) bDoMore = false;

        map<unsigned long int, int> mWrongObsKF;
        if (bDoMore) {
            // Check inlier observations
            int badMonoMP = 0, badStereoMP = 0;
            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
                ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
                MapPoint *pMP = vpMapPointEdgeMono[i];

                if (pMP->isBad()) continue;

                if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                    e->setLevel(1);
                    badMonoMP++;
                }
                e->setRobustKernel(nullptr);
            }

            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
                g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
                MapPoint *pMP = vpMapPointEdgeStereo[i];

                if (pMP->isBad()) continue;

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
            ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad()) continue;

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

            if (pMP->isBad()) continue;

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
            for (auto &i: vToErase) {
                KeyFrame *pKFi = i.first;
                MapPoint *pMPi = i.second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
            }
        }
        for (auto pMPi: vpMPs) {
            if (pMPi->isBad()) continue;

            const map<KeyFrame *, tuple<int, int>> observations =
                    pMPi->GetObservations();
            for (const auto &observation: observations) {
                KeyFrame *pKF = observation.first;
                if (pKF->isBad() || pKF->mnId > maxKFid ||
                    pKF->mnBALocalForKF != pMainKF->mnId ||
                    !pKF->GetMapPoint(get<0>(observation.second)))
                    continue;

                if (pKF->mvuRight[get<0>(observation.second)] < 0)  // Monocular
                {
                    mpObsFinalKFs[pKF]++;
                } else  // RGBD or Stereo
                {
                    mpObsFinalKFs[pKF]++;
                }
            }
        }

        // Recover optimized data
        // Keyframes
        for (KeyFrame *pKFi: vpAdjustKF) {
            if (pKFi->isBad()) continue;

            auto *vSE3 = dynamic_cast<g2o::VertexSE3Expmap *>(
                    optimizer.vertex((int) (pKFi->mnId)));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                             SE3quat.translation().cast<float>());

            int numMonoBadPoints = 0, numMonoOptPoints = 0;
            int numStereoBadPoints = 0, numStereoOptPoints = 0;
            vector<MapPoint *> vpMonoMPsOpt, vpStereoMPsOpt;
            vector<MapPoint *> vpMonoMPsBad, vpStereoMPsBad;

            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
                ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
                MapPoint *pMP = vpMapPointEdgeMono[i];
                KeyFrame *pKFedge = vpEdgeKFMono[i];

                if (pKFi != pKFedge) {
                    continue;
                }

                if (pMP->isBad()) continue;

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

                if (pMP->isBad()) continue;

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
        for (MapPoint *pMPi: vpMPs) {
            if (pMPi->isBad()) continue;

            auto *vPoint = dynamic_cast<g2o::VertexSBAPointXYZ *>(
                    optimizer.vertex((int) (pMPi->mnId + maxKFid + 1)));
            pMPi->SetWorldPos(vPoint->estimate().cast<float>());
            pMPi->UpdateNormalAndDepth();
        }
    }

    void Optimizer::MergeInertialBA(KeyFrame *pCurrKF, KeyFrame *pMergeKF,
                                    bool *pbStopFlag, Map *pMap,
                                    LoopClosing::KeyFrameAndPose &corrPoses) {
        const int Nd = 6;
        const unsigned long maxKFid = pCurrKF->mnId;

        vector<KeyFrame *> vpOptimizableKFs;
        vpOptimizableKFs.reserve(2 * Nd);

        // For cov KFS, inertial parameters are not optimized
        const int maxCovKF = 30;
        vector<KeyFrame *> vpOptimizableCovKFs;
        vpOptimizableCovKFs.reserve(maxCovKF);

        // Add sliding window for current KF
        vpOptimizableKFs.push_back(pCurrKF);
        pCurrKF->mnBALocalForKF = pCurrKF->mnId;
        for (int i = 1; i < Nd; i++) {
            if (vpOptimizableKFs.back()->mPrevKF) {
                vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
                vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
            } else
                break;
        }

        list<KeyFrame *> lFixedKeyFrames;
        if (vpOptimizableKFs.back()->mPrevKF) {
            vpOptimizableCovKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mPrevKF->mnBALocalForKF = pCurrKF->mnId;
        } else {
            vpOptimizableCovKFs.push_back(vpOptimizableKFs.back());
            vpOptimizableKFs.pop_back();
        }

        // Add temporal neighbours to merge KF (previous and next KFs)
        vpOptimizableKFs.push_back(pMergeKF);
        pMergeKF->mnBALocalForKF = pCurrKF->mnId;

        // Previous KFs
        for (int i = 1; i < (Nd / 2); i++) {
            if (vpOptimizableKFs.back()->mPrevKF) {
                vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
                vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
            } else
                break;
        }

        // We fix just once the old map
        if (vpOptimizableKFs.back()->mPrevKF) {
            lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pCurrKF->mnId;
        } else {
            vpOptimizableKFs.back()->mnBALocalForKF = 0;
            vpOptimizableKFs.back()->mnBAFixedForKF = pCurrKF->mnId;
            lFixedKeyFrames.push_back(vpOptimizableKFs.back());
            vpOptimizableKFs.pop_back();
        }

        // Next KFs
        if (pMergeKF->mNextKF) {
            vpOptimizableKFs.push_back(pMergeKF->mNextKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
        }

        while (vpOptimizableKFs.size() < (2 * Nd)) {
            if (vpOptimizableKFs.back()->mNextKF) {
                vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mNextKF);
                vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
            } else
                break;
        }

        size_t N = vpOptimizableKFs.size();

        // Optimizable points seen by optimizable keyframes
        list<MapPoint *> lLocalMapPoints;
        map<MapPoint *, int> mLocalObs;
        for (size_t i = 0; i < N; i++) {
            vector<MapPoint *> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
            for (auto pMP: vpMPs) {
                // Using mnBALocalForKF we avoid redundance here, one MP can not be added
                // several times to lLocalMapPoints
                if (pMP) {
                    if (!pMP->isBad()) {
                        if (pMP->mnBALocalForKF != pCurrKF->mnId) {
                            mLocalObs[pMP] = 1;
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF = pCurrKF->mnId;
                        }
                    } else {
                        mLocalObs[pMP]++;
                    }
                }
            }
        }

        std::vector<std::pair<MapPoint *, int>> pairs;
        pairs.reserve(mLocalObs.size());
        for (auto &mLocalOb: mLocalObs) pairs.push_back(mLocalOb);
        sort(pairs.begin(), pairs.end(), sortByVal);

        // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local
        // Keyframes
        int j = 0;
        for (auto lit = pairs.begin(), lend = pairs.end(); lit != lend; lit++, j++) {
            map<KeyFrame *, tuple<int, int>> observations =
                    lit->first->GetObservations();
            if (j >= maxCovKF) break;
            for (auto &observation: observations) {
                KeyFrame *pKFi = observation.first;

                if (pKFi->mnBALocalForKF != pCurrKF->mnId &&
                    pKFi->mnBAFixedForKF !=
                    pCurrKF->mnId)  // If optimizable or already included...
                {
                    pKFi->mnBALocalForKF = pCurrKF->mnId;
                    if (!pKFi->isBad()) {
                        vpOptimizableCovKFs.push_back(pKFi);
                        break;
                    }
                }
            }
        }

        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;
        linearSolver =
                new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        auto *solver_ptr = new g2o::BlockSolverX(linearSolver);

        auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

        solver->setUserLambdaInit(1e3);

        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        // Set Local KeyFrame vertices
        N = vpOptimizableKFs.size();
        for (size_t i = 0; i < N; i++) {
            KeyFrame *pKFi = vpOptimizableKFs[i];

            auto *VP = new VertexPose(pKFi);
            VP->setId((int) (pKFi->mnId));
            VP->setFixed(false);
            optimizer.addVertex(VP);

            if (pKFi->bImu) {
                auto *VV = new VertexVelocity(pKFi);
                VV->setId((int) (maxKFid + 3 * (pKFi->mnId) + 1));
                VV->setFixed(false);
                optimizer.addVertex(VV);
                auto *VG = new VertexGyroBias(pKFi);
                VG->setId((int) (maxKFid + 3 * (pKFi->mnId) + 2));
                VG->setFixed(false);
                optimizer.addVertex(VG);
                auto *VA = new VertexAccBias(pKFi);
                VA->setId((int) (maxKFid + 3 * (pKFi->mnId) + 3));
                VA->setFixed(false);
                optimizer.addVertex(VA);
            }
        }

        // Set Local cov keyframes vertices
        size_t Ncov = vpOptimizableCovKFs.size();
        for (int i = 0; i < Ncov; i++) {
            KeyFrame *pKFi = vpOptimizableCovKFs[i];

            auto *VP = new VertexPose(pKFi);
            VP->setId((int) (pKFi->mnId));
            VP->setFixed(false);
            optimizer.addVertex(VP);

            if (pKFi->bImu) {
                auto *VV = new VertexVelocity(pKFi);
                VV->setId((int) (maxKFid + 3 * (pKFi->mnId) + 1));
                VV->setFixed(false);
                optimizer.addVertex(VV);
                auto *VG = new VertexGyroBias(pKFi);
                VG->setId((int) (maxKFid + 3 * (pKFi->mnId) + 2));
                VG->setFixed(false);
                optimizer.addVertex(VG);
                auto *VA = new VertexAccBias(pKFi);
                VA->setId((int) (maxKFid + 3 * (pKFi->mnId) + 3));
                VA->setFixed(false);
                optimizer.addVertex(VA);
            }
        }

        // Set Fixed KeyFrame vertices
        for (auto pKFi: lFixedKeyFrames) {
            auto *VP = new VertexPose(pKFi);
            VP->setId((int) (pKFi->mnId));
            VP->setFixed(true);
            optimizer.addVertex(VP);

            if (pKFi->bImu) {
                auto *VV = new VertexVelocity(pKFi);
                VV->setId((int) (maxKFid + 3 * (pKFi->mnId) + 1));
                VV->setFixed(true);
                optimizer.addVertex(VV);
                auto *VG = new VertexGyroBias(pKFi);
                VG->setId((int) (maxKFid + 3 * (pKFi->mnId) + 2));
                VG->setFixed(true);
                optimizer.addVertex(VG);
                auto *VA = new VertexAccBias(pKFi);
                VA->setId((int) (maxKFid + 3 * (pKFi->mnId) + 3));
                VA->setFixed(true);
                optimizer.addVertex(VA);
            }
        }

        // Create intertial constraints
        vector<EdgeInertial *> vei(N, (EdgeInertial *) nullptr);
        vector<EdgeGyroRW *> vegr(N, (EdgeGyroRW *) nullptr);
        vector<EdgeAccRW *> vear(N, (EdgeAccRW *) nullptr);
        for (size_t i = 0; i < N; i++) {
            // cout << "inserting inertial edge " << i << endl;
            KeyFrame *pKFi = vpOptimizableKFs[i];

            if (!pKFi->mPrevKF) {
                Verbose::PrintMess("NOT INERTIAL LINK TO PREVIOUS FRAME!!!!",
                                   Verbose::VERBOSITY_NORMAL);
                continue;
            }
            if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
                pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
                g2o::HyperGraph::Vertex *VP1 =
                        optimizer.vertex((int) (pKFi->mPrevKF->mnId));
                g2o::HyperGraph::Vertex *VV1 =
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1));
                g2o::HyperGraph::Vertex *VG1 =
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2));
                g2o::HyperGraph::Vertex *VA1 =
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3));
                g2o::HyperGraph::Vertex *VP2 = optimizer.vertex((int) (pKFi->mnId));
                g2o::HyperGraph::Vertex *VV2 =
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 1));
                g2o::HyperGraph::Vertex *VG2 =
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 2));
                g2o::HyperGraph::Vertex *VA2 =
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 3));

                if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
                    cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
                         << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2
                         << endl;
                    continue;
                }

                vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

                vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
                vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG1));
                vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA1));
                vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));

                auto *rki = new g2o::RobustKernelHuber;
                vei[i]->setRobustKernel(rki);
                rki->setDelta(sqrt(16.92));
                optimizer.addEdge(vei[i]);

                vegr[i] = new EdgeGyroRW();
                vegr[i]->setVertex(0, VG1);
                vegr[i]->setVertex(1, VG2);
                Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                        .cast<double>()
                        .inverse();
                vegr[i]->setInformation(InfoG);
                optimizer.addEdge(vegr[i]);

                vear[i] = new EdgeAccRW();
                vear[i]->setVertex(0, VA1);
                vear[i]->setVertex(1, VA2);
                Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                        .cast<double>()
                        .inverse();
                vear[i]->setInformation(InfoA);
                optimizer.addEdge(vear[i]);
            } else
                Verbose::PrintMess("ERROR building inertial edge",
                                   Verbose::VERBOSITY_NORMAL);
        }

        Verbose::PrintMess("end inserting inertial edges", Verbose::VERBOSITY_NORMAL);

        // Set MapPoint vertices
        const size_t nExpectedSize =
                (N + Ncov + lFixedKeyFrames.size()) * lLocalMapPoints.size();

        // Mono
        vector<EdgeMono *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        // Stereo
        vector<EdgeStereo *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);

        const double thHuberMono = sqrt(5.991);
        const float chi2Mono2 = 5.991;
        const double thHuberStereo = sqrt(7.815);
        const float chi2Stereo2 = 7.815;

        const unsigned long iniMPid = maxKFid * 5;

        for (auto pMP: lLocalMapPoints) {
            if (!pMP) continue;

            auto *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

            unsigned long id = pMP->mnId + iniMPid + 1;
            vPoint->setId((int) (id));
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

            // Create visual constraints
            for (const auto &observation: observations) {
                KeyFrame *pKFi = observation.first;

                if (!pKFi) continue;

                if ((pKFi->mnBALocalForKF != pCurrKF->mnId) &&
                    (pKFi->mnBAFixedForKF != pCurrKF->mnId))
                    continue;

                if (pKFi->mnId > maxKFid) {
                    continue;
                }

                if (optimizer.vertex((int) (id)) == nullptr ||
                    optimizer.vertex((int) (pKFi->mnId)) == nullptr)
                    continue;

                if (!pKFi->isBad()) {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[get<0>(observation.second)];
                    const float &kpSegVal = pKFi->mvSegVal[get<0>(observation.second)];

                    if (pKFi->mvuRight[get<0>(observation.second)] <
                        0)  // Monocular observation
                    {
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        auto *e = new EdgeMono();
                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(((int) (id)))));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(((int) (pKFi->mnId)))));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        auto *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);
                        // ARK
//          auto* ark = new g2o::RobustKernelAdaptive;
//          double alphaBA;
//          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//          e->setRobustKernel(ark);
//          if (kpSegVal == PEOPLE_COLOR) {
//            alphaBA = -10020.0f;
//          }
//          else {
//            alphaBA = 1.0f;
//          }
//          ark->setAlpha(alphaBA);
                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);
                    } else  // stereo observation
                    {
                        const float kp_ur = pKFi->mvuRight[get<0>(observation.second)];
                        Eigen::Matrix<double, 3, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        auto *e = new EdgeStereo();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) (id))));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) (pKFi->mnId))));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                        auto *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        // ARK
//          auto* ark = new g2o::RobustKernelAdaptive;
//          double alphaBA;
//          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//          e->setRobustKernel(ark);
//          if (kpSegVal == PEOPLE_COLOR) {
//            alphaBA = -10020.0f;
//          }
//          else {
//            alphaBA = 1.0f;
//          }
//          ark->setAlpha(alphaBA);

                        optimizer.addEdge(e);
                        vpEdgesStereo.push_back(e);
                        vpEdgeKFStereo.push_back(pKFi);
                        vpMapPointEdgeStereo.push_back(pMP);
                    }
                }
            }
        }

        if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

        if (pbStopFlag)
            if (*pbStopFlag) return;

        optimizer.initializeOptimization();
        optimizer.optimize(8);

        vector<pair<KeyFrame *, MapPoint *>> vToErase;
        vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

        // Check inlier observations
        // Mono
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            EdgeMono *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad()) continue;

            if (e->chi2() > chi2Mono2) {
                KeyFrame *pKFi = vpEdgeKFMono[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // Stereo
        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
            EdgeStereo *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad()) continue;

            if (e->chi2() > chi2Stereo2) {
                KeyFrame *pKFi = vpEdgeKFStereo[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // Get Map Mutex and erase outliers
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        if (!vToErase.empty()) {
            for (auto &i: vToErase) {
                KeyFrame *pKFi = i.first;
                MapPoint *pMPi = i.second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
            }
        }

        // Recover optimized data
        // Keyframes
        for (size_t i = 0; i < N; i++) {
            KeyFrame *pKFi = vpOptimizableKFs[i];

            auto *VP = dynamic_cast<VertexPose *>(optimizer.vertex((int) (pKFi->mnId)));
            Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                             VP->estimate().tcw[0].cast<float>());
            pKFi->SetPose(Tcw);

            Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
            g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
            corrPoses[pKFi] = g2oSiw;

            if (pKFi->bImu) {
                auto *VV = dynamic_cast<VertexVelocity *>(
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 1)));
                pKFi->SetVelocity(VV->estimate().cast<float>());
                auto *VG = dynamic_cast<VertexGyroBias *>(
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 2)));
                auto *VA = dynamic_cast<VertexAccBias *>(
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 3)));
                Vector6d b;
                b << VG->estimate(), VA->estimate();
                pKFi->SetNewBias(IMU::Bias((float) b[3], (float) b[4], (float) b[5],
                                           (float) b[0], (float) b[1], (float) b[2]));
            }
        }

        for (size_t i = 0; i < Ncov; i++) {
            KeyFrame *pKFi = vpOptimizableCovKFs[i];

            auto *VP = dynamic_cast<VertexPose *>(optimizer.vertex((int) (pKFi->mnId)));
            Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                             VP->estimate().tcw[0].cast<float>());
            pKFi->SetPose(Tcw);

            Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
            g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
            corrPoses[pKFi] = g2oSiw;

            if (pKFi->bImu) {
                auto *VV = dynamic_cast<VertexVelocity *>(
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 1)));
                pKFi->SetVelocity(VV->estimate().cast<float>());
                auto *VG = dynamic_cast<VertexGyroBias *>(
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 2)));
                auto *VA = dynamic_cast<VertexAccBias *>(
                        optimizer.vertex((int) (maxKFid + 3 * (pKFi->mnId) + 3)));
                Vector6d b;
                b << VG->estimate(), VA->estimate();
                pKFi->SetNewBias(IMU::Bias((float) b[3], (float) b[4], (float) b[5],
                                           (float) b[0], (float) b[1], (float) b[2]));
            }
        }

        // Points
        for (auto pMP: lLocalMapPoints) {
            auto *vPoint = dynamic_cast<g2o::VertexSBAPointXYZ *>(
                    optimizer.vertex((int) (pMP->mnId + iniMPid + 1)));
            pMP->SetWorldPos(vPoint->estimate().cast<float>());
            pMP->UpdateNormalAndDepth();
        }

        pMap->IncreaseChangeIndex();
    }

    int Optimizer::PoseInertialOptimizationLastKeyFrame(Frame *pFrame,
                                                        bool bRecInit) {
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver =
                new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        auto *solver_ptr = new g2o::BlockSolverX(linearSolver);

        auto *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
        optimizer.setVerbose(false);
        optimizer.setAlgorithm(solver);

        int nInitialMonoCorrespondences = 0;
        int nInitialStereoCorrespondences = 0;
        int nInitialCorrespondences = 0;

        // Set Frame vertex
        auto *VP = new VertexPose(pFrame);
        VP->setId(0);
        VP->setFixed(false);
        optimizer.addVertex(VP);
        auto *VV = new VertexVelocity(pFrame);
        VV->setId(1);
        VV->setFixed(false);
        optimizer.addVertex(VV);
        auto *VG = new VertexGyroBias(pFrame);
        VG->setId(2);
        VG->setFixed(false);
        optimizer.addVertex(VG);
        auto *VA = new VertexAccBias(pFrame);
        VA->setId(3);
        VA->setFixed(false);
        optimizer.addVertex(VA);

        // Set MapPoint vertices
        const int N = pFrame->N;
        const int Nleft = pFrame->Nleft;
        const bool bRight = (Nleft != -1);

        vector<EdgeMonoOnlyPose *> vpEdgesMono;
        vector<EdgeStereoOnlyPose *> vpEdgesStereo;
        vector<size_t> vnIndexEdgeMono;
        vector<size_t> vnIndexEdgeStereo;
        vpEdgesMono.reserve(N);
        vpEdgesStereo.reserve(N);
        vnIndexEdgeMono.reserve(N);
        vnIndexEdgeStereo.reserve(N);

        const double thHuberMono = sqrt(5.991);
        const double thHuberStereo = sqrt(7.815);

        {
            unique_lock<mutex> lock(MapPoint::mGlobalMutex);

            //    tbb::parallel_for(
            //        tbb::blocked_range<size_t>(0, N), [&](tbb::blocked_range<size_t>
            //        rN) {
            //          for (int i = rN.begin(); i < rN.end(); i++) {
            for (int i = 0; i < N; i++) {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if (pMP) {
                    cv::KeyPoint kpUn;
                    float kpSegVal;
                    // Left monocular observation
                    if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
                        if (i < Nleft)  // pair left-right
                            kpUn = pFrame->mvKeys[i];
                        else
                            kpUn = pFrame->mvKeysUn[i];
                        kpSegVal = pFrame->mvSegVal[i];

                        nInitialMonoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        auto *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        auto *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        // ARK
//          auto* ark = new g2o::RobustKernelAdaptive;
//          double alphaBA;
//          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//          e->setRobustKernel(ark);
//          if (kpSegVal == PEOPLE_COLOR) {
//            alphaBA = -10020.0f;
//          }
//          else {
//            alphaBA = 1.0f;
//          }
//          ark->setAlpha(alphaBA);

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }
                        // Stereo observation
                    else if (!bRight) {
                        nInitialStereoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        kpUn = pFrame->mvKeysUn[i];
                        kpSegVal = pFrame->mvSegVal[i];
                        const float kp_ur = pFrame->mvuRight[i];
                        Eigen::Matrix<double, 3, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        auto *e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

                        const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                        auto *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        // ARK
//          auto* ark = new g2o::RobustKernelAdaptive;
//          double alphaBA;
//          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//          e->setRobustKernel(ark);
//          if (kpSegVal == PEOPLE_COLOR) {
//            alphaBA = -10020.0f;
//          }
//          else {
//            alphaBA = 1.0f;
//          }
//          ark->setAlpha(alphaBA);

                        optimizer.addEdge(e);

                        vpEdgesStereo.push_back(e);
                        vnIndexEdgeStereo.push_back(i);
                    }

                    // Right monocular observation
                    if (bRight && i >= Nleft) {
                        nInitialMonoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        kpUn = pFrame->mvKeysRight[i - Nleft];
                        kpSegVal = pFrame->mvSegVal[i - Nleft];
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        auto *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        auto *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        // ARK
//          auto* ark = new g2o::RobustKernelAdaptive;
//          double alphaBA;
//          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//          e->setRobustKernel(ark);
//          if (kpSegVal == PEOPLE_COLOR) {
//            alphaBA = -10020.0f;
//          }
//          else {
//            alphaBA = 1.0f;
//          }
//          ark->setAlpha(alphaBA);

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }
                }
            }
            //        });
        }
        nInitialCorrespondences =
                nInitialMonoCorrespondences + nInitialStereoCorrespondences;

        KeyFrame *pKF = pFrame->mpLastKeyFrame;
        auto *VPk = new VertexPose(pKF);
        VPk->setId(4);
        VPk->setFixed(true);
        optimizer.addVertex(VPk);
        auto *VVk = new VertexVelocity(pKF);
        VVk->setId(5);
        VVk->setFixed(true);
        optimizer.addVertex(VVk);
        auto *VGk = new VertexGyroBias(pKF);
        VGk->setId(6);
        VGk->setFixed(true);
        optimizer.addVertex(VGk);
        auto *VAk = new VertexAccBias(pKF);
        VAk->setId(7);
        VAk->setFixed(true);
        optimizer.addVertex(VAk);

        auto *ei = new EdgeInertial(pFrame->mpImuPreintegrated);

        ei->setVertex(0, VPk);
        ei->setVertex(1, VVk);
        ei->setVertex(2, VGk);
        ei->setVertex(3, VAk);
        ei->setVertex(4, VP);
        ei->setVertex(5, VV);
        optimizer.addEdge(ei);
        auto *egr = new EdgeGyroRW();
        egr->setVertex(0, VGk);
        egr->setVertex(1, VG);
        Eigen::Matrix3d InfoG =
                pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
        egr->setInformation(InfoG);
        optimizer.addEdge(egr);

        auto *ear = new EdgeAccRW();
        ear->setVertex(0, VAk);
        ear->setVertex(1, VA);
        Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                .cast<double>()
                .inverse();
        ear->setInformation(InfoA);
        optimizer.addEdge(ear);

        auto *eo = new EdgeWOdometry(pFrame->mpOdomPreintegrated);
        eo->setVertex(0, VPk);
        eo->setVertex(1, VP);
        optimizer.addEdge(eo);

        // We perform 4 optimizations, after each optimization we classify observation
        // as inlier/outlier At the next optimization, outliers are not included, but
        // at the end they can be classified as inliers again.
        float chi2Mono[4] = {12, 7.5, 5.991, 5.991};
        float chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};

        int its[4] = {10, 10, 10, 10};

        int nBad = 0;
        int nBadMono = 0;
        int nBadStereo = 0;
        int nInliersMono = 0;
        int nInliersStereo = 0;
        int nInliers = 0;
        for (size_t it = 0; it < 4; it++) {
            optimizer.initializeOptimization(0);
            optimizer.optimize(its[it]);

            nBad = 0;
            nBadMono = 0;
            nBadStereo = 0;
            nInliers = 0;
            nInliersMono = 0;
            nInliersStereo = 0;
            float chi2close = 1.5f * chi2Mono[it];

            // For monocular observations
            //    tbb::parallel_for(
            //        tbb::blocked_range<size_t>(0, vpEdgesMono.size()),
            //        [&](tbb::blocked_range<size_t> rvpEdgesMono) {
            //          for (size_t i = rvpEdgesMono.begin(), iend = rvpEdgesMono.end();
            //               i < iend;
            //               i++) {
            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
                EdgeMonoOnlyPose *e = vpEdgesMono[i];

                const size_t idx = vnIndexEdgeMono[i];
                bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

                if (pFrame->mvbOutlier[idx]) {
                    e->computeError();
                }

                const double chi2 = e->chi2();

                if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) ||
                    !e->isDepthPositive()) {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBadMono++;
                } else {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                    nInliersMono++;
                }

                if (it == 2) e->setRobustKernel(nullptr);
            }
            //        });

            // Stereo
            //    tbb::parallel_for(tbb::blocked_range<size_t>(0, vpEdgesStereo.size()),
            //                      [&](tbb::blocked_range<size_t> rvpEdgesStereo) {
            //                        for (size_t i = rvpEdgesStereo.begin(),
            //                                    iend = rvpEdgesStereo.end();
            //                             i < iend;
            //                             i++) {
            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
                EdgeStereoOnlyPose *e = vpEdgesStereo[i];

                const size_t idx = vnIndexEdgeStereo[i];

                if (pFrame->mvbOutlier[idx]) {
                    e->computeError();
                }

                const double chi2 = e->chi2();

                if (chi2 > chi2Stereo[it]) {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBadStereo++;
                } else {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                    nInliersStereo++;
                }

                if (it == 2) e->setRobustKernel(nullptr);
            }
            //                      });

            nInliers = nInliersMono + nInliersStereo;
            nBad = nBadMono + nBadStereo;

            if (optimizer.edges().size() < 10) {
                break;
            }
        }

        // If not too much tracks, recover not too bad points
        if ((nInliers < 30) && !bRecInit) {
            nBad = 0;
            const float chi2MonoOut = 18.f;
            const float chi2StereoOut = 24.f;
            EdgeMonoOnlyPose *e1;
            EdgeStereoOnlyPose *e2;
            for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
                const size_t idx = vnIndexEdgeMono[i];
                e1 = vpEdgesMono[i];
                e1->computeError();
                if (e1->chi2() < chi2MonoOut)
                    pFrame->mvbOutlier[idx] = false;
                else
                    nBad++;
            }
            for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
                const size_t idx = vnIndexEdgeStereo[i];
                e2 = vpEdgesStereo[i];
                e2->computeError();
                if (e2->chi2() < chi2StereoOut)
                    pFrame->mvbOutlier[idx] = false;
                else
                    nBad++;
            }
        }

        // Recover optimized pose, velocity and biases
        pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                                   VP->estimate().twb.cast<float>(),
                                   VV->estimate().cast<float>());
        Vector6d b;
        b << VG->estimate(), VA->estimate();
        pFrame->mImuBias = IMU::Bias((float) b[3], (float) b[4], (float) b[5],
                                     (float) b[0], (float) b[1], (float) b[2]);

        // Recover Hessian, marginalize keyframe states and generate new prior for
        // frame
        Eigen::Matrix<double, 15, 15> H;
        H.setZero();

        H.block<9, 9>(0, 0) += ei->GetHessian2();
        H.block<3, 3>(9, 9) += egr->GetHessian2();
        H.block<3, 3>(12, 12) += ear->GetHessian2();

        int tot_in = 0, tot_out = 0;
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            EdgeMonoOnlyPose *e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if (!pFrame->mvbOutlier[idx]) {
                H.block<6, 6>(0, 0) += e->GetHessian();
                tot_in++;
            } else
                tot_out++;
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
            EdgeStereoOnlyPose *e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if (!pFrame->mvbOutlier[idx]) {
                H.block<6, 6>(0, 0) += e->GetHessian();
                tot_in++;
            } else
                tot_out++;
        }

        pFrame->mpcpi =
                new ConstraintPoseImu(VP->estimate().Rwb, VP->estimate().twb,
                                      VV->estimate(), VG->estimate(), VA->estimate(), H);

        return nInitialCorrespondences - nBad;
    }

    int Optimizer::PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit) {
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver =
                new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        auto *solver_ptr = new g2o::BlockSolverX(linearSolver);

        auto solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        int nInitialMonoCorrespondences = 0;
        int nInitialStereoCorrespondences = 0;
        int nInitialCorrespondences = 0;

        // Set Current Frame vertex
        auto *VP = new VertexPose(pFrame);
        VP->setId(0);
        VP->setFixed(false);
        optimizer.addVertex(VP);
        auto *VV = new VertexVelocity(pFrame);
        VV->setId(1);
        VV->setFixed(false);
        optimizer.addVertex(VV);
        auto *VG = new VertexGyroBias(pFrame);
        VG->setId(2);
        VG->setFixed(false);
        optimizer.addVertex(VG);
        auto *VA = new VertexAccBias(pFrame);
        VA->setId(3);
        VA->setFixed(false);
        optimizer.addVertex(VA);

        // Set MapPoint vertices
        const int N = pFrame->N;
        const int Nleft = pFrame->Nleft;
        const bool bRight = (Nleft != -1);

        vector<EdgeMonoOnlyPose *> vpEdgesMono;
        vector<EdgeStereoOnlyPose *> vpEdgesStereo;
        vector<size_t> vnIndexEdgeMono;
        vector<size_t> vnIndexEdgeStereo;
        vpEdgesMono.reserve(N);
        vpEdgesStereo.reserve(N);
        vnIndexEdgeMono.reserve(N);
        vnIndexEdgeStereo.reserve(N);

        const double thHuberMono = sqrt(5.991);
        const double thHuberStereo = sqrt(7.815);

        {
            unique_lock<mutex> lock(MapPoint::mGlobalMutex);

            //    tbb::parallel_for(
            //        tbb::blocked_range<size_t>(0, N), [&](tbb::blocked_range<size_t>
            //        rN) {
            //          for (int i = rN.begin(); i < rN.end(); i++) {
            for (int i = 0; i < N; i++) {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if (pMP) {
                    cv::KeyPoint kpUn;
                    float kpSegVal;
                    // Left monocular observation
                    if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
                        if (i < Nleft)  // pair left-right
                            kpUn = pFrame->mvKeys[i];
                        else
                            kpUn = pFrame->mvKeysUn[i];
                        kpSegVal = pFrame->mvSegVal[i];

                        nInitialMonoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        auto *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        auto *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        // ARK
//          auto* ark = new g2o::RobustKernelAdaptive;
//          double alphaBA;
//          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//          e->setRobustKernel(ark);
//          if (kpSegVal == PEOPLE_COLOR) {
//            alphaBA = -10020.0f;
//          }
//          else {
//            alphaBA = 1.0f;
//          }
//          ark->setAlpha(alphaBA);

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }
                        // Stereo observation
                    else if (!bRight) {
                        nInitialStereoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        kpUn = pFrame->mvKeysUn[i];
                        kpSegVal = pFrame->mvSegVal[i];
                        const float kp_ur = pFrame->mvuRight[i];
                        Eigen::Matrix<double, 3, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        auto *e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

                        const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                        auto *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        // ARK
//          auto* ark = new g2o::RobustKernelAdaptive;
//          double alphaBA;
//          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//          e->setRobustKernel(ark);
//          if (kpSegVal == PEOPLE_COLOR) {
//            alphaBA = -10020.0f;
//          }
//          else {
//            alphaBA = 1.0f;
//          }
//          ark->setAlpha(alphaBA);

                        optimizer.addEdge(e);

                        vpEdgesStereo.push_back(e);
                        vnIndexEdgeStereo.push_back(i);
                    }

                    // Right monocular observation
                    if (bRight && i >= Nleft) {
                        nInitialMonoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        kpUn = pFrame->mvKeysRight[i - Nleft];
                        kpSegVal = pFrame->mvSegVal[i - Nleft];
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        auto *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        auto *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        // ARK
//          auto* ark = new g2o::RobustKernelAdaptive;
//          double alphaBA;
//          cv::Vec3b PEOPLE_COLOR(61, 5, 150);
//          e->setRobustKernel(ark);
//          if (kpSegVal == PEOPLE_COLOR) {
//            alphaBA = -10020.0f;
//          }
//          else {
//            alphaBA = 1.0f;
//          }
//          ark->setAlpha(alphaBA);

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }
                }
            }
            //        });
        }

        nInitialCorrespondences =
                nInitialMonoCorrespondences + nInitialStereoCorrespondences;

        // Set Previous Frame Vertex
        Frame *pFp = pFrame->mpPrevFrame;

        auto *VPk = new VertexPose(pFp);
        VPk->setId(4);
        VPk->setFixed(false);
        optimizer.addVertex(VPk);
        auto *VVk = new VertexVelocity(pFp);
        VVk->setId(5);
        VVk->setFixed(false);
        optimizer.addVertex(VVk);
        auto *VGk = new VertexGyroBias(pFp);
        VGk->setId(6);
        VGk->setFixed(false);
        optimizer.addVertex(VGk);
        auto *VAk = new VertexAccBias(pFp);
        VAk->setId(7);
        VAk->setFixed(false);
        optimizer.addVertex(VAk);

        auto *ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

        ei->setVertex(0, VPk);
        ei->setVertex(1, VVk);
        ei->setVertex(2, VGk);
        ei->setVertex(3, VAk);
        ei->setVertex(4, VP);
        ei->setVertex(5, VV);
        optimizer.addEdge(ei);

        auto *egr = new EdgeGyroRW();
        egr->setVertex(0, VGk);
        egr->setVertex(1, VG);
        Eigen::Matrix3d InfoG =
                pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
        egr->setInformation(InfoG);
        optimizer.addEdge(egr);

        auto *ear = new EdgeAccRW();
        ear->setVertex(0, VAk);
        ear->setVertex(1, VA);
        Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                .cast<double>()
                .inverse();
        ear->setInformation(InfoA);
        optimizer.addEdge(ear);

        auto *eo = new EdgeWOdometry(pFrame->mpOdomPreintegratedFrame);
        eo->setVertex(0, VPk);
        eo->setVertex(1, VP);
        optimizer.addEdge(eo);

        if (!pFp->mpcpi)
            Verbose::PrintMess(
                    "pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFp->mnId),
                    Verbose::VERBOSITY_NORMAL);

        auto *ep = new EdgePriorPoseImu(pFp->mpcpi);

        ep->setVertex(0, VPk);
        ep->setVertex(1, VVk);
        ep->setVertex(2, VGk);
        ep->setVertex(3, VAk);
        auto *rkp = new g2o::RobustKernelHuber;
        ep->setRobustKernel(rkp);
        rkp->setDelta(5);
        optimizer.addEdge(ep);

        // We perform 4 optimizations, after each optimization we classify observation
        // as inlier/outlier At the next optimization, outliers are not included, but
        // at the end they can be classified as inliers again.
        const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
        const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
        const int its[4] = {10, 10, 10, 10};

        int nBad = 0;
        int nBadMono = 0;
        int nBadStereo = 0;
        int nInliersMono = 0;
        int nInliersStereo = 0;
        int nInliers = 0;
        for (size_t it = 0; it < 4; it++) {
            optimizer.initializeOptimization(0);
            optimizer.optimize(its[it]);

            nBad = 0;
            nBadMono = 0;
            nBadStereo = 0;
            nInliers = 0;
            nInliersMono = 0;
            nInliersStereo = 0;
            float chi2close = 1.5f * chi2Mono[it];

            // Mono and right fisheye
            //    tbb::parallel_for(
            //        tbb::blocked_range<size_t>(0, vpEdgesMono.size()),
            //        [&](tbb::blocked_range<size_t> rvpEdgesMono) {
            //          for (size_t i = rvpEdgesMono.begin(), iend = rvpEdgesMono.end();
            //               i < iend;
            //               i++) {
            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
                EdgeMonoOnlyPose *e = vpEdgesMono[i];

                const size_t idx = vnIndexEdgeMono[i];
                bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

                if (pFrame->mvbOutlier[idx]) {
                    e->computeError();
                }

                const double chi2 = e->chi2();

                if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) ||
                    !e->isDepthPositive()) {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBadMono++;
                } else {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                    nInliersMono++;
                }

                if (it == 2) e->setRobustKernel(nullptr);
            }
            //        });

            // Stereo
            //    tbb::parallel_for(tbb::blocked_range<size_t>(0, vpEdgesStereo.size()),
            //                      [&](tbb::blocked_range<size_t> rvpEdgesStereo) {
            //                        for (size_t i = rvpEdgesStereo.begin(),
            //                                    iend = rvpEdgesStereo.end();
            //                             i < iend;
            //                             i++) {
            for (size_t i = 0; i < vpEdgesStereo.size(); ++i) {
                EdgeStereoOnlyPose *e = vpEdgesStereo[i];

                const size_t idx = vnIndexEdgeStereo[i];

                if (pFrame->mvbOutlier[idx]) {
                    e->computeError();
                }

                const double chi2 = e->chi2();

                if (chi2 > chi2Stereo[it]) {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBadStereo++;
                } else {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                    nInliersStereo++;
                }

                if (it == 2) e->setRobustKernel(nullptr);
            }
            //  });

            nInliers = nInliersMono + nInliersStereo;
            nBad = nBadMono + nBadStereo;

            if (optimizer.edges().size() < 10) {
                break;
            }
        }

        // Recover if not enough inliers
        if ((nInliers < 30) && !bRecInit) {
            nBad = 0;
            const float chi2MonoOut = 18.f;
            const float chi2StereoOut = 24.f;
            EdgeMonoOnlyPose *e1;
            EdgeStereoOnlyPose *e2;
            for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
                const size_t idx = vnIndexEdgeMono[i];
                e1 = vpEdgesMono[i];
                e1->computeError();
                if (e1->chi2() < chi2MonoOut)
                    pFrame->mvbOutlier[idx] = false;
                else
                    nBad++;
            }
            for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
                const size_t idx = vnIndexEdgeStereo[i];
                e2 = vpEdgesStereo[i];
                e2->computeError();
                if (e2->chi2() < chi2StereoOut)
                    pFrame->mvbOutlier[idx] = false;
                else
                    nBad++;
            }
        }

        nInliers = nInliersMono + nInliersStereo;

        // Recover optimized pose, velocity and biases
        pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                                   VP->estimate().twb.cast<float>(),
                                   VV->estimate().cast<float>());
        Vector6d b;
        b << VG->estimate(), VA->estimate();
        pFrame->mImuBias = IMU::Bias((float) b[3], (float) b[4], (float) b[5],
                                     (float) b[0], (float) b[1], (float) b[2]);

        // Recover Hessian, marginalize previous frame states and generate new prior
        // for frame
        Eigen::Matrix<double, 30, 30> H;
        H.setZero();

        H.block<24, 24>(0, 0) += ei->GetHessian();

        Eigen::Matrix<double, 6, 6> Hgr = egr->GetHessian();
        H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
        H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
        H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
        H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

        Eigen::Matrix<double, 6, 6> Har = ear->GetHessian();
        H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
        H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
        H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
        H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

        H.block<15, 15>(0, 0) += ep->GetHessian();

        int tot_in = 0, tot_out = 0;

        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            EdgeMonoOnlyPose *e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if (!pFrame->mvbOutlier[idx]) {
                H.block<6, 6>(15, 15) += e->GetHessian();
                tot_in++;
            } else
                tot_out++;
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
            EdgeStereoOnlyPose *e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if (!pFrame->mvbOutlier[idx]) {
                H.block<6, 6>(15, 15) += e->GetHessian();
                tot_in++;
            } else
                tot_out++;
        }

        H = Marginalize(H, 0, 14);

        pFrame->mpcpi = new ConstraintPoseImu(
                VP->estimate().Rwb, VP->estimate().twb, VV->estimate(), VG->estimate(),
                VA->estimate(), H.block<15, 15>(15, 15));
        delete pFp->mpcpi;
        pFp->mpcpi = nullptr;

        return nInitialCorrespondences - nBad;
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
        for (auto pKF: vpKFs) {
            if (pKF->isBad()) continue;

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

            if (pKF == pLoopKF) V4DoF->setFixed(true);

            V4DoF->setId((int) (nIDi));
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
        for (const auto &LoopConnection: LoopConnections) {
            KeyFrame *pKF = LoopConnection.first;
            const long unsigned int nIDi = pKF->mnId;
            const set<KeyFrame *> &spConnections = LoopConnection.second;
            const g2o::Sim3 Siw = vScw[nIDi];

            for (auto spConnection: spConnections) {
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
                        optimizer.vertex((int) (nIDj))));
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                        optimizer.vertex((int) (nIDi))));

                e->information() = matLambda;
                optimizer.addEdge(e);

                sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
            }
        }

        // 1. Set normal edges
        for (auto pKF: vpKFs) {
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
                int nIDj = (int) (pParentKF->mnId);

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
                        optimizer.vertex((int) (nIDi))));
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
                        optimizer.vertex((int) (nIDi))));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                        optimizer.vertex((int) (nIDj))));
                e->information() = matLambda;
                optimizer.addEdge(e);
            }

            // 1.2 Loop edges
            const set<KeyFrame *> sLoopEdges = pKF->GetLoopEdges();
            for (auto pLKF: sLoopEdges) {
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
                            optimizer.vertex((int) (nIDi))));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex((int) (pLKF->mnId))));
                    e->information() = matLambda;
                    optimizer.addEdge(e);
                }
            }

            // 1.3 Covisibility graph edges
            const vector<KeyFrame *> vpConnectedKFs =
                    pKF->GetCovisiblesByWeight(minFeat);
            for (auto pKFn: vpConnectedKFs) {
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
                                optimizer.vertex((int) (nIDi))));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex((int) (pKFn->mnId))));
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
        for (auto pKFi: vpKFs) {
            const size_t nIDi = pKFi->mnId;

            auto *Vi = dynamic_cast<VertexPose4DoF *>(optimizer.vertex((int) (nIDi)));
            Eigen::Matrix3d Ri = Vi->estimate().Rcw[0];
            Eigen::Vector3d ti = Vi->estimate().tcw[0];

            g2o::Sim3 CorrectedSiw = g2o::Sim3(Ri, ti, 1.);
            vCorrectedSwc[nIDi] = CorrectedSiw.inverse();

            Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation());
            pKFi->SetPose(Tiw.cast<float>());
        }

        // Correct points. Transform to "non-optimized" reference keyframe pose and
        // transform back with optimized pose
        for (auto pMP: vpMPs) {
            if (pMP->isBad()) continue;

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

}  // namespace ORB_SLAM3