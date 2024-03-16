#include "DynamicDetector.h"

namespace ORB_SLAM3 
{
DynamicDetector::DynamicDetector(){
    dpc_n_near_kfs_= 1;                 // Number of near keyframes.
    dpc_npts_per_cluster_= 4000;        // Number of points per cluster.
    dpc_n_sel_pts_per_cluster_ = 300;   // Number of points per cluster to be selected for dynamic cluster decision.
    dpc_search_square_size_ = 10;        // 9 pixels
    
    // ret_tk_db_= 0.20;                 // Base threshold for erroneous match discard. (m)
    // ret_tk_kd_= 0.025;                // Scale factor of the threshold for erroneous match discard.

    ret_tk_db_= 0.02;                    // Base threshold for erroneous match discard. (m)
    ret_tk_kd_= 0.0;                     // Scale factor of the threshold for erroneous match discard.

    width_ =  640;
    height_ = 480; 
    cam_dmax_= 2;                      //# Max depth value to be used. (m)
    cam_dmin_= 0.01;                     //# Min depth value to be used. (m) 
    // Trc_ = cv::Mat::eye(4,4, CV_32F);
    Eigen::Quaternion<float> qrc(1, 0, 0, 0);
    Eigen::Vector3f trc(0, 0, 0);
    Trc_ = Sophus::SE3<float>(qrc, trc);
};

void DynamicDetector::SetMapper(Atlas* pAtlas)
{
    mpMap = pAtlas->GetCurrentMap();
}

void DynamicDetector::removeDynamicPixels(KeyFrame* mKF)
{
    // Step 1. Detect moving object and Culling it
    
    // Step 2. Segment the depth image using K-means.
    std::vector<Eigen::Vector2d> pts2d;
    std::vector<Eigen::Vector3d> pts3d;
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    computePointCloud (mKF->depth_, pts2d, pts3d, Eigen::Vector4d (mKF->fx, mKF->fy,mKF->cx,mKF->cy) );
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    double tComputePointCloud = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(t2 - t1).count();
    // cout << "pts2d size: " << pts2d.size() << endl; 
    // If no enough pts remain.
    if ( pts2d.size() <= 5 ) {
        return;
    }
    // if empty near kfs
    if ((mpMap->GetNNearKeyFrames(mKF,dpc_n_near_kfs_).empty()))
    {
        cout << "Return rmDynPixels due to empty near kfs" << endl;
        return;
    }

    cout << "------------------------- " << endl;
    // Calc Number of clusters.
    int K_clusters = std::ceil ( ( double ) pts2d.size() / ( double ) dpc_npts_per_cluster_ );
    
    std::vector<std::vector<Eigen::Vector2d>> clusters_2d;
    std::vector<std::vector<Eigen::Vector3d>> clusters_3d;

    std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
    segmentPointCloudByKmeans ( pts2d, pts3d, K_clusters, clusters_2d, clusters_3d );
    std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
    double tSegKmeans = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(t4 - t3).count();
    //// draw
    // drawClustersOnImage ( mKF->image_source_clusters_, width_, height_, clusters_2d, colors_ );

    // Step 3. Detect dynamic clusters based on multi-view geometry consistant.
    std::vector<bool> dynamic_flags;
    std::chrono::high_resolution_clock::time_point t5 = std::chrono::high_resolution_clock::now();
    detectDynamicClusters (mpMap, mKF, clusters_2d, clusters_3d, dynamic_flags );
    dynamicDepthImgCullingByMultiViewConstraint ( mKF->depth_, clusters_2d, dynamic_flags );
    
    // mKF->selectStaticFeatures();
    std::chrono::high_resolution_clock::time_point t6 = std::chrono::high_resolution_clock::now();
    double tDetect = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(t6 - t5).count();

    cout << "tComputePointCloud = " << tComputePointCloud << " ms" << endl;
    cout << "tSegKmeans = " << tSegKmeans << " ms" << endl;
    cout << "tDetect = " << tDetect << " ms" << endl;

    mKF->selectStaticFeatures();
    // // draw.
    drawDynamicClusters ( mKF->image_static_clusters_, clusters_2d, dynamic_flags, colors_ );
    // cv::Mat dynImg = mKF->gray_.clone();
    // cvtColor(dynImg,dynImg,CV_GRAY2BGR);
    // dynImg = drawDynamicsKeysUn(dynImg,mKF);
    // draw mask for visualization.
    // drawResult ( mKF->depth_, dynamic_objects, clusters_2d, dynamic_flags, mKF->image_mask_ );
    // cv::imshow("Statics Clusters",mKF->image_static_clusters_); 
    // cv::imshow("Dynamics KeysUn", dynImg);
    // cv::waitKey(1);
}

/*****************************************************************
 * Step2. Segment by Kmeans
 *****************************************************************/
void DynamicDetector::computePointCloud (const cv::Mat& depth, std::vector<Eigen::Vector2d>& pt2d, std::vector<Eigen::Vector3d>& pt3d, const Eigen::Vector4d& K )
{
    //cout << "d_range: " << cam_dmin_ << "to " << cam_dmax_ << endl; 
    //std::cout<<K<<std::endl; here seems fine
    // I have a question
    // where do we assign pt2d and pt3d to the KeyFrame ? one more thing
    pt3d.clear();
    pt2d.clear();
    for ( int v = 0; v < depth.rows; v ++ ) {
        for ( int u = 0; u < depth.cols; u ++ ) {
            float dp = depth.at<float> ( v,u );
            if ( ( dp > cam_dmin_ ) && ( dp < cam_dmax_ ) ) {
                Eigen::Vector3d ptc = Converter::img2Cam ( Eigen::Vector2d ( u, v ), dp,  K); // This is new smt like unproject I see 
                pt3d.push_back ( ptc );
                pt2d.push_back ( Eigen::Vector2d ( u, v ) );
            }
        } // for all pixels.
    } //  for all pixels.
} // computePointCloud

void DynamicDetector::segmentPointCloudByKmeans ( const vector< Eigen::Vector2d >& pts2d, const vector< Eigen::Vector3d >& pts3d, const int n_clusters, vector< vector< Eigen::Vector2d > >& clusters_2d, vector< vector< Eigen::Vector3d > >& clusters_3d )
{
    // Convert
    cv::Mat points ( pts3d.size(), 3, CV_32F, cv::Scalar ( 0,0,0 ) ); 
    cv::Mat centers ( n_clusters, 1, points.type() );

    // Convert to opencv type
    for ( size_t i = 0; i < pts3d.size(); i ++ ) {
        const Eigen::Vector3d& ept = pts3d.at ( i );
        points.at<float> ( i, 0 ) = ept[0];
        points.at<float> ( i, 1 ) = ept[1];
        points.at<float> ( i, 2 ) = ept[2];
    } // for all points


    // Do Kmeans
    cv::Mat labels;
    cv::TermCriteria criteria = cv::TermCriteria ( cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1.0 );
    cv::kmeans ( points, n_clusters, labels, criteria, 3, cv::KMEANS_PP_CENTERS, centers );//

    // Collect clusters.
    clusters_2d.clear();
    clusters_3d.clear();

    clusters_2d.resize ( n_clusters );
    clusters_3d.resize ( n_clusters );

    for ( size_t i = 0; i < pts3d.size(); i ++ ) {
        int label_idx = labels.at<int> ( i, 0 );
        clusters_2d.at ( label_idx ).push_back ( pts2d.at ( i ) );
        clusters_3d.at ( label_idx ).push_back ( pts3d.at ( i ) );
    }

} // segmentPointCloudByKmeans


void DynamicDetector::drawClustersOnImage ( cv::Mat& io_img, const int width, const int height, const vector< vector< Eigen::Vector2d > >& clusters_2d, const std::vector<uint8_t>& colors )
{
    io_img = cv::Mat ( height, width, CV_8UC3, cv::Scalar ( 0, 0, 0 ) );

    for ( size_t i = 0; i < clusters_2d.size(); i ++ ) {
        uint8_t r = colors.at ( i * 3 );
        uint8_t g = colors.at ( i * 3 + 1 );
        uint8_t b = colors.at ( i * 3 + 2 );

        // draw
        const std::vector<Eigen::Vector2d>& pts = clusters_2d.at ( i );
        for ( size_t j = 0; j < pts.size(); j ++ ) {
            const Eigen::Vector2d& pt = pts.at ( j );
            io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [0] = r;
            io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [1] = g;
            io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [2] = b;
        }
    }
} // drawClustersOnImages



/*****************************************************************
 * Step 3. Detect dynamic clusters use multi-view geometry.
 *****************************************************************/
void DynamicDetector::detectDynamicClusters (Map* pMap, KeyFrame* ckf, const vector< vector< Eigen::Vector2d > >& clusters_2d, const vector< vector< Eigen::Vector3d > >& clusters_3d, vector< bool >& dynamic_flags )
{
    // get last N keyframes.
    std::vector<KeyFrame*> near_kfs;
	near_kfs = pMap->GetNNearKeyFrames(ckf, dpc_n_near_kfs_);
    if(!near_kfs.empty())
    {
        // cout << "Size of Near KFs: " <<near_kfs.size() << endl;
        for ( size_t i = 0; i < clusters_2d.size(); i ++ ) {
            // cout <<"---- New Cluster---------" << endl;
            const std::vector<Eigen::Vector2d>& cluster_2d = clusters_2d.at ( i );
            const std::vector<Eigen::Vector3d>& cluster_3d = clusters_3d.at ( i );
            bool is_dynamic = isDynamicCluster ( cluster_2d, cluster_3d, ckf, near_kfs );
            dynamic_flags.push_back ( is_dynamic );
        }
    }
    else
        cout << "Empty Covisibility KeyFrames" << endl;
} // detectDynamicClusters

bool DynamicDetector::isDynamicCluster ( const vector< Eigen::Vector2d >& cluster_2d, const vector< Eigen::Vector3d >& cluster_3d, KeyFrame* ckf, std::vector< KeyFrame* > near_kfs )
{
    const Eigen::Vector4d K_ = Eigen::Vector4d (ckf->fx, ckf->fy,ckf->cx,ckf->cy);
    const int n_test_points = dpc_n_sel_pts_per_cluster_;
    const int half_search_square_size = dpc_search_square_size_ / 2;
    //const double dist_th = cfg_->dpc_dist_threshold_;
	double dist_th = 0.0;

    std::vector<Eigen::Vector2d> test_pts_2d;
    std::vector<Eigen::Vector3d> test_pts_3d;

    // select n test points.
    if ( cluster_2d.size() <= n_test_points ) {
        test_pts_2d = cluster_2d;
        test_pts_3d = cluster_3d;
    } else {
        int step = cluster_2d.size()  / n_test_points;
        for ( size_t i = 0; i < cluster_2d.size(); i +=step ) {
            test_pts_2d.push_back ( cluster_2d.at ( i ) );
            test_pts_3d.push_back ( cluster_3d.at ( i ) );
        }
    }

    // project each selected points to every near keyframe for comparing.
    // const Sophus::SE3& cTwr = ckf->getSE3Pose(); // current frame pose.
    // const Sophus::SE2& cTwr2 = ckf->getSE2Pose();
    //const cv::Mat& cTwr = ckf->GetPose(); // current frame pose.
    const Sophus::SE3<float>& cTwr = ckf->GetPose(); // current frame pose
    int n_unknown = 0;
    int n_static = 0;
    int n_dynamic = 0;

    for ( size_t np = 0; np < test_pts_2d.size(); np ++ ) {

        // points in current kf.
        const Eigen::Vector2d& pt2d = test_pts_2d.at ( np );
        const Eigen::Vector3d& pt3d = test_pts_3d.at ( np );
        // const Eigen::Vector3d pt3dw =  cTwr * Trc_* pt3d; // point in 3d.
        const Eigen::Vector3d pt3dw = Converter::toSE3Quat(cTwr * Trc_).map(pt3d);
        PointType pt_type = UNKNOWN;

        int pt_n_unknown = 0;
        int pt_n_static = 0;
        int pt_n_dynamic = 0;

        for ( size_t nk = 0; nk < near_kfs.size(); nk ++ ) {
            // near kf.
            KeyFrame* nkf = near_kfs.at ( nk );
            const cv::Mat& ndepth = nkf->depth_;

            // continue when no depth.
            if ( ndepth.empty() ) {
                continue;
            }

            // const Sophus::SE3& nTwr = nkf->getSE3Pose();
            // const Sophus::SE2& nTwr2 = nkf->getSE2Pose();
            // const cv::Mat& nTwr = ckf->GetPose();
            const Sophus::SE3<float>& nTwr = ckf->GetPose();
            // project point 2 near kf.
            // Eigen::Vector3d ptc = ( nTwr * Trc_ ).inverse() * pt3dw;
            Eigen::Vector3d ptc = Converter::toSE3Quat((cTwr * Trc_).inverse()).map(pt3d);
			// Point must be in front of the KF.
            if ( ptc[2] < 0.1 ) {
                continue;
            }
            // const Eigen::Vector4d nK = Eigen::Vector4d(ckf->fy, ckf->fy,ckf->cx,ckf->cy);
            const Eigen::Vector2d nu = Converter::cam2Img ( ptc, K_ );

            // check if is in frame.
            if ( isInFrame ( nu ) ) {
                continue;
            }

            // search in nxn square for best match.
            const double double_max = 1.0e10;
            double min_dist = double_max; // min_dist.
            double z_primer = 0.0;
            for ( int u = nu[0]-half_search_square_size; u<=nu[0]+half_search_square_size; u++ ) {
                for ( int v=nu[1]-half_search_square_size; v<=nu[1]+half_search_square_size; v++ ) {

                    // check if is in frame.
                    if ( isInFrame ( Eigen::Vector2d ( u, v ) ) ) {
                        continue;
                    }

                    float dp = ndepth.at<float> ( v,u );
                    if ( ( dp > cam_dmin_ ) && ( dp < cam_dmax_ ) ) {
                        const Eigen::Vector3d nptc = Converter::img2Cam ( Eigen::Vector2d ( u, v ), dp, K_ );
                        // const Eigen::Vector3d nptw = nTwr * Trc_ * nptc;
                        const Eigen::Vector3d nptw = Converter::toSE3Quat(nTwr * Trc_).map(nptc);
                        // compute distance.
                        const double dist = ( nptw - pt3dw ).norm();
						
                        // update min_dist
                        if ( dist < min_dist ) {
                            min_dist = dist;
							z_primer = dp;
                        }
                    } // check if depth is ok.

                } // for a square
            } // for a square.

            // check if xx is static or dynamic.
            if ( min_dist != double_max ) {

				dist_th = ret_tk_db_ + ret_tk_kd_ * z_primer;
				
                if ( min_dist > dist_th ) {
                    pt_n_dynamic ++;
                    break;
                } else {
                    pt_n_static ++;
                }
            } else {
                pt_n_unknown ++;
            }
        } // for all near kfs.
        
        // define the pt_type
        if ( pt_n_static == 0 && pt_n_dynamic == 0 ) {
            pt_type = UNKNOWN;
        } else if ( pt_n_dynamic > pt_n_static ) {
            pt_type = DYNAMIC;
        } else {
            pt_type = STATIC;
        }

        // Count the number of different types of points
        if ( pt_type == UNKNOWN ) {
            n_unknown++;
        } else if ( pt_type == STATIC ) {
            n_static ++;
        } else if ( pt_type == DYNAMIC ) {
            n_dynamic ++;
        }

    } // for all test pts.
    // cout << "num of pt_n_unknown: " << pt_n_unknown << endl;
    // cout << "num of n_static: " << n_static << endl;
    // if(n_dynamic > 0)
        // cout << "num of n_dynamic: " << n_dynamic << endl;
    
    // Decision the cluster type.
    if ( n_static == 0 && n_dynamic == 0 ) {
        return false;
    } else if ( n_dynamic >= n_static ) {
        // cout << "This cluster is dynamic with " << n_dynamic << " pts " << endl;
        return true;
    } else {
        return false;
    }

} // isDynamicCluster

cv::Mat DynamicDetector::drawDynamicsKeysUn(cv::Mat& io_img, KeyFrame* mKF)
{
    std::vector<cv::KeyPoint> mvKFMovKeys = mKF->mvDynKeysUn;
    const float r = 5;
    int nm = mvKFMovKeys.size();
    for(int i=0;i<nm;i++)
    {
        
        cv::Point2f pt1,pt2;
        pt1.x=mvKFMovKeys[i].pt.x-r;
        pt1.y=mvKFMovKeys[i].pt.y-r;
        pt2.x=mvKFMovKeys[i].pt.x+r;
        pt2.y=mvKFMovKeys[i].pt.y+r;

        // This is a moving point in the KF
        cv::rectangle(io_img,pt1,pt2,cv::Scalar(0,0,255));
        cv::circle(io_img,mvKFMovKeys[i].pt,2,cv::Scalar(0,0,255),-1);
    }
    
    return io_img;
}

void DynamicDetector::drawDynamicClusters ( cv::Mat& io_img, const vector< vector< Eigen::Vector2d > >& clusters_2d, const vector< bool >& dynamic_flags, std::vector<uint8_t> colors )
{
    io_img = cv::Mat ( 480, 640, CV_8UC3, cv::Scalar ( 0, 0, 0 ) );
    for ( size_t i = 0; i < dynamic_flags.size(); i ++ ) {
        
        uint8_t r = colors.at ( i * 3 );
        uint8_t g = colors.at ( i * 3 + 1 );
        uint8_t b = colors.at ( i * 3 + 2 );
        
        if ( dynamic_flags.at ( i ) ) {

            const std::vector<Eigen::Vector2d>& cluster_2d = clusters_2d.at ( i );

            for ( int np = 0; np < cluster_2d.size(); np ++ ) {
                const Eigen::Vector2d& pt = cluster_2d.at ( np );

                io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [0] = r;
                io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [1] = g;
                io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [2] = b;
            } // for all pixels in one cluster.
        } // is is dynamic
    } // for all clusters.

} // drawDynamicClusters


void DynamicDetector::dynamicDepthImgCullingByMultiViewConstraint ( cv::Mat& depth, const vector< vector< Eigen::Vector2d > >& clusters_2d, const vector< bool >& dynamic_flags )
{
    int dyn_count = 0;
    for ( size_t i = 0; i < dynamic_flags.size(); i ++ ) {

        if ( dynamic_flags.at ( i ) ) {

            const std::vector<Eigen::Vector2d>& cluster_2d = clusters_2d.at ( i );

            for ( int np = 0; np < cluster_2d.size(); np ++ ) {
                const Eigen::Vector2d& pt = cluster_2d.at ( np );
                depth.at<float> ( pt[1], pt[0] ) = -1.0f;

            } // for all pixels in one cluster.
            dyn_count++;
        } // is is dynamic
    } // for all clusters.
    cout << "Number of Dyn Clusters: " << dyn_count << endl;
} // dynamicDepthImgCullingByMultiViewConstraint
}