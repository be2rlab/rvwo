#ifndef DYNAMICDETECTOR_H
#define DYNAMICDETECTOR_H

#include"KeyFrame.h"
#include"Frame.h"
#include "Converter.h"
#include "Map.h"
#include "Atlas.h"

#include <Eigen/Dense>

namespace ORB_SLAM3 {
enum PointType{
    UNKNOWN,
    STATIC,
    DYNAMIC
};
class KeyFrame;
class Frame;
class Map;
class Converter;
class Atlas;

class DynamicDetector{
public:
    DynamicDetector();
    void removeDynamicPixels(KeyFrame* mKF);

public:
    // void SetMapper(Map* pMap);
    void SetMapper(Atlas* pAtlas);
    // Kmeans
	void computePointCloud( const cv::Mat& depth, std::vector<Eigen::Vector2d>& pt2d, std::vector<Eigen::Vector3d>& pt3d, const Eigen::Vector4d& K);
	void segmentPointCloudByKmeans( const vector< Eigen::Vector2d >& pts2d, const vector< Eigen::Vector3d >& pts3d, const int n_clusters, vector< vector< Eigen::Vector2d > >& clusters_2d, vector< vector< Eigen::Vector3d > >& clusters_3d );
	void drawClustersOnImage(cv::Mat& io_img, const int width, const int height, const vector< vector< Eigen::Vector2d > >& clusters_2d, const std::vector<uint8_t>& colors);
	
	// Detect dynamic clusters.
	void detectDynamicClusters(Map* mpMap, KeyFrame* mKF, const std::vector< std::vector< Eigen::Vector2d > >& clusters_2d, const vector< vector< Eigen::Vector3d > >& clusters_3d, std::vector<bool>& dynamic_flags );
	bool isDynamicCluster( const std::vector< Eigen::Vector2d >& cluster_2d, const vector< Eigen::Vector3d >& cluster_3d, KeyFrame* mKF, std::vector< KeyFrame* > near_kfs );
	void dynamicDepthImgCullingByMultiViewConstraint( cv::Mat& depth, const std::vector<std::vector<Eigen::Vector2d>>& clusters_2d, const std::vector<bool>& dynamic_flags );
	void drawDynamicClusters(cv::Mat& io_img, const std::vector<std::vector<Eigen::Vector2d>>& clusters_2d, const std::vector<bool>& dynamic_flags, std::vector<uint8_t> colors);

    cv::Mat drawDynamicsKeysUn(cv::Mat& io_img, KeyFrame* mKF); 
    inline bool isInFrame ( const Eigen::Vector2d & obs, int boundary=0 ) const {
     if ( obs[0]>=boundary && obs[0]<width_-boundary
      && obs[1]>=boundary && obs[1]<height_-boundary ) {
      return true;
      }
      return false;
    }
private:
    Map* mpMap;
    // --------------------------------------------------------------------------------------------
    //  Dynamic Pixels Culling
    // --------------------------------------------------------------------------------------------
    int dpc_n_near_kfs_;                 // Number of near keyframes.
    int dpc_npts_per_cluster_;        // Number of points per cluster.
    int dpc_n_sel_pts_per_cluster_;   // Number of points per cluster to be selected for dynamic cluster decision.
    int dpc_search_square_size_ ;        // 9 pixels
    
    float ret_tk_db_;                 // Base threshold for erroneous match discard. (m)
    float ret_tk_kd_;                // Scale factor of the threshold for erroneous match discard.

    int width_;
    int height_; 
    double cam_dmax_;                   //# Max depth value to be used. (m)
    double cam_dmin_;                   //# Min depth value to be used. (m) 
    //cv::Mat Trc_;
    Sophus::SE3<float> Trc_;
    std::vector<uint8_t> colors_ = {213,0,0,197,17,98,170,0,255,98,0,234,48,79,254,41,98,255,0,145,234,0,184,212,0,191,165,0,200,83,100,221,23,174,234,0,255,214,0,255,171,0,255,109,0,221,44,0,62,39,35,33,33,33,38,50,56,144,164,174,224,224,224,161,136,127,255,112,67,255,152,0,255,193,7,255,235,59,192,202,51,139,195,74,67,160,71,0,150,136,0,172,193,3,169,244,100,181,246,63,81,181,103,58,183,171,71,188,236,64,122,239,83,80, 213,0,0,197,17,98,170,0,255,98,0,234,48,79,254,41,98,255,0,145,234,0,184,212,0,191,165,0,200,83,100,221,23,174,234,0,255,214,0,255,171,0,255,109,0,221,44,0,62,39,35,33,33,33,38,50,56,144,164,174,224,224,224,161,136,127,255,112,67,255,152,0,255,193,7,255,235,59,192,202,51,139,195,74,67,160,71,0,150,136,0,172,193,3,169,244,100,181,246,63,81,181,103,58,183,171,71,188,236,64,122,239,83,80};    
}; // DynamicDetector
} // ORB_SLAM3
#endif