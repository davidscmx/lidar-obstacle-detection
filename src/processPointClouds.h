/**
 * @file processPointClouds.h
 * @author David Sosa (david.sosa@protonmail.com)
 * @brief PCL lib Functions for processing point clouds
 * @version 0.1
 * @date 2020-04-06
 * 
 */

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>

using pcl::PointCloud;

template<typename PointT>
class ProcessPointClouds {
public:

    ProcessPointClouds();
        
    ~ProcessPointClouds();

    void numPoints(typename PointCloud<PointT>::Ptr cloud);

    typename PointCloud<PointT>::Ptr FilterCloud(typename PointCloud<PointT>::Ptr cloud, 
                                                      float filterRes, 
                                                      Eigen::Vector4f minPoint, 
                                                      Eigen::Vector4f maxPoint);

    std::pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename PointCloud<PointT>::Ptr cloud);

    std::pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> SegmentPlane(typename PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename PointCloud<PointT>::Ptr> Clustering(typename PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename PointCloud<PointT>::Ptr cluster);

    void savePcd(typename PointCloud<PointT>::Ptr cloud, std::string file);

    typename PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

};

#endif /* PROCESSPOINTCLOUDS_H_ */