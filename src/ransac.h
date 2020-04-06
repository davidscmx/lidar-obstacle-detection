/**
 * @file ransac.h
 * @author David Sosa (david.sosa@protonmail.com)
 * @brief 
 * @version 0.1
 * @date 2020-04-06
 * 
 */

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <unordered_set>

/**
 * @brief Uses the RANSAC algorithm to find the best plane for the road 
 * 
 * @param cloud          Point cloud in the PCL format
 * @param maxIterations  Maximum number of interatios of RANSAC
 * @param distanceTol    Distance tolerance
 * @return std::unordered_set<int> 
 */

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
                                    const int maxIterations,
                                    const float distanceTol);