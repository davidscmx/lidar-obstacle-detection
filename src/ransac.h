

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <unordered_set>


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,                                     const int maxIterations,
                                    const float distanceTol);