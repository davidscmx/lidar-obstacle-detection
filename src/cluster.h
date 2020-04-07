/**
 * @file cluster.h
 * @author David Sosa (david.sosa@protonmail.com)
 * @brief 
 * @version 0.1
 * @date 2020-04-06
 * 
 */


#include <chrono>
#include <string>
#include <iostream>
#include <vector>
#include "kdtree.h"



void createCluster(const vector<vector<float>>& points, 
                   PointCloud<PointXYZ>::Ptr &cluster, 
                   int *flag, KdTree* tree, float distanceTol, int i);


vector<PointCloud<PointXYZ>::Ptr> euclideanCluster(const vector<vector<float>>& points, 
                                                   KdTree* tree, float distanceTol, int minSize);
