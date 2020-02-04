/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "ransac.h"
#include "kdtree.h"
//#include "cluster.cpp"

//void clusterHelper(int index,
//				   const std::vector<std::vector<float>> points,
//				   std::vector<int> &cluster,
//				   std::vector<bool> &processed,
//				   KdTree* tree,
//				   const float distanceTol
//				   )
//{
//	processed[index] = true;
//	cluster.push_back(index);
//
//	std::vector<int> nearest = tree->search(points[index], distanceTol);
//
//	for (int id: nearest)
//	{
//		if (!processed[id])
//		{
//			clusterHelper(id, points, cluster, processed, tree, distanceTol);
//		}
//	}
//}
//
//std::vector<std::vector<int>> euclideanCluster(const //std::vector<std::vector<float>> points, KdTree* tree, float distanceTol)
//{
//	// TODO: Fill out this function to return list of indices for each cluster
//	// list of clusters
//	std::vector<std::vector<int>> clusters;
//
//	std::vector<bool> processed(points.size(), false);
//	int i = 0;
//	while (i < points.size())
//	{
//		if (processed[i])
//		{
//			i++;
//			continue;
//		}
//
//		std::vector<int> cluster;
//        std::cout << "in euc points[0] " << points[0].size() << std::endl;
//		clusterHelper(i, points, cluster, processed, tree, distanceTol);
//        if (cluster.size() > 8)
//        {
//            clusters.push_back(cluster);
//        }
//
//		i++;
//	}
//
//	return clusters;
//}


void createCluster(const std::vector<std::vector<float>>& points, pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster, int *flag, KdTree* tree, float distanceTol, int i)
{
    if(flag[i] == 1)
        return;

    flag[i] = 1;

    pcl::PointXYZ z = {points[i][0], points[i][1], points[i][2]};
    cluster->points.push_back(z);

    std::vector<int> nearPoint = tree->search(points[i], distanceTol);

    for(int i=0;i<nearPoint.size();i++)
    {
        if(flag[nearPoint[i]] == 0)
            createCluster(points, cluster, flag, tree, distanceTol, nearPoint[i]);
    }
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize)
{
    // TODO: Fill out this function to return list of indices for each cluster

    // Inserting point into KD-Tree
    for (int i=0; i<points.size(); i++)
        tree->insert(points[i],i);

   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    int size = points.size();
    int flag[size];

    for(int i=0;i<size;i++)
        flag[i] = 0;

    for(int i=0;i<size;i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

        if(flag[i] == 0)
        {
            createCluster(points, cluster, flag, tree, distanceTol, i);
            if(cluster->points.size() >= minSize)
                clusters.push_back(cluster);
        }
    }
    return clusters;
}


std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    const double slope = 0.0;
    // TODO:: Create lidar sensor
    Lidar * lidar = new Lidar(cars, slope);
    lidar->cloud = lidar->scan();
    //renderRays(viewer, lidar->position, lidar->cloud);

    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    const int max_iters = 100;
    const float distanceThreshold = 0.3;

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(lidar->cloud, max_iters, distanceThreshold);

    renderPointCloud(viewer, segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second,"planeCloud",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = pointProcessor->
    loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    Eigen::Vector4f minVec = Eigen::Vector4f(-10, -6.2, -2, 1);
    Eigen::Vector4f maxVec = Eigen::Vector4f(25, 7, 10, 1);

    inputCloud = pointProcessor->FilterCloud(inputCloud, .2 , minVec, maxVec);
    // segment road

    const int maxIters = 100;
    const float distanceThreshold = 0.20;

    std::unordered_set<int> inliers = RansacPlane(inputCloud, maxIters, distanceThreshold);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < inputCloud->points.size(); index++)
	{
		pcl::PointXYZ point = inputCloud->points[index];
		if(inliers.count(index))
        {
			cloudInliers->points.push_back(point);
        }
        else
        {
		    cloudOutliers->points.push_back(point);
        }
	}

    renderPointCloud(viewer, cloudInliers,"planeCloud",Color(0,1,0));

    KdTree* tree = new KdTree;
    const int dims = 2;

    std::vector<std::vector<float>> point_vectors;
    for (int i = 0; i < cloudOutliers->points.size(); i++)
    {

        std::vector<float> tmp_vec;
        tmp_vec.push_back(cloudOutliers->points[i].x);
        tmp_vec.push_back(cloudOutliers->points[i].y);
        tmp_vec.push_back(cloudOutliers->points[i].z);

    	tree->insert(tmp_vec, i);

        point_vectors.push_back(tmp_vec);
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = euclideanCluster(point_vectors, tree, 0.3, 8);
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    int clusterId = 0;
    int render_box = 1;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : clusters)
  	{
        std::cout<<"Cluster size: ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obsCloud"+std::to_string(clusterId), colors[clusterId % colors.size()]);

        if(render_box)
        {
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        clusterId++;
  	}
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    CameraAngle setAngle = XY;

    initCamera(setAngle, viewer);

    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
}