/* 
  Create simple 3d highway enviroment using PCL
  for exploring self-driving car sensors
*/

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"

// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "ransac.h"
#include "kdtree.h"

using std::vector;

void clusterHelper(int index,
				   const vector<vector<float>> points,
				   vector<int> &cluster,
				   vector<bool> &processed,
				   KdTree* tree,
				   const int distanceTol
				   )
{
	processed[index] = true;
	cluster.push_back(index);

	vector<int> nearest = tree->search(points[index],distanceTol);

	for (int id: nearest)
	{
		if (!processed[id])
		{
			clusterHelper(id, points, cluster, processed, tree, distanceTol);
		}
	}
}

vector<vector<int>> euclideanCluster(const vector<vector<float>> points, 
                                     KdTree* tree, 
                                     float distanceTol)
{
	// list of clusters
	vector<vector<int>> clusters;

	vector<bool> processed(points.size(), false);
	int i = 0;
	while (i < points.size())
	{
		if (processed[i])
		{
			i++;
			continue;
		}

		vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
	}

	return clusters;
}


vector<Car> initHighway(bool renderScene,
                        pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    vector<Car> cars;
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
    vector<Car> cars = initHighway(renderScene, viewer);
    const double slope = 0.0;

    Lidar * lidar = new Lidar(cars, slope);
    lidar->cloud = lidar->scan();
    //renderRays(viewer, lidar->position, lidar->cloud);

    //std::unordered_set<int> inliers = RansacPlane(inputCloud, maxIters, distanceThreshold);
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    const int max_iters = 100;
    const float distanceThreshold = 0.3;

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(lidar->cloud, max_iters, distanceThreshold);

    renderPointCloud(viewer, segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second,"planeCloud",Color(0,1,0));

    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // segment road
    const int maxIters = 100;
    const float distanceThreshold = 0.20;

    std::unordered_set<int> inliers = RansacPlane(inputCloud, maxIters, distanceThreshold);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

	for(int index = 0; index < inputCloud->points.size(); index++)
	{
		pcl::PointXYZI point = inputCloud->points[index];
		if(inliers.count(index))
        {
			cloudInliers->points.push_back(point);
        }
        else
        {
		    cloudOutliers->points.push_back(point);
        }
	}

    //renderPointCloud(viewer, cloudInliers,"inliers", Color(1,0,0));
    //renderPointCloud(viewer, cloudOutliers,"outliers",Color(0,1,0));
    // kd-tree: preparation for clustering
    KdTree* tree = new KdTree;

    const int dims = 3;

    Eigen::Vector4f minPoint = Eigen::Vector4f(1.0f, 2.0f, 3.0f, 4.0f);
    Eigen::Vector4f maxPoint = Eigen::Vector4f(3.0f, 4.0f, 5.0f, 6.0f);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloudOutliers =
    pointProcessorI->FilterCloud(cloudOutliers, 0.5, minPoint, maxPoint);

    std::cout << "filteredCloudOutliers->points.size() " << filteredCloudOutliers->points.size() << std::endl;

    //renderPointCloud(viewer, filteredCloudOutliers,"outliers",Color(0,1,0));

    vector<vector<float>> point_vectors = {};
    for(int i = 0; i < filteredCloudOutliers->points.size(); i++)
    {
        // convert to vector
        vector<float> point_vec = {0,0,0};
        filteredCloudOutliers->points[i].x = point_vec[0];
        filteredCloudOutliers->points[i].y = point_vec[1];
        filteredCloudOutliers->points[i].z = point_vec[2];

    	tree->insert(point_vec, i, dims);
        point_vectors.push_back(point_vec);
    }

    vector<vector<int>> clusters = euclideanCluster(point_vectors, tree, 0.75);
    std::cout << "# of clusters found: " << clusters.size() << std::endl;
    vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    int clusterId = 0;
    for(vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());

		for (int indice: cluster)
		{
  			clusterCloud->points.push_back(pcl::PointXYZ(point_vectors[indice][0],point_vectors[indice][1],point_vectors[indice][2]));
		}
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId) , colors[clusterId%3]);
  		++clusterId;
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
    {
        viewer->addCoordinateSystem (1.0);
    }
}


int main (int argc, char** argv)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    // TopDown, Side, FPS
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    // draws a simple highway with a road, an ego vehicle and 3 cars
    simpleHighway(viewer);

    // This is the important function which does clustering
    //cityBlock()
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
}