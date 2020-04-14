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
#include "cluster.h"

using std::vector;
using pcl::PointCloud;
using pcl::PointXYZ;

vector<Car> initSimpleHighway(bool renderScene,
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


void createSimpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    vector<Car> cars = initSimpleHighway(renderScene, viewer);
    const double slope = 0.0;

    Lidar * lidar = new Lidar(cars, slope);
    lidar->cloud = lidar->scan();
    //renderRays(viewer, lidar->position, lidar->cloud);

    //std::unordered_set<int> inliers = RansacPlane(lidar->cloud, maxIters, distanceThreshold);
    ProcessPointClouds<PointXYZ> *pointProcessor = new ProcessPointClouds<PointXYZ>();

    const int max_iters = 100;
    const float distanceThreshold = 0.3;

    std::pair<PointCloud<PointXYZ>::Ptr, PointCloud<PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(lidar->cloud, max_iters, distanceThreshold);

    renderPointCloud(viewer, segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second,"planeCloud",Color(0,1,0));

    vector<PointCloud<PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(PointCloud<PointXYZ>::Ptr cluster : cloudClusters)
    {
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, 
               ProcessPointClouds<PointXYZ>* pointProcessor, 
               PointCloud<PointXYZ>::Ptr& inputCloud)
{
   
    Eigen::Vector4f minVec = Eigen::Vector4f(-10, -6.2, -2, 1);
    Eigen::Vector4f maxVec = Eigen::Vector4f(15, 7, 10, 1);
    
    inputCloud = pointProcessor->FilterCloud(inputCloud, 0.25, minVec, maxVec);
    
    // segment road
    const int maxIters = 100;
    const float distanceThreshold = 0.20;

    std::unordered_set<int> inliers = RansacPlane(inputCloud, maxIters, distanceThreshold);

    PointCloud<PointXYZ>::Ptr cloudInliers(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr cloudOutliers(new PointCloud<PointXYZ>());

	for(int index = 0; index < inputCloud->points.size(); index++)
	{
		PointXYZ point = inputCloud->points[index];
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

    vector<vector<float>> point_vectors;
    for (int i = 0; i < cloudOutliers->points.size(); i++)
    {

        vector<float> tmp_vec;
        tmp_vec.push_back(cloudOutliers->points[i].x);
        tmp_vec.push_back(cloudOutliers->points[i].y);
        tmp_vec.push_back(cloudOutliers->points[i].z);

        // insert vector in the kd-tree 
    	tree->insert(tmp_vec, i);

        point_vectors.push_back(tmp_vec);
    }
    
    vector<PointCloud<PointXYZ>::Ptr> clusters = euclideanCluster(point_vectors, tree, 0.3, 8);
    
    vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    int clusterId = 0;
    int render_box = 1;

    // loop over the found cluster and draw a bounding box around it
    for(PointCloud<PointXYZ>::Ptr cluster : clusters)
  	{
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
    {
        viewer->addCoordinateSystem (1.0);
    }
}

int main (int argc, char** argv)
{
    if (argc <= 1)
	{
	
		if (argv[0])
        {
			std::cout << "Usage: " << argv[0] << " <number>" << '\n';
        }
        else
        {
			std::cout << "Usage: <program name> <number>" << '\n';
        }
	}
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // TopDown, Side, FPS
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    
    char *action = argv[1];
    if (std::strcmp(action,"stream") == 0)
    {    
        ProcessPointClouds<PointXYZ>* pointProcessor = new ProcessPointClouds<PointXYZ>();
	    vector<boost::filesystem::path> stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_2");
	    auto streamIterator = stream.begin();
	    PointCloud<PointXYZ>::Ptr inputCloud;
    
        while (!viewer->wasStopped ())
        {
            // Clear viewer
		    viewer->removeAllPointClouds();
		    viewer->removeAllShapes();

            // Load pcd and run obstacle detection process
            inputCloud = pointProcessor->loadPcd((*streamIterator).string());
		    cityBlock(viewer, pointProcessor, inputCloud);

            streamIterator++;
		    if(streamIterator == stream.end())
            {
                streamIterator = stream.begin();
            }
			
            viewer->spinOnce ();
        }
    }
    
    else if (std::strcmp(action,"simple")==0)
    {
        createSimpleHighway(viewer);
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce();
        }
    }       
             
}