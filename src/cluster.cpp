/* \author Aaron Brown */
#include "cluster.h"

void createCluster(const vector<vector<float>>& points, 
                   PointCloud<PointXYZ>::Ptr &cluster, 
                   int *flag, KdTree* tree, float distanceTol, int i)
{
    if(flag[i] == 1)
    {
        return;
    }
        
    flag[i] = 1;

    PointXYZ z = {points[i][0], points[i][1], points[i][2]};
    cluster->points.push_back(z);

    vector<int> nearPoint = tree->search(points[i], distanceTol);

    for(int i=0;i<nearPoint.size();i++)
    {
        if(flag[nearPoint[i]] == 0)
		{
            createCluster(points, cluster, flag, tree, distanceTol, nearPoint[i]);
		}
    }
}

vector<PointCloud<PointXYZ>::Ptr> euclideanCluster(const vector<vector<float>>& points, 
                                                   KdTree* tree, float distanceTol, int minSize)
{
    // Inserting point into KD-Tree
    for (int i=0; i<points.size(); i++)
    {
        tree->insert(points[i],i);
    }

    vector<PointCloud<PointXYZ>::Ptr> clusters;

    int size = points.size();
    int flag[size];

    for(int i=0;i<size;i++)
    {
        flag[i] = 0;
    }

    for(int i=0;i<size;i++)
    {
        PointCloud<PointXYZ>::Ptr cluster(new PointCloud<PointXYZ>);

        if(flag[i] == 0)
        {
            createCluster(points, cluster, flag, tree, distanceTol, i);
            if(cluster->points.size() >= minSize)
                clusters.push_back(cluster);
        }
    }
    return clusters;
}


//int main ()
//{
//	// Create viewer
//	Box window;
//  	window.x_min = -10;
//  	window.x_max =  10;
//  	window.y_min = -10;
//  	window.y_max =  10;
//  	window.z_min =   0;
//  	window.z_max =   0;
//	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25)//;
//
//	// Create data
//	std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, //{-5.2,7.1},
//											   {-5.7,6.3}, {7.2,6.1}, //{8.0,5.3},
//											   {7.2,7.1}, {0.2,-7.1}, //{1.7,-6.9},
//											   {-1.2,-7.2}, {2.2,-8.9} };
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);
//
//	KdTree* tree = new KdTree;
//
//    for (int i=0; i<points.size(); i++)
//    	tree->insert(points[i],i);
//
//  	int it = 0;
//  	render2DTree(tree->root,viewer,window, it);
//
//  	std::cout << "Test Search" << std::endl;
//  	std::vector<int> nearby = tree->search({-6,7},3.0);
//  	for(int index : nearby)
//      std::cout << index << ",";
//  	std::cout << std::endl;
//
//  	// Time segmentation process
//  	auto startTime = std::chrono::steady_clock::now();
//  	//
//  	std::vector<std::vector<int>> clusters = euclideanCluster(points, //tree, 3.0);
//  	//
//  	auto endTime = std::chrono::steady_clock::now();
//  	auto elapsedTime = //std::chrono::duration_cast<std::chrono::milliseconds>(endTime - //startTime);
//  	std::cout << "clustering found " << clusters.size() << " and took " //<< elapsedTime.count() << " milliseconds" << std::endl;
//
//  	// Render clusters
//  	int clusterId = 0;
//	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)//};
//  	for(std::vector<int> cluster : clusters)
//  	{
//  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new //pcl::PointCloud<pcl::PointXYZ>());
//
//		for (int indice: cluster)
//		{
//  			clusterCloud->points.push_back(pcl::PointXYZ(points[indice]//[0],points[indice][1],0));
//		}
//  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string//(clusterId),colors[clusterId%3]);
//
//  		++clusterId;
//  	}
//
//	if(clusters.size()==0)
//	{
//  		renderPointCloud(viewer,cloud,"data");
//	}
//
//  	while (!viewer->wasStopped ())
//  	{
//  	  viewer->spinOnce ();
//  	}
//
//}
