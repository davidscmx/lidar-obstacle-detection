

#include "ransac.h"

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const int maxIterations, const float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	const int points2fit = 3;
	for (int i = 0; i < maxIterations; i++)
	{
        // 1. Select three randon points
		std::unordered_set<int> inliers;
		for (int i = 0; i < points2fit; i++)
		{
			inliers.insert((rand() % cloud->size()));
		}

		// Get random points form the cloud
		float x1,y1,z1;
		float x2,y2,z2;
		float x3,y3,z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;

		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;

		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		// calculate
        // 2. Calculate the plane that these points form
        // Ax + By + Cz - d = 0
		const float a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		const float b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		const float c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		const float d = -(a*x1 + b*y1 + c*z1);

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for (size_t i =0; i < cloud->points.size();i++)
		{
			if ( inliers.count(i)>0 )
			{
				continue;
			}

			pcl::PointXYZ point = cloud->points[i];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

             // 3. Estimate the distance of each point and check if it is wichin distanceTol
			const float D = fabs(a*x4+ b*y4 +c*z4 + d)/sqrt(a*a + b*b + c*c);
			if (D < distanceTol)
			{
				inliers.insert(i);
			}
		}

		// the fit with the most inliers will be selected
		if (inliers.size()> inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}
