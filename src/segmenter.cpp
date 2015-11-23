/*
 * segmenter.cpp
 *
 *  Created on: Mar 12, 2014
 *      Author: afilippow
 */

#include "segmenter.h"


pclsegmenter::pclsegmenter()
{
	segments = new mPointCloudType [MAXSEGMENTS];
	cloudChanged = false;
	position.x = 0;
	position.y = 0;
	position.z = 0;
	
}

/*void pclsegmenter::deepCopy(mPointCloudType::Ptr givenCloud, mPointCloudType::Ptr copyCloud ){
	copyCloud.reset(new mPointCloudType);
	copyCloud->height = givenCloud->height;
	copyCloud->width = givenCloud->width;
	for (unsigned int i = 0; i < givenCloud->points.size(); i++)
		copyCloud->points[i] = givenCloud->points[i];
	return;	
}*/

int pclsegmenter::segment()
{	

	if (!rawCloud)
	{
		std::cout << "no point cloud passed to segmenter, aborting segmentation \n";
		return 0;
	}
	if (rawCloud->points.size() < 10) 
	{
		std::cout << "empty point cloud passed to segmenter, aborting segmentation \n";
		return 0;
	}	

	//initialise
	filteredCloud.reset(new mPointCloudType);
	coloredCloud.reset(new mPointCloudTypeColor);
	//remove the table here
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (rawCloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.02, 1.9);
	pass.filter (*rawCloud);

	pass.setInputCloud (rawCloud);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-1, 2);
	pass.filter (*filteredCloud);
	pcl::search::KdTree<mPointType>::Ptr tree (new pcl::search::KdTree<mPointType> ());

	///this plane segmentation is unnecessary right now
	/*pcl::NormalEstimation<mPointType, pcl::Normal> ne;
	ne.setInputCloud(rawCloud);
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (0.05);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	ne.compute (*cloud_normals);

	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
	pcl::PointIndices table_inliers; // point indices belonging to the table plane
	pcl::ModelCoefficients table_coefficients;
	
		


	
	
	pcl::SACSegmentationFromNormals<mPointType,pcl::Normal> sgmtr;
	sgmtr.setOptimizeCoefficients(true);
	sgmtr.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	sgmtr.setMethodType(pcl::SAC_RANSAC);
	sgmtr.setProbability(0.99);

	// Points at less than 1cm over the plane are part of the table
	sgmtr.setDistanceThreshold (0.02);
	sgmtr.setInputCloud(rawCloud);
	sgmtr.setInputNormals(cloud_normals);
	sgmtr.segment(table_inliers, table_coefficients);


	// Now remove weird error points in the back of the vision field

	
	pcl::ExtractIndices<mPointType> filter (true); // Initializing with true will allow us to extract the removed indices
	

	filter.setInputCloud (rawCloud);
	filter.setIndices (boost::make_shared<const std::vector<int> >(table_inliers.indices));
	filter.setNegative (true);			

	
	filter.filter (*filteredCloud); */
	
	
	/*// Now remove outliers
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (filteredCloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*filteredCloud);*/
	//remove points far away
	
	/*
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (filteredCloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-0.9, 0.9);
	pass.filter (*filteredCloud);
	*/
	
	//remove arm (experimental)
	/*pcl::PassThrough<pcl::PointXYZ> pass2;
	pass2.setFilterFieldName ("z");
	pass2.setInputCloud (filteredCloud);
	pass2.setFilterLimits(-1, 0.2);
	pass2.filter (*filteredCloud);*/
		


	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	cluster_indices.clear();
	ec.setClusterTolerance (0.05); // im meters
	ec.setMinClusterSize (24);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (filteredCloud);
	ec.extract (cluster_indices);

	///copyPointCloud(*filteredCloud, *coloredCloud);
	coloredCloud->width = filteredCloud->width;
	coloredCloud->height = filteredCloud->height;
	coloredCloud->points.resize(filteredCloud->points.size());
	for (unsigned int i = 0; i < filteredCloud->points.size(); i++){
		coloredCloud->points[i].x = filteredCloud->points[i].x;
		coloredCloud->points[i].y = filteredCloud->points[i].y;
		coloredCloud->points[i].z = filteredCloud->points[i].z;
		coloredCloud->points[i].r = 0;
		coloredCloud->points[i].g = 0;
		coloredCloud->points[i].b = 0;
	}
	int R[6];
	R[0] = 100;	
	R[1] = 0;	
	R[2] = 0;	
	R[3] = 100;	
	R[4] = 100;
	R[5] = 0;	
	int G[6];
	G[0] = 0;	
	G[1] = 100;	
	G[2] = 0;	
	G[3] = 0;	
	G[4] = 100;
	G[5] = 100;	
	int B[6];
	B[0] = 0;	
	B[1] = 0;	
	B[2] = 100;	
	B[3] = 100;	
	B[4] = 0;
	B[5] = 100;	
	obstacles.resize(0);

	//this version returns the center of mass of the obstacles
	//return findCentersOfMass(cluster_indices);
	
	//this one gives the closest point to the manipulator in cluster 
	int j = findClosestPoints();
	//std::cout << j <<" obstacles found \n";
	return j;

}

int pclsegmenter::findCentersOfMass()
{
	int R[6];
	R[0] = 100;	
	R[1] = 0;	
	R[2] = 0;	
	R[3] = 100;	
	R[4] = 100;
	R[5] = 0;	
	int G[6];
	G[0] = 0;	
	G[1] = 100;	
	G[2] = 0;	
	G[3] = 0;	
	G[4] = 100;
	G[5] = 100;	
	int B[6];
	B[0] = 0;	
	B[1] = 0;	
	B[2] = 100;	
	B[3] = 100;	
	B[4] = 0;
	B[5] = 100;	
	int j = 0;
	unsigned int clusternumber, pointnumber = 0;
	//for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	for (clusternumber = 0; clusternumber < cluster_indices.size(); clusternumber++)
	{	
		obstacles.resize(j+1);
	    obstacles[j].x = 0;
	    obstacles[j].y = 0;
	    obstacles[j].z = 0;
	    float mindistance = 100000;
	    int i = 0;
	    for (pointnumber = 0; pointnumber < cluster_indices[clusternumber].indices.size(); pointnumber++)
			{
				coloredCloud->points[cluster_indices[clusternumber].indices[pointnumber]].r = R[j];
				coloredCloud->points[cluster_indices[clusternumber].indices[pointnumber]].g = G[j];
				coloredCloud->points[cluster_indices[clusternumber].indices[pointnumber]].b = B[j];
				/*float xdist = coloredCloud->points[*pit].x-position.x;
				float ydist = coloredCloud->points[*pit].y-position.y;
				float zdist = coloredCloud->points[*pit].z-position.z;
				if (xdist*xdist+ydist*ydist+zdist*zdist < mindistance)
				{
					obstacles[j].x = coloredCloud->points[*pit].x;
					obstacles[j].y = coloredCloud->points[*pit].y;
					obstacles[j].z = coloredCloud->points[*pit].z;
					mindistance = xdist*xdist+ydist*ydist+zdist*zdist;
				}	*/
				obstacles[j].x += coloredCloud->points[cluster_indices[clusternumber].indices[pointnumber]].x;
				obstacles[j].y += coloredCloud->points[cluster_indices[clusternumber].indices[pointnumber]].y;
				obstacles[j].z += coloredCloud->points[cluster_indices[clusternumber].indices[pointnumber]].z;
				i++;
				
			}
		obstacles[j].x /= (float)i;
		obstacles[j].y /= (float)i;
		obstacles[j].z /= (float)i;
	    obstacles[j].r = R[j];
	    obstacles[j].g = G[j];
	    obstacles[j].b = B[j];
	    //std::cout << "obstacle at: "<< obstacles[j].x << " " << obstacles[j].y << " " << obstacles[j].z << "\n ";
	    j++;
	}
	return j;
}

int pclsegmenter::findClosestPoints()
{

	int R[6];
	R[0] = 100;	
	R[1] = 0;	
	R[2] = 0;	
	R[3] = 100;	
	R[4] = 100;
	R[5] = 0;	
	int G[6];
	G[0] = 0;	
	G[1] = 100;	
	G[2] = 0;	
	G[3] = 0;	
	G[4] = 100;
	G[5] = 100;	
	int B[6];
	B[0] = 0;	
	B[1] = 0;	
	B[2] = 100;	
	B[3] = 100;	
	B[4] = 0;
	B[5] = 100;	
	int j = 0;
	int jprime = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{	
		//printf("index %i\n", jprime);
		//printf("size of obstacle vector: %i\n", obstacles.size());
		obstacles.resize(j+1);
	    obstacles[j].x = 0;
	    obstacles[j].y = 0;
	    obstacles[j].z = 0;
	    float mindistance = 100000;
	    int i = 0;
	    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			{
				coloredCloud->points[*pit].r = R[jprime];
				coloredCloud->points[*pit].g = G[jprime];
				coloredCloud->points[*pit].b = B[jprime];
				float xdist = coloredCloud->points[*pit].x-position.x;
				float ydist = coloredCloud->points[*pit].y-position.y;
				float zdist = coloredCloud->points[*pit].z-position.z;
				if (xdist*xdist+ydist*ydist+zdist*zdist < mindistance)
				{
					obstacles[j].x = coloredCloud->points[*pit].x;
					obstacles[j].y = coloredCloud->points[*pit].y;
					obstacles[j].z = coloredCloud->points[*pit].z;
					mindistance = xdist*xdist+ydist*ydist+zdist*zdist;
				}	
				/*
				obstacles[j].x += coloredCloud->points[*pit].x;
				obstacles[j].y += coloredCloud->points[*pit].y;
				obstacles[j].z += coloredCloud->points[*pit].z;
				* */
				i++;
				
			}
		/*obstacles[j].x /= (float)i;
		obstacles[j].y /= (float)i;
		obstacles[j].z /= (float)i;*/
	    obstacles[j].r = R[jprime];
	    obstacles[j].g = G[jprime];
	    obstacles[j].b = B[jprime];
	    //printf("Obstacle %i at: %f, %f, %f \n",j,obstacles[j].x, obstacles[j].y, obstacles[j].z );
	    j++;
	    jprime = j % 6;
	    //if (j = 5) return j;
	}
	//printf("%u Obstacles segmented \n",j);
	//printf("done\n");
	return j;
}

float pclsegmenter::distanceOfObstacleToPosition(pcl::PointXYZRGB obstacleCenter, mPointType position)
{
	if (obstacleCenter.r == 0 && obstacleCenter.g == 0 && obstacleCenter.b == 0) return -4;
	if (coloredCloud->points.size() < 10) return -3;
	if (!coloredCloud) return -2;
	float distance = 100000;
	for (unsigned int i = 0; i < coloredCloud->points.size(); i++)
	{
		if (coloredCloud->points[i].r == obstacleCenter.r &&
			coloredCloud->points[i].g == obstacleCenter.g &&
			coloredCloud->points[i].b == obstacleCenter.b )
		{
			float xDist = (coloredCloud->points[i].x-position.x)*(coloredCloud->points[i].x-position.x);
			float yDist = (coloredCloud->points[i].y-position.y)*(coloredCloud->points[i].y-position.y);
			float zDist = (coloredCloud->points[i].z-position.z)*(coloredCloud->points[i].z-position.z);
			if (sqrt(xDist + yDist + zDist) < distance) distance = sqrt(xDist + yDist + zDist);
		}
	}
	//std:: cout << distance << std::endl;
	if (distance == 100000) return -1;
	return distance;

}
void pclsegmenter::setPosition(vector<float> par_vec){
		if (par_vec.size() < 3) {
		std::cout << "Error: too small vector passed to pclsegmenter::setPosition(vector<float> par_vec). Needs to be of size >= 3 \n";	
		return;
		}
		position.x = par_vec[0];
		position.y = par_vec[1];
		position.z = par_vec[2];
	}
void pclsegmenter::setPosition(float par_x, float par_y, float par_z){
		position.x = par_x;
		position.y = par_y;
		position.z = par_z;		
	}
	
void pclsegmenter::excludeObstacle(mPointCloudTypeColor::Ptr cCloud, mPointType pos)
{
	if (cCloud->points.size() < 10) return;
	if (!cCloud) return;
	pcl::KdTree<pcl::PointXYZRGB>::Ptr tree_ (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
	tree_->setInputCloud(cCloud);
	std::vector<int> nn_indices (1);
	std::vector<float> nn_dists (1);
	mPointTypeColor excludedPos;
	excludedPos.x = pos.x;
	excludedPos.y = pos.y;
	excludedPos.z = pos.z;
	excludedPos.r = 0;
	excludedPos.g = 0;
	excludedPos.b = 0;
	tree_->nearestKSearch(excludedPos, 1, nn_indices, nn_dists); 
	if ((cCloud->points[nn_indices[0]].x- excludedPos.x)*(cCloud->points[nn_indices[0]].x- excludedPos.x)+(cCloud->points[nn_indices[0]].y- excludedPos.y)*(cCloud->points[nn_indices[0]].y- excludedPos.y)+(cCloud->points[nn_indices[0]].z- excludedPos.z)*(cCloud->points[nn_indices[0]].z- excludedPos.z) > 0.05*0.05) return;
	for (unsigned int i = 0; i < obstacles.size(); i++)
		if (obstacles[i].r == cCloud->points[nn_indices[0]].r 
		&& obstacles[i].g == cCloud->points[nn_indices[0]].g 
		&& obstacles[i].b == cCloud->points[nn_indices[0]].b)
			{
				//std::cout << "ignoring obstacle at: " << obstacles[i].x << " "<< obstacles[i].y << " "<< obstacles[i].z << "\n";
				obstacles.erase(obstacles.begin() + i);
				return;
			}
}
