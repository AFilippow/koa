/*
 * segmenter.h
 *
 *  Created on: Mar 12, 2014
 *      Author: afilippow
 */

#ifndef SEGMENTER_H_
#define SEGMENTER_H_
#include <iostream>
#include <sys/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/tracking/tracking.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <sstream>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/ros/conversions.h>

#include <pcl/filters/extract_indices.h>

#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/point_cloud.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/statistical_outlier_removal.h> 
#include <pcl/filters/passthrough.h>

#define MAXSEGMENTS 20


typedef pcl::PointCloud<pcl::PointXYZ> mPointCloudType;
typedef pcl::PointCloud<pcl::PointXYZRGB> mPointCloudTypeColor;
typedef pcl::PointXYZ mPointType; 
typedef pcl::PointXYZRGB mPointTypeColor; 
using namespace std;
class pclsegmenter {
public:
	
	//void deepCopy(mPointCloudType::Ptr givenCloud, mPointCloudType::Ptr copyCloud );
	void setRawCloud(mPointCloudType::Ptr input) {rawCloud = input->makeShared();}
	mPointCloudType::Ptr getFilteredCloud() {return filteredCloud;}
	mPointCloudTypeColor::Ptr getColoredCloud() {return coloredCloud;}
	std::vector<mPointTypeColor> obstacles;
	std::vector<pcl::PointIndices> cluster_indices;
	mPointCloudType::Ptr rawCloud;
	mPointCloudType::Ptr filteredCloud;
	mPointCloudTypeColor::Ptr coloredCloud;
	//mPointCloudTypeColor::Ptr logicCloud;
	mPointCloudType *segments;
	mPointType position;

	int segmentsFound;
	bool cloudChanged;
	
	pclsegmenter();
	int segment();
	float distanceOfObstacleToPosition(pcl::PointXYZRGB obstacleWeight, mPointType position);
	void excludeObstacle(mPointCloudTypeColor::Ptr cCloud, mPointType position);	
	void excludeObstacle(mPointType paramPos){excludeObstacle(coloredCloud, paramPos);}	
	int findCentersOfMass();
	int findClosestPoints();
	void setPosition(vector<float> par_vec);
	void setPosition(float par_x, float par_y, float par_z);


};

#endif /* SEGMENTER_H_ */
