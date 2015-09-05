/*
 * obstacleSelector.h
 *
 *  Created on: Sep 3, 2015
 *      Author: afilippow
 */

#ifndef SELECTOR_H_
#define SELECTOR_H_
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
#include <ros/package.h>
#include <ros/ros.h>
#define MAXSEGMENTS 20


typedef pcl::PointCloud<pcl::PointXYZ> mPointCloudType;
typedef pcl::PointCloud<pcl::PointXYZRGB> mPointCloudTypeColor;
typedef pcl::PointCloud<pcl::PointXYZL> mPointCloudTypeLabel;
typedef pcl::PointXYZ mPointType; 
typedef pcl::PointXYZRGB mPointTypeColor; 
typedef pcl::PointXYZL mPointTypeLabel; 
using namespace std;
struct pt_dst_lbl{
	vector<float> p;
	float d;
	int l;
	pt_dst_lbl(vector<float> par_pt, float par_d, int par_l){
		p = par_pt;
		d = par_d;
		l = par_l;
	}
	pt_dst_lbl(){
		p.resize(0);
		d = 0;
		l = 0;
	}
};


class obstacleSelector {
public:
	vector<float> asvector(mPointTypeLabel par_pt);
	float min_dist(pt_dst_lbl par_pt);
	float min_dist(vector<float> par_vec);
	float min_dist(mPointTypeLabel ptl);
	float dist(vector<float> a, vector<float> b);
	
	int currKeyFrameID;
	void callback(const sensor_msgs::PointCloud2 inputROSMsg_tracker);
	mPointCloudTypeLabel::Ptr fullCloud;
	vector< vector<float> > obstacles;
	vector<float> distances;
	vector< vector<float> > positions;
	std::string ros_topic;
	ros::Subscriber cloud_sub;
	void set_positions(vector< vector<float> > par_positions){positions = par_positions;}
	vector< vector<float> > get_obstacles() {return obstacles;}
	obstacleSelector();
	void updateObstacles();
	void subscribe(ros::NodeHandle par_handle, const string par_topic);

};

#endif /* SELECTOR_H_ */
