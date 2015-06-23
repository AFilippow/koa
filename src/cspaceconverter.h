#ifndef CSPACE_H_
#define CSPACE_H_
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

#include <vector>
#include <fstream>
#include <sstream>
#include <string>

#include "vectormath.h"
//OROCOS KDL 
#include "KukaLWR_DHnew.h"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <time.h>
#include <list>
//THREADS!
#include <pthread.h>
#define PI 3.141562f
#define THREADNUMBER 51
/* This nasty piece of Work holds all c-Space points that correspond to a 3d-space point o(x,y,z)
 * Point vector values are c-space coordinates: pointsData[x*50*50 + y*50 + z][pointNumber][c-Space dimension]
 * 
 * 
 */
using namespace std;
using namespace KDL;

class cspaceconverter;
struct lister_parameters{
	pthread_mutex_t* list_lock;
	pthread_mutex_t* position_lock;
	pthread_mutex_t* configuration_lock;
	vector<float>* position;
	list<vector<float> >* obstacle_list;
	list<vector<float> >* configuration_list;
	list<float>* distances_list;
	cspaceconverter* converter;
	lister_parameters(cspaceconverter* par_csp, pthread_mutex_t* par_ll, pthread_mutex_t* par_pl, pthread_mutex_t* par_cl, vector<float>* par_pos, list<vector<float> >* par_list, list<vector<float> >* par_conf, list<float>* par_dist){
		converter = par_csp;
		list_lock = par_ll;
		position_lock = par_pl;
		configuration_lock = par_cl;
		position = par_pos;
		obstacle_list = par_list;
		configuration_list = par_conf;
		distances_list = par_dist;
	}
	
};

class cspaceconverter {
	public:
	
	Chain KukaChain;
	ChainFkSolverPos_recursive * kinematic_solver;
	int jointNumber;
	KDL::Frame baseframe;	
	int max_list_size;
	std::list<vector<float> > obstacle_list;
	std::list<vector<float> > configuration_list;
	std::list<float> distances_list;
	vector<float> position;
	pthread_mutex_t list_lock;
	pthread_mutex_t position_lock;
	pthread_mutex_t configuration_lock;
	
	vector<float> get_configurations_as_vectors();
	vector<float> get_distances_as_vectors();
	void set_position(vector<float> par_pos);
	void set_obstacles(vector<vector<float> > par_obst);
	int examineDifference(vector<float> point1, vector<float> point2);
	void launch_obstacle_thread();
	vector<float> joint_to_cartesian(vector<float> jointvalues){ return joint_to_cartesian(jointvalues,-1);}
	vector<float> joint_to_cartesian(vector<float> jointvalues, int segmentnumber);

	cspaceconverter();
	//vector< vector< float > >  getCspaceObstacle(float x, float y, float z) {return pointsData[x*50*50 + y*50 + z];}

};


#endif /* CSPACE_H_ */
