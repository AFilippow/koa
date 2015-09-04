// those next two should be always called first!....  
#include <iostream> 
#include <cstring>
#include <sys/time.h>  
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/tracking/tracking.h>
#include <pcl/common/distances.h>
#include <pcl/search/kdtree.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>

// ROS
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h> 

#include <sstream>  

#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <sys/time.h> 
 
#include <pcl/visualization/pcl_visualizer.h>  
#include <../../opt/ros/hydro/include/ros/init.h>
#include <sstream> 
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/point_cloud.h>   
#include <stdio.h>
#include <stdlib.h>  
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
   
#include "obstacleSelector.h"
#include "xdmp.h"
//#include "vrepComm.h"
#include "rosComm.h"
#include <time.h> 

#include "cspaceconverter.h"
#include "vectormath.h"
//SocketComm was written to communicate with the Morse simulator, V-REP does not need it
//#include "SocketComm.h"

#define NIL (0) 
#define PI 3.14152
typedef pcl::PointCloud<pcl::PointXYZ> mPointCloudType;
typedef pcl::PointCloud<pcl::PointXYZRGB> mPointCloudTypeColor;
typedef pcl::PointXYZ mPointType; 
obstacleSelector oSelect;
mPointCloudType::Ptr currCloud;  
mPointCloudTypeColor::Ptr obstacleCloud;
int dmpDimensions;
boost::mutex m_keycloud;
int currKeyFrameID;
vector<float> y;
vector<float> rsy;
bool mode;
float angle;  
bool perfectObstacle;  
bool fixedcloud;
int dmp_dim;
float T; //seconds
float t;
float tau;
float dt; //seconds
int n; 
float sigma; 
float mass_center_distance;
float closest_distance;
vector<float> obst(0);
vector<float> dist(0);  
KDL::JntArray robot_joints(7);  
std::vector<mPointTypeColor> previousObstacles; 
std::vector<float> obstaclePersistenceWeight;
vector<float> s;
vector<float> e;
tf::TransformListener* listener;
tf::TransformListener* kukabaseListener;
tf::StampedTransform * frameTransform;
tf::StampedTransform * kukaBaseTransform;
xDMP dmp;
bool active;
cspaceconverter * CSP;

FILE * cspobst;
FILE * cspobst2;
FILE * cspobst3;

float randomNumber() {//between 0 and 1
	return ((float) (rand()%10000))/10000;
}


vector<float> partialKinematic(KDL::JntArray q, float linknumber){
	
	KDL::Frame baseframe(KDL::Rotation::Quaternion(-0.444, 0.231, 0.40, 0.768), KDL::Vector(-0.4, 0.15, 0.35) ) ;
	Chain KukaChain = KukaLWR_DHnew();
	ChainFkSolverPos_recursive kinematic_solver(KukaChain);
	KDL::Frame cartpos;
	kinematic_solver.JntToCart(q, cartpos, linknumber);
	KDL::Vector position = baseframe*cartpos.p;

	vector<float> rtrn(3);
	for (int i = 0; i < 3; i++)
		rtrn[i] = position(i);
	return rtrn;
}
vector<float> partialKinematic(vector<float> q, float linknumber){
	KDL::JntArray q2(q.size());
	for (int i = 0; i < q.size(); i++)
		q2(i) = q[i];
	return partialKinematic(q2, linknumber);
	
}
vector<float> fullKinematic(vector<float> q){
	return partialKinematic(q, q.size());
}
vector<float> fullKinematic(KDL::JntArray q){
	KDL::Frame baseframe(KDL::Rotation::Quaternion(-0.444, 0.231, 0.40, 0.768), KDL::Vector(-0.4, 0.15, 0.35) ) ;
	Chain KukaChain = KukaLWR_DHnew();
	ChainFkSolverPos_recursive kinematic_solver(KukaChain);
	KDL::Frame cartpos;
	kinematic_solver.JntToCart(q, cartpos);
	KDL::Vector position = baseframe*cartpos.p;

	vector<float> rtrn(3);
	for (int i = 0; i < 3; i++)
		rtrn[i] = position(i);
	return rtrn;
}
void setPerfectObstacle(vector<float> position){

	///Use this only if not using pregeneration

	position.resize(7);

	position[6] = 0;
	obst.resize(3); //2 dimensions
	dist.resize(1);	
	obst[0] = 0.25;
	obst[1] = 0.125;
	obst[2] = 0.1;
	dist[0] = 0;
	
	KDL::JntArray q(7);
	q(0) = position[0];
	q(1) = position[1];
	q(2) = position[2];
	q(3) = position[3];
	q(4) = position[4];
	q(5) = position[5];
	q(6) = 0;
	
	vector< float > r = fullKinematic(q);


	dist[0] = sqrt((r[0]-obst[0])*(r[0]-obst[0])+(r[1]-obst[1])*(r[1]-obst[1])+(r[2]-obst[2])*(r[2]-obst[2]));
	///Later, we shall use one obstacle below the arm to force it away from the table
/*	obst[3] = spacepos[0];
	obst[4] = spacepos[1];
	obst[5] = 0.01;
	dist[1] = 0;


	dist[1] = dist[1] + 0.05;
*/
	///[TODO] fix distances
}
void load_null_trajectory(int n, int dim, xDMP *dmp){
	float** w = new float*[dim];
	for (int i = 0; i < dim; i++)
		w[i] = new float[n];



	for (int i = 0; i < n; i ++)
	{
		for (int j = 0; j < dim; j++)
			w[j][i] = 0;
	}
	int i = 0;
	vector<float> wtemp;
	wtemp.resize(dim);
    for (int i=0; i<n; i++){
		for(int j = 0; j < dim; j++)
			wtemp[j] = w[j][i];
	   dmp->set_w(i, wtemp);
	}
   	for (int i = 0; i < dim; i++)
		delete [] w[i];
}

void load_trajectory(char* filein, int n, int dim, xDMP *dmp)
{   

	string tempStr;
	fstream fileIn(filein);
	float** w = new float*[dim];
	for (int i = 0; i < dim; i++)
		w[i] = new float[n];



	for (int i = 0; i < n; i ++)
	{
		for (int j = 0; j < dim; j++)
			w[j][i] = 0;
	}
	int i = 0;
	while (!fileIn.eof() && i < n)
	{
		int parsedCharacters = 0;
		getline(fileIn,tempStr);
		for (int j = 0; j < dim; j++)
		{
			int a;
			if (sscanf(tempStr.c_str()+parsedCharacters,"%f %n", &w[j][i], &a) > 0)
			{
				parsedCharacters += a;
			}
			else 
			{
				w[j][i] = 0;
				printf("parse failed");
			}
		}
		i++;
   }
   fileIn.close();
	
	vector<float> wtemp;
	wtemp.resize(dim);
   for (int i=0; i<n; i++)
	{
		for(int j = 0; j < dim; j++)
			wtemp[j] = w[j][i];
	   dmp->set_w(i, wtemp);
	}
   	for (int i = 0; i < dim; i++)
		delete [] w[i];
}


///This function retrieves the obstacles from the segmenter in real time
void getLiveObstacleData(float t){
	obst.resize(0);
	dist.resize(0);
	for (unsigned int i = 0; i < oSelect.obstacles.size(); i++)
	{
		obst.push_back(oSelect.obstacles[i][0]);
		obst.push_back(oSelect.obstacles[i][1]);
		obst.push_back(oSelect.obstacles[i][2]);
		dist.push_back(oSelect.distances[i]);

	}
	/*obst.push_back(rsy[0]);
	obst.push_back(rsy[1]);
	obst.push_back(0.01);
	dist.push_back(rsy[2]-0.01);*/
}


void initGlobalParams(int argc, char** argv){
	srand(time(NULL));
	mass_center_distance = 0;
	closest_distance = 0;
	mode = false;
	perfectObstacle = false;
	fixedcloud = false;
	std::cout << "Initialising point cloud...\n";
	
	//frameTransform = new tf::StampedTransform( tf::Transform::getIdentity() , ros::Time::now(), "world", "kuka");
	//kukaBaseTransform = new tf::StampedTransform( tf::Transform::getIdentity() , ros::Time::now(), "world", "kuka_base");
}

void initDMP(){
	dmpDimensions = 4;
	//printf("currently at %s \n", __func__);
	y.resize(dmpDimensions);
	std::cout << "Initialising dmp\n";
	dmp_dim=dmpDimensions;
	T=3; //seconds
	tau=1;
	dt=0.005; ///change here important ----- seconds
	n=3; //number of cores
	sigma=1; 
	//Start and endpoint
	s.resize(dmp_dim);



	s[0] =-0.71; ///7 dimensions
	s[1] =-0.72;
	s[2] =0.13;
	s[3] =1.59;
	//s[4] =0.12;
	//s[5] =1,3;
	//s[6] =-0.42345395684242;
	e.resize(dmp_dim);
	e[0] =-0.84;
	e[1] =-1.5;
	e[2] =-0.3;
	e[3] =1.5;
	//e[4] =0.10;
	//e[5] =1.43;
	//e[6] =-0.26153537631035;
	//important!
	/*s.resize(dmpDimensions);
	e.resize(dmpDimensions);
	s[0] = -1;
	s[1] = 0.01;
	s[2] = 0;
	e[0] = 1;
	e[1] = 0.01;
	e[2] = 0;*/
	
	dmp.init_dmp(dmp_dim, s, e, T, dt, tau, n, sigma);

}


float vectorarraydistance(KDL::JntArray q1, vector<float> q2){
	float result = 0;
	for (int i = 0; i < dmp_dim; i++)
		result += (q1(i)-q2[i])*(q1(i)-q2[i]);
	return sqrt(result);
}

void find_conf(int type){
	vector<float> rs(3, 0);
	rs[0] = 0.21;
	rs[1] = 0.45;
	rs[2] = 0.09;
	vector<float> re(3, 0);
	if (type != 0){
		re[0] = -0.1+randomNumber()*0.2;
		re[1] = 0.10+((float)type)*0.4;
		re[2] = 0.12;
	}
	else{
		re[0] = 0;
		re[1] = -0.15;
		re[2] = 0.12;
	}
	//printf("rs is: %f, %f, %f; re is: %f, %f, %f \n", rs[0], rs[1], rs[2], re[0], re[1], re[2]);
	
	vector<vector<float> > par_obst;
	if (obst.size() > 1){
		par_obst.resize(obst.size()/3);
		for (int i = 0; i < obst.size(); i += 3){
			par_obst[i/3].resize(3);
			for (int j = 0; j < 3; j++)
				par_obst[i/3][j] = obst[i+j];
		}
	}
	//get s:
	s.resize(0);
	while (s.size() < 3){
		if (type == 0){
			s = CSP->get_smoothest_configuration(rs);
			//printf("Set s w.o. obstacle to %f, %f, %f, %f, corr. to point %f, %f, %f\n", s[0], s[1], s[2], s[3], rs[0], rs[1], rs[2]);
		} else {
			s = e;
			//printf("Set s to previous e\n" );
		}
		for ( int i = 0; i < 3; i++)
			rs[i] += -0.1+0.2*randomNumber();
	}
	//get e:
	e.resize(0);
	//printf("Searching end configuration ");
	while(e.size() < 3){
		//printf(".");
		if (obst.size() > 1)
			e = CSP->get_safest_configuration(re, par_obst);
		else
			e = CSP->get_smoothest_configuration(re);
		for ( int i = 0; i < 3; i+= 2)
			re[i] *= -0.1+0.2*randomNumber();
	}
	//printf(" done\n");
	
	
	//printf("start point set to %f, %f, %f, %f, \n",s[0],s[1],s[2],s[3]);
	//printf("end point set to %f, %f, %f, %f, corresponding to %f, %f, %f\n",e[0],e[1],e[2],e[3],re[0],re[1],re[2]);
	dmp.set_s(s);
	dmp.set_g(e);
	//printf("dmp values: %i, %i :%i, %i\n",s.size(), e.size(), dmp.s.size(), dmp.g.size());
}


mPointCloudType::Ptr obstacles_as_cloud(){
	mPointCloudType::Ptr output;
	output.reset(new mPointCloudType);
	for (int i = 0; i < oSelect.obstacles.size(); i++){
		output->push_back(mPointType(oSelect.obstacles[i][0], oSelect.obstacles[i][1], oSelect.obstacles[i][2]));
	}
	return output;
}
void goal_callback(const geometry_msgs::Point inputROSMsg_tracker){
	
	vector< float> new_goal(3);
	new_goal[0] = inputROSMsg_tracker.x;
	new_goal[1] = inputROSMsg_tracker.y;
	new_goal[2] = inputROSMsg_tracker.z;

	
	if (oSelect.obstacles.size() > 0)
		e = CSP->get_safest_configuration(new_goal, oSelect.obstacles);
	else
		e = CSP->get_smoothest_configuration(new_goal);
	
	active = true;
	dmp.set_s(y);
	dmp.set_g(e);
	t = 0;
}


void pos_callback(const std_msgs::Float32MultiArray inputROSMsg_tracker){
	for (int i = 0; i < 7; i++)
		robot_joints(i) = inputROSMsg_tracker.data[i];
}




//---------------------------------------------------------------------------------------------------------------------//
int main(int argc, char** argv)
{ 
	active = false;
	float timesteps = 0;
	CSP = new cspaceconverter();
	CSP->launch_obstacle_thread();
	KDL::Frame k(KDL::Rotation::Quaternion(-0.400, 0.231, 0.444, 0.768), KDL::Vector(-0.4, 0.0, 0.3)  ); //listen to the rotation with "rosrun tf tf_echo world KUKA_base" from console
	rsy.resize(3, 0);

	ros::init(argc, argv, "koa");
	ros::NodeHandle nh("~");  
	initGlobalParams(argc, argv);
	std::string source_, sim_topic_, goal_topic_, output_topic_;
	nh.getParam ("source", source_);		///<- source node topic
	nh.getParam ("cloud_topic", sim_topic_);   ///<- obstacle cloud topic
	nh.getParam ("goal_topic", goal_topic_);   ///<- obstacle cloud topic
	nh.getParam ("perfect_Obstacle", perfectObstacle);
	nh.getParam ("no_Obstacle", mode);	
	nh.getParam ("fixed_cloud", fixedcloud);
	nh.getParam ("output_topic", output_topic_);
	oSelect.subscribe(nh, "/"+source_+"/"+sim_topic_);
	ros::Publisher output = nh.advertise<sensor_msgs::PointCloud2>("obstacles", 10);
	ros::Subscriber goal = nh.subscribe<geometry_msgs::Point>("/"+source_+"/"+goal_topic_, 1, goal_callback);
	//ros::Subscriber position = nh.subscribe<geometry_msgs::Float32MultiArray>(par_topic, 1, pos_callback);
	
	ros::Rate r(30); 
	// DMP initialisation
	initDMP();
	std::cout << ".\n";
	//[TODO] fix this absolute link
	//load_trajectory("/home/andrej/Workspace/koa/t2.txt", n, dmpDimensions,  &dmp);
	load_null_trajectory(n, dmpDimensions,  &dmp);
	
	t=0;
	//Comm initialisation
	rosComm rcom(&nh, output_topic_);	
	tf::TransformBroadcaster br;
	
	
	/*
	listener = new   tf::TransformListener;
	kukabaseListener = new   tf::TransformListener;

	try{
			listener->lookupTransform( "kinect_visionSensor", "world", ros::Time(0), *frameTransform);
		}
		catch (tf::TransformException ex){
		  //ROS_ERROR("%s",ex.what());;
		}
	try{
			kukabaseListener->lookupTransform( "KUKA_base", "world", ros::Time(0), *kukaBaseTransform);
		}
		catch (tf::TransformException ex){
		  //ROS_ERROR("%s",ex.what());;
		}
		*/

	

	///Find best configurations for start- and endpoint
	find_conf(0);

	while (ros::ok()){
		if (currKeyFrameID >= 0)
			boost::mutex::scoped_lock lock (m_keycloud); 
		
		
		
		
		if(t<=1.0*tau*T && active = true) //run cycle
		{
			
			y=dmp.get_y();
			closest_distance = -1;

			getLiveObstacleData(t);
			if (perfectObstacle)
					setPerfectObstacle(y);
			
			///here we transform our obstacle into joint space;
			vector<float> o(0);

			if (obst.size() > 1 && !mode){
				vector<vector<float> > par_obst(obst.size()/ 3);
				for (int i = 0; i < obst.size(); i += 3){
					par_obst[i/3].resize(3);
					for (int j = 0; j < 3; j++){
						par_obst[i/3][j] = obst[i+j];
					}
				}
				CSP->set_obstacles(par_obst);
			} else{

			}
			o = CSP->get_configurations_as_vectors();
			dist = CSP->get_distances_as_vectors();
			dmp.set_obstacle(o, dist);
			dmp.calculate_one_step_dmp(t);
			y=dmp.get_y();
			CSP->set_position(y);
			vector<float> y2 = y;
			y2.resize(7);
			y2[4] = 0;
			y2[5] = 0;
			y2[6] = 0;

			rcom.sendJointAngles(y2);
			timesteps++;
			
			rsy = fullKinematic(y);
			vector< vector < float > > positions;
			positions.resize(3);
			positions[0] = fullKinematic(y);
			positions[1] = partialKinematic(y,6);
			positions[2] = partialKinematic(y,3);
			oSelect.positions = positions;
			t=t+dt;

		}
		if (t>1.0*tau*T){
			//t = 0;
			active = false;
			load_null_trajectory(n, dmpDimensions, &dmp);			
			}
	
	
		bool collided = false;
		output.publish(obstacles_as_cloud()); 
		ros::spinOnce();
		r.sleep();

	}
	return 0;
}
//---------------------------------------------------------------------------------------------------------------------//
 
