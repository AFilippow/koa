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
   
#include "segmenter.h"
#include "xdmp.h"
#include "vrepComm.h"
#include <time.h> 

#include "cspaceconverter.h"

//SocketComm was written to communicate with the Morse simulator, V-REP does not need it
//#include "SocketComm.h"

#define NIL (0) 
#define PI 3.14152
typedef pcl::PointCloud<pcl::PointXYZ> mPointCloudType;
typedef pcl::PointCloud<pcl::PointXYZRGB> mPointCloudTypeColor;
typedef pcl::PointXYZ mPointType; 

mPointCloudType::Ptr currCloud;  
mPointCloudTypeColor::Ptr newCloud;
mPointCloudTypeColor::Ptr obstacleCloud;
int dmpDimensions;
boost::mutex m_keycloud;
int currKeyFrameID;
vector<float> y;
bool mode;
pclsegmenter segm;
float angle;  
bool perfectObstacle;  
bool fixedcloud;
int dmp_dim;
float T; //seconds
float tau;
float dt; //seconds
int n; 
int flipTrajectory;
float sigma; 
float mass_center_distance;
float closest_distance;
vector<float> obst(0);
vector<float> dist(0);  
std::vector<mPointTypeColor> previousObstacles; 
std::vector<float> obstaclePersistenceWeight;
vector<float> s;
vector<float> e;
tf::TransformListener* listener;
tf::TransformListener* kukabaseListener;
tf::StampedTransform * frameTransform;
tf::StampedTransform * kukaBaseTransform;
xDMP dmp;

cspaceconverter * CSP;

FILE * cspobst;
FILE * cspobst2;
FILE * cspobst3;

float randomNumber() //between 0 and 1
{
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
float weighted_euclidean_distance(vector<float> a, vector<float> b){
	float distance = 0;	
	distance += (a[0]-b[0])*(a[0]-b[0])*0.2; 
	distance += (a[1]-b[1])*(a[1]-b[1])*0.5;
	distance += (a[2]-b[2])*(a[2]-b[2]);
	
return distance;
}

void pregenerateObstacles(){

}	
int selectObstacleBasedOnDist(){
	int index = 0;
	float mindist = 1000;	
	for (int i = 0; i < dist.size(); i++){
		if (dist[i] < mindist){
			mindist = dist[i];
			index = i;
		}
	}
	return index*3;
}
void setPerfectObstacle(vector<float> position){
	//printf("currently at %s \n", __func__);
	/*obst.resize(7);    //7 dimensions
	dist.resize(1);	
	obst[0] =  0;
	obst[1] =  0;
	obst[2] =  0;
	obst[3] =  0;
	obst[4] =  0;
	obst[5] =  0;
	obst[6] =  0;*/
	///Use this only if not using pregeneration
	/*
	 obst.resize(2); //2 dimensions
	dist.resize(1);	
	obst[0] = -1.49;
	obst[1] = 0;
	dist[0] = 0;
	vector<float> spacepos = forwardsKinematic(position);
	vector<float> spaceobst = forwardsKinematic(obst);
	*/
	position.resize(7);
	//position[4] = 0;
	//position[5] = 0;
	position[6] = 0;
	//vector<float> spacepos = CSP->joint_to_cartesian(position);
	obst.resize(3); //2 dimensions
	dist.resize(1);	
	obst[0] = 0.175;
	obst[1] = 0.2;
	obst[2] = 0.19;
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
	//printf("Fwd kinematic position: %f, %f, %f \n",r[0],r[1],r[2]);

	dist[0] = sqrt((r[0]-obst[0])*(r[0]-obst[0])+(r[1]-obst[1])*(r[1]-obst[1])+(r[2]-obst[2])*(r[2]-obst[2]));
	///Later, we shall use one obstacle below the arm to force it away from the table
/*	obst[3] = spacepos[0];
	obst[4] = spacepos[1];
	obst[5] = 0.01;
	dist[1] = 0;


	dist[1] = dist[1] + 0.05;
*/
	///[TODO] fix distances
	//dist[0] = sqrt((obst[0] - segm.position.x)*(obst[0] - segm.position.x) + (obst[1] - segm.position.y) *(obst[1] - segm.position.y) + (obst[2] - segm.position.z)*(obst[2]- segm.position.z));
	//dist[1] = sqrt((obst[3] - segm.position.x)*(obst[3] - segm.position.x) + (obst[4] - segm.position.y) *(obst[4] - segm.position.y) + (obst[5] - segm.position.z)*(obst[5]- segm.position.z));
	//dist[2] = sqrt((obst[6] - segm.position.x)*(obst[6] - segm.position.x) + (obst[7] - segm.position.y) *(obst[7] - segm.position.y) + (obst[8] - segm.position.z)*(obst[8]- segm.position.z));
}

void paintObstacles(){

	//std::cout << "Number of obstacles:" <<segm.obstacles.size() << "\n";
	obstacleCloud->height = 1;
	obstacleCloud->width = segm.obstacles.size()+2;
	obstacleCloud->points.resize(obstacleCloud->width);
	if (obstacleCloud->points.size() > 0)
	{
		obstacleCloud->points[0].x = y[0];
		obstacleCloud->points[0].y = y[1];
		obstacleCloud->points[0].z = y[2];
		obstacleCloud->points[0].r = 0;
		obstacleCloud->points[0].g = 0;
		obstacleCloud->points[0].b = 250;
		obstacleCloud->points[1].x = 0;
		obstacleCloud->points[1].y = 0;
		obstacleCloud->points[1].z = 0;
		obstacleCloud->points[1].r = 0;
		obstacleCloud->points[1].g = 0;
		obstacleCloud->points[1].b = 0;
	
	}
	for (unsigned int i = 0; i < segm.obstacles.size(); i++)
		{
		obstacleCloud->points[i+2].x = segm.obstacles[i].x;
		obstacleCloud->points[i+2].y = segm.obstacles[i].y;
		obstacleCloud->points[i+2].z = segm.obstacles[i].z;
		obstacleCloud->points[i+2].r = segm.obstacles[i].r*2;
		obstacleCloud->points[i+2].g = segm.obstacles[i].g*2;
		obstacleCloud->points[i+2].b = segm.obstacles[i].b*2;			
		
		}	

}

void segmentOnceAndGetClouds(){
	//printf("currently at %s \n", __func__);
	newCloud.reset(new mPointCloudTypeColor);
	obstacleCloud.reset(new mPointCloudTypeColor);
	segm.segment();
	//std::cout <<"Cloud segments found: "<< segm.segment() << std::endl;
	newCloud =  segm.getColoredCloud()->makeShared();
	//segm.excludeObstacle(segm.position);
	paintObstacles();
}


void callback(const sensor_msgs::PointCloud2 inputROSMsg_tracker){
	//printf("currently at %s \n", __func__);
	boost::mutex::scoped_lock lock (m_keycloud);   
	currCloud.reset(new mPointCloudType);

	
	pcl::fromROSMsg ( inputROSMsg_tracker,*currCloud); //convert the cloud 
	
	pcl_ros::transformPointCloud(*currCloud, *currCloud, ((tf::Transform*)(frameTransform))->inverse());
	//pcl_ros::transformPointCloud(*currCloud, *currCloud, *frameTransform);
	
	currCloud->header.seq++; 
	pcl::VoxelGrid< pcl::PointXYZ > sor;
	sor.setInputCloud (currCloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*currCloud);
	currKeyFrameID=currCloud->header.seq;
	
	
	//   std::cout << "point at at: "<< currCloud->points[0].x << " " << currCloud->points[0].y << " " << currCloud->points[0].z << "\n ";
	
	
	segm.setRawCloud(currCloud);
	

	/*->header.frame_id = "some_tf_frame";
	newCloud->height = newCloud->width = 1;
	newCloud->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));
	*/
	//color_depth();
}

void load_trajectory(char* filein, int n, int dim, xDMP *dmp)
{   
	//printf("currently at %s \n", __func__);
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
				printf("parsed w[%i][%i] to %f \n", j,i,w[j][i]);
			}
			else 
			{
				w[j][i] = 0;
				printf("parse failed");
			}
		}
		i++;
   }
   	std::cout << ".\n";
   fileIn.close();
	
	for(int i = 0; i < n; i++)
	{
		printf("W: ");
		for(int j = 0; j < dim; j++){
			printf(" %f,", w[j][i]);
		}
		printf("\n");
	}
		
	
	vector<float> wtemp;
	wtemp.resize(dim);
   for (int i=0; i<n; i++)
	{
		for(int j = 0; j < dim; j++)
			wtemp[j] = w[j][i];
	   dmp->set_w(i, wtemp);
	}
		std::cout << "done\n";
   	for (int i = 0; i < dim; i++)
		delete [] w[i];
}


void calculate_obstacle_persistence()
{
	//printf("currently at %s \n", __func__);
	//if (!segm) return;
	std::vector<float> newObstaclePersistenceWeight(segm.obstacles.size());
	for (unsigned int i = 0; i < newObstaclePersistenceWeight.size(); i++)
		newObstaclePersistenceWeight[i] = 0;
	std::vector<bool> obstacleWasFoundAgain(obstaclePersistenceWeight.size());
	for (unsigned int i = 0; i < obstacleWasFoundAgain.size(); i++)
		obstacleWasFoundAgain[i] = false;
	for (unsigned int oldObstacleNumber = 0; oldObstacleNumber < obstaclePersistenceWeight.size(); oldObstacleNumber++)
	{
		for (unsigned int newObstacleNumber = 0; newObstacleNumber < segm.obstacles.size(); newObstacleNumber++)
		{
			if (pcl::euclideanDistance( previousObstacles[oldObstacleNumber], segm.obstacles[newObstacleNumber]) < 0.03)
			{
				obstacleWasFoundAgain[oldObstacleNumber] = true;
				newObstaclePersistenceWeight[newObstacleNumber] = 1;
				//add here if we want obstacles to stick to their previous positions
			}
			
		}
	}
	for (unsigned int oldObstacleNumber = 0; oldObstacleNumber < obstacleWasFoundAgain.size(); oldObstacleNumber++)
	{
		if (!obstacleWasFoundAgain[oldObstacleNumber] && obstaclePersistenceWeight[oldObstacleNumber] > 0.1)
			{
				segm.obstacles.push_back(mPointTypeColor(previousObstacles[oldObstacleNumber]));
				newObstaclePersistenceWeight.push_back(obstaclePersistenceWeight[oldObstacleNumber]-0.1);
			}	
	}
	for (unsigned int newObstacleNumber = 0; newObstacleNumber < segm.obstacles.size(); newObstacleNumber++)
	{
		if (newObstaclePersistenceWeight[newObstacleNumber] == 0)
			newObstaclePersistenceWeight[newObstacleNumber] = 1;
	}
	obstaclePersistenceWeight = newObstaclePersistenceWeight;
	previousObstacles = segm.obstacles;
}





///This function retrieves the obstacles from the segmenter in real time
void getLiveObstacleData(float t){
	//printf("currently at %s \n", __func__);
	segmentOnceAndGetClouds();
	if((segm.obstacles.size()>0 && t > 0.05*tau*T) && !perfectObstacle)
		{
			obst.resize(segm.obstacles.size()*3);
			dist.resize(obst.size()/3);
			//printf("%u obstacles found: \n",segm.obstacles.size());
			for(unsigned int i = 0; i < obst.size(); i+=3)
			{		
				//std::cout << "obstacle" << i/3 << " \n";
				//printf("nr. %i: %f, %f, %f \n",i,segm.obstacles[i/3].x,segm.obstacles[i/3].y,segm.obstacles[i/3].z);
				obst[i]=segm.obstacles[i/3].x;
				obst[i+1]=segm.obstacles[i/3].y;
				obst[i+2]=segm.obstacles[i/3].z;
				//dist[i/3] = pcl::euclideanDistance(segm.obstacles[i], mPointType(y[0], y[1], y[2]));
				//dist[i/3] =sqrt((segm.obstacles[i].x - segm.position.x)*(segm.obstacles[i].x - segm.position.x) + (segm.obstacles[i].y - segm.position.y) *(segm.obstacles[i].y - segm.position.y) + (segm.obstacles[i].z - segm.position.z)*(segm.obstacles[i].z - segm.position.z));
				dist[i/3] = pcl::euclideanDistance(mPointType(obst[i], obst[i+1], obst[i+2]), mPointType(y[0], y[1], y[2]));
				if (dist[i/3] == -1) std::cout <<"Warning: distance estimation failure";
				dist[i/3] -= 0.05;
				//dist[i/3]=segm.distanceOfObstacleToPosition(segm.obstacles[i], mPointType(y[0], y[1], y[2]))-0.1; 
			}
			closest_distance = pcl::euclideanDistance(mPointType(obst[0], obst[1], obst[2]), mPointType(y[0], y[1], y[2]));
		}


}

void getSavedObstacleData(float t){
	//printf("currently at %s \n", __func__);
	segm.findClosestPoints();	
	segm.excludeObstacle(mPointType(-0.143, 0.571, 0.4));
	if(segm.obstacles.size()>0 )
	{
		//just in case, keep the resizing
		obst.resize(segm.obstacles.size()*3);
		dist.resize(segm.obstacles.size());
		for (unsigned int i = 0; i < dist.size(); i++)
			dist[i] = -2;
		for (unsigned int i = 0; i < obst.size(); i++)
			obst[i] = -2;

		for(unsigned int i = 0; i < obst.size(); i+=3)
		{
			obst[i]=segm.obstacles[i/3].x;
			obst[i+1]=segm.obstacles[i/3].y;
			obst[i+2]=segm.obstacles[i/3].z;
			//dist[i/3] = pcl::euclideanDistance(segm.obstacles[i], mPointType(y[0], y[1], y[2]));
			//dist[i/3] =sqrt((segm.obstacles[i].x - segm.position.x)*(segm.obstacles[i].x - segm.position.x) + (segm.obstacles[i].y - segm.position.y) *(segm.obstacles[i].y - segm.position.y) + (segm.obstacles[i].z - segm.position.z)*(segm.obstacles[i].z - segm.position.z));
			dist[i/3] = pcl::euclideanDistance(mPointType(obst[i], obst[i+1], obst[i+2]), mPointType(y[0], y[1], y[2]));
			
			//printf("obst %i is %f,%f,%f  \n",i/3,obst[i],obst[i+1],obst[i+2]);
			
			//printf("dist %i is %f \n",i/3,dist[i/3]);
			if (dist[i/3] == -1) std::cout <<"Warning: distance estimation failure\n";
			if (dist[i/3] == -2) std::cout <<"Warning: distance initialisation failed\n";
			//dist[i/3]=segm.distanceOfObstacleToPosition(segm.obstacles[i], mPointType(y[0], y[1], y[2]))-0.1; 
		}
		closest_distance = pcl::euclideanDistance(mPointType(obst[0], obst[1], obst[2]), mPointType(y[0], y[1], y[2]));
	}
	paintObstacles();
}

void initGlobalParams(int argc, char** argv){
	//printf("currently at %s \n", __func__);
	srand(time(NULL));
	mass_center_distance = 0;
	closest_distance = 0;
	mode = false;
	perfectObstacle = false;
	fixedcloud = false;
	flipTrajectory = -1;
	/*if (argc > 1)
		for(int i = 1; i < argc; i++)
		{
		if (strcmp(argv[i], "-noobstacle"))
		mode = true;
		
		//if (strcmp(argv[i], "-m"))
		//	mode = true;
		//if (strcmp(argv[i], "-perfect"))
		//	perfectObstacle = true;
		}*/
	std::cout << "Initialising point cloud...\n";
	// Point Cloud initialisation
	currKeyFrameID = -1;
	newCloud.reset(new mPointCloudTypeColor);
	newCloud->header.frame_id = "kuka";
	newCloud->height = newCloud->width = 1;
	newCloud->points.push_back (pcl::PointXYZRGB());
	obstacleCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	obstacleCloud->header.frame_id = "kuka";
	obstacleCloud->height = obstacleCloud->width = 1;
	obstacleCloud->points.push_back (pcl::PointXYZRGB());
	frameTransform = new tf::StampedTransform( tf::Transform::getIdentity() , ros::Time::now(), "world", "kuka");
	kukaBaseTransform = new tf::StampedTransform( tf::Transform::getIdentity() , ros::Time::now(), "world", "kuka_base");
}

void initDMP(){
	dmpDimensions = 4;
	//printf("currently at %s \n", __func__);
	y.resize(dmpDimensions);
	std::cout << "Initialising dmp\n";
	dmp_dim=dmpDimensions;
	T=3; //seconds
	tau=1;
	dt=0.01; ///change here important ----- seconds
	n=3; //number of cores
	sigma=1; 
	//Start and endpoint
	s.resize(dmp_dim);



	s[0] =-0.23; ///7 dimensions
	s[1] =-0.76;
	s[2] =-0.27;
	s[3] =1.89;
	//s[4] =0.12;
	//s[5] =1,3;
	//s[6] =-0.42345395684242;
	e.resize(dmp_dim);
	e[0] =-0.37;
	e[1] =-2.25;
	e[2] =-0.95;
	e[3] =1,07;
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
void moveObstacles(vrepComm vcom){
	
	float radius = 0.1+randomNumber()/4;
	float alpha = randomNumber()*3.141572*2;
	for (int i = 1; i <= 3; i++){
		vcom.moveObstacleNr(i-1, radius*sin(alpha+i*3.141572*2/3), radius*cos(alpha+i*3.141572*2/3), i == 3? 0.15:0.075);
	}
	sleep(3);
}

float vectorarraydistance(KDL::JntArray q1, vector<float> q2){
	float result = 0;
	for (int i = 0; i < dmp_dim; i++)
		result += (q1(i)-q2[i])*(q1(i)-q2[i]);
	return sqrt(result);
}
KDL::JntArray lookupConfigurationFromPoint(vector<float> point, vector<float> configuration){
	/*
	 * I should consider loading the obstacle data once and finding the closest point at every time step.
	 * 
	 * 
	 * 
	 */
	KDL::JntArray loadedpoint(6), closestObstacle(6);
	int x, y, z;
	x = floor((point[0]+0.3)*61/0.6);
	if (x < 0 || x >= 62) { printf("ERROR: x( = %i) is out of bounds.\n", x); return closestObstacle;}
		
	y = floor((point[1]+0.15)*61/0.6);
	if (y < 0 || y >= 62) { printf("ERROR: y( = %i) is out of bounds.\n", y); return closestObstacle;}
		
	z = floor((point[2])*21/0.2);
	if (z < 0 || z >= 22) { printf("ERROR: z( = %i) is out of bounds.\n", z); return closestObstacle;}
	printf("x: %i, y: %i, z: %i \n",x, y, z);
	std::string line;
	//std::ifstream slicefile(("/home/andrej/Workspace/cspoutput/yzreduced/slice_"+boost::to_string(x)+"_"+boost::to_string(y)+"_"+boost::to_string(z)+".dat").c_str(), ios::in);
	std::ifstream slicefile(("/home/andrej/Workspace/cspoutput/4dreduced/slice_"+boost::to_string(x)+"_"+boost::to_string(y)+"_"+boost::to_string(z)+".dat").c_str(), ios::in);

	int loadedx, pointscontrolled;
	float mindist = 10000;
	pointscontrolled = 0;
	while (std::getline(slicefile, line))
	{	
		std::istringstream iss(line);///WARNING still explicit dimensions here
		if (!(iss >> loadedx >> loadedpoint(0) >> loadedpoint(1) >> loadedpoint(2) >> loadedpoint(3) >> loadedpoint(4) >> loadedpoint(5))) { printf("error: cannot read line!\n"); break; }
				pointscontrolled ++;
		if (vectorarraydistance(loadedpoint, configuration) < mindist){

			mindist = vectorarraydistance(loadedpoint, configuration);
			closestObstacle = loadedpoint;
		}		
	}
	//printf("found closest obstacle to be %f, %f, %f, %f of %i points \n", closestObstacle(0) ,closestObstacle(1), closestObstacle(2), closestObstacle(3),pointscontrolled);
	return closestObstacle;
}
KDL::JntArray lookupConfigurationFromPoint_6_Points(vector<float> point, vector<float> configuration){
	KDL::JntArray currconf, closestObstacle(4);
	int x, y, z;
	x = floor((point[0]+0.3)*61/0.6);
	if (x < 0 || x >= 62) { printf("ERROR: x( = %i) is out of bounds.\n", x); return closestObstacle;}
		
	y = floor((point[1]+0.15)*61/0.6);
	if (y < 0 || y >= 62) { printf("ERROR: y( = %i) is out of bounds.\n", y); return closestObstacle;}
		
	z = floor((point[2])*21/0.2);
	if (z < 0 || z >= 22) { printf("ERROR: z( = %i) is out of bounds.\n", z); return closestObstacle;}
		
		
	float mindist = 10000;
	vector<float> temppoint = point;

	if (x >= 1){
		temppoint = point;
		temppoint[0] -=(0.6/61);
		currconf = lookupConfigurationFromPoint(temppoint, configuration);
		if (vectorarraydistance(currconf, configuration) < mindist){
			mindist = vectorarraydistance(currconf, configuration);
			closestObstacle = currconf;
		}
	}
	if (x <= 60){
		temppoint = point;
		temppoint[0] +=(0.6/61);
		currconf = lookupConfigurationFromPoint(temppoint,  configuration);
		if (vectorarraydistance(currconf, configuration) < mindist){
			mindist = vectorarraydistance(currconf, configuration);
			closestObstacle = currconf;
		}
	}


	if (y >= 1){
		temppoint = point;
		temppoint[1] -=(0.6/61);
		currconf = lookupConfigurationFromPoint(temppoint, configuration);
		if (vectorarraydistance(currconf, configuration) < mindist){
			mindist = vectorarraydistance(currconf, configuration);
			closestObstacle = currconf;
		}
	}
	if (y <= 60){
		temppoint = point;
		temppoint[1] +=(0.6/61);
		currconf = lookupConfigurationFromPoint(temppoint, configuration);
		if (vectorarraydistance(currconf, configuration) < mindist){
			mindist = vectorarraydistance(currconf, configuration);
			closestObstacle = currconf;
		}
	}
	
	if (z >= 1){
		temppoint = point;
		temppoint[2] -=(0.6/61);
		currconf = lookupConfigurationFromPoint(temppoint, configuration);
		if (vectorarraydistance(currconf, configuration) < mindist){
			mindist = vectorarraydistance(currconf, configuration);
			closestObstacle = currconf;
		}
	}
	if (z <= 20){
		temppoint = point;
		temppoint[2] +=(0.6/61);
		currconf = lookupConfigurationFromPoint(temppoint, configuration);
		if (vectorarraydistance(currconf, configuration) < mindist){
			mindist = vectorarraydistance(currconf, configuration);
			closestObstacle = currconf;
		}
	}
	//printf("found point in c-space: %f \t %f \t %f \t %f with distance %f \n",closestObstacle(0),closestObstacle(1),closestObstacle(2),closestObstacle(3),mindist);
	return closestObstacle;
}

//---------------------------------------------------------------------------------------------------------------------//
int main(int argc, char** argv)
{ 
	
	CSP = new cspaceconverter();
	CSP->launch_obstacle_thread();
	KDL::Frame k(KDL::Rotation::Quaternion(-0.444, 0.231, 0.40, 0.768), KDL::Vector(-0.4, 0.15, 0.35)  ); //listen to the rotation with "rosrun tf tf_echo KUKA_base world" from console

	/*vector<float> pos;
	KDL::JntArray q(7);
	q(0) = 0;
	q(1) = 0;
	q(2) = 0;
	q(3) = 0;
	q(4) = 0;
	q(5) = 0;
	q(6) = 0;	
		
	pos = partialKinematic(q,6);
	printf("&: %f \t %f \t %f \n", pos[0], pos[1], pos[2]);
		pos = partialKinematic(q,5);
	printf("&: %f \t %f \t %f \n", pos[0], pos[1], pos[2]);
		pos = partialKinematic(q,4);
	printf("&: %f \t %f \t %f \n", pos[0], pos[1], pos[2]);
		pos = partialKinematic(q,3);
	printf("&: %f \t %f \t %f \n", pos[0], pos[1], pos[2]);	
	pos = partialKinematic(q,2);
	printf("&: %f \t %f \t %f \n", pos[0], pos[1], pos[2]);
	pos = partialKinematic(q,1);
	printf("&: %f \t %f \t %f \n", pos[0], pos[1], pos[2]);
		pos = partialKinematic(q,0);
	printf("&: %f \t %f \t %f \n", pos[0], pos[1], pos[2]);	
	return 0;*/

	
	
	
	
	//CSP->generate_points_data(k);
	//return 1;
	
	
	//cspobst = fopen("/home/andrej/Workspace/cspaceobstacles.txt","w");
	//cspobst2 = fopen("/home/andrej/Workspace/cspaceobstacles2.txt","w");
	//cspobst3 = fopen("/home/andrej/Workspace/cspaceobstacles3.txt","w");
	ros::init(argc, argv, "koa");
	ros::NodeHandle nh("~");  
	initGlobalParams(argc, argv);
	std::string source_, sim_topic_;
	
	nh.getParam ("source", source_);
	nh.getParam ("sim_topic", sim_topic_);   
	nh.getParam ("perfect_Obstacle", perfectObstacle);
	nh.getParam ("no_Obstacle", mode);	
	nh.getParam ("fixed_cloud", fixedcloud);
	ros::Subscriber sub_sim = nh.subscribe<sensor_msgs::PointCloud2>("/"+source_+"/"+ sim_topic_, 1, callback);
	ros::Publisher output = nh.advertise<sensor_msgs::PointCloud2>("segmentedOutput", 10);
	ros::Publisher output2 = nh.advertise<sensor_msgs::PointCloud2>("obstacles", 10);
	
	ros::Rate r(30); 
	// DMP initialisation
	initDMP();
	std::cout << ".\n";
	//[TODO] fix this absolute link
	load_trajectory("/home/andrej/Workspace/koa/t2.txt", n, dmpDimensions,  &dmp);
	float t=0;
	//Comm initialisation
		std::cout << "Initialising VREP Comm...\n";
	//SocketComm scom;
	vrepComm vcom(&nh);

		std::cout << "spinning...\n";
	//segmenter init
	segm.position.x = 0;
	segm.position.y = 0;
	segm.position.z = 0;	
		
	tf::TransformBroadcaster br;
	listener = new   tf::TransformListener;
	kukabaseListener = new   tf::TransformListener;
	ofstream fileOutput("/home/andrej/Workspace/trajectiveCollisions.txt");	
	

	//for testing purposes:
	angle = 0.5;
	double angular_increment = 1/16 * 3.14152;



	angular_increment = (3.14152/8);
	std::cout << "angular_increment: " << angular_increment << "\n";

	//pregenerateObstacles();

	while (!currCloud)
	{	
		std::cout << "no cloud yet \n";
		ros::spinOnce();
		sleep(1);
	}
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
		
	moveObstacles(vcom);
	if (fixedcloud) 
	{
		segmentOnceAndGetClouds();
		getSavedObstacleData(t);
	}
	


	int trialrun = 1;
	
	
	while (ros::ok()){
		if (currKeyFrameID >= 0)
			boost::mutex::scoped_lock lock (m_keycloud); 
		
		try{
			listener->lookupTransform( "kinect_visionSensor", "world", ros::Time(0), *frameTransform);
		}
		catch (tf::TransformException ex){
		  //ROS_ERROR("%s",ex.what());;
		}

		
		///change here
		
		tf::StampedTransform nullTransform(frameTransform->inverse(), ros::Time::now(), "world", "kuka");
		nullTransform.setIdentity ();
		br.sendTransform(nullTransform);
				
				
		try{
			kukabaseListener->lookupTransform( "KUKA_base", "world", ros::Time(0), *kukaBaseTransform);
		}
		catch (tf::TransformException ex){
		  //ROS_ERROR("%s",ex.what());;
		}
		
				
		
		
		
		if(t<=1.0*tau*T) //run cycle
		{

			y=dmp.get_y();
			closest_distance = -1;

			
			
			//Automatic obstacles
			/*if (fixedcloud)
				getSavedObstacleData(t);
			else
			{
				getLiveObstacleData(t);
			}*/
			if (perfectObstacle)
				{
					if (trialrun==1 || trialrun==3 || trialrun==5)
						setPerfectObstacle(y);
					else
					{
						obst.resize(0);
						dist.resize(0);
					}
				}
			//printf("Y: %f, %f;  dist: %f \n", y[0], y[1], dist[0]);
			//mass_center_distance = sqrt((-0.4 - segm.position.x)*(-0.4 - segm.position.x) + (-0.4 - segm.position.y) *(-0.4 - segm.position.y) + (0.3 - segm.position.z)*(0.3 - segm.position.z));
			
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

				vector<float> o = CSP->get_configurations_as_vectors();
				dist = CSP->get_distances_as_vectors();
				dmp.set_obstacle(o, dist);
			} else{
			//printf("no obstacles found %i \n",obst.size());
			dmp.set_obstacle(obst, dist);
			}
			//vector<float> x = partialKinematic(y,1);
			//printf("X: %f, %f, %f \n",x[0],x[1],x[2]);
			dmp.calculate_one_step_dmp(t);
			y=dmp.get_y();
			CSP->set_position(y);
			vector<float> y2 = y;
			y2.resize(7);
			y2[4] = 0;
			y2[5] = 0;
			y2[6] = 0;
			//printf("continuing \n");
			vcom.sendJointAngles(y2);
			/*segm.position.x = y[0];
			segm.position.y = y[1];
			segm.position.z = y[2];*/
			
			if ( t < 0.05*tau*T && angle < 0.1 ) 
			{
			//std::cout << "Attempting grasp.\n";
			//scom.sendGrasp(grasp);
		
			}
					
			t=t+dt;
		}
		if (t>1.0*tau*T){
			t = 0;
			load_trajectory("/home/andrej/Workspace/koa/t2.txt", n, dmpDimensions, &dmp);
			angle += angular_increment;
			flipTrajectory = -flipTrajectory;
			//std::cout << "added "<<   angular_increment << "to angle\n";
			/*s[0] = 0.4*sin(angle)*flipTrajectory;
			s[1] = 0.5*cos(angle)*flipTrajectory;
			s[2] = 0.15;*/
			
			//e[0] = 0.3 + 0.2*randomNumber();
			//e[1] = 0.1 + 0.2*randomNumber();
			//e[2] = 0.1;			
			//s[0] = -0.5 + 0.2*randomNumber();
			//s[1] = -0.3 + 0.2*randomNumber();
			//s[2] = 0.15;			
			
			/*s[0] = 0.4;
			s[1] = 0.05;
			s[2] = 0.14;
			e[0] = -0.4;
			e[1] = -0.05;
			e[2] = 0.14;*/
			dmp.set_s(s);
			dmp.set_g(e);

			
			/*fclose(dmp.positionAndMotion);
			
			char result[100];
			sprintf(result,"/home/andrej/Workspace/xdmp_positionmotion%i.txt",trialrun);
		
			dmp.positionAndMotion = fopen(result,"w");*/
			
			/*switch (trialrun){ //0.1, 0.05
			case 1: dmp.motion_persistence_weight = 0.90; break;
			case 2: dmp.motion_persistence_weight = 0.95;break;			
			case 3: dmp.motion_persistence_weight = 0.97; break;
			case 4: dmp.motion_persistence_weight = 0.99; break;						
			}*/
			/*switch (trialrun){ //0.1, 0.05
			case 1: dmp.LAMBDA = 0.05; break;
			case 2: dmp.LAMBDA = 0.025; break;			
			case 3: dmp.LAMBDA = 0.01; break;
			case 4: dmp.LAMBDA = 0.005; break;						
			}*/
			trialrun++;
			//e[0] = -0.4*sin(angle)*flipTrajectory;
			//e[1] = -0.4*cos(angle)*flipTrajectory;
			//e[2] = 0.15;
			
			/*if (1 == flipTrajectory){
				dmp.set_s(y);
				dmp.set_g(s);
			}else{
				dmp.set_s(y);
				dmp.set_g(e);
			}*/
			
			moveObstacles(vcom);
			previousObstacles.clear(); 
			obstaclePersistenceWeight.clear();
			//segm.obstacles.clear();
			if (angle > 2.1)
			{
				fileOutput.close();
				std::cout << "Test finished! exiting...\n";
				return 1;
			}	
			
			std::cout<< "Collision status: " << vcom.getAnyCollisions() << endl;
			y = s;
			/*segm.position.x = y[0];
			segm.position.y = y[1];
			segm.position.z = y[2];
			dmp.reset();
			if (fixedcloud){
				 segmentOnceAndGetClouds();
				 //segm.excludeObstacle(mPointType(-0.143, 0.571, 0.4));
				 getSavedObstacleData(t);
			}*/
			}
	
	
		bool collided = false;
		//std::cout<< "Collision status: " << vcom.getAnyCollisions() << endl;
		
		
		
		//cout<< "obstacles memorized: " << previousObstacles.size() << "\n";
		//fileOutput << y[0] << "\t" << y[2] << "\t" << collided << "\t";
		//fileOutput << mass_center_distance << "\t" << closest_distance << "\t";
		//fileOutput << angle+ (angular_increment)*(t/(tau*T)) << "\t";
		//std::cout <<"current angle:" << angle+ (angular_increment)*(t/(tau*T)) << "\n";
		//fileOutput <<y[0] << "\t" <<  y[1] << "\t" <<  y[2] << "\t";
		if (dmp.deviation.size() > 0)
			fileOutput << dmp.deviation[0] << "\t" <<  dmp.deviation[1] << "\t" <<  dmp.deviation[2] << "\t";
		else
			fileOutput << "0 \t 0 \t 0 \t";
		//fileOutput << pcl::euclideanDistance(mPointType(additional_track_points[0], additional_track_points[1], additional_track_points[2]), mPointType(-0.5,-0.4,0.15)) << "\t";
		//fileOutput << pcl::euclideanDistance(mPointType(additional_track_points[3], additional_track_points[4], additional_track_points[5]), mPointType(-0.5,-0.4,0.15)) << "\t";
		fileOutput << std::endl;
		newCloud->header.frame_id = "kuka";
		obstacleCloud->header.frame_id = "kuka";
		output.publish(newCloud); 
		output2.publish(obstacleCloud); 
		ros::spinOnce();
		r.sleep();

	}
	
			//fclose(cspobst);
			//fclose(cspobst2);
			//fclose(cspobst3);
	return 0;
}
//---------------------------------------------------------------------------------------------------------------------//
 
