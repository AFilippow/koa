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
#include "vectormath.h"
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
vector<float> rsy;
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
		obstacleCloud->points[0].x = rsy[0];
		obstacleCloud->points[0].y = rsy[1];
		obstacleCloud->points[0].z = rsy[2];
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
				//printf("parsed w[%i][%i] to %f \n", j,i,w[j][i]);
			}
			else 
			{
				w[j][i] = 0;
				printf("parse failed");
			}
		}
		i++;
   }
   //	std::cout << ".\n";
   fileIn.close();
	
	/*for(int i = 0; i < n; i++)
	{
		printf("W: ");
		for(int j = 0; j < dim; j++){
			printf(" %f,", w[j][i]);
		}
		printf("\n");
	}*/
		
	
	vector<float> wtemp;
	wtemp.resize(dim);
   for (int i=0; i<n; i++)
	{
		for(int j = 0; j < dim; j++)
			wtemp[j] = w[j][i];
	   dmp->set_w(i, wtemp);
	}
		//std::cout << "done\n";
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
	obst.resize(0);
	dist.resize(0);
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
			dist[i/3] = pcl::euclideanDistance(mPointType(obst[i], obst[i+1], obst[i+2]), mPointType(rsy[0], rsy[1], rsy[2]));
			if (dist[i/3] == -1) std::cout <<"Warning: distance estimation failure";
			dist[i/3] -= 0.05;
			//dist[i/3]=segm.distanceOfObstacleToPosition(segm.obstacles[i], mPointType(y[0], y[1], y[2]))-0.1; 
		}
		closest_distance = pcl::euclideanDistance(mPointType(obst[0], obst[1], obst[2]), mPointType(rsy[0], rsy[1], rsy[2]));
	}
	/*obst.push_back(rsy[0]);
	obst.push_back(rsy[1]);
	obst.push_back(0.01);
	dist.push_back(rsy[2]-0.01);*/
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
void moveObstacles(vrepComm vcom){
	
	float radius = 0.1+randomNumber()/12;
	float alpha = randomNumber()*3.141572*2;
	for (int i = 1; i <= 2; i++){
		vcom.moveObstacleNr(i-1, radius*sin(alpha+i*3.141572*2/3)+0.1, 0.1+randomNumber()*0.1, 0.12+radius*cos(alpha+i*3.141572*2/3));
	}
	sleep(3);
	if (vcom.getAnyCollisions() == true)
		moveObstacles(vcom);
}

float vectorarraydistance(KDL::JntArray q1, vector<float> q2){
	float result = 0;
	for (int i = 0; i < dmp_dim; i++)
		result += (q1(i)-q2[i])*(q1(i)-q2[i]);
	return sqrt(result);
}
void test_file(vrepComm vcom){
	std::ifstream slicefile(("/home/andrej/Workspace/cspoutput/4dhandonly/slice_44_50_3.dat"), ios::in);
	std::list<vector<float> > output;
	if (!slicefile)
	{
		printf("error opening file\n");
		return ;
	}
	int pointscontrolled;
	int dim = 4;
	vector<float> loadedpoint(7, 0);
	vector<float> xyz(3, 0);
	std::string line;
	while (std::getline(slicefile, line))
	{	
		std::istringstream iss(line); 
		/*for (int i = 0; i < 3; i++){
			if (!(iss >> xyz[i])) { printf("error: cannot read line!\n"); break; }
		}*/
		for (int i = 0; i < dim; i++){
			if (!(iss >> loadedpoint[i])) { printf("error: cannot read line!\n"); break; }
		}
		vcom.sendJointAngles(loadedpoint);
		sleep(3);
	}
	return;
	
}

void save_some_points(vrepComm vcom){
	FILE * out;
	out = fopen("/home/andrej/Workspace/somepoints.txt","w");
	Chain KukaChain = KukaLWR_DHnew();
	ChainFkSolverPos_recursive* kinematic_solver = new ChainFkSolverPos_recursive(KukaChain);
	int jointNumber = KukaChain.getNrOfJoints();
	KDL::Frame baseframe(KDL::Rotation::Quaternion(- 0.444, 0.231, 0.40, 0.768), KDL::Vector(-0.4, 0.0, 0.3) ) ;

	KDL::JntArray q(jointNumber);
	

		/*q(0) =-0.7;
		q(1) =-0.72;
		q(2) =0.13;
		q(3) =1.5900;
		q(4) = 0;
		q(5) = 0;
		q(6) = 0;*/
		q(0) = 0;
		q(1) = 0;
		q(2) = 0;
		q(3) = 0;
		q(4) = 0;
		q(5) = 0;
		q(6) = 0;

	vector<float> ytemp(7,0);
	vcom.sendJointAngles(ytemp);

	KDL::Frame cartpos;
	KDL::Vector position;
	KDL::Vector extraposition;
	KDL::Vector handposition;
	handposition.x(0);
	handposition.y(0);
	handposition.z(0.3);
	kinematic_solver->JntToCart(q, cartpos);
	
	
	position = baseframe*cartpos.p;
	handposition = baseframe*cartpos*handposition;
	fprintf(out, "%f \t %f \t %f \t \n", position.x(), position.y(), position.z());
	KDL::Vector xtrapos;
		for (float i = 0; i < 64; i++){
			vcom.sendJointAngles(ytemp);
			  float angle = 2*3.14152*((float)i)/11.1;
			  //int odd = !!(i % 2);  // using !! to ensure 0 or 1 value.
			  xtrapos.x(sin(angle)*0.1);
			  xtrapos.y(cos(angle)*0.1);
			  xtrapos.z(-0.05+i/64.0*0.35);
					extraposition = baseframe*cartpos*xtrapos;
					fprintf(out, "%f \t %f \t %f \t \n", extraposition.x(), extraposition.y(), extraposition.z());
					vcom.moveObstacleNr(3, extraposition.x(), extraposition.y(), extraposition.z());
					sleep(1);
				}
	for (int i = 0; i < 7; i++){
		kinematic_solver->JntToCart(q, cartpos, i);
		KDL::Vector position2 = baseframe*cartpos.p;
		fprintf(out, "%f \t %f \t %f \t \n", position2.x(), position2.y(), position2.z());
	}

	/*KDL::Vector position2 = baseframe*cartpos.p;
	fprintf(out, "%f \t %f \t %f \t \n", position2.x(), position2.y(), position2.z());
	KDL::Vector position4;
	position4.x(position.x()*2.00/3.00+position2.x()/3.00);
	position4.y(position.y()*2.00/3.00+position2.y()/3.00);
	position4.z(position.z()*2.00/3.00+position2.z()/3.00);
	fprintf(out, "%f \t %f \t %f \t \n", position4.x(), position4.y(), position4.z());
	vcom.moveObstacleNr(3, handposition.x(), handposition.y(), handposition.z());*/
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

//---------------------------------------------------------------------------------------------------------------------//
int main(int argc, char** argv)
{ 
	float mean_velocity = 0;
	float timesteps = 0;
	CSP = new cspaceconverter();
	CSP->launch_obstacle_thread();
	KDL::Frame k(KDL::Rotation::Quaternion(-0.444, 0.231, 0.40, 0.768), KDL::Vector(-0.4, 0.0, 0.3)  ); //listen to the rotation with "rosrun tf tf_echo KUKA_base world" from console
	rsy.resize(3);
	/*vector<float> pos;
	KDL::JntArray q(7);
	q(0) = 0;
	q(1) = 0;
	q(2) = 0;
	q(3) = 0;
	q(4) = 0;
	q(5) = 0;
	q(6) = 0;	
	
	pos = partialKinematic(q,7);
	printf("&: %f \t %f \t %f \n", pos[0], pos[1], pos[2]);	
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
	/*save_some_points(vcom);
	return 1; */


	/*test_file(vcom);
	return 1;*/
	
		std::cout << "spinning...\n";
	//segmenter init
	segm.position.x = 0;
	segm.position.y = 0;
	segm.position.z = 0;	
		
			
		
		
	tf::TransformBroadcaster br;
	listener = new   tf::TransformListener;
	kukabaseListener = new   tf::TransformListener;
	
	///outputs and measurments
	

	int trialrun = 0;
	int trial = 3;
	string folders[7];
	string prefix = "series2/";
	folders[0] = prefix+"examplerun";
	folders[1] = prefix+"examplerun";
	folders[2] = prefix+"examplerun";
	folders[3] = prefix+"examplerun";
	folders[4] = prefix+"examplerun";
	folders[5] = prefix+"examplerun";
	folders[6] = prefix+"examplerun";
	float lambda[7];
	lambda[0] = 0.2;
	lambda[1] = 0.3;
	lambda[2] = 0.4;
	lambda[3] = 0.5;
	lambda[4] = 0.6;
	lambda[5] = 0.7;
	lambda[6] = 0.8; 
	
	
	string trajectoryoutput = "/home/andrej/Workspace/"+folders[trial]+"/"+boost::lexical_cast<string>(trialrun)+".dat";
	string collisionoutput = "/home/andrej/Workspace/"+folders[trial]+"/collisions.dat";

	std::cout << trajectoryoutput << std::endl;
	std::cout << collisionoutput << std::endl;
	FILE * fileOutput; 
	fileOutput = fopen(collisionoutput.c_str(), "w");	
	FILE * multirun;
	multirun = fopen(trajectoryoutput.c_str(),"w");
	FILE * datasets;
	datasets = fopen("/home/andrej/Workspace/xdmp_dts.txt","w");
	dmp.LAMBDA = lambda[trial];
	//CSP->pointsconsidered = lambda[trial];

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
	

	///Find best configurations for start- and endpoint
	find_conf(0);


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
		//if(vector_length(vector_difference(dmp.get_y(), dmp.g)) > 0.1) //run cycle
		{
			
			y=dmp.get_y();
			closest_distance = -1;

			
			
			//Automatic obstacles
			if (fixedcloud && !perfectObstacle)
				getSavedObstacleData(t);
			else
			{
				getLiveObstacleData(t);
			}
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

			/*printf("Obst: ");
			for (int i = 0; i < obst.size(); i++)
				printf("%f, ", obst[i]);
			printf("\n");*/
			if (obst.size() > 1 && !mode){
				vector<vector<float> > par_obst(obst.size()/ 3);
				for (int i = 0; i < obst.size(); i += 3){
					par_obst[i/3].resize(3);
					for (int j = 0; j < 3; j++){
						par_obst[i/3][j] = obst[i+j];
					}
				}
				/*vector<float> tempvector; //push back into obst instead
				tempvector.push_back(rsy[0]);
				tempvector.push_back(rsy[1]);
				tempvector.push_back(0.01);
				par_obst.push_back(tempvector);*/
				CSP->set_obstacles(par_obst);


				/*printf("Distances: ");
				for (int i = 0; i < dist.size(); i++)
					printf("%f, ", dist[i]);
				printf("\n");*/
				/*for (int i = 0; i < o.size(); i+= dmpDimensions){
					float localdist = 0;
					for (int j = 0; j < dmpDimensions; j++)
						localdist += (o[i+j]-y[j])*(o[i+j]-y[j]);
					dist[i/dmpDimensions] = sqrt(localdist);
				}*/
				
				
				
				//dist = CSP->get_distances_as_vectors();
				/*printf("Distances: ");
				for (int i = 0; i < dist.size(); i++)
					printf("%f, ", dist[i]);
				printf("\n");*/
			} else{
			//printf("no obstacles found %i \n",obst.size());

			}
			o = CSP->get_configurations_as_vectors();
			dist = CSP->get_distances_as_vectors();
			dmp.set_obstacle(o, dist);
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
			
			KDL::Vector handposition;
			handposition.x(0);
			handposition.y(0);
			handposition.z(0.15);
			KDL::Vector temp = k*(CSP->joint_to_KDLFrame(y2))*handposition;
			mean_velocity += (temp.x()-rsy[0])*(temp.x()-rsy[0])+(temp.y()-rsy[1])*(temp.y()-rsy[1])+(temp.z()-rsy[2])*(temp.z()-rsy[2]);
			//fprintf(multirun, "%f \t %f \n",sqrt((temp.x()-rsy[0])*(temp.x()-rsy[0])+(temp.y()-rsy[1])*(temp.y()-rsy[1])+(temp.z()-rsy[2])*(temp.z()-rsy[2])), vector_length(dmp.get_z()) );
			fprintf(multirun, "%f \t %f \t %f \n", vcom.cubeDistance, vcom.brickDistance, vector_length(dmp.get_z()) );
			timesteps++;
			rsy[0] = temp.x();
			rsy[1] = temp.y();
			rsy[2] = temp.z();
			//printf("rsy: %f, %f, %f \n", rsy[0], rsy[1], rsy[2]);
			
			segm.setPosition(rsy);
			
			if ( t < 0.05*tau*T && angle < 0.1 ) 
			{
			//std::cout << "Attempting grasp.\n";
			//scom.sendGrasp(grasp);
		
			}
					
			t=t+dt;
		}
		if (t>1.0*tau*T){
		//if (vector_length(vector_difference(dmp.get_y(), dmp.g)) < 0.1){
			fprintf(datasets, "%i, \n", dmp.timestep);
			fflush(datasets);
			t = 0;
			load_trajectory("/home/andrej/Workspace/koa/t2.txt", n, dmpDimensions, &dmp);
			angle += angular_increment;
			flipTrajectory = -flipTrajectory;
					
			find_conf(flipTrajectory);
			trialrun++;
			moveObstacles(vcom);
			previousObstacles.clear(); 
			obstaclePersistenceWeight.clear();
			int collided = vcom.getAnyCollisions();
			fprintf(fileOutput,"%i\n", collided);
			printf("Collision status: %i at trial %i, run %i \n", collided, trial, trialrun);
			
			fclose(multirun);

			if (trialrun > 25)
			{
				fclose(fileOutput);
				trialrun = 0;
				trial++;
				if (trial ==4){
					std::cout << "Test finished! exiting...\n";
					return 1;
				}				
				
					
				collisionoutput = "/home/andrej/Workspace/"+folders[trial]+"/collisions.dat";
				fileOutput = fopen(collisionoutput.c_str(), "w");	
				dmp.LAMBDA = lambda[trial];
				//CSP->pointsconsidered = lambda[trial];

			}	
			trajectoryoutput = "/home/andrej/Workspace/"+folders[trial]+"/"+boost::lexical_cast<string>(trialrun)+".dat";
			multirun = fopen(trajectoryoutput.c_str(),"w");
			//std::cout<< "Collision status: " << vcom.getAnyCollisions() << endl;
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
		
		//fileOutput << pcl::euclideanDistance(mPointType(additional_track_points[0], additional_track_points[1], additional_track_points[2]), mPointType(-0.5,-0.4,0.15)) << "\t";
		//fileOutput << pcl::euclideanDistance(mPointType(additional_track_points[3], additional_track_points[4], additional_track_points[5]), mPointType(-0.5,-0.4,0.15)) << "\t";
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
 
