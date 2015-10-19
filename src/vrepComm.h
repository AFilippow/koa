#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <vector>
/* VREP communication object
 * send either IK target positon OR joint values
 * if you want to use joint values, disable the IK within vrep
 * 
 * 
 * */
using namespace std;
class vrepComm {
public:
	ros::Publisher* obstaclePositions;
	ros::Subscriber* distanceData;
	ros::Publisher* ikPosition;
	ros::Publisher* jointValues;
	ros::Subscriber* collisionResponses;
	ros::Subscriber* Jacobian;
	ros::Subscriber* Joints;
	ros::NodeHandle* nh;
	vector<float> jacobian;
	vector<float> jacobian1;
	vector<float> joints;
	int anyCollision;
	float cubeDistance;
	float brickDistance;
	vrepComm(ros::NodeHandle *parentnh);
	bool getAnyCollisions();
	void moveObstacleNr(int obstacleNr, float x, float y, float z);
	void callback(const std_msgs::Int32 inputROSMsg_tracker);
	void cubeDistanceCallback(const std_msgs::Float32 inputROSMsg_tracker);
	void brickDistanceCallback(const std_msgs::Float32 inputROSMsg_tracker);
	void placeIKTarget(float x, float y, float z);
	void sendJointAngles(float* val);
	void sendJointAngles( std::vector<float> val);
	void jacobianCallback(const std_msgs::String inputROSMsg_tracker);
	void jacobianCallback1(const std_msgs::String inputROSMsg_tracker);
	void jointvalueCallback(const std_msgs::String inputROSMsg_tracker);
	vector<float> getJacobian() {jacobianUpdated = 0;return jacobian;}
	vector<float> getJacobian1() {jacobianUpdated = 0;return jacobian1;}
	vector<float> getJoints() {return joints;}
	bool jacobianUpdated;
};
