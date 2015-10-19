#include "vrepComm.h"


using namespace std;

vrepComm::vrepComm(ros::NodeHandle *parentnh){
	jacobianUpdated = 1;
	anyCollision = 0;
	cubeDistance = 0;
	brickDistance = 0;
	/*string vrepTopics2[3];
	vrepTopics2[0] = 'cubeCollision';
	vrepTopics2[1] = 'brickCollision';
	vrepTopics2[2] = 'cylinderCollision';*/
	nh = parentnh;
	ikPosition = new ros::Publisher;
	*ikPosition = nh->advertise<geometry_msgs::Point>("koa/ikPosition",10);
	obstaclePositions = new ros::Publisher[4];
	obstaclePositions[0] = nh->advertise<geometry_msgs::Point>("koa/cubePosition",10);
	obstaclePositions[1] = nh->advertise<geometry_msgs::Point>("koa/brickPosition",10);
	obstaclePositions[2] = nh->advertise<geometry_msgs::Point>("koa/cylinderPosition",10);
	obstaclePositions[3] = nh->advertise<geometry_msgs::Point>("koa/dummyPosition",10);
	collisionResponses = new ros::Subscriber[3];
	collisionResponses[0] = nh->subscribe<std_msgs::Int32>("/vrep/cubeCollision", 1, &vrepComm::callback, this);
	collisionResponses[1] = nh->subscribe<std_msgs::Int32>("/vrep/brickCollision", 1, &vrepComm::callback, this);
	collisionResponses[2] = nh->subscribe<std_msgs::Int32>("/vrep/cylinderCollision", 1, &vrepComm::callback, this);
	jointValues = new ros::Publisher;
	*jointValues = nh->advertise<std_msgs::String>("koa/jointValString",1);
	Jacobian = new ros::Subscriber[2];
	Jacobian[0] = nh->subscribe<std_msgs::String>("/vrep/Jacobian0",1, &vrepComm::jacobianCallback, this);
	Jacobian[1] = nh->subscribe<std_msgs::String>("/vrep/Jacobian1",1, &vrepComm::jacobianCallback1, this);
	
	Joints = new ros::Subscriber;
	*Joints = nh->subscribe<std_msgs::String>("/vrep/JointValues",1, &vrepComm::jointvalueCallback, this);
	distanceData =  new ros::Subscriber[2];
	distanceData[0] = nh->subscribe<std_msgs::Float32>("/vrep/cubeDistance", 1, &vrepComm::cubeDistanceCallback, this);
	distanceData[1] = nh->subscribe<std_msgs::Float32>("/vrep/brickDistance", 1, &vrepComm::brickDistanceCallback, this);
}
void vrepComm::jointvalueCallback(const std_msgs::String inputROSMsg_tracker){
	string input = inputROSMsg_tracker.data;
	//VREP occasionally uses commas as decimal separators
	std::replace(input.begin(), input.end(), ',', '.'); 
	std::istringstream iss((input));
	joints.resize(0);
	double number;
	while (iss){
		iss >> number;
		joints.push_back(number);
		if (iss.eof()) break;
	}


}
void vrepComm::jacobianCallback(const std_msgs::String inputROSMsg_tracker){
	string input = inputROSMsg_tracker.data;
	//VREP occasionally uses commas as decimal separators
	std::replace(input.begin(), input.end(), ',', '.'); 
	//printf("received %s \n", input.c_str());
	std::istringstream iss((input));
	jacobian.resize(0);
	double number;
	while (iss){
		iss >> number;
		jacobian.push_back(number);
	}
	jacobianUpdated = 1;

}
void vrepComm::jacobianCallback1(const std_msgs::String inputROSMsg_tracker){
	string input = inputROSMsg_tracker.data;
	//VREP occasionally uses commas as decimal separators
	std::replace(input.begin(), input.end(), ',', '.'); 
	std::istringstream iss((input));
	jacobian1.resize(0);
	double number;
	while (iss){
		iss >> number;
		jacobian1.push_back(number);
	}
	jacobianUpdated = 1;

}

void vrepComm::callback(const std_msgs::Int32 inputROSMsg_tracker)
{
	//std::cout << "callback called with data:"<< inputROSMsg_tracker.data << " \n";
	if (inputROSMsg_tracker.data > 0)
	anyCollision = 1;
	return;
}
void vrepComm::cubeDistanceCallback(const std_msgs::Float32 inputROSMsg_tracker)
{
	//std::cout << "callback called with data:"<< inputROSMsg_tracker.data << " \n";
	cubeDistance = inputROSMsg_tracker.data;
	return;
}
void vrepComm::brickDistanceCallback(const std_msgs::Float32 inputROSMsg_tracker)
{
	//std::cout << "callback called with data:"<< inputROSMsg_tracker.data << " \n";
	brickDistance = inputROSMsg_tracker.data;
	return;
}
bool vrepComm::getAnyCollisions(){
	if (anyCollision == 1){
		anyCollision = 0;
		return true;
	}
	else return false;
}

void vrepComm::moveObstacleNr(int obstacleNr, float x, float y, float z){
	
	geometry_msgs::Point newPoint;
	newPoint.x = x;
	newPoint.y = y;
	newPoint.z = z;
	obstaclePositions[obstacleNr].publish(newPoint);

}
void vrepComm::placeIKTarget(float x, float y, float z){
	geometry_msgs::Point newPoint;
	newPoint.x = x;
	newPoint.y = y;
	newPoint.z = z;
	ikPosition->publish(newPoint);
}
void vrepComm::sendJointAngles(std::vector<float> val){
	float* newval = new float[7];
	for (int i = 0; i < 7; i++)
		newval[i] = val[i];
	sendJointAngles(newval);
	
}
void vrepComm::sendJointAngles(float* val){
	stringstream message;
	for (int i = 0; i < 7; i++) { //for full kuka
		if (isnan(val[i])){
			printf("Error: Attempting to pass NaN value to VREP.\n");
			return;
		}
		message << val[i] << 't';
	}
	//message << "1.35t" << val[0] << "t0.0t" << val[1] << "t0.0t0.0t0.0t"; //for kuka as two-joint arm
	//message  << val[0] << "t" << val[1]; //for two-joint arm
	//message  << val[0] << "t" << val[1] << "t" << val[2]; //for 3-joint arm
	std::string message_str(message.str());
	///IMPORTANT: The Lua in V-Rep uses either commas or dots as decimal
	///separators, depending on some unknown variable. if the com does 
	///not work, comment or uncomment this following part:
	std::replace(message_str.begin(), message_str.end(), '.', ','); 
		
	//std::cout << message_str << "\n";
		
	jointValues->publish(message_str);
}



