#include "rosComm.h"


using namespace std;

rosComm::rosComm(ros::NodeHandle *parentnh, const string par_topic){
	
	nh = parentnh;
	
	jointValues = new ros::Publisher;
	*jointValues = nh->advertise<std_msgs::Float32MultiArray>(par_topic,1);
}

void rosComm::sendJointAngles(std::vector<float> val){
	float* newval = new float[7];
	for (int i = 0; i < 7; i++)
		newval[i] = val[i];
	sendJointAngles(newval);
	
}
void rosComm::sendJointAngles(float* val){
	std_msgs::Float32MultiArray message;
	
	message.data.resize(7);
	for (int i = 0; i < 7; i++) { //for full kuka
		if (isnan(val[i])){
			printf("Error: Attempting to pass NaN value to VREP.\n");
			return;
		}
		
		message.data[i] = val[i];
	}
		message.data[1] = 90-message.data[1];
		message.data[3] = -message.data[3];
		message.data[5] = -message.data[5];
		
	jointValues->publish(message);
}



