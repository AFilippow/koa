#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <vector>
/* ROS communication object
 * sends joint values
 * 
 * */

class rosComm {
public:
	ros::Publisher* jointValues;
	ros::NodeHandle* nh;
	rosComm(ros::NodeHandle *parentnh, const std::string par_topic);
	void sendJointAngles(float* val);
	void sendJointAngles( std::vector<float> val);
	
};
