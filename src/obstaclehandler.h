#include <iostream>
#include <vector>
#include <list>
#include <pcl_ros/transforms.h>

using namespace std;


class obstaclehandler {
public:
	
	obstaclehandler();
	~obstaclehandler();
	KDL::JntArray lookupConfigurationFromPoint(vector<float> point, vector<float> configuration)



	int max_list_size;
	list<vector<float>> obstacle_list;
	vector<float> position;




};
