#include "obstacleSelector.h"
obstacleSelector::obstacleSelector(){
	fullCloud.reset(new mPointCloudTypeLabel);
	obstacles.resize(0);
	distances.resize(0);
	positions.resize(0);
	currKeyFrameID = 0;

	viewer = boost::make_shared<pcl::visualization::PCLVisualizer> ("obstacle cloud viewer");

}
void obstacleSelector::subscribe(ros::NodeHandle par_handle, const string par_topic){
	cloud_sub = par_handle.subscribe<sensor_msgs::PointCloud2>(par_topic, 1, &obstacleSelector::callback, this);
}
float obstacleSelector::dist(vector<float> a, vector<float> b){
	return sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1])+(a[2]-b[2])*(a[2]-b[2]));
}

void obstacleSelector::callback(const sensor_msgs::PointCloud2 inputROSMsg_tracker){
	fullCloud.reset(new mPointCloudTypeLabel);

	
	pcl::fromROSMsg ( inputROSMsg_tracker, *fullCloud); //convert the cloud 
	
	//pcl_ros::transformPointCloud(*currCloud, *currCloud, ((tf::Transform*)(frameTransform))->inverse());

	fullCloud->header.seq++;
	
	///REMOVE THIS as soon as the pont cloud works properly
	for (int i = 0; i < fullCloud->points.size(); i++){
		while (fullCloud->points[i].x > 0.2)
			fullCloud->points[i].x -= 0.1;
		while (fullCloud->points[i].x < -0.2)
			fullCloud->points[i].x += 0.1;
		
		while (fullCloud->points[i].y > 0.2)
			fullCloud->points[i].y -= 0.1;
		while (fullCloud->points[i].y < -0.2)
			fullCloud->points[i].y += 0.1;			
		
		while (fullCloud->points[i].z > 0.2)
			fullCloud->points[i].z -= 0.1;
		while (fullCloud->points[i].z < 0.0)
			fullCloud->points[i].z += 0.1;			
			
	}
	
	///downsample
	/*pcl::VoxelGrid< pcl::PointXYZ > sor;
	sor.setInputCloud (fullCloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*fullCloud);*/
	currKeyFrameID=fullCloud->header.seq;
	
		if(!viewer->updatePointCloud<pcl::PointXYZL>(oSelect.fullCloud, "raw_cloud"))
          viewer->addPointCloud<pcl::PointXYZL>(oSelect.fullCloud, "raw_cloud");
}



float obstacleSelector::min_dist(vector<float> par_vec){
	float output = 1000;
	for (unsigned int i = 0; i < positions.size(); i++){
		float current_distance = dist(par_vec, positions[i]);
		if (current_distance < output)
		output = current_distance;
	}
	return output;
}

float obstacleSelector::min_dist(pt_dst_lbl par_pt){
	min_dist(par_pt.p);
}
float obstacleSelector::min_dist(mPointTypeLabel ptl){
	vector<float> v(3);
	v[0] = ptl.x;
	v[1] = ptl.y;
	v[2] = ptl.z;
	return min_dist(v);
}
vector<float> obstacleSelector::asvector(mPointTypeLabel par_pt){
	vector<float> output;
	output.push_back(par_pt.x);
	output.push_back(par_pt.y);
	output.push_back(par_pt.z);
	return output;
}

void obstacleSelector::updateObstacles(){
	if (fullCloud->points.size() < 10) 
	{
		std::cout << "Input cloud empty, skipping obstacle update.\n";
		return;
	}
	if (positions.size() < 1) 
	{
		std::cout << "No positions set, skipping obstacle update.\n";
		return;
	}
	
	std::vector< pt_dst_lbl > closestPointWithDistanceAndLabel(0);
	
	for (unsigned int i = 0; i < fullCloud->points.size(); i++){
		bool found = false;
		for (unsigned int j = 0; j < closestPointWithDistanceAndLabel.size(); j++){
			float dist = min_dist(closestPointWithDistanceAndLabel[j]);
			if (fullCloud->points[i].label == closestPointWithDistanceAndLabel[j].l){
				found = true;
				if (dist < closestPointWithDistanceAndLabel[j].d){
					closestPointWithDistanceAndLabel[j].p = asvector(fullCloud->points[i]);
					closestPointWithDistanceAndLabel[j].d = dist;
				}
			}
		}
		if (!found){
			closestPointWithDistanceAndLabel.push_back(pt_dst_lbl(asvector(fullCloud->points[i]), min_dist(fullCloud->points[i]),fullCloud->points[i].label));
		}
	}
	
	
	obstacles.resize(0);
	
	for (int i = 0; i < closestPointWithDistanceAndLabel.size(); i++){
		obstacles.push_back(closestPointWithDistanceAndLabel[i].p);
		distances.push_back(closestPointWithDistanceAndLabel[i].d);
	}
	
}
