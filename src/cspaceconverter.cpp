#include "cspaceconverter.h"


using namespace std;
using namespace KDL;
cspaceconverter::cspaceconverter(){
	KukaChain = KukaLWR_DHnew();
	kinematic_solver = new ChainFkSolverPos_recursive(KukaChain);
	jointNumber = KukaChain.getNrOfJoints();


}
float vectordistance(vector<float> x, vector<float> y)
{
	vector<float> difference;
	float output = 0;
	difference.resize(x.size());
	if (x.size() != y.size()) {
		cout << "Vector subtraction error: sizes not the same!\n";
		return 100000;
	}
	for (unsigned int i =0; i < x.size(); i++)
		output += (x[i]-y[i])*(x[i]-y[i]);
	return sqrt(output);
}
float vectordistance(vector<float> x, vector<int> y)
{
	vector<float> yprime(y.size());
	for (int i = 0; i < y.size(); i++)
	yprime[i] = (float)(y[i]);
	return vectordistance(x, yprime);
}

std::list<vector<float> > load_closest_obstacle(vector<float> position, vector<int> obstacle, int num_points){
	std::ifstream slicefile(("/home/andrej/Workspace/cspoutput/4dreduced/slice_"+boost::to_string(obstacle[0])+"_"+boost::to_string(obstacle[1])+"_"+boost::to_string(obstacle[2])+".dat").c_str(), ios::in);
	std::list<vector<float> > output;
	if (!slicefile)
	{
		printf("error opening file \n");
		return output;
	}
	int pointscontrolled;
	int dim = position.size();
	vector<float> loadedpoint(dim);
	float mindist = 10000;
	pointscontrolled = 0;
	std::string line;
	while (std::getline(slicefile, line))
	{	
		std::istringstream iss(line); 
		for (int i = 0; i < dim; i++){
			if (!(iss >> loadedpoint[i])) { printf("error: cannot read line!\n"); break; }
		}
		pointscontrolled ++;
		bool inserted = false;
		std::list<vector<float> >::iterator iter;
		for (iter = output.begin(); iter != output.end(); iter++){
			if (vectordistance(loadedpoint, position) < vectordistance(*iter, position)){
				output.insert(iter, loadedpoint);
				inserted = true;
				break;
			}
		}
		if (output.size() < num_points && inserted == false)
			output.push_back(loadedpoint);
		while (output.size() > num_points) 
			output.pop_back();
	}
	return output;
}


void uniquely_bin(vector< vector< int > >* par_list, vector<float> par_vector){
	vector<int> binned_vector(3);
	//printf("binning Vector %f, %f, %f\n", par_vector[0], par_vector[1], par_vector[2]);
	
	binned_vector[0] = floor((par_vector[0]+0.3)*61.0/0.6);
	binned_vector[1] = floor((par_vector[1]+0.15)*61.0/0.6);
	binned_vector[2] = floor((par_vector[2])*21/0.2);
	
	for (int i = 0; i < par_list->size(); i++){
		if ((*par_list)[i][0] == binned_vector[0] && (*par_list)[i][1] == binned_vector[1] && (*par_list)[i][2] == binned_vector[2])
			return;
	}
	par_list->push_back(binned_vector);
	//printf("Binned Vector to %i, %i, %i\n",binned_vector[0],binned_vector[1],binned_vector[2]);
	return;
}
vector<float> unbin(vector<int> par_vec){
	vector<float> output(3);
	output[0] = 0.6/61*par_vec[0]-0.3;
	output[1] = 0.6/61*par_vec[1]-0.15;
	output[2] = 0.2/21*par_vec[2];
	return output;
	
}


void* update_obstacle_list(void* par_void){
	lister_parameters params = *(lister_parameters*) par_void;
	
	while (true){
		vector<vector<int> > binned_obstacles(0);
		vector<float> position;
		
		pthread_mutex_lock(params.position_lock);
		position = *(params.position);
		pthread_mutex_unlock(params.position_lock);
		if (position.size() < 1){
			printf("No position received:\n");
			sleep(2);
			continue;
		}
		//printf("received: %f, %f, %f, %f \n", position[0], position[1], position[2], position[3]);
		
		
		pthread_mutex_lock(params.list_lock);	
		std::list<vector<float> >::iterator iter;
		for (iter = params.obstacle_list->begin(); iter != params.obstacle_list->end(); iter++){
			uniquely_bin(&binned_obstacles, *iter);
		}
		pthread_mutex_unlock(params.list_lock);
		
		std::list< vector < float > > concat_obstacle_list;
		std::list < float> dist_list;
		
		vector< vector < int > >::iterator iter2;
		for (iter2 = binned_obstacles.begin(); iter2 != binned_obstacles.end(); iter2++){
			std::list<vector<float> > newlist = load_closest_obstacle(position, *iter2, 8);
			std::list< vector < float > >::iterator iter3;
			for(iter3 = newlist.begin(); iter3!= newlist.end(); iter3++){
				//dist_list.push_back(vectordistance(params.converter->joint_to_cartesian(position, -1), unbin(*iter2) ));
				dist_list.push_back(vectordistance(position, *iter3 ));
			}
			concat_obstacle_list.splice(concat_obstacle_list.end(), newlist);
		}
		int listsize = concat_obstacle_list.size();
		//printf("generated %i positions\n", listsize);
		pthread_mutex_lock(params.configuration_lock);
		params.distances_list->clear();
		*(params.distances_list) = dist_list;
		*(params.configuration_list) = concat_obstacle_list;
		pthread_mutex_unlock(params.configuration_lock);
	
	}
}
void cspaceconverter::launch_obstacle_thread(){
	pthread_t thread;
	lister_parameters* t = new lister_parameters(this, &list_lock, &position_lock, &configuration_lock, &position, &obstacle_list, &configuration_list, &distances_list); 
	int rc = pthread_create(&thread, NULL, update_obstacle_list, (void *)t);
     
	
}
void cspaceconverter::set_position(vector<float> par_pos){
	pthread_mutex_lock(&position_lock);
	
	position = par_pos;	
	pthread_mutex_unlock(&position_lock);	
}

void cspaceconverter::set_obstacles(vector<vector<float> > par_obst){
	pthread_mutex_lock(&list_lock);
	obstacle_list.clear();
	for (int i = 0; i < par_obst.size(); i++)
		obstacle_list.push_back(par_obst[i]);
	pthread_mutex_unlock(&list_lock);	
}


int cspaceconverter::examineDifference(vector<float> point1, vector<float> point2){
	//printf(" %f \n",vector_length(vector_difference(point1, point2)));
	if(vector_length(vector_difference(point1, point2))<0.03){
		//printf("Point found at %f,%f,%f \n",point1[0],point1[1],point1[2]);
		return 1;
	}
	else
		return 0;
}



/*
 *float vectordistance(KDL::JntArray q1, KDL::JntArray q2){
	float result = 0;
	for (int i = 0; i < 7; i++)
		result += (q1(i)-q2(i))*(q1(i)-q2(i));
	return sqrt(result);
}*/
float specialdistance(KDL::JntArray q1, vector<float> q2){
	float result = 0;
	for (int i = 0; i < 4; i++)
		result += (q1(i)-q2[i])*(q1(i)-q2[i]);
	return sqrt(result);
}
KDL::JntArray toJointArray(vector< float > i){
	KDL::JntArray output(i.size());
	for (int j = 0; j < i.size(); j++)
		output(j) = i[j];
	return output;
}
vector< float > toVector(KDL::JntArray i){
	vector< float > output(i.rows());
	for (int j = 0; j < i.rows(); j++)
		output[j] = i(j);
	return output;
}

vector<float> cspaceconverter::get_configurations_as_vectors(){
	vector<float> output(0);
	pthread_mutex_lock(&configuration_lock);
			
	std::list<vector<float> >::iterator iter;
	for(iter = configuration_list.begin(); iter != configuration_list.end(); iter++)	
		for (int i = 0; i < iter->size(); i++)
			output.push_back((*iter)[i]);
	
	pthread_mutex_unlock(&configuration_lock);
	return output;
}
vector<float> cspaceconverter::get_distances_as_vectors(){
	vector<float> output(0);
	pthread_mutex_lock(&configuration_lock);
			
	std::list<float>::iterator iter;
	for(iter = distances_list.begin(); iter != distances_list.end(); iter++)	
		output.push_back(*iter);
	
	pthread_mutex_unlock(&configuration_lock);
	return output;
	
	
}
vector<float> cspaceconverter::joint_to_cartesian(vector<float> jointvalues, int segmentnumber){
	
	//printf("parsing %i values for %i joints \n", jointvalues.size(), jointNumber);
	vector<float> cartesianvalues(3, 0);
	if (jointvalues.size() == 0) 
		return cartesianvalues;

	KDL::Frame cartpos;
	KDL::JntArray q(jointNumber);
	for (int i = 0; i < jointNumber; i++)
		q(i) = jointvalues[i];
		
		
	bool kinematics_status = kinematic_solver->JntToCart(q, cartpos, segmentnumber);
	

	cartesianvalues[0] = cartpos.p.x();
	cartesianvalues[1] = cartpos.p.y();
	cartesianvalues[2] = cartpos.p.z();
	return cartesianvalues;
}


/*
 * 
0.39*cos(a)*cos(d)*sin(b) + (0.39*cos(a)*cos(b)*cos(c) - 0.39*sin(a)*sin(c))*sin(e) + 0.4*cos(a)*sin(b)
0.39*cos(d)*sin(a)*sin(b) + (0.39*cos(b)*cos(c)*sin(a) + 0.39*cos(a)*sin(c))*sin(e) + 0.4*sin(a)*sin(b)
                                           -0.39*cos(c)*sin(e)*sin(b) + 0.39*cos(b)*cos(d) + 0.4*cos(b)
 * 
 * 
 */
