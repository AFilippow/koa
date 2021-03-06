#include "cspaceconverter.h"


using namespace std;
using namespace KDL;
cspaceconverter::cspaceconverter(){
	baseframe = KDL::Frame(KDL::Rotation::Quaternion(0.446, 0.120, 0.857, 0.230), KDL::Vector(-0.050, 0.000, 0.330) ) ;
	mode = 1;
	KukaChain = KukaLWR_DHnew();
	kinematic_solver = new ChainFkSolverPos_recursive(KukaChain);
	jointNumber = KukaChain.getNrOfJoints();
	lowerbounds = new float[3];
	upperbounds = new float[3];
	lowerbounds[0] = -0.7;
	lowerbounds[1] = -0.7;
	lowerbounds[2] = 0.4;
	upperbounds[0] = 0.7;
	upperbounds[1] = 0.7;
	upperbounds[2] = 1.4;
	xycoarseness = 71; 
	zcoarseness = 51;
	pointsconsidered = 100;
	pthread_mutex_init(&list_lock, NULL);
	pthread_mutex_init(&position_lock, NULL);
	pthread_mutex_init(&configuration_lock, NULL);
	pthread_mutex_init(&intermediate_configuration_lock, NULL);

}
float vectordistance(vector<float> x, vector<float> y)
{
	vector<float> difference;
	float output = 0;
	difference.resize(x.size());
	if (x.size() != y.size()) {
		printf("Vector subtraction error: sizes not the same! x.size() = %i, y.size() = %i.\n",x.size(), y.size() );
		return 100000;
	}
	/*vector<float> weight(6);
	weight[0] = 0.8;
	weight[1] = 0.8;
	weight[2] = 0.65;
	weight[3] = 0.65;
	weight[4] = 0.2;
	weight[5] = 0.2;*/
	for (unsigned int i =0; i < x.size(); i++)
		output += (x[i]-y[i])*(x[i]-y[i]);//*weight[i];
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
	std::ifstream slicefile(("/home/andrej/Workspace/cspoutput/4dsreduced/slice_"+boost::to_string(obstacle[0])+"_"+boost::to_string(obstacle[1])+"_"+boost::to_string(obstacle[2])+".dat").c_str(), ios::in);
	std::list<vector<float> > output;
	if (!slicefile)
	{
		//printf("error 0 opening file %i, %i, %i\n", obstacle[0], obstacle[1], obstacle[2]);
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


void uniquely_bin(vector< vector< int > >* par_list, vector<float> par_vector, float* lowerbounds, float* upperbounds, float xycoarseness, float zcoarseness){
	if (par_vector[0] < lowerbounds[0] || par_vector[1]  < lowerbounds[1] || par_vector[2]  < lowerbounds[2] || par_vector[0]  > upperbounds[0] || par_vector[1]  > upperbounds[1] || par_vector[2]  > upperbounds[2]){
	//	printf("Vector %f, %f, %f out of range\n", par_vector[0], par_vector[1], par_vector[2]);
		return;
	}
	vector<int> binned_vector(3);
	//printf("binning Vector %f, %f, %f\n", par_vector[0], par_vector[1], par_vector[2]);
	
	binned_vector[0] = floor((par_vector[0]-lowerbounds[0])*xycoarseness/(upperbounds[0]-lowerbounds[0]));
	binned_vector[1] = floor((par_vector[1]-lowerbounds[1])*xycoarseness/(upperbounds[1]-lowerbounds[1]));
	binned_vector[2] = floor((par_vector[2]-lowerbounds[2])*zcoarseness/(upperbounds[2]-lowerbounds[2]));
	
	for (int i = 0; i < par_list->size(); i++){
		if ((*par_list)[i][0] == binned_vector[0] && (*par_list)[i][1] == binned_vector[1] && (*par_list)[i][2] == binned_vector[2])
			return;
	}
	par_list->push_back(binned_vector);
	//printf("Binned Vector %f, %f, %f to %i, %i, %i\n", par_vector[0], par_vector[1], par_vector[2], binned_vector[0],binned_vector[1],binned_vector[2]);
	return;
}
vector<float> unbin(vector<int> par_vec, float* lowerbounds, float* upperbounds, float xycoarseness, float zcoarseness){
	vector<float> output(3);
	output[0] = (upperbounds[0]-lowerbounds[0])/xycoarseness*par_vec[0]+lowerbounds[0];
	output[1] = (upperbounds[1]-lowerbounds[1])/xycoarseness*par_vec[1]+lowerbounds[1];
	output[2] = (upperbounds[2]-lowerbounds[2])/zcoarseness*par_vec[2]+lowerbounds[2];
	return output;
	
}
vector<int> to_binned_vector(KDL::Vector par_kdlvector, float* lowerbounds, float* upperbounds, float xycoarseness, float zcoarseness){
	vector<int> output(3);
	output[0] = floor((par_kdlvector(0)-lowerbounds[0])*xycoarseness/(upperbounds[0]-lowerbounds[0]));
	output[1] = floor((par_kdlvector(1)-lowerbounds[1])*xycoarseness/(upperbounds[1]-lowerbounds[1]));
	output[2] = floor((par_kdlvector(2)-lowerbounds[2])*zcoarseness/(upperbounds[2]-lowerbounds[2]));
	return output;
}

vector< float > append_to_begining(float par_a, float par_b, vector<float> par_vector){
	vector<float> output(par_vector.size()+2);
	for (int i = 0; i < par_vector.size(); i++)
		output[i+2] = par_vector[i];
	output[0] = par_a;
	output[1] = par_b;
	
	return output;
}
double get_time()
{
    struct timeval t;
    gettimeofday(&t, NULL);
    double d = t.tv_sec + (double) t.tv_usec/1000000;
    return d;
}
void* sort_obstacle_list(void* par_void){
	lister_parameters* params = (lister_parameters*) par_void;
	std::list<vector<float> > reduced_concat_list; ///put N closest obstacles here
	std::list<vector<float> > concat_obstacle_list; // list of all current obstacles
	int maxpoints = params->converter->pointsconsidered;
		
	while(true){
		vector<float> position;
		///grab stuff
		pthread_mutex_lock(params->position_lock);
		position = *(params->position);
		pthread_mutex_unlock(params->position_lock);
		
		
		
		pthread_mutex_lock(params->intermediate_configuration_lock);
		concat_obstacle_list = *(params->intermediate_configuration_list);
		pthread_mutex_unlock(params->intermediate_configuration_lock);
		
		//start sorting
		std::list<vector<float> >::iterator iter;
		std::list<vector<float> >::iterator iter4;
		double time_start = get_time();
		for	(iter = concat_obstacle_list.begin(); iter != concat_obstacle_list.end(); iter++){
			vector< float > loadedpoint = *iter;
			bool inserted = false;
			for (iter4 = reduced_concat_list.begin(); iter4 != reduced_concat_list.end(); iter4++){
				if (vectordistance(loadedpoint, position) < vectordistance(*iter4, position)){
					reduced_concat_list.insert(iter4, loadedpoint);
					inserted = true;
					//printf("point added\n");
					break;
				}
			}			
			if (reduced_concat_list.size() < maxpoints && inserted == false)
				reduced_concat_list.push_back(loadedpoint);
			while (reduced_concat_list.size() > maxpoints) {
				reduced_concat_list.pop_back();
				//printf("point discarded\n");
			}

		}
		///get distances
		std::list < float> dist_list;
		for(iter4 = reduced_concat_list.begin(); iter4!= reduced_concat_list.end(); iter4++){
			dist_list.push_back(vectordistance(position, *iter4));
		}
		int listsize = reduced_concat_list.size();
		//printf("generated %i positions from %i \n", listsize, local_obstacles.size());
		if (listsize > 0){			
			vector<float> firstelement = *(reduced_concat_list.begin());
			float firstdist = *(dist_list.begin());
			printf("Closest of %i is %f, %f, %f, %f, %f, %f at %f\n",listsize, firstelement[0], firstelement[1], firstelement[2], firstelement[3], firstelement[4], firstelement[5], firstdist);
			//printf("Own position is  %f, %f, %f, %f, %f, %f \n", position[0], position[1], position[2], position[3], position[4], position[5]);
		}
		double time_end = get_time();
		if (concat_obstacle_list.size() >3)
			printf("sorted %i in %f sec.\n",concat_obstacle_list.size(), time_end - time_start);
		pthread_mutex_lock(params->configuration_lock);
		//printf("locked mutex at %i from obstacle thread, code %i\n", (params->configuration_lock), pthread_mutex_lock(params->configuration_lock));
		params->distances_list->clear();
		*(params->distances_list) = dist_list;
		*(params->configuration_list) = reduced_concat_list;
		pthread_mutex_unlock(params->configuration_lock);
	}
	
	
}

void* update_obstacle_list_split(void* par_void){
	///set up forward kinematics
	//add baseframe here
	//printf("received mutex at %i\n", ((*(lister_parameters*) par_void).configuration_lock));
	lister_parameters* params = (lister_parameters*) par_void;
	//printf("new mutex at %i\n", (params->configuration_lock));
	Chain KukaChain = KukaLWR_DHnew();
	ChainFkSolverPos_recursive kinematic_solver(KukaChain);
	KDL::Frame cartpos;
	KDL::Frame k(KDL::Rotation::Quaternion(0.446, 0.120, 0.857, 0.230), KDL::Vector(-0.050, 0.000, 0.330));
	KDL::Vector newpos;
	KDL::JntArray q(7);
	for ( int i = 0; i < 7; i++){
		q(i) = 0.0;
	}
	kinematic_solver.JntToCart(q, cartpos, 3);
	KDL::Frame inverse_null = cartpos;
	vector<int> bobs;
	while (true){
		double time_start = get_time();
		vector<vector<int> > binned_obstacles(0);
		vector<vector<float> > local_obstacles(0);
		vector<float> position;
		///grab stuff
		pthread_mutex_lock(params->position_lock);
		position = *(params->position);
		pthread_mutex_unlock(params->position_lock);
		if (position.size() < 1){
			printf("No position received:\n");
			sleep(2);
			continue;
		}
		for ( int i = 0; i < position.size(); i++){
			q(i) = position[i];
		}
		q(6) = 0.0;
		KDL::JntArray q_temp(7);
		//printf("received: %f, %f, %f, %f \n", position[0], position[1], position[2], position[3]);
		
		
		pthread_mutex_lock(params->list_lock);	
		std::list<vector<float> >::iterator iter;
		for (iter = params->obstacle_list->begin(); iter != params->obstacle_list->end(); iter++){
			//uniquely_bin(&binned_obstacles, *iter, params->converter->lowerbounds, params->converter->upperbounds, params->converter->xycoarseness, params->converter->zcoarseness);
			local_obstacles.push_back(*iter);
		}
		//printf("%i obstacles received\n", local_obstacles.size());
		pthread_mutex_unlock(params->list_lock);
		
		std::list< vector < float > > concat_obstacle_list;

		
		vector< vector < float > >::iterator iter2;
		//for (iter2 = binned_obstacles.begin(); iter2 != binned_obstacles.end(); iter2++){
		///over all obstacles:
		for (iter2 = local_obstacles.begin(); iter2 != local_obstacles.end(); iter2++){
				KDL::Vector obstacle((*iter2)[0], (*iter2)[1], (*iter2)[2]);
				
			for (float i = q(0)-0.06; i <= q(0)+0.06; i += 0.02)
				for (float j = q(1)-0.06; j <= q(1)+0.06; j += 0.02){
					q_temp = q;
					q_temp(0) = i;
					q_temp(1) = j;
					
					kinematic_solver.JntToCart(q_temp, cartpos, 3);
					newpos = inverse_null*cartpos.Inverse(k.Inverse(obstacle));
					//printf("newpos is %f %f %f \n", newpos.x(), newpos.y(), newpos.z());
					bobs = to_binned_vector(newpos, params->converter->lowerbounds, params->converter->upperbounds, params->converter->xycoarseness, params->converter->zcoarseness);
					//printf("oldpos is %f, %f, %f newpos is %f %f %f, binned to %i %i %i  \n", obstacle.x(), obstacle.y(), obstacle.z(), newpos.x(), newpos.y(), newpos.z(), bobs[0], bobs[0], bobs[2]);
					//printf("Params are %f %f %f, %f %f %f, %f %f \n", params->converter->lowerbounds[0], params->converter->lowerbounds[0], params->converter->lowerbounds[2], params->converter->upperbounds[0], params->converter->upperbounds[0], params->converter->upperbounds[2], params->converter->xycoarseness, params->converter->zcoarseness);
					///need to add first two coords to newlist
					std::list<vector<float> > newlist = load_closest_obstacle(position, bobs, params->converter->pointsconsidered);
					std::list< vector < float > >::iterator iter3;
					for(iter3 = newlist.begin(); iter3!= newlist.end(); iter3++){
						(*iter3)[0] = i;
						(*iter3)[1] = j;
					}
					concat_obstacle_list.splice(concat_obstacle_list.end(), newlist);
				}
		
		}
		
		pthread_mutex_lock(params->intermediate_configuration_lock);
		*(params->intermediate_configuration_list) = concat_obstacle_list;
		pthread_mutex_unlock(params->intermediate_configuration_lock);
		//sleep(1);
		double time_end = get_time();
		if(concat_obstacle_list.size() > 3)
			printf("loaded %i in %f sec.\n",concat_obstacle_list.size(), time_end - time_start);
	}
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
			uniquely_bin(&binned_obstacles, *iter, params.converter->lowerbounds, params.converter->upperbounds, params.converter->xycoarseness, params.converter->zcoarseness);
		}
		pthread_mutex_unlock(params.list_lock);
		
		std::list< vector < float > > concat_obstacle_list;
		std::list < float> dist_list;
		
		vector< vector < int > >::iterator iter2;
		for (iter2 = binned_obstacles.begin(); iter2 != binned_obstacles.end(); iter2++){
			std::list<vector<float> > newlist = load_closest_obstacle(position, *iter2, params.converter->pointsconsidered);
			std::list< vector < float > >::iterator iter3;
			float diminishingFactor = 1;
			for(iter3 = newlist.begin(); iter3!= newlist.end(); iter3++){
				//dist_list.push_back(diminishingFactor*vectordistance(params.converter->joint_to_complete_cartesian(position, -1), unbin(*iter2, params.converter->lowerbounds, params.converter->upperbounds, params.converter->xycoarseness, params.converter->zcoarseness) ));
				dist_list.push_back(diminishingFactor*vectordistance(position, *iter3));
				//diminishingFactor*=1.1;
				//dist_list.push_back(vectordistance(position, *iter3 ));
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
		//sleep(1);
	
	}
}

vector<float> cspaceconverter::get_safest_configuration(vector<float> par_position, vector< vector <float> > obstacles){  ///TODO NOT FIXED
	int dim = 4;
	vector<float> best_configuration(0,0);
	if (par_position[0] < lowerbounds[0] || par_position[1]  < lowerbounds[1] || par_position[2]  < lowerbounds[2] || par_position[0]  > upperbounds[0] || par_position[1]  > upperbounds[1] || par_position[2]  > upperbounds[2]){
		printf("Vector %f, %f, %f out of range\n", par_position[0], par_position[1], par_position[2]);
		return best_configuration;
	}
	vector<int> binned_vector(3);
	//printf("binning Vector %f, %f, %f\n", par_vector[0], par_vector[1], par_vector[2]);
	
	binned_vector[0] = floor((par_position[0]-lowerbounds[0])*61/(upperbounds[0]-lowerbounds[0]));//4dhandonly has own parameters
	binned_vector[1] = floor((par_position[1]-lowerbounds[1])*61/(upperbounds[1]-lowerbounds[1]));
	binned_vector[2] = floor((par_position[2]-lowerbounds[2])*21/(upperbounds[2]-lowerbounds[2]));
	
	vector< vector <int> > binned_obstacles;
	for (int i = 0; i < obstacles.size(); i++){
		vector<int> temp(3,0);
		if (obstacles[i][0] < lowerbounds[0] || obstacles[i][1]  < lowerbounds[1] || obstacles[i][2]  < lowerbounds[2] || obstacles[i][0]  > upperbounds[0] || obstacles[i][1]  > upperbounds[1] || obstacles[i][2]  > upperbounds[2]){
			printf("Obstacle Vector %f, %f, %f out of range\n", obstacles[i][0], obstacles[i][1], obstacles[i][2]);
			continue ;
		}
		temp[0] = floor((obstacles[i][0]-lowerbounds[0])*xycoarseness/(upperbounds[0]-lowerbounds[0]));
		temp[1] = floor((obstacles[i][1]-lowerbounds[1])*xycoarseness/(upperbounds[1]-lowerbounds[1]));
		temp[2] = floor((obstacles[i][2]-lowerbounds[2])*zcoarseness/(upperbounds[2]-lowerbounds[2]));
		binned_obstacles.push_back(temp);
	}
	
	std::ifstream slicefile(("/home/andrej/Workspace/cspoutput/4dhandonly/slice_"+boost::to_string(binned_vector[0])+"_"+boost::to_string(binned_vector[1])+"_"+boost::to_string(binned_vector[2])+".dat").c_str(), ios::in);
	if (!slicefile)
	{
		printf("error 1 opening file %i, %i, %i\n", binned_vector[0], binned_vector[1], binned_vector[2]);
		return best_configuration;
	}
	float max_distance = 0;
	int comparisons_performed = 0;
 ///WARNING remove explicit dimensions here
	vector<float> current_configuration(dim,0);
	std::string line;
	vector<float> current_obstacle(dim,0);
	std::string line2;

	while (std::getline(slicefile, line))
	{	
		std::istringstream iss(line); 
		for (int i1 = 0; i1 < dim; i1++){
			if (!(iss >> current_configuration[i1])) { printf("error 1: cannot read line!\n"); break; }
		}
		
		for (int j = 0; j < binned_obstacles.size(); j++){
			std::ifstream obstfile(("/home/andrej/Workspace/cspoutput/4dsreduced/slice_"+boost::to_string(binned_obstacles[j][0])+"_"+boost::to_string(binned_obstacles[j][1])+"_"+boost::to_string(binned_obstacles[j][2])+".dat").c_str(), ios::in);
			if (!obstfile)
			{
				printf("error 2 opening file %i, %i, %i\n", binned_obstacles[j][0], binned_obstacles[j][1], binned_obstacles[j][2]);
				continue;
			}	
				while (std::getline(obstfile, line2))
				{
					std::istringstream iss2(line2); 
					for (int i = 0; i < dim; i++){
						if (!(iss2 >> current_obstacle[i])) { printf("error 2: cannot read line!\n"); break; }
					}
					vector<float> rpos = joint_to_complete_cartesian(current_configuration, 4);
					if (vectordistance(current_obstacle, current_configuration) > max_distance && rpos[2] > 0.2 && current_configuration[3] > 0  && current_configuration[1] < 0){
						max_distance = vectordistance(current_obstacle, current_configuration);
						best_configuration = current_configuration;
					}
					comparisons_performed++;
				}
		}
	}
	/*if (comparisons_performed == 0 || best_configuration.size() < 3)
		//printf("Warning: no configurations found for file %i, %i, %i \n", binned_vector[0], binned_vector[1], binned_vector[2]);
	else
		//printf("Best configuration found over %i comparisons for %i, %i, %i\n", comparisons_performed, binned_vector[0], binned_vector[1], binned_vector[2]);
	*/
	return best_configuration;
	
}


void cspaceconverter::launch_obstacle_thread(){
	pthread_t thread;
	lister_parameters* t = new lister_parameters(this, &list_lock, &position_lock, &configuration_lock, &intermediate_configuration_lock, &position, &obstacle_list, &intermediate_configuration_list, &configuration_list, &distances_list); 
	if (mode == 0)
		int rc = pthread_create(&thread, NULL, update_obstacle_list, (void *)t);
	else{
		int rc = pthread_create(&thread, NULL, update_obstacle_list_split, (void *)t);
		int rc2 = pthread_create(&thread, NULL, sort_obstacle_list, (void *)t);
		}
     
	
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
	//printf("locked mutex at %i from main thread, code %i\n", &(configuration_lock), pthread_mutex_lock(&configuration_lock));
	std::list<float>::iterator iter;
	for(iter = distances_list.begin(); iter != distances_list.end(); iter++)	
		output.push_back(*iter);
	
	pthread_mutex_unlock(&configuration_lock);
	return output;
	
	
}
vector<float> cspaceconverter::get_smoothest_configuration(vector<float> par_vector){  ///TODO NOT FIXED
	vector<int> binned_vector(3);
	binned_vector[0] = floor((par_vector[0]-lowerbounds[0])*61/(upperbounds[0]-lowerbounds[0])); //WARNING: 4dhandonly uses its own parameters
	binned_vector[1] = floor((par_vector[1]-lowerbounds[1])*61/(upperbounds[1]-lowerbounds[1]));
	binned_vector[2] = floor((par_vector[2]-lowerbounds[2])*21/(upperbounds[2]-lowerbounds[2]));
	std::ifstream slicefile(("/home/andrej/Workspace/cspoutput/4dhandonly/slice_"+boost::to_string(binned_vector[0])+"_"+boost::to_string(binned_vector[1])+"_"+boost::to_string(binned_vector[2])+".dat").c_str(), ios::in);
	vector<float> output;
	if (!slicefile)
	{
		printf("error 3 opening file %i, %i, %i\n", binned_vector[0], binned_vector[1], binned_vector[2]);
		return output;
	}
	int pointscontrolled;
	int dim = 4; ///[TODO] warning this caused trouble before. fix it later
	vector<float> loadedpoint(dim);
	//float mindist = 1000000.0;
	float mindist = 000.0;
	pointscontrolled = 0;
	std::string line;
	vector<float> optimal(4);
	optimal[0] =-0.71; ///7 dimensions
	optimal[1] =-0.72;
	optimal[2] =0.13;
	optimal[3] =1.59;
	int linecount = 0;
	while (std::getline(slicefile, line))
	{	
		std::istringstream iss(line); 
		//std::cout << line << std::endl;
		for (int i = 0; i < dim; i++){
			if (!(iss >> loadedpoint[i])) { printf("error: cannot read line!\n"); break; }
		}
		//printf("loaded point: %f, %f, %f, %f\n", loadedpoint[0], loadedpoint[1], loadedpoint[2], loadedpoint[3]);
		float dist = 0;
		for (int i = 0; i < dim; i++)
			dist += (loadedpoint[i])*(loadedpoint[i]);
			//dist += (loadedpoint[i]-optimal[i])*(loadedpoint[i]-optimal[i]);
		dist = sqrt(dist);
		vector<float> rpos = joint_to_complete_cartesian(loadedpoint, 4);
		//if (dist < mindist /*&& vectordistance(rpos,  par_vector) < 0.1*/ && rpos[2] > 0.25){
		if (rpos[2] > mindist && loadedpoint[1] < 0 && loadedpoint[3] > 0){
			mindist = rpos[2];
			output = loadedpoint;
		}
		linecount++;
	}
	if (linecount == 0){
		printf("Error: no lines in file %i, %i, %i \n", binned_vector[0], binned_vector[1], binned_vector[2]);
		sleep(5);
	}
	if (mindist > 99999.9)
		printf("Error: no configuration found for point; %i, %i, %i over %i lines\n", binned_vector[0], binned_vector[1], binned_vector[2], linecount);
	if (output.size() <1){
		printf("Error: bad output size %i, dist is %f\n", output.size(), mindist);
	sleep(5);
	}//printf("Found configuration over %i lines for point %i, %i, %i\n", linecount, binned_vector[0], binned_vector[1], binned_vector[2]);
	return output;

}

vector<float> cspaceconverter::joint_to_cartesian(vector<float> jointvalues, int segmentnumber){
	
	//printf("parsing %i values for %i joints \n", jointvalues.size(), jointNumber);
	vector<float> cartesianvalues(3, 0);
	if (jointvalues.size() == 0) 
		return cartesianvalues;

	KDL::Frame cartpos;
	KDL::JntArray q(jointNumber);
	for (int i = 0; i < jointNumber; i++){
		if (jointvalues.size()> i)
			q(i) = jointvalues[i];
		else  q(i) = 0;
	}
		
	bool kinematics_status = kinematic_solver->JntToCart(q, cartpos, segmentnumber);
	

	cartesianvalues[0] = cartpos.p.x();
	cartesianvalues[1] = cartpos.p.y();
	cartesianvalues[2] = cartpos.p.z();
	return cartesianvalues;
}
vector<float> cspaceconverter::joint_to_complete_cartesian(vector<float> jointvalues, int segmentnumber){
	
	//printf("parsing %i values for %i joints \n", jointvalues.size(), jointNumber);
	vector<float> cartesianvalues(3, 0);
	if (jointvalues.size() == 0) 
		return cartesianvalues;

	KDL::Frame cartpos;
	KDL::JntArray q(jointNumber);
	for (int i = 0; i < jointNumber; i++){
		if (jointvalues.size()> i)
			q(i) = jointvalues[i];
		else  q(i) = 0;
	}
		
	bool kinematics_status = kinematic_solver->JntToCart(q, cartpos, segmentnumber);
	cartpos = baseframe*cartpos;

	cartesianvalues[0] = cartpos.p.x();
	cartesianvalues[1] = cartpos.p.y();
	cartesianvalues[2] = cartpos.p.z();
	return cartesianvalues;
}
KDL::Vector cspaceconverter::joint_to_KDLvector(vector<float> jointvalues, int segmentnumber){
	
	//printf("parsing %i values for %i joints \n", jointvalues.size(), jointNumber);
	KDL::Vector cartesianvalues;
	if (jointvalues.size() == 0) 
		return cartesianvalues;

	KDL::Frame cartpos;
	KDL::JntArray q(jointNumber);
	for (int i = 0; i < jointNumber; i++)
		q(i) = jointvalues[i];
		
		
	bool kinematics_status = kinematic_solver->JntToCart(q, cartpos, segmentnumber);
	
	return cartpos.p;
}

KDL::Frame cspaceconverter::joint_to_KDLFrame(vector<float> jointvalues, int segmentnumber){
	
	//printf("parsing %i values for %i joints \n", jointvalues.size(), jointNumber);
	KDL::Frame cartesianvalues;
	if (jointvalues.size() == 0) 
		return cartesianvalues;

	KDL::Frame cartpos;
	KDL::JntArray q(jointNumber);
	for (int i = 0; i < jointNumber; i++)
		q(i) = jointvalues[i];
		
		
	bool kinematics_status = kinematic_solver->JntToCart(q, cartpos, segmentnumber);
	
	return cartpos;
}


