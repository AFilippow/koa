
#include <string.h> 
#include <stdio.h>   
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include "vectormath.h"
#include "xdmp.h"

using namespace std;

xDMP::xDMP(){
	alpha_z = ALPHA_Z;
	beta_z = BETA_Z;
	alpha_v = ALPHA_V;
	alpha_z = ALPHA_Z;
	alpha_w = ALPHA_W;
	LAMBDA = 0.5;

}

xDMP::~xDMP(){
//fclose(positionAndDistanceOutput);
}

void xDMP::init_dmp(int dim, std::vector<float> start, std::vector<float> goal, float total_t, float delta_t, float temp_scaling, int n_kernels, float width){
	//DISTANCE_SHORTENER = 0.10;
	LAMBDA = 0.5;
	motion_persistence_weight = 0.95;
	
	dimensions = dim;
	s=start;
	g=goal;
	T=total_t;
	dt=delta_t;
	tau=temp_scaling;
	n=n_kernels;
	v=1;
	r=s;
	y=s;
	dz_previous.resize(dim, 0);
	f.resize(dimensions);
	z.resize(dimensions);
	y.resize(dimensions);
	r.resize(dimensions);
	for (int i =0; i < dimensions;i++)
	{
		f[i]=0;
		z[i]=0;
	}
	c.resize(n);
	sigma.resize(n);
	w.resize(n);
	deviation.resize(dimensions);
	persistent_deviation.resize(dimensions,0);
	persistentDeviationChange.resize(dimensions,0);
	for (int i=0; i<n; i++){
		c[i] = i/(float)(n-1);
		sigma[i]=width;
		w[i].resize(dimensions);
		for (int j =0; j < dimensions;j++)
		{
			w[i][j]=0;
		}
	}
	obstacle.resize(0);
	positionAndDistanceOutput = fopen("/home/andrej/Workspace/xdmp_posdist.txt","w");
	deviationOutput = fopen("/home/andrej/Workspace/xdmp_deviation.txt","w");
	cosAngles = fopen("/home/andrej/Workspace/xdmp_cosangle.txt","w");
	gradientValues = fopen("/home/andrej/Workspace/xdmp_grad.txt","w");
	speedAndAcceleration = fopen("/home/andrej/Workspace/xdmp_speedaccel.txt","w");
	obstacleList = fopen("/home/andrej/Workspace/xdmp_obstacles.txt","w");
	persistenceOutput = fopen("/home/andrej/Workspace/xdmp_persistence.txt","w");
	positionAndMotion = fopen("/home/andrej/Workspace/xdmp_positionmotion.txt","w");
	goalfunction = fopen("/home/andrej/Workspace/xdmp_goalfunction.txt","w");
	verbose = fopen("/home/andrej/Workspace/xdmp_verbose.txt","w");
	forces = fopen("/home/andrej/Workspace/xdmp_forces.txt","w");
	internaldistances = fopen("/home/andrej/Workspace/xdmp_idist.txt","w");
	angleanddistance = fopen("/home/andrej/Workspace/xdmp_adist.txt","w");

	timestep = 0;
	persistent_direction_of_motion.resize(dim, 0);
}

void xDMP::reset()
{
	v=1;
	r=s;
	y=s;
	for (int i =0; i < dimensions;i++)
	{
		f[i]=0;
		z[i]=0;
	}
	persistent_deviation.resize(dimensions,0);
}


void xDMP::calculate_one_step_dmp(float t){
	double dv;
	vector<float> dr;
	vector<float> dz;
	dr.resize(dimensions);
	dy.resize(dimensions);
	dz.resize(dimensions);
	double a;
	float psi;
	float sum_psi;

	if (t==0){
		fprintf(positionAndMotion,"----------------------------------------------\n\n\n\n\n\n",0);
		for (int i =0; i < dimensions;i++)
		{
			z[i]=0;
			f[i]=0;
		}
		r=s;
		y=s;
		v=1;
	}
	else
	{
		
		a=exp((alpha_v/dt)*(tau*T-t));		//warning: this is NAN for SMALL DT
		dv=-(alpha_v*a)/((1+a)*(1+a));//*dt;
		if (isnan(dv)){
			dv=0;
		}
		//printf("V: %f \n", v);
		
		//dv = -alpha_v * v / tau/400;
		v=v+dv;


		for (int i =0; i < dimensions;i++)
		{
			f[i]=0;
		}
		sum_psi=0;
		for (int i=0; i<n; i++){

			a=t/(tau*T)-c[i];
			psi=exp(-a*a/(2*sigma[i]*sigma[i]));
			sum_psi=sum_psi+psi;
			for (int j =0; j < dimensions;j++)
			{
				f[j]=f[j]+psi*w[i][j]*v;
				//printf("f[%i]: %f \n",j,f[j]);
			}
		}

		// unmodified DMP:
		//for (int j =0; j < dimensions;j++)
		//{
			//f[j]=f[j]/sum_psi;
			//dz[j]=(1/tau)*(alpha_z*(beta_z*(r[j]-y[j])-z[j])+f[j]);
			//z[j]=z[j]+dz[j];
			//dy[j]=(1/tau)*z[j];
			//y[j]=y[j]+dy[j];
		//}
		//Experimental redefinition of R as a radius from the goal
		vector<float> radialR = vector_difference(y, g);
		float rlength = vector_length(vector_difference(g, r));
		float distance_to_goal = vector_length(vector_difference(y, g));

		radialR= scalar_product(rlength / distance_to_goal, radialR);
		radialR= vector_sum(g, radialR);
		//fprintf(goalfunction, "%i \t %f \t %f \t %f \t %f \n", timestep, r[0], r[1], radialR[0], radialR[1]);
		//experimental: if the speed is to high, we slow it down and ...
		float time_scaling_factor = 1;
		//First half
		///here is the transformation half of the DMP system
		for (int j =0; j < dimensions;j++)
		{
			f[j]=f[j]/sum_psi;
			//EXPERIMENTAL CHANGE
			//dz[j]=(1/tau)*(alpha_z*(beta_z*(r[j]-y[j])-z[j])+f[j]);
			dz[j]=(1/tau)*(alpha_z*(beta_z*(radialR[j]-y[j])-z[j])+f[j]);  ///good one
			//float K = alpha_z*beta_z;
			//float D = alpha_z;
			//dz[j]=(1/tau)*K*(g[j]-y[j])-D*z[j]/*-K*(g[j]-s[j])*v*/+K*f[j];
		}
	
		fprintf(forces,"%i \t %f \t", timestep, vector_length(dz));
		fprintf(positionAndDistanceOutput, "%i \t %f \t %f \t %f \t", timestep, y[0], y[1], y[2]);
		vector<float> dz_addition(dimensions, 0);
		vector<float> current_dz_addition(dimensions, 0);
		//debugging
		float total_dz = 0;
		
		///obstacle avoidance here
		float avoidances = 0;
		if (obstacle.size()>0)
		{
			vector<float> currentMotion = dy;
			for(int i = 0; i < obstacle.size(); i+=dimensions)
			{
				vector<float> o(obstacle.begin()+i, obstacle.begin()+i+dimensions);
				//printf("Vector o: %f \t %f \t %f \t %f \n", obstacle[i+0], obstacle[i+1], obstacle[i+2], obstacle[i+3]);
				current_dz_addition = avoid_obstacle(o, y, dy, distances[i/dimensions]-DISTANCE_SHORTENER, tau);///good old one
				//current_dz_addition = avoid_obstacle_other_way(o, y, dy, distances[i/dimensions]-DISTANCE_SHORTENER, tau); ///new experimental one 
				//fprintf(forces,"%f \t", vector_length(current_dz_addition));
				if (vector_length(current_dz_addition) >= 0.0001)
					avoidances+=1.0;
				dz_addition = vector_sum(dz_addition, current_dz_addition); 
				fprintf(positionAndDistanceOutput, "%f \t", distances[i/dimensions]);
				fprintf(obstacleList, "%i \t %f \t %f \t %f \n", timestep, o[0], o[1], o[2]);
				total_dz += vector_length(current_dz_addition);
				/*for (int j = 0; j < trackingPoints.size(); j+=dimensions)
				{
					vector<float> k(trackingPoints.begin()+j, trackingPoints.begin()+j+dimensions);
					float local_distance = vector_length(vector_difference(k, o));
					dz_addition = vector_sum(dz_addition, avoid_obstacle(o, k, currentMotion, local_distance-DISTANCE_SHORTENER, tau));
				}*/
			}
		}
		/*double scalar_product_of_additions = 0;
		for (int i = 0; i < dimensions; i++){
			scalar_product_of_additions += dz_addition[i]*dz_previous[i];
			printf("scalar product until now: %f\n",scalar_product_of_additions);
		}
		scalar_product_of_additions = scalar_product_of_additions/(vector_length(dz_addition)*vector_length(dz_previous));
		std::cout << scalar_product_of_additions << std::endl;
		printf("Ratio: %f, consistency %f \n",vector_length(dz_addition)/total_dz), scalar_product_of_additions;
		printf("dz_previous %f %f %f %f \n",dz_previous[0],dz_previous[1],dz_previous[2],dz_previous[3]);
		printf("dz_addition %f %f %f %f \n",dz_addition[0],dz_addition[1],dz_addition[2],dz_addition[3]);
		printf("vector_length(dz_addition)*vector_length(dz_previous) %f %f %f \n",vector_length(dz_addition)*vector_length(dz_previous),vector_length(dz_addition),vector_length(dz_previous));*/
		dz_previous = dz_addition;
		fprintf(forces,"%f \t", vector_length(dz_addition));
				
		fprintf(forces,"\n");
			fflush(forces);
		fprintf(positionAndDistanceOutput, "\n");
		//printf("%f additions\n", avoidances);
		///Table evasion does not work in joint space
		/*
		 * vector<float> table_evasion(3,0);
		table_evasion[2] = exp(-50*y[2]);
		dz_addition = vector_sum(dz_addition, table_evasion);*/
		//avoidance addition
		if (avoidances > 2.0){
			dz_addition = scalar_product(1/avoidances, dz_addition);
		}
		float goal_distance = vector_length(vector_difference(y,g));
		if (goal_distance < 0.8)
			dz_addition = scalar_product(goal_distance/0.8, dz_addition);
		dz = vector_sum(dz_addition, dz);
		

		
		
		///spring force towards zero configuration
		for (int i = 0; i < dimensions; i++){  ///something is still wrong here [TODO fix it]
			if (abs(y[i])>1.7)
				dz[i] -= (y[i]-1.7)*(y[i]+1.7)*y[i]/1000;
		}
		
		
		
		//second half of calculation step
		for (int j =0; j < dimensions;j++)
		{
			z[j]=z[j]+dz[j];
			dy[j]=(1/tau)*z[j];
			//changed here:
			}
		
		//velocity limit
		/*if (vector_length(dy)>0.08)	{
				time_scaling_factor = 0.08 / vector_length(dy);
				dy = scalar_product(time_scaling_factor, dy);
		}*/
			y= vector_sum(y,dy);
	/*	if (vector_length(dy) > 0.002)
			time_scaling_factor = scalar_product(vector_difference(g, y), dy)/(vector_length(dy)*vector_length(vector_difference(g, y)));
		else 
			time_scaling_factor = 1;
		
		t -= dt*(1-time_scaling_factor);*/
		//printf("Time scaling : %f, Velocity %f \n", time_scaling_factor, vector_length(dy));

		//The delayed goal function r causes the DMP to follow the trajectory more closely
		//reworking
		if (t<=tau*T){
			dr=(vector_difference(g,s));
			for (int i =0; i < dimensions; i ++)
				dr[i]=dr[i]*(1/tau)*(dt/T);
				//dr[i]=dr[i]*(1/tau)*(dt/T)*time_scaling_factor;
		}
		else{
			for (int i =0; i < dimensions;i ++)
				dr[i]=0;
		}
		r=vector_sum(r,dr);
		//r = g;
		
		fprintf(speedAndAcceleration, "%i \t %f \t %f \t %f \t %f \t %f \t %f \n", timestep, z[0], z[1], z[2], dz[0], dz[1], dz[2]);
		
	}
	if (t< 0.05) persistent_direction_of_motion = dy;
    //persistent_direction_of_motion = scalar_product(0.95,persistent_direction_of_motion);
    //persistent_direction_of_motion = vector_sum(scalar_product(motion_persistence_weight, persistent_direction_of_motion), scalar_product(1-motion_persistence_weight,dy));
    fprintf(persistenceOutput,"%i \t %f \t %f \t %f \t %f \t %f \t %f \n",timestep, dy[0],dy[1],dy[2],persistent_direction_of_motion[0],persistent_direction_of_motion[1],persistent_direction_of_motion[2]);
	fprintf(positionAndMotion," %f \t %f \t %f \t %f \t %f \n", y[0], y[1], y[2], y[3], sqrt(dy[0]*dy[0]+dy[1]*dy[1]+dy[2]*dy[2]+dy[3]*dy[3]));
	timestep++;
}


float xDMP::repulsive_field_value(vector<float> obstacle, vector<float> mobilePoint, vector<float> speed, float distance){

	 vector<float> vector_from_obstacle_to_actuator = vector_difference(mobilePoint, obstacle);
	 float cos_angle = cosine_angle(speed, vector_from_obstacle_to_actuator);
	 float traditional_field_value = LAMBDA*pow(-cos_angle, BETA)*vector_length(speed)/distance;
	 return traditional_field_value;//*(0.0005/pow(cos_angle+1.001, 1)*0.8/distance+1);

}
//CAUTION while the DMP receives distance values from outside, those are the regular euclidean distances minus DISTANCE_SHORTENER
//maybe necessary to pass proper distances later
vector<float> xDMP::gradient(vector<float> obstacle, vector<float> mobilePoint, vector<float> speed, float distance, float tau){
	vector<float> gradient;
	gradient.resize(dimensions);
	//double precision = 1.05-repulsive_field_value(obstacle, mobilePoint, speed, vector_length(vector_difference(mobilePoint, obstacle))-DISTANCE_SHORTENER)/0.8;
	double precision = 1.05-repulsive_field_value(obstacle, mobilePoint, speed, distance-DISTANCE_SHORTENER)/0.8;
	precision *= 0.001;
	precision = 0.0001;
	for (int i = 0; i < dimensions; i++){
		vector<float> closePoint1, closePoint2;
		closePoint1 = mobilePoint;
		closePoint2 = mobilePoint;
		closePoint1[i] += precision;
		closePoint2[i] -= precision;
		
		gradient[i] = (repulsive_field_value(obstacle, closePoint1, speed, vector_length(vector_difference(closePoint1, obstacle)))-repulsive_field_value(obstacle, closePoint2, speed, vector_length(vector_difference(closePoint2, obstacle))))/precision/2;
	}
	 //printf("gradient: %f,  %f,  %f \n", gradient[0], gradient[1], gradient[2]); 
    fprintf(gradientValues, "%i \t %f \t %f \t %f \n",timestep, gradient[0], gradient[1], gradient[2]); 

	return gradient;
}

vector<float> xDMP::mat_gradient(vector<float> obstacle, vector<float> mobilePoint, vector<float> speed, float distance, float tau){
	vector<float> return_gradient(dimensions, 0);
	vector<float> vector_from_obstacle_to_actuator = vector_difference(mobilePoint, obstacle);
	float cos_angle = cosine_angle(speed, vector_from_obstacle_to_actuator);
	
	if (cos_angle > 0 )	return return_gradient;
	
	float factor= LAMBDA * pow(-cos_angle, BETA - 1) * vector_length(speed) / distance;
	vector<float> nabla_angle_first_part = scalar_product(1/distance, speed);
	vector<float> nabla_angle_second_part = scalar_product(1/(distance*distance*distance),vector_from_obstacle_to_actuator);
	nabla_angle_second_part = scalar_product(scalar_product(speed, vector_from_obstacle_to_actuator), nabla_angle_second_part);
	
	vector<float> nabla_angle = scalar_product(BETA/vector_length(speed),vector_difference(nabla_angle_first_part, nabla_angle_second_part));
	vector<float> nabla_position = scalar_product(1/distance,vector_from_obstacle_to_actuator);
	nabla_position = scalar_product(cos_angle/distance,nabla_position);
	
	return_gradient = scalar_product(factor, vector_difference(nabla_angle ,nabla_position));
	
	//return_gradient[2] += 0.001/(cos_angle + 1);
	return return_gradient;
}



vector<float> xDMP::avoid_obstacle_other_way( vector<float> obstacle, vector<float> mobilePoint, vector<float> speed, float distance, float tau){
	for(int i = 0; i < dimensions; i++)
		fprintf(verbose, "%f \t", y[i]);
	float maxdistance = 0.5;
	if (distance >= maxdistance)
		{
			for ( int i = 0; i < deviation.size(); i++)
				deviation[i] = 0;
			fprintf(verbose, "\n");
			fflush(verbose);
			return deviation;
		}
	float force_factor = LAMBDA/100*tau*(1/distance/distance/maxdistance-1/distance/distance/distance);

	float euc_distance = vector_length(vector_difference(obstacle, mobilePoint));
	//printf("Force: %f made of %f, %f, %f, distance: %f\n", force_factor, LAMBDA/500*tau,1/distance/distance/maxdistance,1/distance/distance/distance, euc_distance);
	for (int i = 0; i < dimensions; i++){
			//deviation[i]*=0.95;
		deviation[i]=force_factor*(obstacle[i]-mobilePoint[i])/euc_distance;
		}
		

	for(int i = 0; i < dimensions; i++)
		fprintf(verbose, "%f \t", mobilePoint[i]-obstacle[i]);
	for(int i = 0; i < dimensions; i++)
		fprintf(verbose, "%f \t", obstacle[i]);
	for(int i = 0; i < dimensions; i++)
		fprintf(verbose, "%f \t", deviation[i]);
	fprintf(verbose, "%f \t %f", euc_distance, distance );
	fprintf(verbose, "\n");
	fflush(verbose);
	return deviation;
}
vector<float> xDMP::avoid_obstacle( vector<float> obstacle, vector<float> mobilePoint, vector<float> speed, float distance, float tau)
{
	for (int i = 0; i < dimensions; i++)
		deviation[i]= 0;//.8*deviation[i];
	vector<float> vector_from_obstacle_to_actuator = vector_difference(mobilePoint, obstacle);
	float cos_angle = cosine_angle(speed, vector_from_obstacle_to_actuator);
	fprintf(cosAngles, "%i \t %f \n", timestep, cos_angle); 
	/*printf("first vector: %f  %f  %f \n",speed[0], speed[1], speed[2]);
	printf("second vector: %f  %f  %f \n",vector_from_obstacle_to_actuator[0], vector_from_obstacle_to_actuator[1], vector_from_obstacle_to_actuator[2]);*/
	
	
	if (cos_angle < 0 )//(acos(cos_angle) <= 3.141527/4 /*&& acos(cos_angle) < 3.141527*/)
		{
			float p = distance;
			float pstrike = vector_length(vector_from_obstacle_to_actuator);
			//std::cout << "Distance p: shortest:"<< p << "\t cos of angle "<< cos_angle << std::endl ;
			if (p == -1) p = vector_length(vector_from_obstacle_to_actuator);
			//std::cout << "Distance p: to center:"<< vector_length(vector_from_obstacle_to_actuator)<< "\t used: ";
			//p = vector_length(vector_from_obstacle_to_actuator);
			//std::cout << p << "\n";
			fprintf(internaldistances, "%i \t %f \n", timestep, distance);
			fflush(internaldistances);
			fprintf(angleanddistance, "%i \t %f \t %f \n", timestep, distance, cos_angle);
			fflush(angleanddistance);

			
			if (CALCULATED_GRADIENT ==  1){
				deviation = mat_gradient(obstacle, mobilePoint, speed, distance, tau);
			}else{ 
				deviation = scalar_product(-1, gradient(obstacle, mobilePoint, speed, distance, tau));
			}

			/*vector<float> removeme = vector_difference(nabla_angle,nabla_position);
			fprintf(positionAndDistanceOutput, "%f \t %f \t %f \t %f \t",distance, removeme[0], removeme[1], removeme[2]);*/
				//printf("Calculated deviation with:\n positional part %f, %f, %f;\n angular part %f, %f, %f;\n", nabla_position[0],nabla_position[1],nabla_position[2],nabla_angle[0],nabla_angle[1],nabla_angle[2]);
			 //deviation[2] += 0.05*exp(-(cos_angle*cos_angle)*10);
			float total_deviation = 0;
			for(int i = 0; i < dimensions; i++)
			{
				total_deviation+= deviation[i]*deviation[i];
			}
			if (total_deviation >= 0.05*0.05)
				for(int i = 0; i < dimensions; i++)
					deviation[i] = deviation[i] * 0.05 /sqrt(total_deviation);
			//std::cout << "deviation: " << deviation[0]*deviation[0]+ deviation[1]*deviation[1] << "\n";
										//14, 15
			for(int i = 0; i < dimensions; i++)
				fprintf(verbose, "%f \t", y[i]);
			for(int i = 0; i < dimensions; i++)
				fprintf(verbose, "%f \t", vector_from_obstacle_to_actuator[i]);
			for(int i = 0; i < dimensions; i++)
				fprintf(verbose, "%f \t", obstacle[i]);
			fprintf(verbose, "\n");
				fflush(verbose);
			
		}
		else
		{
			/*printf("obstacle");
			for (unsigned int i = 0; i < obstacle.size(); i++)
				printf("%f, ",obstacle[i]);
			printf(" not in trajectory. \n");*/
		}
	fprintf(deviationOutput, "%i \t %f \t %f \t %f \n",timestep, deviation[0], deviation[1], deviation[2]);
	persistent_deviation = vector_sum(scalar_product(motion_persistence_weight, persistent_deviation),scalar_product(1-motion_persistence_weight, deviation));

	//return deviation;
	return persistent_deviation;
}






void xDMP::scan_vector_field()
{
	ofstream output("/home/andrej/Workspace/obstacleVectorField.txt");	
	
	
	vector<float> point(3);
	point[0] = 0;
	point[1] = 0;
	point[2] = 0;
	vector<float> dy(3);
	dy[0] = 0.5;
	dy[1] = 0;
	dy[2] = 0;
	vector<float> y(3);
	float fieldvalue;
	for (float i = -1; i <=0.3; i+=0.04)
		for (float j = -0.5; j <=0.5; j+=0.04)
		{
			y[0] = i;
			y[1] = j;
			y[2] = 0.00;
			fieldvalue=repulsive_field_value(point,y,dy,vector_length(vector_difference(y, point))-DISTANCE_SHORTENER);
			
			avoid_obstacle(point, y, dy, vector_length(vector_difference(y, point)), tau);
						
			vector<float> mat_deviation = mat_gradient(point, y, dy, vector_length(vector_difference(y, point))-DISTANCE_SHORTENER, tau);
			for(int i1 = 0; i1 < 3; i1++)
			{
				if ((mat_deviation[i1])*(mat_deviation[i1]) > 0.05*0.05) mat_deviation[i1] = ((mat_deviation[i1] > 0) - (mat_deviation[i1] < 0))*0.05;
				mat_deviation[i1] = mat_deviation[i1]*(1/(tau*tau));
			}
			/*vector<float> vector_from_obstacle_to_actuator = vector_difference(y, point);
			float cos_angle = cosine_angle(dy, vector_from_obstacle_to_actuator);
			float pstrike = vector_length(vector_difference(y, point));
			float p = pstrike;
			
			
			vector<float> nabla_angle_first_part = scalar_product(1/pstrike, dy);
			vector<float> nabla_angle_second_part = scalar_product(1/(pstrike*pstrike*pstrike),vector_from_obstacle_to_actuator);
			nabla_angle_second_part = scalar_product(scalar_product(dy, vector_from_obstacle_to_actuator), nabla_angle_second_part);
			
			
			vector<float> nabla_angle = scalar_product(BETA/vector_length(dy),vector_difference(nabla_angle_first_part, nabla_angle_second_part));
			vector<float> nabla_position = scalar_product(1/p,vector_from_obstacle_to_actuator);
	
			nabla_position = scalar_product(cos_angle/p,nabla_position);
			if (vector_length(nabla_position)>10) nabla_position = scalar_product(10/vector_length(nabla_position),nabla_position);
			deviation = scalar_product(LAMBDA*pow(-cos_angle, BETA-1)*vector_length(dy)/p/tau/tau, vector_difference(scalar_product(BETA, nabla_angle),nabla_position));
		
			//std::cout << "deviation: " << deviation[0] << " "<< deviation[1] << " "<< deviation[2] << "\n\n\n";
			float scf1 = sqrt(nabla_angle[0]*nabla_angle[0]+nabla_angle[1]*nabla_angle[1]);
			//y = vector_difference(y, deviation);*/
			if (fieldvalue > 0.03) fieldvalue = 0.03;
			if (i>0) fieldvalue = 0;
			output << y[0] << "\t" << y[1] << "\t" << deviation[0] << "\t" << deviation[1] <<"\t" << fieldvalue <<"\t" << mat_deviation[0] <<"\t" << mat_deviation[1] <<  "\n"  ;
			
			
		}
	output.close();
}

std::vector<float>  xDMP::get_y(){
	for (int i = 0; i < y.size(); i++)
		if (isnan(y[i])){
			printf("Error: Attempting to retrieve invalid value from DMP\n");
			return vector<float>(y.size(),0);
		}
	return y;	
}
void xDMP::set_obstacle(vector<float>& o, vector<float>& p)
{

	obstacle.resize(o.size());
	for(int i = 0; i < obstacle.size(); i++)
		obstacle[i] = o[i];
	distances.resize(p.size());
	for(int i = 0; i < distances.size(); i++)
		distances[i] = p[i];	
}
