/*
 * vectormath.cpp
 *
 *  Created on: Mar 7, 2014
 *      Author: afilippow
 */

#include "vectormath.h"
using namespace std;
float scalar_product(vector<float> x, vector<float> y)
{
	if (x.size() != y.size()) {
		cout << "Vector multiplication error: sizes not the same!\n";
		return 0;
	}
	float product =0;
	for (unsigned int i =0; i < x.size(); i++)
		product += x[i]*y[i];
	return product;
}
vector<float> vector_difference(vector<float> x, vector<float> y, string str){
	std::cout << str << "\n \n";
	return vector_difference(x,y);
	}
	
vector<float> vector_difference(vector<float> x, vector<float> y)
{
	vector<float> difference;
	difference.resize(x.size());
	if (x.size() != y.size()) {
		cout << "Vector subtraction error: size not the same!\n";
		return difference;
	}

	for (unsigned int i =0; i < x.size(); i++)
		difference[i] = x[i]-y[i];
	return difference;
}
				
				
vector<float> vector_sum(vector<float> x, vector<float> y)
{
	vector<float> sum;
	sum.resize(x.size());
	if (x.size() != y.size()) {
		cout << "Vector addition error: sizes not the same!\n";
		return sum;
	}

	for (unsigned int i =0; i < x.size(); i++)
		sum[i] = x[i]+y[i];
	return sum;
}
				
float vector_length(vector<float> x)
{
	float result = 0;
	for (unsigned int i = 0; i< x.size(); i++)
		result += x[i]*x[i];
	return sqrt(result);
}

float cosine_angle(vector<float> a, vector<float> b)
{
	return scalar_product(a, b)/vector_length(a)/vector_length(b);
}
vector<float> derivative_angle(vector<float> a, vector<float> b)
		{
	vector<float> output;
	output.resize(a.size());
	vector<float> delta;
	delta.resize(a.size());
	for (unsigned int i = 0; i < a.size(); i++)
	{
		for (unsigned int j = 0; j < delta.size(); j++)
		{
			delta[j] =0;
		}
		delta[i]=0.01;
		output[i] = (cosine_angle(a, vector_sum(b, delta)) - cosine_angle(a, vector_difference(b, delta))) / 0.02;
	}
	return output;
		}

vector<float> derivative_distance(vector<float> position, vector<float> obstaclePosition)
		{
	vector<float> output;
	output.resize(position.size());
	vector<float> delta;
	delta.resize(position.size());
	for (unsigned int i = 0; i < position.size(); i++)
	{
		for (unsigned int j = 0; j < delta.size(); j++)
		{
			delta[j] =0;
		}
		delta[i]=0.01;

		output[i] = (vector_length(vector_difference(obstaclePosition, vector_sum( position, delta))) - vector_length(vector_difference(obstaclePosition, vector_difference( position, delta)))) / 0.02  ;
	}
	return output;
}
vector<float> scalar_product(float a, vector<float> x)
{
	for (unsigned int i = 0; i < x.size(); i++)
	{
		x[i] *= a;
	}
	return x;
}




