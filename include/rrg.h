#pragma once

#include<iostream>
#include<utility>
#include <random>
#include <ctime>


#include "custom_graph.h"

using namespace std;

class RRG{
public:
	double lowerBoundX, lowerBoundY, upperBoundX, upperBoundY;

	pair<double, double> v_init;

	// obstacle data
	vector<pair<double, double>> obstacleCenter;
	vector<double> obstacleRadius;

	// algorithm parameters
	unsigned int nIterations;
	double stepSize;
	double neighbourRadius;

	// solution
	Graph rrg;

public:
	RRG(double lowerBoundX_, double lowerBoundY_, 
		double upperBoundX_, double upperBoundY_,
		pair<double, double> start,
		vector<pair<double, double>> obstacleCenter_,
		vector<double> obstacleRadius_);

	void setParameters(unsigned int nIterations_, 
						double stepSize_, 
						double neighbourRadius_);

	pair<double,double> createRandomPosition();

	pair<Vertex, double> findNearestVertex(pair<double, double> posXY);

	Vertex steerVertex(Vertex x_nearest, pair<double, double> posXY, 
						unsigned int ID);

	bool obstacleFree(Vertex x_nearest, Vertex x_new);

	vector<Vertex> findNeighbour(Vertex x_new);
};