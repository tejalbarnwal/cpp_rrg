#include <iostream>
#include <utility>
#include <vector>
#include <limits>
#include <list>

#include "../include/rrg.h"
#include "../include/matplotlibcpp.h"
#include "../include/geometry.h"
#include "../include/tspnn.h"

namespace plt = matplotlibcpp;

using namespace std;



int main(int argc, char const *argv[])
{
	
	cout << "\n---   Beginning of the script   ---\n";

	srand(time(0));
	// define worksapce boundary
	double lowerBoundX_ = -1.0;
	double upperBoundX_ = 1.0;
	double lowerBoundY_ = -1.0;
	double upperBoundY_ = 1.0;

	// define start positon
	pair<double, double> start_position = make_pair(-0.99, -0.99);

	// define obstacles
	vector<pair<double, double>> obsCenter;
	vector<double> obsRadius;

	// define obstacles
	obsCenter.push_back(make_pair(-0.5, -0.5));
	obsRadius.push_back(0.2);

	obsCenter.push_back(make_pair(0.2, 0.2));
	obsRadius.push_back(0.05);

	obsCenter.push_back(make_pair(0.7, -0.7));
	obsRadius.push_back(0.2);

	obsCenter.push_back(make_pair(-0.2, 0.2));
	obsRadius.push_back(0.1);

	obsCenter.push_back(make_pair(0.5, -0.2));
	obsRadius.push_back(0.05);

	// draw obstacles
	unsigned int n=50;
	vector<vector<double>> obsX(obsRadius.size(), vector<double>(n, 0));
	vector<vector<double>> obsY(obsRadius.size(), vector<double>(n, 0));

	for(unsigned int i=0; i<obsRadius.size(); i++)
	{
		for(unsigned int j=0; j<n; j++)
		{
			// cout << "i, j: " << i << ", " << j << endl;
			double t = 2*M_PI*j/n;
			obsX[i][j] = obsRadius[i]*cos(t) + (obsCenter[i].first);
	      	obsY[i][j] = obsRadius[i]*sin(t) + (obsCenter[i].second);
		}
	}
	plt::figure_size(1200, 780);

	cout << "\n---   Obstacle Details   ---\n";

	for(unsigned int i=0; i<obsRadius.size(); i++)
	{
		
		cout << "Obstacle Number: " << i+1 << endl;
		cout << "Obstacle Center: x= " << obsCenter[i].first << " y= " << obsCenter[i].second;
		cout << "Obstacle Radius: " << obsRadius[i] << endl;
		plt::scatter(obsX[i], obsY[i]);
	}

	// algortithm helper functions object
	RRG alg = RRG(lowerBoundX_, lowerBoundY_, upperBoundX_, upperBoundY_,
					start_position, obsCenter, obsRadius);

	// set parameters
	unsigned int nIter = 70;
	double stepSize_ = 0.2;
	double neighbourRadius_ = 0.45;
	alg.setParameters(nIter, stepSize_, neighbourRadius_);

	cout << "\n---   Create Rapidly Exploring Random Graph(RRG)   ---\n";

	unsigned int nIterI = 0;
	Vertex v = Vertex(nIterI, alg.v_init.first, alg.v_init.second);
	alg.rrg.addVertex(v);

	nIterI += 1;

	while(nIterI < nIter)
	{
		// cout << "Iteration: " << nIterI << endl;
		// create a random position
		pair<double, double> x_rand = alg.createRandomPosition();
		// find the nearest vertex
		pair<Vertex, double> v_nearest_info = alg.findNearestVertex(x_rand);
		Vertex v_nearest = v_nearest_info.first;
		Vertex v_new = Vertex(nIterI, x_rand.first, x_rand.second);

		// make sure that the vertex created is not too close
		while(v_nearest_info.second < alg.stepSize){
			x_rand = alg.createRandomPosition();
			v_nearest_info = alg.findNearestVertex(x_rand);
			v_nearest = v_nearest_info.first;
			v_new = Vertex(nIterI, x_rand.first, x_rand.second);
		}
		// create the vertex at the found "good" position
		Vertex temp = alg.steerVertex(v_nearest, x_rand, nIterI);
		v_new.posXY = temp.posXY;

		// check if the segment from neaest to new vertex is colllision free
		if (alg.obstacleFree(v_nearest, v_new))
		{
			alg.rrg.addVertex(v_new);
			alg.rrg.addEdge(v_nearest, v_new, geometry::eucDist(v_nearest.posXY, v_new.posXY));

			// find neighbour to add to the graph for the new vertex
			vector<Vertex> neighbours = alg.findNeighbour(v_new);
			for(auto neighbour : neighbours)
			{
				if(alg.obstacleFree(neighbour, v_new))
				{
					alg.rrg.addEdge(neighbour, v_new, 
							geometry::eucDist(neighbour.posXY, v_new.posXY));
				}
			}
			nIterI += 1;
		}
		
		// alg.rrg.printGraph();
		
	}

	// plot all the chosen vertices
	vector<double> vX, vY;
	for (unsigned int i =0; i<alg.rrg.vertices.size(); i++)
	{
		Vertex temp = alg.rrg.vertices.at(i);
		vX.push_back(temp.posXY.first);
		vY.push_back(temp.posXY.second);	
	}

	plt::scatter(vX, vY);


	// now that we have all the chosen vertices
	// we order the vertices into sequenced waypoints such that 
	// robot can cover all the waypoints in a sequence that
	// requires least amount of travel euclidean distance

	// temp variable created
	unsigned int n_ = alg.nIterations;

	// create an adjacency matrix out of adjacent list graph
	vector<vector<double>> 
		distanceMatrix( n_ , vector<double> (n_, numeric_limits<double>::infinity()));
	tspNN::adjMatrix(distanceMatrix, alg.rrg, n);

	// now we aim to solve the variaton of TSP
	// it is different in the ways as follows
	// simple TSP says that each vertex should be visited excatly once
	// but since RRG is a random graph, it might not be possible
	// to cover all vertices exactly once in just one and some vertices
	// may need revisits in order cover all the waypoints
	// so are TSP has a constraint of that each vertex should be visited
	// atleast once

	// create an updatedAdjaceny matrix to store euclidean distance from
	// each to each vertex using the shortest path between them
	vector<vector<double>> 
		updatedDistanceMatrix( n_ , vector<double> (n_, numeric_limits<double>::infinity()));

	
	// store the shortest paths from each each vertex to each vertex corresponding
	// the previous adjaceny matrix
	vector<vector<vector<unsigned int>>> explicitParents(n_, 
				vector<vector<unsigned int>>(n_));

	for(unsigned int k=0; k<n_; k++)
	{
		vector<int> parents(n_, 0);
		// update adjaceny matrix and obtain parents to each vertex
		tspNN::Dijkstra(distanceMatrix, k, updatedDistanceMatrix.at(k), parents);

		// store shortest paths
		for (unsigned int j=0; j<n_; j++){
			vector<unsigned int> v;
			tspNN::storeExplicitParents(k, j, parents, v);
			explicitParents.at(k).at(j) = v;
		}		
	}

	cout << "\n---   solve TSP   ---\n";
	vector<unsigned int> tour;
	tspNN::solveTSP(updatedDistanceMatrix, tour);

	cout << "\n---   Solution   ---\n";
	list<solutionNode> solution;
	tspNN::storeSolution(distanceMatrix, tour, explicitParents, alg.rrg, solution);

	plt::show();


	return 0;
}