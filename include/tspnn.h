#ifndef TSPNN_H
#define TSPNN_H

#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include "custom_graph.h"

using namespace std;

namespace tspNN
{
	void adjMatrix(vector<vector<double>> &v, Graph &g, unsigned int n);

	unsigned int minDist(vector<double> &dist, vector<bool> &visited);

	void storeExplicitParents(unsigned int src, unsigned int dest, 
							vector<int> &parents, 
							vector<unsigned int> &explicitParents);

	void Dijkstra(vector<vector<double>> &distMat, 
				unsigned int src, vector<double> &dist, vector<int> &parents);

	void printTour(vector<unsigned int> &l);

	void solveTSP(vector<vector<double>> &distMat, vector<unsigned int> &tour);

	void storeSolution(vector<vector<double>> &distMat, vector<unsigned int> &tour, 
					vector<vector<vector<unsigned int>>> &paths, Graph &g, list<solutionNode> solution);

}

#endif