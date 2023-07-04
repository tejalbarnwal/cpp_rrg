#include <iostream>
#include <vector>
#include <list>
#include <limits>
#include "cmath"
#include "custom_graph.h"

using namespace std;

namespace tspNN
{
	void adjMatrix(vector<vector<double>> &v, Graph &g, unsigned int n)
	{
		for(unsigned int i=0; i<g.vertices.size(); i++)
		{
			Vertex temp = g.vertices.at(i);

			for (unsigned int j=0; j<temp.edgeList.size(); j++)
			{
				Edge e = temp.edgeList.at(j);
				v[i][e.destVertexID] = e.weight;
			}
		}
	}


	unsigned int minDist(vector<double> &dist, vector<bool> &visited)
	{
		double min = numeric_limits<double>::infinity();
		unsigned int index;

		for(unsigned int i=0; i<visited.size(); i++)
		{
			if(visited.at(i) == false && dist.at(i) <= min)
			{
				min = dist.at(i);
				index = i;
			}
		}
		return index;
	}


	void storeExplicitParents(unsigned int src, unsigned int dest, 
							vector<int> &parents, 
							vector<unsigned int> &explicitParents)
	{
		unsigned int n = parents.size();

		if (dest == -1) {
        return;
	    }
	    storeExplicitParents(src, parents[dest], parents, explicitParents);
	    // cout << dest << "\n";
	    explicitParents.push_back(dest);

	}

	void Dijkstra(vector<vector<double>> &distMat, unsigned int src, vector<double> &dist, vector<int> &parents)
	{
		unsigned int n = distMat.size();
		// vector<double> dist(n, numeric_limits<double>::infinity());
		vector<bool> visited(n, false);

		dist.at(src) = 0;
		parents.at(src) = -1;

		for(unsigned int i=0; i<n; i++)
		{
			unsigned int m = minDist(dist, visited);

			visited.at(m) = true;
			for(unsigned int j=0; j<n; j++)
			{
				if(( !visited.at(j) ) && 
					( dist.at(m) + distMat[m][j] < dist.at(j) ) &&
					distMat[m][j]!=numeric_limits<double>::infinity())
				{
					parents.at(j) = m;
					dist.at(j) = dist.at(m) + distMat[m][j];
				}
			}
		}

		// cout<<"Vertex\t\tDistance from source"<<endl;
		// for(unsigned int i = 0; i<n; i++)                      
		// { //Printing
		// 	char str=65+i; // Ascii values for pritning A,B,C..
		// 	cout<<str<<"\t\t\t"<<dist[i]<<endl;
		// }
	}


	void printTour(vector<unsigned int> &l)
	{
		cout << "The near optimal path: \n";
		for (auto it = l.begin(); it!=l.end(); it++){
			cout << (*it) << " , ";
		}
		cout << endl;
	}

	void solveTSP(vector<vector<double>> &distMat, vector<unsigned int> &tour)
	{
		cout << "Solve TSP with heuristic based method to increase speed\n";
		
		unsigned int num_vertex = distMat.size();

		vector<bool> visited(num_vertex, false);
		double total_distance = 0.0;

		// start the at the 0 vertex
		unsigned int current_city = 0;
		tour.push_back(current_city);
		visited.at(current_city) = true;

		// repeat until all cities are visted
		while (tour.size() < num_vertex)
		{
			unsigned int nearest_city = 0;
			double nearest_distance = numeric_limits<double>::infinity(); 

			// find the nearest unvisited city
			for (unsigned int i=0; i<num_vertex; i++)
			{
				if (!visited.at(i))
				{
					double distance = distMat[current_city][i];
					if (distance < nearest_distance)
					{
						nearest_city = i;
						nearest_distance = distance;
					}
				}
			}

			// move to nearest city
			current_city = nearest_city;
			tour.push_back(current_city);
			visited.at(current_city) = true;
			total_distance += nearest_distance;
			// cout << "total distance: " << total_distance << endl;
		}

		tour.push_back(0);
		total_distance += distMat[current_city][0];

		printTour(tour);
		cout << "Total path length: " << total_distance << endl;

	}

	
	void storeSolution(vector<vector<double>> &distMat, vector<unsigned int> &tour, 
					vector<vector<vector<unsigned int>>> &paths, Graph &g, list<solutionNode> solution)
	{
		cout << "Detailed Sequence of Waypoints:\n";

		double eucDistance_ = 0.0;
		double cumEucDistance_ = 0.0;
		double manhDistance_ = 0.0;
		double cumManhDistance_ = 0.0;

		solutionNode start_node = solutionNode(tour[0], 
												tour[0],
												g, 
												cumEucDistance_,
												cumManhDistance_);
		start_node.print();

		solution.push_back(start_node);

		for (unsigned int i=1; i<tour.size(); i++)
		{
			// check the index of the vertex
			unsigned int prevVertexID_ = tour[i-1];
			unsigned int vertexID_ = tour[i];
			// check if there exist an edge between prev index and current index
			bool edgeExist = (distMat[prevVertexID_][vertexID_] < numeric_limits<double>::infinity()) ;

			// if edge exists
				// use dist matrix to get the euc distance
				// use difference of x and y for manhattan distance
				// update cumulative distances
				// push back the new node
			if (edgeExist)
			{
				solutionNode temp = solutionNode(prevVertexID_,
												vertexID_,
												g, 
												cumEucDistance_,
												cumManhDistance_);
				cumEucDistance_ = temp.cumEucDistance;
				cumManhDistance_ = temp.cumManhDistance;
				temp.print();
				solution.push_back(temp);
				// prevVertexID_ = vertexID_;
			}
			else
			{
				unsigned int tempPrevVertexID_ = prevVertexID_;
				for(unsigned int j=1; j<paths.at(prevVertexID_).at(vertexID_).size(); j++)
				{
					unsigned int tempVertexID_ = paths[prevVertexID_][vertexID_][j];
					// cout << "j: " << j << " vid: " << tempVertexID_ << "\n"; 
					solutionNode temp = solutionNode(tempPrevVertexID_,
												tempVertexID_,
												g, 
												cumEucDistance_,
												cumManhDistance_);

					cumEucDistance_ = temp.cumEucDistance;
					cumManhDistance_ = temp.cumManhDistance;
					temp.print();
					solution.push_back(temp);
					tempPrevVertexID_ = tempVertexID_;
				}
			}
			// if edge not exist
				// use the paths DS to extract the path in between
				// since the path is explicit
				// we can use the logic stated above and keep pushing back
			
		}
		cout << endl;

	}


	
}