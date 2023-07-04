#include <iostream>
#include <vector>
#include <list>
#include <limits>

#include "../include/tspnn.h"

using namespace std;

int main(int argc, char const *argv[])
{
	Graph G;
	Vertex v1 = Vertex(0, 0.0, 0.0);
	Vertex v2 = Vertex(1, 3.0, 4.0);
	Vertex v3 = Vertex(2, 6.0, 8.0);
	Vertex v4 = Vertex(3, 12.0, 16.0);
	Vertex v5 = Vertex(4, 9.0, 12.0);

	G.addVertex(v1);
	G.addVertex(v2);
	G.addVertex(v3);
	G.addVertex(v4);
	G.addVertex(v5);

	G.addEdge(v1, v2, 5.0);
	// G.addEdge(v1, v3, 504);
	G.addEdge(v1, v4, 20.0);
	// G.addEdge(v1, v5, 423);

	G.addEdge(v2, v3, 5.0);
	G.addEdge(v2, v4, 15.0);
	G.addEdge(v2, v5, 10.0);

	G.addEdge(v3, v4, 10.0);
	G.addEdge(v3, v5, 5.0);

	G.addEdge(v4, v5, 5.0);

	G.printGraph();

	unsigned int n = 5;
	vector<vector<double>> 
		distanceMatrix( n , vector<double> (n, numeric_limits<double>::infinity()));

	tspNN::adjMatrix(distanceMatrix, G, n);

	for(unsigned int i=0; i<n; i++)
	{
		for(unsigned int j=0; j<n; j++)
		{
			cout << distanceMatrix[i][j] << " ";
		}
		cout << endl;
	}

	// tspNN::solveTSP(distanceMatrix);
	vector<vector<double>> 
		updatedDistanceMatrix( n , vector<double>(n, numeric_limits<double>::infinity()));

	
	// vector<int> parents(n, 0);
	// vector<vector<unsigned int>> explicitParents(5);

	// tspNN::Dijkstra(distanceMatrix, 0, updatedDistanceMatrix.at(0), parents);

	// for(unsigned int i=0; i<n; i++)
	// {
	// 	cout << parents[i] << " ";
	// }
	// cout << "\n-------------\n";

	// for (unsigned int j=0; j<n; j++){
	// 	cout << "dest: " << j << endl;
	// 	vector<unsigned int> v;
	// 	tspNN::storeExplicitParents(0, j, parents, v);
	// 	explicitParents.at(j) = v;
	// }

	// cout << "-------------\n";

	// for(unsigned int i=0; i<explicitParents.size(); i++)
	// {
	// 	for(unsigned int j=0; j<explicitParents.at(i).size(); j++)
	// 	{
	// 		cout << explicitParents[i][j] << " ";
	// 	}
	// 	cout << endl;
	// }

	vector<vector<vector<unsigned int>>> explicitParents(n, 
				vector<vector<unsigned int>>(n));

	for(unsigned int k=0; k<n; k++)
	{
		// vector<double> &dist = updatedDistanceMatrix.at(k);
		vector<int> parents(n, 0);

		tspNN::Dijkstra(distanceMatrix, k, updatedDistanceMatrix.at(k), parents);
		// for(unsigned int i=0; i<n; i++)
		// {
		// 	cout << parents[i] << " ";
			
		// }
		cout << endl;
		for (unsigned int j=0; j<n; j++){
			vector<unsigned int> v;
			tspNN::storeExplicitParents(k, j, parents, v);
			explicitParents.at(k).at(j) = v;
		}
		cout << "--------at k: " << k << endl;
		for(unsigned int a=0; a<explicitParents.at(k).size(); a++)
		{
			for(unsigned int j=0; j<explicitParents.at(k).at(a).size(); j++)
			{
				cout << explicitParents[k][a][j] << " ";
			}
			cout << endl;
		}
		
	}

	vector<unsigned int> tour;
	tspNN::solveTSP(updatedDistanceMatrix, tour);

	list<solutionNode> solution;
	tspNN::storeSolution(distanceMatrix, tour, explicitParents, G, solution);


	return 0;
}

