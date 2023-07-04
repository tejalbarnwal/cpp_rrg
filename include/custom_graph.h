#ifndef CUSTOM_GRAPH_H
#define CUSTOM_GRAPH_H

#include<iostream>
#include<utility>
#include<vector>

using namespace std;

struct Edge{
	unsigned int destVertexID;
	double weight;

	Edge(unsigned int destVertexID_, double weight_)
	{
	    destVertexID = destVertexID_;
	    weight = weight_;
	}

};

class Vertex{
public:
    unsigned int vertexID;
    pair<double, double> posXY;
    vector<Edge> edgeList;

    Vertex(unsigned int id, double x, double y){
        vertexID = id;
        posXY.first = x;
        posXY.second = y;
    }

    void printEdgeList();
};



class Graph{
public:
    vector<Vertex> vertices;

    bool checkIfVertexExist(unsigned int id);

    void addVertex(Vertex newVertex);

    bool checkIfEdgeExist(Vertex from, Vertex to);
    
    void addEdge(Vertex v1, Vertex v2, double weight);

    void printGraph();
 
};

class solutionNode{
public:
    unsigned int parentID;
    unsigned int vertexID;
    pair<double, double> posXY;
    double eucDistance;
    double cumEucDistance;
    double manhDistance;
    double cumManhDistance;

    solutionNode(unsigned int parentID_, 
                unsigned int vertexID_,
                Graph &g,
                double cumEucDistance_,
                double cumManhDistance_);

    void print();

};


#endif


