#include "../include/custom_graph.h"
#include "../include/geometry.h"

void Vertex::printEdgeList(){
  for (auto& it: edgeList)
      {
        cout << it.destVertexID << ", ";
      }
  cout << "\n";
}


bool Graph::checkIfVertexExist(unsigned int id){
  for(unsigned int i=0; i<vertices.size(); i++){
    if (vertices.at(i).vertexID == id){
      return true;
    }
  }
  return false;
}


bool Graph::checkIfEdgeExist(Vertex from, Vertex to){
  vector<Edge> vEdgeList = from.edgeList;
  for(auto i : vEdgeList){
    if (i.destVertexID == to.vertexID){
      return true;
    }
  }
  return false;
}


void Graph::addVertex(Vertex newVertex){
  // check if a vertex with that ID already exists
  if (!checkIfVertexExist(newVertex.vertexID)){
    vertices.push_back(newVertex);
  }
  
}

void Graph::addEdge(Vertex v1, Vertex v2, double weight){
  // check if an edge already exist
  if (!checkIfEdgeExist(v1, v2)){
    Edge e12(v2.vertexID, weight);
    ((vertices.at(v1.vertexID)).edgeList).push_back(e12);
    Edge e21(v1.vertexID, weight);
    ((vertices.at(v2.vertexID)).edgeList).push_back(e21);
  }
}

void Graph::printGraph(){
  for(unsigned int i=0; i<vertices.size(); i++)
  {
    Vertex temp = vertices.at(i);
    cout << "vertex :" << temp.vertexID << " --> \t";
    temp.printEdgeList();
  }
}


solutionNode::solutionNode(unsigned int parentID_, 
                          unsigned int vertexID_,
                          Graph &g,
                          double cumEucDistance_,
                          double cumManhDistance_)
{
    parentID = parentID_;
    vertexID = vertexID_;
    posXY = g.vertices.at(vertexID).posXY;
    eucDistance = geometry::eucDist(g.vertices.at(parentID).posXY, 
                                  g.vertices.at(vertexID).posXY);
    manhDistance = geometry::manhDist(g.vertices.at(parentID).posXY, 
                                  g.vertices.at(vertexID).posXY);
    cumEucDistance = cumEucDistance_ + eucDistance;
    cumManhDistance = cumManhDistance_ + manhDistance;
}


void solutionNode::print()
{
  cout << "---------\n";
  cout << "parent waypoint vertex ID: " << parentID << endl;
  cout << "current waypoint vertex ID: " << vertexID << endl;
  cout << "current waypoint pos: x= " << posXY.first << " , y= " << posXY.second << endl;
  cout << "cumEucDistance from beginning: " << cumEucDistance << endl;
  cout << "cumManhDistance from beginning: " << cumManhDistance << endl;

}