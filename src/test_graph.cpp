#include<iostream>
#include "../include/custom_graph.h"
#include "../include/matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int argc, char const *argv[])
{
  Graph G;
  Vertex v1 = Vertex(0, 1.0, 2.0);
  G.addVertex(v1);
  // G.printGraph();

  Vertex v2 = Vertex(1, 2.0, 6.0);
  G.addVertex(v2);
  G.addEdge(v1, v2, 2.0);

  Vertex v3 = Vertex(2, 6.0, 7.0);
  G.addVertex(v3);
  G.addEdge(v1, v3, 4.0);

  G.printGraph();

  int n = 5000; // number of data points
  vector<double> x(n),y(n);
  for(int i=0; i<n; ++i) {
      double t = 2*M_PI*i/n;
      x.at(i) = 16*sin(t)*sin(t)*sin(t);
      y.at(i) = 13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t);
  }
  plt::plot(x, y, "r-", x, [](double d) { return 12.5+abs(sin(d)); }, "k-");
  // show plots
  plt::show();

  return 0;
}

// test syntax:
// g++ test_graph.cpp ../custom_graph.cpp -o test_graph
