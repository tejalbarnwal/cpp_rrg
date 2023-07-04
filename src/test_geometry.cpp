#include<iostream>
#include<utility>
#include "../include/geometry.h"

using namespace std;

int main(int argc, char const *argv[])
{
	pair<double, double> a = make_pair(0.0, 0.0);
	pair<double, double> b = make_pair(4.0, -9.0);

	pair<double, double> lineCoeff = geometry::findLineCoeff(a, b);
	cout << "slope: " << lineCoeff.first << " ,y-intercept: " << lineCoeff.second << endl;

	double distance = geometry::eucDist(a, b);
	cout << "euclidean distance: " << distance << endl;

	double dotP = geometry::dotProduct(a, b);
	cout << "dot product: " << dotP << endl;

	pair<double, double> stepXY = geometry::stepLinear(a, b, 1.0);
	cout << "at 1.0 units from a lies, x: " << stepXY.first <<
					" y:" << stepXY.second << endl;
	cout << "verify euclidean distance for the above: " << geometry::eucDist(a, stepXY) << endl;				


	bool result = geometry::lineSegmentIntersectCircle(2.0, 
							make_pair(0.0, 0.0),
							lineCoeff,
							a,
							b);
	cout << "does the linesegment collide? " << result << endl;

	return 0;
}

// test syntax:
// g++ test_geomtery.cpp -o test_geom