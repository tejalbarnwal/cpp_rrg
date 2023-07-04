#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <iostream>
#include <utility>
#include <cmath>
#include <algorithm>

using namespace std;

namespace geometry
{
	pair<double, double> findLineCoeff(pair<double, double> source,
								pair<double, double> dest);

	double eucDist(pair<double, double> source,
					pair<double, double> dest);

	double manhDist(pair<double, double> source,
					pair<double, double> dest);

	double dotProduct(pair<double, double> a,
						pair<double, double> b);



	pair<double, double> stepLinear(pair<double, double> source,
								pair<double, double> dest, double stepSize);

	bool lineSegmentIntersectCircle(double circleRadius, 
							pair<double, double> circleCenter,
							pair<double, double> lineCoeff,
							pair<double, double> source,
							pair<double, double> dest);
}

#endif