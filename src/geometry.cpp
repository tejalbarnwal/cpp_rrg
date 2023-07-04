#include "../include/geometry.h"

namespace geometry
{
	pair<double, double> findLineCoeff(pair<double, double> source,
								pair<double, double> dest)
	{
		double x1 = source.first;
		double y1 = source.second;
		double x2 = dest.first;
		double y2 = dest.second;

		double m = (y2 - y1) / (x2 - x1);
    	double b = y1 - m * x1;

    	return make_pair(m, b);
	}

	double eucDist(pair<double, double> source,
					pair<double, double> dest)
	{
		return sqrt(pow(source.first - dest.first, 2) + 
				pow(source.second - dest.second, 2));
	}

	double manhDist(pair<double, double> source,
					pair<double, double> dest)
	{
		return (abs(source.first - dest.first) + 
				abs(source.second - dest.second) );
	}

	double dotProduct(pair<double, double> a,
						pair<double, double> b)
	{
		return (a.first * b.first) + (a.second * b.second);
	}



	pair<double, double> stepLinear(pair<double, double> source,
								pair<double, double> dest, double stepSize)
	{
		double dx = dest.first - source.first;
		double dy = dest.second - source.second;

		double magnitude = sqrt(dx*dx + dy*dy);

		double x = (source.first) + (dx / magnitude) * stepSize;
		double y = (source.second) + (dy / magnitude) * stepSize;

    	return make_pair(x, y);

    	// cannot handle vertical lines!
	}

	bool lineSegmentIntersectCircle(double circleRadius, 
							pair<double, double> circleCenter,
							pair<double, double> lineCoeff,
							pair<double, double> source,
							pair<double, double> dest)
	{
		double h = circleCenter.first;
		double k = circleCenter.second;

		double m = lineCoeff.first;
		double c = lineCoeff.second;

		double x1 = source.first;
		double y1 = source.second;
		double x2 = dest.first;
		double y2 = dest.second;

		double centerSourceDist = eucDist(circleCenter, source);
		double centerDestDist = eucDist(circleCenter, dest);

		double minDist = min(centerSourceDist, centerDestDist);
		double maxDist = max(centerSourceDist, centerDestDist);

		// cout << "radius of circle: " << circleRadius << endl;
		// cout << "minDist: " << minDist << endl;
		// cout << "maxDist: " << maxDist << endl;

		if ((dotProduct(make_pair(h-x1, k-y1), make_pair(x2-x1, y2-y1)) > 0) &&
			(dotProduct(make_pair(h-x2, k-y2), make_pair(x1-x2, y1-y2)) > 0))
		{
			minDist = abs(m * h - k + c) / sqrt(pow(m, 2) + 1);
		}
		// cout << "updated minDist: " << minDist << endl;
		
		if (minDist <= circleRadius)
		{
			// cout << "intersection happened \n";
			return true;
		}
		else
		{
			// cout << "no intersection \n";
			return false;
		}
	}
}