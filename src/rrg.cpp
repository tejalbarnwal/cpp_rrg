#include "../include/rrg.h"
#include "../include/geometry.h"
// #include "../include/matplotlibcpp.h"

// namespace plt = matplotlibcpp;

RRG::RRG(double lowerBoundX_, double lowerBoundY_, 
		double upperBoundX_, double upperBoundY_,
		pair<double, double> start,
		vector<pair<double, double>> obstacleCenter_,
		vector<double> obstacleRadius_)
{
	lowerBoundX = lowerBoundX_;
	lowerBoundY = lowerBoundY_;
	upperBoundX = upperBoundX_;
	upperBoundY = upperBoundY_;

	v_init = start;

	obstacleCenter = obstacleCenter_;
	obstacleRadius = obstacleRadius_;

	cout << "\n---   Problem Initialization   ---\n";

	cout << "X bounds: lower bound= " << lowerBoundX << 
						"upper bound= " << upperBoundX << " " << endl;

	cout << "Y bounds: lower bound= " << lowerBoundY << 
						"upper bound= " << upperBoundY << " " << endl;

	cout << "start position: x= " << v_init.first << " y= " << v_init.second << endl;
}

void RRG::setParameters(unsigned int nIterations_, 
				   double stepSize_, 
				   double neighbourRadius_)
{
	nIterations = nIterations_;
	stepSize = stepSize_;
	neighbourRadius = neighbourRadius_;

	cout << "\n---   Parameters for RRG   ---\n";
	cout << "Iterations: " << nIterations << endl;
	cout << "stepSize: " << stepSize << endl;
	cout << "neighbourRadius: " << neighbourRadius << endl;
}


pair<double, double> RRG::createRandomPosition()
{
	double x = lowerBoundX + static_cast<float>(rand()) 
								* static_cast<float>(upperBoundX - lowerBoundX) / RAND_MAX;
	double y = lowerBoundY + static_cast<float>(rand()) 
								* static_cast<float>(upperBoundY - lowerBoundY) / RAND_MAX;
	
	// cout << "random x: " << x << endl;
	// cout << "random y: " << y << endl;
	return make_pair(x, y);
}


pair<Vertex, double> RRG::findNearestVertex(pair<double, double> posXY)
{
	unsigned int indexNearestVertex = 0;
	double distNearestVertex = 100.0;
	for (unsigned int i =0; i<rrg.vertices.size(); i++)
	{
		Vertex temp = rrg.vertices.at(i);
		double dist = geometry::eucDist(posXY, temp.posXY);
		if (dist < distNearestVertex)
		{
			distNearestVertex = dist;
			indexNearestVertex = i;
		}
	}
	// cout << "neasrest vertex is at index: " << indexNearestVertex << endl;
	// cout << "nearest vertex coord, x:" << rrg.vertices.at(indexNearestVertex).posXY.first <<
				// " , y: " << rrg.vertices.at(indexNearestVertex).posXY.second << endl;

	return make_pair(rrg.vertices.at(indexNearestVertex), distNearestVertex); 
}


Vertex RRG::steerVertex(Vertex x_nearest, pair<double, double> posXY, 
						unsigned int ID)
{
	pair<double, double> source = x_nearest.posXY;
	pair<double, double> dest = posXY;
	pair<double, double> newPosXY = geometry::stepLinear(source, dest, stepSize);

	// cout << "new verex formed at, x: " << newPosXY.first <<
					// ", y:" << newPosXY.second << endl;
	Vertex v = Vertex(ID, newPosXY.first, newPosXY.second);
	return v;
}

bool RRG::obstacleFree(Vertex x_nearest, Vertex x_new)
{
	// check if x_new is winthin the bounds of the workspace
	// if not then we can directly say that it collides
	unsigned int numObs = obstacleRadius.size();
	pair<double, double> lineCoeff = 
					geometry::findLineCoeff(x_nearest.posXY, x_new.posXY);

	for(unsigned int i=0; i< numObs; i++)
	{
		if (geometry::lineSegmentIntersectCircle(obstacleRadius.at(i), 
												obstacleCenter.at(i),
												lineCoeff,
												x_nearest.posXY,
												x_new.posXY))
		{
			return false;
		}
	}
	return true;
}

vector<Vertex> RRG::findNeighbour(Vertex x_new)
{
	vector<Vertex> neighbours;
	for (unsigned int i =0; i<rrg.vertices.size(); i++)
	{
		Vertex temp = rrg.vertices.at(i);
		double dist = geometry::eucDist(x_new.posXY, temp.posXY);
		if ((dist <= neighbourRadius) && dist > 0)
		{
			neighbours.push_back(temp);
		}
	}

	return neighbours;
}