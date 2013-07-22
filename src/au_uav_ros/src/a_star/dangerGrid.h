//
// airSpaceGrid.h
// Chris Tate
#ifndef DANGER_GRID
#define DANGER_GRID

#include <vector>
#include <map>
#include <iostream>
#include <math.h>

#include "DiscretizedPlane.h"
#include "map_tools.h"
#include "gridSquare.h"

#define TIME_STEPS 20

// The discretized air space of planes along with the danger rating of each square
// through time. This is for use in calculating the best cost with A*. Only needs
// plane data and takes care of the rest for you. Use this with BestCostGrid to 
// calculate the heuristic, then use BestCostGrid with A*.
class DangerGrid {
public:
	// constructs an DangerGrid from the map of DiscretizedPlaneCoordinators, calling all the necessary private
    // functions. This will construct the size of the airspace. TODO allow size to be dynamic
	DangerGrid(std::map<int, DiscretizedPlane> *planes, int planeID, int startTime, int endTime, map_tools::gridValues gridVals);

	// Updates the air space with the map of planes, calling all the necessary private funtions.
    // This will update the danger ratings to be used by the BestCostGrid
	bool update(std::map<int, DiscretizedPlane> *planes);

	// methods to adhere to best_cost_straight and now astar_sparse0 file TODO change
    double operator()( int x, int y, int time ) const;
	double get_pos(int x, int y, int time) const;
    int get_width_in_squares() const;
    int get_height_in_squares() const;
	
	// methods for the distance grid
	
	// Calculates the straight line distance from each grid to the
	// goal. This is the euclidian straight line distance. Not actual distance
	// but grid distance. So if you are only one square away then distance is 1
	void calculateDistances(int xg, int yg);
	int getXGoal() const;
	int getYGoal() const;
	int getTimeLength() const;

private:
	// This is the air space, represented in three dimensions, x, y, and time.
	// To get the GridSquare at x=8, y=9, and time=2, you would do: airSpace[8][9][2]
	std::vector< std::vector< std::vector<GridSquare> > > airSpace;

	// This is the 2D grid representing the straight line distance from each square
	// to the goal
	std::vector< std::vector<double> > distanceGrid;
	// x and y position on the grid where the goal is
	int xGoal;
	int yGoal;
	// planeID that the goals are for
	int planeID;
	
	// Grid variables
	// height and width in "units" (meters)
	double width;
	double height;
	// resolution in "units" (meters) size of square
	double resolution;
	// how many squares wide and high, width / resolution, height / resolution
	int squaresWide;
	int squaresHigh;

	// length of time represented
	int timeLength;

	void placeDangerRatings(std::map<int, DiscretizedPlane> *planes, int planeID, int startTime, int endTime);
	void addBuffer(int x, int y, int time, int planeID, int maxRecur);
	bool isXYInGrid(int x, int y);
};

DangerGrid::DangerGrid(std::map<int, DiscretizedPlane> *planes, int planeID, int startTime, int endTime, map_tools::gridValues gridVals) {
    //TODO calculate width and height based on planes list of waypoints, or could just use the passed in 
	// planes?
	double upperLeftLon = gridVals.upperLeftLongitude;
	double upperLeftLat = gridVals.upperLeftLatitude;
	double lonWidth = gridVals.longitudeWidth;
	double latWidth = gridVals.latitudeWidth;
	double res = gridVals.resolution;
    
	resolution = res;
	width = map_tools::calculate_distance_between_points(upperLeftLat, upperLeftLon, 
														upperLeftLat, upperLeftLon + lonWidth, "meters");
	height = map_tools::calculate_distance_between_points(upperLeftLat, upperLeftLon,
														upperLeftLat + latWidth, upperLeftLon, "meters");
	squaresWide = map_tools::find_width_in_squares(width, height, resolution);
	squaresHigh = map_tools::find_height_in_squares(width, height, resolution);

	// Increase endtime in case plane needs to go further out of its way. This can decrease and increase
	// as needed. In my view, it only increases the time and space needed to build this grid
	int extendedEndTime = endTime + 50;
	
	// set the time length represented
	timeLength = extendedEndTime - startTime;

	// Resize x 
	airSpace.resize(squaresWide);
	distanceGrid.resize(squaresWide);
	// Resize y
	for (int i = 0; i < airSpace.size(); i++) {
		airSpace[i].resize(squaresHigh);
		distanceGrid[i].resize(squaresHigh);
		// Resize time
		for (int j = 0; j < airSpace[i].size(); j++) {
			airSpace[i][j].resize(extendedEndTime - startTime);
		}
	}

	//TODO Call function to predict danger values
	placeDangerRatings(planes, planeID, startTime, extendedEndTime);
}

bool DangerGrid::isXYInGrid(int x, int y) {
	return x > 0 && x < width && y > 0 && y < height;
}

void DangerGrid::addBuffer(int x, int y, int time, int planeID, int maxRecur) {
	if (maxRecur < 1) {
		int xAround=x, yAround=y;
		for (int i = 0; i < 8; i++) {
			if (i == 0 && isXYInGrid(x+1, y)) {
				xAround = x+1;
				yAround = y;
				addBuffer(xAround, yAround, time, planeID, maxRecur + 1);
			}
			if (i == 1 && isXYInGrid(x+1, y+1)) {
				xAround = x+1;
				yAround = y+1;
				addBuffer(xAround, yAround, time, planeID, maxRecur + 1);
			}
			if (i == 2 && isXYInGrid(x, y+1)) {
				xAround = x;
				yAround = y+1;
				addBuffer(xAround, yAround, time, planeID, maxRecur + 1);
			}
			if (i == 3 && isXYInGrid(x-1, y+1)) {
				xAround = x-1;
				yAround = y+1;
				addBuffer(xAround, yAround, time, planeID, maxRecur + 1);
			}
			if (i == 4 && isXYInGrid(x-1, y)) {
				xAround = x-1;
				yAround = y;
				addBuffer(xAround, yAround, time, planeID, maxRecur + 1);
			}
			if (i == 5 && isXYInGrid(x-1, y-1)) {
				xAround = x-1;
				yAround = y-1;
				addBuffer(xAround, yAround, time, planeID, maxRecur + 1);
			}
			if (i == 6 && isXYInGrid(x, y-1)) {
				xAround = x;
				yAround = y-1;
				addBuffer(xAround, yAround, time, planeID, maxRecur + 1);
			}
			if (i == 7 && isXYInGrid(x+1, y-1)) {
				xAround = x+1;
				yAround = y-1;
				addBuffer(xAround, yAround, time, planeID, maxRecur + 1);
			}
			GridSquare *square = &airSpace[xAround][yAround][time];
			DangerDescriptor dan;
			dan.planeID = planeID;
			dan.danger = 15;
			square->addDanger(dan);
		}
	}
}

void DangerGrid::placeDangerRatings(std::map<int, DiscretizedPlane> *planes, int planeID, int startTime, int endTime) {
	//TODO might want to add a buffer or have multiple ways of placing danger to test
	for (std::map<int, DiscretizedPlane>::iterator it = planes->begin(); it != planes->end(); it++) {
		if (it->first != planeID) {
			std::vector<Position> *locations = it->second.getLocationsThroughTime();
			int j = 0;
			for (int i = startTime; i < locations->size() && i < endTime && j < endTime - startTime; i++) {
				//std::cout << (*locations)[i].getX() << " " << (*locations)[i].getY() << " " << i << "\n";
				// Place the danger rating relative to this danger grid's plane, not the plane.
				// So here we use current to convert the latitude and longitude to the proper x, y
				// locations in this plane's grid. Basically, each plane has its own grid that it is
				// centered on so don't trust the x, y of the plane, trust the lat, lon
				Position temp = (*planes)[planeID].getLocation();
				temp.setLatLon((*locations)[i].getLat(), (*locations)[i].getLon());
				GridSquare *square = &airSpace[temp.getX()][temp.getY()][j];
				DangerDescriptor dan;
				dan.planeID = it->first;
				dan.danger = 30;
				square->addDanger(dan);
				addBuffer((*locations)[i].getX(), (*locations)[i].getY(), j, it->first, 0);
				j++;
			}
		}
	}
}

int DangerGrid::getTimeLength() const {
	return timeLength;
}

// same as get_pos
double DangerGrid::operator()( int x, int y, int time ) const
{
    return airSpace[x][y][time].getTotalDanger() + distanceGrid[x][y];
}

// TODO FIX name. right now its just to match with astar_sparse0 and take best_cost_straight out of the mix
// TODO fix the heuristic generation to multiply by a scale. need to understand that first though.
double DangerGrid::get_pos(int x, int y, int time) const {
//	if (x > 15 && x < 35 && y > 15 && y < 35) {
//		return 100000;
//	}
	//return airSpace[x][y][time].getTotalDanger() + distanceGrid[x][y];
	//TODO fix this time + 1 hack to make A* line up with the right danger grids
	return airSpace[x][y][time].getTotalDanger() + distanceGrid[x][y];
}

int DangerGrid::get_width_in_squares() const
{
    return squaresWide;
}

int DangerGrid::get_height_in_squares() const
{
    return squaresHigh;
}

void DangerGrid::calculateDistances(int xg, int yg) {
	xGoal = xg;
	yGoal = yg;
	for (int xNow = 0; xNow < squaresWide; xNow++) {
		for (int yNow = 0; yNow < squaresHigh; yNow++) {
			distanceGrid[xNow][yNow] = 0.5 * sqrt(pow(xNow - xGoal, 2) + pow(yNow - yGoal, 2));
		}
	}
}

int DangerGrid::getXGoal() const {
	return xGoal;
}

int DangerGrid::getYGoal() const {
	return yGoal;
}

#endif
