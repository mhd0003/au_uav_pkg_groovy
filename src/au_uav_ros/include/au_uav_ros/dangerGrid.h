//
// airSpaceGrid.h
// Chris Tate
#ifndef _DANGER_GRID_H_
#define _DANGER_GRID_H_

#include "au_uav_ros/discretizedPlane.h"
#include "au_uav_ros/mapTools.h"
#include "au_uav_ros/gridSquare.h"

#define TIME_STEPS 20

// The discretized air space of planes along with the danger rating of each square
// through time. This is for use in calculating the best cost with A*. Only needs
// plane data and takes care of the rest for you. Use this with BestCostGrid to 
// calculate the heuristic, then use BestCostGrid with A*.
namespace au_uav_ros {
	class DangerGrid {
	public:
		// constructs an DangerGrid from the map of DiscretizedPlaneCoordinators, calling all the necessary private
	    // functions. This will construct the size of the airspace. TODO allow size to be dynamic
		DangerGrid(std::map<int, DiscretizedPlane> &planes, int planeID, int startTime, int endTime, map_tools::gridValues gridVals);

		// Updates the air space with the map of planes, calling all the necessary private funtions.
	    // This will update the danger ratings to be used by the BestCostGrid
		bool update(std::map<int, DiscretizedPlane> &planes);

		// methods to adhere to best_cost_straight and now astar_sparse0 file TODO change
	    double operator()( unsigned int x, unsigned int y, int time ) const;
		double get_pos(unsigned int x, unsigned int y, int time) const;
	    unsigned int get_width_in_squares() const;
	    unsigned int get_height_in_squares() const;
		
		// methods for the distance grid
		
		// Calculates the straight line distance from each grid to the
		// goal. This is the euclidian straight line distance. Not actual distance
		// but grid distance. So if you are only one square away then distance is 1
		void calculateDistances(unsigned int xg, unsigned int yg);
		unsigned int getXGoal() const;
		unsigned int getYGoal() const;

	private:
		// This is the air space, represented in three dimensions, x, y, and time.
		// To get the GridSquare at x=8, y=9, and time=2, you would do: airSpace[8][9][2]
		std::vector< std::vector< std::vector<GridSquare> > > airSpace;

		// This is the 2D grid representing the straight line distance from each square
		// to the goal
		std::vector< std::vector<double> > distanceGrid;
		// x and y position on the grid where the goal is
		unsigned int xGoal;
		unsigned int yGoal;
		// planeID that the goals are for
		unsigned int planeID;
		
		// Grid variables
		// height and width in "units" (meters)
		double width;
		double height;
		// resolution in "units" (meters) size of square
		double resolution;
		// how many squares wide and high, width / resolution, height / resolution
		unsigned int squaresWide;
		unsigned int squaresHigh;

		void placeDangerRatings(std::map<int, DiscretizedPlane> &planes, int planeID, int startTime, int endTime);
	};
};

// DangerGrid::DangerGrid(std::map<int, DiscretizedPlane> *planes, int planeID, int startTime, int endTime, map_tools::gridValues gridVals) {
//     //TODO calculate width and height based on planes list of waypoints, or could just use the passed in 
// 	// planes?
// 	double upperLeftLon = gridVals.upperLeftLongitude;
// 	double upperLeftLat = gridVals.upperLeftLatitude;
// 	double lonWidth = gridVals.longitudeWidth;
// 	double latWidth = gridVals.latitudeWidth;
// 	double res = gridVals.resolution;
    
// 	resolution = res;
// 	width = map_tools::calculate_distance_between_points(upperLeftLat, upperLeftLon, 
// 														upperLeftLat, upperLeftLon + lonWidth, "meters");
// 	height = map_tools::calculate_distance_between_points(upperLeftLat, upperLeftLon,
// 														upperLeftLat + latWidth, upperLeftLon, "meters");
// 	squaresWide = map_tools::find_width_in_squares(width, height, resolution);
// 	squaresHigh = map_tools::find_height_in_squares(width, height, resolution);
// 	std::cout << "\n\n\n" << squaresWide << " " << squaresHigh << "\n\n\n";

// 	// Increase endtime in case plane needs to go further out of its way. This can decrease and increase
// 	// as needed. In my view, it only increases the time and space needed to build this grid
// 	int extendedEndTime = endTime + 50;

// 	// Resize x 
// 	airSpace.resize(squaresWide);
// 	distanceGrid.resize(squaresWide);
// 	// Resize y
// 	for (unsigned int i = 0; i < airSpace.size(); i++) {
// 		airSpace[i].resize(squaresHigh);
// 		distanceGrid[i].resize(squaresHigh);
// 		// Resize time
// 		for (unsigned int j = 0; j < airSpace[i].size(); j++) {
// 			airSpace[i][j].resize(extendedEndTime - startTime);
// 		}
// 	}

// 	//TODO Call function to predict danger values
// 	placeDangerRatings(planes, planeID, startTime, extendedEndTime);
// }

// void DangerGrid::placeDangerRatings(std::map<int, DiscretizedPlane> *planes, int planeID, int startTime, int endTime) {
// 	//TODO might want to add a buffer or have multiple ways of placing danger to test
// 	for (std::map<int, DiscretizedPlane>::iterator it = planes->begin(); it != planes->end(); it++) {
// 		if (it->first != planeID) {
// 			std::vector<Position> *locations = it->second.getLocationsThroughTime();
// 			int j = 0;
// 			for (int i = startTime; i < locations->size() && i < endTime && j < endTime - startTime; i++) {
// 				//std::cout << (*locations)[i].getX() << " " << (*locations)[i].getY() << " " << i << "\n";
// 				GridSquare *square = &airSpace[(*locations)[i].getX()][(*locations)[i].getY()][j];
// 				DangerDescriptor dan;
// 				dan.planeID = it->first;
// 				dan.danger = 1;
// 				square->addDanger(dan);
// 				j++;
// 			}
// 		}
// 	}
// }

// // same as get_pos
// double DangerGrid::operator()( unsigned int x, unsigned int y, int time ) const
// {
//     return airSpace[x][y][time].getTotalDanger() + distanceGrid[x][y];
// }

// // TODO FIX name. right now its just to match with astar_sparse0 and take best_cost_straight out of the mix
// // TODO fix the heuristic generation to multiply by a scale. need to understand that first though.
// double DangerGrid::get_pos(unsigned int x, unsigned int y, int time) const {
// //	if (x > 15 && x < 35 && y > 15 && y < 35) {
// //		return 100000;
// //	}
// 	return airSpace[x][y][time].getTotalDanger() + distanceGrid[x][y];
// }

// unsigned int DangerGrid::get_width_in_squares() const
// {
//     return squaresWide;
// }

// unsigned int DangerGrid::get_height_in_squares() const
// {
//     return squaresHigh;
// }

// void DangerGrid::calculateDistances(unsigned int xg, unsigned int yg) {
// 	xGoal = xg;
// 	yGoal = yg;
// 	for (unsigned int xNow = 0; xNow < squaresWide; xNow++) {
// 		for (unsigned int yNow = 0; yNow < squaresHigh; yNow++) {
// 			distanceGrid[xNow][yNow] = 0.5 * sqrt(pow(xNow - xGoal, 2) + pow(yNow - yGoal, 2));
// 		}
// 	}
// }

// unsigned int DangerGrid::getXGoal() const {
// 	return xGoal;
// }

// unsigned int DangerGrid::getYGoal() const {
// 	return yGoal;
// }

#endif
