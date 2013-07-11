#include "au_uav_ros/dangerGrid.h"
using namespace au_uav_ros;

DangerGrid::DangerGrid(std::map<int, DiscretizedPlane> &planes, int planeID, int startTime, int endTime, const map_tools::gridValues &gridVals) {
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
	std::cout << "\n\n\n" << squaresWide << " " << squaresHigh << "\n\n\n";

	// Increase endtime in case plane needs to go further out of its way. This can decrease and increase
	// as needed. In my view, it only increases the time and space needed to build this grid
	int extendedEndTime = endTime + 50;

	// Resize x 
	airSpace.resize(squaresWide);
	distanceGrid.resize(squaresWide);
	// Resize y
	for (unsigned int i = 0; i < airSpace.size(); i++) {
		airSpace[i].resize(squaresHigh);
		distanceGrid[i].resize(squaresHigh);
		// Resize time
		for (unsigned int j = 0; j < airSpace[i].size(); j++) {
			airSpace[i][j].resize(extendedEndTime - startTime);
		}
	}

	//TODO Call function to predict danger values
	placeDangerRatings(planes, planeID, startTime, extendedEndTime);
}

void DangerGrid::placeDangerRatings(std::map<int, DiscretizedPlane> &planes, int planeID, int startTime, int endTime) {
	//TODO might want to add a buffer or have multiple ways of placing danger to test
	for (std::map<int, DiscretizedPlane>::iterator it = planes->begin(); it != planes->end(); it++) {
		if (it->first != planeID) {
			std::vector<Position> locations = it->second.getLocationsThroughTime();
			int j = 0;
			for (int i = startTime; i < locations->size() && i < endTime && j < endTime - startTime; i++) {
				//std::cout << (locations)[i].getX() << " " << (locations)[i].getY() << " " << i << "\n";
				GridSquare *square = &airSpace[(locations)[i].getX()][(locations)[i].getY()][j];
				DangerDescriptor dan;
				dan.planeID = it->first;
				dan.danger = 1;
				square->addDanger(dan);
				j++;
			}
		}
	}
}

// same as get_pos
double DangerGrid::operator()( unsigned int x, unsigned int y, int time ) const
{
    return airSpace[x][y][time].getTotalDanger() + distanceGrid[x][y];
}

// TODO FIX name. right now its just to match with astar_sparse0 and take best_cost_straight out of the mix
// TODO fix the heuristic generation to multiply by a scale. need to understand that first though.
double DangerGrid::get_pos(unsigned int x, unsigned int y, int time) const {
//	if (x > 15 && x < 35 && y > 15 && y < 35) {
//		return 100000;
//	}
	return airSpace[x][y][time].getTotalDanger() + distanceGrid[x][y];
}

unsigned int DangerGrid::get_width_in_squares() const
{
    return squaresWide;
}

unsigned int DangerGrid::get_height_in_squares() const
{
    return squaresHigh;
}

void DangerGrid::calculateDistances(unsigned int xg, unsigned int yg) {
	xGoal = xg;
	yGoal = yg;
	for (unsigned int xNow = 0; xNow < squaresWide; xNow++) {
		for (unsigned int yNow = 0; yNow < squaresHigh; yNow++) {
			distanceGrid[xNow][yNow] = 0.5 * sqrt(pow(xNow - xGoal, 2) + pow(yNow - yGoal, 2));
		}
	}
}

unsigned int DangerGrid::getXGoal() const {
	return xGoal;
}

unsigned int DangerGrid::getYGoal() const {
	return yGoal;
}