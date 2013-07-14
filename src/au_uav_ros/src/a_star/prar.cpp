// Combination of A* and RIPNA

#include "astar_sparse0.cpp"
#include "au_uav_ros/standardDefs.h"
#include "au_uav_ros/planeObject.h"
#include "DiscretizedPlane.h"
#include "dangerGrid.h"
#include "Position.h"
#include "map_tools.h"

#include <vector>
#include <map>
#include <queue>
#include <iostream>
#include <fstream>
#include <math.h>

// TODO pass gridVals to Position instead of using their constructors still
map_tools::gridValues gridVals;

// PROTOTYPES
std::map<int, DiscretizedPlane> convertAllPlanesAndAddWaypoints(std::map<int, au_uav_ros::PlaneObject> *planes, std::map<int, std::vector<au_uav_ros::waypoint> > *allPlanesWaypoints);
std::vector<waypointPath> astarPathPlan(std::map<int, DiscretizedPlane> *discretePlanes, int planeID);
void generateMapValuesFromWaypoints(map_tools::gridValues &gridValsOut, std::map<int, std::vector<au_uav_ros::waypoint> > *allPlanesWaypoints);
std::vector<waypointPath> astarPathPlanEachLeg(std::map<int, DiscretizedPlane> *discretePlanes, int planeID);
waypointPath fitWaypointsToFullPath(std::queue<point> fullPath, double startLat, double startLon, int currentWaypointIndex);
waypointPath astarPlaneToDestination(std::map<int, DiscretizedPlane> *discretePlanes, int planeID);
waypointPath findLocalAstarPath(std::map<int, DiscretizedPlane> *discretePlanes, int planeID, int currentWaypointIndex, int lastWaypointTime, int collisionTime);


/* path planning to next destination only methods
 */

// This method is to be called by collision avoidance and will give you a collision free A* path to the next destination only
std::map<int, map_tools::waypointPath> getAStarPath(std::map<int, au_uav_ros::PlaneObject> *planes, std::map<int, std::vector<au_uav_ros::waypoint> > *allPlanesWaypoints) {
	generateMapValuesFromWaypoints(gridVals, allPlanesWaypoints);
	std::map<int, DiscretizedPlane> discretePlanes = convertAllPlanesAndAddWaypoints(planes, allPlanesWaypoints);
	std::map<int, map_tools::waypointPath> allPlanesPaths;

	for (int i = 0; i < discretePlanes.size(); i++) {
		DiscretizedPlane *dPlane = &discretePlanes[i];
		std::cout << "Moving plane: " << i << " through time\n";
		dPlane->moveThroughTime(dPlane->getBearing(), false);
		std::cout << "Finished with plane: " << i << "\n";
	}
	for (int i = 0; i < discretePlanes.size(); i++) {
		std::cout << "Planning Collision path for plane: " << i << "\n";
		allPlanesPaths[i] = astarPlaneToDestination(&discretePlanes, i);
		std::cout << "Finished planning for plane: " << i << "\n";
	}

	return allPlanesPaths;
}

// This plans the path for a single plane to the next destination
waypointPath astarPlaneToDestination(std::map<int, DiscretizedPlane> *discretePlanes, int planeID) {
	int startx = (*discretePlanes)[planeID].getLocation().getX();
	int starty = (*discretePlanes)[planeID].getLocation().getY();
	int endx = (*discretePlanes)[planeID].getDestination().getX();
	int endy = (*discretePlanes)[planeID].getDestination().getY();
	map_tools::bearing_t bearingNamed = map_tools::name_bearing((*discretePlanes)[planeID].getBearing());
	//TODO hardcode time values. Telling it to generate DangerGrid for planes between 0 and 50 seconds. Then danger grid adds another 50 seconds so we have 100 seconds
	// worth of danger grids for all the plane's from. This should hopefully be enough. Could decrease if speed matters, and all the planes never take longer than 100
	// seconds to reach their destinations
	DangerGrid bc = DangerGrid(discretePlanes, planeID, 0, 50, gridVals);
	std::queue<point> fullPath = astar_point(&bc, startx, starty, endx, endy, planeID, bearingNamed, discretePlanes, gridVals);

	// generates full waypoint path from A* queue
	waypointPath legPath;
	legPath.startWaypointIndex = -1;
	Position pos = Position(gridVals, 0, 0);
	while (!fullPath.empty()) {
		point nextPoint = fullPath.front();
		pos.setXY(nextPoint.x, nextPoint.y);
		au_uav_ros::waypoint nextWay;
		nextWay.latitude = pos.getLat();
		nextWay.longitude = pos.getLon();
		legPath.pathWaypoints.push_back(nextWay);
		fullPath.pop();
	}

	(*discretePlanes)[planeID].addAvoidancePath(legPath);
	(*discretePlanes)[planeID].moveThroughTime((*discretePlanes)[planeID].getBearing(), false);
	return legPath;
}


/* Methods for Astar path planning from the beginning of simulation to the end
 */

// The function for collision avoidance to call. returns a map of all the paths that it planned for each plane
std::map<int, std::vector<waypointPath> > * getAvoidancePlan(std::map< int, au_uav_ros::PlaneObject> *planes, int planeID, std::map<int, std::vector<au_uav_ros::waypoint> > *allPlanesWaypoints) {
	// initialization activites
	generateMapValuesFromWaypoints(gridVals, allPlanesWaypoints);
	std::map<int, DiscretizedPlane> discretePlanes = convertAllPlanesAndAddWaypoints(planes, allPlanesWaypoints);
	std::map<int, std::vector<waypointPath> > *allPlanesPaths = new std::map<int, std::vector<waypointPath> >();
	
	for (int i = 0; i < discretePlanes.size(); i++) {
		DiscretizedPlane *dPlane = &discretePlanes[i];
		std::cout << "Moving plane: " << i << " through time\n";
		dPlane->moveThroughTime(0, false);
		std::cout << "Finished with plane: " << i << "\n";
	}
	for (int i = 0; i < discretePlanes.size(); i++) {
		std::cout << "Planning Collision path for plane: " << i << "\n";
		(*allPlanesPaths)[i] = astarPathPlanEachLeg(&discretePlanes, i);
		std::cout << "Finished planning for plane: " << i << "\n";
	}
	

	return allPlanesPaths;
}

// Completely plans the paths from start of mission to the end but only plans a path between two waypoints 
// if a collision is found between those two waypoints
std::vector<waypointPath> astarPathPlan(std::map<int, DiscretizedPlane> *discretePlanes, int planeID) {
	std::vector<Position> *locations = (*discretePlanes)[planeID].getLocationsThroughTime();
	int lastWaypointTime = 0;
	// the first waypoint is 0 and the start position can be thought of as waypoint -1
	int numWaypoints = -1;
	std::vector<waypointPath> pathsForPlane;
	for (int i = 0; i < locations->size(); i++) {
		// TODO what if waypoint is where collision occurs! This needs fixing
		if ((*locations)[i].getIsWaypoint()) {
			lastWaypointTime = i;
			numWaypoints++;
		}

		for (std::map<int, DiscretizedPlane>::iterator it = discretePlanes->begin(); it != discretePlanes->end(); it++) {
			if (it->first != planeID) {
				std::vector<Position> *otherPlaneLocs = it->second.getLocationsThroughTime();
				if (i < (*otherPlaneLocs).size() && map_tools::calculate_distance_between_points(
							(*otherPlaneLocs)[i].getLat(), (*otherPlaneLocs)[i].getLon(), 
							(*locations)[i].getLat(), (*locations)[i].getLon(), "meters") < 12)
				{
					std::cout << "Actually found a collision at time: " << i << "\n";
					waypointPath cFreePath = findLocalAstarPath(discretePlanes, planeID, numWaypoints, lastWaypointTime, i);
					(*discretePlanes)[planeID].addAvoidancePath(cFreePath);
					(*discretePlanes)[planeID].moveThroughTime(0, false);
					pathsForPlane.push_back(cFreePath);
					locations = (*discretePlanes)[planeID].getLocationsThroughTime();
					// get to the next waypoint time
					for (int j = i + 1; j < locations->size(); j++) {
						if ((*locations)[i].getIsWaypoint()) {
							i = j;
							break;
						}
					}
				}
			}
		}
	}
	return pathsForPlane;
}

waypointPath findLocalAstarPath(std::map<int, DiscretizedPlane> *discretePlanes, int planeID, int currentWaypointIndex, int lastWaypointTime, int collisionTime) {
	// get initial things for A* TODO could change this awful parameter list
	std::vector<Position> *locations = (*discretePlanes)[planeID].getLocationsThroughTime();
	int startx = (*locations)[lastWaypointTime].getX();
	int starty = (*locations)[lastWaypointTime].getY();
	int endx, endy, endIndex = lastWaypointTime;
	// find the next waypoint from the collision time onward
	for (int i = lastWaypointTime + 1; i < locations->size(); i++) {
		endIndex = i;
		if((*locations)[i].getIsWaypoint()) {
			break;
		}
	}
	endx = (*locations)[endIndex].getX();
	endy = (*locations)[endIndex].getY();
	map_tools::bearing_t bearingNamed = map_tools::name_bearing((*locations)[lastWaypointTime].getBearing());
	DangerGrid bc = DangerGrid(discretePlanes, planeID, lastWaypointTime, endIndex, gridVals);
	// Finally actually call the A* code to find the path for this small leg of the full path
	std::queue<point> fullPath = astar_point(&bc, startx, starty, endx, endy, planeID, bearingNamed, discretePlanes, gridVals);
	
	// this sections will return the full path, below will return a shortened path
	waypointPath legPath;
	legPath.startWaypointIndex = currentWaypointIndex;
	Position pos = Position(gridVals, 0, 0);
	while (!fullPath.empty()) {
		point nextPoint = fullPath.front();
		pos.setXY(nextPoint.x, nextPoint.y);
		au_uav_ros::waypoint nextWay;
		nextWay.latitude = pos.getLat();
		nextWay.longitude = pos.getLon();
		legPath.pathWaypoints.push_back(nextWay);
		fullPath.pop();
	}
	return legPath;
	//return fitWaypointsToFullPath(fullPath, (*locations)[lastWaypointTime].getLat(), (*locations)[lastWaypointTime].getLon(), currentWaypointIndex);
}

// Can be called instead of astarPathPlan to plan each leg, even if there isn't a collision between the two waypoints
std::vector<waypointPath> astarPathPlanEachLeg(std::map<int, DiscretizedPlane> *discretePlanes, int planeID) {
	std::vector<Position> *locations = (*discretePlanes)[planeID].getLocationsThroughTime();
	int lastWaypointTime = 0;
	// the first waypoint is 0 and the start position can be thought of as waypoint -1
	int numWaypoints = -1;
	std::vector<waypointPath> pathsForPlane;
	
	// path between start and first waypoint
	std::cout << "Planning path between waypoint " << -1 << "and " << 0 << "\n";
	waypointPath cFreePath = findLocalAstarPath(discretePlanes, planeID, numWaypoints, lastWaypointTime, 1);
	(*discretePlanes)[planeID].addAvoidancePath(cFreePath);
	(*discretePlanes)[planeID].moveThroughTime(0, false);
	pathsForPlane.push_back(cFreePath);
	locations = (*discretePlanes)[planeID].getLocationsThroughTime();

	for (int i = 0; i < locations->size(); i++) {
		// TODO what if waypoint is where collision occurs! This needs fixing
		if ((*locations)[i].getIsWaypoint()) {
			lastWaypointTime = i;
			numWaypoints++;
			std::cout << "Planning path between waypoint " << numWaypoints << "and " << numWaypoints + 1 << "\n";
			waypointPath cFreePath = findLocalAstarPath(discretePlanes, planeID, numWaypoints, lastWaypointTime, i);
			(*discretePlanes)[planeID].addAvoidancePath(cFreePath);
			(*discretePlanes)[planeID].moveThroughTime(0, false);
			pathsForPlane.push_back(cFreePath);
			locations = (*discretePlanes)[planeID].getLocationsThroughTime();
		}

	}
	return pathsForPlane;
}

/*
 * Shared functions between full path planning and to destination path planning
 */

// Shrinks a long path to fit a path that is close to the full path with less waypoints
waypointPath fitWaypointsToFullPath(std::queue<point> fullPath, double startLat, double startLon, int currentWaypointIndex) {
	waypointPath legPath;
	legPath.startWaypointIndex = currentWaypointIndex;

	double previousLat = startLat, previousLon = startLon, bearingBetween = 0;
	point nextPoint;
	Position nextPointPosition, previousPosition;
	nextPointPosition = Position(gridVals, 0, 0);

	// Pop off the first one since it is directly in front of plane. THIS SHOULD CHANGE IF THAT IS CHANGED.
	fullPath.pop();
	if (!fullPath.empty()) {
		nextPoint = fullPath.front();
		fullPath.pop();
	}
	previousPosition = Position(gridVals, nextPoint.x, nextPoint.y);
	bearingBetween = map_tools::calculateBearing(startLat, startLon, previousPosition.getLat(), previousPosition.getLon());
	// TODO THIS NEEDS TO BE FIXED CAN'T but some sort of segfault is happening 
	while (!fullPath.empty() && currentWaypointIndex < 20) {
		point nextPoint = fullPath.front();
		cout << nextPoint.x << " " << nextPoint.y << "\n";
		nextPointPosition.setXY(nextPoint.x, nextPoint.y);
		int nextBearingBetween = map_tools::calculateBearing(previousLat, previousLon, nextPointPosition.getLat(), nextPointPosition.getLon());
		if (fabs(nextBearingBetween - bearingBetween) > 5) {
			au_uav_ros::waypoint nextWay;
			nextWay.latitude = previousPosition.getLat();
			nextWay.longitude = previousPosition.getLon();
			legPath.pathWaypoints.push_back(nextWay);
			previousLat = previousPosition.getLat();
			previousLon = previousPosition.getLon();
			bearingBetween = map_tools::calculateBearing(previousLat, previousLon, nextPointPosition.getLat(), nextPointPosition.getLon());
		}
		previousPosition = nextPointPosition;
		fullPath.pop();	
	}
	return legPath;
}

// Sets the gridVals global variable to the size of the grid we want, based on all the plane's waypoints
void generateMapValuesFromWaypoints(map_tools::gridValues &gridValsOut, std::map<int, std::vector<au_uav_ros::waypoint> > *allPlanesWaypoints) {
	// gridValsOut is changing the global variable gridVals. Just don't want to hard code that if we move away from global
	double upperLeftLat = -1000;
	double upperLeftLon = 1000;
	double bottomLeftLat = 1000;
	double upperRightLon = -1000;
	for (std::map<int, std::vector<au_uav_ros::waypoint> >::iterator it =allPlanesWaypoints->begin(); it != allPlanesWaypoints->end(); it++) {
		//it->second is the vector of that plane's waypoints
		for (int i = 0; i < it->second.size(); i++) {
			if (it->second[i].latitude > upperLeftLat) {
				upperLeftLat = it->second[i].latitude;
			}
			if (it->second[i].longitude < upperLeftLon) {
				upperLeftLon = it->second[i].longitude;
			}
			if (it->second[i].latitude < bottomLeftLat) {
				bottomLeftLat = it->second[i].latitude;
			}
			if (it->second[i].longitude > upperRightLon) {
				upperRightLon = it->second[i].longitude;
			}
		}
	}
	// this is where you can change the padding and resolution on the fly if you wanted
	gridValsOut.resolution = DEFAULT_RESOLUTION;
	gridValsOut.padding = DEFAULT_PADDING;

	// apply the padding now, you can get the original values by the appropriate math
	gridValsOut.upperLeftLatitude = upperLeftLat + gridValsOut.padding;
	gridValsOut.upperLeftLongitude = upperLeftLon - gridValsOut.padding;
	gridValsOut.latitudeWidth = (upperLeftLat - bottomLeftLat) +  2 * gridValsOut.padding;
	gridValsOut.longitudeWidth = (upperRightLon - upperLeftLon) + 2 * gridValsOut.padding;


	std::cout << gridVals.upperLeftLatitude << " " << gridVals.upperLeftLongitude << " " << gridVals.latitudeWidth << " " << gridVals.longitudeWidth << "\n";
}


/* Methods to convert between PlaneObject map to DiscritizedPlane map
 */

DiscretizedPlane convertRealPlane(au_uav_ros::PlaneObject continuousPlane, std::vector<au_uav_ros::waypoint> allWaypoints) {
	Position current = Position(gridVals, continuousPlane.getCurrentLoc().latitude, continuousPlane.getCurrentLoc().longitude);
	Position goal = Position(gridVals, continuousPlane.getDestination().latitude, continuousPlane.getDestination().longitude);
	DiscretizedPlane dPlane(continuousPlane.getID(), current, goal);
	dPlane.setBearing(continuousPlane.getCurrentBearing());

	// Change point to something better, like waypoint, just try to not tie the code to au_uav_ros too much but should in the end anyways
	vector<Position> convertedWaypoints;
	for (int i = 0; i < allWaypoints.size(); i++) {
		Position current = Position(gridVals, allWaypoints[i].latitude, allWaypoints[i].longitude);
		convertedWaypoints.push_back(current);
	}
	dPlane.setAllWaypoints(convertedWaypoints);

	return dPlane; 
}

std::map<int, DiscretizedPlane> convertAllPlanesAndAddWaypoints(std::map<int, au_uav_ros::PlaneObject> *planes, std::map<int, std::vector<au_uav_ros::waypoint> > *allPlanesWaypoints) {
	std::map<int, DiscretizedPlane> newPlanes;
	for (std::map<int, au_uav_ros::PlaneObject>::iterator it =planes->begin(); it != planes->end(); it++) {
		newPlanes[it->first] = convertRealPlane(it->second, (*allPlanesWaypoints)[it->second.getID()]);
	}
	return newPlanes;
}
