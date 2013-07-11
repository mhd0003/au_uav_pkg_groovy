// Combination of A* and RIPNA
#include "au_uav_ros/prar.h"
using namespace au_uav_ros;

// TODO pass gridVals to Position instead of using their constructors still
map_tools::gridValues gridVals;

//TODO change back to waypoint not point will have to convert from point to waypoint, not hard at all
prar::waypointVector prar::getAvoidancePlan(int planeID, PlaneObjectMap &allPlanes, WaypointVectorMap &allPlanesWaypoints) {
	// initialization activites
	generateMapValuesFromWaypoints(gridVals, allPlanesWaypoints);
	DiscretePlaneMap discretePlanes;
	convertAllPlanesAndAddWaypoints(allPlanes, discretePlanes, allPlanesWaypoints);
	
	// TODO figure out how to return all the collision avoidance waypoints back
	waypointVector way;

	for (int i = 0; i < discretePlanes.size(); i++) {
		std::cout << "Moving plane: " << i << " through time\n";
		discretePlanes[i].moveThroughTime(0);
		std::cout << "Finished with plane: " << i << "\n";
	}
	for (int i = 0; i < discretePlanes.size(); i++) {
		std::cout << "Planning Collision path for plane: " << i << "\n";
		astarPathPlan(discretePlanes, i);
		std::cout << "Finished planning for plane: " << i << "\n";
	}

	return way;
}


// Methods for Astar path planning
std::queue<point> prar::findLocalAstarPath(DiscretePlaneMap &discretePlanes, int planeID, int lastWaypointTime, int collisionTime) {
	// get initial things for A* TODO could change this awful parameter list
	std::vector<Position> *locations = discretePlanes[planeID].getLocationsThroughTime();
	int startx = locations->at(lastWaypointTime).getX();
	int starty = locations->at(lastWaypointTime).getY();
	int endx, endy, endIndex = lastWaypointTime;
	// find the next waypoint from the collision time onward
	ROS_INFO_STREAM(locations->size());
	for (int i = lastWaypointTime + 1; i < locations->size(); i++) {
		endIndex = i;
		if(locations->at(i).getIsWaypoint()) {
			break;
		}
	}
	endx = locations->at(endIndex).getX();
	endy = locations->at(endIndex).getY();
	bearing_t bearingNamed = map_tools::name_bearing(locations->at(lastWaypointTime).getBearing());
	ROS_INFO_STREAM(endIndex);
	DangerGrid bc = DangerGrid(discretePlanes, planeID, lastWaypointTime, endIndex, gridVals);
	// Finally actually call the A* code to find the path for this small leg of the full path
	std::queue<point> fullPath = astar_point(bc, startx, starty, endx, endy, planeID, bearingNamed, discretePlanes, gridVals);
	return fullPath;
}

void prar::astarPathPlan(DiscretePlaneMap &discretePlanes, int planeID) {
	std::vector<Position> *locations = discretePlanes[planeID].getLocationsThroughTime();
	int lastWaypointTime = 0;
	for (int i = 0; i < locations->size(); i++) {
		// TODO what if waypoint is where collision occurs! This needs fixing
		if (locations->at(i).getIsWaypoint()) {
			lastWaypointTime = i;
		}

		DiscretePlaneMap::iterator it;
		for (it = discretePlanes.begin(); it != discretePlanes.end(); ++it) {
			if (it->first != planeID) {
				std::vector<Position> *otherPlaneLocs = it->second.getLocationsThroughTime();
				if (i < otherPlaneLocs->size() && map_tools::calculate_distance_between_points(
							otherPlaneLocs->at(i).getLat(), otherPlaneLocs->at(i).getLon(), 
							locations->at(i).getLat(), locations->at(i).getLon(), "meters") < 12)
				{
					ROS_WARN_STREAM("Actually found a collision at time: " << i);
					std::queue<point> cFreePath = findLocalAstarPath(discretePlanes, planeID, lastWaypointTime, i);
				}
			}
		}
	}
}


void prar::generateMapValuesFromWaypoints(map_tools::gridValues &gridValsOut, WaypointVectorMap &allPlanesWaypoints) {
	// gridValsOut is changing the global variable gridVals. Just don't want to hard code that if we move away from global
	double upperLeftLat = -1000;
	double upperLeftLon = 1000;
	double bottomLeftLat = 1000;
	double upperRightLon = -1000;

	WaypointVectorMap::iterator it;
	for (it = allPlanesWaypoints.begin(); it != allPlanesWaypoints.end(); ++it) {
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


	ROS_INFO_STREAM(gridVals.upperLeftLatitude << " " << gridVals.upperLeftLongitude << " " << gridVals.latitudeWidth << " " << gridVals.longitudeWidth);
}


// Methods to convert between PlaneObject map to DiscritizedPlane map
void prar::convertAllPlanesAndAddWaypoints(PlaneObjectMap &allPlanes, DiscretePlaneMap &discretePlanes, WaypointVectorMap &allPlanesWaypoints) {
	PlaneObjectMap::iterator it;
	for (it = allPlanes.begin(); it != allPlanes.end(); ++it) {
		discretePlanes[it->first] = convertRealPlane(it->second, allPlanesWaypoints[it->second.getID()]);
	}
}

DiscretizedPlane prar::convertRealPlane(const PlaneObject &continuousPlane, const waypointVector &allWaypoints) {
	Position current = Position(gridVals, continuousPlane.getCurrentLoc().latitude, continuousPlane.getCurrentLoc().longitude);
	Position goal = Position(gridVals, continuousPlane.getDestination().latitude, continuousPlane.getDestination().longitude);
	DiscretizedPlane dPlane(continuousPlane.getID(), current, goal);


	// Change point to something better, like waypoint, just try to not tie the code to AU_UAV_ROS too much but should in the end anyways
	vector<Position> convertedWaypoints;
	for (int i = 0; i < allWaypoints.size(); i++) {
		Position current = Position(gridVals, allWaypoints[i].latitude, allWaypoints[i].longitude);
		convertedWaypoints.push_back(current);
	}
	dPlane.setAllWaypoints(convertedWaypoints);

	return dPlane; 
}
