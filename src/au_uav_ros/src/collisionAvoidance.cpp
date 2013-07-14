#include "au_uav_ros/collisionAvoidance.h"
#include "a_star/prar.cpp"
using namespace au_uav_ros;

#define CA_PRINT_DEBUG true

void CollisionAvoidance::avoid(int id, std::map<int, PlaneObject> planes, std::map<int, SimPlaneObject> simPlanes, std::vector<waypoint> &wps) {
	std::map<int, PlaneObject> allPlanes;
	allPlanes.insert(planes.begin(), planes.end());
	allPlanes.insert(simPlanes.begin(), simPlanes.end());

	waypointContainer bothNewWaypoints = findNewWaypoint(allPlanes[id], planes);
	waypoint newWaypoint = bothNewWaypoints.plane1WP;
	
	if (bothNewWaypoints.plane2ID >= 0) {
		waypoint newWaypoint2 = bothNewWaypoints.plane2WP;
		newWaypoint2.planeID = bothNewWaypoints.plane2ID;
		wps.push_back(newWaypoint2);

		newWaypoint.planeID = id;
		wps.push_back(newWaypoint);
		
	} /* TODO THIS IS BROKE */
	/* TODO if ((requestWaypointInfoSrv.response.longitude == newWaypoint.longitude) 
		&& (requestWaypointInfoSrv.response.latitude == newWaypoint.latitude)) {return;} what does this do */
	/**newWaypoint.planeID = id;
	wps.push_back(newWaypoint); */
}

void CollisionAvoidance::distrubuted_avoid(int id, std::map<int, PlaneObject> planes, std::map<int, SimPlaneObject> simPlanes, waypoint &avoidanceWP) {
	// This should never happen since the function is only called
	// after the id is confirmed. But just in case..
	if (simPlanes.find(id) == simPlanes.end()) {
		return;
	}

	std::map<int, PlaneObject> allPlanes;
	allPlanes.insert(planes.begin(), planes.end());
	allPlanes.insert(simPlanes.begin(), simPlanes.end());

	// std::map<int, SimPlaneObject>::iterator it;
	// for (it = simPlanes.begin(); it != simPlanes.end(); ++it) {
	// 	// Don't run CA on the plane that just pushed the update?
	// 	// This can be changed.
	// 	if (it->first == id) {
	// 		continue;
	// 	}

	// 	if (ipn::checkForThreats(simPlanes[it->first], allPlanes, avoidanceWP)) {
	// 		avoidanceWP.planeID = it->first;
	// 		wps.push_back(avoidanceWP);
	// 	}
		
	// }

	if (ipn::checkForThreats(simPlanes[id], allPlanes, avoidanceWP)) {
		avoidanceWP.planeID = id;
	}
	
}


/////////////////////////////////////////////////////////////////
//////                       Begin A*                       /////
/////////////////////////////////////////////////////////////////

vector<au_uav_ros::waypoint> generateCombinedWaypoints(vector<au_uav_ros::waypoint> *realWaypoints, vector<map_tools::waypointPath> *avoidPaths) {
	int avoidanceIndex = 0;
	vector<au_uav_ros::waypoint> combined;
	//TODO fix this using gridVals, this only works because gridVals is global
	for (int i = 0; i < realWaypoints->size(); i++) {
		// i-1 is for if the first avoidance path is from the plane's start position to the first waypoint
		if (avoidanceIndex < avoidPaths->size() && i-1 == (*avoidPaths)[avoidanceIndex].startWaypointIndex) {
			for (int j = 0; j < (*avoidPaths)[avoidanceIndex].pathWaypoints.size(); j++) {
				combined.push_back((*avoidPaths)[avoidanceIndex].pathWaypoints[j]);
			}
			avoidanceIndex++;
		}
		// add next waypoint
		combined.push_back((*realWaypoints)[i]);
	}
	return combined;
}



void CollisionAvoidance::astar_planPath(std::map<int, PlaneObject> planes,
										std::map<int, SimPlaneObject> simPlanes,
										std::map<int, std::vector<waypoint> > &allPlanesPath) {
	std::map<int, PlaneObject> allPlanes;
	std::map<int, std::vector<waypoint> > allPlanesWaypoints;
	std::map<int, map_tools::waypointPath> allPlanesWaypointPath;
	std::map<int, PlaneObject>::iterator it;
	std::map<int, map_tools::waypointPath>::iterator itt;

	allPlanes.insert(planes.begin(), planes.end());
	allPlanes.insert(simPlanes.begin(), simPlanes.end());

	// Add all normal waypoints for every plane to allPlanesWaypoints.
	for (it = allPlanes.begin(); it != allPlanes.end(); it++) {
		allPlanesWaypoints[it->first] = it->second.getNormalPath();
	}

	// Get an avoidance plan for each plane. (this calls function in prar)
	allPlanesWaypointPath = getAStarPath(&allPlanes, &allPlanesWaypoints);

	// Combine the avoidance plan with the current waypoints.
	for (itt = allPlanesWaypointPath.begin(); itt != allPlanesWaypointPath.end(); itt++) {

		allPlanesPath[itt->first] = allPlanesWaypointPath[itt->first].pathWaypoints;

		int pathSize = allPlanesPath[itt->first].size();
		ROS_ERROR("CA: Planned path for id #%d should have %d points", itt->first, pathSize);

		// Coordinator will handle setting the waypoints for everyone.
		// It will call the service for Simulator to do the same.
	}
}
