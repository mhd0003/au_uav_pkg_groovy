#include "au_uav_ros/collisionAvoidance.h"
using namespace au_uav_ros;

#define CA_PRINT_DEBUG true

void CollisionAvoidance::avoid(int id, std::map<int, PlaneObject> planes, std::map<int, SimPlaneObject> simPlanes, std::vector<waypoint> &wps) {
	std::map<int, PlaneObject> allPlanes;
	allPlanes.insert(planes.begin(), planes.end());
	allPlanes.insert(simPlanes.begin(), simPlanes.end());

	// md
	// waypointContainer bothNewWaypoints = findNewWaypoint(allPlanes[id], planes);
	waypointContainer bothNewWaypoints = findNewWaypoint(allPlanes[id], allPlanes);
	waypoint newWaypoint = bothNewWaypoints.plane1WP;
	
	// md
	// ID: -1000 means dubins path was taken... (plane not in danger)
	if (bothNewWaypoints.plane2ID != -1000) {
		newWaypoint.planeID = id;
		wps.push_back(newWaypoint);
	}

	if (bothNewWaypoints.plane2ID >= 0) {
		waypoint newWaypoint2 = bothNewWaypoints.plane2WP;
		newWaypoint2.planeID = bothNewWaypoints.plane2ID;
		wps.push_back(newWaypoint2);
	}
	/* TODO THIS IS BROKE */
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
