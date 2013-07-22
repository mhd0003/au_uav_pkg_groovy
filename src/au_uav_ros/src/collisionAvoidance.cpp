#include "au_uav_ros/collisionAvoidance.h"
#include "a_star/prar.cpp"
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

        newWaypoint.planeID = id;
        wps.push_back(newWaypoint);
    }
}

void CollisionAvoidance::distrubuted_avoid(int id, std::map<int, PlaneObject> planes, std::map<int, SimPlaneObject> simPlanes, waypoint &avoidanceWP) {
    std::map<int, PlaneObject> allPlanes;
    allPlanes.insert(planes.begin(), planes.end());
    allPlanes.insert(simPlanes.begin(), simPlanes.end());

/*
    std::map<int, SimPlaneObject>::iterator it;
    for (it = simPlanes.begin(); it != simPlanes.end(); ++it) {
        // Don't run CA on the plane that just pushed the update?
        // This can be changed.
        if (it->first == id) {
            continue;
        }

        if (ipn::checkForThreats(simPlanes[it->first], allPlanes, avoidanceWP)) {
            avoidanceWP.planeID = it->first;
            wps.push_back(avoidanceWP);
        }

    }
*/
    // This should never happen since the function is only called
    // after the id is confirmed. But just in case..
    if (simPlanes.find(id) == simPlanes.end()) {
        ROS_ERROR("WOAHHH #%d ISNT IN SIMPLANEMAP!", id);
        return;
    }

    if (ipn::checkForThreats(simPlanes[id], allPlanes, avoidanceWP)) {
        avoidanceWP.planeID = id;
    }
}


/////////////////////////////////////////////////////////////////
//////                       Begin A*                       /////
/////////////////////////////////////////////////////////////////

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

    // Convert between map_tools::waypointPath and std::vector<waypoint>
    for (itt = allPlanesWaypointPath.begin(); itt != allPlanesWaypointPath.end(); itt++) {
        allPlanesPath[itt->first] = allPlanesWaypointPath[itt->first].pathWaypoints;
    }
}
