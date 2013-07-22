#include "au_uav_ros/planeObject.h"
using namespace au_uav_ros;

/* Implementation of the default constructor: Member variables are set to zero */
PlaneObject::PlaneObject(void) {
    this->id = 0;
    this->currentLoc.altitude = 0.0;
    this->currentLoc.latitude = 0.0;
    this->currentLoc.longitude = 0.0;
    this->previousLoc.altitude = 0.0;
    this->previousLoc.latitude = 0.0;
    this->previousLoc.longitude = 0.0;
    this->targetBearing = 0.0;
    this->currentBearing = 0.0;

    this->speed = 0.0;
    this->lastUpdateTime = ros::Time::now().toSec();
    this->collisionRadius = 0.0;

    avoidWp = INVALID_WP;

}

PlaneObject::PlaneObject(struct waypoint wp) {
    this->id = wp.planeID;
    this->currentLoc.altitude = 0.0;
    this->currentLoc.latitude = 0.0;
    this->currentLoc.longitude = 0.0;
    this->previousLoc.altitude = 0.0;
    this->previousLoc.latitude = 0.0;
    this->previousLoc.longitude = 0.0;
    this->targetBearing = 0.0;
    this->currentBearing = 0.0;

    this->speed = 0.0;
    this->collisionRadius = 0.0;
    this->lastUpdateTime = ros::Time::now().toSec();
    this->normalPath.push_back(wp);

    avoidWp = INVALID_WP;
}

PlaneObject::PlaneObject(int _id) {
    this->id = _id;
}

/* Explicit value constructor using Telemetry */
PlaneObject::PlaneObject(double cRadius, const Telemetry &msg) {
    this->id = msg.planeID;
    this->currentLoc.altitude = msg.currentAltitude;
    this->currentLoc.latitude = msg.currentLatitude;
    this->currentLoc.longitude = msg.currentLongitude;
    this->previousLoc.altitude = 0.0;
    this->previousLoc.latitude = 0.0;
    this->previousLoc.longitude = 0.0;
    this->targetBearing = msg.targetBearing;
    this->currentBearing = 0.0;

    this->speed = msg.groundSpeed;
    waypoint wp;
    wp.latitude = msg.destLatitude;
    wp.longitude = msg.destLongitude;
    wp.altitude = msg.destAltitude;
    this->normalPath.push_back(wp);
    this->lastUpdateTime = ros::Time::now().toSec();
    this->collisionRadius = cRadius;
}

/* mutator functions to update member variables */
void PlaneObject::setID(int id){
    this->id = id;
}

void PlaneObject::setPreviousLoc(double lat, double lon, double alt) {
    this->previousLoc.latitude = lat;
    this->previousLoc.longitude = lon;
    this->previousLoc.altitude = alt;
}

void PlaneObject::setCurrentLoc(double lat, double lon, double alt) {
    this->currentLoc.latitude = lat;
    this->currentLoc.longitude = lon;
    this->currentLoc.altitude = alt;
}

void PlaneObject::setTargetBearing(double tBearing) {
    this->targetBearing = tBearing;
}

void PlaneObject::setCurrentBearing(double cBearing) {
    this->currentBearing = cBearing;
}

void PlaneObject::setSpeed(double speed) {
    this->speed = speed;
}

void PlaneObject::updateTime(void) {
    this->lastUpdateTime = ros::Time::now().toSec();
}

// md
//
bool PlaneObject::update(const Telemetry &msg, Command &newCommand) {
    //Update previous and current position
    this->setPreviousLoc(this->currentLoc.latitude, this->currentLoc.longitude, this->currentLoc.altitude);
    this->setCurrentLoc(msg.currentLatitude, msg.currentLongitude, msg.currentAltitude);

    // md
    // Changed from cardinal to cartesian.
    // TODO: Get rid of those DELTA_LXX_TO_METERS constants
    double numerator = (this->currentLoc.latitude - this->previousLoc.latitude);
    double denominator = (this->currentLoc.longitude - this->previousLoc.longitude);
    double angle = atan2(numerator*DELTA_LAT_TO_METERS,denominator*DELTA_LON_TO_METERS)*180/PI;

    // md
    if (numerator != 0 || denominator != 0) {
        this->setCurrentBearing(angle);
    }

    // Update everything else
    this->setTargetBearing(msg.targetBearing);
    this->setSpeed(msg.groundSpeed);
    this->updateTime();

    //this bool is set true only if there is something available to be sent to the UAV
    bool isCommand = false;

    //this bool is set if the command available is an avoidance maneuver

    //store the last update
    //this->latestUpdate = update;

    //first see if we need to dump any points from the avoidance path (dump wp if within 2s of it)
    if((avoidWp != INVALID_WP) && msg.distanceToDestination > -WAYPOINT_THRESHOLD && msg.distanceToDestination < WAYPOINT_THRESHOLD)
    {
        // md
        // TODO: What happens if plane pops all mission WPs and then later the CA node makes it avoid a plane?
        //this means we reached the avoidance wp and so the next command should be issued
        avoidWp = INVALID_WP;
        if (!normalPath.empty()) isCommand = true;
    }
    //if here then destination is in normalPath
    //next see if we need to dump any points from the normal path (dump wp if within 1s of it)
    else if(!normalPath.empty() && msg.distanceToDestination > -WAYPOINT_THRESHOLD && msg.distanceToDestination < WAYPOINT_THRESHOLD)
    {
        //this means we met the normal path's waypoint, so pop it
        normalPath.pop_front();
        //if we pop'd a wp, and there is another wp in normalPath, command is true.
        if (!normalPath.empty()) isCommand = true;
    }

    // while (!plannedPath.empty() && distanceBetween(currentLoc, plannedPath.front()) < WAYPOINT_THRESHOLD) {
    //     plannedPath.pop_front();
    //     isCommand = true;
    // }

    // if (!plannedPath.empty()) {
    //     plannedPath.pop_front();
    //     isCommand = true;
    // }

    //if we have a command to process, process it
    if(isCommand)
    {
        if (avoidWp != INVALID_WP) {
            newCommand.commandID = COMMAND_AVOID_WP;
            newCommand.commandHeader.stamp = ros::Time::now();
            newCommand.planeID = id;
            newCommand.latitude = avoidWp.latitude;
            newCommand.longitude = avoidWp.longitude;
            newCommand.altitude = avoidWp.altitude;
            return true;
        }
        // else if (!plannedPath.empty()) {
        //     newCommand.commandID = COMMAND_AVOID_WP;
        //     newCommand.commandHeader.stamp = ros::Time::now();
        //     newCommand.planeID = id;
        //     newCommand.latitude = plannedPath.front().latitude;
        //     newCommand.longitude = plannedPath.front().longitude;
        //     newCommand.altitude = plannedPath.front().altitude;
        //     return true;
        // }
        else if (!normalPath.empty()) {
            newCommand.commandID = COMMAND_NORMAL_WP;
            newCommand.commandHeader.stamp = ros::Time::now();
            newCommand.planeID = id;
            newCommand.latitude = normalPath.front().latitude;
            newCommand.longitude = normalPath.front().longitude;
            newCommand.altitude = normalPath.front().altitude;
            return true;
        }
    }
    else {
        //if we get here, then avoidance queue is empty and no new wps need to be sent for normal path
        return false;
    }
}

/* accessor functions */
int PlaneObject::getID(void) const {
    return this->id;
}

waypoint PlaneObject::getPreviousLoc(void) const {
    return this->previousLoc;
}

waypoint PlaneObject::getCurrentLoc(void) const {
    return this->currentLoc;
}

double PlaneObject::getTargetBearing(void) const {
    return this->targetBearing;
}

double PlaneObject::getCurrentBearing(void) const {
    return this->currentBearing;
}

double PlaneObject::getSpeed(void) const {
    return this->speed;
}

double PlaneObject::getLastUpdateTime(void) const {
    return this->lastUpdateTime;
}

waypoint PlaneObject::getDestination(void) const {
    if (avoidWp != INVALID_WP ) {
        return avoidWp;
    } else if (!plannedPath.empty()) {
        return plannedPath.front();
    } else {
        return normalPath.front();
    }
}

double PlaneObject::getSimSpeed(void) const {
    return 0;
}

/* Find distance between this plane and another plane, returns in meters */
double PlaneObject::findDistance(const PlaneObject& plane) const {
    return this->findDistance(plane.currentLoc.latitude, plane.currentLoc.longitude);
}


/* Find distance between this plane and another pair of coordinates,
returns value in meters */
double PlaneObject::findDistance(double lat2, double lon2) const {
    double xdiff = (lon2 - this->currentLoc.longitude)*DELTA_LON_TO_METERS;
    double ydiff = (lat2 - this->currentLoc.latitude)*DELTA_LAT_TO_METERS;

    return sqrt(pow(xdiff, 2) + pow(ydiff, 2));
}

/* Find Cartesian angle between this plane and another plane, using this plane
as the origin */
double PlaneObject::findAngle(const PlaneObject& plane) const {
    return this->findAngle(plane.currentLoc.latitude, plane.currentLoc.longitude);
}

/* Find Cartesian angle between this plane and another plane's latitude/longitude
using this plane as the origin */
double PlaneObject::findAngle(double lat2, double lon2) const {
    double xdiff = (lon2 - this->currentLoc.longitude)*DELTA_LON_TO_METERS;
    double ydiff = (lat2 - this->currentLoc.latitude)*DELTA_LAT_TO_METERS;

    return atan2(ydiff, xdiff);
}


PlaneObject& PlaneObject::operator=(const PlaneObject& plane) {

    this->id = plane.id;
    this->currentLoc.altitude = plane.currentLoc.altitude;
    this->currentLoc.latitude = plane.currentLoc.latitude;
    this->currentLoc.longitude = plane.currentLoc.longitude;

    this->previousLoc.altitude = plane.previousLoc.altitude;
    this->previousLoc.latitude = plane.previousLoc.latitude;
    this->previousLoc.longitude = plane.previousLoc.longitude;

    this->plannedPath = plane.plannedPath;
    this->normalPath = plane.normalPath;
    this->avoidWp = plane.avoidWp;

    this->targetBearing = plane.targetBearing;
    this->currentBearing = plane.currentBearing;

    this->speed = plane.speed;
    this->lastUpdateTime = plane.lastUpdateTime;
    this->collisionRadius = plane.collisionRadius;

    return *this;
}

void PlaneObject::setPlannedPath(std::vector<waypoint> path) {
    plannedPath.clear();
    plannedPath.insert(plannedPath.begin(), path.begin(), path.end());
}

void PlaneObject::addNormalWp(struct au_uav_ros::waypoint wp) {
    normalPath.push_back(wp);
}

void PlaneObject::addAvoidanceWp(struct au_uav_ros::waypoint wp) {
    avoidWp = wp;
}

void PlaneObject::removeNormalWp(struct au_uav_ros::waypoint wp) {
    std::list<waypoint>::iterator i;
    for (i = normalPath.begin(); i != normalPath.end(); i++) {
        if (wp == *i) {
            break;
        }
    }
    if (i != normalPath.end()) {
        normalPath.erase(i);
    } else {
        ROS_ERROR("Waypoint not found");
    }
}

void PlaneObject::removeAvoidanceWp(void) {
    avoidWp = INVALID_WP;
}

Command PlaneObject::getPriorityCommand(void) {
    //start with defaults
    Command ret;
    ret.planeID = -1;
    ret.latitude = -1000;
    ret.longitude = -1000;
    ret.altitude = -1000;

    //check avoidance queue
    if(avoidWp != INVALID_WP)
    {
        //we have an avoidance point, get that one
        ret.planeID = this->id;
        ret.latitude = avoidWp.latitude;
        ret.longitude = avoidWp.longitude;
        ret.altitude = avoidWp.altitude;
        ret.commandID = COMMAND_AVOID_WP;
    }
    // else if (!plannedPath.empty())
    // {
    //     //we have a point from the planning algorithm
    //     ret.planeID = this->id;
    //     ret.latitude = plannedPath.front().latitude;
    //     ret.longitude = plannedPath.front().longitude;
    //     ret.altitude = plannedPath.front().altitude;
    //     ret.commandID = COMMAND_AVOID_WP;
    // }
    else if (!normalPath.empty())
    {
        //we have a normal path point at least, fill it out
        ret.planeID = this->id;
        ret.latitude = normalPath.front().latitude;
        ret.longitude = normalPath.front().longitude;
        ret.altitude = normalPath.front().altitude;
        ret.commandID = COMMAND_NORMAL_WP;

    }

    //fill out our header and return this bad boy
    ret.commandHeader.stamp = ros::Time::now();
    return ret;
}

std::vector<waypoint> PlaneObject::getNormalPath(void) {
    std::vector<waypoint> allWaypoints;
    if (avoidWp != INVALID_WP) {
        normalPath.push_front(avoidWp);
    }

	// md
	// Something in A* code causes Coordinator node to crash
	// if it doesn't get any waypoints
    if (normalPath.size() == 0) {
        ROS_ERROR("No normal waypoints! Adding current location...");
        normalPath.push_back(currentLoc);
    }
    allWaypoints.reserve(normalPath.size());
    allWaypoints.insert(allWaypoints.begin(), normalPath.begin(), normalPath.end());

    if (avoidWp != INVALID_WP) {
        normalPath.pop_front();
    }

    return allWaypoints;
}
