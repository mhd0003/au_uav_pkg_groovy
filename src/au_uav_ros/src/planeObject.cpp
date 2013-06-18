/* Implementation of planeObject.h

*/

#include "ros/ros.h"
#include "au_uav_ros/planeObject.h"
#include "au_uav_ros/SimulatedPlane.h"
#include <math.h>
#include "au_uav_ros/standardFuncs.h" /* for PI, EARTH_RADIUS in meters */

/* Implementation of the default constructor: Member variables are set to zero */
au_uav_ros::PlaneObject::PlaneObject(void) {
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
	this->destination.latitude = 0.0;
	this->destination.longitude = 0.0;
	this->destination.altitude = 0.0;
	this->lastUpdateTime = ros::Time::now().toSec();
	this->collisionRadius = 0.0;

}
/* Explicit value constructor using TelemetryUpdate */
au_uav_ros::PlaneObject::PlaneObject(double cRadius, const au_uav_ros::TelemetryUpdate &msg) {
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
	this->destination.latitude = msg.destLatitude;
	this->destination.longitude = msg.destLongitude;
	this->destination.altitude = msg.destAltitude;
	this->lastUpdateTime = ros::Time::now().toSec();
	this->collisionRadius = cRadius;

}

/* mutator functions to update member variables */
void au_uav_ros::PlaneObject::setID(int id){
	this->id = id;
}

void au_uav_ros::PlaneObject::setPreviousLoc(double lat, double lon, double alt) {
	this->previousLoc.latitude = lat;
	this->previousLoc.longitude = lon;
	this->previousLoc.altitude = alt;
}

void au_uav_ros::PlaneObject::setCurrentLoc(double lat, double lon, double alt) {
	this->currentLoc.latitude = lat;
	this->currentLoc.longitude = lon;
	this->currentLoc.altitude = alt;
}

void au_uav_ros::PlaneObject::setTargetBearing(double tBearing) {
	this->targetBearing = tBearing;
}

void au_uav_ros::PlaneObject::setCurrentBearing(double cBearing) {
	this->currentBearing = cBearing;
}

void au_uav_ros::PlaneObject::setSpeed(double speed) {
	this->speed = speed;
}

void au_uav_ros::PlaneObject::setDestination(const au_uav_ros::waypoint &destination) {
	this->destination = destination;
}

void au_uav_ros::PlaneObject::updateTime(void) {
	this->lastUpdateTime = ros::Time::now().toSec();
}


void au_uav_ros::PlaneObject::update(const au_uav_ros::TelemetryUpdate &msg) {

	//Update previous and current position
	this->setPreviousLoc(this->currentLoc.latitude, this->currentLoc.longitude, this->currentLoc.altitude);
	this->setCurrentLoc(msg.currentLatitude, msg.currentLongitude, msg.currentAltitude);
	
	//Calculate actual Cardinal Bearing
	double numerator = (this->currentLoc.latitude - this->previousLoc.latitude);
	double denominator = (this->currentLoc.longitude - this->previousLoc.longitude);
	double angle = atan2(numerator*DELTA_LAT_TO_METERS,denominator*DELTA_LON_TO_METERS)*180/PI;

	if (this->currentLoc.latitude != this->previousLoc.latitude && this->currentLoc.longitude != this->previousLoc.longitude)
	{ 
			this->setCurrentBearing(toCardinal(angle));
	}
	else this->setCurrentBearing(0.0);

	// Update everything else
	this->setTargetBearing(msg.targetBearing);
	this->setSpeed(msg.groundSpeed);
	this->updateTime();
}

/* accessor functions */
int au_uav_ros::PlaneObject::getID(void) const {
	return this->id;
}

au_uav_ros::coordinate au_uav_ros::PlaneObject::getPreviousLoc(void) const {
	return this->previousLoc;
}

au_uav_ros::coordinate au_uav_ros::PlaneObject::getCurrentLoc(void) const {
	return this->currentLoc;
}

double au_uav_ros::PlaneObject::getTargetBearing(void) const {
	return this->targetBearing;
}

double au_uav_ros::PlaneObject::getCurrentBearing(void) const {
	return this->currentBearing;
}
	
double au_uav_ros::PlaneObject::getSpeed(void) const {
	return this->speed;
}

double au_uav_ros::PlaneObject::getLastUpdateTime(void) const {
	return this->lastUpdateTime;
}

au_uav_ros::waypoint au_uav_ros::PlaneObject::getDestination(void) const {
	return this->destination;
}

/* Find distance between this plane and another plane, returns in meters */
double au_uav_ros::PlaneObject::findDistance(const au_uav_ros::PlaneObject& plane) const {
	return this->findDistance(plane.currentLoc.latitude, plane.currentLoc.longitude);
}


/* Find distance between this plane and another pair of coordinates, 
returns value in meters */
double au_uav_ros::PlaneObject::findDistance(double lat2, double lon2) const {
	double xdiff = (lon2 - this->currentLoc.longitude)*DELTA_LON_TO_METERS;
	double ydiff = (lat2 - this->currentLoc.latitude)*DELTA_LAT_TO_METERS;

	return sqrt(pow(xdiff, 2) + pow(ydiff, 2));
}

/* Find Cartesian angle between this plane and another plane, using this plane
as the origin */
double au_uav_ros::PlaneObject::findAngle(const au_uav_ros::PlaneObject& plane) const {
	return this->findAngle(plane.currentLoc.latitude, plane.currentLoc.longitude);
}

/* Find Cartesian angle between this plane and another plane's latitude/longitude 
using this plane as the origin */
double au_uav_ros::PlaneObject::findAngle(double lat2, double lon2) const {
	double xdiff = (lon2 - this->currentLoc.longitude)*DELTA_LON_TO_METERS;
	double ydiff = (lat2 - this->currentLoc.latitude)*DELTA_LAT_TO_METERS;
	
	return atan2(ydiff, xdiff);
}


au_uav_ros::PlaneObject& au_uav_ros::PlaneObject::operator=(const au_uav_ros::PlaneObject& plane) {

	this->id = plane.id;
	this->currentLoc.altitude = plane.currentLoc.altitude;
	this->currentLoc.latitude = plane.currentLoc.latitude;
	this->currentLoc.longitude = plane.currentLoc.longitude;

	this->previousLoc.altitude = plane.previousLoc.altitude;
	this->previousLoc.latitude = plane.previousLoc.latitude;
	this->previousLoc.longitude = plane.previousLoc.longitude;

	this->destination.latitude = plane.destination.latitude;
	this->destination.longitude = plane.destination.longitude;
	this->destination.altitude = plane.destination.latitude;

	this->targetBearing = plane.targetBearing;
	this->currentBearing = plane.currentBearing;

	this->speed = plane.speed;
	this->lastUpdateTime = plane.lastUpdateTime;
	this->collisionRadius = plane.collisionRadius;

	return *this;
}
