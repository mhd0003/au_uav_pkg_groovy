#include "au_uav_ros/discretizedPlane.h"
using namespace au_uav_ros;

#define maxTurningAngle 22.5

// @param bearingAtStart is assumed to be zero at start of simulation. This parameter can also be used
// to move the plane through time from any point in the simulation onward though
void DiscretizedPlane::moveThroughTime(double bearingAtStart) {
	int waypointIndex = 0;
	locationsThroughTime.push_back(current);
	double actualBearing = bearingAtStart;
	Position currentPosition = current;
	while (waypointIndex < allWaypoints.size()) {
		double lat1 = currentPosition.getLat() * DEGREES_TO_RADIANS;
		double long1 = currentPosition.getLon() * DEGREES_TO_RADIANS;
		double lat2 = allWaypoints[waypointIndex].getLat() * DEGREES_TO_RADIANS;
		double long2 = allWaypoints[waypointIndex].getLon() * DEGREES_TO_RADIANS;
		double deltaLat = lat2 - lat1;
		double deltaLong = long2 - long1;
		
		double y = sin(deltaLong)*cos(lat2);
		double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(deltaLong);
		double bearingToGoal = atan2(y, x)*RADIANS_TO_DEGREES;

		//calculate the real bearing based on our maximum angle change
		//first create a temporary bearing that is the same as bearing but at a different numerical value
		double tempBearing = -1000;
		if(bearingToGoal < 0) {
			tempBearing = bearingToGoal + 360;
		} else {
			tempBearing = bearingToGoal - 360;
		}
		
		double diff1 = fabs(actualBearing - bearingToGoal);
		double diff2 = fabs(actualBearing - tempBearing);
	
		//check for easy to calculate values first
		if(diff1 < maxTurningAngle || diff2 < maxTurningAngle) {
			//the difference is less than our maximum angle, set it to the bearing
			actualBearing = bearingToGoal;
		} else {
			//we have a larger difference than we can turn, so turn our maximum
			double mod;
			if(diff1 < diff2) {
				if(bearingToGoal > actualBearing) mod = maxTurningAngle;
				else mod = 0 - maxTurningAngle;
			} else {
				if(tempBearing > actualBearing) mod = maxTurningAngle;
				else mod = 0 - maxTurningAngle;
			}
		
			//add our mod, either +22.5 or -22.5
			actualBearing = actualBearing + mod;
		
			//tweak the value to keep it between -180 and 180
			if(actualBearing > 180) actualBearing = actualBearing - 360;
			if(actualBearing <= -180) actualBearing = actualBearing + 360;
		}

		//1) Estimate new latitude using basic trig and this equation
		double currentLatitude = lat1*RADIANS_TO_DEGREES + (11.176*cos(actualBearing*DEGREES_TO_RADIANS))*(1.0/111200.0);
		
		//2) Use the law of haversines to find the new longitude
		//double temp = pow(sin((MPS_SPEED/EARTH_RADIUS)/2.0), 2);
		double temp = 7.69303281*pow(10, -13); //always the same, see above calculation
		temp = temp - pow(sin((currentLatitude*DEGREES_TO_RADIANS - lat1)/2.0), 2);
		temp = temp / (sin(M_PI/2.0 - lat1)*sin((M_PI/2.0)-currentLatitude*DEGREES_TO_RADIANS));
		temp = 2.0 * RADIANS_TO_DEGREES * asin(sqrt(temp));
		
		double currentLongitude = currentPosition.getLon();
		//depending on bearing, we should be either gaining or losing longitude
		if(actualBearing > 0) {
			currentLongitude += temp;
		}
		else {
			currentLongitude -= temp;
		}	
		
		currentPosition.setLatLon(currentLatitude, currentLongitude);
		currentPosition.setBearing(actualBearing);
		currentPosition.setIsWaypoint(false);

		// Do haversine calculations because that's what simulation does to caculate
		// if the waypoint has been reached
		lat1 = currentLatitude*DEGREES_TO_RADIANS;
		//lat2 up above still applicable
		long1 = currentLongitude*DEGREES_TO_RADIANS;
		//long2 above still applicable
		deltaLat = lat2 - lat1;
		deltaLong = long2 - long1;

		double a = pow(sin(deltaLat / 2.0), 2);
		a = a + cos(lat1)*cos(lat2)*pow(sin(deltaLong/2.0), 2);
		a = 2.0 * asin(sqrt(a));
		if((a * 6371000) < 30) {
			waypointIndex++;
			currentPosition.setIsWaypoint(true);
		}
		locationsThroughTime.push_back(currentPosition);
	}
}

void DiscretizedPlane::setLocationsThroughTime(vector<Position> locs) {
	locationsThroughTime = locs;
}

vector<Position>* DiscretizedPlane::getLocationsThroughTime() {
	return &locationsThroughTime;
}

void DiscretizedPlane::setAllWaypoints(vector<Position> allWaypointsIn) {
	allWaypoints = allWaypointsIn;
}

vector<Position> DiscretizedPlane::getAllWaypoints() {
	return allWaypoints;
}

void DiscretizedPlane::update_current(Position newcurrent) {
    // Only if the "current" location came from a telemetry update should it affect
    // our last position
    if(!current_is_virtual)
        lastPosition.setLatLon(current.getLat(), current.getLon());
    
	current = newcurrent; //.setLatLon( newcurrent.getLat(), newcurrent.getLon() );
    current_is_virtual = false;
    
    // Find both the plane's current bearing and its bearing to its destination,
    // saving them to the proper variables
	calculateBearings();
}

void DiscretizedPlane::virtual_update_current(Position virtual_current) {
	current = virtual_current;
    current_is_virtual = true;
    
    // Note: We do not change bearings, since this is only a "virtual" update
}

void DiscretizedPlane::update_intermediate_wp(Position next) {
	destination = next;
    
    // NOTE: Changing the intermediate waypoint does not change our bearing
}

void DiscretizedPlane::update(Position newcurrent, Position newdestination, double newspeed) {
    if(!current_is_virtual)
        lastPosition.setLatLon(current.getLat(), current.getLon());
    
	current = newcurrent; //.setLatLon( newcurrent.getLat(), newcurrent.getLon() );
    current_is_virtual = false;
    
	destination = newdestination;//.setLatLon( newdestination.getLat(), newdestination.getLon() );
    
	speed=newspeed;
    
    // finds both the plane's current bearing and its bearing to its destination,
    // saving them to the proper variables
	calculateBearings();
}

void DiscretizedPlane::setFinalDestination(double lon, double lat) {
	finalDestination.setLatLon(lat, lon);
    
    calculateBearings();
}

void DiscretizedPlane::setDestination(double lon, double lat) {
    destination.setLatLon(lat, lon);
}

void DiscretizedPlane::setFinalDestination(int x, int y) {
	finalDestination.setXY(x, y);
    
    calculateBearings();
}

Position DiscretizedPlane::getFinalDestination() {
	return finalDestination;
}

void DiscretizedPlane::setDestination(int x, int y)
{
	destination.setXY(x, y);
}

double DiscretizedPlane::getBearing() const {
	return bearing;
}

map_tools::bearing_t DiscretizedPlane::get_named_bearing() const {
    return map_tools::name_bearing(bearing);
}

double DiscretizedPlane::getBearingToDest() const {
	return bearingToDest;
}

map_tools::bearing_t DiscretizedPlane::get_named_bearing_to_dest() const {
    return map_tools::name_bearing(bearingToDest);
}

void DiscretizedPlane::calculateBearings() {
    // NOTE: the map_tools::calculateBearing() fn takes lat and long in DEGREES
    double lat1 = current.getLat();
    double lon1 = current.getLon();
    
    // If our current location isn't the same as our previous location . . .
	if(!(current==lastPosition)) {
		//uses the same method that the simulator uses to find the planes bearing
		bearing = map_tools::calculateBearing( lastPosition.getLat(),
                                              lastPosition.getLon(),
                                              lat1, lon1 );
	}
    
    bearingToDest = map_tools::calculateBearing( lat1, lon1,
                                                finalDestination.getLat(),
                                                finalDestination.getLon() );
}

double DiscretizedPlane::getSpeed() {
	return speed;
}

int DiscretizedPlane::getId() {
	return id;
}

bool DiscretizedPlane::is_initialized() {
	// if( id == -1 ) {
	// 	return false;
	// }
	// return true;
	return (id != -1);
}


Position DiscretizedPlane::getLocation() {
	return current;
}

Position DiscretizedPlane::getDestination() {
	return destination;
}

DiscretizedPlane::DiscretizedPlane(int newid) {
	id = newid;
}

DiscretizedPlane::DiscretizedPlane(int newid, Position initial, Position goal ) {
    id = newid;
    current = Position(initial);
    destination = Position(goal);
    finalDestination = Position(goal);
    lastPosition = Position(initial);
    bearing = 0;
    current_is_virtual = false;
    
    calculateBearings();
}
