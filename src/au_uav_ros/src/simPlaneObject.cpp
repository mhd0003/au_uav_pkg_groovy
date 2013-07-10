#include "au_uav_ros/simPlaneObject.h"
using namespace au_uav_ros;

#define MAXIMUM_TURNING_ANGLE 22.5 //degrees

SimPlaneObject::SimPlaneObject(void) : PlaneObject() {
	simSpeed = 1;
}

SimPlaneObject::SimPlaneObject(struct waypoint wp) : PlaneObject(wp) {
	/* TODO Why does wp index need to be -1 on start vs 0 */
	//this->speed = MPH_SPEED;
	this->speed = MPS_SPEED;
	simSpeed = 1;
}

void SimPlaneObject::setSimSpeed(double _simSpeed) {
	simSpeed = _simSpeed; //TODO guard input if negatives don't work
}

double SimPlaneObject::getSimSpeed(void) const {
	return this->simSpeed;
}

bool SimPlaneObject::simulate(double duration, au_uav_ros::Telemetry *telem) {
	fillTelemetryUpdate(duration, telem);
	return true;
}

bool SimPlaneObject::handleNewCommand(au_uav_ros::Command newCommand) {
	
	if(this->id != newCommand.planeID)
	{
		return false;
	}
	if (newCommand.commandID == COMMAND_AVOID_WP)  {
		avoidWp.latitude = newCommand.latitude;
		avoidWp.longitude = newCommand.longitude;
		avoidWp.altitude = newCommand.altitude;
	} else if (newCommand.commandID == COMMAND_NORMAL_WP) {
		waypoint wp;
		wp.latitude = newCommand.latitude;
		wp.longitude = newCommand.longitude;
		wp.altitude = newCommand.altitude;
		normalPath.push_back(wp);
	}	
	return true;
}

bool SimPlaneObject::fillTelemetryUpdate(double duration, au_uav_ros::Telemetry *update) {
	//difference in latitudes in radians
	double lat1 = currentLoc.latitude*DEGREES_TO_RADIANS;
	double long1 = currentLoc.longitude*DEGREES_TO_RADIANS;
	double lat2, long2;
	if (avoidWp != INVALID_WP) {
		lat2 = avoidWp.latitude*DEGREES_TO_RADIANS;
		long2 = avoidWp.longitude*DEGREES_TO_RADIANS;
	} else {
		lat2 = normalPath.front().latitude*DEGREES_TO_RADIANS;
		long2 = normalPath.front().longitude*DEGREES_TO_RADIANS;
	}
	
	double deltaLat = lat2 - lat1;
	double deltaLong = long2 - long1;

	//calculate bearing from current position to destination
	double y = sin(deltaLong)*cos(lat2);
	double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(deltaLong);
	this->targetBearing = atan2(y, x)*RADIANS_TO_DEGREES;
	

	//calculate the real bearing based on our maximum angle change
	//first create a temporary bearing that is the same as bearing but at a different numerical value
	double tempBearing = -1000;
	if((this->targetBearing) < 0)
	{
		tempBearing = this->targetBearing + 360;
	}
	else
	{
		tempBearing = this->targetBearing - 360;
	}

	double diff1 = abs(this->currentBearing - this->targetBearing);
	double diff2 = abs(this->currentBearing - tempBearing);

	//check for easy to calculate values first
	if(diff1 < (MAXIMUM_TURNING_ANGLE*simSpeed*duration) || diff2 < (MAXIMUM_TURNING_ANGLE*simSpeed*duration))
	{
		//the difference is less than our maximum angle, set it to the bearing
		this->currentBearing = this->targetBearing;
	}
	else
	{
		//we have a larger difference than we can turn, so turn our maximum
		double mod;
		if(diff1 < diff2)
		{
			if(this->targetBearing > this->currentBearing) mod = (MAXIMUM_TURNING_ANGLE*simSpeed*duration);
			else mod = 0 - (MAXIMUM_TURNING_ANGLE*simSpeed*duration);
		}
		else
		{
			if(tempBearing > this->currentBearing) mod = (MAXIMUM_TURNING_ANGLE*simSpeed*duration);
			else mod = 0 - (MAXIMUM_TURNING_ANGLE*simSpeed*duration);
		}

		//add our mod, either +22.5 or -22.5
		this->currentBearing = this->currentBearing + mod;

		//tweak the value to keep it between -180 and 180
		if(this->currentBearing > 180) this->currentBearing = this->currentBearing - 360;
		if(this->currentBearing <= -180) this->currentBearing = this->currentBearing + 360;
	}

	//time to calculate the new positions, God help us
	/*
	Algorithm for updating position:
	1) Estimate new latitude using basic trig and this equation:
	   lat2 = lat1 + (MPS_SPEED*cos(bearing))*METERS_TO_LATITUDE
	2) Use law of haversines to find the new longitude
	   haversin(c) = haversin(a-b) + sin(a)*sin(b)*haversin(C)
	   where haversin(x) = (sin(x/2.0))^2
	   where c = MPS_SPEED/EARTH_RADIUS (radians)
	   where a = 90 - lat1 (degrees)
	   where b = 90 - lat2 (degrees)
	   where C = the change in longitude, what we are solving for
	   
	   C = 2.0 * arcsin(sqrt((haversin(c) - haversin(a-b))/(sin(a)*sin(b))))
	*/

	//1) Estimate new latitude using basic trig and this equation
	this->currentLoc.latitude = lat1*RADIANS_TO_DEGREES + ((MPS_SPEED*duration*simSpeed)*cos(this->currentBearing*DEGREES_TO_RADIANS))*METERS_TO_LATITUDE;

	//2) Use the law of haversines to find the new longitude
	double temp = pow(sin(((MPS_SPEED*simSpeed*duration)/EARTH_RADIUS)/2.0), 2); //TODO verify
	temp = temp - pow(sin((this->currentLoc.latitude*DEGREES_TO_RADIANS - lat1)/2.0), 2);
	temp = temp / (sin(M_PI/2.0 - lat1)*sin((M_PI/2.0)-this->currentLoc.latitude*DEGREES_TO_RADIANS));
	temp = 2.0 * RADIANS_TO_DEGREES * asin(sqrt(temp));
	
	//depending on bearing, we should be either gaining or losing longitude
	if(currentBearing > 0)
	{
		this->currentLoc.longitude += temp;
	}
	else
	{
		this->currentLoc.longitude -= temp;
	}

	if (avoidWp != INVALID_WP) {
		this->distanceToDestination = distanceBetween(getCurrentLoc(), avoidWp);
	} else {
		this->distanceToDestination = distanceBetween(getCurrentLoc(), normalPath.front());
	}

	update->planeID = this->id;
	update->currentLatitude = this->currentLoc.latitude;
	update->currentLongitude = this->currentLoc.longitude;
	update->currentAltitude = this->currentLoc.altitude;

	//COLLISION_THRESHOLD is 12 meters - defined in standardDefs.h

	//first see if we need to dump any points from the avoidance path (dump wp if within 2s of it)
	if( (avoidWp != INVALID_WP) && distanceToDestination > -WAYPOINT_THRESHOLD && distanceToDestination < WAYPOINT_THRESHOLD)
	{
		//this means we reached the avoidance wp and so the next command should be issued
		avoidWp = INVALID_WP;
	}
	//if here then destination is in normalPath
	//next see if we need to dump any points from the normal path (dump wp if within 1s of it)
	else if(!normalPath.empty() && distanceToDestination > -WAYPOINT_THRESHOLD && distanceToDestination < WAYPOINT_THRESHOLD)
	{
		// sim planes dont pop the last point
		if (normalPath.size() > 1) {
			normalPath.pop_front();
		}
	}

	
	if (avoidWp != INVALID_WP) {
		update->destLatitude = avoidWp.latitude;
		update->destLongitude = avoidWp.longitude;
		update->destAltitude = avoidWp.altitude;
	} else {
		waypoint wp = normalPath.front();
		update->destLatitude = wp.latitude;
		update->destLongitude = wp.longitude;
		update->destAltitude = wp.altitude;
	}
	
	this->speed = MPS_SPEED*duration*simSpeed; // md
	update->groundSpeed = this->speed;
	//update->targetBearing = this->targetBearing;
	update->targetBearing = this->currentBearing;
	
	update->distanceToDestination = this->distanceToDestination;
	update->telemetryHeader.stamp = ros::Time::now();
	updateTime();
	
	return true;
}
