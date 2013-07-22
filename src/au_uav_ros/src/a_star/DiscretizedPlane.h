//
// DiscretizedPlane.h
// Header file for the plane class for use in the Auburn REU program 2011
// By Thomas Crescenzi, with additions from Tyler Young

#ifndef PLANE
#define PLANE

#include <iostream>
#include <math.h>
#include <vector>

#include "Position.h"
#include "map_tools.h"
#include "au_uav_ros/standardFuncs.h"


#ifndef RADIAN_CONSTANTS
#define RADIAN_CONSTANTS
const double MY_PI = 4.0*atan(1.0);// pi
const double TWO_MY_PI = 2.0*MY_PI;
const double MY_RADIANS_TO_DEGREES = 180.0/MY_PI;//Conversion factor from Radians to Degrees
const double MY_DEGREES_TO_RADIANS = MY_PI/180.0;//Conversion factor from Degrees to Radians
#endif

class DiscretizedPlane
{
private:
    // The plane's ID number; should be unique
	int id;
    
    // The plane's current location, as might be updated through a telemetry callback
	Position current;
    
    // The plane's "next" destination; this might be a collision avoidance waypoint,
    // or it might be identical to the final destination
	Position destination;
    
    // The plane's true goal; NOT a collision avoidance waypoint
	Position finalDestination;
    
    // The plane's previous location; used to calculate its current baring
	Position lastPosition;
    
    // The bearing from the plane's current location to its FINAL destination
    // (its goal), in degrees
	double bearingToDest;
    
    // The bearing from the plane's previous location to its current one, in degrees
	double bearing;
    
    // The plane's speed, in whatever units you want to use (we aren't using this)
	double speed;
    
    /**
     * Updates the plane's current bearing and its bearing to its goal.
     * Current bearing is calculated as the bearing from the previous location to
     * the current one, so you should only call this function when you update the
     * current location.
     */
	void calculateBearings();
    
    // Indicates whether your current position is a "virtual" position; see the
    // virtual_update_current() for more
    bool current_is_virtual;

	// additions for full path planning A*
	// list of waypoints (including collision avoidance), all the way to the end of mission
    vector<Position> allWaypoints;
	// index represents what time step in the entire simulation the plane is at that point
	vector<Position> locationsThroughTime;
	//avoidance waypoint paths for the plane
	std::vector<map_tools::waypointPath> avoidancePaths;
	vector<Position> generateCombinedWaypoints(vector<int> &realWaypoints);

public:

	// Additions for full path planning A*
	void setAllWaypoints(vector<Position> allWaypointsIn);
	vector<Position> getAllWaypoints();
	
	void addAvoidancePath(map_tools::waypointPath wayPath);
	vector<map_tools::waypointPath> getAvoidancePaths();
	
	void setLocationsThroughTime(vector<Position> locs);
	vector<Position> * getLocationsThroughTime();
	void moveThroughTime(double bearingAtStart, bool isFirstPositionWaypoint);

    /**
     * Update a plane's current position without affecting its goal. This will
     * automatically update the plane's bearing
     *
     * TODO: Take positions as pointers rather than memory-heavy objects
     * @param current The plane's location, as might be obtained through a telemetry
     *                update
     */
    void update_current( Position current );
    
    /**
     * Update a plane's waypoint on its way to the goal. This will not affect the
     * plane's bearing.
     *
     * TODO: Take positions as pointers rather than memory-heavy objects
     * @param next The plane's intermediate waypoint, possibly a collision avoidance
     *             waypoint
     */
    void update_intermediate_wp( Position next );
    
    /**
     * It's probably a bad idea to use this function. It changes what is stored as the
     * plane's current location without affecting the plane's bearing or bearing to
     * destination. This might be useful if you wanted other aircraft to "see" this
     * plane as being somewhere else.
     *
     * TODO: Take positions as pointers rather than memory-heavy objects
     *
     * Note that this DOES set the Boolean flag current_is_virtual so that you can
     * always tell whether a plane is ACTUALLY where it says it is.
     * @param virtual_current The position to store as
     */
    void virtual_update_current( Position virtual_current );
    
    /**
     * Updates a plane's current location, its given intermediate waypoint, and its
     * speed. The plane will automatically calculate its bearing as the angle between
     * its previous, real (i.e., non-virtual) position and the new current location.
     *
     * TODO: Take positions as pointers rather than memory-heavy objects
     *
     * @param current The plane's current location, as might be obtained through a
     *                telemetry update
     * @param destination The plane's commanded "next" location; might be the same
     *                    as its final goal, or it might be an intermediate waypoint
     *                    for collision avoidance
     * @param speed The plane's speed (you could use ground speed, indicated airspeed,
     *              or true air speed... we don't actually use it for anything!)
     */
    void update(Position current, Position destination, double speed);
    
    /**
     * Sets the plane's final destination (i.e., its goal) to a given
     * latitude-longitude
     *
     * TODO: Swap lat and lon parameter order for the sake of consistency!
     * @param lon The longitude coordinate of the goal
     * @param lat The latitude coordinate of the goal
     */
	void setFinalDestination(double lon, double lat);
    
    /**
     * Sets the plane's final destination (i.e., its goal) to a given (x, y) in your
     * grid space.
     * @param x The x coordinate of the goal
     * @param y The y coordinate of the goal
     */
	void setFinalDestination(int x, int y);
    
    /**
     * Sets the plane's next destination (i.e., its intermediate waypoint, such as
     * a collision avoidance waypoint) to a given latitude-longitude
     *
     * TODO: Swap lat and lon parameter order for the sake of consistency!
     * @param lon The longitude coordinate of the "next" location
     * @param lat The latitude coordinate of the "next" location
     */
    void setDestination(double lon, double lat);
    
    /**
     * Sets the plane's next destination (i.e., its intermediate waypoint, such as
     * a collision avoidance waypoint) to a given (x, y) in your grid space.
     * @param x The x coordinate of the "next" location
     * @param y The y coordinate of the "next" location
     */
    void setDestination(int x, int y);
    
    /**
     * @return the plane's current bearing, calculated as the bearing from its previous
     *         location to its current location, with any intervening "virtual"
     *         position updates ignored
     */
	double getBearing() const;
	double setBearing(double bearingIn);
    
    /**
     * @return a discretized version of the current bearing--this is N, NE, E, &c.
     */
    map_tools::bearing_t get_named_bearing() const;
    
    /**
     * @return the bearing from the plane's current location to its FINAL destination
     *         (i.e., its goal)
     */
	double getBearingToDest() const;
    
    /**
     * @return a discretized version of the bearing to the destination--this is N, NE, E, &c.
     */
    map_tools::bearing_t get_named_bearing_to_dest() const;
    
    /**
     * @return the plane's stored speed. At the moment, this isn't used for anything.
     */
	double getSpeed();
    
    /**
     * @return the plane's ID number--if you're using this right, this number will be
     *         unique in your airspace
     */
	int getId();
    
    /**
     * @return TRUE if this plane was not created with the default, dummy constructor;
     *         FALSE otherwise
     */
    bool is_initialized();
    
    /**
     * TODO: Return a pointer instead of the whole, memory-heavy object
     *
     * @return The Position object representing plane's NEXT location (may be an
     *         intermediate, collision-avoidance waypoint or may be its final goal)
     */
	Position getDestination();
	
    /**
     * TODO: Return a pointer instead of the whole, memory-heavy object
     *
     * @return The Position object representing plane's final location (its goal)
     */
    Position getFinalDestination();
    
    /**
     * TODO: Return a pointer instead of the whole, memory-heavy object
     *
     * @return The Position object representing plane's current location
     */
	Position getLocation();
    
    /**
     * The default constructor for a plane object. Since a plane needs so much
     * information to actually be useful, you should probably never use this.
     * @param id The plane's unique ID, which defaults to -100
     */
	DiscretizedPlane(int id=-100);
    
    /**
     * The only constructor you should probably use.
     *
     * TODO: Take positions as pointers rather than memory-heavy objects
     * @param newid The unique ID number of the plane
     * @param initial The Position object representing the plane's current location
     * @param goal The Position object representing the plane's final goal
     */
    DiscretizedPlane(int newid, Position initial, Position goal );
};

// TODO definitely comment this to make it more understandable
vector<Position> DiscretizedPlane::generateCombinedWaypoints(vector<int> &realWaypoints) {
	int avoidanceIndex = 0;
	vector<Position> combined;
	Position temp = current;
	temp.setIsWaypoint(true);
	int tracker = 0;
	for (int i = 0; i < allWaypoints.size(); i++) {
		// i-1 is for if the first avoidance path is from the plane's start position to the first waypoint
		if (avoidanceIndex < avoidancePaths.size() && i-1 == avoidancePaths[avoidanceIndex].startWaypointIndex) {
			for (int j = 0; j < avoidancePaths[avoidanceIndex].pathWaypoints.size(); j++) {
				double latTemp = avoidancePaths[avoidanceIndex].pathWaypoints[j].latitude;
				double lonTemp = avoidancePaths[avoidanceIndex].pathWaypoints[j].longitude;
				temp.setLatLon(latTemp, lonTemp);
				combined.push_back(temp);
				tracker++;
			}
			avoidanceIndex++;
		}
		// add next waypoint
		temp.setLatLon(allWaypoints[i].getLat(), allWaypoints[i].getLon());
		combined.push_back(temp);
		realWaypoints.push_back(tracker);
		tracker++;
	}
	return combined;
}

#define maxTurningAngle 45.0
// @param bearingAtStart is assumed to be zero at start of simulation. This parameter can also be used
// to move the plane through time from any point in the simulation onward though
void DiscretizedPlane::moveThroughTime(double bearingAtStart, bool isFirstPositionWaypoint) {
	locationsThroughTime.clear();
	int waypointIndex = 0;
	current.setIsWaypoint(isFirstPositionWaypoint);
	locationsThroughTime.push_back(current);
	double actualBearing = bearingAtStart;
	Position currentPosition = current;
	vector<int> realWaypointIndices;
	int realIndex = 0;
	vector<Position> combined = generateCombinedWaypoints(realWaypointIndices);
	while (waypointIndex < combined.size()) {
		double lat1 = currentPosition.getLat() * MY_DEGREES_TO_RADIANS;
		double long1 = currentPosition.getLon() * MY_DEGREES_TO_RADIANS;
		double lat2 = combined[waypointIndex].getLat() * MY_DEGREES_TO_RADIANS;
		double long2 = combined[waypointIndex].getLon() * MY_DEGREES_TO_RADIANS;
		double deltaLat = lat2 - lat1;
		double deltaLong = long2 - long1;
		
		double y = sin(deltaLong)*cos(lat2);
		double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(deltaLong);
		double bearingToGoal = atan2(y, x)*MY_RADIANS_TO_DEGREES;

		//calculate the real bearing based on our maximum angle change
		//first create a temporary bearing that is the same as bearing but at a different numerical value
		double tempBearing = -1000;
		if(bearingToGoal < 0)
		{
			tempBearing = bearingToGoal + 360;
		}
		else
		{
			tempBearing = bearingToGoal - 360;
		}
		
		double diff1 = fabs(actualBearing - bearingToGoal);
		double diff2 = fabs(actualBearing - tempBearing);
	
		//check for easy to calculate values first
		if(diff1 < maxTurningAngle || diff2 < maxTurningAngle)
		{
			//the difference is less than our maximum angle, set it to the bearing
			actualBearing = bearingToGoal;
		}
		else
		{
			//we have a larger difference than we can turn, so turn our maximum
			double mod;
			if(diff1 < diff2)
			{
				if(bearingToGoal > actualBearing) mod = maxTurningAngle;
				else mod = 0 - maxTurningAngle;
			}
			else
			{
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
		double currentLatitude = lat1*MY_RADIANS_TO_DEGREES + (11.176*cos(actualBearing*MY_DEGREES_TO_RADIANS))*(1.0/111200.0);
		
		//2) Use the law of haversines to find the new longitude
		//double temp = pow(sin((MPS_SPEED/EARTH_RADIUS)/2.0), 2);
		double temp = 7.69303281*pow(10, -13); //always the same, see above calculation
		temp = temp - pow(sin((currentLatitude*MY_DEGREES_TO_RADIANS - lat1)/2.0), 2);
		temp = temp / (sin(M_PI/2.0 - lat1)*sin((M_PI/2.0)-currentLatitude*MY_DEGREES_TO_RADIANS));
		temp = 2.0 * MY_RADIANS_TO_DEGREES * asin(sqrt(temp));
		
		double currentLongitude = currentPosition.getLon();
		//depending on bearing, we should be either gaining or losing longitude
		if(actualBearing > 0)
		{
			currentLongitude += temp;
		}
		else
		{
			currentLongitude -= temp;
		}	
		
		currentPosition.setLatLon(currentLatitude, currentLongitude);
		currentPosition.setBearing(actualBearing);
		currentPosition.setIsWaypoint(false);

		// Do haversine calculations because that's what simulation does to caculate
		// if the waypoint has been reached
		lat1 = currentLatitude*MY_DEGREES_TO_RADIANS;
		//lat2 up above still applicable
		long1 = currentLongitude*MY_DEGREES_TO_RADIANS;
		//long2 above still applicable
		deltaLat = lat2 - lat1;
		deltaLong = long2 - long1;

		double a = pow(sin(deltaLat / 2.0), 2);
		a = a + cos(lat1)*cos(lat2)*pow(sin(deltaLong/2.0), 2);
		a = 2.0 * asin(sqrt(a));
		if((a * 6371000) < 30) {
			if (waypointIndex == realWaypointIndices[realIndex]) {
				currentPosition.setIsWaypoint(true);
				realIndex++;
			}
			waypointIndex++;
		}
		locationsThroughTime.push_back(currentPosition);
	}
}

void DiscretizedPlane::addAvoidancePath(map_tools::waypointPath wayPath) {
	avoidancePaths.push_back(wayPath);
}

vector<map_tools::waypointPath> DiscretizedPlane::getAvoidancePaths() {
	return avoidancePaths;
}

void DiscretizedPlane::setLocationsThroughTime(vector<Position> locs) {
	locationsThroughTime = locs;
}

vector<Position> * DiscretizedPlane::getLocationsThroughTime() {
	return &locationsThroughTime;
}

void DiscretizedPlane::setAllWaypoints(vector<Position> allWaypointsIn) {
	allWaypoints = allWaypointsIn;
}

vector<Position> DiscretizedPlane::getAllWaypoints() {
	return allWaypoints;
}

void DiscretizedPlane::update_current( Position newcurrent )
{
    // Only if the "current" location came from a telemetry update should it affect
    // our last position
    if( !current_is_virtual )
        lastPosition.setLatLon( current.getLat(), current.getLon() );
    
	current = newcurrent; //.setLatLon( newcurrent.getLat(), newcurrent.getLon() );
    current_is_virtual = false;
    
    // Find both the plane's current bearing and its bearing to its destination,
    // saving them to the proper variables
	calculateBearings();
}

void DiscretizedPlane::virtual_update_current( Position virtual_current )
{
	current = virtual_current;
    current_is_virtual = true;
    
    // Note: We do not change bearings, since this is only a "virtual" update
}

void DiscretizedPlane::update_intermediate_wp( Position next )
{
	destination = next;
    
    // NOTE: Changing the intermediate waypoint does not change our bearing
}

void DiscretizedPlane::update(Position newcurrent, Position newdestination, double newspeed)
{
    if( !current_is_virtual )
        lastPosition.setLatLon( current.getLat(), current.getLon() );
    
	current = newcurrent; //.setLatLon( newcurrent.getLat(), newcurrent.getLon() );
    current_is_virtual = false;
    
	destination = newdestination;//.setLatLon( newdestination.getLat(), newdestination.getLon() );
    
	speed=newspeed;
    
    // finds both the plane's current bearing and its bearing to its destination,
    // saving them to the proper variables
	calculateBearings();
}

void DiscretizedPlane::setFinalDestination(double lon, double lat)
{
	finalDestination.setLatLon(lat, lon);
    
    calculateBearings();
}

void DiscretizedPlane::setDestination(double lon, double lat)
{
    destination.setLatLon( lat, lon);
}

void DiscretizedPlane::setFinalDestination(int x, int y)
{
	finalDestination.setXY(x, y);
    
    calculateBearings();
}

Position DiscretizedPlane::getFinalDestination()
{
	return finalDestination;
}

void DiscretizedPlane::setDestination(int x, int y)
{
	destination.setXY(x, y);
}

double DiscretizedPlane::setBearing(double bearingIn) {
	bearing = bearingIn;
}

double DiscretizedPlane::getBearing() const
{
	return bearing;
}

map_tools::bearing_t DiscretizedPlane::get_named_bearing() const
{
    return map_tools::name_bearing( bearing );
}

double DiscretizedPlane::getBearingToDest() const
{
	return bearingToDest;
}

map_tools::bearing_t DiscretizedPlane::get_named_bearing_to_dest() const
{
    return map_tools::name_bearing( bearingToDest );
}

void DiscretizedPlane::calculateBearings()
{
    // NOTE: the map_tools::calculateBearing() fn takes lat and long in DEGREES
    double lat1 = current.getLat();
    double lon1 = current.getLon();
    
    // If our current location isn't the same as our previous location . . .
	if( !(current==lastPosition) )
	{
		//uses the same method that the simulator uses to find the planes bearing
		bearing = map_tools::calculateBearing( lastPosition.getLat(),
                                              lastPosition.getLon(),
                                              lat1, lon1 );
	}
    
    bearingToDest = map_tools::calculateBearing( lat1, lon1,
                                                finalDestination.getLat(),
                                                finalDestination.getLon() );
}

double DiscretizedPlane::getSpeed()
{
	return speed;
}

int DiscretizedPlane::getId()
{
	return id;
}

bool DiscretizedPlane::is_initialized()
{
    if( id == -1 )
        return false;
    return true;
}


Position DiscretizedPlane::getLocation()
{
	return current;
}

Position DiscretizedPlane::getDestination()
{
	return destination;
}

DiscretizedPlane::DiscretizedPlane(int newid)
{
	id=newid;
}

DiscretizedPlane::DiscretizedPlane(int newid, Position initial, Position goal )
{
    id=newid;
    current = Position(initial);
    destination = Position(goal);
    finalDestination = Position(goal);
    lastPosition = Position(initial);
    bearing = 0;
    current_is_virtual = false;
    
    calculateBearings();
}

#endif
