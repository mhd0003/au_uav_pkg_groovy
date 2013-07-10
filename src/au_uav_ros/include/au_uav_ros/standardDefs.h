/*
standardDefs.h
This file is meant to contain things that are used across multiple executables that don't change
*/

#ifndef _STANDARD_DEFS_H_
#define _STANDARD_DEFS_H_

#include <math.h>
#include "boost/thread.hpp"

//12 meters - set to this because simulator makes 11 meter jumps
#define COLLISION_THRESHOLD 12 //meters
#define CONFLICT_THRESHOLD 24 //meters
#define WAYPOINT_THRESHOLD 30

/*UAV SPECIFIC DEFINES*/
//25 mph = 11.17600 meters / second
#define MPS_SPEED 11.176 
#define MPH_SPEED 25

/*
Many defines for simulator calculations
*/
#define EARTH_RADIUS 6371000 //meters

//pi radians = 180 degrees
#define DEGREES_TO_RADIANS (M_PI/180.0)
#define RADIANS_TO_DEGREES (180.0/M_PI)

//1 degree latitude ~= 111.2 km
#define LATITUDE_TO_METERS (111200.0)
#define METERS_TO_LATITUDE (1.0/111200.0)

/*
Command Types for Xbee mavlink command types
*/
#define COMMAND_NORMAL_WP 1
#define COMMAND_AVOID_WP 2
#define COMMAND_SET_ID 3

namespace au_uav_ros
{
	//Here's our standard waypoint definition
	struct waypoint
	{
		double latitude;
		double longitude;
		double altitude;
		int planeID;

		waypoint(void) {
			latitude = -1000.0;
			longitude = -1000.0;
			altitude = -1000.0;
			planeID = -1;
		}
	};

	//Mutex to lock serial port read and writes
	boost::mutex serialPort;
	waypoint INVALID_WP;
}

/*
isBlankLine(...)
simple function for parsing to determine is a string is a "blank" line
*/
bool isBlankLine(char str[]);

/*
isValidYesNo(...)
returns true if the character is a 'y', 'Y', 'n', or 'N'
*/
bool isValidYesNo(char c);

/*
distanceBetween(...)
Returns the distance in meters between the two waypoints provided.  Note that it does take into account
the earth's curvature.
*/
double distanceBetween(struct au_uav_ros::waypoint first, struct au_uav_ros::waypoint second);

bool operator==(const struct au_uav_ros::waypoint &wp1, const struct au_uav_ros::waypoint &wp2);

bool operator!=(const struct au_uav_ros::waypoint &wp1, const struct au_uav_ros::waypoint &wp2);
#endif
