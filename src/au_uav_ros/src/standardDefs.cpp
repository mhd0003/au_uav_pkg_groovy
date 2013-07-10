
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "au_uav_ros/standardDefs.h"

/*
isBlankLine(...)
simple function for parsing to determine is a string is a "blank" line
*/
bool isBlankLine(char str[])
{
	for(int i = 0; i < strlen(str); i++)
	{
		switch(str[i])
		{
			case ' ':
			case '\n':
			case '\t':
			{
				//keep checking
				break;
			}
			default:
			{
				//not a blank line character
				return false;
				break;
			}
		}
	}
	
	//we made it here, must be blank
	return true;
}

/*
isValidYesNo(...)
returns true if the character is a 'y', 'Y', 'n', or 'N'
*/
bool isValidYesNo(char c)
{
	c = tolower(c);
	if(c == 'y' || c == 'n') return true;
	else return false;
}

/*
distanceBetween(...)
Returns the distance in meters between the two waypoints provided.  Note that it does take into account
the earth's curvature.
*/
double distanceBetween(struct au_uav_ros::waypoint first, struct au_uav_ros::waypoint second)
{
	//difference in latitudes in radians
	double lat1 = first.latitude*DEGREES_TO_RADIANS;
	double lat2 = second.latitude*DEGREES_TO_RADIANS;
	double long1 = first.longitude*DEGREES_TO_RADIANS;
	double long2 = second.longitude*DEGREES_TO_RADIANS;
	
	double deltaLat = lat2 - lat1;
	double deltaLong = long2 - long1;
	
	//haversine crazy math, should probably be verified further beyond basic testing
	//calculate distance from current position to destination
	double a = pow(sin(deltaLat / 2.0), 2);
	a = a + cos(lat1)*cos(lat2)*pow(sin(deltaLong/2.0), 2);
	a = 2.0 * asin(sqrt(a));
	
	return EARTH_RADIUS * a;
}

bool operator==(const struct au_uav_ros::waypoint &wp1, const struct au_uav_ros::waypoint &wp2) {
	return ((wp1.altitude == wp2.altitude) && (wp1.longitude == wp2.longitude) && (wp1.latitude == wp2.latitude));
}

bool operator!=(const struct au_uav_ros::waypoint &wp1, const struct au_uav_ros::waypoint &wp2) {
	return ( (wp1.longitude != wp2.longitude) || (wp1.latitude != wp2.latitude) || (wp1.altitude != wp2.altitude) );
}
