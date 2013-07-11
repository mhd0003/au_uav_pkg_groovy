#include "au_uav_ros/mapTools.h"
using namespace au_uav_ros;

map_tools::bearing_t map_tools::name_bearing(double the_bearing) {
	the_bearing = fmod(the_bearing, 360); // modular division for floats
	
	if(the_bearing > -22.5 && the_bearing <= 22.5)
		return N;
	else if(the_bearing > 22.5 && the_bearing <= 67.5)
		return NE;
	else if(the_bearing > 67.5 && the_bearing <= 112.5)
		return E;
	else if(the_bearing > 112.5 && the_bearing <= 157.5)
		return SE;
	else if(the_bearing > 157.5 && the_bearing <= 202.5)
		return S;
	else if(the_bearing > 202.5 && the_bearing <= 247.5)
		return SW;
	else if(the_bearing > 247.5 && the_bearing <= 292.5)
		return W;
	else if(the_bearing > 292.5 && the_bearing <= 337.5)
		return NW;
	else if(the_bearing > -67.5 && the_bearing <= -22.5)
		return NW;
	else if(the_bearing > -112.5 && the_bearing <= -67.5)
		return W;
	else if(the_bearing > -157.5 && the_bearing <= -112.5)
		return SW;
	else if(the_bearing > -202.5 && the_bearing <= -157.5)
		return S;
	else if(the_bearing > -247.5 && the_bearing <= -202.5)
		return SE;
	else if(the_bearing > -292.5 && the_bearing <= -247.5)
		return E;
	else if(the_bearing > -337.5 && the_bearing <= -292.5)
		return NW;
	else
	{
#ifdef ROS_ASSERT_ENABLED
		ROS_ASSERT(the_bearing > -361 && the_bearing < 361);
#endif
		return N;
	}
}

std::string map_tools::bearing_to_string(bearing_t the_bearing) {
	switch(the_bearing) {
		case N:
			return "N";
		case NE:
			return "NE";
		case E:
			return "E";
		case SE:
			return "SE";
		case S:
			return "S";
		case SW:
			return "SW";
		case W:
			return "W";
		default:
			return "NW";
	}
}

double map_tools::bearing_to_double(bearing_t the_bearing) {
	if(the_bearing == N)
		return 0.0;
	else if(the_bearing == NE)
		return 45.0;
	else if(the_bearing == E)
		return 90.0;
	else if(the_bearing == SE)
		return 135.0;
	else if(the_bearing == S)
		return 180.0;
	else if(the_bearing == SW)
		return 225.0;
	else if(the_bearing == W)
		return 270.0;
	else
		return 315.0;
}


map_tools::bearing_t map_tools::reverse_bearing(bearing_t start_bearing) {
	switch(start_bearing) {
		case N:
			return map_tools::S;
		case NE:
			return map_tools::SW;
		case E:
			return map_tools::W;
		case SE:
			return map_tools::NW;
		case S:
			return map_tools::N;
		case SW:
			return map_tools::NE;
		case W:
			return map_tools::E;
		case NW:
			return map_tools::SE;
#ifdef ROS_ASSERT_ENABLED
		default:
			ROS_ASSERT(false);
#endif
	}
}

unsigned int map_tools::find_width_in_squares(double width_of_field, double height_of_field, double map_resolution) {
	return (int)(ceil((double)(width_of_field) / map_resolution) + 0.1);
}

unsigned int map_tools::find_height_in_squares(double width_of_field, double height_of_field, double map_resolution) {
	return (int)(ceil((double)(height_of_field) / map_resolution) + 0.1);
}

double map_tools::findDistanceLikeSim(double lat1, double long1, double lat2, double long2) {
	/* Get difference in radians */
	double latDiff = (lat2 - lat1)*111200;
	double lonDiff = (long2 - long1)*93670;

	/* Return result in meters */
	return sqrt(pow(latDiff, 2) + pow(lonDiff, 2));
}


double map_tools::calculate_distance_between_points(double latitude_1, double longitude_1, double latitude_2, double longitude_2, string units) {
	double the_distance;
	double d_lat = to_radians(latitude_2 - latitude_1);
	double d_long = to_radians(longitude_2 - longitude_1);
	double sin_d_lat = sin(d_lat / 2);
	double sin_d_long = sin(d_long / 2);
	double a = (sin_d_lat * sin_d_lat +
				cos(to_radians(latitude_1)) * cos(to_radians(latitude_2)) *
				sin_d_long * sin_d_long);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	
	the_distance = fabs(earth_radius * c); // make sure it's positive
	
#ifdef ROS_ASSERT_ENABLED
	if(the_distance > 100000)
	{
		ROS_ERROR_STREAM("You broke the distance calc. You gave us these coordinates:");
		ROS_ERROR_STREAM("Point 1: " << latitude_1 << ", " << longitude_1);
		ROS_ERROR_STREAM("Point 2: " << latitude_2 << ", " << longitude_2);
	}
	ROS_ASSERT(the_distance < 100000);
#endif
	
	if(units == "feet")
		return the_distance * 3.28083989501312;
	if(units == "yards")
		return the_distance * 3.28083989501312 / 3;
	if(units == "miles")
		return the_distance * 3.28083989501312 / 5280;
	if(units == "kilometers")
		return the_distance / 1000;
	if(units == "attoparsecs")
		return the_distance * 32.4077649;
	else
		return the_distance;
}

void map_tools::calculate_point(double latitude_1, double longitude_1, double distance_in_meters, double bearing_in_deg, double & out_latitude_2, double & out_longitude_2) {
	double ang_dist_in_rad = (distance_in_meters / earth_radius);
	double bearing_in_rad = to_radians(bearing_in_deg);
	
	latitude_1 = to_radians(latitude_1);
	longitude_1 = to_radians(longitude_1);
	out_latitude_2 = to_radians(out_latitude_2);
	
	out_latitude_2 = asin((sin(latitude_1) * cos(ang_dist_in_rad)) +
						  (cos(latitude_1) * sin(ang_dist_in_rad) *
						   cos(bearing_in_rad)));
	
	out_longitude_2 = to_radians(out_longitude_2);
	
	out_longitude_2 = longitude_1 +
	atan2(sin(bearing_in_rad) * sin(ang_dist_in_rad) * cos(latitude_1),
		  cos(ang_dist_in_rad)- (sin(latitude_1) * sin(out_latitude_2)));
	
	out_longitude_2 *= RADIANS_TO_DEGREES;
	out_latitude_2 *= RADIANS_TO_DEGREES;
}

double map_tools::calculateBearing(double latitude_1, double longitude_1, double latitude_2, double longitude_2) {
	latitude_1 = to_radians(latitude_1);
	latitude_2 = to_radians(latitude_2);
	longitude_1 = to_radians(longitude_1);
	longitude_2 = to_radians(longitude_2);
	
	double deltalon=longitude_2 - longitude_1;
	
	double y = sin(deltalon)*cos(latitude_2);
	double x = cos(latitude_1) * sin(latitude_2) - sin(latitude_1) * cos(latitude_2) * cos(deltalon);
	return atan2(y, x)*RADIANS_TO_DEGREES;
}

double map_tools::calculate_bearing_in_rad(double latitude_1, double longitude_1, double latitude_2, double longitude_2) {
	latitude_1 = to_radians(latitude_1);
	latitude_2 = to_radians(latitude_2);
	longitude_1 = to_radians(longitude_1);
	longitude_2 = to_radians(longitude_2);
	
	double deltalon = longitude_2 - longitude_1;
	
	double y = sin(deltalon)*cos(latitude_2);
	double x = cos(latitude_1) * sin(latitude_2) - sin(latitude_1) * cos(latitude_2) * cos(deltalon);
	return atan2(y, x);
}

double map_tools::calculate_euclidean_bearing(int x_1, int y_1, int x_2, int y_2) {
	int d_y = y_2 - y_1;
	int d_x = x_2 - x_1;
	return atan2(d_y, d_x)*RADIANS_TO_DEGREES + 90;
}


double map_tools::to_radians(double angle_in_degrees)
{
	return (angle_in_degrees * DEGREES_TO_RADIANS);
}

double map_tools::get_euclidean_dist_between(int x_1, int y_1, int x_2, int y_2) {
	return sqrt((x_2 - x_1)*(x_2 - x_1) + (y_2 - y_1)*(y_2 - y_1));
}
