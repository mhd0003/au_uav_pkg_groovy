#include "au_uav_ros/positon.h"
using namespace au_uav_ros;

// astar plan function implementations
void Position::setIsWaypoint(bool isWay) {
	isWaypoint = isWay;
}

bool Position::getIsWaypoint() const {
	return isWaypoint;
}

void Position::setBearing(double bear) {
	bearing = bear;
}

double Position::getBearing() const {
	return bearing;
}


bool Position::operator==(Position &rhs) {
	return ( decimal_y - rhs.getDecimalY() < EPSILON &&
			decimal_y - rhs.getDecimalY() > -EPSILON &&
			decimal_x - rhs.getDecimalX() < EPSILON &&
			decimal_x - rhs.getDecimalX() > -EPSILON );
}

double Position::getLat() const {
	return lat;
}

double Position::getLon() const {
	return lon;
}

void Position::setLatLon( double latitude, double longitude ) {
	lon = longitude;
	lat = latitude;
	latLonToXY( x, y );
	lat_lon_to_decimal_xy( decimal_x, decimal_y );
}

// TODO FIX THIS LOGIC CRAP BY MAKING GRID BETTER
int Position::getX() {
	return x;
}

int Position::getY() {
	return y;
}

double Position::getDecimalX() const {
	return decimal_x;
}

double Position::getDecimalY() const {
	return decimal_y;
}

void Position::setXY(int x1, int y1) {
#ifdef ROS_ASSERT_ENABLED
	ROS_ASSERT( y1 >= 0 && y1 < h );
	ROS_ASSERT( x1 >= 0 && x1 < w );
#endif
	
	x=x1;
	y=y1;
	decimal_x = x1;
	decimal_y = y1;
	
	xy_to_latlon( lat, lon );
	
}
Position::Position(double upperLeftLongitude, double upperLeftLatitude, double lonwidth, double latwidth) {
	top_left_long=upperLeftLongitude;
	top_left_lat=upperLeftLatitude;
	lonWidth=lonwidth;
	latWidth=latwidth;
	resolution = -1.0;
	
	set_up_grid_parms();
}

Position::Position(double upperLeftLongitude, double upperLeftLatitude, double lonwidth, double latwidth,
					double longitude, double latitude, double resolution_to_use) {
	top_left_long=upperLeftLongitude;
	top_left_lat=upperLeftLatitude;
	lonWidth=lonwidth;
	latWidth=latwidth;
	
	resolution = resolution_to_use;
	
	set_up_grid_parms();
	
#ifdef ROS_ASSERT_ENABLED
	ROS_ASSERT( (int)latitude != 0 && (int)longitude != 0 );
#endif
	setLatLon( latitude, longitude );
}

Position::Position(double upperLeftLongitude, double upperLeftLatitude, double lonwidth, double latwidth, int x1, int y1, double res) {
	top_left_long=upperLeftLongitude;
	top_left_lat=upperLeftLatitude;
	lonWidth=lonwidth;
	latWidth=latwidth;
	
	resolution = res;
	lat=0;
	lon=0;
	set_up_grid_parms();
	setXY(x1,y1); // changes latitude and longitude
}

Position::Position(map_tools::gridValues gridVals, double latitude, double longitude) {
	top_left_long= gridVals.upperLeftLongitude;
	top_left_lat = gridVals.upperLeftLatitude;
	lonWidth = gridVals.longitudeWidth;
	latWidth = gridVals.latitudeWidth;
	resolution = gridVals.resolution;
	
	set_up_grid_parms();
	
	setLatLon( latitude, longitude );

}

Position::Position(map_tools::gridValues gridVals, int x1, int y1) {
	top_left_long= gridVals.upperLeftLongitude;
	top_left_lat = gridVals.upperLeftLatitude;
	lonWidth = gridVals.longitudeWidth;
	latWidth = gridVals.latitudeWidth;
	resolution = gridVals.resolution;

	lat=0;
	lon=0;
	set_up_grid_parms();
	setXY(x1,y1); // changes latitude and longitude

}

void Position::xy_to_latlon( double & out_lat, double & out_lon ) {
	double d_from_origin_to_pt = resolution * sqrt( x*x + y*y );
	double bearing_between_pts;
	
	if( d_from_origin_to_pt == 0 )
		bearing_between_pts = 0;
	else
		bearing_between_pts = 90 + (RADIANS_TO_DEGREES * asin( y*resolution / d_from_origin_to_pt ));
	
	map_tools::calculate_point( top_left_lat, top_left_long,
							   d_from_origin_to_pt, bearing_between_pts,
							   out_lat, out_lon );
	
	out_lat += (latWidth / (2 * getHeight() ) );
	out_lon += (lonWidth / (2 * getWidth() ) );
#ifdef ROS_ASSERT_ENABLED
	ROS_ASSERT( d_from_origin_to_pt > -EPSILON ); // non-negative
#endif
}

void Position::latLonToXY( int & out_x, int & out_y) {
	double d_from_origin = map_tools::calculate_distance_between_points(
																		top_left_lat, top_left_long,
																		lat, lon, "meters");
	double bearing; // in radians!
	
	if( d_from_origin > -EPSILON && d_from_origin < EPSILON )
	{
		bearing = 0;
	}
	else
	{
		bearing = map_tools::calculate_bearing_in_rad( top_left_lat, top_left_long,
													  lat, lon );
		
		if( bearing > 0 )
		{
			if( bearing < MY_PI/2 )
			{
				bearing -= MY_PI/2;
			}
			else
				bearing = MY_PI/2 - bearing;
		}
		bearing = fmod( bearing, MY_PI/2 );
	}
	
#ifdef ROS_ASSERT_ENABLED
	if( bearing > 0.001 )
	{
		ROS_ERROR("That bearing of " << bearing << "is gonna break things!");
		ROS_ERROR("Your point was (" << lat << ", " << lon << ")");
	}
	ROS_ASSERT( bearing < 0.001 );
	ROS_ASSERT( bearing > -MY_PI/2 - 0.01 );
#endif
	out_x = (int)( (int)(cos( bearing ) * d_from_origin + 0.5) / resolution );
	out_y = -(int)( (int)(sin( bearing ) * d_from_origin - 0.5) / resolution );
	
#ifdef ROS_ASSERT_ENABLED
	if( out_x >= w || out_y >= h )
	{
		ROS_ERROR("You calculated (x, y) of (" << out_x << ", " << out_y << ") from bearing ");
		ROS_ERROR(bearing*RADIANS_TO_DEGREES << " and dist from origin " << d_from_origin);
		ROS_ERROR("Does this surprise you? Your origin is " << top_left_lat << ", " << top_left_long);
	}
	ROS_ASSERT( out_x < w );
	ROS_ASSERT( out_y < h );
	ROS_ASSERT( out_x >= 0 );
	ROS_ASSERT( out_y >= 0 );
#endif
}

void Position::lat_lon_to_decimal_xy( double & out_x, double & out_y) {
	double d_from_origin = map_tools::calculate_distance_between_points(
																		top_left_lat, top_left_long,
																		lat, lon, "meters");
	double bearing; // in radians!
	
	if( d_from_origin > -EPSILON && d_from_origin < EPSILON )
	{
		bearing = 0;
	}
	else
	{
		bearing = map_tools::calculate_bearing_in_rad( top_left_lat, top_left_long,
													  lat, lon );
		
		if( bearing > 0 )
		{
			if( bearing < MY_PI/2 )
			{
				bearing -= MY_PI/2;
			}
			else
				bearing = MY_PI/2 - bearing;
		}
		bearing = fmod( bearing, MY_PI/2 );
	}
	
#ifdef ROS_ASSERT_ENABLED
	if( bearing > 0.001 )
	{
		ROS_ERROR("That bearing of " << bearing << "is gonna break things!");
		ROS_ERROR("Your point was (" << lat << ", " << lon << ")");
	}
	ROS_ASSERT( bearing < 0.001 );
	ROS_ASSERT( bearing > -MY_PI/2 - 0.01 );
#endif
	out_x = (cos( bearing ) * d_from_origin) / resolution;
	out_y = -(sin( bearing ) * d_from_origin) / resolution;
	
#ifdef ROS_ASSERT_ENABLED
	ROS_ASSERT( (int)out_x < w );
	ROS_ASSERT( (int)out_y < h );
	ROS_ASSERT( (int)out_x >= 0 );
	ROS_ASSERT( (int)out_y >= 0 );
#endif
}

int Position::getWidth() {
	return w;
}
int Position::getHeight() {
	return h;
}
double Position::getUpperLeftLongitude() {
	return top_left_long;
}
double Position::getUpperLeftLatitude() {
	return top_left_lat;
}

void Position::set_up_grid_parms() {
	if ( resolution < 1 ) // wasn't set in constructor, so . . .
		resolution = 10.0; // default to a grid with 10 meter squares
	
	top_right_lat = top_left_lat;
	top_right_long = top_left_long + lonWidth;
	btm_left_lat = top_left_lat + latWidth;
	btm_left_long = top_left_long;
	
	width_in_meters = 
	map_tools::calculate_distance_between_points( top_left_lat, top_left_long,
												 top_right_lat, top_right_long,
												 "meters" );
	height_in_meters = 
	map_tools::calculate_distance_between_points( top_left_lat, top_left_long,
												 btm_left_lat, btm_left_long,
												 "meters" );    
	w = map_tools::find_width_in_squares( width_in_meters, height_in_meters,
										 resolution );
	h = map_tools::find_height_in_squares( width_in_meters, height_in_meters,
										  resolution );
}
