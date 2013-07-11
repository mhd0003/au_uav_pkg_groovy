//
//  mapTools.h
//  AU_UAV_ROS
//
//  Created by Tyler Young on 5/26/11.
//
// A utility class for the map and Position classes

#ifndef _MAP_TOOLS_H_
#define _MAP_TOOLS_H_

static const double earth_radius = 6371000; // meters, on average

#ifndef RADIAN_CONSTANTS
#define RADIAN_CONSTANTS
const double MY_PI = 4.0*atan(1.0);
const double TWO_MY_PI = 2.0*MY_PI;
//const double RADIANS_TO_DEGREES = 180.0/MY_PI; // Conversion factor from radians to degrees
//const double DEGREES_TO_RADIANS = MY_PI/180.0; // Conversion factor from degrees to radians
#endif

//TODO figure out where to put this
#define DEFAULT_PADDING 0.00075
#define DEFAULT_RESOLUTION 10

#ifndef _BEARING_T_
#define _BEARING_T_
namespace au_uav_ros {
	enum bearing_t { N, NE, E, SE, S, SW, W, NW };
};
#endif

namespace au_uav_ros {

	namespace map_tools {
		// Struct for passing around the values to create the grid for Position and dangerGrid
		struct gridValues {
			double upperLeftLatitude;
			double upperLeftLongitude;
			double latitudeWidth;
			double longitudeWidth;
			double padding;
			double resolution;
		};

		// enum bearing_t { N, NE, E, SE, S, SW, W, NW };
		
		/**
		 * Converts a bearing in degrees to a "named" version, for use in deciding which
		 * nearby squares are in the path of the aircraft
		 * NOTE: Bearings must be between -360 and 360 degrees
		 * @param the_bearing Bearing of the aircraft in degrees (0 is due north,
		 *                    90 due east, and so on)
		 * @return A named version of the direction (N for bearings -22.5 to 22.5 deg,
		 *         NE for bearings 22.5 to 67.5 deg, and so on)
		 */
		bearing_t name_bearing(double the_bearing);
		
		/**
		 * Returns a string version of the enumerated type bearing_t
		 * @param the_bearing The bearing, obtainable by using name_bearing, to convert
		 *                    to a string
		 * @return A string "N", "NE", "E", or what have you
		 */
		std::string bearing_to_string(bearing_t the_bearing);
		
		/**
		 * Returns a string version of the enumerated type bearing_t
		 * @param the_bearing The bearing, obtainable by using name_bearing, to convert
		 *                    to a string
		 * @return 0.0 for N, 22.5 for NE, 45 for E, and so on
		 */
		double bearing_to_double(bearing_t the_bearing);
		
		/**
		 * Gives the opposite of a "named" bearing; the opposite of N is S, opposite of
		 * SE is NW, and so on.
		 * @param start_bearing The bearing whose opposite will be returned
		 * @return The opposite of the starting bearing
		 */
		bearing_t reverse_bearing(bearing_t start_bearing);
		
		/**
		 * Using the width, height, and resolution (in whatever system of measurement
		 * you're using, such as meters), this returns the width of the field IN SQUARES.
		 * @param width_of_field The width of the flyable area, in meters
		 * @param height_of_field The height of the flyable area, in meters
		 * @param map_resolution The resolution (width and height of a given square),
		 *                       in meters
		 * @return The width of the map grid in grid squares; pass this in
		 *         empty and the function will assign it the proper value.
		 */
		unsigned int find_width_in_squares(double width_of_field,
										   double height_of_field,
										   double map_resolution);
		
		/**
		 * Using the width, height, and resolution (in whatever system of measurement
		 * you're using, such as meters), this returns the width of the field IN SQUARES.
		 * @param width_of_field The width of the flyable area, in meters
		 * @param height_of_field The height of the flyable area, in meters
		 * @param map_resolution The resolution (width and height of a given square),
		 *                       in meters
		 * @return The height of the map grid in grid squares; pass this in
		 *         empty and the function will assign it the proper value.
		 */
		unsigned int find_height_in_squares(double width_of_field,
											double height_of_field,
											double map_resolution);
		
		/**
		 * Uses the haversine formula to calculate the distance between two points.
		 *
		 * Returns the distance in feet, yards, meters, kilometers, or attoparsecs.
		 * @param latitude_1 The latitude (in decimal degrees) for point 1
		 * @param longitude_1 The longitude (in decimal degrees) for point 1
		 * @param latitude_2 The latitude (in decimal degrees) for point 2
		 * @param longitude_2 The longitude (in decimal degrees) for point 2
		 * @param units The units for the returned distance. Allowed values are:
		 *                   - 'feet'
		 *                   - 'yards'
		 *                   - 'miles'
		 *                   - 'meters'
		 *                   - 'kilometers'
		 *                   - 'attoparsecs'
		 *              Default value is meters.
		 * @return The distance, measured in whatever units were specified (default is meters)
		 */
		double calculate_distance_between_points(double latitude_1, double longitude_1,
												 double latitude_2, double longitude_2,
												 std::string units);
		
		/**
		 * Calculates an ending lat-long coordinate given a starting lat-long position,
		 * a distance between the two points, and a bearing from the starting point to the
		 * final point.
		 * @param latitude_1 The latitude (in decimal degrees) for point 1
		 * @param longitude_1 The longitude (in decimal degrees) for point 1
		 * @param distance_in_meters Distance between starting and ending points
		 * @param bearing_in_deg The bearing from the starting to the ending point
		 * @param out_latitude_2 The latitude (in decimal degrees) for the ending point
		 * @param out_longitude_2 The longitude (in decimal degrees) for the ending point
		 * @param
		 */
		void calculate_point(double latitude_1, double longitude_1,
							 double distance_in_meters, double bearing_in_deg,
							 double & out_latitude_2, double & out_longitude_2);
		
		/**
		 * Calculate the bearing, in degrees, between two points
		 * @param latitude_1 The latitude (in decimal degrees) for point 1
		 * @param longitude_1 The longitude (in decimal degrees) for point 1
		 * @param latitude_2 The latitude (in decimal degrees) for point 2
		 * @param longitude_2 The longitude (in decimal degrees) for point 2
		 * @return The bearing, in degrees, from point 1 to point 2
		 */
		double calculateBearing(double latitude_1, double longitude_1,
								double latitude_2, double longitude_2);
		
		/**
		 * Calculate the bearing, in radians, between two points (which themselves are
		 * given in decimal degrees)
		 * @param latitude_1 The latitude (in decimal degrees) for point 1
		 * @param longitude_1 The longitude (in decimal degrees) for point 1
		 * @param latitude_2 The latitude (in decimal degrees) for point 2
		 * @param longitude_2 The longitude (in decimal degrees) for point 2
		 * @return The bearing, in radians, from point 1 to point 2
		 */
		double calculate_bearing_in_rad(double latitude_1, double longitude_1,
										double latitude_2, double longitude_2);
		
		/**
		 * Calculate the bearing, in degrees, between two points (FROM point 1, TO point 2)
		 * @param x_1, y_1 The x and y coordinates of the first point
		 * @param x_2, y_2 The x and y coordinates of the other point
		 * @return The bearing, in degrees, from point 1 to point 2
		 */
		double calculate_euclidean_bearing(int x_1, int y_1,
										   int x_2, int y_2);
		
		/**
		 * Converts an angle, for use in the haversine formula
		 * @param angle_in_degrees The angle you wish to convert from degrees to radians
		 * @return The angle converted to radians
		 */
		double to_radians(double angle_in_degrees);
		
		/**
		 * Calculates the distance between two points in a plane using the Pythagorean
		 * theorem. Note that this will be in GRID SQUARES.
		 * @param x_1, y_1 The x and y coordinates of the first point
		 * @param x_2, y_2 The x and y coordinates of the other point
		 * @return The calculated distance between (x_1, y_1) and (x_2, y_2), in grid squares
		 */
		double get_euclidean_dist_between(int x_1, int y_1,
										  int x_2, int y_2);

		// TODO needed to match simulation calculation of distance
		double findDistanceLikeSim(double lat1, double long1, double lat2, double long2);
	};
};

#endif
