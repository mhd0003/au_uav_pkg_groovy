//
// Position.h
// Header file for the Position class with use in the Auburn REU program 2011
// By Thomas Crescenzi, with additions from Tyler Young
//

#ifndef _POSITION_H_
#define _POSITION_H_

#include "au_uav_ros/mapTools.h"

#ifndef EPSILON
#define EPSILON 0.000001
#endif

#ifndef RADIAN_CONSTANTS
#define RADIAN_CONSTANTS
const double MY_PI = 4.0*atan(1.0);
const double TWO_MY_PI = 2.0*MY_PI;
const double RADIANS_TO_DEGREES = 180.0/MY_PI;//Conversion factor from Radians to Degrees
const double DEGREES_TO_RADIANS = MY_PI/180.0;//Conversion factor from Degrees to Radians
#endif

namespace au_uav_ros {
    class Position
    {
    private:
    	//position within the grid
    	int x;//long
    	int y;//lat
        double decimal_x; // decimal version of the x and y coordinates
        double decimal_y;
    	double resolution; // meters in a grid square(meters/square)
    	//position on the earth
    	double lat;//y
    	double lon;//x
    	double altidue;//just there for kicks
    	//size of the grid or matrix
    	int w;// number of squares wide
    	int h;// squares high
    	//size of the "earth"
    	//this part is kinda an issue for now the position class is built around a top left corner of the area it is contained within
    	double top_left_lat;//the value of the latitude of the top left corner
    	double top_left_long;//same as top_left_lat but for longitude
        double top_right_lat;
     	double top_right_long;
        double btm_left_lat;
     	double btm_left_long;
    	double latWidth;
    	double lonWidth;
     	double width_in_meters;
     	double height_in_meters;
        
    	//functions for converting between systems
    	/**
         A function that converts from latitude and longitude to x and y. It is called in
         setLatLon(). The latitude and longitude must be within the bounds set by setLatLon()
         input:
         @param out_x the x value to be changed. note it is an out parameter.
         @param out_y the y value to be changed. note it is an out parameter.
         **/
        void latLonToXY( int & out_x, int & out_y);
    	/**
         A function for converting between latitude and longitude and decimal x and y. Decimal x and y are used by A*.
         A value of (5.5,5.5) would be the middle of the square (5,5). The latitude and longitude must be within the bounds set by setLatLon().
         input:
         @param out_x the x value to be changed. note it is an out parameter.
         @param out_y the y value to be changed. note it is an out parameter.
         **/
        void lat_lon_to_decimal_xy( double & out_x, double & out_y);
        
        
        
        // The constructor's "helper"; used to set the width and height of the grid in
        // which the position exists.
        void set_up_grid_parms();

    	// Astar plan variables
    	bool isWaypoint;
    	double bearing;
        
    public:
    	// Astar plan functions
    	void setIsWaypoint(bool isWay);
    	bool getIsWaypoint() const;
    	void setBearing(double bear);
    	double getBearing() const;


    	/**
         A function used in setXY() to set the lat and longitude values.
         input:
         @param out_lon the latitude value to be changed. note it is an out parameter.
         @param out_lon the longitude value to be changed. note it is an out parameter.
         **/
        void xy_to_latlon( double & out_lat, double & out_lon );
    	/**
         A function that sets the x and y values of the position. They MUST be set together, if the Earth is round.
         If it happens that in the future the Earth becomes flat most of this code would be unusable and would have to be
         replaced with Euclidian equations. After setting the xy the latlon is then set as well to keep everything up to date.
         input:
         @param x1 the x value to be set. It must be >=0 and <w
         @param y1 the y value to be set. It must be >=0 and <h
         **/
        void setXY(int x1, int y1);
    	/**
         A function used to set the latitude and longitude. As with setXY() they must be set together unless someone flattens the earth.
         The x and y values are also set with this function
         input:
         @param latitude the latitude to be set. must be >=top_left_lat and <top_left_lat+latWidth
         @param longitude the latitude to be set. must be >=top_left_lon and <top_left_long+lonWidth
         **/
        void setLatLon( double latitude, double longitude );
        
    	//the preceding methods can be used interchangebly as setXY also sets the longitude/latidue and so forth
    	//note there will be some error if using x and y mainly as they only can convert to
        //the longitude and latitude of the top left of their block
    	
    	/**
         Getter for the latitude. Returns a value in decimal degrees.
         output:
         @return the latitude of the position in decimal degrees.
         **/
    	double getLat() const;
    	/**
         Getter for the longitude. Returns a value in decimal degrees.
         output:
         @return the longitude of the position in decimal degrees.
         **/
    	double getLon() const;
    	/**
         Getter for the x. Returns a value of a grid position.
         output:
         @return the x.
         **/
    	int getX();
    	/**
         Getter for the y. Returns a value of a grid position.
         output:
         @return the y.
         **/
    	int getY();
    	/**
         Getter for the decimal x. Returns a value of a decimal grid position.
         output:
         @return the decimal x.
         **/
        double getDecimalX() const;
    	/**
         Getter for the decimal y. Returns a value of a decimal grid position.
         output:
         @return the decimal y.
         **/
        double getDecimalY() const;
    	/**
         Getter for the width in squares aka the number of xs.
         output:
         @return the width of the grid, in grid squares
         **/
    	int getWidth();
    	/**
         Getter for the height in squares aka the number of ys.
         output:
         @return the height of the grid, in grid squares
         **/
    	int getHeight();
    	/**
         Getter for the top longitude left of the field in which the position exists.
         output:
         @return the top left longitude
         **/
    	double getUpperLeftLongitude();
    	/**
         Getter for the top latitude left of the field in which the position exists.
         output:
         @return the top left latitude
         **/
        double getUpperLeftLatitude();
    	/**
         An overloaded == operator. Two points are equal if they have the same decimal x and y.
         **/
    	bool operator==(Position &equal);
        
    	/**
         A default constructor for whenever a default might be needed.
         **/
        Position(double upperLeftLongitude=0.0, double upperLeftLatitude=0.0,
                 double lonwidth=0.0, double latwidth=0.0);
    	/**
         A constructor to be used when making a plane with latitude and longitude.
         input:
         @param upperLeftLongitude the upper left longitude of the field to be used in decimal degrees
         @param upperLeftLatitude the upper left latitude of the field to be used in decimal degrees
         @param lonwidth the width of the field in longitude in decimal degrees
         @param latwidth the width of the field in latitude in decimal degrees.
         @param longitude the longitude at which this position is being constructed
         @param latitude the latitude at which this position in being constructed
         @resolution_to_use the resolution to use for the grid in meters per square
         **/
    	Position(double upperLeftLongitude, double upperLeftLatitude, double lonwidth,
                 double latwidth, double longitude, double latitude, double resolution_to_use);
        /**
         A constructor to be used when making a plane with x and y.
         input:
         @param upperLeftLongitude the upper left longitude of the field to be used in decimal degrees
         @param upperLeftLatitude the upper left latitude of the field to be used in decimal degrees
         @param lonwidth the width of the field in longitude in decimal degrees
         @param latwidth the width of the field in latitude in decimal degrees.
         @param x the x at which this position is being constructed
         @param y the y at which this position in being constructed
         @resolution_to_use the resolution to use for the grid in meters per square
         **/
    	Position(double upperLeftLongitude, double upperLeftLatitude, double lonwidth, double latwidth, int x, int y, double resolution);


    	Position(map_tools::gridValues gridVals, double latitude, double longitude);
    	Position(map_tools::gridValues gridVals, int x1, int y1);
    };
};

#endif
