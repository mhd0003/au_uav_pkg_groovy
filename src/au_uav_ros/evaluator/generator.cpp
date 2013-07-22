/*
generator.cpp
This file is intended to generate massive amounts of test cases with simple configuration


It is now interactive, asknig the user for info via command line, with only a few
constants left predefined

Compiled with g++.
*/



#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <iostream>
#include <sstream>

//#include "ros/ros.h"
//#include "ros/package.h"
//*********************************************************************
//USER DEFINED SETTINGS - you can put as many options as you want in {}

//coordinates of top-left corner of field
#define WEST_MOST_LONGITUDE -85.490356
#define NORTH_MOST_LATITUDE 32.606573
#define MIN_ALTITUDE 400
#define MAX_ALTITUDE 400

//number of test cases for each setting
//#define COURSES_PER_SETTING 3

//number of points per plane
//#define WAYPOINTS_PER_PLANE 50

//amount of space in meters between each plane at start
#define BUFFER_SPACE 36.0 //meters

///home/phil/fuerte_workspace/sandbox/AU_UAV_stack/au_uav_ros
//output directory
#define OUTPUT_DIRECTORY "/home/monzy/catkin_ws/src/au_uav_ros/courses/"

#define COURSE_EXTENSION ".course"
// #define AUTO_EXTENSION ".txt"
#define AUTO_EXTENSION ""

//list of the number of planes to run tests on
//const int numPlanes[] = {4, 8, 16, 32};

//list of the field size lengths; {500} will create tests for 500m by 500m fields
//const int fieldSizes[] = {500, 1000}; //meters

//seed values for RNG
const int seed = time(NULL);

const char * fileName;
FILE * autoFP;


//the maximum number of courses to be generated
#define MAX_SIMULATIONS 10

//END USER DEFINED SETTINGS
//*********************************************************************

//1 degree latitude ~= 111.2 km
#define METERS_TO_LATITUDE (1.0/111200.0)
#define EARTH_RADIUS 6371000.0 //meters

#define DEGREES_TO_RADIANS (M_PI/180.0)
#define RADIANS_TO_DEGREES (180.0/M_PI)

struct waypoint
{
	double latitude;
	double longitude;
	double altitude;
};

/*
distanceBetween(...)
Returns the distance in meters between the two waypoints provided.  Note that it does take into account
the earth's curvature.
NOTE: copied from other source code
*/
double distanceBetween(struct waypoint first, struct waypoint second)
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


//writes the data to the .course file
void generateCourse(int numberOfPlanes, int numberOfWayPoints, int fieldLength, int courseNumber)
{
	//let the user know what we're doing
	//printf("Generating course #%d with %d planes and %d field length...\n", courseNumber, numberOfPlanes, fieldLength);
	
	char filename[1000];
	sprintf(filename, "%s/%s_%d_%dm_%d.course", OUTPUT_DIRECTORY,fileName, numberOfPlanes, fieldLength, courseNumber);
	//std::string outDir = ros::package::getPath("au_uav_ros")+"/courses/";
	//sprintf(filename, "%s/%s%d_%dm_%d.course", outDir.c_str(),fileName, numberOfPlanes, fieldLength, courseNumber);

	
	char baseName[1000];
	sprintf(baseName, "%s_%d_%dm_%d.course",fileName, numberOfPlanes, fieldLength, courseNumber);

	fprintf(autoFP,"%s\n",baseName);
	
	struct waypoint planeStarts[numberOfPlanes];
	
	FILE *fp;
	fp = fopen(filename, "w");
	
	if(fp == NULL)
	{
		printf("ERROR: Couldn't open file %s!\n", filename);
	}
	else
	{
		struct waypoint newPoint;
		int altitudeDifference = MAX_ALTITUDE - MIN_ALTITUDE;
		
		fprintf(fp, "#Auburn University ATTRACT Project - au_uav_ros Sub-project\n");
		fprintf(fp, "#Randomly generated course file\n");
		fprintf(fp, "#Settings:\n");
		fprintf(fp, "#\tNumber of planes: %d\n", numberOfPlanes);
		fprintf(fp, "#\tField size: %d meters by %d meters\n", fieldLength, fieldLength);
		fprintf(fp, "#\tWaypoints per plane: %d\n", numberOfWayPoints);
		fprintf(fp, "#\tNorth-west corner: (%lf, %lf)\n", NORTH_MOST_LATITUDE, WEST_MOST_LONGITUDE);
		fprintf(fp, "#\tAltitude range: %d - %d\n", MIN_ALTITUDE, MAX_ALTITUDE);
		fprintf(fp, "\n");
		fprintf(fp, "#Starting waypoints\n");
		fprintf(fp, "#Plane ID\tLatitude\tLongitude\tAltitude\n");
		for(int planeid = 0; planeid < numberOfPlanes; planeid++)
		{
			//latitude is easy to calculate
			newPoint.latitude = NORTH_MOST_LATITUDE - METERS_TO_LATITUDE*(rand()%(fieldLength+1));
			
			//longitude requires a little more stuff
			int longitudeMeters = rand()%(fieldLength+1);
			double temp = pow(sin((longitudeMeters/EARTH_RADIUS)/2.0), 2);
			temp = temp / (sin(M_PI/2.0 - newPoint.latitude*DEGREES_TO_RADIANS)*sin((M_PI/2.0)-newPoint.latitude*DEGREES_TO_RADIANS));
			newPoint.longitude = WEST_MOST_LONGITUDE + 2.0*RADIANS_TO_DEGREES*asin(sqrt(temp));
			
			//altitude is easy too
			newPoint.altitude = MIN_ALTITUDE + rand()%(altitudeDifference+1);
			
			//   SHOULD BE ABLE TO IMPORVE *******************************************************************
			//make sure our distance between each other point is enough
			bool isTooClose = false;
			for(int i = 0; i < planeid && !isTooClose; i++)
			{
				if(distanceBetween(newPoint, planeStarts[i]) < BUFFER_SPACE)
				{
					//subtract a value to put us back where we were and re-roll the dice
					isTooClose = true;
					planeid--;
				}
			}
			if(isTooClose) continue;

			//save our point for later
			planeStarts[planeid] = newPoint;
			
			//output to our file
			fprintf(fp, "%d\t\t%lf\t%lf\t%lf\n", planeid, newPoint.latitude, newPoint.longitude, newPoint.altitude);
		}
		fprintf(fp, "\n");
		
		//for each plane, create a list of values
		for(int planeid = 0; planeid < numberOfPlanes; planeid++)
		{
			//create a label
			fprintf(fp, "#Plane ID: %d\n", planeid);
		
			for(int x = 0; x < numberOfWayPoints; x++)
			{
				//latitude is easy to calculate
				newPoint.latitude = NORTH_MOST_LATITUDE - METERS_TO_LATITUDE*(rand()%(fieldLength+1));
			
				//longitude requires a little more stuff
				int longitudeMeters = rand()%(fieldLength+1);
				double temp = pow(sin((longitudeMeters/EARTH_RADIUS)/2.0), 2);
				temp = temp / (sin(M_PI/2.0 - newPoint.latitude*DEGREES_TO_RADIANS)*sin((M_PI/2.0)-newPoint.latitude*DEGREES_TO_RADIANS));
				newPoint.longitude = WEST_MOST_LONGITUDE + 2.0*RADIANS_TO_DEGREES*asin(sqrt(temp));
			
		//**** if diff == 0, bad result
				//altitude is easy too
				newPoint.altitude = MIN_ALTITUDE + rand()%(altitudeDifference+1); 
			
				//output to our file
				fprintf(fp, "%d\t\t%lf\t%lf\t%lf\n", planeid, newPoint.latitude, newPoint.longitude, newPoint.altitude);
			}
			fprintf(fp, "\n");
		}
		
		//close the file
		fclose(fp);
	}
}

//reads the next int from the command line
//returns true if it was an int, false otherwise 

//takes the int var to store the read int
//takes what to deliminate on
bool getInt(int *val,char delim){
	std::cin.clear();
	std::string in;
	std::stringstream ss1;
	int temp;
	//std::cout << "Here";

	std::getline(std::cin,in,delim);
	ss1 << in;
	ss1 >> temp;
	if(ss1.rdstate() == 4){//error
		return false;
	}
	*val = temp;
	return true;
		
}//getInt

//returns true if the user wants to enter a common value for the given variable
//and assgins that value to the given int pointer

bool needOne(std::string var,int *value){
	//std::cin.clear();
	char yn = '\0';	
	std::string input;	
	std::cout << "\nIs the number of " << var <<" the same for each simulation?[y,n]: ";
	std::getline(std::cin, input);
	//std::cin >> input;
	yn = input[0];
	if(yn == 'y'){
		std::cout << "Enter the common value: ";
		getInt(value,'\n');
		return true;
	}

	return false;
}//needOne


//populates the first n places of the given array with the given value
void populateArray(int array[], int value, int n){
	int i = 0;
	while(i<n){
		array[i++] = value;	
	}
}

int main()
{

	//changed

	std::string input;
	std::stringstream ss1;
	int numSims;
	bool prompt = false;

	//get the number of courses to generate, put it in numSims,
	//and make sure the user enters a number
	do{
		std::cout << "Enter the number of courses to generate: ";
	}while(!getInt(&numSims,'\n'));
	

	//only ask certain questions if more than 3 courses are being made
	if(numSims > 3){prompt = true;}

	//these hold one value for each simulation
	int allPlanes [numSims]; //holds the number of planes for each sim
	int allWayPoints[numSims];	//the number of waypoints for each sim
	int allFields [numSims];	//length of the square field for each sim

	//these will potentially hold values that are common for all simulations
	//if they exists
	int nPlanes = -1;		
	int nWayPoints = -1;
	int nFields = -1;

	//other
	int count = 0;	//iteration variable
	char yn = '\0';	//for response to a yes no question

	
	//see if there is a number of planes for all sims, otherwise prompt for all values
	if(prompt && needOne("planes",&nPlanes)){}//do nothing else
	else{
		std::cout << "Enter number of planes for each simulation separated by a space:\n\n ";
		while(count<numSims-1){
			//std::cin >> allPlanes[count++];
			getInt(&(allPlanes[count++]),' ');
		}
		//get the last one, delim on the newline
		getInt(&(allPlanes[count]),'\n');
		count = 0;
	}
	//std::cin.clear();


	//same thing as with the planes but with waypoints
	if(prompt && needOne("wayPoints",&nWayPoints)){}
	else{
		std::cout << "Enter number of wayPoints for each plane in each simulation separated by a space:\n ";
		std::cout << "(The number of wayPoints in a given simulation is constant fo every plane.)\n";
		while(count<numSims-1){
			getInt(&(allWayPoints[count++]),' ');
		}
		getInt(&(allWayPoints[count++]),'\n');
		count = 0;
	}

	

	//same thing with field length
	if(prompt && needOne("field length",&nFields)){}
	else{
		std::cout << "Enter length of the field for each simulation separated by a space:\n\n ";
		while(count<numSims-1){
			getInt(&(allFields[count++]),' ');
		}
		getInt(&(allFields[count++]),'\n');
		count = 0;
	}

	std::string fn;
	std::cout << "\nEnter the base name for all files: ";
	//std::cout << "(the number of planes, and size of field will be added to the name, as well as a counter)\n";
	std::getline(std::cin,fn);
	
	fileName = fn.c_str();


	char autoPathName [1000];
	sprintf(autoPathName,"%s/%s%s",OUTPUT_DIRECTORY,fileName,AUTO_EXTENSION);
	//sprintf(autoPathName,"%s/%s%s",(ros::package::getPath("au_uav_ros")+"/courses/"+autoFile+AUTOEXTENSION).c_str());
	autoFP = fopen(autoPathName,"w");



	//if a common value was entered, put that value in each position of the array
	if(nPlanes > 0){
		populateArray(allPlanes,nPlanes,numSims);
	}

	if(nWayPoints > 0){
		populateArray(allWayPoints,nWayPoints,numSims);
	}

	if(nFields > 0){
		populateArray(allFields,nFields,numSims);
	}


	//these are the predefined constants
	std::cout << "\nSettings:\n";
	std::cout << "OUTPUT DIRECTORY: " << OUTPUT_DIRECTORY <<"\n";
	std::cout << "Northwest Corner: ("<<NORTH_MOST_LATITUDE<<","<<WEST_MOST_LONGITUDE<<")\n";
	std::cout << "Plane buffer space: " << BUFFER_SPACE <<"\n";
	std::cout << "Generating...";


	//generate all the course files
	count =0;
	while(count<numSims){
		generateCourse(allPlanes[count],allWayPoints[count],allFields[count],count);
		count = count + 1;
	}


	fclose(autoFP);

}//end of main
