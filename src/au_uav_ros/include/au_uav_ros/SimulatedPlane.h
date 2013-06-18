/*
SimulatedPlane
This class is the data structures and functions required to perform plane simulation.  Each one
instantiated will be considered one "plane" in the system.
*/

#ifndef SIMULATED_PLANE_H
#define SIMULATED_PLANE_H

#define MAXIMUM_TURNING_ANGLE 22.5 //degrees

#include "au_uav_ros/standardDefs.h"
#include "au_uav_ros/Command.h"
#include "au_uav_ros/TelemetryUpdate.h"
#include "au_uav_ros/CreateSimulatedPlane.h"

namespace au_uav_ros
{
	class SimulatedPlane
	{
	private:
		//last received command info
		au_uav_ros::Command lastCommand;
		
		//current information (used mostly in update)
		long long int planeID;
		
		au_uav_ros::waypoint currentLocation;
		au_uav_ros::waypoint currentDest;
		
		//these two values are sent in the telemetry update
		double groundSpeed;
		double bearing;
		
		//this is stored as part of the UAV info
		double actualBearing;
		
		long long int currentWaypointIndex;
		double distanceToDestination;
		
		//index of sent message
		int updateIndex;
		
	public:
		//dummy constructor, shouldn't really be used
		SimulatedPlane();
		
		//primary constructor
		SimulatedPlane(long long int planeID, au_uav_ros::CreateSimulatedPlane::Request &requestFromUser);
	
		//function for handling a command from the coordinator
		bool handleNewCommand(au_uav_ros::Command newCommand);
		
		//periodic function for filling in a new telemetry update for this UAV
		bool fillTelemetryUpdate(au_uav_ros::TelemetryUpdate *tUpdate);
	};
}

#endif
