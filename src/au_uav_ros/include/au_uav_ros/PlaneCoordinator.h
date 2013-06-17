/*
PlaneCoordinator
Class responsible for storing:
A) The latest update from the plane
B) The waypoints to go to (aka path)
C) Any collision avoidance waypoints
*/

#ifndef PLANE_COORDINATOR_H
#define PLANE_COORDINATOR_H

//normal headers
#include <stdio.h>
#include <list>
#include <string>

//ROS headers
#include "au_uav_ros/standardDefs.h"
#include "au_uav_ros/TelemetryUpdate.h"
#include "au_uav_ros/Command.h"

namespace au_uav_ros
{
	class PlaneCoordinator
	{
	private:
		//the most recent update received from a plane
		au_uav_ros::TelemetryUpdate latestUpdate;
		
		//two queues related to where the plane should go next, note that avoidance takes priority
		std::list<struct au_uav_ros::waypoint> normalPath;
		std::list<struct au_uav_ros::waypoint> avoidancePath;
		
		//index of the next command to send, starts at 0
		int commandIndex;
		
	public:
		//this is set if the coordinator has allocated this UAV to somewhere
		bool isActive;
		
		//constructors
		PlaneCoordinator();
		
		//command related functions
		bool goToPoint(struct au_uav_ros::waypoint receivedPoint, bool isAvoidanceManeuver, bool isNewQueue);
		struct au_uav_ros::waypoint getWaypointOfQueue(bool isAvoidanceQueue, int position);
		au_uav_ros::Command getPriorityCommand();
		
		//update related functions
		bool handleNewUpdate(au_uav_ros::TelemetryUpdate update, au_uav_ros::Command *newCommand);
		int getNormalSize();
	};
}

#endif
