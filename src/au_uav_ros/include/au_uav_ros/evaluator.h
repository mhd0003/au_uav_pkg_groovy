#ifndef _EVALUATOR_H_
#define _EVALUATOR_H_

#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <queue>

#include <signal.h>
#include <unistd.h>

//ros includes
#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"

#include "au_uav_ros/standardDefs.h"

//service includes
#include "au_uav_ros/AddPlane.h"
#include "au_uav_ros/LoadCourse.h"
#include "au_uav_ros/RemovePlane.h"

//message includes
#include "au_uav_ros/Telemetry.h"

//USER DEFINED EVALUATION SETTINGS
//#define TIME_LIMIT 180 //10 minutes
#define WAYPOINT_SCORE 5 //5 points for each waypoint reached
#define CONFLICT_SCORE -1 //-1 point for each conflict during flight


#define DONE_FILE "doneWithCourse.txt"
#define ALL_SCORES_ENDING "_scores.txt"

// #define PID_FILE_FULL "/home/phil/fuerte_workspace/sandbox/AU_UAV_stack/au_uav_ros/courses/pid.txt"
#define PID_FILE_FULL "/home/monzy/catkin_ws/src/au_uav_ros/courses/pid.txt"

namespace au_uav_ros {
	class Evaluator {
	public:
		void run(void);
		void init(ros::NodeHandle _n);
		void setup(void);
		void shutdown(void);
		void component_shutdown(const std_msgs::String::ConstPtr &msg);
		void displayOutput(void);
		void endEvaluation(void);
		bool createCourseUAVs(std::string filename);
		void telemetry(const au_uav_ros::Telemetry &msg);


	private:
		ros::Publisher shutdownTopic;
		// ros::Subscriber shutdownTopic;
		ros::Subscriber telemetryTopic;

		ros::ServiceClient addPlaneClient;
		ros::ServiceClient loadCourseClient;
		ros::ServiceClient removePlaneClient;
		// ros::ServiceClient saveFlightDataClient;

		ros::NodeHandle n;
		ros::Time startTime;
		ros::Duration delta;

		int numAvoids;
		int TIME_LIMIT;

		std::map<int, au_uav_ros::Telemetry> previousUpdatesMap;
		std::map<int, au_uav_ros::Telemetry> latestUpdatesMap;
		std::map<int, std::queue<au_uav_ros::waypoint> > waypointQueues;
		std::map<int, std::list<au_uav_ros::waypoint> > waypointLists;
		std::map<int,au_uav_ros::waypoint> lastAvoidWaypoints;
		std::map<int, bool> isDead;
		std::map<int, ros::Duration> timeOfDeath;
		std::map<int, double> distanceTraveled;
		std::map<int, double> waypointDistTraveled;//this value gets changed only when we reach a new waypoint
		std::map<int, double> distSinceLastWP;//this stores the distance traveled since our last waypoint
		std::map<int, int> waypointsAchieved;
		std::map<int, double> minimumTravelDistance;
		std::map<int, double> waypointMinTravelDist;//this is distance from the last waypoint to the current

		//list for deletion
		std::queue<int> planesToDelete;

		//overall data
		int waypointsTotal;
		int numConflicts;
		int numCollisions;
		int deadPlaneCount;
		float totalDistTraveled;
		float totalMinDist;
		int score;

		//store the "last" ID
		int lastPlaneID;
		int maxAlivePlane;

		//file to store our final score sheet
		char scoresheetFilename[256];
	};
};

#endif
