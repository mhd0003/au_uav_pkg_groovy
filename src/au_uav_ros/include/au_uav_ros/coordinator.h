#ifndef _COORDINATOR_
#define _COORDINATOR_

#include <bitset>
#include <string>
#include <vector>

//ros includes
#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"

#include "au_uav_ros/collisionAvoidance.h"
#include "au_uav_ros/planeObject.h"
#include "au_uav_ros/planeBuilder.h"
#include "au_uav_ros/simPlaneObject.h"
#include "au_uav_ros/standardDefs.h"

//service includes
#include "au_uav_ros/AddPlane.h"
#include "au_uav_ros/Centralize.h"
#include "au_uav_ros/LoadCourse.h"
#include "au_uav_ros/RemovePlane.h"
#include "au_uav_ros/RemoveWp.h"
#include "au_uav_ros/SetWp.h"
#include "au_uav_ros/SimPlane.h"

//message includes
#include "au_uav_ros/Command.h"
#include "au_uav_ros/Telemetry.h"


namespace au_uav_ros {
	class Coordinator {
	private:
		// md
		ros::Timer planPathTimer;
		/*
		* Publish to this topic to shutdown the system
		* Topic Name: "component_shutdown"
		*
		*/
		ros::Subscriber shutdownTopic;

		/*
		* Call this service to add a plane
		* Service Name: "add_plane"
		* see AddPlane.srv for paramters	
		*/
		ros::ServiceServer addPlaneService;

		/*
		* Call this service to add a waypoint to a plane
		* Service Name: "set_wp"
		* see SetWp.srv for paramters	
		*/
		ros::ServiceServer setWpService;

		/*
		* Call this service to load a course file
		* Service Name: "load_course"
		* see LoadCourse.srv for paramters	
		*/
		ros::ServiceServer loadCourseService;

		/*
		* Call this service to remove a waypoint from a plane
		* Service Name: "remove_wp"
		* see RemoveWaypoint.srv for paramters	
		*/
		ros::ServiceServer removeWpService;
		
		/*
		* Call this service to remove a plane
		* Service Name: "remove_plane"
		* see RemovePlane.srv for paramters	
		*/
		ros::ServiceServer removePlaneService; 
	
		/*
		* Call this service to set/clear centralize flag
		* Service Name: "centralize"
		* see Centralize.srv for paramters	
		*/
		ros::ServiceServer centralizeService;

		//Service and topics to simulator
		ros::ServiceClient manageSimPlanesClient;

		//Broadcasted topics
		ros::Subscriber telemetryTopic;
		ros::Publisher commandTopic;

		//coorindator internals
		ros::NodeHandle n;
		CollisionAvoidance ca;
		friend class PlaneBuilder;
		std::map<int, au_uav_ros::PlaneObject> planes;
		std::map<int, au_uav_ros::SimPlaneObject> simPlanes;
		std::list<int> newPlanes; //ids of real planes that have been loaded into system but have not been set in the air
		bool centralized;
		bool planPath;

		// used for A*
		std::bitset<255> planeUpdated;
		std::bitset<255> planeAvoiding;
		
	public:
		void run(void);
		void init(ros::NodeHandle _n);
		void setup(void);
		// md
		void plan(const ros::TimerEvent& e);
		void component_shutdown(const std_msgs::String::ConstPtr &msg);
		bool add_plane(AddPlane::Request &req, AddPlane::Response &res);
		bool set_wp(SetWp::Request &req, SetWp::Response &res);
		bool load_course(LoadCourse::Request &req, LoadCourse::Response &res);
		bool remove_wp(RemoveWp::Request &req, RemoveWp::Response &res);
		bool remove_plane(RemovePlane::Request &req, RemovePlane::Response &res);
		bool centralize(Centralize::Request &req, Centralize::Response &res);
		void telemetry(const au_uav_ros::Telemetry &msg);
		void resolvePlaneID(const au_uav_ros::Telemetry &msg);
	}; 
}
#endif
