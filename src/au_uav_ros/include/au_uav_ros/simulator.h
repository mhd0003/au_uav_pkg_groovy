#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include "ros/console.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

//service includes
#include "au_uav_ros/SimPlane.h"

//message incldues
#include "au_uav_ros/Command.h"
#include "au_uav_ros/Double.h"
#include "au_uav_ros/Telemetry.h"

#include "au_uav_ros/simPlaneObject.h"

namespace au_uav_ros {
	class Simulator {
	private:
		//Services and topics from master
		ros::Subscriber shutdownTopic;
		ros::ServiceServer setFreqService;
		ros::ServiceServer setSimSpeed;

		//Services and topics from coordinator
		ros::ServiceServer managePlanesService;

		//Broadcasted topics
		ros::Publisher telemetryTopic;
		ros::Subscriber commandTopic;

		//simulator internals
		ros::NodeHandle n;
		ros::WallTimer simulateTimer;
		double simSpeed;
		std::map<int, au_uav_ros::SimPlaneObject> simPlanes;
		bool centralized;
	public:
		void run(void);
		void init(ros::NodeHandle _n);
		void setup(void);
		void component_shutdown(const std_msgs::String::ConstPtr &msg);
		bool sim_freq(au_uav_ros::Double::Request &req, au_uav_ros::Double::Response &res);
		bool sim_speed(au_uav_ros::Double::Request &req, au_uav_ros::Double::Response &res);
		bool manage_simplanes(au_uav_ros::SimPlane::Request &req, au_uav_ros::SimPlane::Response &res);
		void commands(const au_uav_ros::Command &cmd);
		void simulate(const ros::WallTimerEvent &e);
	};
}
#endif
