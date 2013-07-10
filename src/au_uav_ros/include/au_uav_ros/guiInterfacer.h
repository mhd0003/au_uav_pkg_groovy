#ifndef _GUI_INTERFACER_H
#define _GUI_INTERFACER_H

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include "au_uav_ros/AddPlane.h"
#include "au_uav_ros/CloseComms.h"
#include "au_uav_ros/InitComms.h"
#include "au_uav_ros/LoadCourse.h"
#include "au_uav_ros/Plane.h"
#include "au_uav_ros/Wp.h"
#include "au_uav_ros/RemovePlane.h"
#include "au_uav_ros/RemoveWp.h"
#include "au_uav_ros/SendFilePath.h"
#include "au_uav_ros/SetWp.h"

namespace au_uav_ros {
	class guiInterfacer {
	private:
		//Services and Topics sent from GUI
		ros::ServiceServer fileNameService;
		ros::Subscriber masterShutdownTopic;
		ros::ServiceServer masterAddPlaneService;
		ros::ServiceServer masterSetWpService;
		ros::ServiceServer masterRemoveWpService;
		ros::ServiceServer masterRemovePlaneService;

		//Services and topic for all components
		ros::Publisher shutdownTopic;

		//Services and Topics sent to Coordinator
		ros::ServiceClient addPlaneClient;
		ros::ServiceClient setWpClient;
		ros::ServiceClient loadCourseClient;
		ros::ServiceClient removeWpClient;
		ros::ServiceClient removePlaneClient;

		//Services and Topics sent to Xbee
		ros::ServiceClient initInCommsClient;
		ros::ServiceClient initOutCommsClient;
		ros::ServiceClient closeInCommsClient;
		ros::ServiceClient closeOutCommsClient;

		//guiInterfacer interals
		ros::NodeHandle n;

	public:
		void run(void);
		void init(ros::NodeHandle _n);
		void setup(void);
		bool set_file_name(SendFilePath::Request &req, SendFilePath::Response &res);
		void master_shutdown(const std_msgs::String::ConstPtr &msg);
		bool add_plane(Plane::Request &req, Plane::Response &res);
		bool get_wp(Wp::Request &req, Wp::Response &res);
		bool remove_wp(Wp::Request &req, Wp::Response &res);
		bool remove_plane(Plane::Request &req, Plane::Response &res);
	};		
}
#endif
