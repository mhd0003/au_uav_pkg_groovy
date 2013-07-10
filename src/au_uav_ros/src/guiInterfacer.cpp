#include "au_uav_ros/guiInterfacer.h"
using namespace au_uav_ros;

void guiInterfacer::run(void) {
	ros::spin();
}
/**
* This is ready to be tested but I don't know how you would be able to test this
*
* Expected Input: Valid Node handle
* Expected Ouput: None
*/
void guiInterfacer::init(ros::NodeHandle _n) {
	n = _n;
	setup();	
}

/**
* Ready to be tested.  This method instantiates all topics and services
*
* Expected Input: None
* Expected Output: None
*/

void guiInterfacer::setup(void) {
	fileNameService = n.advertiseService("send_file_path", &guiInterfacer::set_file_name, this);
	masterShutdownTopic = n.subscribe("master_shutdown", 1000, &guiInterfacer::master_shutdown, this); //TODO Latch shutdown topics
	masterAddPlaneService = n.advertiseService("master_add_plane", &guiInterfacer::add_plane, this);
	masterSetWpService = n.advertiseService("master_set_wp", &guiInterfacer::get_wp, this);
	masterRemoveWpService = n.advertiseService("master_remove_wp", &guiInterfacer::remove_wp, this);
	masterRemovePlaneService = n.advertiseService("master_remove_plane", &guiInterfacer::remove_plane, this);
	//for all components
	shutdownTopic = n.advertise<std_msgs::String>("component_shutdown", 1000, true);
	//for Coordinator
	addPlaneClient = n.serviceClient<au_uav_ros::AddPlane>("add_plane");
	setWpClient = n.serviceClient<au_uav_ros::SetWp>("set_wp");
	loadCourseClient = n.serviceClient<au_uav_ros::LoadCourse>("load_course");
	removeWpClient = n.serviceClient<au_uav_ros::RemoveWp>("remove_wp");
	removePlaneClient = n.serviceClient<au_uav_ros::RemovePlane>("remove_plane");
	//for Xbee
	initInCommsClient = n.serviceClient<au_uav_ros::InitComms>("init_incomms");
	closeInCommsClient = n.serviceClient<au_uav_ros::CloseComms>("close_incomms");
	initOutCommsClient = n.serviceClient<au_uav_ros::InitComms>("init_outcomms");
	closeOutCommsClient = n.serviceClient<au_uav_ros::CloseComms>("close_outcomms");

	//TODO I'm hard coding Xbee starups
	InitComms in, out;
	initInCommsClient.call(in);
	initOutCommsClient.call(out);
}

bool guiInterfacer::set_file_name(SendFilePath::Request &req, SendFilePath::Response &res) {
	LoadCourse loadCourse;
	loadCourse.request.wipe = true; //TODO hard coded
	loadCourse.request.filename = req.filename;
	if (loadCourseClient.call(loadCourse)) {
		res.error = "None";
		return true;
	} else {
		res.error = "Failed to call service";
		return true;
	}
}
/**
* Ready to be tested. This method shutsdown this node
*
* Expected Input: Any valid std_msgs::String type (string)
* Expected Ouput: None
*/
void guiInterfacer::master_shutdown(const std_msgs::String::ConstPtr &msg) {
	shutdownTopic.publish(msg);
	//TODO possibly wait for components to shutdown
	ros::shutdown();
}

/**
* 
*
* Expected Input: Any valid waypoint information 
* Expected Output: True if service call was succesful
*/
bool guiInterfacer::add_plane(Plane::Request &req, Plane::Response &res) {
	AddPlane addPlane;
	addPlane.request.sim = req.sim;
	addPlane.request.latitude = req.latitude;
	addPlane.request.longitude = req.longitude;
	addPlane.request.altitude = req.altitude;
	if (addPlaneClient.call(addPlane)) {
		res.error = "None";
		return true;
	} else {
		res.error = "Failed to call service";
		return false;	
	}
}

/**
* 
*
* Expected Input: Any valid planeID
* Expected Output: True if service call is succesful
*/
bool guiInterfacer::remove_plane(Plane::Request &req, Plane::Response &res) {
	RemovePlane removePlane;
	removePlane.request.sim = req.sim;
	removePlane.request.planeID = req.planeID;
	if (removePlaneClient.call(removePlane)) {
		res.error = "None";
		return true;
	} else {
		res.error = "Failed to call service";
		return false;
	}
}

/**
* 
* This function adds a new waypoint for a plane
*
* Expected Input: Any valid planeID and waypoint
* Expected Output: True if service call is succesful
*/
bool guiInterfacer::get_wp(Wp::Request &req, Wp::Response &res) {
	SetWp wp;
	wp.request.sim = req.sim;
	wp.request.planeID = req.planeID;
	wp.request.latitude = req.latitude;
	wp.request.longitude = req.longitude;
	wp.request.altitude = req.altitude;
	if (setWpClient.call(wp)) {
		res.error = "None";
		return true;
	} else {
		res.error = "Failed to call service";
		return false;
	}
}

/**
* 
* This function removes a new waypoint for a plane
*
* Expected Input: Any valid planeID and waypoint
* Expected Output: True if service call is succesful
*/
bool guiInterfacer::remove_wp(Wp::Request &req, Wp::Response &res) {
	RemoveWp wp;
	wp.request.sim = req.sim;
	wp.request.planeID = req.planeID;
	wp.request.latitude = req.latitude;
	wp.request.longitude = req.longitude;
	wp.request.altitude = req.altitude;
	if (removeWpClient.call(wp)) {
		res.error = "None";
		return true;
	} else {
		res.error = "Failed to call service";
		return false;
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "guiInterfacer");
	guiInterfacer g;
	ros::NodeHandle n;
	g.init(n);
	g.run();

	return 0;
}
