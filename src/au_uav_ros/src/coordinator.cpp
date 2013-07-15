#include "au_uav_ros/coordinator.h"
using namespace au_uav_ros;

#define COORD_PRINT_DEBUG true

void Coordinator::run(void) {
	ros::spin();
}

void Coordinator::init(ros::NodeHandle _n) {
	n = _n;
	setup();
}

void Coordinator::setup(void) {
	// Set centralized through launch file parameter
	// Default to centralized
	if (n.getParam("runCentralized", centralized)) {
		//centralized = $(arg centralized)
	} else {
		centralized = true;
	}

	if (COORD_PRINT_DEBUG) {
		if (centralized) {
			ROS_INFO("Running centralized collision avoidance!");
		}
		else {
			ROS_INFO("Running DEcentralized collision avoidance!");
		}
	}
	
	shutdownTopic = n.subscribe("component_shutdown", 1000, &Coordinator::component_shutdown, this);
	addPlaneService = n.advertiseService("add_plane", &Coordinator::add_plane, this);
	setWpService = n.advertiseService("set_wp", &Coordinator::set_wp, this);
	loadCourseService =n.advertiseService("load_course", &Coordinator::load_course, this);
	removeWpService = n.advertiseService("remove_wp", &Coordinator::remove_wp, this);
	removePlaneService = n.advertiseService("remove_plane", &Coordinator::remove_plane, this);

	manageSimPlanesClient = n.serviceClient<au_uav_ros::SimPlane>("manage_simplanes");

	telemetryTopic = n.subscribe("telemetry", 1000, &Coordinator::telemetry, this);
	commandTopic = n.advertise<au_uav_ros::Command>("commands", 1000);
}

void Coordinator::component_shutdown(const std_msgs::String::ConstPtr &msg) {
	ros::shutdown();
}

bool Coordinator::add_plane(AddPlane::Request &req, AddPlane::Response &res) {
	//one waypoint is required for a plane to be added
	struct waypoint wp;
	wp.altitude = req.altitude;
	wp.latitude = req.latitude;
	wp.longitude = req.longitude;
	PlaneBuilder b;
	if (req.sim) {
		SimPlane s;
		int id = b.buildSimPlane(wp, *this, s);
		if (id != -1) {
			manageSimPlanesClient.call(s);
			Command cmd = simPlanes[id].getPriorityCommand();
			cmd.sim = true;
			commandTopic.publish(cmd);
			return true;
		} else {
			res.error = "Failed to add new plane";
			return false;
		}
	} else {
		int id = b.buildPlane(wp, *this);
		if (id != -1) {
			Command cmd = simPlanes[id].getPriorityCommand();
			cmd.sim = false;
			commandTopic.publish(cmd);
		} else {
			res.error = "Failed to add new plane";
			return false;
		}
	}
}

bool Coordinator::set_wp(SetWp::Request &req, SetWp::Response &res) {
	/* TODO guard input */
	struct waypoint wp;
	wp.altitude = req.altitude;
	wp.latitude = req.latitude;
	wp.longitude = req.longitude;
	if (req.sim) {
		simPlanes[req.planeID].addNormalWp(wp);
		SimPlane s;
		s.request.clear = false;
		s.request.add = true;
		s.request.size = 1;
		s.request.planeID = req.planeID;
		s.request.latitudes.push_back(req.latitude);
		s.request.longitudes.push_back(req.longitude);
		s.request.altitudes.push_back(req.altitude);
		manageSimPlanesClient.call(s);
	} else {
		planes[req.planeID].addNormalWp(wp);
	}
	return true;
}

bool Coordinator::load_course(LoadCourse::Request &req, LoadCourse::Response &res) {
	if (req.wipe == true) {
		planes.clear();
		for (std::map<int, SimPlaneObject>::iterator i = simPlanes.begin(); i != simPlanes.end(); i++) {
			SimPlane msg;
			msg.request.clear = true;
			if (!manageSimPlanesClient.call(msg)) {
				res.error = "Failed to connect to simulator";
				return false;
			}
		}
		simPlanes.clear();
	}
	PlaneBuilder b;
	std::string file = req.filename.c_str();
	std::map<int , SimPlane> s;
	if (b.buildCourse(file, *this, s)) {
		//publish new commands
		for (std::map<int, PlaneObject>::iterator i = planes.begin(); i != planes.end(); i++) {
			Command cmd = i->second.getPriorityCommand();
			cmd.sim = false;
			commandTopic.publish(cmd);
		}
		for (std::map<int, SimPlane>::iterator i = s.begin(); i != s.end(); i++) {
			manageSimPlanesClient.call(i->second);
		}
		for (std::map<int, SimPlaneObject>::iterator i = simPlanes.begin(); i != simPlanes.end(); i++) {
			Command cmd = i->second.getPriorityCommand();
			cmd.sim = true;
			commandTopic.publish(cmd);
		}
		return true;
	} else {
		res.error = "Failed to load course";
		return false;
	}
}

bool Coordinator::remove_wp(RemoveWp::Request &req, RemoveWp::Response &res) {
	/* TODO guard input */
	struct waypoint wp;
	wp.altitude = req.altitude;
	wp.latitude = req.latitude;
	wp.longitude = req.longitude;
	if (req.sim) {
		SimPlane s;
		s.request.clear = false;
		s.request.planeID = req.planeID;
		s.request.add = false;
		s.request.latitudes.push_back(req.latitude);
		s.request.longitudes.push_back(req.longitude);
		s.request.altitudes.push_back(req.altitude);
		s.request.size = 1;
		manageSimPlanesClient.call(s);
		simPlanes[req.planeID].removeNormalWp(wp);
	} else {
		planes[req.planeID].removeNormalWp(wp);
	}
	return true;
}

bool Coordinator::remove_plane(RemovePlane::Request &req, RemovePlane::Response &res) {
	/* TODO guard input */
	if (req.sim) {
		SimPlane s;
		s.request.clear = false;
		s.request.planeID = req.planeID;
		s.request.size = 0;
		manageSimPlanesClient.call(s);
		simPlanes.erase(req.planeID);
	} else {
		planes.erase(req.planeID);
	}
	return true;
}

void Coordinator::telemetry(const au_uav_ros::Telemetry &msg) {
	std::vector<waypoint> avoidanceWps;
	Command cmd;
	if (planes.find(msg.planeID) != planes.end()) { /*TODO Handle ID duplicates-- map doesnt allow key duplicates */
		bool b = planes[msg.planeID].update(msg, cmd);	/* so use telem or something else to check for dup planes */
		if (b) {
			cmd.sim = false;
			commandTopic.publish(cmd);
		}
		//call collision here			/* if all planes are set a def val we can use that val to check */
		if (centralized) {
			ca.avoid(msg.planeID, planes, simPlanes, avoidanceWps);
			for (unsigned int i = 0; i < avoidanceWps.size(); i++) {
				if (planes.find(avoidanceWps[i].planeID) != planes.end()) {
					planes[avoidanceWps[i].planeID].addAvoidanceWp(avoidanceWps[i]);
				} else {
					simPlanes[avoidanceWps[i].planeID].addAvoidanceWp(avoidanceWps[i]);
				}

				Command cmd;
				cmd.planeID = avoidanceWps[i].planeID;
				if (simPlanes.find(cmd.planeID) != simPlanes.end()) {
					cmd.sim = true;
				} else {
					cmd.sim = false;
				}
				cmd.commandID = COMMAND_AVOID_WP;
				cmd.latitude = avoidanceWps[i].latitude;
				cmd.longitude = avoidanceWps[i].longitude;
				cmd.altitude = avoidanceWps[i].altitude;
				commandTopic.publish(cmd);
			}
		} else {
			// Real plane decentralized CA
			// is in the air on the Pi
		}
	} else if (simPlanes.find(msg.planeID) != simPlanes.end()) {
		bool b = simPlanes[msg.planeID].update(msg, cmd);
		if (b) {
			//This is where normal wps are sent but sims already know all their normal wps
			//cmd.sim = true;
			//commandTopic.publish(cmd);
		}
		//call collision here
		if (centralized) {
			ca.avoid(msg.planeID, planes, simPlanes, avoidanceWps);
			for (unsigned int i = 0; i < avoidanceWps.size(); i++) {
				if (planes.find(avoidanceWps[i].planeID) != planes.end()) {
					planes[avoidanceWps[i].planeID].addAvoidanceWp(avoidanceWps[i]);
				} else {
					simPlanes[avoidanceWps[i].planeID].addAvoidanceWp(avoidanceWps[i]);
				}

				Command cmd;
				cmd.planeID = avoidanceWps[i].planeID;
				if (simPlanes.find(cmd.planeID) != simPlanes.end()) {
					cmd.sim = true;
				} else {
					cmd.sim = false;
				}
				cmd.commandID = COMMAND_AVOID_WP;
				cmd.latitude = avoidanceWps[i].latitude;
				cmd.longitude = avoidanceWps[i].longitude;
				cmd.altitude = avoidanceWps[i].altitude;
				commandTopic.publish(cmd);
			}
		} else {
			// md
			// Decentralized collision avoidance here

			// md
			// TODO: Two ways to do this
			// (1) Planes run CA only right after they push out an update.
			// (2) Planes run CA every time an update is received.
			ca.distrubuted_avoid(msg.planeID, planes, simPlanes, avoidanceWps);
			int id;
			for (unsigned int i = 0; i < avoidanceWps.size(); i++) {
				id = avoidanceWps[i].planeID;
				simPlanes[id].addAvoidanceWp(avoidanceWps[i]);

				Command cmd;
				cmd.planeID = id;
				cmd.sim = true;
				cmd.commandID = COMMAND_AVOID_WP;
				cmd.latitude = avoidanceWps[i].latitude;
				cmd.longitude = avoidanceWps[i].longitude;
				cmd.altitude = avoidanceWps[i].altitude;
				commandTopic.publish(cmd);
			}
		}
	} else {
		resolvePlaneID(msg);
	}
}

void Coordinator::resolvePlaneID(const Telemetry &msg) {
	Command cmd; //TODO assumptions -- only real planes can have id conflict -- 
	if (!newPlanes.empty()) {//a plane has already been created for a real plane to fill
		cmd.commandID = COMMAND_SET_ID;
		cmd.planeID = msg.planeID;
		cmd.param = newPlanes.front();
		cmd.sim = false;
		cmd.commandHeader.stamp = ros::Time::now();
		commandTopic.publish(cmd);
		newPlanes.pop_front();
	} else {
		waypoint dest;
		dest.altitude = msg.destAltitude;
		dest.longitude = msg.destLongitude;
		dest.latitude = msg.destLatitude;
		PlaneBuilder b;
		cmd.commandID = COMMAND_SET_ID;
		cmd.planeID = msg.planeID;
		cmd.param = b.buildPlane(dest, *this);
		if (cmd.param != -1) {
			cmd.sim = false;
			cmd.commandHeader.stamp = ros::Time::now();
			commandTopic.publish(cmd);
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Coordinator");
	ros::NodeHandle n;
	Coordinator c;
	c.init(n);
	c.run();
	return 0;
}
