#include "au_uav_ros/simulator.h"
using namespace au_uav_ros;

void Simulator::run(void) {
	ros::spin();
}

void Simulator::init(ros::NodeHandle _n) {
	n = _n;
	setup();
}

void Simulator::setup(void) {
	shutdownTopic = n.subscribe("component_shutdown", 1000, &Simulator::component_shutdown, this);
	setFreqService = n.advertiseService("sim_freq", &Simulator::sim_freq, this);
	setSimSpeed = n.advertiseService("sim_speed", &Simulator::sim_speed, this);

	managePlanesService = n.advertiseService("manage_simplanes", &Simulator::manage_simplanes, this);

	telemetryTopic = n.advertise<au_uav_ros::Telemetry>("telemetry", 1000);
	commandTopic = n.subscribe("commands", 1000, &Simulator::commands, this);

	double temp;
	// Set initial simulation frequency through launch file parameter
	// Default to centralized
	if (n.getParam("runSimFreq", temp)) {
		simulateTimer = n.createWallTimer(ros::WallDuration(temp), &Simulator::simulate, this);
	} else {
		simulateTimer = n.createWallTimer(ros::WallDuration(1.0), &Simulator::simulate, this);
	}
	// if (n.getParam("runSimSpeed", temp)) {
	// 	simulateTimer = n.createWallTimer(ros::WallDuration(temp), &Simulator::simulate, this);
	// } else {
	// 	simulateTimer = n.createWallTimer(ros::WallDuration(1.0), &Simulator::simulate, this);
	
	// Set de/centralized through launch file parameter
	// Default to centralized
	if (n.getParam("runCentralized", centralized)) {
		//centralized = $(arg centralized)
	} else {
		centralized = true;
	}
}

void Simulator::component_shutdown(const std_msgs::String::ConstPtr &msg) {
	ros::shutdown();
}

bool Simulator::sim_freq(Double::Request &req, Double::Response &res) {
	simulateTimer = n.createWallTimer(ros::WallDuration(req.val), &Simulator::simulate, this);

	return true;
}

bool Simulator::sim_speed(Double::Request &req, Double::Response &res) {
	std::map<int, SimPlaneObject>::iterator i;
	for (i = simPlanes.begin(); i != simPlanes.end(); i++) {
		i->second.setSimSpeed(req.val);
	}
	return true;
}

bool Simulator::manage_simplanes(au_uav_ros::SimPlane::Request &req, au_uav_ros::SimPlane::Response &res) {
	if (req.clear) {
		simPlanes.clear();
	}
	else if (simPlanes.find(req.planeID) != simPlanes.end()) {
		if (req.size == 0) { //no waypoints given so delete the plane
			simPlanes.erase(req.planeID);
		} else {
			if (req.add) {
				waypoint wp;
				wp.latitude = req.latitudes.front();
				wp.longitude = req.longitudes.front();
				wp.altitude = req.altitudes.front();
				simPlanes[req.planeID].addNormalWp(wp);
			} else {
				//TODO simPlanes[req.planeID].removeWp();
			}
		}
	} else {//new plane id so add it to map
		simPlanes[req.planeID] = SimPlaneObject();
		simPlanes[req.planeID].setID(req.planeID);
		simPlanes[req.planeID].setCurrentLoc(req.latitudes[0], req.longitudes[0], req.altitudes[0]);
		for (int i = 1; i < req.size; i++) {
			waypoint wp;
			wp.latitude = req.latitudes[i];
			wp.longitude = req.longitudes[i];
			wp.altitude = req.altitudes[i];
			simPlanes[req.planeID].addNormalWp(wp);
		}

		double temp;
		if (n.getParam("runSimSpeed", temp)) {
			simPlanes[req.planeID].setSimSpeed(temp);
		} else {
			simPlanes[req.planeID].setSimSpeed(1.0);
		}
	}
	res.error = "None";
	return true;
}

void Simulator::commands(const au_uav_ros::Command &cmd) {
	if (simPlanes.find(cmd.planeID) != simPlanes.end()) {
		simPlanes[cmd.planeID].handleNewCommand(cmd);
	}
}

void Simulator::simulate(const ros::WallTimerEvent &e) {
	double duration = (e.current_real - e.last_real).toSec();
	if (e.profile.last_duration.toSec() == 0) { //on startup this should be zero 
		return;
	} else {
		double stamp = ros::Time::now().toSec();
		std::map<int, SimPlaneObject>::iterator i;
		for (i = simPlanes.begin(); i != simPlanes.end(); i++) {
			Telemetry telem;
			i->second.simulate(duration, &telem);
			telemetryTopic.publish(telem);
			duration += (ros::Time::now().toSec() - stamp);
			stamp = ros::Time::now().toSec();
		} 
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Simulator");
	ros::NodeHandle n;
	Simulator s;
	s.init(n);
	s.run();
	return 1;
}
