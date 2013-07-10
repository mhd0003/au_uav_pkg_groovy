#include "au_uav_ros/Command.h"

#include "ros/ros.h"

using namespace au_uav_ros;

int main(int argc, char **argv) {
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<au_uav_ros::Command>("commands", 1000);
	int i = 4;
	ros::Rate r(10);
	Command wp, id;
	wp.sim = false;
	wp.planeID = 3;
	wp.altitude = 0;
	wp.longitude = 0;
	wp.latitude = 0;
	wp.commandID = 1;
	wp.param = 2;

	id.sim = false;
	id.planeID = 2;
	id.altitude = 0;
	id.longitude = 0;
	id.latitude = 0;
	id.commandID = 3;
	id.param = 3;

	int j = 0;
	while (ros::ok()) {
		/*if (j == 20) {
			i = (i + 1) % 100;
			id.param = i;
			pub.publish(id);
			wp.planeID  = id.param;
			id.planeID = id.param;

			j = 0;
		} */
		pub.publish(wp);
		wp.longitude++;
		//pub.publish(id);
		r.sleep();
		//j++;
	}
}
