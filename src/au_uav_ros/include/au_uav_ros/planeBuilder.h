#ifndef _PLANE_BUILDER_
#define _PLANE_BUILDER_

#include <stdlib.h>
#include <string>

#include "ros/console.h"
#include "ros/package.h"

#include "au_uav_ros/SimPlane.h"

#include "au_uav_ros/coordinator.h"
#include "au_uav_ros/standardDefs.h"
using namespace au_uav_ros;

namespace au_uav_ros {
	/**
	* ID resolution should happen here during creation
	* In flight ID conflicts can be handled by the coord
	*/
	class Coordinator;
	class PlaneBuilder {
	public:
		int buildPlane(struct waypoint wp, Coordinator &c);
		int buildSimPlane(struct waypoint wp, Coordinator &c, SimPlane &s);
		bool buildCourse(std::string filename, Coordinator &c, std::map<int, SimPlane> &s);
		int findPlaneID(Coordinator &c);	
	};
}
#endif
