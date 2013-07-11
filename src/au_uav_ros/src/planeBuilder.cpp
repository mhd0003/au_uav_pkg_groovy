#include "au_uav_ros/planeBuilder.h"
using namespace au_uav_ros;

int PlaneBuilder::buildPlane(struct waypoint wp, Coordinator &c) {
	int planeID = findPlaneID(c);
	if (planeID != -1) {
		wp.planeID = planeID;
		c.planes[planeID] = PlaneObject(wp);
		c.newPlanes.push_back(planeID);
	}
	return planeID;
}

int PlaneBuilder::buildSimPlane(struct waypoint wp, Coordinator &c, SimPlane &s) {
	int planeID = findPlaneID(c);
	if (planeID != -1) {
		wp.planeID = planeID;
		c.simPlanes[planeID] = SimPlaneObject(wp);
		s.request.planeID = planeID;
		s.request.clear = false;
		s.request.size = 1;
		s.request.latitudes.push_back(wp.latitude);
		s.request.longitudes.push_back(wp.longitude);
		s.request.altitudes.push_back(wp.altitude);
	}
	return planeID;
}

bool PlaneBuilder::buildCourse(std::string filename, Coordinator &c, std::map<int, SimPlane> &s) {
	//open our file
	FILE *fp;
	fp = fopen(filename.c_str(), "r");

	//check for a good file open
	if(fp != NULL)
	{
		char buffer[256];

		// md
		enum { SIMULATED, REAL };

		std::map<int, bool> isFirstPoint;
		while(fgets(buffer, sizeof(buffer), fp))
		{
			if(buffer[0] == '#' || isBlankLine(buffer))
			{
				//this line is a comment
				continue;
			}
			else
			{
				//set some invalid defaults
				int planeID = -1;
				struct waypoint tempWP;
				int normal;

				// md
				int type;
				//tempWP.latitude = tempWP.longitude = tempWP.altitude = -1000;

				//parse the string
				sscanf(buffer, "%d %lf %lf %lf %d\n", &planeID, &tempWP.latitude, &tempWP.longitude, &tempWP.altitude, &type);
				//check for the invalid defaults
				if(planeID == -1 || tempWP.latitude == -1000 || tempWP.longitude == -1000 || tempWP.altitude == -1000)
				{
					//this means we have a bad file somehow
					ROS_ERROR("Bad file parse");
					return false;
				}

				//check our map for an entry, if we dont have one then this is the first time
				//that this plane ID has been referenced so it's true
				if(isFirstPoint.find(planeID) == isFirstPoint.end())
				{
					// md
					// Logic changed here.
					// If 'type' was not read from course file, default to simulated plane object.
					if (type == REAL) { 
						isFirstPoint[planeID] = true;
						if(planeID == -1)
						{
							ROS_ERROR("Couldn't create plane with ID %d", planeID);
							return false;
						}

						//Create New Plane Here
						tempWP.planeID = planeID;
						c.planes[planeID] = PlaneObject(tempWP);
						c.newPlanes.push_back(planeID);
					} else { //if (type == SIMULATED) {
						isFirstPoint[planeID] = true;
						if (planeID == -1) {
							ROS_ERROR("Couldn't create plane");
							return false;
						}

						//Create New Plane Here
						tempWP.planeID = planeID;
						c.simPlanes[planeID] = SimPlaneObject(tempWP);
						s[planeID].request.size = 1;
						s[planeID].request.clear = false;
						s[planeID].request.planeID = planeID;
						s[planeID].request.latitudes.push_back(tempWP.latitude);
						s[planeID].request.longitudes.push_back(tempWP.longitude);
						s[planeID].request.altitudes.push_back(tempWP.altitude);
					}
				} else {
					isFirstPoint[planeID] = false;
					if (type == REAL) {
						c.planes[planeID].addNormalWp(tempWP);
					} else {
						c.simPlanes[planeID].addNormalWp(tempWP);
						s[planeID].request.size++;
						s[planeID].request.latitudes.push_back(tempWP.latitude);
						s[planeID].request.longitudes.push_back(tempWP.longitude);
						s[planeID].request.altitudes.push_back(tempWP.altitude);
					}
				}

				//only clear the queue with the first point
				//if(isFirstPoint[planeID]) isFirstPoint[planeID] = false;
			}
		}

		//if we make it here everything happened according to plan
		return true;
	}
	else
	{
		ROS_ERROR("Invalid filename or location: %s", filename.c_str());
		return false;
	}

}

int PlaneBuilder::findPlaneID(Coordinator &c) {
	int newId = 2; //planeid of 1 is for new planes who need a planeid

	while(newId < 256) //plane id can't be larger than 255
	{
		//check if the ID is occupied or inactive
		if((c.planes.find(newId) != c.planes.end()) || (c.simPlanes.find(newId) != c.simPlanes.end())) //removed && planesArray[id].isActive
		{
			//this id already exists, increment our id and try again
			newId++;
		}
		else
		{
			//we found an unused ID, lets steal it
			//creation happens later planes[newId] = PlaneObject();
			//removed planesArray[id].isActive = true;
			//numPlanes++;
				
			return  newId;
		}
	}

		//plane id is out of bounds
		return -1;
}
