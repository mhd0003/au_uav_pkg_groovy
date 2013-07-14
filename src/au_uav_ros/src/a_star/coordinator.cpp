/*
coordinator
This will be responsible for storing plane data for both real and simulated planes (will not be able to tell
a difference here) and for sending commands to those planes.  It will receive commands from collision 
avoidance as well.
*/

//Standard C++ headers
#include <sstream>
#include <vector>

//ROS headers
#include "ros/ros.h"
#include "ros/package.h"
#include "au_uav_ros/TelemetryUpdate.h"
#include "au_uav_ros/Command.h"
#include "au_uav_ros/RequestPlaneID.h"
#include "au_uav_ros/GoToWaypoint.h"
#include "au_uav_ros/PlaneCoordinator.h"
#include "au_uav_ros/LoadPath.h"
#include "au_uav_ros/RequestWaypointInfo.h"
#include "au_uav_ros/LoadCourse.h"
#include "au_uav_ros/standardDefs.h"
#include "au_uav_ros/StartCoordinator.h"

// A* Path Planning headers
#include "au_uav_ros/planeObject.h"
#include "a_star/prar.cpp"

//publisher is global so callbacks can access it
ros::Publisher commandPub;

//coordinator list of UAVs, may want to lengthen this or perhaps change it to a map, not sure
std::map<int, au_uav_ros::PlaneCoordinator> planesArray;

//just a count of the number of planes so far, init to zero
int numPlanes = 0;

//global boolean to start this node.
bool readyToStart = false;

/*
isValidPlanID(...)
simple function to make sure that an ID sent to us is known by the coordinator
*/
bool isValidPlaneID(int id)
{
	//if the number id is valid
	//AND we have that id in the map
	//AND that UAV is active
	if(id >= 0 && planesArray.find(id) != planesArray.end() && planesArray[id].isActive) return true;
	else return false;
}

//service to run whenever a new plane enters the arena to tell it the ID number it should use
//should only be called for simulated UAVs. Real UAVs have predefined planeID.
bool requestPlaneID(au_uav_ros::RequestPlaneID::Request &req, au_uav_ros::RequestPlaneID::Response &res)
{
	//ROS_ERROR("The ID is #%d.\n", req.requestedID);
	//check to see if we've been given an ID
	if(req.requestedID == -1)
	{
		//we weren't given an ID, so pick the first that's free
		int id = 0;
		while(true)
		{
			//check if the ID is occupied or inactive
			if(planesArray.find(id) != planesArray.end() && planesArray[id].isActive)
			{
				//this id already exists, increment our id and try again
				id++;
			}
			else
			{
				//we found an unused ID, lets steal it
				planesArray[id] = au_uav_ros::PlaneCoordinator();
				planesArray[id].isActive = true;
				numPlanes++;
				
				res.planeID = id;
				return true;
			}
		}
		
		//not sure how we'd get here, but better to handle it
		return false;
	}
	else
	{
		//we've been given an ID, check if it's open or inactive
		if(planesArray.find(req.requestedID) == planesArray.end() || !planesArray[req.requestedID].isActive)
		{
			planesArray[req.requestedID] = au_uav_ros::PlaneCoordinator();
			planesArray[req.requestedID].isActive = true;
			numPlanes++;
			res.planeID = req.requestedID;
			return true;
		}
		else
		{
			ROS_ERROR("The ID #%d is already taken.\n", req.requestedID);
			return false;
		}
	}
}

/*
goToWaypoint(...)
This is the key waypoint function to be used by any external program for telling the coordinator to add
something to our queue.  The request gives it a point, a queue, and whether the queue should be erased before
this new point is added.
*/
bool goToWaypoint(au_uav_ros::GoToWaypoint::Request &req, au_uav_ros::GoToWaypoint::Response &res)
{
	ROS_INFO("Service Request Received: Plane #%d go to (%f, %f, %f)", req.planeID, req.latitude, req.longitude, req.altitude);	
	
	//check for valid plane ID
	if(isValidPlaneID(req.planeID))
	{
		//construct waypoint
		struct au_uav_ros::waypoint pointFromService;
		pointFromService.latitude = req.latitude;
		pointFromService.longitude = req.longitude;
		pointFromService.altitude = req.altitude;
		
		//attempt to set waypoint
		if(planesArray[req.planeID].goToPoint(pointFromService, req.isAvoidanceManeuver, req.isNewQueue))
		{
			//success!
			if (req.isAvoidanceManeuver && req.planeID >= 32 && req.planeID <= 63) {
			ROS_ERROR("UAV # %d is performing an avoidance maneuver.", req.planeID);
			}
			/*
			if we have a new queue, we want to forward the new point immediately, otherwise just
			let the normal command sending do it's work
			*/
			if(req.isNewQueue)
			{
				//get the command
				au_uav_ros::Command commandToSend = planesArray[req.planeID].getPriorityCommand();
				commandToSend.planeID = req.planeID;
				
				if(commandToSend.latitude == -1000 || commandToSend.longitude == -1000 || commandToSend.altitude == -1000)
				{
					//dont send it
				}
				else
				{
					commandPub.publish(commandToSend);
					ROS_INFO("Sent command to plane #%d: (%f, %f, %f)", commandToSend.planeID, commandToSend.latitude, commandToSend.longitude, commandToSend.altitude);
				}
				
			}
			
			return true;
		}
		else
		{
			//this should never happen in the current setup
			ROS_ERROR("Error in planesArray[%d].goToPoint(...)", req.planeID);
			return false;
		}
	}
	else
	{
		ROS_ERROR("Invalid plane ID in GoToWaypoint service request.\n");
		return false;
	}
}

/*
loadPathCallback(...)
This is the callback used when the user requests for a plane to start a certain path.
*/
bool loadPathCallback(au_uav_ros::LoadPath::Request &req, au_uav_ros::LoadPath::Response &res)
{
	ROS_INFO("Received request: Load path from \"%s\" to plane #%d\n", req.filename.c_str(), req.planeID);
	
	//check for a valid plane ID sent
	if(isValidPlaneID(req.planeID))
	{
		//open our file
		FILE *fp;
		fp = fopen((ros::package::getPath("au_uav_ros")+"/paths/"+req.filename).c_str(), "r");
		
		//check for a good file open
		if(fp != NULL)
		{
			char buffer[256];
			bool isFirstLine = true;
			bool isAvoidance = false;
			
			//while we have something in the file
			while(fgets(buffer, sizeof(buffer), fp))
			{
				//check first character for comment or if the line is blank
				if(buffer[0] == '#' || isBlankLine(buffer))
				{
					//this line is a comment
					continue;
				}
				else
				{
					//set some invalid defaults
					struct au_uav_ros::waypoint temp;
					temp.latitude = temp.longitude = temp.altitude = -1000;
					
					//parse the string
					sscanf(buffer, "%lf %lf %lf\n", &temp.latitude, &temp.longitude, &temp.altitude);
					ROS_INFO("Parsing path file: %lf %lf %lf", temp.latitude, temp.longitude, temp.altitude);
					//check for the invalid defaults
					if(temp.latitude == -1000 || temp.longitude == -1000 || temp.altitude == -1000)
					{
						//this means we have a bad file somehow
						ROS_ERROR("Bad file parse");
						res.error = "Bad file parse, some points loaded";
						return false;
					}
					
					//call the goToPoint function for the correct plane
					planesArray[req.planeID].goToPoint(temp, isAvoidance, isFirstLine);
					
					//only clear the queue with the first point
					if(isFirstLine) isFirstLine = false;
				}
			}
			
			//end of file, return
			return true;
		}
		else
		{
			ROS_ERROR("Invalid filename or location: %s", req.filename.c_str());
			res.error = "Invalid filename or location";
			return false;
		}
	}
	else
	{
		ROS_ERROR("Invalid plane ID");	
		res.error = "Invalid plane ID";
		return false;
	}
}

std::vector<au_uav_ros::waypoint> generateCombinedWaypoints(vector<au_uav_ros::waypoint> *realWaypoints, std::vector<map_tools::waypointPath> *avoidPaths) {
	int avoidanceIndex = 0;
	vector<au_uav_ros::waypoint> combined;
	for (int i = 0; i < realWaypoints->size(); i++) {
		// i-1 is for if the first avoidance path is from the plane's start position to the first waypoint
		if (avoidanceIndex < avoidPaths->size() && i-1 == (*avoidPaths)[avoidanceIndex].startWaypointIndex) {
			for (int j = 0; j < (*avoidPaths)[avoidanceIndex].pathWaypoints.size(); j++) {
				combined.push_back((*avoidPaths)[avoidanceIndex].pathWaypoints[j]);
			}
			avoidanceIndex++;
		}
		// add next waypoint
		combined.push_back((*realWaypoints)[i]);
	}
	return combined;
}

/*
loadCourseCallback(...)
This callback takes a filename and parses that file to load a course into the coordinator. A course in
this sense would be several paths for multiple UAVs.  We want this so we can test a collision avoidance
course by preplanning the course.
*/
bool loadCourseCallback(au_uav_ros::LoadCourse::Request &req, au_uav_ros::LoadCourse::Response &res)
{
	ROS_INFO("Received request: Load course from \"%s\"\n", req.filename.c_str());
	
	//open our file
	FILE *fp;
	//JC - April 9th - GUI gives full path
	fp = fopen(req.filename.c_str(), "r");
	//fp = fopen((ros::package::getPath("au_uav_ros")+"/courses/"+req.filename).c_str(), "r");
	
	//check for a good file open
	if(fp != NULL)
	{
		char buffer[256];
		
		std::map<int, bool> isFirstPoint;
		std::map<int, std::vector<au_uav_ros::waypoint> > allPlanesWaypoints;
		std::map<int, au_uav_ros::PlaneObject> realPlanes;
		bool isAvoidance = false;
		
		//while we have something in the file
		while(fgets(buffer, sizeof(buffer), fp))
		{
			//check first character
			if(buffer[0] == '#' || isBlankLine(buffer))
			{
				//this line is a comment
				continue;
			}
			else
			{
				//set some invalid defaults
				int planeID = -1;
				struct au_uav_ros::waypoint temp;
				temp.latitude = temp.longitude = temp.altitude = -1000;
				
				//parse the string
				sscanf(buffer, "%d %lf %lf %lf\n", &planeID, &temp.latitude, &temp.longitude, &temp.altitude);
				ROS_INFO("Loaded %d to %lf %lf %lf", planeID, temp.latitude, temp.longitude, temp.altitude);
				
				//check for the invalid defaults
				if(planeID == -1 || temp.latitude == -1000 || temp.longitude == -1000 || temp.altitude == -1000)
				{
					//this means we have a bad file somehow
					ROS_ERROR("Bad file parse");
					res.error = "Bad file parse, some points loaded";
					return false;
				}
				
				//check our map for an entry, if we dont have one then this is the first time
				//that this plane ID has been referenced so it's true
				if(isFirstPoint.find(planeID) == isFirstPoint.end())
				{
					isFirstPoint[planeID] = true;
					realPlanes[planeID] = au_uav_ros::PlaneObject();
					// add this as waypoint to the front of queue, simulation is set up so that the 
					// first waypoint is the starting location, then the simulation really begins
					realPlanes[planeID].setCurrentLoc(temp.latitude, temp.longitude, temp.altitude);
					realPlanes[planeID].setID(planeID);
				}
				else {
					allPlanesWaypoints[planeID].push_back(temp);
				}
				
				//call the goToPoint function for the correct plane
				//planesArray[planeID].goToPoint(temp, isAvoidance, isFirstPoint[planeID]);
				
		
				//only clear the queue with the first point
				if(isFirstPoint[planeID]) isFirstPoint[planeID] = false;
			}
		}
		
		std::map<int, std::vector<waypointPath> > *allPlanesPaths = getAvoidancePlan(&realPlanes, 0, &allPlanesWaypoints);
		// set the first waypoint to the planes starting location; this is for the simulation
		for (std::map<int, au_uav_ros::PlaneObject>::iterator it = realPlanes.begin(); it != realPlanes.end(); it++) {
			au_uav_ros::waypoint temp;
			temp.latitude = it->second.getCurrentLoc().latitude;
			temp.longitude = it->second.getCurrentLoc().longitude;
			temp.altitude = it->second.getCurrentLoc().altitude;
			planesArray[it->first].goToPoint(temp, false, true);
		}	

		for (std::map<int, std::vector<waypointPath> >::iterator it = allPlanesPaths->begin(); it != allPlanesPaths->end(); it++) {
			vector<au_uav_ros::waypoint> combinedWays = generateCombinedWaypoints(&(allPlanesWaypoints[it->first]), &(it->second));
			ROS_INFO("Size of combined waypoints for plane %d is %d", it->first, combinedWays.size());
			for (int i = 0; i < combinedWays.size(); i++) {
				planesArray[it->first].goToPoint(combinedWays[i], false, false);
			}
		}


		//end of file, return
		return true;
	}
	else
	{
		ROS_ERROR("Invalid filename or location: %s", req.filename.c_str());
		res.error = "Invalid filename or location";
		return false;
	}
}

/*
requestWaypointInfoCallback(...)
This callback allows the avoidance algorithms (or I suppose anything really) to view the waypoint queues
in the system.  It takes a plane ID, a boolean value that reflects whether it's a request for the avoidance
queue or the normal queue, and finally a position in that queue.  If there isn't a value at the requested
place, we fill in the error message and return a point (-1000, -1000, -1000) simply because it's not a valid
position.

@position - follows normal array standards, aka 0 is the front of the queue
*/
bool requestWaypointInfoCallback(au_uav_ros::RequestWaypointInfo::Request &req, au_uav_ros::RequestWaypointInfo::Response &res)
{
	//check that the request ID is valid
	if(isValidPlaneID(req.planeID))
	{
		//fill out the waypoint to return
		au_uav_ros::waypoint temp = planesArray[req.planeID].getWaypointOfQueue(req.isAvoidanceWaypoint, req.positionInQueue);
		res.latitude = temp.latitude;
		res.longitude = temp.longitude;
		res.altitude = temp.altitude;
		
		//check if we need to return the error message
		if(res.latitude == -1000 && res.longitude == -1000 && res.altitude == -1000)
		{
			res.error = "No points in that queue";
		}
		
		return true;
	}
	else
	{
		ROS_ERROR("Invalid plane ID");
		res.error = "Invalid plane ID";
		return false;
	}
}

/*
telemetryCallback(...)
This function is run whenever a new telemetry update from any plane is recieved.  Mainly, it forwards the
update onwards and it will send new commands if the plane coordinators deem it necessary.
*/
//NOTE:  For now making real UAVs ID > 31;  Jan 29th edit
void telemetryCallback(const au_uav_ros::TelemetryUpdate::ConstPtr& msg)
{
	//ROS_INFO("Received update #[%d] from plane ID %d", msg->telemetryHeader.seq, msg->planeID);
	
	//check the make sure the update is valid first
	if(isValidPlaneID(msg->planeID))
	{
		//prep in case a command needs to be sent
		au_uav_ros::Command commandToSend;

		//check whether the update warrants a new command or not
		if(planesArray[msg->planeID].handleNewUpdate(*msg, &commandToSend))
		{
			//send new command
			commandPub.publish(commandToSend);
			ROS_INFO("Sent command to plane #%d: (%f, %f, %f)", commandToSend.planeID, commandToSend.latitude, commandToSend.longitude, commandToSend.altitude);
		}
		else
		{
			//don't send a new command
		}
	}
	/*  Dont need now because we are init planes with a course file
	else
	{	//were here bc this UAV isn't in planeArray
		if (msg->planeID > 31 and msg->planeID < 65) {
			//later might need more robust code for multiple UAVs
			//if here this is a real UAV so add planeCoordinator to array
			planesArray[msg->planeID] = au_uav_ros::PlaneCoordinator();
			planesArray[msg->planeID].isActive = true;
			//load path for that planeCoordinator object
			//can use loadPathCallback method code for a specific path.
			au_uav_ros::LoadPath::Request req;
			req.filename = "SDTest.path";
			req.planeID = msg->planeID;
			au_uav_ros::LoadPath::Response res;
			loadPathCallback(req, res);
			ROS_INFO("Added new UAV[#%d] to planeArray", msg->planeID);

			//now send first command to fly to first wp.
			au_uav_ros::Command commandToSend;
			au_uav_ros::waypoint destination;
			//set command info
			destination = planesArray[msg->planeID].getWaypointOfQueue(false, 0);
			commandToSend.commandHeader.seq = 0;
			commandToSend.commandHeader.stamp = ros::Time::now();
			commandToSend.planeID = msg->planeID;
			commandToSend.latitude = destination.latitude;
			commandToSend.longitude = destination.longitude;
			commandToSend.altitude = destination.altitude;
			//send command
			commandPub.publish(commandToSend);
			ROS_INFO("Sent FIRST command to plane #%d: (%f, %f, %f)", commandToSend.planeID, commandToSend.latitude, commandToSend.longitude, commandToSend.altitude);
		}
*/
		else {
		ROS_ERROR("Received update from invalid plane ID #%d", msg->planeID);
		}
	//}
}

bool startCoordinator(au_uav_ros::StartCoordinator::Request &req, au_uav_ros::StartCoordinator::Response &res)
{
    //ROS_INFO(req.indicator);
    readyToStart = true;
    
    // we shouldn't ever have an error, so populate it with "passed"
    res.error = "passed";
    return true;
}


int main(int argc, char **argv)
{
	//Standard ROS startup
	ros::init(argc, argv, "coordinator");
	
	ros::NodeHandle n;


	//init service to start this node        
	ros::ServiceServer startCoordinatorServer = n.advertiseService("start_coordinator", startCoordinator);

	ros::Rate r(10); // 10 hz
        while (!readyToStart)
        {
        ros::spinOnce();
        r.sleep();
        }

	
	
	//Subscribe to telemetry message and advertise avoid collision service
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	ros::ServiceServer newPlaneServer = n.advertiseService("request_plane_ID", requestPlaneID);
	ros::ServiceServer goToWaypointServer = n.advertiseService("go_to_waypoint", goToWaypoint);
	ros::ServiceServer loadPathServer = n.advertiseService("load_path", loadPathCallback);
	ros::ServiceServer requestWaypointInfo = n.advertiseService("request_waypoint_info", requestWaypointInfoCallback);
	ros::ServiceServer loadCourseServer = n.advertiseService("load_course", loadCourseCallback);
	commandPub = n.advertise<au_uav_ros::Command>("commands", 1000);

	//Needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}


