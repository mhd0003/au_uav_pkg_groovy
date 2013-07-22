/*
evaluator
This is the end-of-semester REU evaluation program intended to score the system based on a number
of pre-determined variables and to output that information to a file as well as perform logging
functions on that data.
*/

#include "au_uav_ros/evaluator.h"
using namespace au_uav_ros;
using std::list;
using std::map;
using std::string;
using std::cout;
using std::endl;


void Evaluator::run(void) {
	ros::spin();
}

void Evaluator::init(ros::NodeHandle _n) {
	n = _n;
	setup();
}

void Evaluator::setup(void) {
	//overall data
	waypointsTotal = 0;
	numConflicts = 0;
	numCollisions = 0;
	deadPlaneCount = 0;
	totalDistTraveled = 0;
	totalMinDist = 0;
	score = 0;

	//store the "last" ID
	lastPlaneID = -1;
	maxAlivePlane = -1;

	addPlaneClient = n.serviceClient<AddPlane>("add_plane");
	loadCourseClient = n.serviceClient<LoadCourse>("load_course");
	removePlaneClient = n.serviceClient<RemovePlane>("remove_plane");

	shutdownTopic = n.advertise<std_msgs::String>("component_shutdown", 1000, true);
	// shutdownTopic = n.subscribe("component_shutdown", 1000, &Evaluator::component_shutdown, this);
	telemetryTopic = n.subscribe("telemetry", 1000, &Evaluator::telemetry, this);

	//get the file input
	char filename[256];
	printf("\nEnter the filename of the course to load 22:");
	scanf("%s", filename);
	printf("\nEnter the filename for output:");
	scanf("%s", scoresheetFilename);

	printf("Enter the length of simulation in seconds: ");
	scanf("%d",&TIME_LIMIT);

	//create all our UAVs
	if(!createCourseUAVs(filename))
	{
		ROS_ERROR("createCourseUAVs failed! Shutting down now...");
		this->shutdown();
	}

	//success
	system("clear");
	printf("\n");
	printf("Last Plane ID: %d\n", lastPlaneID);
	printf("KML File: %s.kml\n", scoresheetFilename);
	printf("Score File: %s.score\n", scoresheetFilename);

	//call the load course function on the coordinator
	LoadCourse srv;
	srv.request.filename = (ros::package::getPath("au_uav_ros")+"/courses/"+filename).c_str();
	loadCourseClient.call(srv);

	numAvoids = 0;


	//set the start time to now
	startTime = ros::Time::now();
}

/*
 * Shutdown all nodes listening to component_shutdown topic.
 */
void Evaluator::shutdown(void) {
	std_msgs::String msg;
	msg.data = "shut it down";
	shutdownTopic.publish(msg);

	ros::Duration(1.0).sleep();
	ROS_ERROR("Evaluator: %s", msg.data.c_str());
	ros::shutdown();
}
// void Evaluator::component_shutdown(const std_msgs::String::ConstPtr &msg) {
// 	ros::shutdown();
// }

/*
displayOutput()
Just a consolidated function for updating what's reflected on the screen during a test
*/
void Evaluator::displayOutput(void)
{
	system("clear");

	printf("Plane ID\tDistance Traveled(m)\tMinimum Travel(m)\t\tWaypoints Achieved\tTime of Death(s)\n");
	printf("--------\t--------------------\t-----------------\t\t------------------\t----------------\n");
	for(int id = 0; id <= lastPlaneID; id++)
	{
		printf("%d\t\t%lf\t\t%lf\t\t\t%d\t\t\t", id, waypointDistTraveled[id], minimumTravelDistance[id], waypointsAchieved[id]);
		if(isDead[id])
		{
			printf("%lf\n", timeOfDeath[id].toSec());
		}
		else
		{
			printf("ALIVE\n");
		}
	}
	printf("\n");
	printf("Totals:\n");
	printf("Elapsed time:%lf\n", (ros::Time::now() - startTime).toSec());
	printf("Waypoints reached: %d\n", waypointsTotal);
	printf("Number of conflicts: %d\n", numConflicts);
	printf("Number of collisions: %d\n", numCollisions);
	printf("Dead plane count: %d\n", deadPlaneCount);
	if(totalMinDist != 0) printf("Distance actual/distance minimum: %lf\n", totalDistTraveled/totalMinDist);
	printf("Score: %d\n", score);


}

/*
endEvaluation()
This function is called upon termination of the simulation
*/
void Evaluator::endEvaluation(void)
{
	//open our scoring file
	FILE *fp;
	fp = fopen((ros::package::getPath("au_uav_ros")+"/scores/"+scoresheetFilename+".score").c_str(), "w");

	//make sure we got a good open
	if(fp == NULL)
	{
		printf("\nERROR SAVING DATA, COPY TERMINAL OUTPUT!!!\n");
	}
	else
	{

		//for automator!!!!!!!!!!!!!!!!!
		fprintf(fp,"%d %d %f %f %d\n\n",numConflicts,numCollisions,totalDistTraveled,totalMinDist,numAvoids);


		//dump our data into
		fprintf(fp, "Plane ID\tDistance Traveled(m)\tMinimum Travel(m)\t\tWaypoints Achieved\tTime of Death(s)\n");
		fprintf(fp, "--------\t--------------------\t-----------------\t\t------------------\t----------------\n");
		for(int id = 0; id <= lastPlaneID; id++)
		{
			fprintf(fp, "%d\t\t\t\t%lf\t\t\t\t%lf\t\t\t\t\t%d\t\t\t\t\t", id, waypointDistTraveled[id], minimumTravelDistance[id], waypointsAchieved[id]);
			if(isDead[id])
			{
				fprintf(fp, "%lf\n", timeOfDeath[id].toSec());
			}
			else
			{
				fprintf(fp, "ALIVE\n");
			}
		}
		fprintf(fp, "--------\t--------------------\t-----------------\t\t------------------\t----------------\n");
		ros::Duration averageDeath = ros::Duration(0);
		for(int x = 0; x <=lastPlaneID; x++)
		{
			if(isDead[x])
			{
				//we died
				averageDeath+=timeOfDeath[x];
			}
			else
			{
				//we lasted the whole time
				averageDeath+=ros::Duration(TIME_LIMIT);
			}
		}
		fprintf(fp, "Averages:\t\t%lf\t\t\t\t%lf\t\t\t%lf\t\t\t\t%lf\n", totalDistTraveled/(lastPlaneID+1), totalMinDist/(lastPlaneID+1), waypointsTotal/(lastPlaneID+1.0), averageDeath.toSec()/(lastPlaneID+1.0));
		fprintf(fp, "\n");
		fprintf(fp, "Totals:\n");
		fprintf(fp, "Elapsed time:%lf\n", (ros::Time::now() - startTime).toSec());
		fprintf(fp, "Waypoints reached: %d\n", waypointsTotal);
		fprintf(fp, "Number of conflicts: %d\n", numConflicts);
		fprintf(fp, "Number of collisions: %d\n", numCollisions);
		fprintf(fp, "Dead plane count: %d of %d\n", deadPlaneCount, (lastPlaneID+1));
		if(totalMinDist != 0) fprintf(fp, "Distance actual/distance minimum: %lf\n", totalDistTraveled/totalMinDist);
		fprintf(fp, "Final Score: %d\n", score);
		fprintf(fp,"Number of avoidance manuevers: %d", numAvoids);

		//close the file
		fclose(fp);

		//add this score sheet to the list of all score sheets for this group(for automator)
		string allScoresName = string(scoresheetFilename);
		int pos = allScoresName.find_first_of("_");
		if(pos>0){//if we found it
			allScoresName = allScoresName.substr(0,pos);
		}
		allScoresName = allScoresName + ALL_SCORES_ENDING;
		FILE *allFP = fopen((ros::package::getPath("au_uav_ros")+"/scores/"+allScoresName).c_str(),"a");
		fprintf(allFP,"%s%s\n",scoresheetFilename,".score");

		fclose(allFP);
	}//else

	this->shutdown();
	//get the pid of the automator
	int ppid;
	FILE *pidFD = fopen(PID_FILE_FULL,"r");
	if(pidFD != NULL){
		fscanf(pidFD,"%d",&ppid);
		fclose(pidFD);
		kill(ppid,SIGUSR1); //send a signal to the automator telling it we are done
	}
/*
	//open file to let automator know we are done
	FILE *dp;
	dp = fopen((ros::package::getPath("au_uav_ros")+"/courses/"+DONE_FILE).c_str(), "w");
	fprintf(dp,"y");
	fclose(dp);
*/


	//terminate the program
	// exit(0);

}

/*
createCourseUAVs(...)
takes a filename and will parse it to determine how many UAVs there are and create them as needed
NOTE: this was copied from the ControlMenu and modified somewhat
*/
bool Evaluator::createCourseUAVs(string filename)
{
	//open our file
	FILE *fp;
	fp = fopen((ros::package::getPath("au_uav_ros")+"/courses/"+filename).c_str(), "r");

	//check for a good file open
	if(fp != NULL)
	{
		char buffer[256];

		map<int, bool> isFirstPoint;
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

				//parse the string
				sscanf(buffer, "%d %lf %lf %lf\n", &planeID, &tempWP.latitude, &tempWP.longitude, &tempWP.altitude);
				//check for the invalid defaults
				if(planeID == -1 || tempWP.latitude == -1000 || tempWP.longitude == -1000 || tempWP.altitude == -1000)
				{
					//this means we have a bad file somehow
					ROS_ERROR("Bad file parse");
					return false;
				}

				//add this point to the correct queue
				waypointQueues[planeID].push(tempWP);

				waypointLists[planeID].push_back(tempWP);//phil

				//check our map for an entry, if we dont have one then this is the first time
				//that this plane ID has been referenced so it's true
				if(isFirstPoint.find(planeID) == isFirstPoint.end())
				{
					isFirstPoint[planeID] = true;

					//set our last plane ID to this one
					lastPlaneID = planeID;
					maxAlivePlane = planeID;

					//set up some base values for a new plane
					// latestUpdatesMap[planeID] = Telemetry();
					latestUpdatesMap[planeID].currentLatitude = tempWP.latitude;
					latestUpdatesMap[planeID].currentLongitude = tempWP.longitude;
					latestUpdatesMap[planeID].currentAltitude = tempWP.altitude;

					// previousUpdatesMap[planeID] = Telemetry();
					previousUpdatesMap[planeID].currentLatitude = tempWP.latitude;
					previousUpdatesMap[planeID].currentLongitude = tempWP.longitude;
					previousUpdatesMap[planeID].currentAltitude = tempWP.altitude;

					distanceTraveled[planeID] = 0;
					waypointDistTraveled[planeID] = 0;
					distSinceLastWP[planeID] = 0;
					waypointsAchieved[planeID] = -1; //we get a "free" waypoint
					minimumTravelDistance[planeID] = 0;
					waypointMinTravelDist[planeID] = 0;
					isDead[planeID] = false;

					//subtract points to make up for the "free" waypoint at the start
					score = score - 5;
					waypointsTotal--;
				}

				//only clear the queue with the first point
				if(isFirstPoint[planeID]) {
					isFirstPoint[planeID] = false;
					waypoint wp;
					wp.latitude = wp.longitude = wp.altitude = 0;
					lastAvoidWaypoints[planeID] = wp;
				}
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

/*
telemetry
This is called whenever a new telemetry message is received.  We should store this any waypoint info received
and perform analysis on it once all planes have received new data.
*/
void Evaluator::telemetry(const Telemetry &msg)
{
	//make sure the sim isn't lagging somehow and still reporting dead planes
	if(isDead[msg.planeID]) return;

	//store the telemetry information received
	previousUpdatesMap[msg.planeID] = latestUpdatesMap[msg.planeID];
	latestUpdatesMap[msg.planeID] = msg;

	int id = msg.planeID;

	//to see if there has been an avoidance waypoint added
	bool normal = false;
	// md
	waypoint msgDestWP;
	msgDestWP.latitude = msg.destLatitude;
	msgDestWP.longitude = msg.destLongitude;
	msgDestWP.altitude = msg.destAltitude;

	// md
	if(msg.destLatitude != lastAvoidWaypoints[id].latitude){//only count each avoid waypoint once
		list<waypoint>::iterator it;
		for(it=waypointLists[id].begin(); it != waypointLists[id].end(); ++it){//go through all normal waypoints
			if(msg.destLatitude == it->latitude){//if this is normal waypoint, set flag
				normal = true;
			}
		}//for
		//if this is not a normal waypoint, it must be an avoidance way point
		if(!normal){
			numAvoids++;//add to the running total
			ROS_WARN("numAvoids: %d", numAvoids);
			//ROS_ERROR(" %f ", msg.destLatitude);
			lastAvoidWaypoints[id].latitude = msg.destLatitude;
			lastAvoidWaypoints[id].longitude = msg.destLongitude;
		}

	}//lastAvoidWP

	//if all UAVs have an update, run some analysis on this timestep
	if(msg.planeID == maxAlivePlane)
	{
		//get the current time
		delta = ros::Time::now() - startTime;

		struct waypoint current, other;
		//perform calculations on each plane
		for(int id = 0; id <= lastPlaneID; id++)
		{
			//nothing to do... WHEN YOU'RE DEAD!
			if(isDead[id]) continue;

			//change to waypoint format
			other.latitude = previousUpdatesMap[id].currentLatitude;
			other.longitude = previousUpdatesMap[id].currentLongitude;
			other.altitude = previousUpdatesMap[id].currentAltitude;
			current.latitude = latestUpdatesMap[id].currentLatitude;
			current.longitude = latestUpdatesMap[id].currentLongitude;
			current.altitude = latestUpdatesMap[id].currentAltitude;

			//add the distance traveled
			double d = distanceBetween(current, other);
			distanceTraveled[id] = distanceTraveled[id] + d;
			//totalDistTraveled = totalDistTraveled + d;
			distSinceLastWP[id] = distSinceLastWP[id] + d;

			//empty any items from the queue we've reached
			while(!waypointQueues[id].empty() && distanceBetween(waypointQueues[id].front(), current) < WAYPOINT_THRESHOLD)//COLLISION_THRESHOLD)
			{
				//the front item is reached, pop it and increase our waypoints reached
				struct waypoint temp = waypointQueues[id].front();
				waypointQueues[id].pop();

				if(waypointQueues[id].empty())
				{
					//we ran out of points x_x
					ROS_ERROR("Deleting %d", id);
					planesToDelete.push(id);

				}
				else
				{
					//add the last waypoint to our minimum distance
					minimumTravelDistance[id] = minimumTravelDistance[id] + waypointMinTravelDist[id];
					totalMinDist = totalMinDist + waypointMinTravelDist[id];
					waypointMinTravelDist[id] = distanceBetween(temp, waypointQueues[id].front());
				}

				//modify scoring values
				waypointsAchieved[id]++;
				waypointsTotal++;
				score += WAYPOINT_SCORE;

				//if(id == 0){ROS_ERROR("GOT HERE0");}
				//set our distance traveled for waypoints
				waypointDistTraveled[id] = waypointDistTraveled[id] + distSinceLastWP[id];
				totalDistTraveled = totalDistTraveled + distSinceLastWP[id];
				distSinceLastWP[id] = 0;
			}

			//time to check for collisions with any other UAVs
			for(int otherID = 0; otherID < id; otherID++)
			{
				//we assume the wreckage disapates very quickly... lol
				if(isDead[otherID]) continue;

				other.latitude = latestUpdatesMap[otherID].currentLatitude;
				other.longitude = latestUpdatesMap[otherID].currentLongitude;
				other.altitude = latestUpdatesMap[otherID].currentAltitude;

				//check for a conflict
				double d = distanceBetween(current, other);
				if(d < CONFLICT_THRESHOLD)
				{
					if (lastConflictTime.find(msg.planeID) == lastConflictTime.end()) {
						lastConflictTime[msg.planeID] = ros::Time::now();
					}

					if ((ros::Time::now()-lastConflictTime[msg.planeID]).toSec() > 1) {
						//we detected a conflict, increase counter and subtract some points
						numConflicts++;
						score = score + CONFLICT_SCORE;
					}

					lastConflictTime[msg.planeID] = ros::Time::now();
				}

				//check for collisions
				if(d < COLLISION_THRESHOLD)
				{

					if (lastCollisionTime.find(msg.planeID) == lastCollisionTime.end()) {
						lastCollisionTime[msg.planeID] = ros::Time::now();
					}

					if ((ros::Time::now()-lastCollisionTime[msg.planeID]).toSec() > 1) {
						// //fire & death awaits these two planes...
						// planesToDelete.push(id);
						// planesToDelete.push(otherID);

						//increment our collision counter
						numCollisions++;
					}

					lastCollisionTime[msg.planeID] = ros::Time::now();
				}
			}
		}//for each plane

		//we've parsed everything, delete some planes and display a new output to screen
		while(!planesToDelete.empty())
		{
			int id = planesToDelete.front();
			planesToDelete.pop();

			//we're already dead, nothing to do about it
			if(isDead[id]) continue;
			else
			{
				//delete the simulated plane
				//construct the service request
				RemovePlane srv;
				srv.request.sim = true;
				srv.request.planeID = id;

				//send the request
				printf("\nRequesting to delete plane... %d\n",id);
				if(removePlaneClient.call(srv))
				{
					printf("Plane with ID #%d has been deleted!\n", id);
				}
				else
				{
					ROS_ERROR("Did not receive a response from removePlaneClient");
				}

				//mark us dead
				timeOfDeath[id] = delta;
				isDead[id] = true;
				deadPlaneCount++;
			}

			//if our max plane ID is a dead plane, we no longer receive updates, so we need
			//to be waiting on a new max for updating the screen
			while(maxAlivePlane >= 0 && isDead[maxAlivePlane])
			{
				//decrement our valu
				maxAlivePlane--;
				ROS_ERROR("MAX: %d",maxAlivePlane);
			}

			//check to make sure not all planes are dead
			if(maxAlivePlane < 0)
			{
				//displayOutput();
				endEvaluation();
			}
		}

		//dump our info
		//displayOutput();

		//check for the end of times
		if((delta).toSec() > TIME_LIMIT)
		{
			//our time is up, time to write to files and wrap everything up
			endEvaluation();
		}
	}//if max planeID
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Evaluator");
	ros::NodeHandle n;
	Evaluator e;
	e.init(n);
	e.run();
	return 0;
}
