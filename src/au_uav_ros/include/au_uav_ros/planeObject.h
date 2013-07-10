#ifndef _PLANE_OBJECT_H_
#define _PLANE_OBJECT_H_

#include "ros/ros.h"
#include <math.h>

#include "au_uav_ros/standardDefs.h"
#include "au_uav_ros/standardFuncs.h"

//Message Includes
#include "au_uav_ros/Command.h"
#include "au_uav_ros/Telemetry.h"

namespace au_uav_ros {
	class PlaneObject {
	public:
            /* Default constructor. Sets everything to zero. */
            PlaneObject(void);

	    PlaneObject(struct waypoint wp);

	    PlaneObject(int _id);
            
            /* Explicit value constructor: Takes a collision radius and a
            telemetry update and creates a new PlaneObject. */
            PlaneObject(double cRadius, const Telemetry &msg);

            /* Mutator functions */
            void setID(int id);
            void setPreviousLoc(double lat, double lon, double alt);
            void setCurrentLoc(double lat, double lon, double alt);
            void setTargetBearing(double tBearing);		/* set bearing to destination */
            void setCurrentBearing(double cBearing); 	/* set current bearing in the air */
            void setSpeed(double speed);
            //void setDestination(const waypoint &destination);

            void updateTime(void);

            /* Update the plane's data members with the information contained within the telemetry update. */
            bool update(const Telemetry &msg, Command &newCommand);

            /* Accessor functions */
            int getID(void) const;
            waypoint getPreviousLoc(void) const;
            waypoint getCurrentLoc(void) const;
            double getTargetBearing(void) const;
            double getCurrentBearing(void) const;
            double getSpeed(void) const;
            double getLastUpdateTime(void) const;
            waypoint getDestination(void) const;
            virtual double getSimSpeed(void) const;

            /* Find distance between this plane and another plane */
            double findDistance(const PlaneObject& plane) const;
            /* Find distance between this plane and another plane's latitude/longitude */
            double findDistance(double lat2, double lon2) const;

            /* Find Cartesian angle between this plane and another plane */
            double findAngle(const PlaneObject& plane) const;
            /* Find Cartesian angle between this plane and another plane's latitude/longitude */
            double findAngle(double lat2, double lon2) const;

            /* Overloaded equality operator */
            PlaneObject& operator=(const PlaneObject& pobj);

            /* Returns true if a plane object is within the cRadius meters of this plane object, false otherwise */
            bool isColliding(const PlaneObject& planeObj) const;

	    void addAvoidanceWp(struct waypoint wp);
	    void addNormalWp(struct waypoint wp);
	    void removeAvoidanceWp(void);
	    void removeNormalWp(struct waypoint wp);
	    au_uav_ros::Command getPriorityCommand(void);

        protected:
            /* Private data members */
            int id;
            double collisionRadius;
            double targetBearing;		/* get bearing to destination */
            double currentBearing;		/* get current bearing in the air */
            double speed;
            double lastUpdateTime;
            waypoint previousLoc;	/*used to calculate currentBearing*/
            waypoint currentLoc;
		std::list<struct waypoint> normalPath;
		waypoint avoidWp;
	};
}
#endif
