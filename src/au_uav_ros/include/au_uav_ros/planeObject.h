/* PlaneObject */

#ifndef PLANE_OBJECT_H
#define PLANE_OBJECT_H

#include "au_uav_ros/TelemetryUpdate.h"
#include "au_uav_ros/standardDefs.h"
#include "au_uav_ros/standardFuncs.h"

namespace au_uav_ros {

	class PlaneObject {

        public:
            /* Default constructor. Sets everything to zero. */
            PlaneObject(void);
            
            /* Explicit value constructor: Takes a collision radius and a
            telemetry update and creates a new PlaneObject. */
            PlaneObject(double cRadius, const au_uav_ros::TelemetryUpdate &msg);

            /* Mutator functions */
            void setID(int id);
            void setPreviousLoc(double lat, double lon, double alt);
            void setCurrentLoc(double lat, double lon, double alt);
            void setTargetBearing(double tBearing);		/* set bearing to destination */
            void setCurrentBearing(double cBearing); 	/* set current bearing in the air */
            void setSpeed(double speed);
            void setDestination(const au_uav_ros::waypoint &destination);

            void updateTime(void);

            /* Update the plane's data members with the information contained within the telemetry update. */
            void update(const au_uav_ros::TelemetryUpdate &msg);

            /* Accessor functions */
            int getID(void) const;
            au_uav_ros::coordinate getPreviousLoc(void) const;
            au_uav_ros::coordinate getCurrentLoc(void) const;
            double getTargetBearing(void) const;
            double getCurrentBearing(void) const;
            double getSpeed(void) const;
            double getLastUpdateTime(void) const;
            au_uav_ros::waypoint getDestination(void) const;
            

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

        private:
            /* Private data members */
            int id;
            double collisionRadius;
            double targetBearing;		/* get bearing to destination */
            double currentBearing;		/* get current bearing in the air */
            double speed;
            double lastUpdateTime;
            au_uav_ros::coordinate previousLoc;	/*used to calculate currentBearing*/
            au_uav_ros::coordinate currentLoc;
            au_uav_ros::waypoint destination;
    };
};

#endif
