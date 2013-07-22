#ifndef _IPN_H_
#define _IPN_H_

#include "au_uav_ros/planeObject.h"
#include "au_uav_ros/simPlaneObject.h"
#include "au_uav_ros/vector2D.h"

#include "au_uav_ros/ripna.h"
#include "au_uav_ros/vmath.h"
#include "au_uav_ros/standardFuncs.h"

namespace au_uav_ros {
    namespace ipn {
        static double SEPARATION_THRESHOLD = 0.0;
        static double ZEM_THRESHOLD = 0.0;
        const static double T_GO_THRESHOLD = 10.0;
        const static double MU = 50.0;


        struct threatInfo {
            PlaneObject *threatPlane;
            Vector2D displacement, direction;
            double separationDistance, t_go, ZEM;
        };

        Vector2D getSeparationVector(const PlaneObject &plane1, const PlaneObject &plane2);
        Vector2D getDirectionVector(const PlaneObject &plane1, const PlaneObject &plane2);

        bool checkForThreats(SimPlaneObject &thisPlane, std::map<int, PlaneObject> &allPlanes, waypoint &avoidanceWP);
        threatInfo getThreatInfo(SimPlaneObject &thisPlane, PlaneObject &otherPlane);
        threatInfo* findGreatestThreat(std::vector<threatInfo> &allThreats);
        bool shouldTurnRight(SimPlaneObject &thisPlane, threatInfo &threat);
        waypoint createAvoidanceWaypoint(SimPlaneObject &thisPlane, threatInfo &threat);
        bool setWaypoint(SimPlaneObject &thisPlane, waypoint &wp);
    };
};


#endif
