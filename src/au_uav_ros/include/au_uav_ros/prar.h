#ifndef _PRAR_H_
#define _PRAR_H_

#include "au_uav_ros/standardDefs.h"
#include "au_uav_ros/planeObject.h"

#include "../src/astar_sparse0.cpp"
#include "au_uav_ros/discretizedPlane.h"
#include "au_uav_ros/dangerGrid.h"
#include "au_uav_ros/position.h"
#include "au_uav_ros/mapTools.h"


#ifndef _BEARING_T_
#define _BEARING_T_
namespace au_uav_ros {
	enum bearing_t { N, NE, E, SE, S, SW, W, NW };
};
#endif

#ifndef _POINT_STRUCT_
#define _POINT_STRUCT_
namespace au_uav_ros {
	struct point {
		int x;
		int y;
		int t;
		bearing_t b;
	};
};
#endif

namespace au_uav_ros {
	namespace prar {
		typedef std::vector<waypoint> waypointVector;

		typedef std::map<int, PlaneObject> PlaneObjectMap;
		typedef std::map<int, DiscretizedPlane> DiscretePlaneMap;
		typedef std::map<int, waypointVector> WaypointVectorMap;

		// md
		// Called in coordinator
		waypointVector getAvoidancePlan(int planeID, PlaneObjectMap &allPlanes, WaypointVectorMap &allPlanesWaypoints);


		void astarPathPlan(DiscretePlaneMap &discretePlanes, int planeID);

		std::queue<point> findLocalAstarPath(DiscretePlaneMap &discretePlanes, int planeID, int lastWaypointTime, int collisionTime);
		void generateMapValuesFromWaypoints(map_tools::gridValues &gridValsOut, WaypointVectorMap &allPlanesWaypoints);

		// Methods to convert between PlaneObject map to DiscritizedPlane map
		void convertAllPlanesAndAddWaypoints(PlaneObjectMap &allPlanes, DiscretePlaneMap &discretePlanes, WaypointVectorMap &allPlanesWaypoints);
		DiscretizedPlane convertRealPlane(const PlaneObject &continuousPlane, const waypointVector &allWaypoints);
	};
};

#endif