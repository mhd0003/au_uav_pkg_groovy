/*
Simulation
- Singleton Class to create WIND and GPS Error Simulation models
*/

#ifndef SIMULATION_H
#define SIMULATION_H

#include "au_uav_ros/standardDefs.h"
#include <au_uav_ros/WindSimulation.h>
#include <au_uav_ros/GPSErrorSimulation.h>

namespace au_uav_ros
{

  class CSimulation {

  public:
    // Singleton Access functions
    static au_uav_ros::CSimulation& GetInstance();

    // Function to update the simulation values
    void UpdateSimulatedValues(double&,double&,double&);

    // 	Get Distance and Bearing between tow lat/longs
    void GetDistanceAndBearing(double lat1,double lat2,
			       double long1,double long2,
			       double& newLat, double& newLong,
			       double& actualBearing,
			       double& bearing,
			       double& distanceToDesitnation,
			       double& groundSpeed);
  private:
    // Constructor
    CSimulation() {
      wind_model = new au_uav_ros::CWindSimulation(gps_error_model);
    };
  
    CSimulation(const CSimulation&);
    CSimulation& operator=(const CSimulation&);
    
    
    // WindSimulation Object
    au_uav_ros::CWindSimulation* wind_model;
    
    // GPS Error Simulation Object
    au_uav_ros::CGPSErrorSimulation gps_error_model;

    // Calculate distance and bearing using Haversines
    void HaversinesCalculation(double lat1,double lat2,
			       double long1,double long2,
			       double& newLat,double& newLong,
			       double& actualBearing,
			       double& bearing,
			       double& distanceToDesitnation,
			       double& groundSpeed);

    // Calculate distance and bearing using Geodetic Datums
    void GeodeticCalculation(double lat1,double lat2,
			     double long1,double long2,
			     double& newLat,double& newLong,
			     double& actualBearing,
			     double& bearing,
			     double& distanceToDesitnation,
			     double& groundSpeed);

    // Utility Function to check the turning radius
    double CheckTurningRadius(const double,double);
  };
}

#endif
