/*
GPSErrorSimulation
- Model for the GPS Error simulation
*/

#ifndef GPS_SIMULATION_H
#define GPS_SIMULATION_H

#include <noise/noise.h>
using namespace noise;

namespace au_uav_ros
{

  class CGPSErrorSimulation {
  public:
    // Constructor
    CGPSErrorSimulation();

    // Function to generate the error to model the receiver noise
    float correction();  

  private:
    // Function to calculate a uniform random noise value
    float uniform_random_noise();

    // Function to calculate a non-coherent noise value
    float white_noise_value();

    // Function to calculate a non-coherent noise value
    float coherent_noise_value();

    // Function to calculate a moving average
    double calculate_moving_average(double new_value);

    // Error sign, used in uniform random noise and white noise to get initial direction
    int error_sign;

    // GPS Noise Model
    int gps_noise_type;

    // Lbnoise module
    module::Perlin coherent_noise;
    int noise_counter;

    // Function to get value from environment
    int get_env_value(const char* ENV_VARIABLE);
  };
}

#endif
