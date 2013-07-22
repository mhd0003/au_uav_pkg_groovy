#ifndef _VECTOR2D_H_
#define _VECTOR2D_H_

#include "au_uav_ros/standardDefs.h"

namespace au_uav_ros
{
    class Vector2D
    {
    public:
        // Empty Constructor
        Vector2D();

        // Constructor for general use. Do not stick in a latitude and longitude here.
        Vector2D(double xIn, double yIn);

        // Constructor for geographic positions (latitude, longitude, altitude)
        Vector2D(waypoint wp1, waypoint wp2);

        // Getters and setters. Again, do not set these to latitude and longitude values
        double getX() const;
        double getY() const;
        void setX(double xIn);
        void setY(double yIn);

        // Getters for conversion factors
        double getLatToMeters() const;
        double getLonToMeters() const;

        // Calculate the magnitude and cartesian angle of this vector
        double getMagnitude() const;
        double getAngle() const;

        // Calculate the dot product with (*this) and otherVector
        double dot(const Vector2D& otherVector) const;

        // Add/Subtract two Vector2D objects
        const Vector2D operator+(const Vector2D& otherVector) const;
        const Vector2D operator-(const Vector2D& otherVector) const;
        Vector2D& operator+=(const Vector2D& otherVector);
        Vector2D& operator-=(const Vector2D& otherVector);

        // Multiply/divide by a scalar constant
        const Vector2D operator*(double scalar) const;
        const Vector2D operator/(double scalar) const;
        Vector2D& operator*=(double scalar);
        Vector2D& operator/=(double scalar);

    private:
        // Values of the first and second components of the vector
        double x, y;

        // Conversion factors for coordinates in this area (within 0.5 degrees)
        double latToMeters, lonToMeters;
    };
};

#endif
