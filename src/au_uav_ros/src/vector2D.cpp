/* Vector2D
 *
 * This class serves the same purpose as vmath.cpp, but stores the vector
 * components rather than a magnitude and direction. This class uses the
 * Cartesian coordinate system rather than the polar coordinate system in vmath.cpp.
 */

#include "au_uav_ros/vector2D.h"
using namespace au_uav_ros;

// Empty Constructor
Vector2D::Vector2D() {
    this->x = 0;
    this->y = 0;
}

// Constructor for general use. Do not stick in a latitude and longitude here.
Vector2D::Vector2D(double xIn, double yIn) {
    this->x = xIn;
    this->y = yIn;
}

// Constructor for geographic positions (latitude, longitude, altitude)
Vector2D::Vector2D(waypoint wp1, waypoint wp2) {
    // Set up reference latitude (in radians) as average of the two points
    double lat = (wp1.latitude+wp2.latitude)/2 * DEGREES_TO_RADIANS;

    // Find the difference in latitude and longitude
    double deltaLat = (wp1.latitude - wp2.latitude);
    double deltaLon = (wp1.longitude - wp2.longitude);

    // Assuming Earth is a perfect sphere...
    // 1 degree of latitude is circumference of the earth divided by 360
    // EARTH_RADIUS = 6371000. (2*pi*6371000) / 360 = 111,194.9266.
    // 1 degree of longitude is circumference of earth *at this latitude* divided by 360
    // radius = EARTH_RADIUS * cos(lat).
    this->latToMeters = 111194.926644559;
    this->lonToMeters = this->latToMeters * cos(lat);

    // Get difference in degrees, then multiply by conversion factor
    this->x = deltaLon * lonToMeters;
    this->y = deltaLat * latToMeters;
}


// Getters and setters. Again, do not set these to latitude and longitude values
double Vector2D::getX() const {
    return this->x;
}

double Vector2D::getY() const {
    return this->y;
}

void Vector2D::setX(double xIn) {
    this->x = xIn;
}

void Vector2D::setY(double yIn) {
    this->y = yIn;
}


// Getters for conversion factors
double Vector2D::getLatToMeters() const {
    return this->latToMeters;
}

double Vector2D::getLonToMeters() const {
    return this->lonToMeters;
}


// Calculate the magnitude of this vector
double Vector2D::getMagnitude() const {
    return sqrt(x*x + y*y);
}
// Calculate the Cartesian angle, in degrees
double Vector2D::getAngle() const {
    // double angle = atan2(this->y, this->x) * RADIANS_TO_DEGREES;
    return atan2(this->y, this->x) * RADIANS_TO_DEGREES;
}


// Calculate the dot product with (*this) and otherVector
double Vector2D::dot(const Vector2D& otherVector) const {
    return this->x*otherVector.x + this->y*otherVector.y;
}


// Add/Subtract two Vector2D objects
const Vector2D Vector2D::operator+(const Vector2D& otherVector) const {
    Vector2D result;

    result.setX(this->x + otherVector.x);
    result.setY(this->y + otherVector.y);

    return result;
}

const Vector2D Vector2D::operator-(const Vector2D& otherVector) const {
    Vector2D result;

    result.setX(this->x - otherVector.x);
    result.setY(this->y - otherVector.y);

    return result;
}

Vector2D& Vector2D::operator+=(const Vector2D& otherVector) {
    this->x += otherVector.x;
    this->y += otherVector.y;

    return *this;
}

Vector2D& Vector2D::operator-=(const Vector2D& otherVector) {
    this->x -= otherVector.x;
    this->y -= otherVector.y;

    return *this;
}


// Multiply/divide by a scalar constant
const Vector2D Vector2D::operator*(double scalar) const {
    Vector2D result;

    result.setX(this->x * scalar);
    result.setY(this->y * scalar);

    return result;
}

const Vector2D Vector2D::operator/(double scalar) const {
    Vector2D result;

    result.setX(this->x / scalar);
    result.setY(this->y / scalar);

    return result;
}

Vector2D& Vector2D::operator*=(double scalar) {
    this->x *= scalar;
    this->y *= scalar;

    return *this;
}

Vector2D& Vector2D::operator/=(double scalar) {
    this->x /= scalar;
    this->y /= scalar;

    return *this;
}
