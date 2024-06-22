// Point2D.h
#ifndef POINT2D_H
#define POINT2D_H

#include <cmath>
#include <vector>
#include <array>

struct pos{
    std::vector<float> w = {0,0,0,0};
    double x, y, h;
};

class Point2D {
private:
    float x;
    float y;
    float h;
public:
    // Constructors
    Point2D();               // Default constructor
    Point2D(float x, float y,float h); // Parameterized constructor

    // Getter methods
    float getX() const;
    float getY() const;
    float getH() const;

    // Setter methods
    void setX(float x);
    void setY(float y);
    void setH(float h);

    // Other member functions
    float distanceTo(const Point2D& other) const;
};

// Default constructor
Point2D::Point2D() : x(0), y(0),h(0) {}

// Parameterized constructor
Point2D::Point2D(float x, float y,float h) : x(x), y(y),h(h) {}

// Getter methods
float Point2D::getX() const {
    return x;
}

float Point2D::getY() const {
    return y;
}

float Point2D::getH() const {
    return h;
}

// Setter methods
void Point2D::setX(float x) {
    this->x = x;
}

void Point2D::setY(float y) {
    this->y = y;
}

void Point2D::setH(float h){
    this->h = h;
}
// Calculate distance to another point
float Point2D::distanceTo(const Point2D& other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    return std::sqrt(dx * dx + dy * dy);
}
#endif // POINT2D_H