#ifndef SPLINE_HPP
#define SPLINE_HPP

#include <glm/glm.hpp>
#include <vector>
#include <algorithm>
using glm::vec3;

class SplinePoint3 {
public:
    float t;    // time
    vec3 p, dp; // value and derivative
    SplinePoint3(float t, vec3 p, vec3 dp): t(t), p(p), dp(dp) {}
};

class Spline3 {
public:
    // list of spline control points. assumed to be in increasing order of t
    std::vector<SplinePoint3> points;

    // value of t for first and last control points
    float minTime() {return points.front().t;}
    float maxTime() {return points.back().t;}

    // returns i such that t lies between points[i].t and points[i+1].t.
    // also modifies t to lie in [minTime(), maxTime()] if necessary.
    int findSegment(float &t);

    // returns value of spline function at time t
    vec3 getValue(float t);

    // returns d/dt of spline function at time t
    vec3 getDerivative(float t);
};

inline int Spline3::findSegment(float &t) {

    // TODO: If t is outside the range [minTime(), maxTime()], replace
    // it with the closest time in that range. Then, find the segment
    // that contains t, so that you can perform cubic Hermite
    // interpolation within it.
    t = max(minTime(), min(maxTime(), t));
    int n = points.size();
    int i;
    for (i = 0; i < (n - 1) && !(points[i].t <= t && t <= points[i+1].t); i++);
    return i;
}

inline vec3 Spline3::getValue(float t) {

    // TODO: Find the segment that contains t, and use the cubic
    // Hermite interpolation formula to find the interpolated value
    // within it. Note that the formula discussed in class is only
    // valid for t0 = 0, t1 = 1, so you will have to use a modified
    // formula.
    int seg = findSegment(t);
    float tRange = (points[seg + 1].t - points[seg].t);
    t = (t - points[seg].t) / tRange;
    vec3 p0 = points[seg].p;
    vec3 p0d = points[seg].dp * tRange;
    vec3 p1 = points[seg + 1].p;
    vec3 p1d = points[seg + 1].dp * tRange;
    float t3 = t*t*t;
    float t2 = t*t;
    return (2*t3 - 3*t2 + 1)*p0 + (t3 - 2*t2 + t)*p0d + (-2*t3 + 3*t2)*p1 + (t3 - t2)*p1d;
}

inline vec3 Spline3::getDerivative(float t) {

    // TODO: Find the segment that contains t. Differentiate the cubic
    // Hermite interpolation formula to find the derivative of the
    // spline function. Be careful about how rescaling affects
    // derivatives.
    int seg = findSegment(t);
    float tRange = (points[seg + 1].t - points[seg].t);
    t = (t - points[seg].t) / tRange;
    vec3 p0 = points[seg].p;
    vec3 p0d = points[seg].dp * tRange;
    vec3 p1 = points[seg + 1].p;
    vec3 p1d = points[seg + 1].dp * tRange;
    float t3 = t*t*t;
    float t2 = t*t;
    // f'(t) = 3at^2                       +        2bt                        + c
    vec3 ret = 3*(2*p0 + p0d - 2*p1 + p1d)*t2  +  2*(-3*p0 - 2*p0d + 3*p1 - p1d)*t + p0d;
    return ret / tRange;
    // return (6*t2 - 6*t)*p0 + (3*t2 - 4*t + 1)*p0d + (-6*t2 + 6*t)*p1 + (3*t2 - 2*t)*p1d;

}

#endif
