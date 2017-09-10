#ifndef CAMERA_HPP
#define CAMERA_HPP

#define _USE_MATH_DEFINES
#include <cmath>
#include "graphics.hpp"
using glm::vec3;

class Perspective {
public:
    Perspective(float fov = 90, float aspect = 1,
                float zmin = 0.1, float zmax = 10):
        fov(fov), aspect(aspect), zmin(zmin), zmax(zmax) {}
    void apply();
protected:
    float fov, aspect, zmin, zmax;
};

class OrbitCamera {
public:
    OrbitCamera(float dist = 1, float lat = 0, float lon = 0,
                Perspective pers = Perspective()):
        dist(dist), lat(lat), lon(lon), pers(pers), center(0,0,0) {}
    void apply();
    vec3 getCenter();
    void setCenter(vec3 center);
    void onMouseMotion(SDL_MouseMotionEvent&);
protected:
    vec3 center;
    float dist, lat, lon;
    Perspective pers;
};

// Definitions below

inline void Perspective::apply() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, aspect, zmin, zmax);
}

inline void OrbitCamera::apply() {
    pers.apply();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    vec3 eye = center + dist*vec3(sin(lon)*cos(lat), sin(lat), cos(lon)*cos(lat));
    gluLookAt(eye.x,eye.y,eye.z, center.x,center.y,center.z, 0,1,0);
}

inline vec3 OrbitCamera::getCenter() {
    return center;
}

inline void OrbitCamera::setCenter(vec3 center) {
    this->center = center;
}

inline void OrbitCamera::onMouseMotion(SDL_MouseMotionEvent &e) {
    if (!(e.state & SDL_BUTTON_LMASK))
        return;
    lon -= 0.005f*e.xrel;
    lat += 0.005f*e.yrel;
    if (lat < -M_PI/2 + 0.001)
        lat = -M_PI/2 + 0.001;
    if (lat > M_PI/2 - 0.001)
        lat = M_PI/2 - 0.001;
}

#endif
