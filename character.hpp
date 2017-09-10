#ifndef CHARACTER_HPP
#define CHARACTER_HPP

#include <map>
#include <string>
#include <vector>
#include <glm/ext.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include "draw.hpp"
#include "reader.hpp"
using namespace std;
using glm::vec3;
using glm::mat4;

// Forward declarations
class Bone;
class RotationBounds;

// This is the root class for the animated character. You can also
// think of this as a root node a scene graph. The class takes care of
// loading the skeleton file and also animation file(s) needed to draw
// the skeleton. The key routine that you need to code is the draw(..)
// routine. In it, you will want to draw all of the character's bones
// in the correct positions and orientations based on the current
// animation data.
class Character {
    public:

        Character(std::string asfFilename, std::string amcFilename,
                vec3 basePosition, vec3 baseVelocity);

        // Advance the mocap data by a time dt. Note that this need not be
        // the same as the animation time in the program, if you want to
        // play back the mocap animation at a different speed from what it
        // was recorded. The frame rate of the mocap data is 120 fps, so
        // if you want to advance exactly one mocap frame you should use
        // dt = 1/120.f.
        void advance(float dt);

        // This returns the current coordinate frame of the ROOT NODE of
        // the character, typically this is the character's pelvis -- all
        // of the root node bones should be drawn relative to this
        // coordinate frame. This will change every frame of the
        // animation.
        mat4 getCurrentCoordinateFrame();

        // This returns just the current position of the ROOT NODE.
        vec3 getCurrentPosition();

        // This is an array of pointers to bones. It will contain one or
        // more bones that are attached to the root node of the character.
        // Each of these bones will, in turn, have 0 or more child bones.
        vector<Bone*> rootNodeBones;

        // TODO: Implement this routine to draw all the character's bones
        // in the correct pose based on the current animation data.
        void draw();

        bool hasAnimation() {return !anim->fail();}
        bool hasSkeleton() {return !boneTable.empty();}

    protected:
        void loadAnimation(std::string amcFilename);
        void loadSkeleton(std::string asfFilename);  
        void resetFile();
        void nextFrame();
        // float deg2rad(float d);
        void parseUnits(Reader &r);
        void parseRoot(Reader &r);
        void parseBonedata(Reader &r);
        void parseHierarchy(Reader &r);
        bool deg;
        float time;
        vec3 position;
        vec3 orientation;
        int animationFrame;
        vec3 basePosition, baseVelocity; // to compensate for translation in amc
        std::map<string, Bone*> boneTable;
        std::ifstream *anim;
};

// This class just provides a data structure to store information
// about how each bone can move, including whether it can rotate in
// the x, y, and z directions (some joints support rotation along all
// axes, others are just 1-dimensional) and the min and max angles of
// rotation for each axis.
class RotationBounds {
    public:
        RotationBounds();  
        void setdof(bool rx, bool ry, bool rz);
        void setR(int index, float min, float max);
        bool dofRX;
        bool dofRY;
        bool dofRZ;
        int dofs;
        float minRX;
        float maxRX;
        float minRY;
        float maxRY;
        float minRZ;
        float maxRZ;
};

// This class holds the data for a single articulated joint and bone
// of an animated character. It also stores a list of 0 or more child
// bones. So, it can be used to create a scenegraph for an animated
// character.
class Bone {
    public:

        // This constructor is setup to read data from the CMU motion
        // capture database files.
        Bone(Reader &r, bool deg);

        // Bones are named based on parts of the body
        std::string getName();    

        // Returns the rotation of this joint in the body.
        mat4 getCurrentLocalRotation();

        // Returns a vector that is scaled to the length of the bone and
        // points in the direction of the bone. In the bone's local
        // coordinate frame, you can think of the bone as beginning at
        // (0,0,0) and ending at the postion (0,0,0) + getBoneVector().
        vec3 getBoneVector() { return length * direction; }

        // An array of pointers to the other bones that are children of this
        // one in the scenegraph. All children attach to the end of the
        // bone.
        std::vector<Bone*> children;

        // This is the key routine that you need to fill in.
        void draw();

        void addChild(Bone* child);
        void readPose(Reader &r);
    protected:
        //TaperedCylinder *cylinder;
        void constructFromFile(Reader &r, bool deg);
        // float deg2rad(float d);
        std::string name;
        float length;
        vec3 direction;
        RotationBounds rotationBounds;    
        vec3 axis;
        mat4 initialRotation;
        mat4 currentRotation;
        int id;
        bool deg;
};

inline Character::Character(std::string asfFilename, std::string amcFilename,
        vec3 basePosition, vec3 baseVelocity) {
    anim = NULL;
    time = 0;
    loadSkeleton(asfFilename);
    loadAnimation(amcFilename);
    this->basePosition = basePosition;
    this->baseVelocity = baseVelocity;
}

inline void Character::advance(float dt) {
    float fps = 120;
    int f0 = round(fps*time), f1 = round(fps*(time + dt));
    for (int f = f0; f < f1; f++)
        nextFrame();
    time += dt;
}

inline mat4 Character::getCurrentCoordinateFrame() {
    mat4 frame;
    frame = glm::translate(frame, position);
    frame = glm::rotate(frame, glm::radians(orientation.z), vec3(0,0,1));
    frame = glm::rotate(frame, glm::radians(orientation.y), vec3(0,1,0));
    frame = glm::rotate(frame, glm::radians(orientation.x), vec3(1,0,0));
    return frame;
}

inline vec3 Character::getCurrentPosition() {
    return position;
}

inline void Character::draw() {
    glPushMatrix();

    // TODO: Apply the current coordinate frame and then draw the root
    // node bones of the character.
    mat4 f = getCurrentCoordinateFrame();
    glMultMatrixf(&f[0][0]);
    Draw::sphere(vec3(0,0,0), .05);
    // glTranslatef(position.x, position.y, position.z);
    int num_children = rootNodeBones.size();
    for (int c = 0; c < num_children; c++) {
        rootNodeBones[c]->draw();
    }

    glPopMatrix();
}

inline Bone::Bone(Reader &r, bool deg) {
    constructFromFile(r, deg);
}

inline void Bone::draw() {
    glPushMatrix();

        // TODO: Draw the bone as a capsule (a cylinder capped by
        // spheres). Translate to the end of the bone vector and draw the
        // bone's children, recursively.
        mat4 f = getCurrentLocalRotation();
        glMultMatrixf(&f[0][0]);
        vec3 bnorm = glm::normalize(getBoneVector());
        vec3 bone = getBoneVector();

        glPushMatrix();
            float angle = acos(glm::dot(bnorm, vec3(0, 0, 1)));
            vec3 norm = glm::cross(vec3(0, 0, 1), bnorm);
            glRotatef(angle * 180.0 / M_PI, norm.x, norm.y, norm.z);

            glPushMatrix();
                glScalef(.05, .05, length);
                Draw::unitCylinderZ();
            glPopMatrix();

        glPopMatrix();

        glTranslatef(bone.x, bone.y, bone.z);
        Draw::sphere(vec3(0,0,0), .05);

        int num_children = children.size();
        for (int c = 0; c < num_children; c++) {
            children[c]->draw();
        }

    glPopMatrix();
}

// The rest of the Character, Bone, RotationBounds implementation is
// in character_impl.hpp. You should not need to modify it.

#include "character_impl.hpp"

#endif
