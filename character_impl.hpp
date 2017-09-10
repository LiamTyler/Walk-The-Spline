#ifndef CHARACTER_IMPL_HPP
#define CHARACTER_IMPL_HPP

#include <map>
#include <string>
#include <vector>
#include <glm/gtc/matrix_transform.hpp>
#include "reader.hpp"
using namespace std;
using glm::vec3;
using glm::mat4;

mat4 fromEulerAnglesZYX(float degz, float degy, float degx) {
    mat4 r;
    r = glm::rotate(r, glm::radians(degz), vec3(0,0,1));
    r = glm::rotate(r, glm::radians(degy), vec3(0,1,0));
    r = glm::rotate(r, glm::radians(degx), vec3(1,0,0));
    return r;
}

template <typename T>
T amc2meter(T t) {
  return t * 0.056444f;
}

inline void Character::loadSkeleton(std::string asfFilename) {
    std::ifstream in(asfFilename.c_str());
    Reader r(&in);  
    while (r.good()) {
        if (r.expect("#")) {
            std::cerr << "Ignoring comment line" << std::endl;
            r.swallowLine();
        }    
        else if (r.expect(":version")) {
            r.swallowLine();
        }    
        else if (r.expect(":name")) {
            std::cerr << "Swallowing name" << std::endl;
            r.swallowLine();
        }    
        else if (r.expect(":units")) {
            std::cerr << "Reading units" << std::endl;
            parseUnits(r);
        }    
        else if (r.expect(":documentation")) {
            std::cerr << "Reading documentation" << std::endl;
            while (r.good() && !r.peek(":")) {
                r.swallowLine();
            }
        }    
        else if (r.expect(":root")) {
            std::cerr << "Reading root" << std::endl;
            parseRoot(r);
        }    
        else if (r.expect(":bonedata")) {
            std::cerr << "Reading bonedata" << std::endl;
            parseBonedata(r);
        }    
        else if (r.expect(":hierarchy")) {
            std::cerr << "Reading hierarchy" << std::endl;
            parseHierarchy(r);
        }    
        else {      
            std::string tok;
            r.readToken(tok);
            if (!r.good()) {
                break;
            }
            std::cerr << "Encountered unknown token" << std::endl;
            std::cerr << "'" << tok << "'" << std::endl;
            std::abort();
        }
    } // end while (looping over file) 
}

inline void Character::parseUnits(Reader &r) {
    bool cont;
    do {
        cont = false;    
        if (r.expect("mass")) {
            float trash;
            r.readFloat(trash);
            cont = true;
        }    
        if (r.expect("length")) {
            float trash;
            r.readFloat(trash);
            cont = true;
        }    
        if (r.expect("angle")) {
            std::string token;
            r.readToken(token);
            if (token == "deg") {
                deg = true;
            }
        }    
    } while (cont);
}

inline void Character::parseRoot(Reader &r) {
    bool cont;
    do {
        cont = false;    
        if (r.expect("order")) {
            cont = true;
            if (!r.expect("TX TY TZ RX RY RZ")) {
                std::cerr << "'order' not in order expected" << std::endl;
                std::abort();
            }
        }    
        if (r.expect("axis")) {
            cont = true;
            if (!r.expect("XYZ")) {
                std::cerr << "'axis' not in order expected" << std::endl;
                std::abort();
            }
        }    
        if (r.expect("position")) {
            cont = true;
            r.readFloat(position.x);
            r.readFloat(position.y);
            r.readFloat(position.z);
            position = amc2meter(position);
        }    
        if (r.expect("orientation")) {
            cont = true;
            r.readFloat(orientation.x);
            r.readFloat(orientation.y);
            r.readFloat(orientation.z);
        }    
    } while (cont);
}

inline void Character::parseBonedata(Reader &r) {
    while (r.expect("begin")) {
        Bone *bone = new Bone(r, deg);
        boneTable[bone->getName()] = bone;
    }
}

inline void Character::parseHierarchy(Reader &r) {
    if (!r.expect("begin")) {
        std::cerr << "Reading hierarchy, expected 'begin', not found" << std::endl;
        std::abort();
    }
    while (!r.expect("end")) {
        std::string line;
        std::string parent;
        r.readToken(parent);
        r.readLine(line);    
        std::stringstream ss(line);
        std::string child;
        ss >> child;
        while (ss) {
            if (parent == "root") {
                rootNodeBones.push_back(boneTable[child]);
            }
            else {
                boneTable[parent]->addChild(boneTable[child]);
            }
            ss >> child;
        }
    }
}

inline void Character::loadAnimation(std::string amcFilename) {
    if (anim)
        delete anim;
    anim = new std::ifstream(amcFilename.c_str());
    resetFile();
    nextFrame();
}

inline void Character::nextFrame() {
    animationFrame++;
    Reader r(anim);
    int frame;
    r.readInt(frame);
    if (!r.good()) {
        resetFile();
        return;
    }
    while (!r.upcomingInt()) {
        std::string bone;
        r.readToken(bone);
        if (!r.good()) {
            resetFile();
            return;
        }
        if (bone == "root") {
            r.readFloat(position.x);
            r.readFloat(position.y);
            r.readFloat(position.z);
            position = amc2meter(position);
            position -= basePosition + baseVelocity*animationFrame/120.f;
            r.readFloat(orientation.x);
            r.readFloat(orientation.y);
            r.readFloat(orientation.z);
        } 
        else {
            boneTable[bone]->readPose(r);
        }
    }
}

inline void Character::resetFile() {
    if (!anim) {
        return;
    }
    animationFrame = 0;
    anim->clear();
    anim->seekg(0);
    Reader r(anim);
    r.swallowLine();
    r.swallowLine();
    r.swallowLine();
}

inline RotationBounds::RotationBounds() {
    dofRX = false;
    dofRY = false;
    dofRZ = false;
    minRX = 0;
    maxRX = 0;
    minRY = 0;
    maxRY = 0;
    minRZ = 0;
    maxRZ = 0;
    dofs = 0;
}

inline void RotationBounds::setdof(bool rx, bool ry, bool rz) {
    dofRX = rx;
    dofRY = ry;
    dofRZ = rz;
    dofs = rx + ry + rz;
}

inline void RotationBounds::setR(int index, float min, float max) {
    if (index > dofs) {
        std::abort(); // Trying to set past the max index.
    }
    if (index == 0) {
        if (dofRX) {
            minRX = min;
            maxRX = max;
        } else if (dofRY) {
            minRY = min;
            maxRY = max;
        } else if (dofRZ) {
            minRZ = min;
            maxRZ = max;
        } else {
            abort();
        }
    } else if (index == 1) {
        if (dofRX && dofRY) {
            minRY = min;
            maxRY = max;
        } else if (dofRX && dofRZ) {
            minRZ = min;
            maxRZ = max;
        } else if (dofRY && dofRZ) {
            minRZ = min;
            maxRZ = max;
        } else {
            abort();
        }
    } else if (index == 2) {
        if (dofRX && dofRY && dofRZ) {
            minRZ = min;
            maxRZ = max;
        } else {
            abort();
        }
    } else {
        abort();
    }
}

const bool ABORT_ON_ERROR=true;

inline void assume(bool b) {
    if (!b && ABORT_ON_ERROR) {
        std::abort();
    }
}

inline void Bone::constructFromFile(Reader &r, bool deg) {
    this->deg = deg;
    currentRotation = mat4();
    while (!r.expect("end")) {    
        if (r.expect("id")) {
            r.readInt(id);      
        }    
        if (r.expect("name")) {
            r.readToken(name);
        }    
        if (r.expect("direction")) {
            r.readFloat(direction.x);
            r.readFloat(direction.y);
            r.readFloat(direction.z);
        }    
        if (r.expect("length")) {
            r.readFloat(length);
            length = amc2meter(length);
        }    
        if (r.expect("axis")) {
            float ax, ay, az;
            std::string axisType;
            r.readFloat(ax);
            r.readFloat(ay);
            r.readFloat(az);
            r.readToken(axisType);
            if (axisType == "XYZ") {
                initialRotation = fromEulerAnglesZYX(az, ay, ax);
            } else {
                std::abort();
            }      
        }    
        if (r.expect("dof")) {
            bool rx, ry, rz;
            rx = r.expect("rx");
            ry = r.expect("ry");
            rz = r.expect("rz");
            rotationBounds.setdof(rx, ry, rz);
        }    
        if (r.expect("limits")) {
            for (int dof=0; dof<rotationBounds.dofs; dof++) {
                assume(r.expect("("));
                float min, max;
                r.readFloat(min);
                r.readFloat(max);
                assume(r.expect(")"));
                rotationBounds.setR(dof, min, max);
            }
        }    
    } // read "end" token  
    vec3 skin(0.8, 0.7, 0.4);
    vec3 shirt(1.0, 0.07, 0.57);
    vec3 pants(0.19, 0.31, 0.31);
    vec3 shoes(0.9, 0.9, 0.8);  
    float r1=0.02, r2=0.01;
    vec3 c1 = vec3(1,1,1), c2 = vec3(1,1,1);
    if (name == "lhipjoint" || name == "rhipjoint") {
        r1 = 0.09;
        r2 = 0.05;
        c1 = pants;
        c2 = pants;
    }
    if (name == "lfemur" || name == "rfemur") {
        r1 = 0.05;
        r2 = 0.05;
        c1 = pants;
        c2 = pants;
    }
    if (name == "ltibia" || name == "rtibia") {
        r1 = 0.05;
        r2 = 0.04;
        c1 = pants;
        c2 = pants;
    } 
    if (name == "lfoot" || name == "rfoot") {
        r1 = 0.06;
        r2 = 0.05;
        c1 = shoes;
        c2 = shoes;
    }
    if (name == "ltoes" || name == "rtoes") {
        r1 = 0.05;
        r2 = 0.03;
        c1 = shoes;
        c2 = shoes;
    }
    if (name == "lowerback" || name == "upperback" || name == "thorax") {
        r1 = 0.10;
        r2 = 0.10;
        c1 = shirt;
        c2 = shirt;
    }
    if (name == "lowerback") {
        r1 = 0.08;
    }
    if (name == "lowerneck" || name == "upperneck") {
        r1 = 0.04;
        r2 = 0.04;
        c1 = skin;
        c2 = skin;
    }
    if (name == "head") {
        r1 = 0.06;
        r2 = 0.08;
        c1 = skin;
        c2 = skin;
    }
    if (name == "lclavicle" || name == "rclavicle") {
        r1 = 0.05;
        r2 = 0.05;
        c1 = shirt;
        c2 = shirt;
    }
    if (name == "lhumerus" || name == "rhumerus" || name == "lradius" || name == "rradius") {
        r1 = 0.03;
        r2 = 0.025;
        c1 = skin;
        c2 = skin;
    }
    if (name == "lwrist" || name == "rwrist") {
        r1 = 0.025;
        r2 = 0.02;
        c1 = skin;
        c2 = skin;
    }
    if (name == "lhand" || name == "rhand" || name == "lthumb" || name == "rthumb" || name == "rfingers" || name == "lfingers") {
        r1 = 0.025;
        r2 = 0.02;
        c1 = skin;
        c2 = skin;
    }
    //cylinder = new TaperedCylinder(length, r1, r2, c1, c2);
}

inline std::string Bone::getName() {
    return name;
}

inline void Bone::addChild(Bone* child) {
    children.push_back(child);
}

inline void Bone::readPose(Reader &r) {
    float rx=0, ry=0, rz=0;
    if (rotationBounds.dofRX) {
        r.readFloat(rx);
    }
    if (rotationBounds.dofRY) {
        r.readFloat(ry);
    }
    if (rotationBounds.dofRZ) {
        r.readFloat(rz);
    }
    currentRotation = fromEulerAnglesZYX(rz, ry, rx);
}

inline mat4 Bone::getCurrentLocalRotation() {
    return initialRotation * currentRotation * glm::inverse(initialRotation);
}

#endif
