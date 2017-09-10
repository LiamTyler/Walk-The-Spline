#include "engine.hpp"
#include "camera.hpp"
#include "character.hpp"
#include "config.hpp"
#include "draw.hpp"
#include "spline.hpp"
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <iomanip>
using namespace std;
using glm::vec3;
using glm::vec4;

class SplineWalker: public Engine {
public:

    SDL_Window *window;
    OrbitCamera *camera;

    Character *character;
    Spline3 *path;
    float time; // time along the path

    SplineWalker() {
        window = createWindow("Walk the Spline", 1280, 720);
        camera = new OrbitCamera(5, 0, 0, Perspective(30, 16/9., 0.1, 20));
        character = new Character(Config::asfFile, Config::amcFile,
                                  Config::basePosition, Config::baseVelocity);
        if (!character->hasSkeleton()) {
            errorMessage("Failed to load file " + Config::asfFile);
            exit(EXIT_FAILURE);
        }
        if (!character->hasAnimation()) {
            errorMessage("Failed to load file " + Config::amcFile);
            exit(EXIT_FAILURE);
        }
        path = new Spline3;
        // Constant-velocity line
        /*
        path->points.push_back(SplinePoint3(0, vec3(0,0,0), vec3(1.5,0,0)));
        path->points.push_back(SplinePoint3(10, vec3(15,0,0), vec3(1.5,0,0)));
        */
        /*
        // Ease-in ease-out line
        path->points.push_back(SplinePoint3(0, vec3(0,0,0), vec3(0,0,0)));
        path->points.push_back(SplinePoint3(10, vec3(15,0,0), vec3(0,0,0)));
        */
        // Approximately circular path
        /*
        path->points.push_back(SplinePoint3(0, vec3(5,0,0), vec3(0,0,1.5)));
        path->points.push_back(SplinePoint3(5, vec3(0,0,5), vec3(-1.5,0,0)));
        path->points.push_back(SplinePoint3(10, vec3(-5,0,0), vec3(0,0,-1.5)));
        path->points.push_back(SplinePoint3(15, vec3(0,0,-5), vec3(1.5,0,0)));
        path->points.push_back(SplinePoint3(20, vec3(5,0,0), vec3(0,0,1.5)));
        */
        // Figure-eight path
        path->points.push_back(SplinePoint3(0, vec3(5,0,0), vec3(0,0,1)));
        path->points.push_back(SplinePoint3(5, vec3(0,0,0), vec3(-1,0,-1)));
        path->points.push_back(SplinePoint3(10, vec3(-5,0,0), vec3(0,0,1)));
        path->points.push_back(SplinePoint3(15, vec3(0,0,0), vec3(1,0,-1)));
        path->points.push_back(SplinePoint3(20, vec3(5,0,0), vec3(0,0,1)));
        time = 0;
    }

    ~SplineWalker() {
        SDL_DestroyWindow(window);
    }

    void run() {
        float fps = 60, dt = 1/fps;
        while (!shouldQuit()) {
            handleInput();
            advanceState(dt);
            drawGraphics();
            waitForNextFrame(dt);
        }
    }

    void advanceState(float dt) {
        time += dt;
        if (time > path->maxTime())
            time = path->minTime();

        // TODO: Modify this to control the speed of the character's
        // walk cycle animation.
        float t;
        float curspeed = glm::length(path->getDerivative(t));
        
        character->advance(dt * curspeed / glm::length(Config::baseVelocity));
        // character->advance(dt);

        vec3 p = path->getValue(time);
        vec3 c = camera->getCenter();
        camera->setCenter(glm::mix(c, vec3(p.x, 0.8, p.z), 10*dt));
    }

    void setAmbientLight(vec3 color) {
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, &color[0]);
    }

    void addLight(GLenum light, vec4 position, vec3 color) {
        glEnable(light);
        glLightfv(light, GL_POSITION, &position[0]);
        glLightfv(light, GL_DIFFUSE, &color[0]);
    }

    void drawGraphics() {
        // Blue sky background
        glClearColor(0.6,0.8,1.0, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Allow lines to show up on top of filled polygons
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1,1);

        camera->apply();

        //Draw::axes(1);

        // Light from above
        glEnable(GL_LIGHTING);
        setAmbientLight(vec3(0.5,0.5,0.5));
        addLight(GL_LIGHT0, vec4(+1,1,0,0), 0.2f*vec3(1,1,1));
        addLight(GL_LIGHT1, vec4(-1,1,0,0), 0.2f*vec3(1,1,1));
        addLight(GL_LIGHT2, vec4(0,1,-1,0), 0.2f*vec3(1,1,1));
        addLight(GL_LIGHT3, vec4(0,1,+1,0), 0.2f*vec3(1,1,1));

        // Draw floor
        drawFloor(path->getValue(time));

        glColor3f(0.8,0.2,0.2);
        drawSpline(path);

        // TODO: Translate the character to align with the position
        // obtained from the path spline, i.e. path->getValue(time).
        // Also rotate it so its z-axis aligns with the path spline's
        // velocity, path->getDerivative(time).
        glColor3f(1,0.8,0.2);
        glPushMatrix();
            vec3 pos = path->getValue(time);
            vec3 der = glm::normalize(path->getDerivative(time));
            glTranslatef(pos.x, pos.y, pos.z);
            float angle = atan2(der.x, der.z) * 180.0 / M_PI;
            glRotatef(angle, 0, 1, 0);
            character->draw();
        glPopMatrix();

        SDL_GL_SwapWindow(window);
    }

    void drawSpline(Spline3 *spline) {
        glPushAttrib(GL_LINE_BIT);
        glLineWidth(2);
        glBegin(GL_LINE_STRIP);
        glNormal3f(0,1,0);
        for (float t = spline->minTime(); t < spline->maxTime(); t+=0.1) {
            vec3 value = spline->getValue(t);
            glVertex3f(value.x, value.y, value.z);
        }
        glEnd();
        for (int i = 0; i < spline->points.size(); i++) {
            vec3 p = spline->points[i].p;
            glPushMatrix();
            glTranslatef(p.x, p.y, p.z);
            glScalef(0.1,0.1,0.1);
            glTranslatef(-0.5,-0.5,-0.5);
            Draw::unitCube();
            glPopMatrix();
        }
        glPopAttrib();
    }

    void drawFloor(vec3 center) {
        int cx = 2*(int)round(center.x/2), cz = 2*(int)round(center.z/2);
        int n = 10;
        for (int i = -n; i < n; i++) {
            for (int j = -n; j < n; j++) {
                if ((i+j) % 2 == 0)
                    glColor3f(0.8,0.8,0.8);
                else
                    glColor3f(0.6,0.6,0.6);
                glPushMatrix();
                glTranslatef(cx+i, 0, cz+j);
                glTranslatef(0,0,1);
                glRotatef(-90, 1,0,0);
                Draw::unitSquareXY();
                glPopMatrix();
            }
        }
    }

    void onMouseMotion(SDL_MouseMotionEvent &e) {
        camera->onMouseMotion(e);
    }
};

int main(int argc, char **argv) {
    SplineWalker app;
    app.run();
    return EXIT_SUCCESS;
}
