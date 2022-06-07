#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include  "ofxAssimpModelLoader.h"
#include "Octree.h"
#include "Particle.h"
#include "ParticleEmitter.h"
#include <glm/gtx/intersect.hpp>



class ofApp : public ofBaseApp {

public:
    void setup();
    void update();
    void draw();
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent2(ofDragInfo dragInfo);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    void drawAxis(ofVec3f);
    void initLightingAndMaterials();
    void savePicture();
    void toggleWireframeMode();
    void togglePointsDisplay();
    void toggleSelectTerrain();
    bool doPointSelection();
    void setCameraTarget();
    void initLightingMaterials();
    bool mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point);
    bool raySelectWithOctree(ofVec3f &pointRet);
    glm::vec3 getMousePointOnPlane(glm::vec3 p, glm::vec3 n);



    ofEasyCam cam;
    ofCamera bottomCam, trackingCam, *currentCam;

    
    //start game or end game
    bool bStart = false;
    bool bOver = false;

    //models
    ofxAssimpModelLoader moon, lander;

    ofLight light;
    
    Box landerBounds;
    Box testBox;
    vector<Box> colBoxList;
    bool bLanderSelected = false;
    Octree octree;
    TreeNode selectedNode;
    glm::vec3 mouseDownPos, mouseLastPos;
    bool bInDrag = false;

    //ofxIntSlider numLevels;


    bool bAltKeyDown;
    bool bCtrlKeyDown;
    bool bWireframe;
    bool bDisplayPoints;
    bool bPointSelected;
    bool bHide;
    bool pointSelected = false;
    bool bDisplayLeafNodes = false;
    bool bDisplayOctree = false;
    bool bDisplayBBoxes = false;
    
    bool bLanderLoaded;
    bool bTerrainSelected;
    
    ofVec3f selectedPoint;
    ofVec3f intersectPoint;
    
    bool bLanded = false;
    bool bWin = false;
    


    float score = 0.0;
    float altitude = 0.0;


    vector<Box> bboxList;

    const float selectionRange = 4.0;

    int startTime;
    int timer;



    ofxPanel gui;
    glm::vec3 landerPos;
    glm::vec3 position;


    
    glm::vec3 velocity = glm::vec3(0, 0, 0);
    glm::vec3 acceleration = glm::vec3(0, 0, 0);
    glm::vec3 force = glm::vec3(0, 0, 0);
    glm::vec3 gravity = glm::vec3(0, -3, 0);
    glm::vec3 impulseForce = glm::vec3(0, 0, 0);
    glm::vec3 prevPos = glm::vec3(0, 0, 0);
    glm::vec3 pos1 = glm::vec3(0, 0, 0);
    float mass = 1.0;
    float damping = .99;
    float angularForce = 0;
    float angularVelocity = 0.0;
    float angularAcceleration = 0.0;
    bool bThrust = false;
    float prevDist = 0;
    float rotation = 90.0;
    float thrust = 20;
    float slipping = 20;
    float angle = 0;
    float restitution = 0.5;

    

    glm::vec3 heading() {
        return glm::normalize(glm::rotate(glm::vec3(1, 0, 0), glm::radians(rotation), glm::vec3(0, 1, 0)));
    }

    void integrate() {
        
        float framerate = ofGetFrameRate();
        float dt = 1.0 / framerate;
        
        acceleration = ((force + gravity + impulseForce) * 1.0 / mass);
        velocity += acceleration * dt;
        velocity *= damping;
        lander.setPosition(lander.getPosition().x + (velocity * dt).x,lander.getPosition().y + (velocity * dt).y, lander.getPosition().z + (velocity * dt).z);

        // angular motion
        lander.setRotation(0, rotation, 0, 1, 0);
        rotation += (angularVelocity * dt);
        float a = angularAcceleration;;
        a += (angularForce * 1.0 / mass);
        angularVelocity += a * dt;
        angularVelocity *= damping;
        velocity += (float)slipping * glm::cross(velocity, glm::vec3(0, 0, angularVelocity)) / ((float)thrust * (float)thrust);


    }
   
    map<int, bool> keymap;

    void loadVbo();


    int fuel = 0;
    bool noFuel = false;

    //effects
    ParticleEmitter exhaustEmitter;
    ParticleEmitter explosion;
    //forces
    TurbulenceForce *turbulenceForce;
    GravityForce *gravityForce;
    ImpulseRadialForce *radialForce;
    //light
    ofLight keyLight, rimLight, fillLight;

    ofSoundPlayer jet;
    ofSoundPlayer back;
    ofSoundPlayer hit;

    ofTexture  particleTex;

    // shaders
    //
    ofVbo vbo;
    ofShader shader;
    int radius = 0;
  
};

