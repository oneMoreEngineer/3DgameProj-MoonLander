
//  Kevin Smith   CS134
// Final Project
//  Student Name:   Andrei Titoruk
//  Date: May 10, 2022

#include "ofApp.h"
#include "Util.h"




//--------------------------------------------------------------
// setup scene, lighting, state and load g eometry
//
void ofApp::setup() {
    bWireframe = false;
    bDisplayPoints = false;
    bAltKeyDown = false;
    bCtrlKeyDown = false;
    bLanderLoaded = false;
    bTerrainSelected = true;

    ofSetVerticalSync(true);
    cam.disableMouseInput();
    ofEnableSmoothing();
    ofEnableDepthTest();


    ofDisableArbTex();     // disable rectangular textures

    fuel = 12000;
    initLightingMaterials();

    
    if (moon.loadModel("geo/moon-houdini.obj")) {
        moon.setScaleNormalization(false);
        octree.create(moon.getMesh(0), 20);
    }
    else
    {
        printf("Error. Map could not be loaded.\n");
        ofExit(0);
    }

    jet.load("sound/jet_sound.mp3");
    jet.setVolume(0.4);
    jet.setLoop(true);
    
    back.load("sound/background.mp3");
    back.setVolume(0.05);
    back.setLoop(true);
    back.play();
    
    hit.load("sound/hit.mp3");
    hit.setVolume(0.5);

//
    if (!ofLoadImage(particleTex, "images/dot.png")) {
        cout << "Particle Texture File: images/dot.png not found" << endl;
        ofExit();
    }

    // load the shader
    //
#ifdef TARGET_OPENGLES
    shader.load("shaders_gles/shader");
#else
    shader.load("shaders/shader");
#endif

    radius = 2;
    bHide = false;

    testBox = Box(Vector3(3, 3, 0), Vector3(5, 5, 2));
    float time = ofGetElapsedTimef();


    turbulenceForce = new TurbulenceForce(ofVec3f(-20, -20, -20), ofVec3f(20, 20, 20));
    gravityForce = new GravityForce(ofVec3f(0, -3, 0));
    radialForce = new ImpulseRadialForce(500.0);
    
    
    //Emitters

    exhaustEmitter.setOneShot(true);
    exhaustEmitter.setEmitterType(DiscEmitter);
    exhaustEmitter.setGroupSize(50);
    exhaustEmitter.spawn(1);
    exhaustEmitter.setParticleRadius(0.05);
    exhaustEmitter.setRate(1.0);
    exhaustEmitter.setLifespan(5.0);
    exhaustEmitter.sys->addForce(turbulenceForce);
    exhaustEmitter.sys->addForce(gravityForce);
    exhaustEmitter.setVelocity(ofVec3f(0, 0, 0));

    explosion.setOneShot(true);
    explosion.setEmitterType(RadialEmitter);
    explosion.setGroupSize(500);
    explosion.spawn(0.5);
    explosion.setParticleRadius(0.25);
    explosion.setRate(1.0);
    explosion.setLifespan(1.0);
    explosion.sys->addForce(turbulenceForce);
    explosion.sys->addForce(gravityForce);



    //cameras

    cam.setDistance(10);
    cam.setNearClip(.1);
    cam.setFov(65.5);   // approx equivalent to 28mm in 35mm format
    ofSetVerticalSync(true);
    
    position = lander.getPosition();

    trackingCam.setPosition(10, 15, 15);
    trackingCam.lookAt(glm::vec3(position.x, position.y, position.z));
    trackingCam.setNearClip(.1);

    bottomCam.setPosition(glm::vec3(position.x, position.y, position.z));
    bottomCam.lookAt(glm::vec3(0, -1, 0));
    bottomCam.setNearClip(.1);


    currentCam = &cam;




}
//--------------------------------------------------------------
// incrementally update scene (animation)
//
void ofApp::update() {

    if (bStart)
    {
        if (!back.isPlaying()) back.play();

        
        if (bLanded) {
            velocity = glm::vec3(0, 0, 0);
            acceleration = glm::vec3(0, 0, 0);
            force = glm::vec3(0, 0, 0);
        }

        if (fuel <= 0) {
            force = glm::vec3(0, -3, 0);
            bOver = true;
            noFuel = true;
        }

        landerPos = lander.getPosition();
        trackingCam.lookAt(landerPos);
        bottomCam.setPosition(landerPos.x, landerPos.y - 1, landerPos.z);
        
        ofVec3f min = lander.getSceneMin() + lander.getPosition();
        ofVec3f max = lander.getSceneMax() + lander.getPosition();

        Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
        colBoxList.clear();
        if (octree.intersect(bounds, octree.root, colBoxList)) {
            if (velocity.y <= -3){
                explosion.sys -> reset();
                explosion.start();
                bLanded = true;
                bOver = true;
                //cout << "exploaded" <<endl;
            }
            if (altitude > 2) {
                velocity.x = -velocity.x * restitution;
                velocity.y = glm::abs(velocity.y) * restitution;
                velocity.z = -velocity.z * restitution;
            }
            else {
                impulseForce = (restitution) * glm::dot(-velocity, glm::vec3(0, 1, 0)) * glm::vec3(0, 1, 0) * ofGetFrameRate();
                velocity.y = 0;
            }
            
        }
        if (( landerPos.x >= -20 &&  landerPos.x <= 20) && (landerPos.y >= 0 && landerPos.y <= 0.15) && (landerPos.z >= -20 && landerPos.z <= 20)) {
            bLanded = true;
            bWin = true;
        }

        Ray altitudeRay = Ray(Vector3(lander.getPosition().x, lander.getPosition().y, lander.getPosition().z), Vector3(lander.getPosition().x, lander.getPosition().y - 200, lander.getPosition().z));
        TreeNode altNode;
        if (octree.intersect(altitudeRay, octree.root, altNode)){
            altitude = glm::length(octree.mesh.getVertex(altNode.points[0]) - lander.getPosition());
        }

        glm::vec3 pos = lander.getPosition();
        exhaustEmitter.setPosition(glm::vec3(pos.x, pos.y, pos.z));
        explosion.setPosition(glm::vec3(pos.x, pos.y, pos.z));

        exhaustEmitter.update();
        explosion.update();
        
        integrate();
        
        force = glm::vec3(0, 0, 0);
        impulseForce = glm::vec3(0, 0, 0);
        angularForce = 0;
    }


}
//--------------------------------------------------------------
void ofApp::draw() {

    loadVbo();
    glDepthMask(false);
    ofBackground(ofColor::black);
    glDepthMask(true);

    currentCam->begin();

    ofPushMatrix();
    explosion.draw();
    
    if (bWireframe) {
        ofDisableLighting();
        ofSetColor(ofColor::slateGray);
        moon.drawWireframe();
        if (bLanderLoaded) {
            lander.drawWireframe();
            if (!bTerrainSelected) drawAxis(lander.getPosition());
        }
        if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
    }
    else {
        ofEnableLighting();
        moon.drawFaces();
        ofMesh mesh;
        if (bLanderLoaded) {
            lander.drawFaces();
            if (!bTerrainSelected) drawAxis(lander.getPosition());
            if (bDisplayBBoxes) {
                ofNoFill();
                ofSetColor(ofColor::white);
                for (int i = 0; i < lander.getNumMeshes(); i++) {
                    ofPushMatrix();
                    ofMultMatrix(lander.getModelMatrix());
                    ofRotate(-90, 1, 0, 0);
                    Octree::drawBox(bboxList[i]);
                    ofPopMatrix();
                }
            }

            if (bLanderSelected) {

                ofVec3f min = lander.getSceneMin() + lander.getPosition();
                ofVec3f max = lander.getSceneMax() + lander.getPosition();

                landerBounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));

                ofSetColor(ofColor::white);
                Octree::drawBox(landerBounds);

                ofSetColor(ofColor::lightBlue);
                for (int i = 0; i < colBoxList.size(); i++) {
                    Octree::drawBox(colBoxList[i]);
                }
            }
        }
    }
    if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));



    if (bDisplayPoints) {                // display points as an option
        glPointSize(3);
        ofSetColor(ofColor::green);
        moon.drawVertices();
    }

    // highlight selected point (draw sphere around selected point)
    //
    if (bPointSelected) {
        ofSetColor(ofColor::blue);
        ofDrawSphere(selectedPoint, .1);
    }

    glDepthMask(GL_FALSE);
        ofSetColor(ofColor::orange);

        // this makes everything look glowy :)
        ofEnableBlendMode(OF_BLENDMODE_ADD);
        ofEnablePointSprites();

        shader.begin();

        particleTex.bind();
        vbo.draw(GL_POINTS, 0, (int)exhaustEmitter.sys->particles.size());
        exhaustEmitter.draw();
        particleTex.unbind();

        shader.end();

        ofDisablePointSprites();
        ofDisableBlendMode();
        glDepthMask(GL_TRUE);
        ofEnableAlphaBlending();
    //
    ofDisableLighting();
    int level = 0;
    //    ofNoFill();

    if (bDisplayLeafNodes) {
        octree.drawLeafNodes(octree.root);
        cout << "num leaf: " << octree.numLeaf << endl;
    }
    else if (bDisplayOctree) {
        ofNoFill();
        ofSetColor(ofColor::white);
        octree.draw(1, 0);
    }

    if (pointSelected) {
        ofVec3f p = octree.mesh.getVertex(selectedNode.points[0]);
        ofVec3f d = p - cam.getPosition();
        ofSetColor(ofColor::lightGreen);
        ofDrawSphere(p, .02 * d.length());
    }


    ofPopMatrix();
    currentCam->end();
    
    if (!bStart) {
        ofSetColor(ofColor::white);
        ofDrawBitmapString("\n Press C to enable camera movement\n Drag the lander (will be fixed in the fututre updates :)\n After lander is loaded - Press Spacebar to start\n  ", (ofGetWindowWidth() / 2) - 92, ofGetWindowHeight() / 2 - 5);
    }
    if (bStart && !bOver) {
        ofSetColor(ofColor::white);
        ofDrawBitmapString("Altitude: " + ofToString(altitude), 10, 15);
        ofDrawBitmapString("FPS: " + ofToString(ofGetFrameRate()), ofGetWindowWidth() - 130, 15);
        ofDrawBitmapString("Fuel: " + ofToString(fuel), 10, 40);
        ofDrawBitmapString("Chance of crash (should be more than -3): " + ofToString(velocity.y), 10, 65);
        
    }

    if (bLanded && bOver) {
        ofSetColor(ofColor::red);
        ofDrawBitmapString("GAME OVER\n", (ofGetWindowWidth() / 2) - 92, ofGetWindowHeight() / 2 - 5);

    }
    if (noFuel && bOver) {
        ofSetColor(ofColor::red);
        ofDrawBitmapString("GAME OVER\n  OUT OF FUEL\n", (ofGetWindowWidth() / 2) - 92, ofGetWindowHeight() / 2 - 5);
    }

    if (bWin  && bLanded) {
        ofSetColor(ofColor::green);
        ofDrawBitmapString("WE HAVE a WINNER!", ofGetWindowWidth() / 2, ofGetWindowHeight() / 2);
       
    }

}


//
void ofApp::drawAxis(ofVec3f location) {

    ofPushMatrix();
    ofTranslate(location);

    ofSetLineWidth(1.0);

    // X Axis
    //RED
    ofSetColor(ofColor(255, 0, 0));
    ofDrawLine(ofPoint(0, 0, 0), ofPoint(5, 0, 0));
    

    // Y Axis
    //GREEN
    ofSetColor(ofColor(0, 255, 0));
    ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 5, 0));

    // Z Axis
    //BLUE
    ofSetColor(ofColor(0, 0, 255));
    ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 0, 5));
    

    ofPopMatrix();
}

void ofApp::keyPressed(int key) {

    glm::vec3 landerposition = lander.getPosition();
    switch (key) {
    case '1':
        currentCam = &cam;
        break;
    case '2':
        currentCam = &trackingCam;
        break;
    case '3':
        currentCam = &bottomCam;
        break;
    case'u':
    case'U':
        cam.lookAt(landerposition);
        break;
    case 'C':
    case 'c':
        if (cam.getMouseInputEnabled())
            cam.disableMouseInput();
        else
            cam.enableMouseInput();
        break;
    case 'O':
    case 'o':
        bDisplayOctree = !bDisplayOctree;
        break;
    case 'F':
    case 'f':
        ofToggleFullscreen();
        break;
    case 'H':
    case 'h':
        bHide = !bHide;
        break;
    case 'n':
    case 'N':
        bDisplayLeafNodes = !bDisplayLeafNodes;
        break;
    case 'z':
        cam.reset();
        break;
    case 'r':
        if (bLanderLoaded){
            lander.setPosition(-40, 40 , 40);
            bStart = false;
            bOver = false;
            bLanded = false;
            bWin = false;
            fuel = 12000;
            integrate();
        }
        break;
    case 'v':
        togglePointsDisplay();
        break;
    case 'V':
        break;
    case OF_KEY_UP:
        fuel -= 1;
        exhaustEmitter.sys->reset();
        exhaustEmitter.start();
        force = thrust * heading();
        if (!jet.isPlaying()) jet.play();
        angularVelocity = 0;
        break;

    case OF_KEY_DOWN:
        fuel -= 1;
        exhaustEmitter.sys->reset();
        exhaustEmitter.start();
        force = -thrust * glm::vec3(heading().x, heading().y, heading().z);
        if (!jet.isPlaying()) jet.play();
        break;
    case OF_KEY_LEFT:
        rotation +=2;
    break;
    case OF_KEY_RIGHT:
        rotation -=2;
    break;
    case 'w':
    case 'W':
        fuel -= 10;
        exhaustEmitter.sys->reset();
        exhaustEmitter.start();
        force = float(thrust) * ofVec3f(0, 1, 0);
        if (!jet.isPlaying()) jet.play();
        break;
    case ' ':
        bStart = !bStart;
        break;
    case 's':
    case 'S':
        fuel -= 10;
        exhaustEmitter.sys->reset();
        exhaustEmitter.start();
        force = float(thrust) * ofVec3f(0, -1, 0);
        if (!jet.isPlaying()) jet.play();
        break;
    case 'a':
    case 'A':
           
            fuel -= 1;
            exhaustEmitter.sys->reset();
            exhaustEmitter.start();
            force = thrust * glm::cross(glm::vec3(0, 1, 0), heading());
            if (!jet.isPlaying()) jet.play();
            break;
    case 'D':
    case 'd':
            
            fuel -= 1;
            exhaustEmitter.sys->reset();
            exhaustEmitter.start();
            force = -thrust * glm::cross(glm::vec3(0, 1, 0), heading());
            if (!jet.isPlaying()) jet.play();
            break;
        default:
        break;
    }
}

void ofApp::toggleWireframeMode() {
    bWireframe = !bWireframe;
}

void ofApp::toggleSelectTerrain() {
    bTerrainSelected = !bTerrainSelected;
}

void ofApp::togglePointsDisplay() {
    bDisplayPoints = !bDisplayPoints;
}

void ofApp::keyReleased(int key) {

    switch (key)
    {

    case 'w':
    case 'W':
        //bThrust = false;
        force = glm::vec3(0, 0, 0);
        jet.stop();

        break;

        case 'S':
        case 's':
        //bThrust = false;
        force = glm::vec3(0, 0, 0);
        jet.stop();

        break;
    
    case OF_KEY_ALT:
        cam.disableMouseInput();
        bAltKeyDown = false;
        break;
    case OF_KEY_SHIFT:
        break;
            
    case 'd':
    case 'D':
            force = glm::vec3(0, 0, 0);
            jet.stop();
        break;
    case 'a':
    case 'A':
            force = glm::vec3(0, 0, 0);
            jet.stop();
        break;
    case OF_KEY_UP:
            force = glm::vec3(0, 0, 0);
            jet.stop();
        break;
    case OF_KEY_DOWN:
            force = glm::vec3(0, 0, 0);
            jet.stop();
        break;
    default:
        break;

    }
}



//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {


}


//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

    // if moving camera, don'ft allow mouse interaction
    //
    if (cam.getMouseInputEnabled()) return;

    // if moving camera, don't allow mouse interaction
//
    if (cam.getMouseInputEnabled()) return;

   
    if (bLanderLoaded) {
        glm::vec3 origin = cam.getPosition();
        glm::vec3 mouseWorld = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
        glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);

        ofVec3f min = lander.getSceneMin() + lander.getPosition();
        ofVec3f max = lander.getSceneMax() + lander.getPosition();

        Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
        bool hit = bounds.intersect(Ray(Vector3(origin.x, origin.y, origin.z), Vector3(mouseDir.x, mouseDir.y, mouseDir.z)), 0, 10000);
        if (hit) {
            bLanderSelected = true;
            mouseDownPos = getMousePointOnPlane(lander.getPosition(), cam.getZAxis());
            mouseLastPos = mouseDownPos;
            bInDrag = true;
        }
        else {
            bLanderSelected = false;
        }
    }
    else {
        ofVec3f p;
        raySelectWithOctree(p);
        cout << "POINT SELECTED: " << raySelectWithOctree(p) << endl;
    }
}

bool ofApp::raySelectWithOctree(ofVec3f &pointRet) {
    ofVec3f mouse(mouseX, mouseY);
    ofVec3f rayPoint = cam.screenToWorld(mouse);
    ofVec3f rayDir = rayPoint - cam.getPosition();
    rayDir.normalize();
    Ray ray = Ray(Vector3(rayPoint.x, rayPoint.y, rayPoint.z),
        Vector3(rayDir.x, rayDir.y, rayDir.z));

    //determines
    pointSelected = octree.intersect(ray, octree.root, selectedNode);

    if (pointSelected) {
        pointRet = octree.mesh.getVertex(selectedNode.points[0]);
        cout << "POINT RET:" << pointRet << endl;
    }
    return pointSelected;
}





//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

    // if moving camera, don't allow mouse interaction
    //
    if (cam.getMouseInputEnabled()) return;

    if (bInDrag) {

        glm::vec3 landerP = lander.getPosition();

        glm::vec3 mousePos = getMousePointOnPlane(landerP, cam.getZAxis());
        glm::vec3 delta = mousePos - mouseLastPos;

        landerP += delta;
        lander.setPosition(landerP.x, landerP.y, landerP.z);
        lander.setRotation(0, rotation, 1, 0, 0);
        mouseLastPos = mousePos;

        ofVec3f min = lander.getSceneMin() + lander.getPosition();
        ofVec3f max = lander.getSceneMax() + lander.getPosition();

        Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));

        colBoxList.clear();
        octree.intersect(bounds, octree.root, colBoxList);

    }
    else {
        ofVec3f p;
        raySelectWithOctree(p);
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
    bInDrag = false;
}



// Set the camera to use the selected point as it's new target
//
void ofApp::setCameraTarget() {

}


//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}



//--------------------------------------------------------------
// setup basic ambient lighting in GL  (for now, enable just 1 light)
//

void ofApp::savePicture() {
    ofImage picture;
    picture.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
    picture.save("screenshot.png");
    cout << "picture saved" << endl;
}

//--------------------------------------------------------------

//
void ofApp::dragEvent2(ofDragInfo dragInfo) {

    ofVec3f point;
    mouseIntersectPlane(ofVec3f(0, 0, 0), cam.getZAxis(), point);
    //loading of the model
    if (lander.loadModel(dragInfo.files[0])) {
        lander.setScaleNormalization(false);
        lander.setScale(.1, .1, .1);
        lander.setPosition(1, 10, 0);

        bLanderLoaded = true;
        for (int i = 0; i < lander.getMeshCount(); i++) {
            bboxList.push_back(Octree::meshBounds(lander.getMesh(i)));
        }

        cout << "Mesh Count: " << lander.getMeshCount() << endl;
    }
    else cout << "Error: Can't load model" << dragInfo.files[0] << endl;
}

bool ofApp::mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point) {
    ofVec2f mouse(mouseX, mouseY);
    ofVec3f rayPoint = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
    ofVec3f rayDir = rayPoint - cam.getPosition();
    rayDir.normalize();
    return (rayIntersectPlane(rayPoint, rayDir, planePoint, planeNorm, point));
}

//--------------------------------------------------------------

void ofApp::dragEvent(ofDragInfo dragInfo) {
    if (lander.loadModel(dragInfo.files[0])) {
        bLanderLoaded = true;
        lander.setScaleNormalization(false);
        lander.setPosition(0, 10, 0);
        cout << "number of meshes: " << lander.getNumMeshes() << endl;
        bboxList.clear();
        for (int i = 0; i < lander.getMeshCount(); i++) {
            bboxList.push_back(Octree::meshBounds(lander.getMesh(i)));
        }

        //lander.setRotation(1, 180, 1, 0, 0);

        // We want to drag and drop a 3D object in space so that the model appears
        // under the mouse pointer where you drop it !
        //
        // Our strategy: intersect a plane parallel to the camera plane where the mouse drops the model
        

        // Setup our rays
        //
        glm::vec3 origin = cam.getPosition();
        glm::vec3 camAxis = cam.getZAxis();
        glm::vec3 mouseWorld = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
        glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
        float distance;

        bool hit = glm::intersectRayPlane(origin, mouseDir, glm::vec3(0, 0, 0), camAxis, distance);
        if (hit) {
            // find the point of intersection on the plane using the distance
            // We use the parameteric line or vector representation of a line to compute
            //
            // p' = p + s * dir;
            //
            glm::vec3 intersectPoint = origin + distance * mouseDir;

                        glm::vec3 min = lander.getSceneMin();
            glm::vec3 max = lander.getSceneMax();
            float offset = (max.y - min.y) / 2.0;
            lander.setPosition(intersectPoint.x, intersectPoint.y - offset, intersectPoint.z);

            
            landerBounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));

        }
    }


}

//  intersect the mouse ray with the plane normal to the camera
//  return intersection point.   (package code above into function)
//
glm::vec3 ofApp::getMousePointOnPlane(glm::vec3 planePt, glm::vec3 planeNorm) {
    // Setup our rays
    //
    glm::vec3 origin = cam.getPosition();
    glm::vec3 camAxis = cam.getZAxis();
    glm::vec3 mouseWorld = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
    glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
    float distance;

    bool hit = glm::intersectRayPlane(origin, mouseDir, planePt, planeNorm, distance);

    if (hit) {
        // find the point of intersection on the plane using the distance
        // We use the parameteric line or vector representation of a line to compute
        //
        // p' = p + s * dir;
        //
        glm::vec3 intersectPoint = origin + distance * mouseDir;

        return intersectPoint;
    }
    else return glm::vec3(0, 0, 0);
}



void ofApp::initLightingMaterials()

{
    static float ambient[] =
        { .5f, .5f, .5, 1.0f };
        static float diffuse[] =
        { 1.0f, 1.0f, 1.0f, 1.0f };

        static float position[] =
        {5.0, 5.0, 5.0, 0.0 };

        static float lmodel_ambient[] =
        { 1.0f, 1.0f, 1.0f, 1.0f };

        static float lmodel_twoside[] =
        { GL_TRUE };
    
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, position);

    glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT1, GL_POSITION, position);


    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
    glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
//    glEnable(GL_LIGHT1);
    glShadeModel(GL_SMOOTH);
    
    keyLight.setup();
    keyLight.enable();
    keyLight.setAreaLight(2, 2);
    keyLight.setAmbientColor(ofFloatColor(1, 1, 1));
    keyLight.setDiffuseColor(ofFloatColor(1, 1, 1));
    keyLight.setSpecularColor(ofFloatColor(1, 1, 1));
    keyLight.rotate(45, ofVec3f(0, 1, 0));
    keyLight.rotate(-45, ofVec3f(1, 0, 0));
    keyLight.setSpotlightCutOff(75);
    keyLight.setPosition(56, 15, 70);

    fillLight.setup();
    fillLight.enable();
    fillLight.setAreaLight(1, 1);
    fillLight.setSpotlightCutOff(25);
    fillLight.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1));
    fillLight.setDiffuseColor(ofFloatColor(0.5, 0.1, 1));
    fillLight.setSpecularColor(ofFloatColor(0.1, 15, 1));
    fillLight.rotate(-90, ofVec3f(1, 0, 0));
    fillLight.rotate(-45, ofVec3f(0, 1, 0));
    fillLight.setPosition(0, 45, 0);

    rimLight.setup();
    rimLight.enable();
    rimLight.setSpotlight();
    rimLight.setScale(.5);
    rimLight.setSpotlightCutOff(10);
    rimLight.setAttenuation(.2, .001, .001);
    rimLight.setAmbientColor(ofFloatColor(1, 1, 0.6));
    rimLight.setDiffuseColor(ofFloatColor(1, 12, 15));
    rimLight.setSpecularColor(ofFloatColor(1, 22, 15));
    rimLight.rotate(180, ofVec3f(0, 1, 0));
    fillLight.rotate(-45, ofVec3f(0, 1, 0));
    rimLight.setPosition(0, 3, -85);


}


void ofApp::loadVbo() {
    if (exhaustEmitter.sys->particles.size() < 1) return;

    vector<ofVec3f> sizes;
    vector<ofVec3f> points;
    for (int i = 0; i < exhaustEmitter.sys->particles.size(); i++) {
        points.push_back(exhaustEmitter.sys->particles[i].position);
        sizes.push_back(ofVec3f(radius));
    }
    // upload the data to the vbo
    //
    int total = (int)points.size();
    vbo.clear();
    vbo.setVertexData(&points[0], total, GL_STATIC_DRAW);
    vbo.setNormalData(&sizes[0], total, GL_STATIC_DRAW);
}

