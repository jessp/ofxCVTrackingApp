#include "testApp.h"
#include "ofAppGLFWWindow.h"

using namespace ofxCv;


void testApp::setup() {
	
    ofBackground(0, 0, 0);
    ofSetVerticalSync(true);
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofDisableArbTex();
    ofSetFrameRate(31);
    ofDisableArbTex();
    
    cam.initGrabber(640, 480);
    
    setupCamProj();
    setupTracker();
    
    model.loadModel("astroBoy_walk.dae", true);
    model.setLoopStateForAllAnimations(OF_LOOP_NORMAL);
    model.playAllAnimations();
    
    bDrawDebug = true;

    bFound = false;
    
    img.loadImage("smacss.png");
    img2.loadImage("sunglasses.png");
    
    
    
}

void testApp::setupCamProj(){
    rotObjToProj = Mat::zeros(3, 1, CV_64F);
    transObjToProj = Mat::zeros(3, 1, CV_64F);
    rotObjToCam = Mat::zeros(3, 1, CV_64F);
    transObjToCam = Mat::zeros(3, 1, CV_64F);
    camproj.load("calibrationCamera.yml", "calibrationProjector.yml", "CameraProjectorExtrinsics.yml");
}

void testApp::setupTracker(){
    trackedImg.loadImage("smacss.png");
    tracker.setup(camproj.getCalibrationCamera());
    tracker.add(trackedImg);
//    tracker.startThread();
}

void testApp::exit() {
    ofLog() << "Exiting app";
    //tracker.waitForThread(true);
}

void testApp::update() {
    
    ofSetWindowTitle(ofToString(ofGetFrameRate(),1)+"fps");
    
	cam.update();
    
    if(cam.isFrameNew()) {
        
        tracker.update(cam);
        
        bFound = false;
        
        if(tracker.isFound()){
            

            
            // get object-to-camera transforms
            cv::Mat rvec, tvec;
            tracker.getRT(camproj.getCalibrationCamera().getDistortedIntrinsics().getCameraMatrix(), camproj.getCalibrationProjector().getDistCoeffs(), rvec, tvec);
            
            
            
            // hacks to adjust results
                    for (int i=0; i<3; i++) { *tvec.ptr<double>(i) *= 9.35; } // TODO : find this scaling value in configuration files
            *tvec.ptr<double>(1) += 1; // TODO : remove this y axis hack
        
            
            // smooth results
            rotObjToCam += (rvec-rotObjToCam) * 0.3;
            transObjToCam += (tvec-transObjToCam) * 0.3;
            
 

            
            bFound = true;
            
            model.update();
            mesh = model.getCurrentAnimatedMesh(0);
     
            
            
        }
	}
}

void testApp::draw() {
    
    ofSetColor(255);
    cam.draw(0, 0, 1280, 800);
    
    
    
	if(bFound) {
        
        if(bDrawDebug){
//            tracker.draw();
            
        }
        
        
        

        drawUsingGL();

    }

    


}


void testApp::drawUsingGL(){
    
    cout << "FOUND!" << endl;
    
    
    glMatrixMode( GL_MODELVIEW );
    
    ofPushMatrix();
    
    // Set perspective matrix using the projector intrinsics
    
    camproj.getCalibrationProjector().getDistortedIntrinsics().loadProjectionMatrix(20., 100000., cv::Point2d(0, 0));


    // apply model to projector transformations
    cv::composeRT(rotObjToCam,  transObjToCam,
                  camproj.getCamToProjRotation(), camproj.getCamToProjTranslation(),
                  rotObjToProj, transObjToProj);
    applyMatrix(makeMatrix(rotObjToProj, transObjToProj));
    
    ofVec2f bookSizeCm = ofVec2f(13.5, 20);
    ofVec2f bookSizePx = ofVec2f(200, 298);
    ofScale(bookSizeCm.x/bookSizePx.x, bookSizeCm.y/bookSizePx.y, 0.1);
    
    // project some square animation
    float w = trackedImg.width;
    float h = trackedImg.height;
    
    

    
    ofFill();

//    ofDrawBox(0, 0, 0, 100, 100, 50);
    
    //Model Stuff
    
    ofSetColor(255);
    
    ofEnableBlendMode(OF_BLENDMODE_ALPHA);
    
	ofEnableDepthTest();
    
    glShadeModel(GL_SMOOTH); //some model / light stuff
    light.enable();
    ofEnableSeparateSpecularLight();

    
    glShadeModel(GL_SMOOTH); //some model / light stuff
    light.enable();
    ofEnableSeparateSpecularLight();
    
    ofPushMatrix();
    
    ofScale(0.3, 0.3, 0.3);
    ofRotateX(90);
     model.drawFaces();
    
    ofPopMatrix();
    
    if(ofGetGLProgrammableRenderer()){
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);
    }
    glEnable(GL_NORMALIZE);
    
    ofPushMatrix();
    ofTranslate(model.getPosition().x-300, model.getPosition().y, 0);
    ofTranslate(-model.getPosition().x, -model.getPosition().y, 0);
    
    ofxAssimpMeshHelper & meshHelper = model.getMeshHelper(0);
    
    ofMultMatrix(model.getModelMatrix());
    ofMultMatrix(meshHelper.matrix);
    
    ofMaterial & material = meshHelper.material;
    if(meshHelper.hasTexture()){
        meshHelper.getTexturePtr()->bind();
    }
    material.begin();
    mesh.drawWireframe();
    material.end();
    if(meshHelper.hasTexture()){
        meshHelper.getTexturePtr()->unbind();
    }
    ofPopMatrix();
    
    if(ofGetGLProgrammableRenderer()){
    	glPopAttrib();
    }
    
    ofDisableDepthTest();
    light.disable();
    ofDisableLighting();
    ofDisableSeparateSpecularLight();
    
    ofSetColor(255, 255, 255 );

    
    


    
    ofPopMatrix();


    // TODO : restore GL matrices
    
    
    
}




string testApp::getRTMatInfos(const cv::Mat rvecs, const cv::Mat tvecs){
    cv::Mat rot3x3 = cv::Mat::zeros(3, 3, CV_32F);
    Rodrigues(rvecs, rot3x3);
    stringstream imgRTs;
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++)  imgRTs << ofToString(rot3x3.at<double>(i,j),1) << "\t";
        imgRTs << ofToString(tvecs.at<double>(i),1) << endl;
    }
    return imgRTs.str();
}

void testApp::keyPressed(int key){
    switch (key) {
        case 's':
            bDrawDebug = !bDrawDebug;
            break;
               default:
            break;
    }
}


    void testApp::mousePressed(int x, int y, int button) {
        selectedCorner = -1;
        for (int i=0; i<4; i++) {
            if (ofDist(corners[i].x, corners[i].y, x-30, y-30)<10) {
                selectedCorner = i;
            }
        }
    }
    
    void testApp::mouseDragged(int x, int y, int button) {
        corners[selectedCorner].set(x-30,y-30);
    }
    
    void testApp::mouseReleased(int x, int y, int button) {
        selectedCorner = -1;
    }
    
    
    ofPoint testApp::ofxLerp(ofPoint start, ofPoint end, float amt) {
        return start + amt * (end - start);
    }
    
    int testApp::ofxIndex(float x, float y, float w) {
        return y*w+x;
    }
    
    






