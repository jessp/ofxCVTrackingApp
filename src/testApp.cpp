#include "testApp.h"
#include "ofAppGLFWWindow.h"

using namespace ofxCv;


void testApp::setup() {
	
    ofBackground(0, 0, 0);
    ofSetVerticalSync(true);
    ofEnableDepthTest();
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofSetFrameRate(31);
    
    cam.initGrabber(640, 480);
    
    setupCamProj();
    setupTracker();
    
    bDrawDebug = true;
    bDrawWithCV = false;
    bFound = false;
    
    img.loadImage("yidunion.jpg");
    img2.loadImage("sunglasses.png");
    
    
    selectedCorner = -1;
    
    //set corners
    corners[0].set(0,0);
    corners[1].set(img2.width,0);
    corners[2].set(img2.width,img.height);
    corners[3].set(0,img2.height);
    
    markerObjectPoints.push_back(cv::Point3d(-6, -6, 0));
    markerObjectPoints.push_back(cv::Point3d(6, -6, 0));
    markerObjectPoints.push_back(cv::Point3d(6, 6, 0));
    markerObjectPoints.push_back(cv::Point3d(-6, 6, 0));
    cv::Mat(markerObjectPoints).convertTo(objPM,CV_32F);
   
    
    imagePoints.push_back(cv::Point2d(-6, 0));
    imagePoints.push_back(cv::Point2d(6, 0));
    imagePoints.push_back(cv::Point2d(6, 6));
    imagePoints.push_back(cv::Point2d(-6, -6));
    
}

void testApp::setupCamProj(){
    rotObjToProj = Mat::zeros(3, 1, CV_64F);
    transObjToProj = Mat::zeros(3, 1, CV_64F);
    rotObjToCam = Mat::zeros(3, 1, CV_64F);
    transObjToCam = Mat::zeros(3, 1, CV_64F);
    camproj.load("calibrationCamera.yml", "calibrationProjector.yml", "CameraProjectorExtrinsics.yml");
}

void testApp::setupTracker(){
    trackedImg.loadImage("yidunion.jpg");
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
     
            
            
        }
	}
}

void testApp::draw() {
    
    ofSetColor(255);
    cam.draw(0, 0, 1280, 800);
    
    
    
	if(bFound) {
        
        if(bDrawDebug){
//            tracker.draw();
            
            int i;
            const auto pts = tracker.getQuad();
            
            if (pts.size() > 3){
                
                corners[0].set(pts[0].x, pts[0].y);
                corners[1].set(pts[1].x, pts[1].y);
                corners[2].set(pts[2].x, pts[2].y);
                corners[3].set(pts[3].x, pts[3].y);
                
//                ofxQuadWarp(img2, corners[0], corners[1], corners[2], corners[3], 40, 40);
            }
            
            
        }
        
        
        
        if(bDrawWithCV) drawUsingCV();
        else drawUsingGL();
	}
    

    


}

void testApp::drawUsingCV(){
    
    cout << "CV!" <<endl;
    
    // set some input points
    float w = 12.5 / 2;
    int h = 19 / 2;
    vector<Point3f> inPts;
    inPts.push_back(Point3f(-w, -h, 0));
    inPts.push_back(Point3f(w, -h, 0));
    inPts.push_back(Point3f(w, h, 0));
    inPts.push_back(Point3f(-w, h, 0));
    
    // get videoproj's projection of inputPts
    vector<cv::Point2f> outPts = camproj.getProjected(inPts, rotObjToCam, transObjToCam);
    
    // project the inputPts over the object
    ofPushMatrix();
    ofTranslate(1280, 0);
    ofSetColor(ofColor::red);
    for (int i=0; i<outPts.size(); i++){
        int next = (i+1) % outPts.size();
        ofLine(outPts[i].x, outPts[i].y, outPts[next].x, outPts[next].y);
    }
    
    ofTranslate(0,0.15*0.5,0);
    ofFill();
    ofSetColor(ofColor::red,50);
    ofDrawBox(0.15);
    ofNoFill();
    ofSetColor(ofColor::red);
    ofDrawBox(0.15);

    ofPopMatrix();
    
    // display some infos
    ofSetColor(ofColor::white);
    stringstream ptss;
    ptss << "Points projections : \n";
    for (int i=0; i<4; i++) {
        ptss << "Input : " << inPts[i] << "\t> Output :" << outPts[i] << "\n";
    }
    
   
}

void testApp::drawUsingGL(){
    
    cout << "FOUND!" << endl;
    
    
    glMatrixMode( GL_MODELVIEW );
    
    ofPushMatrix();
    
    // Set perspective matrix using the projector intrinsics
    
    camproj.getCalibrationProjector().getDistortedIntrinsics().loadProjectionMatrix(10., 10000., cv::Point2d(0, 0));


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
    
    

        ofSetColor(ofColor::seaShell);
    
    ofNoFill();
        ofRect(-w * 0.5, -h * 0.5, w, h);
    
    ofFill();

    ofDrawBox(0, 0, 0, 100, 100, 50);

    
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
        case ' ':
            bDrawWithCV = !bDrawWithCV;
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
    
    
    void testApp::ofxQuadWarp(ofBaseHasTexture &tex, ofPoint lt, ofPoint rt, ofPoint rb, ofPoint lb, int rows, int cols) {
        float tw = tex.getTextureReference().getWidth();
        float th = tex.getTextureReference().getHeight();
        
        ofMesh mesh;
        
        for (int x=0; x<=cols; x++) {
            float f = float(x)/cols;
            ofPoint vTop(ofxLerp(lt,rt,f));
            ofPoint vBottom(ofxLerp(lb,rb,f));
            ofPoint tTop(ofxLerp(ofPoint(0,0),ofPoint(tw,0),f));
            ofPoint tBottom(ofxLerp(ofPoint(0,th),ofPoint(tw,th),f));
            
            for (int y=0; y<=rows; y++) {
                float f = float(y)/rows;
                ofPoint v = ofxLerp(vTop,vBottom,f);
                mesh.addVertex(v);
                mesh.addTexCoord(ofxLerp(tTop,tBottom,f));
            }
        }
        
        for (float y=0; y<rows; y++) {
            for (float x=0; x<cols; x++) {
                mesh.addTriangle(ofxIndex(x,y,cols+1), ofxIndex(x+1,y,cols+1), ofxIndex(x,y+1,cols+1));
                mesh.addTriangle(ofxIndex(x+1,y,cols+1), ofxIndex(x+1,y+1,cols+1), ofxIndex(x,y+1,cols+1));
            }
        }
        
        tex.getTextureReference().bind();
        mesh.draw();
        tex.getTextureReference().unbind();
//        mesh.drawVertices();
    }

ofMatrix4x4 testApp::getProjectionMatrix(){
	camParams.glGetProjectionMatrix(size,size,projMatrix,0.05,100,false);
	for(int i=0;i<16;i++){
		ofprojMatrix.getPtr()[i]=projMatrix[i];
	}
	return ofprojMatrix;
}





