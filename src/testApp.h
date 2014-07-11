#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxCvFeaturesTrackerThreaded.h"
#include "ofxCvCameraProjectorCalibration.h"
#include "cameraparameters.h"

#define SCREEN_WIDTH 1280
#define SCREEN_HEIGHT 800

#define PROJECTOR_WIDTH 1280
#define PROJECTOR_HEIGHT 800

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
    void exit();
    void keyPressed(int key);
	
    // draw with OpenCV projection method (slower but useful for debug)
    void drawUsingCV();
    
    // draw using OpenGL projection directly
    void drawUsingGL();
    
//    std::vector<cv::Point3d> vertices();
    
//    cv::Point3d markerObjectPoints[4];
    vector<cv::Point3d> markerObjectPoints;
     vector<cv::Point2d> imagePoints;
    void drawCube();
    
    ofImage img;
    ofImage img2;
    
    ofPoint corners[4];
    int selectedCorner;
    
    cv::Mat objPM;
    cv::Mat obj2;
    
    cv::Size size;
    double projMatrix[16];
	float projfMatrix[16];
    ofMatrix4x4 ofprojMatrix;
    
    ofMatrix4x4 getProjectionMatrix();
    	aruco::CameraParameters camParams;
    
    
    
    
    ofPoint ofxLerp(ofPoint start, ofPoint end, float amt);
    int ofxIndex(float x, float y, float w);
    void ofxQuadWarp(ofBaseHasTexture &tex, ofPoint lt, ofPoint rt, ofPoint rb, ofPoint lb, int rows, int cols);
    void mousePressed(int x, int y, int button);
    void mouseDragged(int x, int y, int button);
    void mouseReleased(int x, int y, int button);

    
private:
    
    void setupCamProj();
    void setupTracker();
    
	ofVideoGrabber cam;
    ofImage trackedImg;
    
    ofxCv::FeaturesTracker tracker;
    ofxCv::CameraProjectorCalibration camproj;
    cv::Mat rotObjToCam, transObjToCam;
    cv::Mat rotObjToProj, transObjToProj;
    
    bool bDrawDebug;
    bool bDrawWithCV;
    
    string getRTMatInfos(const cv::Mat rvecs, const cv::Mat tvecs);
    
    // since we're using the threaded tracker, "found" status can change between update() and draw() calls
    // so we store its value here
    bool bFound;
};
