//
//  openFrameworks + Kinect Dorkshop Fall 2015
//  Parsons School of Design
//  oF_Kinect_05-OpenCV_Ribbon
//
//  Created by Umi Syam on 11/21/15.
//
//  Modified from https://github.com/ofTheo/ofxKinect/tree/master/kinectExample , with GUI interaction
//  Combining examples by Ben McChesney https://github.com/benMcChesney/OF_Kinect_Tutorials


#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"
#include "ofxUI.h"

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    void exit();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    ofxKinect kinect;
    ofxCvColorImage colorImg;
    
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    
    ofxCvContourFinder contourFinder;
    
    bool bThreshWithOpenCV;
    int nearThreshold;
    int farThreshold;
    int angle;
    float minBlobSize , maxBlobSize;
    bool bKinectOpen;
    
    //ofxUI variables
    ofxUICanvas *gui;
    float guiWidth ;
    void guiEvent(ofxUIEventArgs &e);
    
    //Each frame take the number of blobs and create cursors at their centroids
    vector<ofVec2f> cursors ;
    float cursorXSensitivity ;
    float cursorYSensitivity ;
    bool bRestrictCursors ;
    float cursorBorderPadding ;
    
    //this holds all of our points
    vector<ofVec3f> points;
    //this keeps track of the center of all the points
    ofVec3f center;
    
    //our camera objects for looking at the scene from multiple perspectives
//    ofCamera camera;
    ofEasyCam camera;
    
    //if usecamera is true, we'll turn on the camera view
    bool usecamera;
    
    ofLight light ;
    
    int maxCursors ;
};
