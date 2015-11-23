#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(100, 100, 100);
    ofSetFrameRate(60);
    
    // First thing's first! This line enables depth->video image calibration
    kinect.setRegistration(true);
    
    kinect.init();                  // by default, shows RGB video
    //    kinect.init(true);            // shows Infrared video instead of RGB
    //    kinect.init(false, false);    // disable video image (faster fps)
    
    kinect.open();                  // opens first available kinect
    //    kinect.open(1);               // open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //    kinect.open("A00366908064103A");	// open a kinect using it's unique serial #
    
    // Get all the kinect information
    if(kinect.isConnected()) {
        ofLogNotice() << "Get device serial number: " << kinect.getSerial() << endl;
        ofLogNotice() << "Num of kinect connected: " << kinect.numConnectedDevices() << endl;
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
    // Set the logging level - useful for debugging. Default is `OF_LOG_NOTICE`
    ofSetLogLevel(OF_LOG_VERBOSE);
    
}

//--------------------------------------------------------------
void ofApp::update(){
    ofSetWindowTitle( "oF+Kinect Dorkshop - FPS: "+ ofToString(ofGetFrameRate())) ;
    kinect.update();
    
    // there is a new frame and we are connected
    if(kinect.isFrameNew()) {
        cout << "new frame " << endl;
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    kinect.drawDepth(10, 10, 400, 300);
    kinect.draw(420, 10, 400, 300);
}

//--------------------------------------------------------------
void ofApp::exit(){
    kinect.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 
    
}
