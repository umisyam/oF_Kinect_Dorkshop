//
//  openFrameworks + Kinect Dorkshop Fall 2015
//  Parsons School of Design
//  oF_Kinect_02-OpenCV TEMPLATE
//
//  Created by Umi Syam on 11/21/15.
//
//  Modified from https://github.com/ofTheo/ofxKinect/tree/master/kinectExample , with GUI interaction
//

#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(100);
    ofSetFrameRate(60);
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    // Kinect Setup
    kinect.setRegistration(true);
    kinect.init();
    kinect.open();
    
    // Print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "Get device serial number: " << kinect.getSerial() << endl;
        ofLogNotice() << "Num of kinect connected: " << kinect.numConnectedDevices() << endl;
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);

    bKinectOpen = true ;
    
    //---------------------------------Setup ofxUI
    float size = 24;
    guiWidth = 200;
    
    gui = new ofxUISuperCanvas("oF + Kinect Dorkshop", 0, 0, guiWidth, ofGetHeight(), OFX_UI_FONT_MEDIUM);
    gui->addLabel("KINECT PARAMETERS", OFX_UI_FONT_MEDIUM);
    gui->addSpacer();
    gui->addSpacer();
    gui->addLabel("'g' to show/hide GUI.", OFX_UI_FONT_SMALL);
    gui->addLabel("0-5 to change the led mode.", OFX_UI_FONT_SMALL);
    gui->addLabel("'p' or 'P' to show/hide padding.", OFX_UI_FONT_SMALL);
    gui->addLabel("'w' to enableDepthNearValueWhite.", OFX_UI_FONT_SMALL);

    gui->addSpacer();
    gui->addRangeSlider("DEPTH RANGE", 0.0, 255.0, farThreshold, nearThreshold);
    gui->addRangeSlider("BLOB SIZE", 0.0, ((kinect.width * kinect.height ) / 2 ), minBlobSize , maxBlobSize);
    gui->addSlider("MOTOR ANGLE", -20.0f, 30.0f, angle, guiWidth, size);
    gui->addSpacer();
    gui->addToggle("OPEN KINECT", bKinectOpen, size, size);
    gui->addToggle("THRESHOLD OPENCV", bThreshWithOpenCV, size, size);
    gui->addToggle("FULLSCREEN", false, size, size);
    
    ofAddListener(gui->newGUIEvent, this, &ofApp::guiEvent);
    gui->autoSizeToFitWidgets();
    gui->loadSettings("kinectSettings.xml");

}

//--------------------------------------------------------------
void ofApp::update(){
    
    ofSetWindowTitle( "oF+Kinect Dorkshop - FPS: "+ ofToString(ofGetElapsedTimef())) ;
    kinect.update();
    
    // there is a new frame and we are connected
    if(kinect.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        if(bThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {
            
            // or we do it ourselves - show people how they can work with the pixels
            unsigned char * pix = grayImage.getPixels();
            
            int numPixels = grayImage.getWidth() * grayImage.getHeight();
            for(int i = 0; i < numPixels; i++) {
                if(pix[i] < nearThreshold && pix[i] > farThreshold) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
        }
        
        // update the cv images
        grayImage.flagImageChanged();
        
        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        // findContours( ofxCvGrayscaleImage&  input, int minArea, int maxArea, int nConsidered, bool bFindHoles, bool bUseApproximation ) ;
        contourFinder.findContours(grayImage, minBlobSize , maxBlobSize , 20, false);
    }

}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255);
    
    ofPushMatrix() ;
        // draw from the live kinect
        kinect.drawDepth(220, 0, 400, 300);
        kinect.draw(640, 0, 400, 300);
    
        grayImage.draw(220, 310, 400, 300);
        contourFinder.draw(220, 320, 400, 300);
    ofPopMatrix() ;
    
}

//--------------------------------------------------------------
void ofApp::exit(){
    kinect.close();
    gui->saveSettings("kinectSettings.xml");
    delete gui;
}

//--------------------------------------------------------------
void ofApp::guiEvent(ofxUIEventArgs &e)
{
    string name = e.widget->getName();
    int kind = e.widget->getKind();
    
    if(name == "DEPTH RANGE")
    {
        ofxUIRangeSlider *slider = (ofxUIRangeSlider *) e.widget;
        farThreshold = slider->getScaledValueLow() ;
        nearThreshold = slider->getScaledValueHigh() ;
    }
    
    if(name == "BLOB SIZE")
    {
        ofxUIRangeSlider *slider = (ofxUIRangeSlider *) e.widget;
        minBlobSize = slider->getScaledValueLow() ;
        maxBlobSize = slider->getScaledValueHigh() ;
    }
    
    if(name == "THRESHOLD OPENCV" )
    {
        ofxUIToggle *toggle = (ofxUIToggle *) e.widget;
        bThreshWithOpenCV = toggle->getValue() ;
    }
    
    if(name == "MOTOR ANGLE" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        angle = slider->getScaledValue() ;
        kinect.setCameraTiltAngle(angle);
    }
    
    if(name == "OPEN KINECT" )
    {
        ofxUIToggle *toggle = (ofxUIToggle *) e.widget;
        bKinectOpen = toggle->getValue() ;
        if ( bKinectOpen == true )
            kinect.open() ;
        else
            kinect.close() ;
    }
    
    if(name == "FULLSCREEN")
    {
        ofxUIToggle *toggle = (ofxUIToggle *) e.widget;
        ofSetFullscreen(toggle->getValue());
    }
    
    gui->saveSettings("GUI/kinectSettings.xml") ; 
}



//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch (key)
    {
        case 'g':
            gui->toggleVisible();
            break;
            
        case 'p':
            gui->setDrawWidgetPadding(true);
            break;
            
        case 'P':
            gui->setDrawWidgetPadding(false);
            break;
            
        case 'w':
            kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
            break;
            
        case '1':
            kinect.setLed(ofxKinect::LED_GREEN);
            break;
            
        case '2':
            kinect.setLed(ofxKinect::LED_YELLOW);
            break;
            
        case '3':
            kinect.setLed(ofxKinect::LED_RED);
            break;
            
        case '4':
            kinect.setLed(ofxKinect::LED_BLINK_GREEN);
            break;
            
        case '5':
            kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
            break;
            
        case '0':
            kinect.setLed(ofxKinect::LED_OFF);
            break;
            
        default:
            break;
    }
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
