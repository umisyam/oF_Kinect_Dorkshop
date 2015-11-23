//
//  openFrameworks + Kinect Dorkshop Fall 2015
//  Parsons School of Design
//  oF_Kinect_04-OpenCV_PointCloudTrails
//
//  Created by Umi Syam on 11/21/15.
//
//  Modified from https://github.com/ofTheo/ofxKinect/tree/master/kinectExample , with GUI interaction
//

#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
//    ofBackground(100);
    ofSetFrameRate(60);
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofEnableSmoothing();
    
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
    bDrawPointCloud = false;
    
    trailsAlpha = 7.0f;
    trailsFbo.allocate(ofGetWidth(), ofGetHeight(), GL_RGBA32F);
    trailsFbo.begin();
        ofClear(1,1,1,0);
    trailsFbo.end();
    
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
    gui->addSpacer();
    //add new toggle here
    gui->addToggle("DRAW POINT CLOUD", bDrawPointCloud, size, size);
    //add new slider here
    gui->addRangeSlider("Z RANGE", 0, 2000, pointCloudMinZ , pointCloudMaxZ);
    gui->addSpacer();
    gui->addToggle("FULLSCREEN", false, size, size);
    
    ofAddListener(gui->newGUIEvent, this, &ofApp::guiEvent);
    gui->autoSizeToFitWidgets();
    gui->loadSettings("kinectSettings.xml");
    
}

//--------------------------------------------------------------
void ofApp::update(){
    
    ofSetWindowTitle( "oF+Kinect Dorkshop - FPS: "+ ofToString(ofGetFrameRate())) ;
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
    //let's draw black background
    ofBackground(0,0,0);
    
    ofSetColor(255,255,255);
    ofPushMatrix();
    
    if(bDrawPointCloud) {
        easyCam.begin();
        drawPointCloud();
        easyCam.end();
        
        trailsFbo.begin();
            ofSetColor(0,0,0,trailsAlpha);
            ofDrawRectangle(0, 0, ofGetWidth(), ofGetHeight());
        
            easyCam.begin();
            drawPointCloud();
            easyCam.end();
        trailsFbo.end();
        
        ofSetColor(255,255,255);
        trailsFbo.draw(0,0);
    }
    else {
        ofPushMatrix() ;
        // draw from the live kinect
        kinect.drawDepth(220, 0, 400, 300);
        kinect.draw(640, 0, 400, 300);
        
        grayImage.draw(220, 310, 400, 300);
        contourFinder.draw(220, 320, 400, 300);
        ofPopMatrix() ;
    }
    
    ofPopMatrix();
    
}

void ofApp::drawPointCloud() {
//    int w = 640;
//    int h = 480;
    int w = ofGetWidth();
    int h = ofGetHeight();
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    
    // ------------Uncomment this if you want just a normal PointCloud
    //    int step = 3;
    //    for(int y = 0; y < h; y += step) {
    //        for(int x = 0; x < w; x += step) {
    //            if(kinect.getDistanceAt(x, y) > 0) {
    //                mesh.addColor(kinect.getColorAt(x,y));
    //                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
    //            }
    //        }
    //    }
    //    glPointSize(5);
    //    ofPushMatrix();
    //        // the projected points are 'upside down' and 'backwards'
    //        ofScale(1, -1, -1);
    //        ofTranslate(0, 0, -1000); // center the points a bit
    //        glEnable(GL_DEPTH_TEST);
    //        mesh.drawVertices();
    //        glDisable(GL_DEPTH_TEST);
    //    ofPopMatrix();
    
    
    // Create an offset value to make rainbow using out-of-phase sine waves
    // Calculation based on this: http://krazydad.com/tutorials/makecolors.php
    // You can also try using ofNoise( ofGetElapsedTimef() ) instead sine waves
    // ofColor has a function called "fromHSB( hue , saturation , brightness )" that allows for easy color offset
    
    ofColor offset = ofColor::fromHsb( sin( ofGetElapsedTimef() ) * 128.0f + 128.0f, 255, 255);
    
    int step = 3;
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(kinect.getDistanceAt(x, y) > 0) {
                
                ofVec3f vertex = kinect.getWorldCoordinateAt(x, y) ;
                if ( vertex.z > pointCloudMinZ && vertex.z < pointCloudMaxZ )
                {
                    mesh.addVertex( vertex );
                    //Offset the color here
                    //ofColor col = kinect.getColorAt(x,y) + offset ;
                    ofColor col = kinect.getColorAt(x,y);
                    int hue = col.getHue();
                    hue += offset.getHue();
                    if (hue > 255) {
                        hue = 255 - hue;
                    }
                    ofColor newCol = ofColor::fromHsb( hue, 255, 255);
                    mesh.addColor( newCol );
                }
                
            }
        }
    }
    
    ofEnableBlendMode( OF_BLENDMODE_ALPHA ) ;
    glPointSize( ofNoise( ofGetElapsedTimef() ) * 12 + 8 );
//    glPointSize(3);
    
    ofPushMatrix();
        // the projected points are 'upside down' and 'backwards'
        ofScale(1, -1, -1);
        ofTranslate(0, 0, -1000); // center the points a bit
        glEnable(GL_DEPTH_TEST);
        mesh.drawVertices();
        glDisable(GL_DEPTH_TEST);
    ofPopMatrix();
    
    ofEnableBlendMode( OF_BLENDMODE_ALPHA ) ;
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
    
    if(name == "Z RANGE" )
    {
        ofxUIRangeSlider *slider = (ofxUIRangeSlider *) e.widget;
        pointCloudMinZ = slider->getScaledValueLow() ;
        pointCloudMaxZ = slider->getScaledValueHigh() ;
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
    
    if(name == "DRAW POINT CLOUD" )
    {
        ofxUIToggle *toggle = (ofxUIToggle *) e.widget;
        bDrawPointCloud = toggle->getValue() ;
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
