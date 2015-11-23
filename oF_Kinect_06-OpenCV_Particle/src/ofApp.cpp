//
//  openFrameworks + Kinect Dorkshop Fall 2015
//  Parsons School of Design
//  oF_Kinect_04-OpenCV_Particle
//
//  Created by Umi Syam on 11/21/15.
//
//  Modified from https://github.com/ofTheo/ofxKinect/tree/master/kinectExample , with GUI interaction
//  Combining examples by Ben McChesney https://github.com/benMcChesney/OF_Kinect_Tutorials


#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(100);
    ofSetFrameRate(60);
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofEnableSmoothing();
    
    // Kinect Setup
    kinect.setRegistration(true);
    kinect.init();
    kinect.open();
    
    // Print the intrinsic IR sensor values
    if(kinect.isConnected()) {
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
    
    int num = 20000;
    ofSetCircleResolution( 10 ) ;
    p.assign(num, Particle());
    currentMode = PARTICLE_MODE_ATTRACT;
    currentModeStr = "1 - PARTICLE_MODE_ATTRACT: attracts to mouse";
    resetParticles();

    
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
    gui->addToggle("DRAW POINT CLOUD", bDrawPointCloud, size, size);
    gui->addToggle("FULLSCREEN", false, size, size);
    
    gui->addSpacer();
    gui->addSlider("X SENSITIVITY", 1.0, 5.0, cursorXSensitivity, guiWidth, size);
    gui->addSlider("Y SENSITIVITY", 1.0, 5.0, cursorYSensitivity, guiWidth, size);
    gui->addToggle("RESTRICT CURSORS", bRestrictCursors, size, size);
    gui->addSlider("CURSOR BORDER PADDING", 0.0, 50.0f, cursorBorderPadding, guiWidth, size);
    
    ofAddListener(gui->newGUIEvent, this, &ofApp::guiEvent);
    gui->autoSizeToFitWidgets();
    gui->loadSettings("kinectSettings.xml");
    
}

//--------------------------------------------------------------
void ofApp::resetParticles(){
    //these are the attraction points used in the forth demo
    attractPoints.clear();
    for(int i = 0; i < 4; i++){
        attractPoints.push_back( ofPoint( ofMap(i, 0, 4, 100, ofGetWidth()-100) , ofRandom(100, ofGetHeight()-100) ) );
    }
    
    attractPointsWithMovement = attractPoints;
    
    for(int i = 0; i < p.size(); i++){
        p[i].setMode(currentMode);
        p[i].setAttractPoints(&attractPointsWithMovement);;
        p[i].reset();
    }
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
        
        //Reset the cursors
        cursors.clear() ;
        
        //For each openCV blob normalize it and scale it by the x + y sensitivty
        //and then scale it by the window space , and prevent it from going off screen like a cursor
        for ( int i = 0 ; i < contourFinder.nBlobs ; i++ )
        {
            //Get the centroid ( which is relative to the depthImage
            ofVec2f iCentroid = contourFinder.blobs[i].centroid ;
            
            //Normalize the centroid ( make it between 0 and 1 )
            iCentroid.x /= kinect.width ;
            iCentroid.y /= kinect.height ;
            
            //offset coordinates so that the sensitivity scales in all directions ( 0 , 0 ) in the middle
            iCentroid += ofVec2f( -0.5 , -0.5 ) ;
            
            //Scale it by cursor sensitivity
            iCentroid.x *= cursorXSensitivity ;
            iCentroid.y *= cursorYSensitivity ;
            
            //remove the offset but keep the sensitivity scaing
            iCentroid -= ofVec2f( -0.5 , -0.5 ) ;
            
            
            iCentroid.x *= ofGetWidth() ;
            iCentroid.y *= ofGetHeight() ;
            
            if ( bRestrictCursors == true )
            {
                iCentroid.x = ofMap ( iCentroid.x , 0.0 , ofGetWidth() , cursorBorderPadding , ofGetWidth() - cursorBorderPadding , true ) ;
                iCentroid.y = ofMap ( iCentroid.y , 0.0 , ofGetHeight() , cursorBorderPadding , ofGetHeight() - cursorBorderPadding , true ) ;
            }
            cursors.push_back( iCentroid ) ;
        }
    }
    
    if ( cursors.size() > 0 )
    {
        for(int i = 0; i < p.size(); i++){
            p[i].setMode(currentMode);
            p[i].setAttractPoints(&cursors) ;
            p[i].update( cursors[0].x , cursors[0].y );
        }
    }
    else
    {
        ofPoint center = ofPoint ( ofGetWidth() / 2 , ofGetHeight() / 2 ) ;
        for(int i = 0; i < p.size(); i++){
            p[i].setMode(currentMode);
            // p[i].setAttractPoints( &cursors ) ;
            p[i].update( center.x , center.y );
        }
    }
    
    //lets add a bit of movement to the attract points
    for(int i = 0; i < attractPointsWithMovement.size(); i++){
        attractPointsWithMovement[i].x = attractPoints[i].x + ofSignedNoise(i * 10, ofGetElapsedTimef() * 0.7) * 12.0;
        attractPointsWithMovement[i].y = attractPoints[i].y + ofSignedNoise(i * -10, ofGetElapsedTimef() * 0.7) * 12.0;
    }
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255);
    
    if(bDrawPointCloud) {
        easyCam.begin();
        drawPointCloud();
        easyCam.end();
        
    } else {
        ofPushMatrix() ;
        // draw from the live kinect
        kinect.drawDepth(220, 0, 400, 300);
        kinect.draw(640, 0, 400, 300);
        
        grayImage.draw(220, 310, 400, 300);
        contourFinder.draw(220, 320, 400, 300);
        ofPopMatrix() ;
    }
    
    //If there are any cursors
    if ( cursors.size() > 0 )
    {
        //Iterate through
        for ( int i = 0 ; i < cursors.size() ; i++ )
        {
            //Give each a unique color based on it's index
            float ratio = (float) i / ( float) cursors.size() ;
            ofSetColor( ofColor::fromHsb(225.0f * ratio , 255 , 255 ) ) ;
            ofCircle( cursors[i] , 12 ) ;
        }
    }
    
    for(int i = 0; i < p.size(); i++){
        p[i].draw();
    }
    
    ofSetColor(190);
    if( currentMode == PARTICLE_MODE_NEAREST_POINTS ){
        for(int i = 0; i < attractPoints.size(); i++){
            ofNoFill();
            ofCircle(attractPointsWithMovement[i], 10);
            ofFill();
            ofCircle(attractPointsWithMovement[i], 4);
        }
    }
    
    ofSetColor(230);
    ofDrawBitmapString(currentModeStr + "\n\nSpacebar to reset. \nKeys 1-4 to change mode.", 10, 20);
    
}

void ofApp::drawPointCloud() {
    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 3;
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(kinect.getDistanceAt(x, y) > 0) {
                mesh.addColor(kinect.getColorAt(x,y));
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
            }
        }
    }
    glPointSize(5);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000); // center the points a bit
    glEnable(GL_DEPTH_TEST);
    mesh.drawVertices();
    glDisable(GL_DEPTH_TEST);
    ofPopMatrix();
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
    
    if(name == "X SENSITIVITY" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        cursorXSensitivity = slider->getScaledValue() ;
    }
    
    if(name == "Y SENSITIVITY" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        cursorYSensitivity = slider->getScaledValue() ;
    }
    
    if(name == "RESTRICT CURSORS" )
    {
        ofxUIToggle *toggle = (ofxUIToggle *) e.widget;
        bRestrictCursors = toggle->getValue() ;
    }
    
    if(name == "CURSOR BORDER PADDING" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        cursorBorderPadding = slider->getScaledValue() ;
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
            currentMode = PARTICLE_MODE_ATTRACT;
            currentModeStr = "1 - PARTICLE_MODE_ATTRACT: attracts to mouse";
            break;
            
        case '2':
            kinect.setLed(ofxKinect::LED_YELLOW);
            currentMode = PARTICLE_MODE_REPEL;
            currentModeStr = "2 - PARTICLE_MODE_REPEL: repels from mouse";
            break;
            
        case '3':
            kinect.setLed(ofxKinect::LED_RED);
            currentMode = PARTICLE_MODE_NEAREST_POINTS;
            currentModeStr = "3 - PARTICLE_MODE_NEAREST_POINTS: hold 'f' to disable force";
            break;
            
        case '4':
            kinect.setLed(ofxKinect::LED_BLINK_GREEN);
            currentMode = PARTICLE_MODE_NOISE;
            currentModeStr = "4 - PARTICLE_MODE_NOISE: snow particle simulation";
            break;
            
        case '5':
            kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
            break;
            
        case '0':
            kinect.setLed(ofxKinect::LED_OFF);
            resetParticles();
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
