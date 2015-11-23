//
//  openFrameworks + Kinect Dorkshop Fall 2015
//  Parsons School of Design
//  oF_Kinect_05-OpenCV_Ribbon
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
    usecamera = false;
    maxCursors = 2 ;
    for ( int i = 0 ; i < maxCursors ; i++ )
    {
        cursors.push_back( ofVec2f() ) ;
    }
    camera.enableMouseInput();
    
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
    
    gui->addSpacer();
    gui->addSlider("X SENSITIVITY", 1.0, 5.0, cursorXSensitivity, guiWidth, size);
    gui->addSlider("Y SENSITIVITY", 1.0, 5.0, cursorYSensitivity, guiWidth, size);
    gui->addToggle("RESTRICT CURSORS", bRestrictCursors, size, size);
    gui->addSlider("CURSOR BORDER PADDING", 0.0, 50.0f, cursorBorderPadding, guiWidth, size);
    gui->addSpacer();
    gui->addToggle("USE CAMERA", usecamera, size, size);
    
    ofAddListener(gui->newGUIEvent, this, &ofApp::guiEvent);
    gui->autoSizeToFitWidgets();
    gui->loadSettings("kinectSettings.xml");
    
}

//--------------------------------------------------------------
void ofApp::update(){
    
    
    //don't move the points if we are using the camera
    if(!usecamera){
        ofVec3f sumOfAllPoints(0,0,0);
        for(int i = 0; i < points.size(); i++){
            points[i].z -= 4;
            sumOfAllPoints += points[i];
        }
        center = sumOfAllPoints / points.size();
    }


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
        for ( int i = 0 ; i < cursors.size() ; i++)
        {
            ofVec3f point = cursors[i] ;
            //if we are using the camera, the mouse moving should rotate it around the whole sculpture
            if(usecamera){
                float rotateAmount = ofMap(point.x, 0, point.y , 0, 360);
                ofVec3f furthestPoint;
                if (points.size() > 0) {
                    furthestPoint = points[0];
                }
                else
                {
                    furthestPoint = ofVec3f(point.x, point.y, 0);
                }
                
                ofVec3f directionToFurthestPoint = (furthestPoint - center);
                ofVec3f directionToFurthestPointRotated = directionToFurthestPoint.rotated(rotateAmount, ofVec3f(0,1,0));
                camera.setPosition(center + directionToFurthestPointRotated);
                camera.lookAt(center);
            }
            //otherwise add points like before
            else{
                points.push_back( point );
            }
        }
        
    }
    
    //setup  light animations
    
    ofQuaternion xRotation , yRotation ;
    xRotation.makeRotate( sin( ofGetElapsedTimef() ), 1.0f, 0.0f, 0.0f ) ;
    yRotation.makeRotate( cos( ofGetElapsedTimef() ), 0.0f, 1.0f, 0.0f ) ;
    
    ofQuaternion lightRotation = xRotation * yRotation ;
    
    light.setGlobalOrientation( lightRotation ) ;

}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255);
    ofPushMatrix() ;
    ofTranslate( guiWidth + 10 , 0 ) ;
    
    ofEnableAlphaBlending() ;
    ofPushMatrix() ;
    ofTranslate( ofGetWidth() - 210 - guiWidth - 10  , ofGetHeight() - 160 ) ;
    // draw from the live kinect
    kinect.drawDepth(0 , 0, 200 , 150 );
    
    ofSetColor( 255 , 65 ) ;
    kinect.draw(0, 0, 200 , 150);
    contourFinder.draw(0, 0, 200, 150);
    ofPopMatrix() ;
    
    ofPopMatrix() ;
    
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
    //to debug to the screen use
    //ofDrawBitmapString( "STRING" , x , y ) ;
    
    //if we're using the camera, start it.
    //everything that you draw between begin()/end() shows up from the view of the camera
    if(usecamera){
        camera.begin();
    }
    ofSetColor(0);
    //do the same thing from the first example...
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
    for(int i = 1; i < points.size(); i++){
        
        //find this point and the next point
        ofVec3f thisPoint = points[i-1];
        ofVec3f nextPoint = points[i];
        
        //get the direction from one to the next.
        //the ribbon should fan out from this direction
        ofVec3f direction = (nextPoint - thisPoint);
        
        //get the distance from one point to the next
        float distance = direction.length();
        
        //get the normalized direction. normalized vectors always have a length of one
        //and are really useful for representing directions as opposed to something with length
        ofVec3f unitDirection = direction.normalized() + 0.1f ;
        
        //find both directions to the left and to the right
        ofVec3f toTheLeft = unitDirection.getRotated(-90, ofVec3f(0,0,1));
        ofVec3f toTheRight = unitDirection.getRotated(90, ofVec3f(0,0,1));
        
        //use the map function to determine the distance.
        //the longer the distance, the narrower the line.
        //this makes it look a bit like brush strokes
        float thickness = ofMap(distance, 0, 60, 40, 10, true);
        
        //calculate the points to the left and to the right
        //by extending the current point in the direction of left/right by the length
        ofVec3f leftPoint = thisPoint+toTheLeft*thickness;
        ofVec3f rightPoint = thisPoint+toTheRight*thickness;
        
        //add these points to the triangle strip
        
//        mesh.addColor ( ofColor::fromHsb( sin ( (float)i ) * 40.0f + 128.0f, 255.0f , 255.0f ) ) ;
//        mesh.addColor ( ofColor::fromHsb( sin ( (float)i ) * 40.0f + 128.0f, 255.0f , 255.0f ) ) ;
        
        mesh.addVertex(ofVec3f(leftPoint.x, leftPoint.y, leftPoint.z));
        mesh.addVertex(ofVec3f(rightPoint.x, rightPoint.y, rightPoint.z));
        
        mesh.addColor(ofColor::fromHsb( sin( ofGetElapsedTimef() ) * 128.0f + 128.0f, 255, 255));
        
        
    }
    
    // enable lighting //
    ofEnableLighting();
    
    glEnable(GL_DEPTH_TEST);
    
    // the position of the light must be updated every frame,
    // call enable() so that it can update itself //
    light.enable();
    
    
    //end the shape
    mesh.draw();
    
    light.disable() ;
    
    glDisable(GL_DEPTH_TEST) ;
    ofDisableLighting() ;
    
    //if we're using the camera, take it away
    if(usecamera){
        camera.end();
    }
    
    
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
    
    if(name == "USE CAMERA" )
    {
        ofxUIToggle *toggle = (ofxUIToggle *) e.widget;
        usecamera = toggle->getValue() ;
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
