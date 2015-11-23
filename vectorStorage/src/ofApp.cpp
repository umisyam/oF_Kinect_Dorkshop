#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(0);
    for (int i=0; i<10; i++) {
        int temp = ofRandom(30);
        myNumbers.push_back(temp);
    }
    for (int i=0; i<10; i++) {
        Bubble tempBubble;
        tempBubble.setup(ofRandomWidth(), ofRandomHeight());
        myBubbles.push_back(tempBubble);
    }
}

//--------------------------------------------------------------
void ofApp::update(){
    for (int i=0; i<myBubbles.size(); i++) {
        myBubbles[i].update();
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    for (int i=0; i<myNumbers.size(); i++) {
        ofDrawBitmapString(ofToString(myNumbers[i]), 10, i*20);
    }
    for (int i=0; i<myBubbles.size(); i++) {
        myBubbles[i].draw();
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if (myBubbles.size() > 0) {
        myBubbles.erase(myBubbles.begin());
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
    for (int i=0; i<5; i++) {
        Bubble newBubble;
        newBubble.setup(x, y);
        myBubbles.push_back(newBubble);
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

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
