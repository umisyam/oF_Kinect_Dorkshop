//
//  Bubble.cpp
//  vectorStorage
//
//  Created by Umi Syam on 11/4/14.
//
//

#include "Bubble.h"

Bubble::Bubble() {
    x = ofGetWindowWidth() * 0.5;
    y = ofGetWindowHeight() * 0.5;
    xVel = ofRandom(-5,5);
    yVel = ofRandom(-5,5);
}

void Bubble::setup(int _x, int _y) {
    x = _x;
    y = _y;
    bubbleSize = ofRandom(10,50);
    
    color = ofColor(ofRandom(255),ofRandom(255),ofRandom(255));
    
    
}

void Bubble::update() {
    x += xVel;
    y += yVel;
    
    if ((x > ofGetWindowWidth()) || (x < 0)) {
        xVel *= (-1);
    }
    if ((y > ofGetWindowHeight()) || (y < 0)) {
        yVel *= (-1);
    }
}

void Bubble::draw() {
    ofSetColor(color);
    ofCircle(x, y, bubbleSize);
}