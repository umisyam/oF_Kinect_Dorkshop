//
//  Bubble.h
//  vectorStorage
//
//  Created by Umi Syam on 11/4/14.
//
//

#pragma once
#include "ofMain.h"

class Bubble {
public:
    Bubble();
    
    int x, y, xVel, yVel;
    ofColor color;
    float bubbleSize;
    
    void setup(int _x, int _y);
    void update();
    void draw();
};