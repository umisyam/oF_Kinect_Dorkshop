//
//  Particle.hpp
//  oF_Kinect_04-OpenCV_Particle
//
//  Created by Umi Syam on 11/21/15.
//  Combining examples by Ben McChesney https://github.com/benMcChesney/OF_Kinect_Tutorials
//

#pragma once
#include "ofMain.h"

enum particleMode{
    PARTICLE_MODE_ATTRACT = 0,
    PARTICLE_MODE_REPEL,
    PARTICLE_MODE_NEAREST_POINTS,
    PARTICLE_MODE_NOISE
};

class Particle{
public:
    Particle();
    
    void setMode(particleMode newMode);
    void setAttractPoints( vector <ofPoint> * attract );
    
    void reset();
    void update( float x , float y );
    void draw();
    
    ofPoint pos;
    ofPoint vel;
    ofPoint frc;
    
    float drag;
    float uniqueVal;
    float scale;
    
    particleMode mode;
    
    vector <ofPoint> * attractPoints; 
};