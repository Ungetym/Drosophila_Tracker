#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#pragma once
#include "opencv2/opencv.hpp"
#include "rjmcmc_target.h"
#include "system_state.h"

class Motion_Model
{
public:
    Motion_Model(System_State* state);
    //propose update for given target
    bool update(target_data* target);
    //set average spine length for the current sequence
    void setSpineLength(float length);

private:
    System_State* state;

    RNG rng;

    //curve larva head
    void curveSpineHead(target_data* target);
    //curve larva tail
    void curveSpineTail(target_data* target);
    //expand or retract to head
    void expandSpine(target_data* target);
    //expand or retract to tail
    void expandSpineReverse(target_data* target);
    //translation 1
    void translateSpinePart(target_data* target);
    //translation 2
    void translateCurvedPart(target_data* target, vector<int>* curved);

    //check model restrictions
    bool checkRestrictions(target_data* target);

    //mirrors head to opposite larva side
    void correctHeadDirection(target_data *target);

};

#endif // MOTION_MODEL_H
