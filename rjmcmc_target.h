/////////////////////////////////////////////////////////////////////////////////////////////////
///           This class implements different operations on larvae models                     ///
/////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef RJMCMC_TARGET_H
#define RJMCMC_TARGET_H

#pragma once
#include "opencv3/include/opencv2/opencv.hpp"
#include "basic_calc.h"

using namespace std;
using namespace cv;

//struct to save larva model X_{i,t}
struct target_data{
    //larva model
    vector<circ> model;
    vector<Point2f> spine;

    //additional information are saved in order to reduce recalculations
    Rect bounding_box;
    vector<int> mid_idx;//indices of circle midpoints on spine
    vector<float> angles;//angles between model segments
    vector<float> distances;//distances between model points
};

//struct to save a sample
struct sample{
    std::vector<target_data> targets;

    //additional lists are saved in order to reduce recalculations
    std::vector<int> active;//indices of active larvae
    std::vector<int> inactive;//indices of inactive larvae
    std::vector<bool> is_active;//list of current larvae states

    double probability = 1.0; //saves probability of sample in order to find best sample

    void clear(){
        targets.clear();
        active.clear();
        inactive.clear();
        is_active.clear();
    }

    void changeTargetStatus(int idx){
        if(0<=idx && idx < (int)(targets.size())){
            if(is_active[idx]){
                is_active[idx]=false;
                inactive.push_back(idx);
                active.erase(std::find(active.begin(), active.end(), idx));
            }
            else{
                is_active[idx]=true;
                active.push_back(idx);
                inactive.erase(std::find(inactive.begin(), inactive.end(), idx));
            }
        }
    }
};

namespace RJMCMC_Target
{
    //create new target struct from calculated model
    target_data targetFromModel(const vector<circ>& model);
    //calculate bounding box of model
    Rect calcBoundingBox(const vector<circ>& model);
    //returns true if the bounding boxes of the two targets intersect each other
    bool boxIntersection(const target_data& target_1, const target_data& target_2);
    //returns size of model intersection
    int modelIntersection(const target_data& target_1, const target_data& target_2);
    //returns overlap ratios for modelcircles of target1
    vector<float> modelIntersectionRatios(const target_data& target_1, const target_data& target_2);
    //returns size of contour-model-intersection
    float contourModelIntersection(const target_data& target, const vector<Point>& contour, const Rect& bounding_box);
}

#endif // RJMCMC_TARGET_H
