#ifndef RJMCMC_TRACKER_H
#define RJMCMC_TRACKER_H

#include "opencv2/opencv.hpp"
#include "observation_prob.h"
#include "motion_prior.h"
#include "motion_model.h"
#include "larva_model.h"
#include "basic_calc.h"
#include "system_state.h"

//for testing
#include <chrono>


class RJMCMC_Tracker
{
public:
    //constructor
    RJMCMC_Tracker(System_State* state);
    //init frame counter and clear old data before starting a new sequence
    void init();
    //track larvae based on a binary image and detected contours
    sample track(cv::Mat *image_binary, vector<vector<Point>> contours);

private:

    System_State* state;

    //////////////////////////////// INITIAL ESTIMATION  //////////////////////////////////
    //contains current image, detected contours, detected contour boxes and binary image of filled contours
    observation_data Z_t;
    //estimated new state
    sample estimated_state;
    //some proposals reorder the estimated state targets
    sample proposed_estimated_state;

    //calculates estimated_state and estimated_new_targets based on given sample X
    void estimateSample(sample* X);

    //////////////////////////////  RJMCMC-ALGORITHM  /////////////////////////////////////
    //samples for time t-1 and current time t
    std::vector<sample> last_samples;
    sample last_best_sample;

    std::vector<sample> current_samples;

    //proposal type probabilities
    float p_A, p_D, p_U;
    //target lists
    vector<int> targets_to_add, targets_to_delete;
    //proposed target lists
    vector<int> proposed_targets_to_add, proposed_targets_to_delete;
    //motion prior calculator
    Motion_Prior motion_prior;
    //class for proposing single model motions
    Motion_Model motion_proposer;
    //class for acceptance rate calculation
    Observation_Prob measurement_model;

    //generates N new samples
    bool rjmcmcSampling();
    //calculates acceptance rate for proposed sample
    float acceptanceRate(sample* proposed_sample, int proposal_type, int changed_target);

    ////////////////////////////////  HELPER FUNCTIONS  ///////////////////////////////////
private:
    //calculates average spine length
    void averageSpineAndArea();
    //currently needed for head selection
    static void mouseAction(int evt, int x, int y, int, void*userdata);
    //read heads from file
    void readHeads();
    //write heads to file
    void writeHead(Point2f headpoint);
    //extract best sample from the N calculated samples of current frame
    void calculateBestSample();
    //draws given sample onto image and saves the result
    void drawTargets(sample* sample, Mat *image, float zoom);
    ///////////////////////////////////////////////////////////////////////////////////////
};

#endif // RJMCMC_TRACKER_H
