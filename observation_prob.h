#ifndef OBSERVATION_PROB_H
#define OBSERVATION_PROB_H

#include "rjmcmc_target.h"
#include "basic_calc.h"
#include "system_state.h"

//struct to save observation data Z_t
struct observation_data{
    cv::Mat binary_image;
    std::vector<std::vector<cv::Point>> detected_contours;

    //in addition bounding boxes of the contours are saved to reduce recomputations
    std::vector<cv::Rect> detected_contour_boxes;
    std::vector<float> detected_contour_area_sizes;

    void setContours(std::vector<std::vector<cv::Point>>& contours){
        detected_contours = contours;
        //set bounding boxes
        detected_contour_boxes.clear();
        for(size_t i=0;i<detected_contours.size();i++){
            detected_contour_boxes.push_back(cv::boundingRect(detected_contours[i]));
        }
    }
};

struct prob_factors{
    double a_1;
    double a_2;
    std::vector<double> a_3;
};

class Observation_Prob
{
public:
    Observation_Prob(System_State* state);

    //calculate observation probability ratio between old sample and proposed sample
    double observationProbability(sample *old_sample, sample *proposed_sample, int changed_target, int proposal_type, observation_data *observation);
    //copy proposed data if proposed sample is accepted
    void acceptProposal(){
        unmatched_models=proposed_unmatched_models;
        matched_contour=proposed_matched_contour;
        corresponding_models=proposed_corresponding_models;
        overlapping_noncorresponding_models=proposed_overlapping_noncorresponding_models;
        single_probabilities=proposed_single_probabilities;
        last_prior=proposed_prior;
    }

private:
    System_State* state;

    std::vector<int> unmatched_models, proposed_unmatched_models;// larvae models that do not intersect with any contour
    std::vector<int> matched_contour, proposed_matched_contour;// save the contour idx for every larva

    std::vector<std::vector<std::pair<int,float>>> corresponding_models, proposed_corresponding_models; //save model idx and overlap for every contour
    std::vector<std::vector<std::pair<int,float>>> overlapping_noncorresponding_models, proposed_overlapping_noncorresponding_models; //save model idx and overlap for every contour

    std::vector<prob_factors> single_probabilities, proposed_single_probabilities;// contain the ratios a_1, a_2, a_3 for all contours

    double last_prior, proposed_prior;

    void calculateOverlapRatios(int contour_idx, observation_data *observation, sample* proposed_sample, prob_factors* probs);
};

#endif // OBSERVATION_PROB_H
