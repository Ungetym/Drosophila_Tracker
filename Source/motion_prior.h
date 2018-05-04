#ifndef MOTION_PRIOR_H
#define MOTION_PRIOR_H

#include "rjmcmc_target.h"
#include "basic_calc.h"
#include "system_state.h"


class Motion_Prior
{
public:
    Motion_Prior(System_State* state);

    //init LUT for single motion prior calculation according to avg_spine_length
    void calculateLUT();

    //function to calculate motion prior quotient for the acceptance rate
    float calculateMotionPrior(sample* proposed_sample, sample* previous_sample, int changed_target, int proposal_type, std::vector<sample> *last_samples);

    //calculates interaction potential quotient for the acceptance rate
    float calculateInteractionPotentialRatio(sample* proposed_sample, int target_number ,int proposal_type, std::vector<sample> *current_samples);

    //copies "proposed_"-variables to the regular ones if proposal is accepted
    void acceptProposal();

private:
    System_State* state;

    //motion priors for function calculateMotionPrior
    std::vector<std::vector<float>> proposed_single_motion_priors;
    std::vector<std::vector<float>> last_single_motion_priors;
    float proposed_motion_prior;
    float last_motion_prior;

    //calculates a single motion prior P(X_{i,t}|X_{i,t-1})
    float singleMotionModel(target_data* X, target_data* Y);


    //interaction potentials
    std::vector<std::vector<float>> interaction_potentials;
    std::vector<std::vector<float>> proposed_interaction_potentials;
    std::vector<std::vector<float>> interactions;
    std::vector<std::vector<float>> proposed_interactions;

    //LUT for single motion prior calculation
    cv::Mat lut;
    int lut_size;

};

#endif // MOTION_PRIOR_H
