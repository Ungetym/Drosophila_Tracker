#include "motion_prior.h"
#include "consts.h"

using namespace std;
using namespace cv;

Motion_Prior::Motion_Prior(System_State *state) :
    state(state)
{
}

float Motion_Prior::calculateMotionPrior(sample* proposed_sample, sample* previous_sample, int changed_target, int proposal_type, std::vector<sample> *last_samples){

    int calc_thinning = 10; //reduces the number of calculations for testing

    if(previous_sample==NULL){
        last_single_motion_priors.clear();
        proposed_single_motion_priors.clear();

        for(int sample_id=0; sample_id<state->sampling.N/calc_thinning; sample_id++){
            vector<float> new_priors;
            proposed_single_motion_priors.push_back(new_priors);

            for(size_t i=0; i<proposed_sample->targets.size();i++){
                if(proposed_sample->is_active[i]){//larva is active in proposed sample
                    if(i+1>(*last_samples)[sample_id*calc_thinning].targets.size()){//larva does not exist in old sample
                        proposed_single_motion_priors[sample_id].push_back(1.0);
                    }
                    else{
                        if((*last_samples)[sample_id].is_active[i]){//larva is active in old sample
                            proposed_single_motion_priors[sample_id].push_back(singleMotionModel(&proposed_sample->targets[i],&(*last_samples)[sample_id*calc_thinning].targets[i]));
                        }
                        else{//larva is not active in old sample
                            proposed_single_motion_priors[sample_id].push_back(1.0);
                        }
                    }
                }
                else{//larva is inactive in proposed sample
                    proposed_single_motion_priors[sample_id].push_back(0.0);
                }
            }
        }
    }
    else{
        if(proposal_type == Consts::PROPOSAL_ADD){
            if(proposed_sample->targets.size()>previous_sample->targets.size()){//new target
                for(int sample_id=0; sample_id<state->sampling.N/calc_thinning; sample_id++){
                    proposed_single_motion_priors[sample_id].push_back(1.0);
                }
            }
            else{//reactivated target
                for(int sample_id=0; sample_id<state->sampling.N/calc_thinning; sample_id++){
                    proposed_single_motion_priors[sample_id][changed_target]=1.0;
                }
            }
        }
        else if(proposal_type == Consts::PROPOSAL_DEL){
            for(int sample_id=0; sample_id<state->sampling.N/calc_thinning; sample_id++){
                proposed_single_motion_priors[sample_id][changed_target]=0.0;
            }
        }
        else if(proposal_type == Consts::PROPOSAL_UPDATE){
            for(int sample_id=0; sample_id<state->sampling.N/calc_thinning; sample_id++){
                if(changed_target+1>(int)((*last_samples)[sample_id*calc_thinning].targets.size())){//larva does not exist in old sample
                    proposed_single_motion_priors[sample_id][changed_target]=1.0;
                }
                else{
                    if((*last_samples)[sample_id].is_active[changed_target]){//larva is active in old sample
                        proposed_single_motion_priors[sample_id][changed_target]= singleMotionModel(&proposed_sample->targets[changed_target],&(*last_samples)[sample_id*calc_thinning].targets[changed_target]);
                    }
                    else{//larva is not active in old sample
                        proposed_single_motion_priors[sample_id][changed_target]=1.0;
                    }
                }
            }
        }
    }


    int num_of_factors = proposed_single_motion_priors[0].size();
    proposed_motion_prior=0.0;
    for(size_t i=0; i<proposed_single_motion_priors.size(); i++){
        float current_prior = 1.0;
        for(size_t j=0; j<(size_t)num_of_factors; j++){
            float& val = proposed_single_motion_priors[i][j];
            if(val!=0.0){
                current_prior*=val;
            }
        }
        proposed_motion_prior+=current_prior;
    }

//    std::cerr<<"Prop/Last "<<proposed_motion_prior<<" / "<<last_motion_prior<<"\n";
    proposed_sample->probability*=(proposed_motion_prior*(double)(calc_thinning)/(double)(state->sampling.N)*32.0);

    if(previous_sample!=NULL){
        return proposed_motion_prior/last_motion_prior;
    }
    else{
        return 1.0;
    }
}

float Motion_Prior::singleMotionModel(target_data* X, target_data* Y){
    float result = 1.0;
    vector<circ>& md_x = X->model;
    vector<circ>& md_y = Y->model;

    //calculate probability of tail movement using LUT
    float angle = CV_PI/180.0*Basic_Calc::calcAngleClockwise(md_y[0].p,md_x[1].p,md_y[1].p);
    Point2f v = Basic_Calc::rotate(md_x[1].p-md_y[0].p,angle);
    v.x-=0.3*Basic_Calc::metricEucl(md_y[0].p,md_y[1].p);
    int i=abs(v.x/0.05);
    if(i<0){
        i*=-1;
    }
    int j=abs(v.y/0.05);
    if(j<0){
        j*=-1;
    }
    if(i<lut_size && j<lut_size){
        result*=lut.at<float>(j,i);
    }
    else{
        return 0.0;
    }

    //calculate probability of head movement using LUT
    angle = CV_PI/180.0*Basic_Calc::calcAngleClockwise(md_y[5].p,md_x[6].p,md_y[6].p);
    Point2f w = Basic_Calc::rotate(md_x[6].p-md_y[5].p,angle);
    if(abs(v.x)>abs(v.y)){//approx back or forward movement
        w.x-=v.x*Basic_Calc::metricEucl(md_y[5].p,md_y[6].p);
    }
    else{
        w.y*=0.5;
    }

    i=abs(w.x/0.05);
    if(i<0){
        i*=-1;
    }
    j=abs(w.y/0.05);
    if(j<0){
        j*=-1;
    }
    if(i<lut_size && j<lut_size){
        result*=lut.at<float>(j,i);
    }
    else{
        return 0.0;
    }

    //calculate overlap
    Rect bounding_box = Basic_Calc::surroundingRect(X->bounding_box,Y->bounding_box);
    Point2f offset = Point2f(bounding_box.x,bounding_box.y);

    Mat model_x = Mat::zeros(bounding_box.height,bounding_box.width,CV_8UC1);
    for(int j=0;j<7;j++){
        circle(model_x,X->model[j].p-offset,X->model[j].r,Scalar(255,255,255),-1);
    }
    Mat model_y = Mat::zeros(bounding_box.height,bounding_box.width,CV_8UC1);
    for(int j=0;j<7;j++){
        circle(model_y,Y->model[j].p-offset,Y->model[j].r,Scalar(255,255,255),-1);
    }

    //correct overlap
    double x_count = (double)(cv::sum(model_x)[0])/255.0;
    double y_count = (double)(cv::sum(model_y)[0])/255.0;
    Mat overlap_image = model_x&model_y;

    double overlap = (double)(cv::sum(overlap_image)[0])/255.0;

    result*=pow(min(overlap/x_count, overlap/y_count),5.0);

    return result;
}

float Motion_Prior::calculateInteractionPotentialRatio(sample* proposed_sample, int target_number ,int proposal_type, std::vector<sample> *current_samples){
    proposed_interaction_potentials = interaction_potentials;
    proposed_interactions = interactions;

    if(target_number==-1){//init interaction potentials and interactions for inital frame sample
        if(current_samples->back().targets.size()>0){
            for(size_t i=0;i<proposed_interaction_potentials.size();i++){
                for(size_t j=0;j<proposed_interaction_potentials.size();j++){
                    proposed_interaction_potentials[i][j]=1.0;
                }
            }
            for(size_t i=0;i<proposed_interactions.size();i++){
                for(size_t j=0;j<proposed_interactions.size();j++){
                    proposed_interactions[i][j]=0.0;
                }
            }
        }
        else{//init for new sequence
            proposed_interaction_potentials.clear();
            proposed_interactions.clear();
        }
        return 0.0;
    }

    target_data* changed_target = &(proposed_sample->targets[target_number]);

    if(proposal_type==Consts::PROPOSAL_ADD && proposed_sample->targets.size()!=current_samples->back().targets.size()){//increase matrix size for new target
        proposed_interaction_potentials.push_back({});
        for(int i=0;i<(int)interaction_potentials.size();i++){
            proposed_interaction_potentials.back().push_back(1.0);
        }
        for(int i=0;i<(int)interaction_potentials.size()+1;i++){
            proposed_interaction_potentials[i].push_back(1.0);
        }

        proposed_interactions.push_back({});
        for(int i=0;i<(int)interaction_potentials.size();i++){
            proposed_interactions.back().push_back(0.0);
        }
        for(int i=0;i<(int)interaction_potentials.size()+1;i++){
            proposed_interactions[i].push_back(0.0);
        }
    }

    //calculate new interaction potentials
    if(proposal_type==Consts::PROPOSAL_ADD || proposal_type==Consts::PROPOSAL_UPDATE){
        for(int i=0;i<(int)proposed_interaction_potentials.size();i++){
            target_data current_target = proposed_sample->targets[i];
            if(i!=target_number && proposed_sample->is_active[i]){
                if(RJMCMC_Target::boxIntersection(*changed_target,current_target)){
                    proposed_interactions[target_number][i]=1.0;
                    proposed_interactions[i][target_number]=1.0;

                    vector<float> overlap_ratios_1 = RJMCMC_Target::modelIntersectionRatios(*changed_target,current_target);
                    vector<float> overlap_ratios_2 = RJMCMC_Target::modelIntersectionRatios(current_target,*changed_target);

                    float potential = 1.0;
                    for(int j=0;j<7;j++){
                        if(overlap_ratios_1[j]>0.3 || overlap_ratios_2[j]>0.3){
                            potential*=0.0;
                            break;
                        }
                        //                        else{
                        //                            //potential *= exp(-(overlap_ratios_1[j]+overlap_ratios_2[j]));
                        //                            potential *= (0.53333*(1-(100*(overlap_ratios_1[j]-0.15))/(1.0+abs(100*(overlap_ratios_1[j]-0.15))))-0.03333);
                        //                            potential *= (0.53333*(1-(100*(overlap_ratios_2[j]-0.15))/(1.0+abs(100*(overlap_ratios_2[j]-0.15))))-0.03333);
                        //                        }
                    }

                    proposed_interaction_potentials[target_number][i]=potential;
                    proposed_interaction_potentials[i][target_number]=potential;
                }
            }
        }
    }
    else{//reset interaction potentials for deleted target
        for(int i=0;i<(int)proposed_interaction_potentials.size();i++){
            proposed_interaction_potentials[target_number][i]=1.0;
            proposed_interaction_potentials[i][target_number]=1.0;
            proposed_interactions[target_number][i]=0.0;
            proposed_interactions[i][target_number]=0.0;
        }
    }

    //calculate potential ratio
    float ratio = 1.0;

    switch(proposal_type){
    case Consts::PROPOSAL_ADD:{
        for(int i=0;i<(int)proposed_interaction_potentials.size();i++){
            ratio*=proposed_interaction_potentials[target_number][i];
        }
        break;
    }
    case Consts::PROPOSAL_DEL:{
        for(int i=0;i<(int)interaction_potentials.size();i++){
            ratio/=interaction_potentials[target_number][i];
        }
        break;
    }
    case Consts::PROPOSAL_UPDATE:{
        for(int i=0;i<(int)proposed_interaction_potentials.size();i++){
            ratio*=proposed_interaction_potentials[target_number][i];
        }
        for(int i=0;i<(int)interaction_potentials.size();i++){
            ratio/=interaction_potentials[target_number][i];
        }
    }
    }

//    std::cerr<<"Inter:"<<proposal_type<<" ratio "<<ratio<<"\n";

    return ratio;
}

void Motion_Prior::acceptProposal(){
    interaction_potentials=proposed_interaction_potentials;
    interactions = proposed_interactions;

    last_motion_prior=proposed_motion_prior;
    last_single_motion_priors=proposed_single_motion_priors;
}

void Motion_Prior::calculateLUT(){
    float length = state->sampling.avg_spine_length;
    float sampling_size = 0.05;
    float allowed_distance = length/12.0;
    lut_size = (int)(allowed_distance/sampling_size);
    lut = cv::Mat::zeros(lut_size,lut_size,CV_32FC1);

    float sigma_x = length/12.0;
    float sigma_y = length/16.0;
    float x_size = length/18.0;

    float norm = 1.0/exp(-0.5*x_size*x_size/sigma_x);

    for(int i=0;i<lut_size;i++){
        for(int j=0;j<lut_size;j++){
            cv::Point2f p = cv::Point2f((float)j*0.05,(float)i*0.05);

            float f_of_p = norm*exp(-0.5*(p.x*p.x/sigma_x+p.y*p.y/sigma_y));

            if(f_of_p>1.0){
                f_of_p=1.0;
            }

            lut.at<float>(i,j) = f_of_p;
        }
    }
}


