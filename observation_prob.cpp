#include "observation_prob.h"
#include "consts.h"

Observation_Prob::Observation_Prob(System_State *state) :
    state(state)
{

}

double Observation_Prob::observationProbability(sample* old_sample, sample* proposed_sample, int changed_target, int proposal_type, observation_data* observation){
    if(old_sample==NULL){//proposed_sample == initial sample
        unmatched_models.clear();
        corresponding_models.clear();
        overlapping_noncorresponding_models.clear();
        single_probabilities.clear();
        matched_contour.clear();

        for(size_t j=0; j<observation->detected_contours.size(); j++){
            corresponding_models.push_back({});
            overlapping_noncorresponding_models.push_back({});
        }

        //for all larva find all contours that overlap with it and calculate the overlap size
        vector<vector<pair<int,float>>> overlaps;

        for(size_t i=0; i<proposed_sample->active.size(); i++){
            target_data& tgt = proposed_sample->targets[proposed_sample->active[i]];
            overlaps.push_back({});
            for(size_t j=0; j<observation->detected_contours.size(); j++){
                vector<Point>& contour = observation->detected_contours[j];
                Rect& bounding_box = observation->detected_contour_boxes[j];
                if(Basic_Calc::rectIntersection(bounding_box, tgt.bounding_box)){//check if bounding boxes intersect
                    float intersection = RJMCMC_Target::contourModelIntersection(tgt,contour,bounding_box);
                    if(intersection>0.0){
                        overlaps.back().push_back(pair<int,float>(j,intersection));
                    }
                }
            }
        }

        //only consider contour with biggest intersection for each larva
        float current_best;
        int best_idx;
        for(size_t i=0; i<overlaps.size(); i++){
            vector<pair<int,float>>& overlaps_for_larva = overlaps[i];
            current_best=0.0;
            best_idx=-1;
            for(pair<int,float>& overlap : overlaps_for_larva){
                if(overlap.second>current_best){
                    best_idx = overlap.first;
                    current_best=overlap.second;
                }
            }
            if(best_idx==-1 || current_best<0.3){
                unmatched_models.push_back(i);
                matched_contour.push_back(-1);
            }
            else{
                corresponding_models[best_idx].push_back(pair<int,float>(proposed_sample->active[i],current_best));
                matched_contour.push_back(best_idx);
            }
            for(pair<int,float>& overlap : overlaps_for_larva){
                if(overlap.first!=best_idx){
                    overlapping_noncorresponding_models[overlap.first].push_back(pair<int,float>(proposed_sample->active[i],overlap.second));
                }
            }
        }

        //calculate probabilities
        proposed_unmatched_models=unmatched_models;
        proposed_matched_contour=matched_contour;
        proposed_corresponding_models=corresponding_models;
        proposed_overlapping_noncorresponding_models=overlapping_noncorresponding_models;

        prob_factors probs;
        for(size_t i=0; i<corresponding_models.size(); i++){
            calculateOverlapRatios(i,observation,proposed_sample,&probs);
            single_probabilities.push_back(probs);
        }

        proposed_single_probabilities=single_probabilities;

    }
    else{//update corresponding larvae etc.
        proposed_unmatched_models=unmatched_models;
        proposed_matched_contour=matched_contour;
        proposed_corresponding_models=corresponding_models;
        proposed_overlapping_noncorresponding_models=overlapping_noncorresponding_models;
        proposed_single_probabilities=single_probabilities;

        if(proposal_type==Consts::PROPOSAL_ADD){
            //find contours that intersect with added larvae
            vector<pair<int,float>> overlaps;
            target_data& tgt = proposed_sample->targets[changed_target];
            for(size_t j=0; j<observation->detected_contours.size(); j++){
                vector<Point>& contour = observation->detected_contours[j];
                Rect& bounding_box = observation->detected_contour_boxes[j];
                if(Basic_Calc::rectIntersection(bounding_box, tgt.bounding_box)){//check if bounding boxes intersect
                    overlaps.push_back(pair<int,float>(j,RJMCMC_Target::contourModelIntersection(tgt,contour,bounding_box)));
                }
            }

            //find contour that overlaps most with new larva
            float current_best=0.0;
            int best_idx=-1;
            for(pair<int,float>& overlap : overlaps){
                if(overlap.second>current_best){
                    best_idx = overlap.first;
                    current_best = overlap.second;
                }
            }

            bool is_new = (old_sample->targets.size()<proposed_sample->targets.size());

            //save correspondence
            if(best_idx==-1){
                proposed_unmatched_models.push_back(changed_target);
                if(is_new){
                    proposed_matched_contour.push_back(-1);
                }
                else{
                    proposed_matched_contour[changed_target]=-1;
                }
            }
            else{
                proposed_corresponding_models[best_idx].push_back(pair<int,float>(changed_target,current_best));
                if(is_new){
                    proposed_matched_contour.push_back(best_idx);
                }
                else{
                    proposed_matched_contour[changed_target]=best_idx;
                }
            }
            for(pair<int,float>& overlap : overlaps){
                if(overlap.first!=best_idx){
                    proposed_overlapping_noncorresponding_models[overlap.first].push_back(pair<int,float>(changed_target,overlap.second));
                }
            }

            //calculate updated probabilities
            prob_factors probs;
            for(pair<int,float>& overlap : overlaps){
                calculateOverlapRatios(overlap.first,observation,proposed_sample,&probs);
                proposed_single_probabilities[overlap.first]=probs;
            }
        }

        else if(proposal_type==Consts::PROPOSAL_DEL){
            //delete the deleted larva from unmatched models
            int idx = std::distance(unmatched_models.begin(),find(unmatched_models.begin(),unmatched_models.end(),changed_target));
            if(idx != (int)unmatched_models.size()){
                proposed_unmatched_models.erase(proposed_unmatched_models.begin()+idx);
            }
            proposed_matched_contour[changed_target]=-1;

            //find contours that intersected with deleted larvae
            vector<int> contour_ids;
            for(size_t i=0; i<corresponding_models.size(); i++){
                vector<pair<int,float>>& correspondences = corresponding_models[i];
                for(size_t j=0; j<correspondences.size(); j++){
                    pair<int,float>& cor = correspondences[j];
                    if(cor.first==changed_target){
                        contour_ids.push_back(i);
                        proposed_corresponding_models[i].erase(proposed_corresponding_models[i].begin()+j);
                    }
                }

                vector<pair<int,float>>& non_correspondences = overlapping_noncorresponding_models[i];
                for(size_t j=0; j<non_correspondences.size(); j++){
                    pair<int,float>& cor = non_correspondences[j];
                    if(cor.first==changed_target){
                        contour_ids.push_back(i);
                        proposed_overlapping_noncorresponding_models[i].erase(proposed_overlapping_noncorresponding_models[i].begin()+j);
                    }
                }
            }

            //update the probabilities for these contours
            prob_factors probs;
            for(int& idx : contour_ids){
                calculateOverlapRatios(idx,observation,proposed_sample,&probs);
                proposed_single_probabilities[idx]=probs;
            }

        }

        else if(proposal_type==Consts::PROPOSAL_UPDATE){
            //find contours that intersected with updated larva and erase the correspondences
            vector<int> contour_ids;
            for(size_t i=0; i<corresponding_models.size(); i++){
                vector<pair<int,float>>& correspondences = corresponding_models[i];
                for(size_t j=0; j<correspondences.size(); j++){
                    pair<int,float>& cor = correspondences[j];
                    if(cor.first==changed_target){
                        contour_ids.push_back(i);
                        proposed_corresponding_models[i].erase(proposed_corresponding_models[i].begin()+j);
                    }
                }

                vector<pair<int,float>>& non_correspondences = overlapping_noncorresponding_models[i];
                for(size_t j=0; j<non_correspondences.size(); j++){
                    pair<int,float>& cor = non_correspondences[j];
                    if(cor.first==changed_target){
                        contour_ids.push_back(i);
                        proposed_overlapping_noncorresponding_models[i].erase(proposed_overlapping_noncorresponding_models[i].begin()+j);
                    }
                }
            }

            //calculate new correspondences
            vector<pair<int,float>> overlaps;
            target_data& tgt = proposed_sample->targets[changed_target];
            for(size_t j=0; j<observation->detected_contours.size(); j++){
                vector<Point>& contour = observation->detected_contours[j];
                Rect& bounding_box = observation->detected_contour_boxes[j];
                if(Basic_Calc::rectIntersection(bounding_box, tgt.bounding_box)){//check if bounding boxes intersect
                    overlaps.push_back(pair<int,float>(j,RJMCMC_Target::contourModelIntersection(tgt,contour,bounding_box)));
                }
            }

            //find contour that overlaps most with updated larva
            float current_best=0.0;
            int best_idx=-1;
            for(pair<int,float>& overlap : overlaps){
                if(overlap.second>current_best){
                    best_idx = overlap.first;
                }
            }

            //save correspondence
            if(best_idx==-1){
                proposed_unmatched_models.push_back(changed_target);
                proposed_matched_contour[changed_target]=-1;
            }
            else{
                proposed_corresponding_models[best_idx].push_back(pair<int,float>(changed_target,current_best));
                proposed_matched_contour[changed_target]=best_idx;
            }
            for(pair<int,float>& overlap : overlaps){
                if(overlap.first!=best_idx){
                    proposed_overlapping_noncorresponding_models[overlap.first].push_back(pair<int,float>(changed_target,overlap.second));
                }
            }

            //calculate updated probabilities for all affected contours
            prob_factors probs;
            for(pair<int,float>& overlap : overlaps){
                for(size_t k=0; k<contour_ids.size(); k++){
                    if(overlap.first==contour_ids[k]){
                        contour_ids.erase(contour_ids.begin()+k);
                        break;
                    }
                }
                calculateOverlapRatios(overlap.first,observation,proposed_sample,&probs);
                proposed_single_probabilities[overlap.first]=probs;
            }

            for(int& idx : contour_ids){
                calculateOverlapRatios(idx,observation,proposed_sample,&probs);
                proposed_single_probabilities[idx]=probs;
            }
        }
    }

/*
    ////////////////////////////////////////test///////////////////////////////////////

    vector<int> proposed_corres;
    for(int i=0;i<proposed_corresponding_models.size();i++){
        for(int j=0;j<proposed_corresponding_models[i].size(); j++){
            proposed_corres.push_back(proposed_corresponding_models[i][j].first);
        }
    }

    for(int i=0;i<proposed_corres.size();i++){
        bool exists = false;
        for(int j=0; j<proposed_sample->active.size();j++){
            if(proposed_corres[i]==proposed_sample->active[j]){
                exists=true;
                break;
            }
        }
        if(!exists){
            int stop=42;
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////
    */


    //calculate the result
    double result = 1.0;
    for(prob_factors& probs : proposed_single_probabilities){
        result*=10*exp(-20.0*probs.a_1);
        result*=10*exp(-5.0*probs.a_2);
        result*=10*exp(-3.0*probs.a_3.back());
    }

    //handle unmatched models (very unlikely to happen)
    for(size_t i=0; i<proposed_unmatched_models.size(); i++){
        result*=0.0001;
    }
    if(old_sample==NULL){
        last_prior = result;
    }
    proposed_prior = result;
    proposed_sample->probability*=proposed_prior;

    //In the case of larva deletion or addition only check, if more contourarea is occluded - this is necessary to accept bad initial models
    if(proposal_type==Consts::PROPOSAL_DEL || proposal_type==Consts::PROPOSAL_ADD){
        result=1.0;
        for(prob_factors& probs : proposed_single_probabilities){
            result*=10*exp(-20.0*probs.a_1);
        }
        for(prob_factors& probs : single_probabilities){
            result/=10*exp(-20.0*probs.a_1);
        }
    }
    else{
        result/=last_prior;
    }

    return result;
}

void Observation_Prob::calculateOverlapRatios(int contour_idx, observation_data *observation, sample *proposed_sample, prob_factors *probs){
    //calculate combined bounding box
    Rect bounding_box = observation->detected_contour_boxes[contour_idx];
    for(pair<int,float>& correspondence : proposed_corresponding_models[contour_idx]){
        bounding_box = Basic_Calc::surroundingRect(bounding_box, proposed_sample->targets[correspondence.first].bounding_box);
    }
    for(pair<int,float>& correspondence : proposed_overlapping_noncorresponding_models[contour_idx]){
        bounding_box = Basic_Calc::surroundingRect(bounding_box, proposed_sample->targets[correspondence.first].bounding_box);
    }
    Point offset = Point(bounding_box.x,bounding_box.y);
    Point2f offset_f = Point2f(bounding_box.x,bounding_box.y);

    //draw contour
    Mat single_observation = Mat::zeros(bounding_box.height,bounding_box.width,CV_8UC1);
    vector<Point> contour = observation->detected_contours[contour_idx];
    for(size_t i=0;i<contour.size();i++){
        contour[i]-=offset;
    }
    vector<vector<Point>> contours;
    contours.push_back(contour);
    cv::drawContours(single_observation,contours,0,Scalar(255,255,255),-1);

    //draw corresponding larvae
    probs->a_3.clear();
    double a_3=0.0;
    Mat correspondences = Mat::zeros(bounding_box.height,bounding_box.width,CV_8UC1);
    for(pair<int,float>& correspondence : proposed_corresponding_models[contour_idx]){
        //correspondeces.push_back(Mat::zeros(bounding_box.height,bounding_box.width,CV_8UC1));
        vector<circ>& current_model = proposed_sample->targets[correspondence.first].model;
        for(int j=0;j<7;j++){
            circle(correspondences,(current_model[j].p-offset_f),current_model[j].r,Scalar(255,255,255),-1);
            //calculate a_3
            cv::Mat circle_img = cv::Mat::zeros(correspondences.rows,correspondences.cols,correspondences.type());
            circle(circle_img,(current_model[j].p-offset_f),current_model[j].r,Scalar(255,255,255),-1);
            double temp = (double)(cv::sum(circle_img)[0]);
            circle_img-=single_observation;
            temp = (double)(cv::sum(circle_img)[0])/temp;
            if(fabs(1.0-temp)<0.05){
                temp*=20.0;
            }
            else{
                temp/=fabs(1.0-temp);
            }
            a_3+=temp;
        }
    }
    probs->a_3.push_back(a_3);

    //draw non-corresponding larvae
    Mat non_correspondences = Mat::zeros(bounding_box.height,bounding_box.width,CV_8UC1);
    for(pair<int,float>& correspondence : proposed_overlapping_noncorresponding_models[contour_idx]){
        vector<circ>& current_model = proposed_sample->targets[correspondence.first].model;
        for(int j=0;j<7;j++){
            circle(non_correspondences,(current_model[j].p-offset_f),current_model[j].r,Scalar(255,255,255),-1);
        }
    }

    //calculate ratios

    //correct overlap
    Mat overlap_1 = single_observation-correspondences;
    probs->a_1 = (double)(cv::sum(overlap_1)[0])/(state->sampling.avg_larva_area*255.0);

    //wrong overlap
    non_correspondences &= single_observation;
    probs->a_2 = (double)(cv::sum(non_correspondences)[0])/(state->sampling.avg_larva_area*255.0);

}








