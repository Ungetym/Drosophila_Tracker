#include "rjmcmc_tracker.h"
#include "consts.h"
#include "fstream"
#include <QElapsedTimer>

using namespace cv;
using namespace std;

RJMCMC_Tracker::RJMCMC_Tracker(System_State* state) :
    state(state),
    motion_prior(state),
    motion_proposer(state),
    measurement_model(state)
{
}

void RJMCMC_Tracker::init(){
    frame_counter=0;
    this->current_samples.clear();
    this->last_samples.clear();
    state->heads.headpoints.clear();
}

sample RJMCMC_Tracker::track(cv::Mat *image_binary, vector<vector<Point>> contours){
    //set observation
    Z_t.binary_image=image_binary->clone();
    Z_t.setContours(contours);
    
    //sample
    if(rjmcmcSampling()){
        frame_counter++;

        //calculate and return result sample
        calculateBestSample();

        return state->sampling.best_sample;
    }
    else{
        sample null_sample;
        null_sample.clear();
        return null_sample;
    }
}

void RJMCMC_Tracker::estimateSample(sample* X){
    state->eval.timer.restart();

    vector<vector<Point>> unmatched_contours;
    
    //associate sample X with detected targets, i.e. for every detected target find the nearest larva saved in X
    estimated_state.clear();
    if(X!=NULL){
        //copy old targets to estimated state and set all inactive
        estimated_state.targets=X->targets;
        for(size_t i=0;i<estimated_state.targets.size();i++){
            estimated_state.is_active.push_back(false);
        }

        //check if targets were active in last frame
        vector<bool> was_active;
        for(int i=0; i<X->targets.size(); i++){
            for(sample& smp : last_samples){
                if(smp.is_active.size()>i){
                    if(smp.is_active[i]){
                        was_active.push_back(true);
                        break;
                    }
                }
            }
            if(was_active.size()!=i+1){
                was_active.push_back(false);
            }
        }
        
        //calculate intersection bounding boxes
        vector<vector<int>> box_collisions;
        for(size_t i=0;i<Z_t.detected_contour_boxes.size();i++){
            vector<int> current_collisions;
            for(size_t j=0;j<X->targets.size();j++){
                //check if detected contour and old contour bounding boxes overlap
                if(Basic_Calc::rectIntersection(Z_t.detected_contour_boxes[i],X->targets[j].bounding_box)){
                    if(was_active[j]){
                        current_collisions.push_back(j);
                    }
                }
            }
            box_collisions.push_back(current_collisions);
        }
        
        //check intersection of model and contour for detected box collisions
        vector<vector<int>> contour_collisions;
        for(size_t i=0;i<Z_t.detected_contour_boxes.size();i++){
            vector<int>& box_colls = box_collisions[i];
            vector<int> current_collisions;
            for(size_t j=0;j<box_colls.size();j++){
                if(RJMCMC_Target::contourModelIntersection(X->targets[box_colls[j]],Z_t.detected_contours[i],Z_t.detected_contour_boxes[i])>0.2){
                    current_collisions.push_back(box_colls[j]);
                }
            }
            contour_collisions.push_back(current_collisions);
        }
        
        //save contours without a match and set matched targets to active
        for(size_t i=0;i<contour_collisions.size();i++){
            vector<int>& current_collisions = contour_collisions[i];
            if(current_collisions.size()==0){
                unmatched_contours.push_back(Z_t.detected_contours[i]);
            }
            else{
                for(size_t j=0;j<current_collisions.size();j++){
                    estimated_state.is_active[current_collisions[j]]=true;
                }
            }
        }
        //set active and inactive lists for estimated state
        for(size_t i=0;i<estimated_state.targets.size();i++){
            if(estimated_state.is_active[i]){
                estimated_state.active.push_back(i);
            }
            else{
                estimated_state.inactive.push_back(i);
            }
        }
    }
    else{
        unmatched_contours=Z_t.detected_contours;
    }
    
    //compute model for every detected target not associated to any larva in X
    for(size_t i = 0; i < unmatched_contours.size(); i++){
        std::vector<circ> current_model = Larva_Model::calculateModel(&(unmatched_contours[i]));
        //try reading headfile
        if(this->frame_counter==0){
            readHeads();
        }

        if(this->state->heads.headpoints.size()>=estimated_state.targets.size()+unmatched_contours.size()-i){//if headpoints are already available, check if model has to be switched
            if(Basic_Calc::metricEucl(current_model[0].p,state->heads.headpoints[estimated_state.targets.size()])<Basic_Calc::metricEucl(current_model[6].p,state->heads.headpoints[estimated_state.targets.size()])){
                std::reverse(current_model.begin(), current_model.end());
            }
        }
        else{//ask user to click on heads and save the result to headpoints
            state->eval.time_needed+=state->eval.timer.elapsed();

            namedWindow("Click on head and press a Key", CV_WINDOW_NORMAL);
            resizeWindow("Click on head and press a Key", 1280, 960);
            setMouseCallback("Click on head and press a Key", mouseAction, &current_model);
            Mat cpy = state->sampling.current_image.clone();
            
            for(int j = 0; j < 7; j++){
                circle(cpy,current_model[j].p,current_model[j].r,state->io.colors[42]);
            }
            imshow("Click on head and press a Key",cpy);
            waitKey(0);
            destroyWindow("Click on head and press a Key");
            this->writeHead(current_model[6].p);
            state->heads.headpoints.push_back(current_model[6].p);

            state->eval.timer.restart();
        }
        
        //save new larva
        target_data new_target = RJMCMC_Target::targetFromModel(current_model);
        estimated_state.targets.push_back(new_target);
        estimated_state.active.push_back(i);
        estimated_state.is_active.push_back(true);
    }
    state->eval.time_needed+=state->eval.timer.elapsed();
}

bool RJMCMC_Tracker::rjmcmcSampling(){
    state->eval.timer.restart();

    float number_of_accepted_samples = 0.0f;
    
    sample init_sample;
    
    if(frame_counter==0){//initialization in first frame
        estimateSample(NULL);

        if(estimated_state.targets.size()==0){
            return false;
        }

        //calculates average spine size of detected larvae in estimated state
        averageSpineAndArea();
        //init motion LUT
        motion_prior.calculateLUT();
        
        //save N empty samples -> the sampling needs "previous" samples
        for(int i=0;i<state->sampling.N;i++){
            current_samples.push_back(init_sample);
        }
        state->sampling.best_sample=init_sample;
    }
    
    //save last results
    last_samples=current_samples;
    last_best_sample=state->sampling.best_sample;
    
    current_samples.clear();
    
    srand(time(0));
    
    init_sample=last_best_sample;
    
    for(size_t i=0;i<init_sample.targets.size();i++){
        init_sample.targets[i]=RJMCMC_Target::targetFromModel(init_sample.targets[i].model);
    }
    
    //add later detected targets as inactive targets
    sample last_old_sample = last_samples.back();
    for(size_t i=init_sample.targets.size();i<last_old_sample.targets.size();i++){
        init_sample.inactive.push_back(i);
        init_sample.is_active.push_back(false);
        init_sample.targets.push_back(last_old_sample.targets[i]);
    }
    state->eval.time_needed+=state->eval.timer.elapsed();
    
    //calculate estimated state
    estimateSample(&init_sample);
    
    state->eval.timer.restart();

    //init observation probabilities
    measurement_model.observationProbability(NULL, &init_sample, -1,-1, &Z_t);
    
    current_samples.push_back(init_sample);
    
    //compare initial sample to estimated state in order to determine which targets can be added
    targets_to_add.clear();
    targets_to_delete.clear();
    
    if(estimated_state.targets.size()!=0){
        for(size_t i=0; i<estimated_state.targets.size(); i++){
            if(i<init_sample.targets.size()){
                if(init_sample.is_active[i]){
                    targets_to_delete.push_back(i);
                }
                else if(!init_sample.is_active[i]&&estimated_state.is_active[i]){
                    targets_to_add.push_back(i);
                }
            }
            else{
                targets_to_add.push_back(i);
            }
        }
    }
    
    motion_prior.calculateMotionPrior(&init_sample,NULL,-1,-1,&last_samples);
    motion_prior.calculateInteractionPotentialRatio(&init_sample,-1,-1,&last_samples);
    motion_prior.acceptProposal();
    
    //start sampling algorithm
    for(int i=0;i<state->sampling.B+state->sampling.M*state->sampling.N-1;i++){
        
        sample last_sample = current_samples.back();
        sample proposed_sample = last_sample;
        
        proposed_targets_to_add=targets_to_add;
        proposed_targets_to_delete=targets_to_delete;
        
        //set proposal probabilities
        bool add = (targets_to_add.size()>0);
        bool del = (targets_to_delete.size()>0);
        bool upd = (proposed_sample.is_active.size()>0);
        p_A=0;
        p_D=0;
        p_U=0;
        if(upd){
            if(add){p_A=15;}
            if(del){p_D=15;}
            p_U=100-p_A-p_D;
        }
        else{
            p_A=100;
        }
        
        
        //choose proposal type
        int number = rand()%100;
        int chosen_type = -1;
        int idx_of_changed_target = -1;
        proposed_estimated_state=estimated_state;
        
        if((number-=p_A)<0){//add target
            chosen_type = Consts::PROPOSAL_ADD;
            int rand_idx = rand()%targets_to_add.size();
            int target_idx = targets_to_add[rand_idx];
            
            if(target_idx<(int)proposed_sample.targets.size()){//i.e. the target is inactive in current sample, but active in estimated sample
                proposed_sample.is_active[target_idx]=true;
                proposed_sample.active.push_back(target_idx);
                proposed_sample.inactive.erase(find(proposed_sample.inactive.begin(), proposed_sample.inactive.end(), target_idx));
                //update add and delete lists
                proposed_targets_to_add.erase(proposed_targets_to_add.begin()+rand_idx);
                proposed_targets_to_delete.push_back(target_idx);
                idx_of_changed_target = target_idx;
            }
            else{//add a new target
                proposed_sample.targets.push_back(estimated_state.targets[target_idx]);
                int idx_in_proposed = proposed_sample.targets.size()-1;
                proposed_sample.is_active.push_back(true);
                proposed_sample.active.push_back(idx_in_proposed);
                
                //reorder estimated state
                std::iter_swap(proposed_estimated_state.targets.begin()+idx_in_proposed, proposed_estimated_state.targets.begin()+target_idx);
                std::iter_swap(proposed_estimated_state.is_active.begin()+idx_in_proposed, proposed_estimated_state.is_active.begin()+target_idx);
                
                //update add list
                proposed_targets_to_add.erase(proposed_targets_to_add.begin()+rand_idx);
                
                for(size_t j=0; j<proposed_targets_to_add.size(); j++){
                    if(proposed_targets_to_add[j]==idx_in_proposed){
                        proposed_targets_to_add[j]=target_idx;
                        break;
                    }
                }
                
                //update delete list
                proposed_targets_to_delete.push_back(idx_in_proposed);
                idx_of_changed_target = idx_in_proposed;
            }
            
        }
        
        else if((number-=p_D)<0){//delete target: set target to inactive
            chosen_type = Consts::PROPOSAL_DEL;
            int rand_idx = rand()%(targets_to_delete.size());
            idx_of_changed_target=targets_to_delete[rand_idx];
            
            proposed_sample.changeTargetStatus(idx_of_changed_target);
            
            proposed_targets_to_delete.erase(proposed_targets_to_delete.begin()+rand_idx);
            proposed_targets_to_add.push_back(idx_of_changed_target);
            
        }
        else if((number-=p_U)<0){//update target
            chosen_type = Consts::PROPOSAL_UPDATE;
            idx_of_changed_target = rand()%(proposed_sample.active.size());
            idx_of_changed_target = proposed_sample.active[idx_of_changed_target];
            motion_proposer.update(&(proposed_sample.targets[idx_of_changed_target]));
        }
        
        
        //calculate acceptance rate
        proposed_sample.probability = 1.0;
        float a = acceptanceRate(&proposed_sample ,chosen_type,idx_of_changed_target);
        
        //        //test output
        //        Mat test_1;
        //        this->drawTargets(&last_sample,&test_1,2.0);
        //        cv::imshow("Last",test_1);
        //        Mat test;
        //        this->drawTargets(&proposed_sample,&test,2.0);
        //        cv::imshow("Proposed",test);

        //        cv::waitKey(0);
        

        //accept proposal with probability a, otherwise reject and re-add last sample
        if(a>=1 || (a>state->sampling.a_threshold && (float)((rand()%100)+1.0)/100.0<a)){
            number_of_accepted_samples++;
            current_samples.push_back(proposed_sample);
            targets_to_add=proposed_targets_to_add;
            targets_to_delete=proposed_targets_to_delete;
            estimated_state=proposed_estimated_state;
            motion_prior.acceptProposal();
            measurement_model.acceptProposal();
        }
        else{
            current_samples.push_back(last_sample);
        }
    }
    
    //remove burn-in samples
    current_samples.erase(current_samples.begin(),current_samples.begin()+state->sampling.B);
    //save rate of accepted proposals
    state->sampling.current_rate_of_accepted = number_of_accepted_samples/(float)state->sampling.N;

    state->eval.time_needed+=state->eval.timer.elapsed();

    return true;
}

float RJMCMC_Tracker::acceptanceRate(sample* proposed_sample,int proposal_type, int changed_target){
    
    float observation_probability = 1.0;
    float prior_ratio = 1.0;
    float proposal_ratio = 1.0;
    
    switch(proposal_type){
    case Consts::PROPOSAL_ADD :{
        //calculate |k_D\k_t| and |k_D\cap k'_t|
        float k_D_t = (float)targets_to_add.size();
        float k_t_cap_D = (float)proposed_targets_to_delete.size();
        proposal_ratio=k_D_t/k_t_cap_D;
        observation_probability=measurement_model.observationProbability(&(current_samples.back()), proposed_sample, changed_target,proposal_type, &Z_t);
        motion_prior.calculateMotionPrior(proposed_sample,&current_samples.back(),changed_target,proposal_type,&last_samples);
        motion_prior.calculateInteractionPotentialRatio(proposed_sample,changed_target,proposal_type, &current_samples);
        break;
    }
    case Consts::PROPOSAL_DEL:{
        //calculate |k_D\cap k_t| and |k_D\ k'_t|
        float k_D_cap_t = (float)targets_to_delete.size();
        float k_D_wo_t = (float)proposed_targets_to_add.size();
        proposal_ratio=k_D_cap_t/k_D_wo_t;
        observation_probability=measurement_model.observationProbability(&(current_samples.back()), proposed_sample, changed_target,proposal_type, &Z_t);
        motion_prior.calculateMotionPrior(proposed_sample,&current_samples.back(),changed_target,proposal_type,&last_samples);
        motion_prior.calculateInteractionPotentialRatio(proposed_sample,changed_target,proposal_type, &current_samples);
        break;
    }
    case Consts::PROPOSAL_UPDATE:{
        observation_probability=measurement_model.observationProbability(&(current_samples.back()), proposed_sample, changed_target, proposal_type, &Z_t);
        prior_ratio *= motion_prior.calculateMotionPrior(proposed_sample,&current_samples.back(),changed_target,proposal_type,&last_samples);
        prior_ratio *= motion_prior.calculateInteractionPotentialRatio(proposed_sample,changed_target,proposal_type, &current_samples);
    }
    }
    
    //        std::cerr<<"Obs:"<<observation_probability<<"\n";
    //        std::cerr<<"Prior:"<<prior_ratio<<"\n";
    
    float result =  observation_probability*prior_ratio*proposal_ratio;
    
    return result;
}


//////////////////////////////////////////////////  HELP FUNCTIONS  //////////////////////////////////////////////////////

void RJMCMC_Tracker::averageSpineAndArea(){
    float avg_spine=0.0;
    float avg_area=0.0;
    
    float num_of_old_targets = (float)estimated_state.targets.size();
    
    for(int i=0;i<num_of_old_targets;i++){
        target_data& tgt = estimated_state.targets[i];
        for(int j=0;j<6;j++){
            avg_spine+=tgt.distances[j];
        }
        //draw target and count pixels
        cv::Mat target_bin = cv::Mat::zeros(tgt.bounding_box.height+1, tgt.bounding_box.width+1,CV_8UC1);
        Point2f offset = Point(tgt.bounding_box.x, tgt.bounding_box.y);
        for(int j=0;j<6;j++){
            cv::circle(target_bin,tgt.model[j].p-offset,tgt.model[j].r,Scalar(255.0,255.0,255.0), CV_FILLED);
        }
        avg_area+=(double)(cv::sum(target_bin)[0])/255.0;
    }
    
    avg_spine /= num_of_old_targets;
    avg_area /= num_of_old_targets;
    
    state->sampling.avg_spine_length = avg_spine;
    state->sampling.avg_larva_area = avg_area;
    state->sampling.min_dist = avg_spine/8.0;
    state->sampling.max_dist = avg_spine/3.0;
}

void RJMCMC_Tracker::mouseAction(int evt, int x, int y, int, void* userdata){
    if (evt == EVENT_LBUTTONDOWN){//add new circle
        vector<circ>* model = static_cast<vector<circ>*>(userdata);
        Point2f clicked = Point2f(x,y);
        if(Basic_Calc::metricEucl((*model)[0].p,clicked)<Basic_Calc::metricEucl((*model)[6].p,clicked)){
            //invert model
            vector<circ> temp;
            for(int i=0;i<7;i++){
                temp.push_back((*model)[6-i]);
            }
            (*model)=temp;
        }
    }
}

void RJMCMC_Tracker::writeHead(Point2f headpoint){
    ofstream headfile;
    headfile.open(state->heads.headfile_path, std::ofstream::out | std::ofstream::app);
    if(headfile.is_open()){
        headfile<<" "<<headpoint.x<<" "<<headpoint.y<<"\n";
    }
}

void RJMCMC_Tracker::readHeads(){
    state->heads.headpoints.clear();
    //read the headfile and save headpoints
    ifstream headfile;
    headfile.open(state->heads.headfile_path);
    if(headfile.is_open()){
        vector<float> point_coords;
        float coord;
        while(headfile>>coord){
            point_coords.push_back(coord);
        }
        headfile.close();
        for(int i=0;i<(int)point_coords.size()/2;i++){
            state->heads.headpoints.push_back(Point2f(point_coords[2*i],point_coords[2*i+1]));
        }
    }
}

void RJMCMC_Tracker::calculateBestSample(){
    /*    vector<float> votes(state->sampling.N, 0.0);
    vector<float> score(state->sampling.N, 0.0);
    int best_idx = 0;
    
    auto scalDist = [](cv::Point2f p, cv::Point2f q){return 0.00001*(p.x*p.x+p.y*p.y);};
    
    for(size_t i=0; i<state->sampling.N; i++){
        sample& smp = current_samples[i];
        bool in_same_cluster = true;
        float dist = 0.0;
        for(size_t j=i+1; j<state->sampling.N; j++){
            sample& smp_compare = current_samples[j];
            in_same_cluster = true;
            if(smp.active.size()==smp_compare.active.size()){
                for(size_t k=0; k<smp.is_active.size(); k++){
                    if((!smp.is_active[k]&&smp_compare.is_active[k]) || (smp.is_active[k]&&!smp_compare.is_active[k])){
                        in_same_cluster = false;
                        break;
                    }
                }
            }
            else{
                in_same_cluster=false;
            }
            if(in_same_cluster){
                votes[i]++;
                votes[j]++;
                for(size_t k=0; k<smp.active.size(); k++){
                    target_data& tgt = smp.targets[smp.active[k]];
                    target_data& tgt_compare = smp_compare.targets[smp.active[k]];
                    for(int circle_idx=0; circle_idx<7; circle_idx++){
                        dist = scalDist(tgt.model[circle_idx].p,tgt_compare.model[circle_idx].p);
                        score[i]+= dist;
                        score[j]+= dist;
                    }
                }
            }
        }
        if(votes[i]>votes[best_idx] || (votes[i]==votes[best_idx] & score[i]<score[best_idx])){
            best_idx = i;
        }
    }
*/

    int best_idx = 0;
    double best_prob = 0.0;
    for(size_t idx=0; idx<current_samples.size(); idx++){
        if(current_samples[idx].probability > best_prob){
            best_idx=idx;
            best_prob=current_samples[idx].probability;
        }
    }

    state->sampling.best_sample = current_samples[best_idx];
}

void RJMCMC_Tracker::drawTargets(sample* sample, Mat* image, float zoom){
    *image = state->sampling.current_image.clone();
    cv::resize(*image,*image,Size(0,0),zoom,zoom);
    
    for(size_t j=0;j<sample->targets.size();j++){
        if(sample->is_active[j]){
            target_data target = sample->targets[j];
            vector<circ> current_model = target.model;
            for(int k=0;k<7;k++){
                cv::circle(*image,zoom*current_model[k].p,zoom*current_model[k].r,state->io.colors[j],1);
            }
            cv::circle(*image, zoom*current_model[6].p, 2, cv::Scalar(0,10,220), CV_FILLED);
        }
    }
}


