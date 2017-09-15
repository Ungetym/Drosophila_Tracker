#include "motion_model.h"

Motion_Model::Motion_Model(System_State *state) :
    state(state)
{

}

bool Motion_Model::update(target_data* target){
    //set propabilities for different motion proposals
    int prob_translate=0;
    int prob_translate_part=0;
    int prob_curve_tail=3;
    int prob_curve = 15;
    int prob_expand =37;

    vector<int> curved;

    if((target->angles[0]>210 && target->angles[1]>210) || (target->angles[0]<150 && target->angles[1]<150)){
        curved.push_back(1);
    }
    if((target->angles[1]>210 && target->angles[2]>210) || (target->angles[1]<150 && target->angles[2]<150)){
        curved.push_back(2);
    }
    if((target->angles[2]>210 && target->angles[3]>210) || (target->angles[2]<150 && target->angles[3]<150)){
        curved.push_back(3);
    }

    if(curved.size()==0){
        prob_translate=7;
    }
    else{
        prob_translate_part=7;
    }

    bool is_regular_update=true;
    //choose motion type
    int type = rand()%100;

    if((type-=prob_translate) < 0){
        //std::cout<<"Translate middle\n";
        translateSpinePart(target);
    }
    else if((type-=prob_translate_part) < 0){
        //std::cout<<"Translate during curve\n";
        translateCurvedPart(target, &curved);
    }
    else if((type-=prob_curve_tail) < 0){
        //std::cout<<"Curve tail\n";
        curveSpineTail(target);
    }
    else if((type-=prob_curve) < 0){
        //std::cout<<"Curve head\n";
        curveSpineHead(target);
    }
    else if((type-=prob_expand) < 0){//expand or contract
        //std::cout<<"Expand\n";
        expandSpine(target);
    }
    else if((type-=prob_expand) < 0){//reverse update for expandSpine()
        //std::cout<<"Expand reverse\n";
        expandSpineReverse(target);
    }

    //update bounding box
    target->bounding_box = RJMCMC_Target::calcBoundingBox(target->model);

    return is_regular_update;
}

void Motion_Model::correctHeadDirection(target_data *target){
    vector<circ> model = target->model;
    vector<int> mid_idx = target->mid_idx;
    vector<Point2f> spine = target->spine;

    Point2f zero = model[4].p;
    Point2f pre = model[3].p;
    Point2f post = model[5].p;

    float current_angle = Basic_Calc::calcAngleClockwise(zero,pre,post);
    float current_angle2 = Basic_Calc::calcAngleClockwise(post,zero,model[6].p);

    int counter=0;
    //calculate rotation angle for both the two first spine parts
    float new_angle = 360.0-current_angle+rng.gaussian(15.0);
    while((new_angle<90.0 || new_angle>270.0)&&counter<150){
        new_angle = 360.0-current_angle+rng.gaussian(15.0);
        counter++;
    }
    //calculate rotation angle for head spine
    float second_angle = 360.0-current_angle2+rng.gaussian(15.0);
    while((second_angle<90.0 || second_angle>270.0)&&counter<150){
        second_angle = 360.0-current_angle2+rng.gaussian(15.0);
        counter++;
    }

    new_angle *= CV_PI/180.0;
    second_angle *= CV_PI/180.0;

    //rotate model points
    model[5].p = zero+Basic_Calc::rotate(model[3].p-zero,new_angle);
    model[6].p = model[5].p+Basic_Calc::rotate(zero-model[5].p,second_angle);

    spine.erase(spine.begin()+mid_idx[4]+1,spine.end());
    for(int i=5;i<7;i++){
        Point2f current = model[i].p;
        Point2f prev = spine.back();
        float step_size = 0.05/Basic_Calc::metricEucl(prev,current);
        Point2f spine_point = prev;
        while(Basic_Calc::metricEucl(spine_point,current)>0.05){
            spine_point+= step_size*(current-prev);
            spine.push_back(spine_point);
        }
        mid_idx[i]=spine.size()-1;
    }

    model[5].p=spine[mid_idx[5]];
    model[6].p=spine.back();

    target_data new_target=*target;
    new_target.model=model;
    new_target.mid_idx=mid_idx;
    new_target.spine=spine;

    if(checkRestrictions(&new_target)){
        //std::cout<<"Head correction accepted.\n";
        (*target)=new_target;
    }
}

void Motion_Model::curveSpineHead(target_data* target){
    vector<circ> model = target->model;
    vector<int> mid_idx = target->mid_idx;
    vector<Point2f> spine = target->spine;

    Point2f zero = model[4].p;
    Point2f pre = model[3].p;
    Point2f post = model[5].p;

    float current_angle = Basic_Calc::calcAngleClockwise(zero,pre,post);

    int counter=0;
    //calculate rotation angle for both the two first spine parts
    float new_angle = current_angle+rng.gaussian(15.0);
    while((new_angle<90.0 || new_angle>270.0)&&counter<150){
        new_angle = current_angle+rng.gaussian(15.0);
        counter++;
    }

    //calculate rotation angle for head spine
    float second_angle = new_angle+rng.gaussian(45.0);
    while(((new_angle<180.0 && (second_angle>180.0||second_angle<new_angle))||(new_angle>=180.0 &&(second_angle<180.0||second_angle>new_angle)))&&counter<150){
        second_angle = new_angle+rng.gaussian(45.0);
        counter++;
    }

    new_angle *= CV_PI/180.0;
    second_angle *= CV_PI/180.0;

    //rotate model points
    model[5].p = zero+Basic_Calc::rotate(model[3].p-zero,new_angle);
    model[6].p = model[5].p+Basic_Calc::rotate(zero-model[5].p,second_angle);

    spine.erase(spine.begin()+mid_idx[4]+1,spine.end());
    for(int i=5;i<7;i++){
        Point2f current = model[i].p;
        Point2f prev = spine.back();
        float step_size = 0.05/Basic_Calc::metricEucl(prev,current);
        Point2f spine_point = prev;
        while(Basic_Calc::metricEucl(spine_point,current)>0.05){
            spine_point+= step_size*(current-prev);
            spine.push_back(spine_point);
        }
        mid_idx[i]=spine.size()-1;
    }

    model[5].p=spine[mid_idx[5]];
    model[6].p=spine.back();

    target_data new_target=*target;
    new_target.model=model;
    new_target.mid_idx=mid_idx;
    new_target.spine=spine;

    if(checkRestrictions(&new_target)){
        (*target)=new_target;
    }
}

void Motion_Model::curveSpineTail(target_data* target){

    vector<circ> model = target->model;
    vector<int> mid_idx = target->mid_idx;
    vector<Point2f> spine = target->spine;

    Point2f zero = model[2].p;
    Point2f pre = model[1].p;
    Point2f post = model[3].p;


    float current_angle = Basic_Calc::calcAngleClockwise(zero,post,pre);
    float angle_head_mid_tail = Basic_Calc::calcAngleClockwise(model[3].p,model[0].p,model[6].p);

    int counter=0;
    //sample new rotation angle
    float new_angle=current_angle+rng.gaussian(30.0);
    if(angle_head_mid_tail>240.0){
        while((new_angle<current_angle-10.0 || new_angle>270.0)&&counter<150){
            new_angle = current_angle+rng.gaussian(30.0);
            counter++;
        }
    }
    else if(angle_head_mid_tail<120.0){
        while((new_angle<90.0 || new_angle>current_angle+10.0)&&counter<150){
            new_angle = current_angle+rng.gaussian(30.0);
            counter++;
        }
    }
    else{
        while((new_angle<90.0 || new_angle>270.0)&&counter<150){
            new_angle = current_angle+rng.gaussian(30.0);
            counter++;
        }
    }
    new_angle-=current_angle;
    new_angle *= CV_PI/180.0;

    //rotate spine tail
    vector<Point2f> spine_part(spine.begin(),spine.begin()+mid_idx[2]);
    spine_part = Basic_Calc::rotatePoints(zero,spine_part,new_angle);

    for(int i=0;i<mid_idx[2];i++){
        spine[i]=spine_part[i];
    }

    model[0].p=spine[mid_idx[0]];
    model[1].p=spine[mid_idx[1]];

    target_data new_target=*target;
    new_target.model=model;
    new_target.mid_idx=mid_idx;
    new_target.spine=spine;

    if(checkRestrictions(&new_target)){
        (*target)=new_target;
    }
}

void Motion_Model::expandSpine(target_data* target){
    vector<Point2f> spine = target->spine;
    vector<int> mid_idx = target->mid_idx;

    int counter=0;
    //sample new spinelength
    float size = 0.05*(float)(spine.size()-1);
    float size_delta = state->sampling.avg_spine_length/60.0+rng.gaussian(state->sampling.avg_spine_length/10.0);
    while((size+size_delta < 6.0*state->sampling.min_dist || size+size_delta > 6.0*state->sampling.max_dist)&&counter<150){
        size_delta = state->sampling.avg_spine_length/60.0+rng.gaussian(state->sampling.avg_spine_length/10.0);
        counter++;
    }

    int steps = (int)floor(0.5+(size_delta/(6.0*0.05)));

    if(steps<0){//retract to head
        for(int i=0;i<6;i++){
            mid_idx[i]-=(6-i)*steps;
        }
        int to_delete = mid_idx[0];
        for(int i=0;i<7;i++){
            mid_idx[i]-=to_delete;
        }
        spine.erase(spine.begin(),spine.begin()+to_delete);
    }
    else if(steps>0){//expand head

        Point2f old_pre_head = spine[spine.size()-2];
        Point2f old_head = spine.back();
        Point2f new_head = old_head+size_delta*(old_head-old_pre_head)*(1.f/static_cast<float>(Basic_Calc::metricEucl(old_head,old_pre_head)));

        spine.erase(spine.end()-2,spine.end());
        float step_size = 0.05/Basic_Calc::metricEucl(old_pre_head,new_head);
        Point2f spine_point = old_pre_head;
        while(Basic_Calc::metricEucl(spine_point,new_head)>0.05){
            spine.push_back(spine_point);
            spine_point+= step_size*(new_head-old_pre_head);
        }
        mid_idx[6]=spine.size()-1;

        int idx_dist = (int)((float)mid_idx[6]/6.0);
        for(int i=1;i<6;i++){
            mid_idx[i]=i*idx_dist;
        }

    }

    vector<circ> model=target->model;
    for(int i=0;i<7;i++){
        model[i].p=spine[mid_idx[i]];
    }

    target_data new_target=*target;
    new_target.model=model;
    new_target.mid_idx=mid_idx;
    new_target.spine=spine;

    if(checkRestrictions(&new_target)){
        (*target)=new_target;
    }
}

void Motion_Model::expandSpineReverse(target_data* target){
    vector<Point2f> spine = target->spine;
    vector<int> mid_idx = target->mid_idx;


    float size = 0.05*(float)(spine.size()-1);
    float size_delta = state->sampling.avg_spine_length/60.0+rng.gaussian(state->sampling.avg_spine_length/10.0);

    int counter=0;
    while((size+size_delta < 6.0*state->sampling.min_dist || size+size_delta > 6.0*state->sampling.max_dist)&&counter<150){
        size_delta = state->sampling.avg_spine_length/60.0+rng.gaussian(state->sampling.avg_spine_length/10.0);
        counter++;
    }

    int steps = (int)floor(0.5+(size_delta/(6.0*0.05)));

    if(steps<0){//retract to tail
        for(int i=1;i<7;i++){
            mid_idx[i]+=i*steps;
        }
        int to_delete = spine.size()-1-mid_idx[6];
        spine.erase(spine.end()-to_delete,spine.end());
    }
    else if(steps>0){//expand tail
        for(int i=5;i>0;i--){
            mid_idx[i]-=(6-i)*steps;
        }
        Point2f old_pre_tail = spine[1];
        Point2f old_tail = spine[0];
        Point2f new_tail = old_tail+size_delta*(old_tail-old_pre_tail)*(1.f/static_cast<float>(Basic_Calc::metricEucl(old_tail,old_pre_tail)));

        spine.erase(spine.begin(),spine.begin()+1);
        float step_size = 0.05/Basic_Calc::metricEucl(old_pre_tail,new_tail);
        Point2f spine_point = old_pre_tail;
        int new_spine_points=-2;
        while(Basic_Calc::metricEucl(spine_point,new_tail)>0.05){
            spine.insert(spine.begin(),spine_point);
            new_spine_points++;
            spine_point+= step_size*(new_tail-old_pre_tail);
        }
        spine.insert(spine.begin(),spine_point);

        int idx_dist = (int)(((float)spine.size())/6.0);
        for(int i=0;i<6;i++){
            mid_idx[i]=i*idx_dist;
        }
        mid_idx[6]=spine.size()-1;
    }


    vector<circ> model=target->model;
    for(int i=0;i<7;i++){
        model[i].p=spine[mid_idx[i]];
    }

    target_data new_target=*target;
    new_target.model=model;
    new_target.mid_idx=mid_idx;
    new_target.spine=spine;

    if(checkRestrictions(&new_target)){
        (*target)=new_target;
    }
}

void Motion_Model::translateSpinePart(target_data* target){

    vector<circ> model = target->model;
    vector<int> mid_idx = target->mid_idx;
    vector<Point2f> spine = target->spine;

    //sample translation distance
    int counter=0;
    float distance = rng.gaussian(state->sampling.avg_spine_length/100.0);
    while((abs(distance)>state->sampling.avg_spine_length/20.0)&&counter<150){
        distance = rng.gaussian(state->sampling.avg_spine_length/100.0);
        counter++;
    }

    //calculate translation direction
    float angle = 2.0*asin(distance/(2.0*Basic_Calc::metricEucl(model[4].p,model[5].p)));
    Point2f translation = spine[mid_idx[5]]+Basic_Calc::rotate(spine[mid_idx[4]]-spine[mid_idx[5]],angle)-spine[mid_idx[4]];

    //translate middle part
    for(int i=mid_idx[2];i<mid_idx[4]+1;i++){
        spine[i]+=translation;
    }

    //recalculate spinepart betweet mid_idx[4] and mid_idx[5]
    for(int i=mid_idx[4]+1;i<mid_idx[5];i++){
        spine[i]=spine[mid_idx[5]]+Basic_Calc::rotate(spine[i]-spine[mid_idx[5]],angle);
    }

    //recalculate spinepart betweet mid_idx[2] and tail
    Point2f tail = spine[0];
    Point2f third = spine[mid_idx[2]];

    float step_size = 0.05/Basic_Calc::metricEucl(tail,third);

    vector<Point2f> new_tail_spine;
    Point2f spine_point = tail;

    while(Basic_Calc::metricEucl(spine_point,third)>0.05){
        new_tail_spine.push_back(spine_point);
        spine_point+= step_size*(third-tail);
    }
    new_tail_spine.push_back(spine_point);

    int new_tail_size = new_tail_spine.size();

    spine.erase(spine.begin(),spine.begin()+mid_idx[2]);
    new_tail_spine.insert(new_tail_spine.end(),spine.begin(),spine.end());
    spine = new_tail_spine;

    //recalculate mid indices
    for(int i=3;i<7;i++){
        mid_idx[i]=mid_idx[i]-mid_idx[2]+new_tail_size;
    }
    mid_idx[1]=new_tail_size/2;
    mid_idx[2]=new_tail_size;

    for(int i=0;i<7;i++){
        model[i].p=spine[mid_idx[i]];
    }

    target_data new_target=*target;
    new_target.model=model;
    new_target.mid_idx=mid_idx;
    new_target.spine=spine;

    if(checkRestrictions(&new_target)){
        (*target)=new_target;
    }
}

void Motion_Model::translateCurvedPart(target_data* target, vector<int>* curved){

    int random = rand()%curved->size();
    random = (*curved)[random]-2;

    vector<circ> model = target->model;
    vector<int> mid_idx = target->mid_idx;
    vector<Point2f> spine = target->spine;

    //sample translation distance
    int counter=0;
    float distance = rng.gaussian(state->sampling.avg_spine_length/20.0);
    while((abs(distance)>state->sampling.avg_spine_length/12.0)&&counter<150){
        distance = rng.gaussian(state->sampling.avg_spine_length/20.0);
        counter++;
    }

    //calculate translation vector
    Point2f translation = distance/target->distances[2+random]*Basic_Calc::rotate(model[3+random].p-model[2+random].p,0.5*CV_PI);
    //translate spine part
    for(int i=mid_idx[2+random];i<mid_idx[3+random]+1;i++){
        spine[i]+=translation;
    }

    //recalculate spine before translated part
    Point2f first = spine[mid_idx[1+random]];
    Point2f second = spine[mid_idx[2+random]];
    float step_size = 0.05/Basic_Calc::metricEucl(first,second);
    vector<Point2f> new_mid_spine;
    Point2f spine_point = first;
    while(Basic_Calc::metricEucl(spine_point,second)>0.05){
        new_mid_spine.push_back(spine_point);
        spine_point+= step_size*(second-first);
    }
    new_mid_spine.push_back(spine_point);
    spine.erase(spine.begin()+mid_idx[1+random],spine.begin()+mid_idx[2+random]);
    spine.insert(spine.begin()+mid_idx[1+random],new_mid_spine.begin(),new_mid_spine.end());

    int idx_delta = new_mid_spine.size()-(mid_idx[2+random]-mid_idx[1+random]);
    for(int i=2+random;i<7;i++){
        mid_idx[i]+=idx_delta;
    }

    //recalculate spine after translated part
    Point2f third = spine[mid_idx[3+random]];
    Point2f fourth = spine[mid_idx[4+random]];
    step_size = 0.05/Basic_Calc::metricEucl(third,fourth);
    vector<Point2f> new_mid_spine2;
    spine_point = third;

    while(Basic_Calc::metricEucl(spine_point,fourth)>0.05){
        new_mid_spine2.push_back(spine_point);
        spine_point+= step_size*(fourth-third);
    }
    new_mid_spine2.push_back(spine_point);

    spine.erase(spine.begin()+mid_idx[3+random],spine.begin()+mid_idx[4+random]);
    spine.insert(spine.begin()+mid_idx[3+random],new_mid_spine2.begin(),new_mid_spine2.end());

    float spine_size= spine.size();

    for(int i=1;i<6;i++){
        mid_idx[i]=(int)(i*spine_size/6.0);
    }
    mid_idx[6]=spine_size-1;

    for(int i=0;i<7;i++){
        model[i].p=spine[mid_idx[i]];
    }

    target_data new_target=*target;
    new_target.model=model;
    new_target.mid_idx=mid_idx;
    new_target.spine=spine;

    if(checkRestrictions(&new_target)){
        (*target)=new_target;
    }
}

bool Motion_Model::checkRestrictions(target_data* target){
    bool legit=true;

    vector<circ> model = target->model;
    target->distances.clear();
    target->angles.clear();

    //distance check
    for(int i=0;i<6;i++){
        Point2f p = model[i].p;
        for(int j=i+1;j<7;j++){
            Point2f q = model[j].p;
            float dist = (float)Basic_Calc::metricEucl(p,q);
            if(state->sampling.min_dist>dist){
                return false;
            }
            if(j==i+1 && dist>state->sampling.max_dist){
                return false;
            }
            if(j==i+1){
                target->distances.push_back(dist);
            }
        }
    }

    //angle check
    float min_angle;
    min_angle = 90;
    float max_angle;
    max_angle = 270;

    for(int i=1;i<6;i++){
        Point2f last = model[i-1].p;
        Point2f current = model[i].p;
        Point2f next = model[i+1].p;

        float angle = Basic_Calc::calcAngleClockwise(current,last,next);
        if(min_angle>angle || angle>max_angle){
            //std::cout<<"Angle "<<i<<" out of bounds: "<<angle<<"\n";
            legit=false;
            return legit;
        }
        else{
            target->angles.push_back(angle);
        }
    }

    return legit;
}
