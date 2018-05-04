#include "larva_model.h"

std::vector<circ> Larva_Model::calculateModel(std::vector<cv::Point> *contour, int dMin, int dMax, int mask_half){

    //calculate curvatures for all contour points
    std::vector<double> curvatures;
    size_t contour_size = contour->size();
    for (int i=0; i < (int) contour_size; i++)
    {
        double angle = 360;
        // for all points in the contour with distance in [dMin, dMax]
        for (int dist = dMin; dist <= dMax; ++dist)
        {
            double cur_angle=calcAngleForDist(contour,i,dist);
            if (cur_angle < angle)
            {
                angle = cur_angle;
            }
        }
        // push the smallest found angle into the curvatures vector
        curvatures.push_back(angle);
    }

    //find max curvature idx
    int max_curv_idx = 0;
    double max_curv = 0;

    for (int i=0; i < (int) contour_size; i++)
    {
        double cur_mean_curv = 0;
        for (int offset = -mask_half; offset <= mask_half; offset++)
        {
            int cur_index = Basic_Calc::mod(i+offset,contour_size);
            cur_mean_curv += curvatures.at(cur_index);
        }
        if (cur_mean_curv > max_curv)
        {
            max_curv = cur_mean_curv;
            max_curv_idx = i;
        }
    }

    //find idx of point with second highest curvature
    double max_curv_2 = 0;
    int start_idx = Basic_Calc::mod(max_curv_idx+(int)(0.125*contour_size), contour_size);
    int max_curv_idx_2 = start_idx;
    int range = (int)(0.75*contour_size);

    for (int i=0; i < range; i++)
    {
        int cur_idx = Basic_Calc::mod(start_idx+i,contour_size);
        double cur_mean_curv = 0;
        for (int offset = -mask_half; offset <= mask_half; offset++)
        {
            int cur_index = Basic_Calc::mod(cur_idx+offset,contour_size);
            cur_mean_curv += curvatures.at(cur_index);
        }
        if (cur_mean_curv > max_curv_2)
        {
            max_curv_2 = cur_mean_curv;
            max_curv_idx_2 = cur_idx;
        }
    }

    //calculate 7 model circles
    std::vector<circ> model;
    int step_1 = Basic_Calc::modDistance(max_curv_idx,max_curv_idx_2,contour_size);
    int step_2 = contour_size-step_1;

    step_1=(int)(step_1/8.0);
    step_2=(int)(step_2/8.0);

    for(int i=1;i<8;i++){
        int idx_1=Basic_Calc::mod(max_curv_idx+(i*step_1),contour_size);
        int idx_2=Basic_Calc::mod(max_curv_idx-(i*step_2),contour_size);
        cv::Point2f mid = contour->at(idx_1)+contour->at(idx_2);
        float rad = 0.5*Basic_Calc::metricEucl(contour->at(idx_1),contour->at(idx_2));
        circ c = {cv::Point2f(mid.x/2.0,mid.y/2.0),rad};
        model.push_back(c);
    }

    float spine_length = 0.0;
    for(int i=1;i<7;i++){
        spine_length+=Basic_Calc::metricEucl(model[i].p,model[i-1].p);
    }

    float avg_rad = 0.0;
    for(int i=1;i<6;i++){
        avg_rad+=model[i].r;
    }
    avg_rad/=5.0;

    float accepted_rad = 0.0;
    if(avg_rad>1.2*spine_length/6.0){
        accepted_rad=1.2*spine_length/6.0;
    }
    else if(avg_rad<0.8*spine_length/6.0){
        accepted_rad=0.8*spine_length/6.0;
    }
    else{
        accepted_rad=avg_rad;
    }

    accepted_rad*=1.05;

    model[0].r=0.8*accepted_rad;
    for(int i=1;i<6;i++){
        model[i].r=accepted_rad;
    }
    model[6].r=0.8*accepted_rad;

    return model;
}

double Larva_Model::calcAngleForDist(vector<Point> *contour, int idx, int distance){

    size_t contour_size = contour->size();
    cv::Point o = contour->at(idx);

    //find predecessor ans successor of Point o with the given euclidian distance
    int p_offset = 1;
    int q_offset = -1;
    int p_Index = Basic_Calc::mod((idx+p_offset),contour_size);
    int q_Index = Basic_Calc::mod((idx+q_offset),contour_size);

    cv::Point p = contour->at(p_Index);
    cv::Point q = contour->at(q_Index);

    double p_dist = Basic_Calc::metricEucl(o, p);
    double q_dist = Basic_Calc::metricEucl(o, q);

    while(p_dist < (double) distance && p_offset < (int) contour_size)
    {
        p_offset++;
        p_Index = Basic_Calc::mod((idx+p_offset),contour_size);
        p = contour->at(p_Index);
        p_dist = Basic_Calc::metricEucl(o, p);
    }

    while(q_dist < (double) distance && q_offset < (int) contour_size)
    {
        q_offset--;
        q_Index = Basic_Calc::mod((idx+q_offset),contour_size);
        q = contour->at(q_Index);
        q_dist = Basic_Calc::metricEucl(o, q);
    }

    //calculate angle between p-o and q-o
    return Basic_Calc::calcAngle(o, p, q);
}
