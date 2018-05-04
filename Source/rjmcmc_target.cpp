#include "rjmcmc_target.h"

target_data RJMCMC_Target::targetFromModel(const vector<circ>& model){
    target_data new_target;
    new_target.model=model;
    new_target.bounding_box = calcBoundingBox(new_target.model);

    //construct spine
    new_target.spine.clear();
    new_target.mid_idx.clear();
    new_target.mid_idx.push_back(0);
    for(int i=1;i<7;i++){
        Point2f last = model[i-1].p;
        Point2f current = model[i].p;
        float step_size = 0.05/Basic_Calc::metricEucl(last,current);
        Point2f spine_point = last;
        while(Basic_Calc::metricEucl(spine_point,current)>0.05){
            new_target.spine.push_back(spine_point);
            spine_point+= step_size*(current-last);
        }
        new_target.mid_idx.push_back(new_target.spine.size());
    }
    new_target.spine.push_back(model[6].p);

    //calculate distances between model points
    new_target.distances.clear();
    for(int i=1;i<7;i++){
        new_target.distances.push_back(Basic_Calc::metricEucl(model[i].p,model[i-1].p));
    }
    //calculate angles between model spine segments
    new_target.angles.clear();
    for(int i=1;i<6;i++){
        new_target.angles.push_back(Basic_Calc::calcAngleClockwise(model[i].p,model[i-1].p,model[i+1].p));
    }

    return new_target;
}


Rect RJMCMC_Target::calcBoundingBox(const vector<circ>& model){
    //calculate min/max x and y coordinates using the circle midpoints and radii
    int x_1 = model[0].p.x-model[0].r;
    int x_2 = model[0].p.x+model[0].r;
    int y_1 = model[0].p.y-model[0].r;
    int y_2 = model[0].p.y+model[0].r;

    for(int i=1;i<7;i++){
        x_1 = min(x_1,(int)(model[i].p.x-model[i].r));
        x_2 = max(x_2,(int)(model[i].p.x+model[i].r));
        y_1 = min(y_1,(int)(model[i].p.y-model[i].r));
        y_2 = max(y_2,(int)(model[i].p.y+model[i].r));
    }

    return Rect(x_1,y_1,x_2-x_1,y_2-y_1);
}


bool RJMCMC_Target::boxIntersection(const target_data& target_1, const target_data& target_2){
    return Basic_Calc::rectIntersection(target_1.bounding_box,target_2.bounding_box);
}


int RJMCMC_Target::modelIntersection(const target_data& target_1, const target_data& target_2){
    //upscale and therefore increase the precision of the circle drawing
    float zoom = 10.0/max(target_1.model[3].r,target_2.model[3].r);

    int x_offset = min(target_1.bounding_box.x, target_2.bounding_box.x);
    int y_offset = min(target_1.bounding_box.y, target_2.bounding_box.y);
    int x_max = max(target_1.bounding_box.x+target_1.bounding_box.width, target_2.bounding_box.x+target_2.bounding_box.width);
    int y_max = max(target_1.bounding_box.y+target_1.bounding_box.height, target_2.bounding_box.y+target_2.bounding_box.height);

    Mat model_image_1 = Mat::zeros(zoom*(y_max-y_offset),zoom*(x_max-x_offset), CV_8U);
    Mat model_image_2 = Mat::zeros(zoom*(y_max-y_offset),zoom*(x_max-x_offset), CV_8U);
    Mat overlap_image = Mat::zeros(zoom*(y_max-y_offset),zoom*(x_max-x_offset), CV_8U);

    const vector<circ>& model_1 = target_1.model;
    const vector<circ>& model_2 = target_2.model;

    //draw models
    for(int i=0;i<7;i++){
        circle(model_image_1,zoom*Point2f(model_1[i].p.x-x_offset,model_1[i].p.y-y_offset),zoom*model_1[1].r,Scalar(255,255,255),-1);
        circle(model_image_2,zoom*Point2f(model_2[i].p.x-x_offset,model_2[i].p.y-y_offset),zoom*model_2[1].r,Scalar(255,255,255),-1);
    }
    //calculate overlap
    overlap_image = model_image_1 & model_image_2;

    int overlap = (int)(cv::sum(overlap_image)[0]/(zoom*255.0));

    return overlap;
}

vector<float> RJMCMC_Target::modelIntersectionRatios(const target_data& target_1, const target_data& target_2){

    vector<float> result;

    //upscale and therefore increase the precision of the circle drawing
    float zoom = 10.0/max(target_1.model[3].r,target_2.model[3].r);
    float overlap_ratio = 0.0;

    int x_offset = min(target_1.bounding_box.x, target_2.bounding_box.x);
    int y_offset = min(target_1.bounding_box.y, target_2.bounding_box.y);
    int x_max = max(target_1.bounding_box.x+target_1.bounding_box.width, target_2.bounding_box.x+target_2.bounding_box.width);
    int y_max = max(target_1.bounding_box.y+target_1.bounding_box.height, target_2.bounding_box.y+target_2.bounding_box.height);


    Mat model_image_2 = Mat::zeros(zoom*(y_max-y_offset),zoom*(x_max-x_offset), CV_8U);
    Mat overlap_image = Mat::zeros(zoom*(y_max-y_offset),zoom*(x_max-x_offset), CV_8U);

    vector<circ> model_1 = target_1.model;
    vector<circ> model_2 = target_2.model;

    //draw model 2
    for(int i=0;i<7;i++){
        circle(model_image_2,zoom*Point2f(model_2[i].p.x-x_offset,model_2[i].p.y-y_offset),zoom*model_2[1].r,Scalar(255,255,255),-1);
    }

    //draw single circles of model 1
    for(int i=0;i<7;i++){
        Mat model_image_1 = Mat::zeros(zoom*(y_max-y_offset),zoom*(x_max-x_offset), CV_8U);
        circle(model_image_1,zoom*Point2f(model_1[i].p.x-x_offset,model_1[i].p.y-y_offset),zoom*model_1[1].r,Scalar(255,255,255),-1);
        overlap_image = model_image_1 & model_image_2;

        float overlap = cv::sum(overlap_image)[0]/(zoom*255.0);
        float circle_size = cv::sum(model_image_1)[0]/(zoom*255.0);

        overlap_ratio = overlap/circle_size;
        result.push_back(overlap_ratio);

    }
    return result;
}

float RJMCMC_Target::contourModelIntersection(const target_data& target, const vector<Point>& contour, const Rect& bounding_box){

    Rect combined_bounding_box = Basic_Calc::surroundingRect(bounding_box,target.bounding_box);

    int x_offset = combined_bounding_box.x;
    int y_offset = combined_bounding_box.y;

    //draw model
    Mat model_image = Mat::zeros(combined_bounding_box.height,combined_bounding_box.width, CV_8U);
    vector<circ> model = target.model;
    for(int i=0;i<8;i++){
            circle(model_image,Point2f(model[i].p.x-x_offset,model[i].p.y-y_offset),model[1].r,Scalar(255,255,255),-1);
    }

    //draw contour
    Mat contour_image = Mat::zeros(combined_bounding_box.height,combined_bounding_box.width, CV_8U);
    vector<vector<Point>> contours;
    vector<Point> contour_translated;
    for(size_t i=0;i<contour.size();i++){
        Point p(contour[i].x-x_offset,contour[i].y-y_offset);
        contour_translated.push_back(p);
    }
    contours.push_back(contour_translated);

    drawContours(contour_image,contours,0,Scalar(255,255,255),CV_FILLED);

    //calculate overlap
    Mat overlap_image = model_image & contour_image;

    float overlap_size = cv::sum(overlap_image)[0];
    float model_size = cv::sum(model_image)[0];

    return overlap_size/model_size;
}
