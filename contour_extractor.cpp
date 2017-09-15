#include "contour_extractor.h"

contour_extractor::contour_extractor(){

}

void contour_extractor::initBackground(cv::Mat* image){
    cv::cvtColor(*image,current_background,CV_BGR2GRAY);
    std::vector<float> hist = this->calcHist(&current_background,8.0);
    int max_color = 0;
    float max = 0;
    for(size_t i=0;i<hist.size();i++){
        if(hist[i]>max){
            max=hist[i];
            max_color=i;
        }
    }
    current_background = cv::Mat(image->rows,image->cols,CV_8U,cv::Scalar(max_color*8.0+4.0));
}

std::vector<std::vector<cv::Point>> contour_extractor::extractContours(cv::Mat *image, cv::Mat *binary_image, int min_area, int max_area){
    //detect targets via background subtraction
    cv::Mat tmpImg, tmpCopy;
    cv::cvtColor(*image,tmpImg,CV_BGR2GRAY);
    tmpCopy=tmpImg.clone();

    //background subtraction and thresholding
    tmpImg-=current_background;

    cv::threshold(tmpImg,tmpImg,20,255,CV_THRESH_BINARY);

    //*binary_image=tmpImg.clone();

    //contour extraction
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(tmpImg,contours,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());

    //calculate bounding boxes for all contours
    std::vector<cv::Rect> bounding_boxes;
    for(size_t i=0;i<contours.size();i++){
        bounding_boxes.push_back(boundingRect(contours[i]));
    }

    //delete contours intersecting the image borders
    std::vector<std::vector<cv::Point>> contours_border;
    std::vector<cv::Rect> bounding_boxes_border;
    for(size_t i=0;i<contours.size();i++){
        if(Basic_Calc::rectInsideImage(1.0,bounding_boxes[i],&tmpImg)){
            contours_border.push_back(contours[i]);
            bounding_boxes_border.push_back(bounding_boxes[i]);
        }
    }
    bounding_boxes=bounding_boxes_border;
    contours=contours_border;

    //delete contours that are too small or big (add them to the background image)
    std::vector<std::vector<cv::Point>> current_contours;
    std::vector<cv::Rect> current_bounding_boxes;
    for(size_t i=0;i<contours.size();i++){
        std::vector<cv::Point> contour = contours[i];
        double area = contourArea(contour);
        if(area>min_area && area<max_area){
            current_contours.push_back(contour);
            current_bounding_boxes.push_back(bounding_boxes[i]);
        }
        else{//very small or large contourareas are added to the background image

            cv::Rect bounding_rect = cv::boundingRect(contour);

            if(bounding_rect.x>0){
                bounding_rect.x--;
            }
            if(bounding_rect.y>0){
                bounding_rect.y--;}
            if(bounding_rect.width<=tmpImg.cols-2){
                bounding_rect.width+=2;
            }
            if(bounding_rect.height<=tmpImg.rows-2){
                bounding_rect.height+=2;
            }

            cv::Mat box = tmpCopy(bounding_rect).clone();
            addBackgroundObject(&box,contour,bounding_rect.x,bounding_rect.y);
        }
    }

    //TODO: analyse the remaining contours regarding their curvature profile

    //draw binary image of filled contours;
    *binary_image = cv::Mat::zeros(tmpImg.rows,tmpImg.cols,tmpImg.type());
    for(size_t i=0;i<current_contours.size();i++){
        cv::drawContours(*binary_image,current_contours,i,cv::Scalar(255,255,255),-1);
    }

    return current_contours;
}


void contour_extractor::addBackgroundObject(cv::Mat* bound_rect, std::vector<cv::Point> contour, int x_pos, int y_pos){
    //create mask
    cv::Mat mask(bound_rect->rows,bound_rect->cols,bound_rect->type(),cvScalar(1));

    //draw contour on mask
    for(size_t i=0;i<contour.size();i++){
        mask.at<uchar>(contour[i].y-y_pos,contour[i].x-x_pos)=0;
    }

    //reverse floodfill
    revFloodFill(&mask,0,0);
    //delete contour from mask
    for(size_t i=0;i<contour.size();i++){
        mask.at<uchar>(contour[i].y-y_pos,contour[i].x-x_pos)=1;
    }

    //add object to background image
    cv::Mat background_roi = current_background(cv::Rect(x_pos,y_pos,mask.cols,mask.rows));
    bound_rect->copyTo(background_roi,mask);
}

void contour_extractor::revFloodFill(cv::Mat* image, int a, int b){
    std::vector<std::pair<int,int>> stack;
    stack.push_back(std::pair<int,int>(a,b));
    int x,y;
    std::pair<int,int> p;
    while(stack.size()>0){
        p = stack.back();
        stack.pop_back();
        x = p.first;
        y = p.second;
        if(x>=0 && x<=image->cols-1 && y>=0 && y<=image->rows-1 && image->at<uchar>(y,x)==1){
            image->at<uchar>(y,x)=0;
            stack.push_back(std::pair<int,int>(x+1,y));
            stack.push_back(std::pair<int,int>(x-1,y));
            stack.push_back(std::pair<int,int>(x,y+1));
            stack.push_back(std::pair<int,int>(x,y-1));
        }
    }
}

std::vector<float> contour_extractor::calcHist(cv::Mat* image, float size){
    std::vector<float> hist;
    for(int i=0;i<256.0/size;i++){
        hist.push_back(0.0);
    }
    for(int i=0;i<image->cols;i++){
        for(int j=0;j<image->rows;j++){
            float current = (float)(image->at<uchar>(j,i));
            current/=size;
            int idx = (int)floor(current);
            hist[idx]+=1.0;
        }
    }

    return hist;
}
