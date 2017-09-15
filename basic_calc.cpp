#include "basic_calc.h"


int Basic_Calc::mod(int a, int b){
    int c = a%b;
    if(c<0){
        c+=b;
    }
    return c;
}

int Basic_Calc::modDistance(int a, int b, int m){
    int c = 0;
    if(a<b){
        c=b-a;
    }
    else if(b<a){
        c=m-(a-b);
    }
    return c;
}

double Basic_Calc::normEucl(cv::Point2f v){
    return sqrt((v.x*v.x)+(v.y*v.y));
}

double Basic_Calc::metricEucl(cv::Point2f p, cv::Point2f q){
    return normEucl(p-q);
}

double Basic_Calc::calcAngle(cv::Point2f o, cv::Point2f p, cv::Point2f q){
    cv::Point2f v = p-o;
    cv::Point2f w = q-o;

    double cos_alpha = (v.x*w.x + v.y*w.y) / (normEucl(v) * normEucl(w));

    double angle;

    if(cos_alpha >= 1 || cos_alpha <=-1)
        angle = 180.0;
    else
        angle = (acos(cos_alpha) * 180 / CV_PI);

    double sgn = (v.x * w.y) - (v.y * w.x);

    if(sgn < 0)
        angle = 360 - angle;
    return angle;
}

double Basic_Calc::calcAngleClockwise(cv::Point2f p, cv::Point2f q){
    double result = -(atan2(((p.x * q.y) - (p.y * q.x)),(p.x*q.x + p.y*q.y))* 180 / CV_PI);

    if(result<0){
        result=360.0+result;
    }

    return result;
}

double Basic_Calc::calcAngleClockwise(cv::Point2f o, cv::Point2f p, cv::Point2f q){
    cv::Point2f v = p-o;
    cv::Point2f w = q-o;
    double result = -(atan2(((v.x * w.y) - (v.y * w.x)),(v.x*w.x + v.y*w.y))* 180 / CV_PI);

    if(result<0){
        result=360.0+result;
    }

    return result;
}


/////////////////////////////////////////////////////////////////////////////////////////////////


bool Basic_Calc::rectIntersection(cv::Rect rect_1, cv::Rect rect_2){
    int x1 = rect_1.x;
    int x2 = x1+rect_1.width;
    int y1 = rect_1.y;
    int y2 = y1+rect_1.height;

    int a1 = rect_2.x;
    int a2 = a1+rect_2.width;
    int b1 = rect_2.y;
    int b2 = b1+rect_2.height;

    bool check1 = false;
    bool check2 = false;
    if((a1<=x1 && a2>x1)||(a1<x2&&a2>=x2)||(a1>x1 && a2<x2)){
        check1 = true;
    }
    if((b1<=y1 && b2>y1)||(b1<y2&&b2>=y2)||(b1>y1 && b2<y2)){
        check2 = true;
    }

    return check1&check2;
}

bool Basic_Calc::rectInsideImage(float scale, cv::Rect rect, cv::Mat* image){
    bool result = true;

    int x = rect.x-(int)((scale-1.0)/2.0*(float)rect.width);
    int y = rect.y-(int)((scale-1.0)/2.0*(float)rect.height);
    int x2 = x+(int)(scale*(float)rect.width);
    int y2 = y+(int)(scale*(float)rect.height);

    if(x<0 || x2>image->cols-1 || y<0 || y2>image->rows-1){
        result = false;
    }
    return result;
}

cv::Rect Basic_Calc::surroundingRect(cv::Rect rect_1, cv::Rect rect_2){
    int x1 = std::min(rect_1.x,rect_2.x);
    int y2 = std::min(rect_1.y,rect_2.y);
    int width = std::max(rect_1.x+rect_1.width,rect_2.x+rect_2.width)-x1;
    int height = std::max(rect_1.y+rect_1.height,rect_2.y+rect_2.height)-y2;

    if(x1<0){
        width+=x1;
        x1=0;
    }
    if(y2<0){
        height+=y2;
        y2=0;
    }


    return cv::Rect(x1,y2,width,height);
}


//////////////////////////////////////////////////////////////////////////////////////


cv::Point2f Basic_Calc::circleLineCut(cv::Point2f a, cv::Point2f b, circ c){
    cv::Point2f result = cv::Point2f(0.0,0.0);
    float dist_ap = metricEucl(c.p,a);
    float dist_bp = metricEucl(c.p,b);

    if(dist_ap < c.r && dist_bp > c.r){
        float dist_ab = metricEucl(a,b);
        dist_ab = 1.0/(2.0*dist_ab);
        cv::Point2f in = a;
        cv::Point2f out = b;
        float factor = 0.5;
        float step = 0.5;
        while(step>dist_ab){
            step*=0.5;
            cv::Point2f current = a + factor*(b-a);
            if(metricEucl(current,c.p)>c.r){
                out = current;
                factor-=step;
            }
            else{
                in = current;
                factor+=step;
            }
        }
        result = 0.5*(in+out);
    }
    return result;
}

cv::Point2f Basic_Calc::rotate(cv::Point2f p, float angle){
    cv::Point2f result;
    float cos_a = cos(angle);
    float sin_a = sin(angle);
    result.x = cos_a*p.x + sin_a*p.y;
    result.y = -sin_a*p.x + cos_a*p.y;
    return result;
}

std::vector<cv::Point2f> Basic_Calc::rotatePoints(cv::Point2f zero,std::vector<cv::Point2f> v, float angle){
    std::vector<cv::Point2f> result = v;

    float cos_a = cos(angle);
    float sin_a = sin(angle);

    for(size_t i=0;i<v.size();i++){
        cv::Point2f temp = v[i]-zero;
        result[i].x=zero.x+cos_a*temp.x + sin_a*temp.y;
        result[i].y=zero.y-sin_a*temp.x + cos_a*temp.y;
    }

    return result;
}

cv::Point2f Basic_Calc::polarCoord(cv::Point2f p){
    float r = normEucl(p);
    float phi = calcAngleClockwise(cv::Point2f(0.0,0.0),cv::Point2f(1.0,0.0),p);
    cv::Point2f result = cv::Point2f(r,phi);
    return result;
}


//////////////////////////////////////////////////////////////////////////////////////////////


float Basic_Calc::gauss1DPlateau(float x, float border, float sigma){
    float norm = 1.0/(2*border+sigma*sqrt(CV_PI/2.0));
    float result=norm;
    if(x<(-border) || x>border){
        result *= exp(-(x-border)*(x-border)/(2.0*sigma*sigma));
    }
    return result;
}

float Basic_Calc::gauss2DPlateau(cv::Point2f p, float sigma_x, float sigma_y, float x_size){
    float norm = 1.0/(2.0*CV_PI*sqrt(sigma_x*sigma_y));
    float plateau_height = norm*exp(-0.5*x_size*x_size/sigma_x);

    float f_of_p = norm*exp(-0.5*(p.x*p.x/sigma_x+p.y*p.y/sigma_y));

    if(f_of_p>plateau_height){
        f_of_p=plateau_height;
    }

    return f_of_p;
}

float Basic_Calc::gaussian2Deasy(cv::Point2f p, float sigma_1, float sigma_2){
    float result = 1.0/(2.0*CV_PI*sqrt(sigma_1*sigma_2));
    result *= exp(-0.5*(p.x*p.x/sigma_1+p.y*p.y/sigma_2));
    return result;
}

float Basic_Calc::gaussConstPdf(cv::Point2f p, float sigma_x1, float sigma_x2, float sigma_y, float plateau){
    float result = 1.0/(sigma_y*sqrt(2.0*CV_PI))*exp(-p.y*p.y/(2*sigma_y*sigma_y));

    float norm = 1.0/(plateau+sqrt(CV_PI/2.0)*(sigma_x1+sigma_x2));
    result*=norm;
    if(p.x<0){
        result *= exp(-p.x*p.x/(2.0*sigma_x1*sigma_x1));
    }
    else if(p.x>plateau){
        result *= exp(-(p.x-plateau)*(p.x-plateau)/(2.0*sigma_x2*sigma_x2));
    }
    return result;
}

float Basic_Calc::gaussRotationPlateau(cv::Point2f p, float sigma_r, float r_1, float r_2, float alpha, float beta, float exponent){
    float r = p.x;
    float phi = p.y;

    exponent*=2.0;
    sigma_r *=sigma_r;
    float norm = 1.0/((beta-alpha+(270.0-beta+alpha-90.0)/(2.0*exponent))*(sigma_r+(r_2-r_1)/2.0+r_1*r_1/(exponent+2.0)));

    float g_r = 1.0;
    if(r<r_1){
        g_r = pow(r/r_1,exponent);
    }
    else if(r>r_2){
        g_r=exp(-0.5*(r-r_2)*(r-r_2)/sigma_r);
    }

    float f_phi = 1.0;
    if(phi<90 || phi>270){
        f_phi = 0.0;
    }
    else if(phi>beta){
        f_phi=pow((phi-270)/(beta-270),exponent);
    }
    else if(phi<alpha){
        f_phi=pow((phi-90)/(alpha-90),exponent);
    }

    return norm*g_r*f_phi;
}







