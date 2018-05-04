#include "system_state.h"

using namespace std;
using namespace cv;

System_State::System_State(){
    //generate some colors for output sequences
    RNG rng;
    io.colors.push_back(Scalar(0,255,0));
    io.colors.push_back(Scalar(255,0,0));
    io.colors.push_back(Scalar(0,0,255));
    io.colors.push_back(Scalar(255,255,0));
    io.colors.push_back(Scalar(255,0,255));
    io.colors.push_back(Scalar(0,255,255));
    for(int i=0;i<100;i++){
        io.colors.push_back(Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255)));
    }
}
