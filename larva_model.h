#ifndef LARVA_MODEL_H
#define LARVA_MODEL_H

#include "basic_calc.h"

using namespace std;
using namespace cv;

namespace Larva_Model
{

//calculates a 7-circle larva model based on the given contour
vector<circ> calculateModel(vector<Point> *contour, int dMin=2, int dMax=10, int mask_half = 4);

//calculates angle between contour point with given index and its neighbors and given distance
double calcAngleForDist(vector<Point> *contour, int idx, int distance);

}

#endif // LARVA_MODEL_H
