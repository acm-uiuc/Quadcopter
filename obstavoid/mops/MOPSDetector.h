#ifndef MOPSDETECTOR__H_
#define MOPSDETECTOR__H_

#include "MOPS_Common.h"
#include "AdaptiveNonMaximalSuppression.h"

class MOPSDetector
{

 public:
    
    MOPSDetector(float cRobust,int nKeyPoints);
    void detect(vector<ImgPyr>& imgPyr, vector<KeyPoint>& keypoints);

 protected:
    
    ANMS anms;
};

#endif
