#ifndef MOPSEXTRACTOR__H_
#define MOPSEXTRACTOR__H_

#include "MOPS_Common.h"

class MOPSExtractor
{
 public:
    MOPSExtractor();
    void compute(const vector<ImgPyr>& imgPyr, vector<KeyPoint>& keypoints,
		 Mat& descriptors) const;
 protected:

};

#endif
