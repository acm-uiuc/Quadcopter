#ifndef MOPSFEATURES__H_
#define MOPSFEATURES__H_

#include "MOPS_Common.h"
#include "MOPSDetector.h"
#include "MOPSExtractor.h"

class MOPSFeatures
{
 public:
    MOPSFeatures(int nUpLevels,int nDnLevels);
    void getFeatures(const Mat& image, vector<KeyPoint>& keypoints, Mat& desc);
    void drawMOPSKeypoints(const Mat& image, 
			   vector<KeyPoint>& keypoints, 
			   Mat& outImg);
 protected:
    void getImagePyrimid(Mat& image, vector<ImgPyr>& imgPyr);

    int nUpLevels_, nDnLevels_;
};

#endif
