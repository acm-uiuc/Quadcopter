#ifndef MOPSFEATURES__H_
#define MOPSFEATURES__H_

#include "MOPS_Common.h"
#include "MOPSDetector.h"
#include "MOPSExtractor.h"

class MOPSFeatures
{
 public:
    MOPSFeatures(int nUpLevels, int nDnLevels, int nKeyPoints, float cRobust);
    void getFeatures(const Mat& image, vector<KeyPoint>& keypoints, Mat& desc);
    void drawMOPSKeypoints(const Mat& image, 
			   vector<KeyPoint>& keypoints, 
			   vector<ImgPyr>& imgPyr,
			   Mat& outImg);
 protected:
    void getImagePyrimid(Mat& image, vector<ImgPyr>& imgPyr);

    int nUpLevels_, nDnLevels_, nKeyPoints_;
    float cRobust_;
};

#endif
