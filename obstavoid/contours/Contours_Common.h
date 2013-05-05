#ifndef CONTOURS_COMMON__H_
#define CONTOURS_COMMON__H_

#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <string.h>
#include <iostream>

using namespace cv;
using namespace std;

struct ImgPyr 
{
    ImgPyr(Mat& im, float adj) { img_ = im; adj_ = adj;}
    Mat img_;
    float adj_;
};

static const int DESCRIPTORSIZE = 40;

#endif
