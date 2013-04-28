#ifndef MOPS_COMMON__H_
#define MOPS_COMMON__H_

#include <opencv/cv.h>
#include <opencv/highgui.h>

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
