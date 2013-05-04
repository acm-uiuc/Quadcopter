#ifndef CONTOURKEYPOINTS__H_
#define CONTOURKEYPOINTS__H_

#include "Contours_Common.h"

class ContourKeypoints
{
 public:
    ContourKeypoints(int thresh, int stepSize);
    void detect(const Mat& image, vector<KeyPoint>& keypoints);
 private:
    void getKeypoints(vector<Point>& contour, vector<KeyPoint>& keypoints);

    int thresh_, stepSize_;
};

#endif
