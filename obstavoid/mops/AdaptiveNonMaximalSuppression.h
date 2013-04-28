#ifndef ADAPTIVENONMAXIMALSUPPRESSION__H_
#define ADAPTIVENONMAXIMALSUPPRESSION__H_

#include "MOPS_Common.h"

class ANMS
{
    struct ANMSKeyPoint
    {
	ANMSKeyPoint(KeyPoint& kp, float radius) : kp_(kp), radius_(radius) {}
	bool operator<(ANMSKeyPoint pt) const { return radius_ > pt.radius_; }
	KeyPoint kp_;
	float radius_;
    };

 public:
    ANMS(float cRobust = 0.9, int nKeyPoints = 15);
    void doANMS(vector<KeyPoint>& keypoints);
    float getRadius(vector<ANMSKeyPoint>& kps, KeyPoint& query);
 private:
    float cRobust_;
    int nKeyPoints_;
};

#endif
