#include "AdaptiveNonMaximalSuppression.h"

bool kpLgResponse(KeyPoint one, KeyPoint two)
{
    return one.response > two.response;
}

float edist(KeyPoint& one, KeyPoint& two)
{
    float x = one.pt.x - two.pt.x;
    float y = one.pt.y - two.pt.y;
    return sqrt(pow(x,2) + pow(y,2));
}

ANMS::ANMS(float cRobust, int nKeyPoints)
{
    cRobust_ = cRobust;
    nKeyPoints_ = nKeyPoints;
}

void ANMS::doANMS(vector<KeyPoint>& keypoints)
{
    if (keypoints.empty())
	return;

    sort(keypoints.begin(),keypoints.end(),kpLgResponse);

    vector<ANMSKeyPoint> anmsKP;
    for (int i = 0; i < keypoints.size(); i++) {
	float radius = getRadius(anmsKP,keypoints[i]);
	anmsKP.push_back(ANMSKeyPoint(keypoints[i],radius));
    }

    sort(anmsKP.begin(),anmsKP.end());

    keypoints.clear();
    for (int i = 0; i < nKeyPoints_ && i < anmsKP.size(); i++) {
	keypoints.push_back(anmsKP[i].kp_);/*
	cout << "Rad: " << anmsKP[i].radius_ << " ";
	cout << " (" << anmsKP[i].kp_.pt.x << " ";
	cout << "," << anmsKP[i].kp_.pt.y << " ";
	cout << ") Res: " << anmsKP[i].kp_.response << endl;*/
    }
}

float ANMS::getRadius(vector<ANMSKeyPoint>& kps, KeyPoint& query)
{
    float minRadius = numeric_limits<float>::infinity();

    if (kps.empty())
	return minRadius;

    for (int i = 0; i < kps.size(); i++) {
	KeyPoint& pt = kps[i].kp_;
	if (query.response < cRobust_*pt.response) {
	    float radius = edist(pt,query);
	    if (radius < minRadius)
		minRadius = radius;
	}
    }

    return minRadius;
}
