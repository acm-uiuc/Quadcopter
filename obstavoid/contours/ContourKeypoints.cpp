#include "ContourKeypoints.h"

bool ltPoint(Point one, Point two)
{
    if (one.x == two.x)
	return one.y < two.y;
    return one.x < two.x;
}

ContourKeypoints::ContourKeypoints(int thresh, int stepSize)
{
    thresh_ = thresh;
    stepSize_ = stepSize;
}

void ContourKeypoints::detect(const Mat& image, vector<KeyPoint>& keypoints)
{
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);

    /// Detect edges using canny
    Canny(image,canny_output,thresh_,thresh_*2,3);
    /// Find contours
    findContours(canny_output,contours,hierarchy,CV_RETR_TREE, 
		 CV_CHAIN_APPROX_SIMPLE,Point(0,0));

    for (int i = 0; i < contours.size(); i++) {
	vector<Point> points(contours[i]);
	//getKeypoints(contours[i],keypoints);
	sort(points.begin(),points.end(),ltPoint);

	for (int j = 0; j < points.size(); j++) {
	    if (keypoints.empty())
		keypoints.push_back(KeyPoint(points[j],1));
	    else {
		float sqdiffx = pow(keypoints.back().pt.x - points[j].x,2);
		float sqdiffy = pow(keypoints.back().pt.y - points[j].y,2);
		if ((sqdiffx + sqdiffy) > stepSize_)
		    keypoints.push_back(KeyPoint(points[j],32));
	    }
	}
    }

    /// Draw contours
    Mat drawing = Mat::zeros(canny_output.size(),CV_8UC3);
    for (int i = 0; i < contours.size(); i++) {
	    Scalar color = Scalar(rng.uniform(0,255),rng.uniform(0,255), 
				  rng.uniform(0,255));
	    drawContours(drawing,contours,i,color,2,8,hierarchy,0,Point());
    }
    imshow("frame",image);
    imshow("Contours", drawing);
    waitKey(0);
}

void ContourKeypoints::getKeypoints(vector<Point>& contour, 
				    vector<KeyPoint>& keypoints)
{
    if (contour.empty())
	return;

    
}
