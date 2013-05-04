#include "MOPSFeatures.h"

MOPSFeatures::MOPSFeatures(int nUpLevels, int nDnLevels, int nKeyPoints,
			   float cRobust)
{
    nUpLevels_ = nUpLevels;
    nDnLevels_ = nDnLevels;
    nKeyPoints_ = nKeyPoints;
    cRobust_ = cRobust;
}

void MOPSFeatures::getFeatures(const Mat& image, vector<KeyPoint>& keypoints, 
			       Mat& desc)
{
    if (image.empty())
	return;
    Mat tmp;
    tmp = image;

    vector<ImgPyr> imgPyr;
    getImagePyrimid(tmp,imgPyr);
    MOPSDetector detector(cRobust_,nKeyPoints_);
    detector.detect(imgPyr,keypoints);
    MOPSExtractor extract;
    extract.compute(imgPyr,keypoints,desc);
    Mat outImg = image.clone();
    //drawMOPSKeypoints(image,keypoints,imgPyr,outImg);
    //imshow("Image Keypoints",outImg);
}

void MOPSFeatures::getImagePyrimid(Mat& image, vector<ImgPyr>& imgPyr)
{
    Mat im = image;
    imgPyr.push_back(ImgPyr(image,0));

    float adj = 2;
    for (int i = 1; i <= nUpLevels_; i++) {
	Mat dst;
	pyrUp(image,dst);
	imgPyr.push_back(ImgPyr(dst,(float)1/adj));
	image = dst;
	adj *= 2;
    }

    image = im;
    adj = 2;
    for (int i = 1; i <= nDnLevels_; i++) {
	Mat dst;
	pyrDown(image,dst);
	imgPyr.push_back(ImgPyr(dst,adj));
	image = dst;
	adj *= 2;
    }
}

int pSize(int size, int oct)
{
    for (int i = 0; i < oct; i++)
	size *= 2;
    return size;
}

void MOPSFeatures::drawMOPSKeypoints(const Mat& image, 
				     vector<KeyPoint>& keypoints,
				     vector<ImgPyr>& imgPyr,
				     Mat& outImg)
{
    for (int i = 0; i < keypoints.size(); i++) {
	double theta = keypoints[i].angle;
	int x = keypoints[i].pt.x;
	int y = keypoints[i].pt.y;
	float adj = imgPyr[keypoints[i].octave].adj_;

	Size2f size(DESCRIPTORSIZE,DESCRIPTORSIZE);
	if (adj != 0) {
	    size.height *= adj;
	    size.width *= adj;
	}
	RotatedRect roi(Point(x,y),size,theta);
	Point2f vert[4];
	roi.points(vert);
	bool inRange = true;
	for (int i = 0; i < 4; i++) {
	    if (vert[i].x < 0 || vert[i].y < 0)
		inRange = false;
	    if (vert[i].x >= image.cols || vert[i].y >= image.rows)
		inRange = false;
	}
	//assert(inRange);
	if (!inRange)
	    continue;

	line(outImg,vert[0],vert[1],Scalar(255,255,255));
	line(outImg,vert[1],vert[2],Scalar(255,255,255));
	line(outImg,vert[2],vert[3],Scalar(255,255,255));
	line(outImg,vert[3],vert[0],Scalar(255,255,255));
	Point2f mid((vert[0].x + vert[1].x)/2,(vert[0].y + vert[1].y)/2);
	line(outImg,roi.center,mid,Scalar(255,255,255));
    }
}
