#include "MOPSFeatures.h"

MOPSFeatures::MOPSFeatures(int nUpLevels, int nDnLevels)
{
    nUpLevels_ = nUpLevels;
    nDnLevels_ = nDnLevels;
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

    int nKeyPoints = 15;
    float cRobust = 0.9;
    MOPSDetector detector(cRobust,nKeyPoints);
    detector.detect(imgPyr,keypoints);
    MOPSExtractor extract;
    extract.compute(imgPyr,keypoints,desc);
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
				     Mat& outImg)
{
    for (int i = 0; i < keypoints.size(); i++) {
	double theta = keypoints[i].angle*M_PI/180;
	int x = keypoints[i].pt.x;
	int y = keypoints[i].pt.y;
	double cTheta = cos(theta), sTheta = sin(theta);
	int patchsize = pSize(40,keypoints[i].octave), 
	    xoff = (patchsize/2)*cTheta, 
	    yoff = (patchsize/2)*sTheta;

	Size2f size(patchsize,patchsize);
	RotatedRect roi(Point(x,y),size,theta*180/M_PI);
	Point2f vert[4];
	roi.points(vert);
	for (int i = 0; i < 4; i++) {
	    if (vert[i].x < 0 || vert[i].y < 0)
		return;
	    if (vert[i].x >= image.cols || vert[i].y >= image.rows)
		return;
	}

	line(outImg,vert[0],vert[1],Scalar(255,255,255));
	line(outImg,vert[1],vert[2],Scalar(255,255,255));
	line(outImg,vert[2],vert[3],Scalar(255,255,255));
	line(outImg,vert[3],vert[0],Scalar(255,255,255));
	Point2f mid((vert[0].x + vert[1].x)/2,(vert[0].y + vert[1].y)/2);
	line(outImg,roi.center,mid,Scalar(255,255,255));
    }
}
