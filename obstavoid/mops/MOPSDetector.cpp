#include "MOPSDetector.h"

bool eqKP(KeyPoint one, KeyPoint two)
{
    float th = 1e-6;
    return (one.pt.x >= (two.pt.x - th) && 
	    one.pt.x <= (two.pt.x + th) &&
	    one.pt.y >= (two.pt.y - th) &&
	    one.pt.y <= (two.pt.y + th) &&
	    one.angle >= (two.angle - th) &&
	    one.angle <= (two.angle + th));
}

bool ltKP(KeyPoint one, KeyPoint two)
{
    if (eqKP(one,two))
	return one.response < two.response;
    return one.pt.x < two.pt.x;
}

void removeDuplicates(vector<KeyPoint>& keypoints)
{
    sort(keypoints.begin(), keypoints.end(),ltKP);
    int nKP = keypoints.size();

    for (vector<KeyPoint>::iterator start = keypoints.begin()+1,
	     end = keypoints.end(); start != end; start++) {
	if (eqKP(*start,*(start-1))) {
	    keypoints.erase(start);
	    end = keypoints.end();
	    cout << "REMOVED!" << endl;
	}
    }
}

MOPSDetector::MOPSDetector(float cRobust, int nKeyPoints) 
  : anms(cRobust,nKeyPoints)
{  
}

Point2f getPoint(int x, int y, int level)
{
    int fx = x,fy = y;
    for (int i = 0; i < level; i++) {
	fx *= 2;
	fy *= 2;
    }
	
    return Point2f(fx,fy);
}

double getOrientation(const Mat& gradX, const Mat& gradY, int x,int y)
{
    double gx = gradX.at<short>(y,x);
    double gy = gradY.at<short>(y,x);

    if (gx == 0 && gy == 0)
	return 0;
    
    double mag = sqrt(pow(gx,2) + pow(gy,2));
    double cosTheta = gx/mag;
    double sinTheta = gy/mag;
    return atan2(sinTheta,cosTheta);
}

bool isWithinBounds(const Mat& image, int x, int y, double theta)
{
    Size2f size(DESCRIPTORSIZE,DESCRIPTORSIZE);
    RotatedRect roi(Point(x,y),size,theta*180/M_PI);
    Point2f vert[4];
    roi.points(vert);

    for (int i = 0; i < 4; i++) {
	if (vert[i].x < 0 || vert[i].y < 0)
	    return false;
	if (vert[i].x >= image.cols || vert[i].y >= image.rows)
	    return false;
    }
    
    return true;
}

void MOPSDetector::detect(vector<ImgPyr>& imgPyr, vector<KeyPoint>& keypoints)
{
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    for (int i = 0; i < imgPyr.size(); i++) {
	int blockSize = 2;
	int apertureSize = 3;
	double k = 0.04;
	int thresh = 93;
	Mat dst_h, dst_norm, dst_norm_scaled, filtered;

	/// Detecting corners
	cornerHarris(imgPyr[i].img_, dst_h, blockSize, apertureSize, k, BORDER_DEFAULT );

	/// Normalizing
	normalize(dst_h,dst_norm,0,255,NORM_MINMAX,CV_32FC1,Mat());
	convertScaleAbs( dst_norm, dst_norm_scaled );
	GaussianBlur(imgPyr[i].img_,filtered,Size(5,5),4.5);
	Mat grad_x, grad_y;

	Sobel(filtered,grad_x,ddepth,1,0,3,scale,delta,BORDER_DEFAULT);
	Sobel(filtered,grad_y,ddepth,0,1,3,scale,delta,BORDER_DEFAULT);

	/// Drawing a circle around corners
	for (int y = 0; y < dst_norm.rows; y++) { 
	    for (int x = 0; x < dst_norm.cols; x++) {
		float response = dst_norm.at<float>(y,x);
		if ((int) response > thresh) {
		     double theta = getOrientation(grad_x,grad_y,x,y);
		     if (isWithinBounds(imgPyr[i].img_,x,y,theta)) {
			Point2f cntr(x,y);
			if (imgPyr[i].adj_ != 0) {
			    cntr.x *= imgPyr[i].adj_;
			    cntr.y *= imgPyr[i].adj_;
			}
			keypoints.push_back(KeyPoint(cntr,i,theta*180/M_PI,
						     response,i));
		     }
		    
		}
	    }
	}
    }

    removeDuplicates(keypoints);
    anms.doANMS(keypoints);
}
