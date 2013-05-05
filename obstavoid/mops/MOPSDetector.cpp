#include "MOPSDetector.h"
#include "Utils.cpp"

bool eqKP(KeyPoint one, KeyPoint two)
{
    float th = 5e-1;
    return (one.pt.x >= (two.pt.x - th) && 
	    one.pt.x <= (two.pt.x + th) &&
	    one.pt.y >= (two.pt.y - th) &&
	    one.pt.y <= (two.pt.y + th));/* &&
	    one.angle >= (two.angle - th) &&
	    one.angle <= (two.angle + th));*/
}

bool ltKP(KeyPoint one, KeyPoint two)
{
    if (eqKP(one,two))
	return one.response < two.response;
    return one.pt.x < two.pt.x;
}

void removeDuplicates(vector<KeyPoint>& keypoints)
{
    if (keypoints.size() < 2)
	return;
    sort(keypoints.begin(), keypoints.end(),ltKP);
    int nKP = keypoints.size();

    for (vector<KeyPoint>::iterator start = keypoints.begin()+1,
	     end = keypoints.end(); start != end; start++) {
	if (eqKP(*start,*(start-1))) {
	    keypoints.erase(start);
	    end = keypoints.end();
	    //cout << "REMOVED!" << endl;
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
    return (atan2(sinTheta,cosTheta)*180/M_PI);
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

void adjustROI(const Mat& image, Rect& roi)
{
    if (roi.x < 0) {
	roi.x = 0;
	roi.width -= roi.x;
    }

    if (roi.y < 0) {
	roi.y = 0;
	roi.height -= roi.y;
    }

    int diffHeight = (roi.y + roi.height) - image.rows;
    int diffWidth = (roi.x + roi.width) - image.cols;
    if (diffHeight > 0)
	roi.height -= diffHeight;
    if (diffWidth > 0)
	roi.width -= diffWidth;
}

double getMaxResponse(vector<ImgPyr>& imgPyr, vector<Mat>& corners, 
		      int x, int y, int& octave, Point& loc)
{
    double maxResp = -1;
    int height = 5, width = 5, xoff = 2, yoff = 2;
    
    for (int i = 0; i < corners.size(); i++) {
	Rect roi(x-xoff,y-yoff,height,width);
	if (imgPyr[i].adj_ != 0) {
	    roi.x = (float)roi.x/imgPyr[i].adj_;
	    roi.y = (float)roi.y/imgPyr[i].adj_;
	    roi.height = (float)roi.height/imgPyr[i].adj_;
	    roi.width = (float)roi.width/imgPyr[i].adj_;
	}

	adjustROI(imgPyr[i].img_,roi);

	Mat img = imgPyr[i].img_(roi);
	double maxVal, minVal;
	Point minLoc, maxLoc;
	minMaxLoc(img,&minVal,&maxVal,&minLoc,&maxLoc);

	if (maxVal > maxResp) {
	    maxResp = maxVal;
	    octave = i;
	    loc = Point(roi.x + maxLoc.x, roi.y + maxLoc.y);
	}
    }

    return maxResp;
}

void MOPSDetector::detect(vector<ImgPyr>& imgPyr, vector<KeyPoint>& keypoints)
{
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    vector<Mat> corners(imgPyr.size());
    vector<Mat> grad_x(imgPyr.size());
    vector<Mat> grad_y(imgPyr.size());
    int nRows = imgPyr[0].img_.rows, nCols = imgPyr[0].img_.cols;
    double thresh = 70;
    for (int i = 0; i < imgPyr.size(); i++) {
	int blockSize = 2;
	int apertureSize = 3;
	double k = 0.04;
	
	Mat dst_h, filtered, dst_norm_scaled = imgPyr[i].img_;

	/// Detecting corners
	cornerHarris(imgPyr[i].img_,dst_h,blockSize,apertureSize,k,
		     BORDER_DEFAULT);

	/// Normalizing
	normalize(dst_h,corners[i],0,255,NORM_MINMAX,CV_32FC1,Mat());
	//convertScaleAbs( corners[i], dst_norm_scaled );
	GaussianBlur(imgPyr[i].img_,filtered,Size(5,5),4.5);
	Sobel(filtered,grad_x[i],ddepth,1,0,3,scale,delta,BORDER_DEFAULT);
	Sobel(filtered,grad_y[i],ddepth,0,1,3,scale,delta,BORDER_DEFAULT);


	for( int y = 0; y < corners[i].rows ; y++ )
	    { for( int x = 0; x < corners[i].cols; x++ )
		    {
			if( (int) corners[i].at<float>(y,x) > thresh )
			    {
				circle( dst_norm_scaled, Point( x, y ), 5,  Scalar(0), 2, 8, 0 );
				double theta = getOrientation(grad_x[i],grad_y[i],x,y);
				Size2f size(DESCRIPTORSIZE,DESCRIPTORSIZE);
				RotatedRect roi(Point(x,y),size,theta);
				if (isRectWithinImage(imgPyr[i].img_,roi.boundingRect())) {
				    float adjx = x, adjy = y;
				    if (imgPyr[i].adj_ != 0) {
					adjx = (float) x*imgPyr[i].adj_;
					adjy = (float) y*imgPyr[i].adj_;
				    }
				    keypoints.push_back(KeyPoint(Point2f(adjx,adjy),i,theta,corners[i].at<float>(y,x),i));
				}
			    }
		    }
	    }
	imshow("showyshow", dst_norm_scaled );
	waitKey(0);
    }

    assert(nCols != 0 && nRows != 0);
    
    for (int y = 0; y < nRows; y++) { 
	for (int x = 0; x < nCols; x++) {
	    int octave = -1;
	    Point loc;
	    double response = getMaxResponse(imgPyr,corners,x,y,octave,loc);
	    assert(octave >= 0);
	    if (response > thresh) {
		double theta = getOrientation(grad_x[octave],grad_y[octave],
					      loc.x,loc.y);
		Size2f size(DESCRIPTORSIZE,DESCRIPTORSIZE);
		RotatedRect roi(Point(loc.x,loc.y),size,theta);
		if (isRectWithinImage(imgPyr[octave].img_,roi.boundingRect())) {
		    Point2f cntr(loc.x,loc.y);
		    if (imgPyr[octave].adj_ != 0) {
			cntr.x = (float) cntr.x*imgPyr[octave].adj_;
			cntr.y = (float) cntr.y*imgPyr[octave].adj_;
		    }
		    //keypoints.push_back(KeyPoint(cntr,octave,theta,response,
		    //				 octave));
		}
		    
	    }
	}
    }

    removeDuplicates(keypoints);
    //anms.doANMS(keypoints);
    cout << "Exiitng Detecting" << endl;
}


/*
  void MOPSDetector::detect(vector<ImgPyr>& imgPyr, vector<KeyPoint>& keypoints)
  {
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;

  vector<Mat> corners(imgPyr.size());
  vector<Mat> grad_x(imgPyr.size());
  vector<Mat> grad_y(imgPyr.size());
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

  for (int y = 0; y < dst_norm.rows; y++) { 
  for (int x = 0; x < dst_norm.cols; x++) {
  float response = dst_norm.at<float>(x,y);
  if ((int) response > thresh) {
  double theta = getOrientation(grad_x,grad_y,x,y);
  Size2f size(DESCRIPTORSIZE,DESCRIPTORSIZE);
  RotatedRect roi(Point(x,y),size,theta*180/M_PI);
  if (isRectWithinImage(imgPyr[i].img_,roi.boundingRect())) {
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

  //removeDuplicates(keypoints);
  anms.doANMS(keypoints);
  }*/
