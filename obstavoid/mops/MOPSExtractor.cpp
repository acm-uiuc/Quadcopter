#include "MOPSExtractor.h"
#include "Utils.h"

MOPSExtractor::MOPSExtractor()
{
}

Mat getDescriptorPoints(const ImgPyr& pyr, KeyPoint kp)
{
    Mat image = pyr.img_;
    float theta = kp.angle;
    int x = kp.pt.x, y = kp.pt.y;

    if (pyr.adj_ != 0) {
    	x = (float)x/pyr.adj_;
    	y = (float)y/pyr.adj_;
    }

    Size2f size(DESCRIPTORSIZE,DESCRIPTORSIZE);
    RotatedRect roi(Point(x,y),size,theta);
    Rect rect = roi.boundingRect();
    if (rect.x == -1) {
	rect.x = 0; rect.width+1;
	roi.center.x+1;
    }
    if (rect.y == -1) {
	rect.y = 0; rect.height+1;
	roi.center.y+1;
    }
    assert(isRectWithinImage(image,rect));
    Mat patch = image(rect);

    Mat M, rotated, cropped;
    float angle = roi.angle;
    Size rect_size = roi.size;

    theta = theta;
    if (theta < -45.) {
	angle += 90.0;
	swap(rect_size.width, rect_size.height);
    }

    M = getRotationMatrix2D(roi.center, angle, 1.0);
    warpAffine(image, rotated, M, image.size(), INTER_CUBIC);
    getRectSubPix(rotated, rect_size, roi.center, cropped);
    return cropped;
}

void MOPSExtractor::compute(const vector<ImgPyr>& imgPyr,
			    vector<KeyPoint>& keypoints,
			    Mat& descriptors) const
{
    int descLength = 64;
    descriptors = Mat(keypoints.size(),descLength,CV_32F);
    for (int i = 0; i < keypoints.size(); i++) {
	//cout << keypoints[i].octave << endl;
	Mat patch = getDescriptorPoints(imgPyr[keypoints[i].octave],
					keypoints[i]);
	assert(patch.rows == DESCRIPTORSIZE && patch.cols == DESCRIPTORSIZE);
	Scalar mean, std;
	meanStdDev(patch,mean,std);
	
	int dIdx = 0;
	for (int y = 0; y < patch.rows; y = y + 5) {
	    for (int x = 0; x < patch.cols; x = x + 5) {
		float pix = (float) patch.at<uchar>(y,x);
		descriptors.at<float>(i,dIdx) =  (pix - mean(0)) / std(0);
		dIdx++;
	    }
	}

	assert(dIdx == descLength);
    }

    cout << "EXTRACTING DONE" << endl;
}

