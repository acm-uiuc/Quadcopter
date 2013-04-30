#include "MOPSFeatures.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"

bool getDescriptors(string fileName, vector<KeyPoint>& keypoints, Mat& desc)
{
    cout << "Loading image: " << fileName << endl;

    Mat image = imread(fileName);
    if (image.empty()) {
	cerr << "Not an image, please give image filename" << endl;
	return false;
    }

    Mat image_gray;
    cvtColor(image,image_gray,CV_BGR2GRAY);
    
    int nUpLevels = 0, nDnLevels = 1;
    MOPSFeatures feats(nUpLevels,nDnLevels);
    feats.getFeatures(image_gray,keypoints,desc);

    Mat outImg = image.clone(), outImg2;
    feats.drawMOPSKeypoints(image,keypoints,outImg);
    drawKeypoints(image,keypoints,outImg2);

    imshow("Image Keypoints",outImg);
    imshow("Image Keypoints2",outImg2);
    //imwrite("mops.jpg",outImg);
    waitKey(0);
    return true;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
	cerr << "Must supply image filename" << endl;
	return -1;
    }

    string fileName1(argv[1]);
    Mat desc1, desc2;
    vector<KeyPoint> keypoints1, keypoints2;
    if (!getDescriptors(fileName1,keypoints1,desc1))
	return -1;
    if (!getDescriptors(fileName1,keypoints2,desc2))
	return -1;
    
    BFMatcher matcher(NORM_L2);
    vector<DMatch> matches;
    matcher.match(desc1,desc2,matches);

    // drawing the results
    namedWindow("matches", 1);
    Mat img_matches;
    Mat img1 = imread(fileName1);
    Mat img2 = imread(fileName1);
    drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches);
    imshow("matches", img_matches);
    waitKey(0);
    return 0;
}
