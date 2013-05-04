#include "ContourKeypoints.h"

bool getDescriptors(string fileName, vector<KeyPoint>& keypoints)
{
    cout << "Loading image: " << fileName << endl;

    Mat image = imread(fileName);
    if (image.empty()) {
	cerr << "Not an image, please give image filename" << endl;
	return false;
    }

    Mat image_gray, outImg;
    cvtColor(image,image_gray,CV_BGR2GRAY);
    int thresh = 75, stepSize = 10;
    ContourKeypoints contours(thresh,stepSize);
    contours.detect(image_gray,keypoints);
    
    drawKeypoints(image,keypoints,outImg);
    
    imshow("Image Keypoints2",outImg);
    //imwrite("mops.jpg",outImg);
    waitKey(0);
    return true;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
	cerr << "Must supply image filename" << endl;
	return -1;
    }

    string fileName1(argv[1]);
    string fileName2(argv[2]);
    Mat desc1, desc2;
    vector<KeyPoint> keypoints1, keypoints2;
    if (!getDescriptors(fileName1,keypoints1))
	return -1;
    if (!getDescriptors(fileName2,keypoints2))
	return -1;
    
    BFMatcher matcher(NORM_L2);
    vector<DMatch> matches;
    matcher.match(desc1,desc2,matches);

    // drawing the results
    namedWindow("matches", 1);
    Mat img_matches;
    Mat img1 = imread(fileName1);
    Mat img2 = imread(fileName2);
    drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches);
    imshow("matches", img_matches);
    waitKey(0);
    return 0;
}
