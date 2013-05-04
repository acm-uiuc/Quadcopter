#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <stdio.h>
#include <vector>

typedef struct {
    cv::Mat img1;
    cv::Mat img2;
} img2_t;

// Threshold for a match to be considered "good".
#define SIFT_DISTANCE_THRESH 20.0
#define MOPS_DISTANCE_THRESH 20.0

struct HomographyDecomposition {
    cv::Mat Rtmp;
    cv::Mat Ttmp;
    cv::Mat n;
    cv::Mat R;
    cv::Mat T;
    float d;
    int score;
};

void match_pipeline(cv::Mat img1, cv::Mat img2);
void* sift_match(void* imgs);
void* mops_match(void* imgs);
std::vector<cv::Point3f> reconstruct_3d(cv::Mat camera_proj, std::vector<cv::DMatch> matches, std::vector<cv::KeyPoint> kp1, std::vector<cv::KeyPoint> kp2);
HomographyDecomposition decompose_homography(cv::Mat, std::vector<std::pair<cv::Point2f, cv::Point2f> > inliers);
std::vector<std::pair<cv::Point2f, cv::Point2f> > homography_inliers(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2, cv::Mat mask);
cv::Point3f triangulate(cv::Mat P, cv::Point2f pt1, cv::Point2f pt2);
