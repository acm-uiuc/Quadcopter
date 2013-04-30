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

void match_pipeline(cv::Mat img1, cv::Mat img2);
void* sift_match(void* imgs);
void* mops_match(void* imgs);
cv::Mat reconstruct_3d(cv::Mat camera_proj, std::vector<cv::DMatch> matches, std::vector<cv::KeyPoint> kp1, std::vector<cv::KeyPoint> kp2);
