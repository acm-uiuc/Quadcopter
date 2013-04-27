#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>

typedef struct {
    cv::Mat img1;
    cv::Mat img2;
} img2_t;

// Threshold for a match to be considered "good".
#define SIFT_DISTANCE_THRESH 20.0

void match_pipeline(cv::Mat img1, cv::Mat img2);
void* sift_match(void* imgs);
void* mops_match(void* imgs);
