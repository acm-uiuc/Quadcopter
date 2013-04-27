#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>

typedef struct {
    cv::Mat img1;
    cv::Mat img2;
} img2_t;

void match_pipeline(cv::Mat img1, cv::Mat img2);
void* sift_match(void* imgs);
void* mops_match(void* imgs);
