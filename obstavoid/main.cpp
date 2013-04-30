#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "obstavoid.h"

int main(int argc, char** argv) {
    if (argc < 3) {
	printf("Need two images to test.\n");
	return 1;
    }
    cv::Mat img1 = cv::imread(argv[1]);
    cv::Mat img2 = cv::imread(argv[2]);
    match_pipeline(img1, img2);
    return 0;
}
