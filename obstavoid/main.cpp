#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "obstavoid.h"

int main(int argc, char** argv) {
    cv::Mat m;
    match_pipeline(m, m);
    return 0;
}
