#include <pthread.h>
#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "obstavoid.h"
#include "MOPSFeatures.h"

void match_pipeline(cv::Mat img1, cv::Mat img2) {
    pthread_t threads[2];
    pthread_attr_t attr;
    img2_t sift_imgs;
    img2_t mops_imgs;

    // Ensure we have good input.
    if (!img1.total() || !img2.total()) {
	printf("Need non-empty images!\n");
	return;
    }

    // Convert matrix to greyscale.
    cv::cvtColor(img1, img1, CV_BGR2GRAY);
    cv::cvtColor(img2, img2, CV_BGR2GRAY);
    
    sift_imgs.img1 = img1;
    sift_imgs.img2 = img2;
    // Clone matrix so there's no dependencies.
    // We may just be able to mark it const?
    mops_imgs.img1 = img1.clone();
    mops_imgs.img2 = img2.clone();
    
    // Ensure threads are joinable.
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    // Spawn threads.
    int rc;
    rc = pthread_create(&threads[0], &attr, sift_match, (void*) &sift_imgs);
    if (rc) {
	printf("Error creating SIFT thread.\n");
	return;
    }
    rc = pthread_create(&threads[1], &attr, mops_match, (void*) &mops_imgs);
    if (rc) {
	printf("Error creating MOPS thread.\n");
	return;
    }

    pthread_attr_destroy(&attr);

    // Wait for threads to finish.
    void* status;
    rc = pthread_join(threads[0], &status);
    if (rc) {
	printf("Error joining SIFT thread.\n");
	return;
    }
    rc = pthread_join(threads[1], &status);
    if (rc) {
	printf("Error joining MOPS thread.\n");
	return;
    }

    pthread_exit(NULL);
}

void* sift_match(void* _imgs) {
    printf("SIFT hello!\n");
    img2_t* imgs = (img2_t*) _imgs;
    cv::SiftFeatureDetector detector;
    cv::SiftDescriptorExtractor extractor;
    cv::FlannBasedMatcher matcher;
    std::vector<cv::KeyPoint> kp1;
    std::vector<cv::KeyPoint> kp2;
    cv::Mat desc1;
    cv::Mat desc2;
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> good_matches;

    // Note: If still too slow, we can thread these.
    // Detect keypoints.
    detector.detect(imgs->img1, kp1);
    detector.detect(imgs->img2, kp2);
    
    // Extract descriptors.
    extractor.compute(imgs->img1, kp1, desc1);
    extractor.compute(imgs->img2, kp2, desc2);

    // Match descriptors.
    matcher.match(desc1, desc2, matches);
    // Threshold to only good matches.
    for (size_t i = 0; i < desc1.rows; ++i) {
	if (matches[i].distance < SIFT_DISTANCE_THRESH) {
	    good_matches.push_back(matches[i]);
	}
    }
    return NULL;
}

void* mops_match(void* _imgs) {
    printf("MOPS hello!\n");
    img2_t* imgs = (img2_t*) _imgs;
    int nUpLevels = 0, nDnLevels = 1;
    MOPSFeatures feats(nUpLevels,nDnLevels);
    cv::FlannBasedMatcher matcher;
    std::vector<cv::KeyPoint> kp1;
    std::vector<cv::KeyPoint> kp2;
    cv::Mat desc1;
    cv::Mat desc2;
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> good_matches;

    // Note: If still too slow, we can thread these.
    // Detect keypoints and get descriptors.
    feats.getFeatures(imgs->img1, kp1, desc1);
    feats.getFeatures(imgs->img2, kp2, desc2);

    // Match descriptors.
    matcher.match(desc1, desc2, matches);
    // Threshold to only good matches.
    for (size_t i = 0; i < desc1.rows; ++i) {
	if (matches[i].distance < MOPS_DISTANCE_THRESH) {
	    good_matches.push_back(matches[i]);
	}
    }
    return NULL;
}
