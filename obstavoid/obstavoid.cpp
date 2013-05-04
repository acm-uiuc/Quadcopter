#include <pthread.h>
#include <vector>
#include <utility>
#include <algorithm>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
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

    // Convert matrix to greyscale if needed.
    // Is there a better way to check if an image is grayscale?
    if (img1.channels() != 1) {
	cv::cvtColor(img1, img1, CV_BGR2GRAY);
    }
    if (img2.channels() != 1) {
	cv::cvtColor(img2, img2, CV_BGR2GRAY);
    }
    
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

    cv::Mat output;
    cv::drawKeypoints(imgs->img1, kp1, output);
    cv::imshow("Keypoints 1", output);
    cv::waitKey(0);
    cv::drawKeypoints(imgs->img2, kp2, output);
    cv::imshow("Keypoints 2", output);
    cv::waitKey(0);
    
    // Extract descriptors.
    extractor.compute(imgs->img1, kp1, desc1);
    extractor.compute(imgs->img2, kp2, desc2);

    // Match descriptors.
    matcher.match(desc1, desc2, matches);
    double min_dist = 100000;
    for (size_t i = 0; i < desc1.rows; ++i) {
	if (matches[i].distance < min_dist) {
	    min_dist = matches[i].distance;
	}
    }
    printf("Min dist = %f\n", min_dist);
    // Threshold to only good matches.
    for (size_t i = 0; i < desc1.rows; ++i) {
	//printf("Distance: %f\n", matches[i].distance);
	if (matches[i].distance < 4 * min_dist) {
	    good_matches.push_back(matches[i]);
	}
    }
    cv::Mat img_matches;
    cv::drawMatches(imgs->img1, kp1, imgs->img2, kp2, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imshow("Matches", img_matches);
    cv::waitKey(0);

    cv::Mat proj = (cv::Mat_<float>(3, 4) <<
		    1, 0, 0, 0,
		    0, 1, 0, 0,
		    0, 0, 1, 0);
    reconstruct_3d(proj, good_matches, kp1, kp2);

    return NULL;
}

void* mops_match(void* _imgs) {
    printf("MOPS hello!\n");
    return NULL;
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

std::vector<cv::Point3f> reconstruct_3d(cv::Mat camera_proj, std::vector<cv::DMatch> matches, std::vector<cv::KeyPoint> kp1, std::vector<cv::KeyPoint> kp2) {
    std::vector<cv::Point2f> img1_pts;
    std::vector<cv::Point2f> img2_pts;
    std::vector<cv::Point3f> pts3d;
    //cv::Mat img1_pts(2, matches.size(), CV_32F);
    //cv::Mat img2_pts(2, matches.size(), CV_32F);

    // Get the points associated with the matches.
    for (size_t i = 0; i < matches.size(); ++i) {
	img1_pts.push_back(kp1[matches[i].queryIdx].pt);
	img2_pts.push_back(kp2[matches[i].trainIdx].pt);
	/*cv::Point2f pt1 = kp1[matches[i].queryIdx].pt;
	cv::Point2f pt2 = kp2[matches[i].trainIdx].pt;
	img1_pts.at<float>(0, i) = pt1.x;
	img1_pts.at<float>(1, i) = pt1.y;
	img2_pts.at<float>(0, i) = pt2.x;
	img2_pts.at<float>(1, i) = pt2.y;*/
    }
    std::cout << "X1 = " << img1_pts << "\nX2 = " << img2_pts << "\n";

    cv::Mat mask;
    cv::Mat dH = cv::findHomography(img1_pts, img2_pts, CV_RANSAC, 3, mask);
    cv::Mat H;
    dH.convertTo(H, CV_32F);
    HomographyDecomposition Hdecomp = decompose_homography(H, homography_inliers(img1_pts, img2_pts, mask));
    std::cout << "H = " << H << "\nR = " << Hdecomp.R << "\nT = " << Hdecomp.T << "\n";
    
    // Triangulate.
    // Create projection matrix.
    cv::Mat P(3, 4, CV_32F);
    Hdecomp.R.copyTo(P(cv::Rect(0, 0, 3, 3)));
    // Rects are (col, row, width, height).
    Hdecomp.T.copyTo(P(cv::Rect(3, 0, 1, 3)));

    for (size_t i = 0; i < matches.size(); ++i) {
	pts3d.push_back(triangulate(P, img1_pts[i], img2_pts[i]));
    }
    std::cout << "Triangulations:\n" << pts3d << "\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < pts3d.size(); ++i) {
	cv::Point3f pt = pts3d[i];
	cloud->points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
    }
    pcl::visualization::PCLVisualizer vis;
    vis.addPointCloud(cloud);
    vis.spin();

    return pts3d;
}

std::vector<std::pair<cv::Point2f, cv::Point2f> > homography_inliers(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2, cv::Mat mask) {
    std::vector<std::pair<cv::Point2f, cv::Point2f> > inliers;
    for (size_t i = 0; i < pts1.size(); ++i) {
	if (mask.at<int>(i) != 0) {
	    inliers.push_back(std::make_pair(pts1[i], pts2[i]));
	}
    }
    return inliers;
}

bool operator<(const HomographyDecomposition l, const HomographyDecomposition r) {
    return l.score < r.score;
}

void printMat(cv::Mat mat) {
    std::cout << mat << "\n";
}

HomographyDecomposition decompose_homography(cv::Mat H, std::vector<std::pair<cv::Point2f, cv::Point2f> > inliers) {
    // Based on code in PTAM.
    cv::SVD svd(H);
    float d1 = fabs(svd.w.at<float>(0));
    float d2 = fabs(svd.w.at<float>(1));
    float d3 = fabs(svd.w.at<float>(2));
    // Note: May need to not transpose vt.
    cv::Mat U = svd.u;
    cv::Mat V = svd.vt.t();
    float s = cv::determinant(U) * cv::determinant(V);
    float prime_PM = d2;
    
    assert((d1 != d2) && (d2 != d3));

    float x1_PM = sqrtf((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
    float x2 = 0.0;
    float x3_PM = sqrtf((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
    float e1[4] = {1.0, -1.0, 1.0, -1.0};
    float e3[4] = {1.0, 1.0, -1.0, -1.0};

    // Compute the eight possible decompositions.
    std::vector<HomographyDecomposition> decomps;
    for (size_t sign = 0; sign < 4; ++sign) {
	float sin_theta = ((d1 - d3) * x1_PM * x3_PM * e1[sign] * e3[sign]) / d2;
	float cos_theta = (d1 * x3_PM * x3_PM + d3 * x1_PM * x1_PM) / d2;
	HomographyDecomposition decomp;
	decomp.d = s * prime_PM;
	decomp.Rtmp = (cv::Mat_<float>(3, 3) <<
		       cos_theta, 0.0, -sin_theta,
		       0.0, 1.0, 0.0,
		       sin_theta, 0.0, cos_theta);
	decomp.Ttmp = (cv::Mat_<float>(3, 1) <<
		       (d1 - d3) * x1_PM * sin_theta * e1[sign],
		       0.0,
		       (d1 - d3) * (-x3_PM) * e3[sign]);
	decomp.n = V * (cv::Mat_<float>(3, 1) <<
			x1_PM * e1[sign],
			x2,
			x3_PM * e3[sign]);
	decomp.R = s * U * decomp.Rtmp * V.t();
	decomp.T = U * decomp.Ttmp;
	decomps.push_back(decomp);
    }
    for (size_t sign = 0; sign < 4; ++sign) {
	float sin_phi = ((d1 + d3) * x1_PM * x3_PM * e1[sign] * e3[sign]) / d2;
	float cos_phi = (d3 * x1_PM * x1_PM - d1 * x3_PM * x3_PM) / d2;
	HomographyDecomposition decomp;
	decomp.d = s * (-prime_PM);
	decomp.Rtmp = (cv::Mat_<float>(3, 3) <<
		       cos_phi, 0.0, sin_phi,
		       0.0, 1.0, 0.0,
		       sin_phi, 0.0, cos_phi);
	decomp.Ttmp = (cv::Mat_<float>(3, 1) <<
		       (d1 + d3) * x1_PM * e1[sign],
		       0.0, 1.0, 0.0,
		       (d1 + d3) * x3_PM * e3[sign]);
	decomp.n = V * (cv::Mat_<float>(3, 1) <<
			x1_PM * e1[sign],
			x2,
			x3_PM * e3[sign]);
	decomp.R = s * U * decomp.Rtmp * V.t();
	decomp.T = U * decomp.Ttmp;
    }

    // Pick the best decomposition.
    // Enforce visibility constraints.
    for (size_t i = 0; i < 8; ++i) {
	HomographyDecomposition& decomp = decomps[i];
	int num_pos = 0;
	for (size_t j = 0; j < inliers.size(); ++j) {
	    cv::Point2f pt = inliers[j].first;
	    float visibility = (H.at<float>(2, 0) * pt.x + H.at<float>(2, 1) * pt.y + H.at<float>(2, 2)) / decomp.d;
	    if (visibility > 0.0) {
		num_pos++;
	    }
	}
	decomp.score = -num_pos;
    }

    std::sort(decomps.begin(), decomps.end());
    decomps.resize(4);

    // Enforce visibility constraints.
    for (size_t i = 0; i < 4; ++i) {
	HomographyDecomposition& decomp = decomps[i];
	int num_pos = 0;
	for (size_t j = 0; j < inliers.size(); ++j) {
	    cv::Point2f pt2d = inliers[j].first;
	    cv::Mat pt = (Mat_<float>(3, 1) <<
			  pt2d.x,
			  pt2d.y,
			  1.0);
	    float visibility = pt.dot(decomp.n) / decomp.d;
	    if (visibility > 0.0) {
		num_pos++;
	    }
	}
	decomp.score = -num_pos;
    }

    std::sort(decomps.begin(), decomps.end());
    decomps.resize(2);

    // Check if there is an ambiguity.
    float ratio = ((float) decomps[1].score) / ((float) decomps[0].score);
    if (ratio < 0.9) {
	return decomps[0];
    }
    else {
	printf("Not implemented: distinguishing between homography ambiguity.\n");
	printf("Returning first homography.\n");
	//assert(0);
    }
    return decomps[0];
}

cv::Point3f triangulate(cv::Mat P, cv::Point2f pt1, cv::Point2f pt2) {
    cv::Mat A(4, 4, CV_32F);
    A.at<float>(0, 0) = -1.0;
    A.at<float>(0, 1) = 0.0;
    A.at<float>(0, 2) = pt2.x;
    A.at<float>(0, 3) = 0.0;
    A.at<float>(1, 0) = 0.0;
    A.at<float>(1, 1) = -1.0;
    A.at<float>(1, 2) = pt2.y;
    A.at<float>(1, 3) = 0.0;
    A.row(2) = pt2.x * P.row(2) - P.row(0);
    A.row(3) = pt2.y * P.row(2) - P.row(1);
    
    cv::SVD svd(A);
    cv::Mat V = svd.vt.t();
    cv::Mat smallest = V.col(3);
    float w = smallest.at<float>(3);
    return cv::Point3f(smallest.at<float>(0) / w,
		       smallest.at<float>(1) / w,
		       smallest.at<float>(2) / w);
}
