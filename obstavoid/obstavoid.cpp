#include <pthread.h>
#include "obstavoid.h"

void match_pipeline(cv::Mat img1, cv::Mat img2) {
    pthread_t threads[2];
    pthread_attr_t attr;
    img2_t sift_imgs;
    img2_t mops_imgs;

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

void* sift_match(void* imgs) {
    printf("SIFT hello!\n");
    return NULL;
}

void* mops_match(void* imgs) {
    printf("MOPS hello!\n");
    return NULL;
}
