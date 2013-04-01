#include "ardrone_helper.h"
#include "ros/ros.h"
#include <boost/thread.hpp>
using namespace cv;
ardrone_helper *arh;
void something();
int main(int argc, char** argv) { 
    ros::init(argc, argv, "ardrone_flight"); //This must be done before the class can be defined.
    arh=new ardrone_helper(argc, argv);
    boost::thread worker(something);
    cout << "Sleeping" << endl;
    ros::spin();
    return 0;
}
void something(){
    // Does commands on it's own thread, good for scripting movements. 
}
