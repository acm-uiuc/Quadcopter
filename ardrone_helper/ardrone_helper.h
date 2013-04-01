// I used the following command to create the ROS package, add libraries as necessary
// roscreate-pkg PACKAGE_NAME std_msgs geometry_msgs rospy roscpp ardrone_autonomy opencv2 cv_bridge image_transport 
// This has only been tested in Groovy on Ubuntu. 
// Please send any bugs you find to dberli2@illinois.edu
//
#ifndef _ARDRONE_HELPER_H
#define _ARDRONE_HELPER_H
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv_bridge;

class ardrone_helper{
    float x,y,z,ax,ay,az;
    bool forced;    
    std_msgs::Empty empty_msg;
    ros::NodeHandle n;
    geometry_msgs::Twist vel;
    ros::Publisher ptakeoff, pland, pfly, preset;
    image_transport::Subscriber pfrontcam;
    image_transport::ImageTransport it;


    public:
    void stop_forward(), stop_strafe();
    void forward(float val), strafe_left(float val), forward_rel(float val), strafe_left_rel(float val);
    void altitude(float val), altitude_rel(float vel);
    void turn(float val);

    void force_stop(),force_reset(), force_land(), stop(), land(), sleep(float seconds);
    void forced_stop();
    void takeoff();

    ardrone_helper(int argc, char **argv);
};

// TODO: Factor this into a nicer funtion that doesn't draw to a hardcoded window
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr= toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::imshow("view", cv_ptr->image);
}

ardrone_helper::ardrone_helper(int argc, char **argv):it(n){
    x=y=z=ay=ax=az=0;
    forced=false;
    pfrontcam = it.subscribe("ardrone/front/image_raw", 1, imageCallback);
    ptakeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 100, true);
    pland = n.advertise<std_msgs::Empty>("/ardrone/land", 100, true);
    preset = n.advertise<std_msgs::Empty>("/ardrone/reset", 100, true);
    pfly = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000, true);
    cvNamedWindow("view",CV_WINDOW_AUTOSIZE);
    cvStartWindowThread(); 
}

/**
 * Commands the drone to move forward between 1 (max forward) and -1 (max backwards)
 * Will ignore commands after a force has been called.
 *
 * @param The magnitude for the forward vector, caps values larger than 1 or smaller than -1
 **/
void ardrone_helper::forward(float val){
    if(forced)
        return;
    if(val > 1)
        val=1;
    if(val < -1)
        val=-1;
    x=val;
    vel.linear.x = val;
    vel.linear.y = 0.0; 
    vel.linear.z = 0.0; 
    pfly.publish(vel);
    
}


/**
 * Commands the drone to move forward between 1 (max forward) and -1 (max backwards)
 * Will ignore commands after a force has been called.
 *
 * @param The magnitude change of the vector
 **/
void ardrone_helper::forward_rel(float val){
    if(forced)
        return;
    forward(val+x);
}


/**
 * Commands the drone to move strafe left between 1 (max left) and -1 (max right)
 * Will be overriden by strafe_right
 * Will ignore commands after a force has been called.
 *
 * @param The magnitude for the forward vector, ignores values outside of 1 and -1
 **/
void ardrone_helper::strafe_left(float val){
    if(forced)
        return;

    if(val > 1)
        val=1;
    if(val < -1)
        val=-1;
    
    y=val;
    vel.linear.x = 0.0;
    vel.linear.y = val; 
    vel.linear.z = 0.0; 
    pfly.publish(vel);
}
/**
 * Commands the drone to chagen it's strafe value by the amoun specified
 * Will be overriden by strafe_right
 * Will ignore commands after a force has been called.
 *
 * @param The magnitude for the change in strafe
 **/
void ardrone_helper::strafe_left_rel(float val){
    if(forced)
        return;
    strafe_left(val+y);
}

void ardrone_helper::strafe_right(float val){
    strafe_left(-val);
}

void ardrone_helper::strafe_right_rel(float val){
    strafe_left_rel(-val);
}

/**
 * Commands the drone to set its altitude value to val (note that val specifies
 *    the *change* in altitude over time, not the altitude)
 * Will ignore commands after a force has been called.
 *
 * @param The magnitude for the altitude vector
 **/
void ardrone_helper::altitude(float val){
    if(forced)
        return;
    if(val > 1)
        val=1;
    if(val < -1)
        val=-1;
    z=val;
    vel.linear.x = 0.0;
    vel.linear.y = 0.0; 
    vel.linear.z = val; 
    pfly.publish(vel);

}


/**
 * Commands the drone to change its altitude vector by val
 * Will ignore commands after a force has been called.
 *
 * @param The magnitude for the altitude vector, ignores values outside of 1 and -1
 **/
void ardrone_helper::altitude_rel(float val){
    if(forced)
        return;
    if(val > 1)
        val=1;
    if(val < -1)
        val=-1;
    altitude(val+z);
}

/**
 * Commands the drone to set its turn vector to val (- is left and + is right, values between -1 and 1)
 * Will be overriden by strafe_right
 * Will ignore commands after a force has been called.
 *
 * @param The magnitude for the forward vector, ignores values outside of 1 and -1
 **/
void ardrone_helper::turn(float val){
    if(forced)
        return;
    if(val > 1)
        val=1;
    if(val < -1)
        val=-1;

    vel.angular.z=val;
    az=vel.angular.z;
    pfly.publish(vel);
}

/**
 * Commands the drone to land
 * Will ignore commands after a force has been called
 **/
void ardrone_helper::land(){
    if(forced)
        return;
    pland.publish(empty_msg);
}


/**
 * Commands the drone to stop all motion
 * Will ignore commands after a force has been called.
 **/
void ardrone_helper::stop(){
    if(forced)
        return;
    x=0;y=0;x=0;
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.anglular.z=0;
    pfly.publish(vel); 
}
/**
 * Commands the drone to stop and tells the script to ignore all further commands other than force_ prefixed functions
 * Drone will no longer respond to non-force_ prefix functions untill forced_stop() is called
 **/
void ardrone_helper::force_stop(){
    stop();
    forced=true;
}
/**
 * Commands the drone to stop and land, tells script to ignore all non force_ prefixed functions
 * Drone will no longer respond to non-force_ prefix functions untill forced_stop() is called
 **/
void ardrone_helper::force_land(){
    force_stop();
    pland.publish(empty_msg);
    forced=true;
}


/**
 * Commands the drone to reset and the script to ignore all further commands other than force_ prefixed functions
 * Drone will no longer respond to non-force_ prefix functions untill force_reset() is called
 * Note that reset has consraints on the drone to the device itself may ignore commands issued by the script
 * until it has been properly set back to fly. 
 **/
void ardrone_helper::force_reset(){
    preset.publish(empty_msg);
    forced=true;
}

/**
 * Turns off the forced flag so commands will start being accepted again
 **/
void ardrone_helper::forced_stop(){
    forced=false;
}

/**
 * Tells the drone to take off. The drone does not clear it's movement vectors between flights.
 * For this reason, stop is called beforehand to prevent unplanned motions from taking place during takeoff.
 * Will ignore commands after a force command has been called
 **/
void ardrone_helper::takeoff(){
    if(forced)
        return;
    stop();
    ptakeoff.publish(empty_msg);
}
#endif
