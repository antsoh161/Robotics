#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Core>

class HomtraPubNode{
private:
    ros::NodeHandle nh;
    tf::TransformBroadcaster tf_broadcaster;
    
public:
    HomtraPubNode() {}
    void publishTransform(tf::Transform t, std::string frame_id, std::string child_frame);
    
};

double doubleRand(double a, double b);