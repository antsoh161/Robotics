/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#ifndef LAB3_CLIENT_H
#define LAB3_CLIENT_H

#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>


class Lab3Client : Lab3Node{
private:
    ros::ServiceClient client;
    
    
};

#endif /*LAB3_CLIENT_H */