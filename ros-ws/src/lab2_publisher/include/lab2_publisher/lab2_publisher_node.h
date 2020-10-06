/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   lab2_publisher_node.h
 * Author: anton
 *
 * Created on September 24, 2020, 3:07 PM
 */

#include <lab2_publisher/joint.h>
#include <lab2_publisher/link.h>
#include <lab2_publisher/chain.h>
#include <lab2_publisher/FKParser.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf/model.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#ifndef LAB2_PUBLISHER_NODE_H
#define LAB2_PUBLISHER_NODE_H

class Lab2PubNode{
private:
    ros::NodeHandle nh;
    tf::TransformBroadcaster tf_broadcaster;
    ros::Timer updateTimer;
    std::vector<tf::Transform> tf_transforms;
    std::vector<HomTransform> homtra_transforms;
public:
    Lab2PubNode(){};
    std::string getParamString(std::string param);
    void publishTransform(tf::Transform t, std::string frame_id, std::string child_frame);
    void setUpdateRate(ros::Duration dur);
    void updateCallback(const ros::TimerEvent& event);
    void getChainTransforms(std::vector<HomTransform>);
    void convertChainToTF(Chain c);
};


#endif /* LAB2_PUBLISHER_NODE_H */

