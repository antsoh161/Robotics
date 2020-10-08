#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#ifndef LAB3_KDL_NODE_H
#define LAB3_KDL_NODE_H


class Lab3Node {
private:
    ros::NodeHandle nh;
    ros::Subscriber js_sub;
    ros::ServiceServer ik_service;
    tf::TransformBroadcaster tf_broadcaster;
    
    
    KDL::Tree tree;
    KDL::Chain chain;
    /*Make boost::shared_ptr instead later*/
    KDL::ChainFkSolverPos_recursive *FKSolver_handle;
    KDL::ChainJntToJacSolver *jacSolver_handle;
    
    KDL::JntArray q_pos;
    KDL::JntArrayVel q_dot;
    KDL::Jacobian jac;
    Eigen::Matrix<double,6,Eigen::Dynamic> jac_matrix;
    
    KDL::Frame frame_ee;
    tf::Transform tf_ee;
public:
    void buildTree(std::string urdfParam);
    void js_callback(const sensor_msgs::JointState::ConstPtr &js_msg );
    void ik_service();
    void kdlToTF(KDL::Frame &f, tf::Transform &t);
    void publishTransform(tf::Transform t, std::string frame, std::string child);
    void initSolvers();
    Lab3Node();
    /*rule of 3 TODO so dont copy any nodes*/
    virtual ~Lab3Node();
    
};

#endif /*LAB3_KDL_NODE_H */