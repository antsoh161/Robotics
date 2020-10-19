/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   lab4_trajectory.h
 * Author: anton
 *
 * Created on October 18, 2020, 3:58 PM
 */

#ifndef LAB4_TRAJECTORY_H
#define LAB4_TRAJECTORY_H

#include <ros/ros.h>
#include <tf_conversions/tf_kdl.h>
#include <boost/smart_ptr.hpp>
#include <lab4_trajectory/lab4_traj.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>

class TrajectoryNode{
private:
    ros::NodeHandle nh;
    ros::ServiceServer traj_srv;
    ros::Publisher command_pub;
    ros::Subscriber js_sub;
    
    KDL::Tree tree;
    KDL::Chain chain;
    
    
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    
    KDL::JntArray q;
    KDL::JntArray q_dot;
    
    geometry_msgs::Twist current_command;
    
    
    

public:
    TrajectoryNode(std::string urdfParam);
    bool traj_srv_callback(lab4_trajectory::lab4_trajRequest &req, lab4_trajectory::lab4_trajResponse &res);
    void js_callback(const sensor_msgs::JointState::ConstPtr& js_msg);
    double trapezoidal_step(double t, double tf, double qi, double qf, double q_cruise);
    KDL::JntArray rectilinear_step(KDL::JntArray initial, KDL::JntArray offset, double arc_length, double length);
    void buildTree(std::string urdfParam);
    void kdlToPose(KDL::Frame &f, geometry_msgs::Pose &pose);
    void kdlToTF(KDL::Frame &f, tf::Transform &t);
    void kdlToJnts(KDL::Frame &f, KDL::JntArray &ja);
    void poseToJntArray(geometry_msgs::Pose &p, KDL::JntArray &ja);
    
};


#endif /* LAB4_TRAJECTORY_H */

