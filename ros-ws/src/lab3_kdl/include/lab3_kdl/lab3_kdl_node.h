#include <stdio.h>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include  <lab3_kdl/lab3ik.h> //My service
#ifndef LAB3_KDL_NODE_H
#define LAB3_KDL_NODE_H


typedef struct myJointLimit{
    double upper, lower;
}myJointLimit;

class Lab3Node {
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_srv;
    ros::Subscriber js_sub;
    ros::Publisher js_pub;
    ros::ServiceServer ik_service;
    tf::TransformBroadcaster tf_broadcaster;
    
    sensor_msgs::JointState current_js;
    
    KDL::Tree tree;
    KDL::Chain chain;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> FKSolver_handle;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jacSolver_handle;
    
    std::map<std::string, myJointLimit> jointLimits;
    
    KDL::JntArray q_pos;
    KDL::JntArrayVel q_dot;
    std::vector<std::string> joint_names; //remove?

    KDL::Jacobian jac;
    
    KDL::Frame frame_ee;
    tf::Transform tf_ee;
public:
    void buildTree(std::string urdfParam);
    void js_callback(const sensor_msgs::JointState::ConstPtr &js_msg );
    bool ik_service_callback(lab3_kdl::lab3ikRequest &req, lab3_kdl::lab3ikResponse &res); /* Take what parameters?*/
    void kdlToTF(KDL::Frame &f, tf::Transform &t);
    void kdlToEigen(KDL::Frame &f, Eigen::Affine3d &e);
    void msgToEigen(const geometry_msgs::Pose pose, Eigen::Affine3d &e );
    void publishTransform(tf::Transform t, std::string frame, std::string child);
    void publishJointState();
    double doubleRand(double a, double b);
    void initSolvers();
    void jacDLSinv(KDL::Jacobian j, Eigen::MatrixXd &j_pinv, double k_factor);
    void jacMPinv(KDL::Jacobian j, Eigen::MatrixXd &j_pinv);
    void checkJointLimits(KDL::JntArray &ja);
    
    void guessJoints(KDL::JntArray &ja);
    int findSegmentNr(const std::string frame_id);
    
    Lab3Node();
    Lab3Node(std::string urdfParam);
};

#endif /*LAB3_KDL_NODE_H */