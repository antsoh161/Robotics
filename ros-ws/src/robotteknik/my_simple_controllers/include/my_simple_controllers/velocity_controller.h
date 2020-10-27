/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   velocity_controller.h
 * Author: anton
 *
 * Created on October 16, 2020, 1:02 PM
 */

#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen3/Eigen/Core>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>
#include <urdf/model.h>
#include <tf/transform_broadcaster.h>


namespace my_simple_controllers {

    class VelocityController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    private:
        ros::Subscriber cmd_sub;
        tf::TransformBroadcaster tf_broadcaster;
        
        KDL::Tree tree;
        KDL::Chain chain;
        KDL::JntArray q;
        KDL::Jacobian jac;
        KDL::Frame fk_pose;
        
        Eigen::Affine3d eigen_current;
        Eigen::VectorXd eigen_pd;
        Eigen::VectorXd pd_dot;
        Eigen::VectorXd control;
        
        Eigen::MatrixXd pinv;
        
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver;
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
        
        std::vector<hardware_interface::JointHandle> js_handles;
        std::vector<std::string> names;
        int nr_joints;
        double p_gain;        

        //boost::scoped_array<double> q;
        boost::scoped_array<double> q_dot;
    public:
        //VelocityController();
        //method executed by controller manager on every loop when controller is running
        //NOTE: MUST BE implemented
        void publishTransform(tf::Transform t, std::string frame_id, std::string child_frame);
        void kdlToEigen(KDL::Frame& f, Eigen::Affine3d& e);
        void kdlToTF(KDL::Frame &f, tf::Transform &t);
        void kdlToPose(KDL::Frame f, KDL::JntArray &pose);
        void jacDLSinv(KDL::Jacobian j, Eigen::MatrixXd &j_pinv, double k_factor);
        virtual void update(const ros::Time& time, const ros::Duration& period);

        //initialize controller with access to hardware interface and node handles
        virtual bool init(hardware_interface::VelocityJointInterface* hw, 
                          ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
        void cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
  };

};



#endif /* VELOCITY_CONTROLLER_H */

