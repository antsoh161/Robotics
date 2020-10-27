/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   nullspace_controller.h
 * Author: anton
 *
 * Created on October 20, 2020, 5:25 PM
 */

#ifndef NULLSPACE_CONTROLLER_H
#define NULLSPACE_CONTROLLER_H

#include <random>
#include <ros/ros.h>
#include <random>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen3/Eigen/Core>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarrayacc.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <my_simple_controllers/chainjnttojacdotsolver.h>

#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>
#include <urdf/model.h>
#include <tf/transform_broadcaster.h>

typedef struct JointLimit{
    double upper;
    double lower;
    JointLimit();
    JointLimit(double _upper, double _lower) : upper(_upper), lower(_lower){}
    
}JointLimit;

namespace my_simple_controllers {

    class NullSpaceController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    private:
        int nr_joints;
        int nr_segments;
        int every_100th;
        double dt;
        std::vector<std::string> joint_names;
        ros::Subscriber cmd_sub;
        
        
        KDL::Tree tree;
        KDL::Chain chain;
        KDL::JntArrayVel qs;
        KDL::JntArray random_null;
        KDL::Jacobian jac;
        KDL::Jacobian jacdot;
        KDL::JntSpaceInertiaMatrix kdl_massMatrix;
        KDL::JntArray gravity;
        Eigen::VectorXd prev_gravity;
        KDL::JntArray coriolis;
        KDL::Frame frame_now;
        
        tf::Transform tf_desired;
        tf::Transform tf_desired3D;
        tf::TransformBroadcaster tf_broadcaster;
        
        std::vector<JointLimit> joint_limits;
        std::vector<double> torque_limits;
        
        std::vector<Eigen::VectorXd> null_signals;
        
        Eigen::MatrixXd massMatrix;
        Eigen::MatrixXd mass_xee;
        Eigen::MatrixXd j_pinv;
        
        Eigen::VectorXd torque_out;
        Eigen::VectorXd torque_null;
        Eigen::VectorXd joint_origins;
        
        Eigen::VectorXd p_gains;
        Eigen::VectorXd d_gains;
        
        /*New stuff remove all others*/
        Eigen::VectorXd x_now;
        Eigen::VectorXd x_des;
        Eigen::VectorXd xdot_now;
        Eigen::VectorXd xdot_des;
        
        Eigen::VectorXd xp_error;
        Eigen::VectorXd xo_error;
        Eigen::VectorXd xpdot_error;
        Eigen::VectorXd xodot_error;
        

        boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver;
        boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jacdot_solver;
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
        boost::scoped_ptr<KDL::ChainDynParam> cdp;
        
        boost::scoped_array<hardware_interface::JointHandle> joint_handles;
        
        
    public:
        //VelocityController();
        //method executed by controller manager on every loop when controller is running
        //NOTE: MUST BE implemented
        
        void buildTree(std::string urdfParam);
        void printJointStates();
        void jacDLSinv(KDL::Jacobian j, Eigen::MatrixXd &j_pinv, double k_factor);
        void buildMassMatrix();
        void randomPose(Eigen::VectorXd &x);
        void randomJointArray(KDL::JntArray &ja);
        
        Eigen::VectorXd computeOrientationControl();
        Eigen::VectorXd computePositionControl();
        Eigen::VectorXd computeNullSpaceControl();
        void computeForceBasedControl();
        void incrementTorqueNull();
        
        void setDesiredTF();
        void publishTransform(tf::Transform t, std::string frame_id, std::string child_frame);
        
        double doubleRand(double a, double b);
        void cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
        virtual void update(const ros::Time& time, const ros::Duration& period);

        //initialize controller with access to hardware interface and node handles
        virtual bool init(hardware_interface::EffortJointInterface* hw, 
                          ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  };

};



#endif /* NULLSPACE_CONTROLLER_H */

