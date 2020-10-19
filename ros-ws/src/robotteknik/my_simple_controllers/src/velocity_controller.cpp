/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include<my_simple_controllers/velocity_controller.h>
#include <pluginlib/class_list_macros.h>  // to allow the controller to be loaded as a plugin

namespace my_simple_controllers {
    

void VelocityController::update(const ros::Time& time, const ros::Duration& period) {
    
    for(int i = 0; i < this->nr_joints; i++){
        this->q(i) = this->js_handles[i].getPosition();
    }
    this->jac_solver->JntToJac(this->q, this->jac);
    //eigen_v << eigen_current.translation(), eigen_current.rotation().eulerAngles(0,1,2);
    double dt = double(period.sec)+double(period.nsec)*1e-9;
    eigen_pd = eigen_pd + pd_dot*dt;
    
    KDL::Frame fk_q;
    this->fk_solver->JntToCart(q,fk_q);
    
    Eigen::Affine3d eigen_fk;
    this->kdlToEigen(fk_q,eigen_fk);
    
    Eigen::VectorXd eigen_fk_v;
    eigen_fk_v.resize(6);
    eigen_fk_v << eigen_fk.translation(), eigen_fk.rotation().eulerAngles(0,1,2);
    
    Eigen::VectorXd diff = eigen_pd - eigen_fk_v;
    
    this->control = this->p_gain*diff + pd_dot;
    this->jacDLSinv(this->jac, this->pinv, 0.2);
    
    Eigen::VectorXd q_dots;
    q_dots = this->pinv*this->control;
    
    for(int i = 0; i < this->nr_joints; i++){
//        if(q_dots[i] > 1.9)
//            q_dots[i] = 1.9;
//        
//        if(q_dots[i] < -1.9)
//            q_dots[i] = -1.9;
//        
//        q.data[i] += q_dots[i] * dt*10;
        this->js_handles[i].setCommand(q_dots[i]);
    } 
}

bool VelocityController::init(hardware_interface::VelocityJointInterface* hw, 
	      ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    controller_nh.param("/position_joint_trajectory_controller/gains/three_dof_planar_joint1/p", this->p_gain, 1.0);
    std::cout << "Gain: " << this->p_gain << std::endl;
    this->cmd_sub = root_nh.subscribe("command",10, &VelocityController::cmd_callback, this);
    
    urdf::Model m;
    m.initParam("robot_description");
    if(!kdl_parser::treeFromUrdfModel(m, this->tree))
        ROS_ERROR("Failed to construct tree");
    this->tree.getChain("three_dof_planar_link0", "three_dof_planar_eef", this->chain);
    
    this->jac_solver.reset(new KDL::ChainJntToJacSolver(this->chain));
    this->fk_solver.reset(new KDL::ChainFkSolverPos_recursive(this->chain));
    
    this->names = hw->getNames();
    this->nr_joints = this->names.size();
    this->q.resize(this->nr_joints);
    
    for(int i = 0; i < this->nr_joints; i++){
        this->js_handles.push_back(hw->getHandle(this->names[i]) );
        this->q(i) = this->js_handles[i].getPosition();
    }
    
    this->fk_solver->JntToCart(this->q, this->fk_pose);
    this->kdlToEigen(this->fk_pose, this->eigen_current);
    eigen_pd.resize(6);
    eigen_pd << eigen_current.translation(), eigen_current.rotation().eulerAngles(0,1,2);
    this->pd_dot.resize(6);
    this->pd_dot.Zero(6);
    this->jac.resize(this->nr_joints);
    ROS_INFO("Initated velocity controller");
    return true;
}

void VelocityController::cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd_msg){
    this->pd_dot(0) = cmd_msg->linear.x;
    this->pd_dot(1) = cmd_msg->linear.y;
    this->pd_dot(2) = cmd_msg->linear.z;
    this->pd_dot(3) = cmd_msg->angular.x;
    this->pd_dot(4) = cmd_msg->angular.y;
    this->pd_dot(5) = cmd_msg->angular.z;
    
    this->fk_solver->JntToCart(this->q, this->fk_pose);
}

void VelocityController::jacDLSinv(KDL::Jacobian j, Eigen::MatrixXd &j_pinv, double k_factor){
    j_pinv = j.data*j.data.transpose();
    j_pinv += k_factor * k_factor * Eigen::MatrixXd::Identity(j_pinv.rows(), j_pinv.cols()); //j_pinv.Identity(j_pinv.rows(), j_pinv.cols());
    j_pinv = j_pinv.inverse();
    j_pinv = j.data.transpose() * j_pinv;
}

void VelocityController::kdlToEigen(KDL::Frame& f, Eigen::Affine3d& e){
    tf::Transform f_tf;
    this->kdlToTF(f, f_tf);
    tf::transformTFToEigen(f_tf, e);
}

void VelocityController::kdlToTF(KDL::Frame &f, tf::Transform &t){
    geometry_msgs::Transform t_msg;
    tf::transformKDLToMsg(f, t_msg);
    tf::transformMsgToTF(t_msg, t);
}

void VelocityController::kdlToPose(KDL::Frame f, KDL::JntArray& pose){
    pose.resize(6);
    pose(0) = f.p(0);
    pose(1) = f.p(1);
    pose(2) = f.p(2);
    
    f.M.GetRPY(pose(3),pose(4),pose(5));
}


void VelocityController::publishTransform(tf::Transform t, std::string frame_id, std::string child_frame){
    this->tf_broadcaster.sendTransform(tf::StampedTransform(t, ros::Time::now(), frame_id, child_frame));
}

}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(my_simple_controllers::VelocityController,
                       controller_interface::ControllerBase)
