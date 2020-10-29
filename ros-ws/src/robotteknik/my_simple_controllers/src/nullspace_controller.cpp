/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "my_simple_controllers/nullspace_controller.h"
#include <pluginlib/class_list_macros.h>  // to allow the controller to be loaded as a plugin


namespace my_simple_controllers {
    
void NullSpaceController::update(const ros::Time& time, const ros::Duration& period) {
    
    for(int i = 0; i < nr_joints; i++){
        qs.q.data[i] = joint_handles[i].getPosition();
        qs.qdot.data[i] = joint_handles[i].getVelocity();
    }
    jac_solver->JntToJac(qs.q, jac);
    jacdot_solver->JntToJacDot(qs, jacdot);
    
    dt = double(period.sec)+double(period.nsec)*1e-9;
    
    cdp->JntToMass(qs.q, kdl_massMatrix);
    
    cdp->JntToGravity(qs.q, gravity);
    
    cdp->JntToCoriolis(qs.q, qs.qdot, coriolis);
   
    this->fk_solver->JntToCart(qs.q, frame_now );
    
    double roll, pitch, yaw;
    frame_now.M.GetRPY(roll, pitch, yaw);
    
    x_now << frame_now.p.x(), frame_now.p.y(), frame_now.p.z(), roll, pitch, yaw;
    xdot_now = jac.data * qs.qdot.data;
    
    //this->computeControl6D();
    
    //this->computeControl3D();
    
    torque_out = this->computePositionControl();
    if(oriOn == 0)
        torque_out += this->computeOrientationControl();
    if(nullOn == 0)
        torque_out += this->computeNullSpaceControl();
    
    
    /*Experiment*/
    //torque_out += gravity.data + coriolis.data;
    
    //this->computeForceBasedControl();
    
    for(int i = 0; i < nr_joints; i++){
        /*
        if(std::fabs(torque_out[i]) > torque_limits[i]){
            if(torque_out[i] < 0)
                torque_out[i] = torque_limits[i]*-1;
            else
                torque_out[i] = torque_limits[i];
        }
        */
        joint_handles[i].setCommand(torque_out[i]);
    }
    publishTransform(tf_desired, "world", "desired_pose");
}

bool NullSpaceController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh){
  
    buildTree("robot_description");
     this->cmd_sub = root_nh.subscribe("command",10, &NullSpaceController::cmd_callback, this);
     this->null_sub = root_nh.subscribe("null",1, &NullSpaceController::null_callback,this);
     this->ori_sub = root_nh.subscribe("ori",1, &NullSpaceController::ori_callback, this);
    
     nullOn = 1;
     oriOn = 0;
     
     /*From urdf, cant find in kdl tree */
    joint_limits.push_back(JointLimit(2.90, -2.90));
    joint_limits.push_back(JointLimit(1.76, -1.76));
    joint_limits.push_back(JointLimit(2.90, -2.90));
    joint_limits.push_back(JointLimit(-0.07, -3.07));
    joint_limits.push_back(JointLimit(2.90, -2.90));
    joint_limits.push_back(JointLimit(3.75, -0.02));
    joint_limits.push_back(JointLimit(2.90, -2.90));
    
    torque_limits.push_back(87);
    torque_limits.push_back(87);
    torque_limits.push_back(87);
    torque_limits.push_back(87);
    torque_limits.push_back(12);
    torque_limits.push_back(12);
    torque_limits.push_back(12);
    
    
    
    
    every_100th = 0;
    
    this->joint_names = hw->getNames();
    this->nr_joints = this->joint_names.size();
    this->nr_segments = chain.getNrOfSegments();

    null_signal.resize(nr_joints);
    for(int i = 0; i < nr_joints; i++){
        null_signal[i] = torque_limits[i];
    }
    
    /* Pointer initalizations*/
    joint_handles.reset(new hardware_interface::JointHandle[nr_joints]);
    jac_solver.reset(new KDL::ChainJntToJacSolver(chain));
    jacdot_solver.reset(new KDL::ChainJntToJacDotSolver(chain));
    fk_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
    cdp.reset(new KDL::ChainDynParam(chain, KDL::Vector(0, 0, -9.81)));

    p_gains.resize(nr_joints);
    p_gains << 12000, 30000, 18000, 18000, 12000, 7000, 2000;
    
    d_gains.resize(nr_joints);
    d_gains << 50, 100, 50, 70, 70, 50, 20;
    
    qs.q.resize(nr_joints);
    qs.qdot.resize(nr_joints);
    jac.resize(nr_joints);
    jacdot.resize(nr_joints);
    kdl_massMatrix.resize(nr_joints);
    gravity.resize(nr_joints);  
    coriolis.resize(nr_joints);
    massMatrix = massMatrix.Zero(nr_joints, nr_joints);
    torque_out.Zero(nr_joints);
    random_null.resize(nr_joints);
    prev_gravity = prev_gravity.Zero(nr_joints);

    /*New stuff remove all others*/
    x_des.resize(6);
    randomPose(x_des);
    x_now.resize(6);
    
    /*Initialize so the error > threshold at t = 0*/
    xp_error = xp_error.Ones(3);
    xo_error = xo_error.Ones(3);
    
    /*-----------*/
    
    xdot_des.resize(6);
    xdot_des << 0, 0, 0, 0, 0, 0;
    
    
    joint_origins.resize(nr_joints);
    for(int i = 0; i < nr_joints; i++){
        this->joint_handles[i] = hw->getHandle(joint_names[i]);
        joint_origins[i] = joint_handles[i].getPosition();
    }
    
    std::cout << "Initialize complete" << std::endl;
    return true;
}

void NullSpaceController::computeForceBasedControl(){
    /*
    double Kp = 20000;
    double Kd = 500;
    
    double roll, pitch, yaw;
    frame_now.M.GetRPY(roll, pitch, yaw);
    x_now6D << frame_now.p.x(), frame_now.p.y(), frame_now.p.z(), roll,pitch,yaw;
    
    xdot_now6D = jac.data * qs.qdot.data;
    
    Eigen::VectorXd xddot_r = xddot_des6D + Kd*(xdot_des6D - xdot_now6D) + Kp*(x_des6D - x_now6D);

   
    
    //Inertia weighted pseduo-inverse 
    j_pinv = jac.data * kdl_massMatrix.data.inverse() * jac.data.transpose();
    j_pinv = j_pinv.inverse();
    j_pinv = kdl_massMatrix.data.inverse() * jac.data.transpose();
    
    Eigen::VectorXd g_comp = gravity.data + coriolis.data;
    
    torque_null = torque_null.Zero(nr_joints);
   
    torque_out = kdl_massMatrix.data * j_pinv *(xddot_r - jacdot.data*qs.qdot.data + jac.data*kdl_massMatrix.data.inverse() *g_comp)
            + (Eigen::MatrixXd::Identity(nr_joints, nr_joints) - jac.data.transpose()*j_pinv.transpose())*torque_null;
    
    double alpha = 1;
    
    torque_null = -Kd*qs.qdot.data -alpha*(gravity.data - prev_gravity); 
    
    torque_out = kdl_massMatrix.data*j_pinv * (xddot_r - jacdot.data*qs.qdot.data) + g_comp;
    torque_out += (Eigen::MatrixXd::Identity(nr_joints, nr_joints) - jac.data.transpose()*j_pinv.transpose() )*torque_null;
     if( every_100th > 1000){
        std::cout << "Desired 6D pose: \n" << x_des6D << std::endl; 
        tf::Quaternion q;
        q.setRPY(x_des6D[3], x_des6D[4], x_des6D[5]);
        std::cout << "desired quaternion: " << q.getW() << ", " << q.getX() << ", " << q.getY() << ", " << q.getZ() << std::endl;
        std::cout << "Mass_xee: \n" << mass_xee << std::endl;
        std::cout << "mass matrix:\n " << kdl_massMatrix.data << std::endl;
        std::cout << "Joint values: \n" << qs.q.data << std::endl;
        std::cout << "Torque out: \n" << torque_out << std::endl;
        every_100th = 0;
    }
    every_100th++;
    
    
    prev_gravity = gravity.data;
    */
}


Eigen::VectorXd NullSpaceController::computePositionControl(){
    /*Get positional vectors*/
    Eigen::MatrixXd jac_p = jac.data.block(0, 0, 3, nr_joints);
    Eigen::VectorXd xp_des = x_des.block<3,1>(0,0);
    Eigen::VectorXd xp_now = x_now.block<3,1>(0,0);
    Eigen::VectorXd xpdot_des = xdot_des.block<3,1>(0,0);
    Eigen::VectorXd xpdot_now = xdot_now.block<3,1>(0,0);
    
    
    //mass_xee = jac.data.block(0,0, 3, nr_joints) * kdl_massMatrix.data.inverse() * jac.data.block(0, 0, 3, nr_joints).transpose();
    mass_xee = jac_p* kdl_massMatrix.data.inverse() * jac_p.transpose();
    mass_xee = mass_xee.inverse();
    
    
    xp_error = xp_des - xp_now;
    xpdot_error = xpdot_des - xpdot_now;
    double Kp, Kd;
    Kp = 15000.0;
    Kd = 150.0;
    
    /*Dynamically consistent generalized inverse*/
    j_pinv  = mass_xee * jac_p * kdl_massMatrix.data.inverse();

    Eigen::VectorXd tau_p;
    tau_p = jac_p.transpose() * mass_xee * (Kp*xp_error + Kd*xpdot_error) + gravity.data + coriolis.data;
    return tau_p;
}

Eigen::VectorXd NullSpaceController::computeOrientationControl(){
    
    Eigen::MatrixXd jac_o = jac.data.block(3, 0, 3, nr_joints);
    Eigen::VectorXd xo_des = x_des.block<3,1>(3,0);
    Eigen::VectorXd xo_now = x_now.block<3,1>(3,0);
    Eigen::VectorXd xodot_des = xdot_des.block<3,1>(3,0);
    Eigen::VectorXd xodot_now = xdot_now.block<3,1>(3,0);
    
    mass_xee = jac_o * kdl_massMatrix.data.inverse() * jac_o.transpose();
    mass_xee = mass_xee.inverse();

    xo_error = xo_des - xo_now;
    xodot_error = xodot_des - xodot_now;
    double Kp, Kd, Kp_null;
    Kp = 15000.0;
    Kd = 150.0;
    Kp_null = 0.0;
    
    /*Dynamically consistent generalized inverse*/
    j_pinv  = mass_xee * jac_o * kdl_massMatrix.data.inverse();
    /*With null space control*/
    Eigen::VectorXd torque_null = Kp_null*(joint_origins - qs.q.data);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nr_joints, nr_joints);
    
    Eigen::VectorXd tau_o;
    
    tau_o = jac_o.transpose() * mass_xee * (Kp*xo_error + Kd*xodot_error);

    return tau_o;
}

Eigen::VectorXd NullSpaceController::computeNullSpaceControl(){
    
    Eigen::MatrixXd jac_p = jac.data.block(0, 0, 3, nr_joints);
    
    mass_xee = jac_p* kdl_massMatrix.data.inverse() * jac_p.transpose();
    mass_xee = mass_xee.inverse();
    
    double Kp_null = 5.0;
    
    /*Choosing this ??*/
    Eigen::VectorXd torque_null_signal = Eigen::VectorXd::Zero(nr_joints);

    if(every_100th > 50){
        this->randomJointArray(random_null);
        every_100th = 0;
    }
    
    //torque_null_signal = Kp_null*(random_null.data - qs.q.data);
    torque_null_signal = Kp_null*(random_null.data);
  
    
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nr_joints, nr_joints);
    
     /*Dynamically consistent generalized inverse*/
    j_pinv  = mass_xee * jac_p * kdl_massMatrix.data.inverse();
    Eigen::MatrixXd null_filter = I - jac_p.transpose()*j_pinv;
    
    Eigen::VectorXd torque_null = -null_filter*torque_null_signal;
    every_100th++;
    return torque_null;
    
}

void NullSpaceController::cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd_msg){
    this->x_des(0) = cmd_msg->linear.x;
    this->x_des(1) = cmd_msg->linear.y;
    this->x_des(2) = cmd_msg->linear.z;
    this->x_des(3) = cmd_msg->angular.x;
    this->x_des(4) = cmd_msg->angular.y;
    this->x_des(5) = cmd_msg->angular.z;
    
    setDesiredTF();
            
}

void NullSpaceController::null_callback(const std_msgs::Int16ConstPtr& null_msg){
    this->nullOn = null_msg->data;
}

void NullSpaceController::ori_callback(const std_msgs::Int16ConstPtr& ori_msg){
    this->oriOn = ori_msg->data;
}

void NullSpaceController::incrementTorqueNull(){
    
    double steps = 100; // 1 [Nm]
    double lambda;
    
    for(int i = 0; i < nr_joints; i++){
        lambda = (torque_limits[i]);
        lambda /= steps;
        
        if(lambda > torque_limits[i])
            null_signal[i] = 0;
        null_signal[i] += lambda;
    }
    
}


void NullSpaceController::randomJointArray(KDL::JntArray& ja){
    double safety_diff = 0.5;
    for(int i = 0; i < nr_joints; i++)
        ja.data[i] = doubleRand(joint_limits[i].lower + safety_diff, joint_limits[i].upper - safety_diff);
}

void NullSpaceController::buildMassMatrix(){
    Eigen::MatrixXd M_i;
    M_i = M_i.Zero(6,6);
    massMatrix = massMatrix.Zero(nr_joints, nr_joints); // nr segments?
    
    KDL::RigidBodyInertia rbi;
    
    for(int k = 0; k < chain.getNrOfSegments(); k++){
        
        rbi = chain.segments[k].getInertia();
        
        Eigen::Matrix3d m_diag;

        m_diag << rbi.getMass(), 0, 0,
                   0, rbi.getMass(), 0,
                    0, 0, rbi.getMass();
        
        Eigen::Matrix3d rotIner;
        /* xx xy xz
        /* yx yy yz
        /* zx zy zz*/
        rotIner << rbi.getRotationalInertia().data[0], rbi.getRotationalInertia().data[1], rbi.getRotationalInertia().data[2],
                   rbi.getRotationalInertia().data[3], rbi.getRotationalInertia().data[4], rbi.getRotationalInertia().data[5],
                   rbi.getRotationalInertia().data[6], rbi.getRotationalInertia().data[7], rbi.getRotationalInertia().data[9];   
        
        M_i.block<3,3>(0,0) = m_diag;
        M_i.block<3,3>(3,3) = rotIner;

        //this->massMatrix +=  J_i[k].data.transpose() * M_i * J_i[k].data;
      
        /*Multiply here*/
        //M_i += M_i;
    }
    //std::cout << "Mass Matrix: \n" << massMatrix << std::endl;
}

void NullSpaceController::randomPose(Eigen::VectorXd& x){
    KDL::JntArray ja;
    ja.resize(nr_joints);
    
    randomJointArray(ja);
    
    KDL::Frame pose;
    
    fk_solver->JntToCart(ja, pose);
    double roll, pitch, yaw;
    pose.M.GetRPY(roll, pitch, yaw);

    x << pose.p.x(), pose.p.y(), pose.p.z(), roll, pitch, -yaw;
        
    setDesiredTF();
}

void NullSpaceController::setDesiredTF(){
    tf_desired.setOrigin(tf::Vector3(x_des(0), x_des(1), x_des(2)));
    tf::Quaternion q;
    q.setRPY(x_des(3), x_des(4), x_des(5));
    tf_desired.setRotation(q);
    tf_desired3D.setOrigin(tf::Vector3(x_des(0), x_des(1), x_des(2)));
    
}

void NullSpaceController::publishTransform(tf::Transform t, std::string frame_id, std::string child_frame){
    this->tf_broadcaster.sendTransform(tf::StampedTransform(t, ros::Time::now(), frame_id, child_frame));
}

double NullSpaceController::doubleRand(double a, double b){
    std::default_random_engine generator;
    std::random_device rd;
    generator.seed(rd());
    std::uniform_real_distribution<double> distr(a, b);
    return distr(generator);
}

void NullSpaceController::buildTree(std::string urdfParam){
    urdf::Model m;
    m.initParam(urdfParam);
    if(!kdl_parser::treeFromUrdfModel(m, tree))
        ROS_ERROR("Could not build tree");
    
    tree.getChain("panda_link0", "panda_link8", chain);
    //tree.getChain("three_dof_planar_link0", "three_dof_planar_link8", chain);
}

void NullSpaceController::jacDLSinv(KDL::Jacobian j, Eigen::MatrixXd &j_pinv, double k_factor){
    j_pinv = j.data*j.data.transpose();
    j_pinv += k_factor * k_factor * Eigen::MatrixXd::Identity(j_pinv.rows(), j_pinv.cols()); //j_pinv.Identity(j_pinv.rows(), j_pinv.cols());
    j_pinv = j_pinv.inverse();
    j_pinv = j.data.transpose() * j_pinv;
}

void NullSpaceController::printJointStates(){
    std::cout << "Printing joint states, nr joints = " << nr_joints << std::endl;
    for(int i = 0; i < nr_joints; i++){
        std::cout << "Joint: " << joint_handles[i].getName() << std::endl;
        std::cout << "Command: " << joint_handles[i].getCommand() <<std::endl;
        std::cout << "Effort: " << joint_handles[i].getEffort() << std::endl;
        std::cout << "Position: " << joint_handles[i].getPosition() << std::endl;
        std::cout << "Velocity: " << joint_handles[i].getVelocity() << std::endl;
    }
}

} //name space
   
PLUGINLIB_EXPORT_CLASS(my_simple_controllers::NullSpaceController,
                       controller_interface::ControllerBase)
   
   
   
   
   
