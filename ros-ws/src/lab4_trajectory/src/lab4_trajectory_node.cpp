
#include "lab4_trajectory/lab4_trajectory.h"

TrajectoryNode::TrajectoryNode(std::string urdfParam) :
        command_pub(this->nh.advertise<geometry_msgs::Twist>("command", 1)),
        js_sub(this->nh.subscribe("joint_states", 1, &TrajectoryNode::js_callback, this)),
        traj_srv(this->nh.advertiseService("lab4traj", &TrajectoryNode::traj_srv_callback, this))
        {
            this->buildTree("robot_description");
            this->fk_solver.reset(new KDL::ChainFkSolverPos_recursive(this->chain));
        }

void TrajectoryNode::buildTree(std::string urdfParam){
    urdf::Model m; /* From model, string/param didnt work */
    m.initParam(urdfParam);
    
    if(!kdl_parser::treeFromUrdfModel(m,this->tree))
        ROS_ERROR("Failed to construct tree");
    
    /*Tmp hardcode*/
    std::string base = "three_dof_planar_link0";
    std::string tip = "three_dof_planar_eef";
    this->tree.getChain(base,tip, this->chain);
}

void TrajectoryNode::js_callback(const sensor_msgs::JointState::ConstPtr& js_msg){
    this->q.resize(js_msg->position.size());
    for(int i = 0; i < js_msg->position.size(); i++){
        this->q(i) = js_msg->position[i];
    }
    
    this->q_dot.resize(js_msg->velocity.size());
    for(int i = 0; i < js_msg->velocity.size(); i++){
        this->q_dot(i) = js_msg->velocity[i];
    }
        
}

bool TrajectoryNode::traj_srv_callback(lab4_trajectory::lab4_trajRequest& req, lab4_trajectory::lab4_trajResponse& res){
    
    KDL::JntArray desired_pose(6);
    this->poseToJntArray(req.pose, desired_pose);
    
    KDL::Frame initial_frame;
    this->fk_solver->JntToCart(this->q, initial_frame);
    
    KDL::JntArray initial_pose(6);
    this->kdlToJnts(initial_frame, initial_pose);
    
    KDL::JntArray offset;
    offset.data = desired_pose.data - initial_pose.data;
    
    double length = offset.data.norm();
    
    std::cout << "Start pose \n" << initial_pose.data << std::endl;
    std::cout << "goal pose \n" << desired_pose.data << std::endl;
    std::cout << "offset \n" << offset.data << std::endl;
    std::cout << "length = " << length << std::endl;
    
    
    double dt = 0.1;
    int current_step = 0;
    int max_steps = 100;
    double t, t_prev, arc, arc_prev;
    
    geometry_msgs::Twist command_out;
     KDL::JntArray vel_out;
    
    ros::Duration time_step = ros::Duration(dt);
    
    while(current_step < max_steps){
        t_prev = (double(current_step) - 1) / double(max_steps);
        t = double(current_step) / double(max_steps);
        if(t > 1.0)
            ROS_ERROR(" t = %d",t);
        
        //std::cout << "t_prev -> t : " << t_prev << " -> " << t << std::endl;
        
        arc_prev = this->trapezoidal_step(t_prev, 1.0, 0.0, length, length*1.5);
        arc = this->trapezoidal_step(t, 1.0, 0.0, length, length*1.5);
        
        //std::cout << "arc_prev -> arc : " << arc_prev << " -> " << arc << std::endl;
        
        KDL::JntArray step_prev = this->rectilinear_step(initial_pose, offset, arc_prev, length);
        KDL::JntArray step = this->rectilinear_step(initial_pose, offset, arc, length);
        
        //std::cout << "step prev : \n" << step_prev.data << std::endl;
        //std::cout << "step : \n" << step.data << std::endl;
        
        vel_out.data = (step.data - step_prev.data) / dt;
        
        command_out.linear.x = vel_out.data(0);
        command_out.linear.y = vel_out.data(1);
        command_out.linear.z = vel_out.data(2);
        command_out.angular.x = vel_out.data(3);
        command_out.angular.y = vel_out.data(4);
        command_out.angular.z = vel_out.data(5);
        
       // std::cout << "Velocity out: \n" << vel_out.data << "\n-------\n"; 

        this->command_pub.publish(command_out);
        
        
        current_step++;
        ros::Duration(time_step).sleep();
    }
    
    
    command_out.linear.x = 0;
    command_out.linear.y = 0;
    command_out.linear.z = 0;
    command_out.angular.x = 0;
    command_out.angular.y = 0;
    command_out.angular.z = 0;
    
    this->command_pub.publish(command_out);
    
    KDL::Frame f_out;
    this->fk_solver->JntToCart(q, f_out);
    KDL::JntArray j_out(6);
    this->kdlToJnts(f_out, j_out);
    std::cout << "End pose: " << j_out.data << std::endl;
    
    ROS_INFO("Trajectory complete");
    res.twist = command_out;
    return true;
}

double TrajectoryNode::trapezoidal_step(double t, double tf, double qi, double qf, double q_cruise){
    
    if(std::fabs(qi - qf)/tf >= q_cruise){
        ROS_ERROR("Cruise speed too small");
        return 0;
    }
    
    if(std::fabs(2 * (qf-qi) / tf) < q_cruise){
        ROS_ERROR("Cruise speed too large");
        return 0;
    }
    double tc = (qi- qf + q_cruise*tf)/q_cruise;
    double acc = (q_cruise*q_cruise) / (qi - qf + q_cruise * tf);
    double vel = 0;
    
    if(t >= 0 && t <= tc){
        /* Accelerate*/
        vel = qi + 0.5*acc*t*t;
        return vel;
    }
    else if(t > tc && t <= tf - tc){
        /*Cruise*/
        vel = (qi + acc * tc * (t - tc/2.0));
        return vel;
    }
    else if((t > (tf - tc) ) && (t <= tf) ){
        /* decelerate*/
        vel = (qf - 0.5*acc * (tf-t)*(tf-t));
        return vel;
    }
    
    ROS_ERROR("t is not between [t0, tf]");
    return 0;
}

KDL::JntArray TrajectoryNode::rectilinear_step(KDL::JntArray initial, KDL::JntArray offset, double arc_length, double length){
    KDL::JntArray step(6);
    step.data = initial.data + (arc_length * offset.data)/length;
    return step;
}

void TrajectoryNode::kdlToJnts(KDL::Frame& f, KDL::JntArray& ja){
    ja.data(0) = f.p(0);
    ja.data(1) = f.p(1);
    ja.data(2) = f.p(2);
    f.M.GetRPY(ja.data(3), ja.data(4), ja.data(5));
}

void TrajectoryNode::poseToJntArray(geometry_msgs::Pose& p, KDL::JntArray& ja){
    KDL::Frame f;
    tf::poseMsgToKDL(p,f);
    
    ja.data(0) = f.p(0);
    ja.data(1) = f.p(1);
    ja.data(2) = f.p(2);
    f.M.GetRPY(ja.data(3), ja.data(4), ja.data(5));
}

void TrajectoryNode::kdlToTF(KDL::Frame &f, tf::Transform &t){
    geometry_msgs::Transform t_msg;
    tf::transformKDLToMsg(f, t_msg);
    tf::transformMsgToTF(t_msg, t);
}

void TrajectoryNode::kdlToPose(KDL::Frame& f, geometry_msgs::Pose &pose){
    pose.position.x = f.p.x();
    pose.position.y = f.p.y();
    pose.position.z = f.p.z();
    tf::quaternionKDLToMsg(f.M, pose.orientation);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "lab4_trajectory_service");
    TrajectoryNode _nh("robot_description");
    
    while(ros::ok()){
        ros::spinOnce();
    }
    
    return 0;
}
