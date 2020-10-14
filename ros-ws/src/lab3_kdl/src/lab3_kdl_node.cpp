#include <lab3_kdl/lab3_kdl_node.h>

Lab3Node::Lab3Node() :
    js_sub(this->nh.subscribe("joint_states", 1, &Lab3Node::js_callback, this)),
    js_pub(this->nh.advertise<sensor_msgs::JointState>("joint_states",1)),
    ik_service(this->nh_srv.advertiseService("lab3ik", &Lab3Node::ik_service_callback, this)),
    FKSolver_handle(NULL),
    jacSolver_handle(NULL)
{
}
Lab3Node::Lab3Node(std::string urdfParam) :
    js_sub(this->nh.subscribe("joint_states",1,&Lab3Node::js_callback, this)),
    js_pub(this->nh.advertise<sensor_msgs::JointState>("joint_states",1)),
    ik_service(this->nh_srv.advertiseService("lab3ik", &Lab3Node::ik_service_callback, this))
    {
        this->buildTree(urdfParam);
        this->FKSolver_handle.reset(new KDL::ChainFkSolverPos_recursive(this->chain));
        this->jacSolver_handle.reset(new KDL::ChainJntToJacSolver(this->chain));
    }

void Lab3Node::buildTree(std::string urdfParam){
    urdf::Model m; /* From model, string/param didnt work */
    m.initParam(urdfParam);
    
    if(!kdl_parser::treeFromUrdfModel(m,this->tree))
        ROS_ERROR("Failed to construct tree");
    
    /*Tmp hardcode*/
    std::string base = "three_dof_planar_link0";
    std::string tip = "three_dof_planar_eef";
    this->tree.getChain(base,tip, this->chain);

    /*Get joint limits from urdf*/
    myJointLimit limit;
    std::map<std::string, urdf::JointSharedPtr>::iterator i;
    for(i = m.joints_.begin(); i != m.joints_.end(); i++){
        if(i->second->limits != NULL){ /* Chain has 3 joints and model has 5 */
            limit.lower = i->second->limits->lower;
            limit.upper = i->second->limits->upper;
            this->jointLimits.insert(std::pair<std::string, myJointLimit>(i->first, limit));
        }
    }
    
    /*Preallocate*/
    int n = this->chain.getNrOfJoints();
    this->q_pos.resize(n);
    this->joint_names.resize(n);
    this->joint_names.at(0) = "three_dof_planar_joint1";
    this->joint_names.at(1) = "three_dof_planar_joint2";
    this->joint_names.at(2) = "three_dof_planar_joint3";
    
    this->q_dot.resize(n);
    this->jac.resize(n);
    this->current_js.header.stamp = ros::Time::now();
    this->current_js.name.push_back("three_dof_planar_joint1");
    this->current_js.name.push_back("three_dof_planar_joint2");
    this->current_js.name.push_back("three_dof_planar_joint3");
    for(int i = 0; i < n; i++){
        this->current_js.position.push_back(0);
    }
    this->js_pub.publish(this->current_js);
}


/*because tf::transformKDLToTF is deprecated*/
void Lab3Node::kdlToTF(KDL::Frame &f, tf::Transform &t){
    geometry_msgs::Transform t_msg;
    tf::transformKDLToMsg(f, t_msg);
    tf::transformMsgToTF(t_msg, t);
}

void Lab3Node::kdlToEigen(KDL::Frame& f, Eigen::Affine3d& e){
    tf::Transform f_tf;
    this->kdlToTF(f, f_tf);
    tf::transformTFToEigen(f_tf, e);
}

void Lab3Node::msgToEigen(const geometry_msgs::Pose pose, Eigen::Affine3d& e){
    tf::Transform p_tf;
    tf::poseMsgToTF(pose, p_tf);
    tf::transformTFToEigen(p_tf, e);
}

void Lab3Node::publishTransform(tf::Transform t, std::string frame, std::string child){
    this->tf_broadcaster.sendTransform(tf::StampedTransform(t, ros::Time::now(), frame, child));
}

void Lab3Node::js_callback(const sensor_msgs::JointState::ConstPtr& js_msg){
    this->current_js = *js_msg;
    for(int i = 0; i < js_msg->position.size(); i++){
        this->q_pos(i) = js_msg->position[i];
        //this->joint_names.at(i) = js_msg->name[i];
    }
    /*Solve and publish end effector frame*/
    tf::Transform current;
    for(int i = 0; i < this->chain.segments.size(); i++){
        this->FKSolver_handle->JntToCart(this->q_pos, this->frame_ee, i);
        this->kdlToTF(this->frame_ee, current);
        /*
        if(i == 0){
            this->publishTransform(current, "world", this->chain.segments[i].getName() );
            std::cout << "Published world -> " << this->chain.segments[i].getName() << std::endl;
        }
        else{
            this->publishTransform(current, "world", this->chain.segments[i].getName());
            std::cout << "Published " << this->chain.segments[i-1].getName() << " -> " << this->chain.segments[i].getName() << std::endl;
        }
        */
        //this->publishTransform(this->tf_ee, "world", "ee_fromFK"+std::to_string(i));
    }
}    

void Lab3Node::initSolvers(){
    this->FKSolver_handle.reset(new KDL::ChainFkSolverPos_recursive(this->chain));
    this->jacSolver_handle.reset(new KDL::ChainJntToJacSolver(this->chain));
}

bool Lab3Node::ik_service_callback(lab3_kdl::lab3ikRequest &req, lab3_kdl::lab3ikResponse &res){
    ROS_INFO("service callback..");
    
    
    tf::Transform wanted_tf;
    tf::poseMsgToTF(req.pose,wanted_tf);
    this->publishTransform(wanted_tf, "world", "desired_pose");
    
    KDL::JntArray q_guess;
    this->guessJoints(q_guess);
    std::cout << "Guessed joints" << std::endl;
    /*Find segment of req.frame_id*/
    int segmentNrOfDesiredFrame = this->findSegmentNr(req.frame_id);
    
    if(segmentNrOfDesiredFrame < 0){
        ROS_ERROR("SegmentNr: %d, Requested frame does not exist in chain, did you enter a joint?", segmentNrOfDesiredFrame);
        ROS_ERROR("Available frames are: ");
        for(int i =0; i < this->chain.segments.size(); i++)
            ROS_ERROR("%s",this->chain.segments[i].getName().c_str() );
        return false;
    }
    else{
        std::cout << "SegmentNr of frame_id: " << segmentNrOfDesiredFrame << std::endl;
    }
        
    /*Frame T_c guess */
    KDL::Frame frame_guess;
    int fk_err = this->FKSolver_handle->JntToCart(q_guess, frame_guess, segmentNrOfDesiredFrame);
    std::cout << this->FKSolver_handle.get()->strError(fk_err) << std::endl;
    tf::Transform tf_guess;
    this->kdlToTF(frame_guess, tf_guess);
    this->publishTransform(tf_guess, "world", "guess");

    Eigen::Affine3d eigen_guess;
    this->kdlToEigen(frame_guess, eigen_guess);

    Eigen::Affine3d eigen_desired;
    this->msgToEigen(req.pose, eigen_desired);

    Eigen::Affine3d eigen_minimalTransform = eigen_desired.inverse() * eigen_guess; /* Bring guess frame onto desired frame */
    Eigen::Vector3d offset_min = eigen_minimalTransform.translation();
    Eigen::Vector3d rpy_min = eigen_minimalTransform.rotation().eulerAngles(0,1,2); /*Roll = 0, pitch = 1, yaw = 2 */
    
    Eigen::VectorXd v(offset_min.rows() +rpy_min.rows());
    v << offset_min, rpy_min;
    std::cout << "v: \n" << v << std::endl;
    
    KDL::Jacobian jac_guess;
    jac_guess.resize(this->chain.getNrOfJoints());
    
    Eigen::MatrixXd jac_pinv;
    
    double dt = 2;
    double lambda = 1/dt;
    v = v/dt;
    
    double lowest_norm = 100;
    Eigen::Vector3d lowest_q;
    
    /*Loop starts here*/
    int counter = 0;
    while(counter < 100){
        this->publishTransform(tf_guess, "world", "guess");
        this->publishTransform(wanted_tf, "world", "desired_pose");
        
        this->jacSolver_handle->JntToJac(q_guess, jac_guess, segmentNrOfDesiredFrame);
        this->jacDLSinv(jac_guess, jac_pinv, 0.5);
        q_guess.data = q_guess.data + lambda*jac_pinv*v;
        
        this->FKSolver_handle->JntToCart(q_guess, frame_guess, segmentNrOfDesiredFrame);
        this->kdlToEigen(frame_guess, eigen_guess);
        
        eigen_minimalTransform = eigen_guess.inverse() * eigen_desired; /* Bring guess frame onto desired frame */
        
        offset_min = eigen_minimalTransform.translation();
        rpy_min = eigen_minimalTransform.rotation().eulerAngles(0,1,2); /*Roll = 0, pitch = 1, yaw = 2 */
        
        v << offset_min, rpy_min;
        v = v/dt;
        std::cout << "v: \n" << v << std::endl;
        
        if(offset_min.norm() < lowest_norm){
            lowest_norm = offset_min.norm();
            lowest_q = Eigen::Vector3d(q_guess.data(0), q_guess.data(1), q_guess.data(2));
        }
        
        if(offset_min.norm() < 0.01 && rpy_min.norm() < 0.01){
            std::cout << "tolerance aquired at i = " << counter << std::endl;
            break;
        }
        
        sensor_msgs::JointState js_out;
        js_out.header.stamp = ros::Time::now();
        js_out.name.push_back("three_dof_planar_joint1");
        js_out.name.push_back("three_dof_planar_joint2");
        js_out.name.push_back("three_dof_planar_joint3");
        js_out.position.push_back(q_guess.data(0));
        js_out.position.push_back(q_guess.data(1));
        js_out.position.push_back(q_guess.data(2));
        this->js_pub.publish(js_out);
        
        counter++;
        ros::Duration(0.2).sleep();
    }
    
    std::cout << "Lowest norm: " << lowest_norm << std::endl;
    std::cout << "lowest q: \n" << lowest_q << std::endl;
    
    this->kdlToTF(frame_guess, tf_guess);
    this->publishTransform(tf_guess, "world", "iterated_guess");

    sensor_msgs::JointState js_out;
    js_out.header.stamp = ros::Time::now();
    js_out.name.push_back("three_dof_planar_joint1");
    js_out.name.push_back("three_dof_planar_joint2");
    js_out.name.push_back("three_dof_planar_joint3");
    js_out.position.push_back(q_guess.data(0));
    js_out.position.push_back(q_guess.data(1));
    js_out.position.push_back(q_guess.data(2));
    //res.jointstate = js_out;
    return true;
}

void Lab3Node::jacDLSinv(KDL::Jacobian j, Eigen::MatrixXd &j_pinv, double k_factor){
    j_pinv = j.data*j.data.transpose();
    j_pinv += k_factor * k_factor * Eigen::MatrixXd::Identity(j_pinv.rows(), j_pinv.cols()); //j_pinv.Identity(j_pinv.rows(), j_pinv.cols());
    j_pinv = j_pinv.inverse();
    j_pinv = j.data.transpose() * j_pinv;
}

void Lab3Node::jacMPinv(KDL::Jacobian j, Eigen::MatrixXd& j_pinv){
    j_pinv = j.data * j.data.transpose();
    j_pinv = j_pinv.inverse();
    j_pinv *= j.data;   
}

int Lab3Node::findSegmentNr(const std::string frame_id){
    int n = 0;
    std::vector<KDL::Segment>::iterator it;
    for(it = this->chain.segments.begin(); it != this->chain.segments.end(); it++ ){
        if(frame_id.compare(it->getName()) == 0 ){
            return n;
        }
        n++;
    }
    return -1;
}

void Lab3Node::guessJoints(KDL::JntArray &ja){
    int n = this->chain.getNrOfJoints();
    ja.resize(n);
    double upper, lower;
    for(int i = 0; i < n; i++){
        upper = this->jointLimits.at(this->joint_names.at(i)).upper;
        lower = this->jointLimits.at(this->joint_names.at(i)).lower;
        std::cout<< this->joint_names.at(i) << ":  up: " << upper << " l: " << lower << std::endl;
        ja(i) = this->doubleRand(lower,upper);
    }
    
}

void Lab3Node::checkJointLimits(KDL::JntArray& ja){
    double upper,lower;
    int n = this->chain.getNrOfJoints();
    for(int i = 0; i < n; i++){
        upper = this->jointLimits.at(this->joint_names.at(i)).upper;
        lower = this->jointLimits.at(this->joint_names.at(i)).lower;
        if(ja.data[i] < lower)
            ja.data[i] = lower;
        if(ja.data[i] > upper)
            ja.data[i] = upper;

    }
}

double Lab3Node::doubleRand(double a, double b){
    std::default_random_engine generator;
    std::random_device rd;
    generator.seed(rd());
    std::uniform_real_distribution<double> distr(a, b);
    return distr(generator);
}


void Lab3Node::publishJointState(){
    this->js_pub.publish(this->current_js);
}

int main(int argc, char *argv[]){
    ros::init(argc,argv, "lab3_node");
    Lab3Node _nh("robot_description");

    ros::Rate r(10);
    while(ros::ok()){
        //_nh.publishJointState();
        ros::spinOnce();
        r.sleep();
    }
    ros::shutdown();
}
