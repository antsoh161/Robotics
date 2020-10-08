#include <lab3_kdl/lab3_kdl_node.h>

Lab3Node::Lab3Node() :
    js_sub(this->nh.subscribe("joint_states", 1, &Lab3Node::js_callback, this)),
    ik_service(this->nh.advertiseService("lab3_ik", &Lab3Node::ik_service_callback, this)), /*Take what parameters? */
    FKSolver_handle(NULL),
    jacSolver_handle(NULL)
{
}

Lab3Node::~Lab3Node(){
    delete this->FKSolver_handle; //prob not necessary
    delete this->jacSolver_handle;
}

void Lab3Node::buildTree(std::string urdfParam){
    std::string urdf_fulltext;
    urdf::Model m; /* From model, string/param didnt work */
    m.initParam(urdfParam);
    if(!kdl_parser::treeFromUrdfModel(m,this->tree))
        ROS_ERROR("Failed to construct tree");
    
    /*Tmp hardcode*/
    std::string base = "three_dof_planar_link0";
    std::string tip = "three_dof_planar_eef";
    this->tree.getChain(base,tip, this->chain);
    
    /*Preallocate*/
    int n = this->chain.getNrOfJoints();
    this->q_pos.resize(n);
    this->q_dot.resize(n);
    this->jac.resize(n);
}
/*because tf::transformKDLToTF is deprecated*/
void Lab3Node::kdlToTF(KDL::Frame &f, tf::Transform &t){
    geometry_msgs::Transform t_msg;
    tf::transformKDLToMsg(f, t_msg);
    tf::transformMsgToTF(t_msg, t);
}

void Lab3Node::publishTransform(tf::Transform t, std::string frame, std::string child){
    this->tf_broadcaster.sendTransform(tf::StampedTransform(t, ros::Time::now(), frame, child));
}

void Lab3Node::js_callback(const sensor_msgs::JointState::ConstPtr& js_msg){
    
    for(int i = 0; i < js_msg->position.size(); i++){
        this->q_pos(i) = js_msg->position[i];
    }
    /*Solve and publish end effector frame*/
    this->FKSolver_handle->JntToCart(this->q_pos, this->frame_ee);
    this->kdlToTF(this->frame_ee, this->tf_ee);
    this->publishTransform(this->tf_ee, "world", "ee_fromFK");
    /*Solve for jacobian at current */
    this->jacSolver_handle->JntToJac(this->q_pos, this->jac);
    this->jac_matrix = this->jac.data;
    
}    

void Lab3Node::initSolvers(){
    //if(this->FKSolver_handle != NULL)
    delete this->FKSolver_handle; // In case of reinitalization
    delete this->jacSolver_handle;
    this->FKSolver_handle = new KDL::ChainFkSolverPos_recursive(this->chain);
    this->jacSolver_handle = new KDL::ChainJntToJacSolver(this->chain);
}

/*Take what parameters?*/
void Lab3Node::ik_service_callback(){
    
    /*Solve with this->iksolver..*/
    
}


int main(int argc, char *argv[]){
    ros::init(argc,argv, "lab3_node");
    Lab3Node _nh;
    _nh.buildTree("robot_description");
    _nh.initSolvers();   
    
    ros::spin();
    
    ros::shutdown();
}
