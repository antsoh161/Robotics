#include <lab2_publisher/lab2_publisher_node.h>

void scaraUnitTest();

int main(int argc, char* argv[]){
  
    ros::init(argc, argv, "lab2_publisher");
    ROS_INFO("Starting lab2_publisher");
    Lab2PubNode _nh;
    _nh.setUpdateRate(ros::Duration(0.1)); //This must be done
    
    std::string urdf_parameter = "robot_description";
    
    Chain chain = Chain::fromUrdf(urdf_parameter);
    chain.printChain();

    _nh.convertChainToTF(chain);
    
    ros::spin();
    ROS_INFO("Shutting down lab2_publisher");
    ros::shutdown();
    
    //scaraUnitTest();
    return 0;
}

void Lab2PubNode::convertChainToTF(Chain c){
    this->homtra_transforms = c.getAllFrames();
    for(int i = 0; i < this->homtra_transforms.size(); i++){
        this->tf_transforms.push_back(HomTransform::homtraToTF(this->homtra_transforms[i]));
    }
    
}

void Lab2PubNode::publishTransform(tf::Transform t, std::string frame_id, std::string child_frame){
    this->tf_broadcaster.sendTransform(tf::StampedTransform(t, ros::Time::now(), frame_id, child_frame));
}

std::string Lab2PubNode::getParamString(std::string param){
    std::string tmp;
    if(this->nh.getParam(param, tmp))
        return tmp;
    else{
        ROS_ERROR("Can not build chain from this parameter");
        exit(0);
    }
}

void Lab2PubNode::setUpdateRate(ros::Duration dur){
    this->updateTimer = this->nh.createTimer(dur, &Lab2PubNode::updateCallback,this);
}

void Lab2PubNode::updateCallback(const ros::TimerEvent&){
    ROS_INFO("updating..");
    for(int i = 0; i < this->tf_transforms.size()-1; i++){
            if(i==0)
                this->publishTransform(tf_transforms[i],"world", "frame"+std::to_string(i+1));
            else
                this->publishTransform(tf_transforms[i], "frame"+std::to_string(i), "frame"+std::to_string(i+1));
        }
        //_nh.publishTransform(tf_be, "world", "endeffect");
    
}
/* Manual description of a scara robot*/
void scaraUnitTest(){
    double l0, l1, l2, l3;
    l0 = 0.5;
    l1 = 0.7;
    l2 = 1.2;
    l3 = 0.05;
    Chain chain;
    
    Joint base("Base", 0, Joint::Fixed);
    Joint j0("j0", 0, Joint::Revolute);
    Link l_b1 = Link::CreateLink("linkb1",&base, &j0);
    l_b1.setFrame(HomTransform::fromDH(0, base.getJointVariable(), 0, 0 ));
    chain.addLinkToEnd(l_b1);
    
    Joint j1("j1", 0, Joint::Revolute);
    Link l_01 = Link::CreateLink("link01",&j0, &j1);
    l_01.setFrame(HomTransform::fromDH(0, j0.getJointVariable(), l1, l0));
    chain.addLinkToEnd(l_01);
    
    Joint j2("j2", 0, Joint::Revolute);
    Link l_12 = Link::CreateLink("link12",&j1, &j2);
    l_12.setFrame(HomTransform::fromDH(0, j1.getJointVariable(), l2, 0));
    chain.addLinkToEnd(l_12);
    
    Joint j3("j3", 0, Joint::Prismatic);
    Link l_23 = Link::CreateLink("link23",&j2, &j3);
    l_23.setFrame(HomTransform::fromDH(M_PI, 0, 0, j2.getJointVariable()));
    chain.addLinkToEnd(l_23);
    
    Joint j4("j4", 0, Joint::Revolute);
    Link l_34 = Link::CreateLink("link34",&j3, &j4);
    l_34.setFrame(HomTransform::fromDH(0, j3.getJointVariable(), 0, l3));
    chain.addLinkToEnd(l_34);
    
    Joint E("EndEffect", 0, Joint::Fixed);
    Link l_4e = Link::CreateLink("link4e",&j4, &E);
    l_4e.setFrame(HomTransform::fromDH(0, 0, 0, 0));
    chain.addLinkToEnd(l_4e);
    
    //std::cout << "Pose: --------" << std::endl;
    //std::cout << l_4e.getPose().position << " : " << std::endl << l_4e.getPose().quaternion << std::endl;
    
    std::cout << l_b1.getFrame()*l_01.getFrame()*l_12.getFrame()*l_23.getFrame()*l_34.getFrame()*l_4e.getFrame()*Eigen::Vector3f(0,0,0) << std::endl;
    
    chain.printChain();
}