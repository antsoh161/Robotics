#include<my_simple_controllers/state_controller.h>
#include <pluginlib/class_list_macros.h>  // to allow the controller to be loaded as a plugin

namespace my_simple_controllers {


void StateController::update(const ros::Time& time, const ros::Duration& period) {
    std::vector<hardware_interface::JointStateHandle>::iterator it;
    
    int i = 0;
    for(it = this->jointStateHandles.begin(); it != this->jointStateHandles.end(); it++){
        double jointValue = it->getPosition() + this->chain.getJointValueOrigin(i); //because the parser cant handle origini rpy??
        chain.updateJoint(it->getName(), jointValue);
        i++;
    }
    //chain.runFK();
    
    std::vector<HomTransform*> ht_transforms = chain.getAllFrames();
    
    std::vector<tf::Transform> tf_transforms;
    for(int i = 0; i < ht_transforms.size(); i++){
        tf_transforms.push_back(HomTransform::homtraToTF(*ht_transforms[i]));
    }
    
    for(int i = 0; i < tf_transforms.size()-1; i++){
        if(i==0)
            this->publishTransform(tf_transforms[i],"world", "frame" + std::to_string(i+1) );
        else
            this->publishTransform(tf_transforms[i],"frame" + std::to_string(i), "frame" + std::to_string(i+1) );
    }
    
    tf::Transform tf_baseToEnd = HomTransform::homtraToTF(this->chain.getEndEffectorTransform());
    this->publishTransform(tf_baseToEnd,"world","endeffect");
   //ROS_INFO("State Controller: here read joint handles, compute forward kinematic model and publish tool tip pose to TF");
}

bool StateController::init(hardware_interface::JointStateInterface* hw, 
	      ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    this->chain = Chain::fromUrdf("robot_description");
    
    std::vector<std::string> jointNames = hw->getNames();
    for(int i = 0; i < jointNames.size(); i++){
        std::cout << jointNames[i] << std::endl;
        this->jointStateHandles.push_back(hw->getHandle(jointNames[i]));
    }
   //ROS_INFO("State Controller: here read robot description from parameter server, initialize publishers, read parameters, load joint handles");
   return true;
}


void StateController::publishTransform(tf::Transform t, std::string frame_id, std::string child_frame){
    this->tf_broadcaster.sendTransform(tf::StampedTransform(t, ros::Time::now(), frame_id, child_frame));
}

}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(my_simple_controllers::StateController,
                       controller_interface::ControllerBase)
