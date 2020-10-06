#ifndef SIMPLE_STATE_CONTROLLER_H
#define SIMPLE_STATE_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <my_simple_controllers/joint.h>
#include <my_simple_controllers/chain.h>
#include <my_simple_controllers/homtra.h>
#include <my_simple_controllers/FKParser.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf/model.h>
#include <tf/transform_broadcaster.h>

namespace my_simple_controllers {

  class StateController : public controller_interface::Controller<hardware_interface::JointStateInterface> {
  private:
        std::vector<hardware_interface::JointStateHandle> jointStateHandles;
        Chain chain;
        tf::TransformBroadcaster tf_broadcaster;
        bool doOnce;
    public:
        //StateController();
         //method executed by controller manager on every loop when controller is running
         //NOTE: MUST BE implemented
        void publishTransform(tf::Transform t, std::string frame_id, std::string child_frame);
        virtual void update(const ros::Time& time, const ros::Duration& period);

         //initialize controller with access to hardware interface and node handles
        virtual bool init(hardware_interface::JointStateInterface* hw, 
		      ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  };


};

#endif
