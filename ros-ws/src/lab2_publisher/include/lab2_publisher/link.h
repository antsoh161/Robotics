#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include "joint.h"
#include "homtra.h"

#ifndef LINK_H
#define LINK_H

typedef struct Pose{
    Eigen::Vector3f position;
    Eigen::Vector4f quaternion; // [w x y z]
}Pose;

class Link{
private:
    std::string name;
    Joint *parent;
    Joint *child;
    float length;
    Pose pose;
    Eigen::Vector3f offset;
    HomTransform frame; //From parent -> child
public:
 
    Link(std::string _name="", Joint *_parent=NULL, Joint *_child=NULL);
    static Link CreateLink(std::string _name, Joint *_parent, Joint *_child);
    
    void setName(std::string _name);
    void setLength(float _length);
    void setChild(Joint _child);
    void setParent(Joint _parent);
    void setFrame(HomTransform _frame);
    void setPose(Eigen::Vector3f _position, Eigen::Vector4f _quat);
    void linkTip(Joint _parent);
    void linkJoints(Joint _parent, Joint _child);
    void updateFrameFromDHDisplacement(double d);
    void updateFrameFromDHAngle(double theta);
    
    Eigen::Vector3f getOffset();
    std::string getName();
    float getLength();
    Pose getPose();
    Joint *getParent();
    Joint *getChild();
    HomTransform getFrame();
};

#endif /* LINK_H  */