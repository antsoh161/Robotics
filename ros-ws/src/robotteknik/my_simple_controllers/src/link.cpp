#include <my_simple_controllers/link.h>

Link::Link(std::string _name, Joint *_parent, Joint *_child) :
    name(_name),    
    parent(_parent),
    child(_child)
    {
    }

Link Link::CreateLink(std::string _name, Joint *_parent, Joint *_child){
    Link link(_name, _parent, _child);
    HomTransform frame; //Todo
    return link;
}

void Link::updateFrameFromDHDisplacement(double d){
    this->frame.updateFromDH_D(d);
}
void Link::updateFrameFromDHAngle(double theta){
    this->frame.updateFromDHTheta(theta);
}

void Link::updateFromJointValueAxis(double _jointValue, Eigen::Vector3f _axis, Joint::JointType _type){
    if(_type == Joint::Prismatic){
        this->frame.setTranslationWithAxis(_jointValue, _axis);
    }
    else if(_type == Joint::Revolute){
        this->frame.setRotationFromAngleAxis(_jointValue, _axis);
    }
    else if(_type == Joint::Fixed){ //Do nothing
        ;
    }
    else
        ROS_ERROR("This version only supports prismatic and revolute joints, cant update");
}

void Link::setName(std::string _name){
    this->name = _name;
}

void Link::setLength(float _length){
    this->length=_length;
}

void Link::setChild(Joint _child){
    this->child = &_child;
}

void Link::setParent(Joint _parent){
    this->parent = &_parent;
}

void Link::linkJoints(Joint _parent, Joint _child){
    this->parent = &_parent;
    this->child = &_child;
}

void Link::linkTip(Joint _parent){
    this->parent = &_parent;
}

void Link::setFrame(HomTransform _frame){
    this->frame= _frame;
}

void Link::setPose(Eigen::Vector3f _position, Eigen::Vector4f _quat){
    this->pose.position = _position;
    this->pose.quaternion = _quat;
}

std::string Link::getName(){
    return this->name;
}

Pose Link::getPose(){
    return this->pose;
}

float Link::getLength(){
    return this->length;
}
Joint *Link::getParent(){
    return this->parent;
}
Joint *Link::getChild(){
    return this->child;
}

HomTransform Link::getFrame(){
    return this->frame;
}

Eigen::Vector3f Link::getOffset(){
    return this->offset;
}
