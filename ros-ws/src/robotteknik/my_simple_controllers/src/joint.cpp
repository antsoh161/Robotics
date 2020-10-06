#include <my_simple_controllers/joint.h>
#include "my_simple_controllers/chain.h"

Joint::Joint(Joint::JointType _type) : 
        name("unnamed"),
        jointVariable(0.0),
        type(_type)
        {}

 Joint::Joint(std::string _name,  double _jv, Joint::JointType _type ) : 
        name(_name),
        jointVariable(_jv),
         type(_type)
        {}
 /*2d coordinates only so far*/
 
std::string Joint::setName(std::string _name){
     this->name = _name;
 }
 

void Joint::setJointVariable(double _jv){
    this->jointVariable = _jv;
}

void Joint::setType(JointType _type){
    this->type = _type;
}

void Joint::setActuationAxis(Eigen::Vector3f _aAxis){
    this->actuationAxis = _aAxis;
}

std::string Joint::getName(){
    return this->name;
}

double Joint::getJointVariable(){
    return this->jointVariable;
}

Eigen::Vector3f Joint::getActuationAxis(){
    return this->actuationAxis;
}

Joint::JointType Joint::getType(){
    return this->type;
}
