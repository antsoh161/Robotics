#include <lab2_publisher/joint.h>
#include "lab2_publisher/chain.h"

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

std::string Joint::getName(){
    return this->name;
}

double Joint::getJointVariable(){
    return this->jointVariable;
}
Joint::JointType Joint::getType(){
    return this->type;
}
