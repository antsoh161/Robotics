#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>
#ifndef JOINT_H
#define JOINT_H


struct Limits{
    double upper;
    double lower;
};

class Joint{
public:
    typedef enum {Revolute, Prismatic, Fixed, Unknown} JointType;
    Joint(){};
    Joint(JointType _type);
    Joint(std::string _name, double jointVarInit, JointType type);
    
    void setType(JointType _type);
    void setJointVariable(double _jv);
    void setActuationAxis(Eigen::Vector3f _aAxis);
    void setLimits(double upper, double lower);
    virtual std::string setName(std::string _name);
    
    std::string getName();
    double getJointVariable();
    Eigen::Vector3f getActuationAxis();
    Joint::JointType getType();
private:
    std::string name;
    double jointVariable;
    Limits limits;
    
    Eigen::Vector3f actuationAxis;
    Joint::JointType type;
    
};


#endif /* JOINT_H */