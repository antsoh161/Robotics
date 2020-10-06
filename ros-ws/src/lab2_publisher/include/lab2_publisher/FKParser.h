/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FKparser.h
 * Author: anton
 *
 * Created on September 24, 2020, 2:44 PM
 */
#include "joint.h"
#include "link.h"
#include "homtra.h"
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <eigen3/Eigen/Core>
#ifndef FKPARSER_H
#define FKPARSER_H

class FKParser{
private:
    urdf::Model model;
    friend class Chain;
protected:
    std::vector<Joint> joints;
    std::vector<Link> links;
public:
    Joint::JointType translateType(urdf::JointSharedPtr jointPtr);
    void parseJoints();
    void parseLinks();
    HomTransform parseTransform(std::string _jointName);
    int findJointIndex(std::string _jointName);
    bool jointExists(urdf::JointSharedPtr ptr);
    std::vector<Link> getLinks();
    void parse();
    void sortLinks();
    FKParser(){};
    FKParser(std::string urdf);
    
};



#endif /* FKPARSER_H */

