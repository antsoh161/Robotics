/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <my_simple_controllers/FKParser.h>

#include "my_simple_controllers/chain.h"

FKParser::FKParser(std::string urdfParam){
    this->model.initParam(urdfParam);
}

void FKParser::parse(){
    this->parseJoints();
    this->parseLinks();
    this->sortLinks();
    
}

void FKParser::parseJoints(){
    std::map<std::string, urdf::JointSharedPtr>::iterator jointIter;
    Joint tmp_joint;
     urdf::JointSharedPtr current;
     for(jointIter = this->model.joints_.begin(); jointIter != this->model.joints_.end(); jointIter++){
        current = jointIter->second;
        Joint::JointType t = this->translateType(current);
        tmp_joint = Joint(current->name, 0, t);
        tmp_joint.setActuationAxis( Eigen::Vector3f(current->axis.x,
                                    current->axis.y,
                                    current->axis.z ));
        std::cout << "Actuation axis: " << "[" << current->axis.x << ", " << current->axis.y << ", " << current->axis.z << "]\n";
        this->joints.push_back(tmp_joint);
    }
}

void FKParser::parseLinks(){
    std::map<std::string, urdf::LinkSharedPtr>::iterator linkIter;
    Link tmp_link;
    Joint *p, *c;
    int parentIndex, childIndex;
    for(linkIter = this->model.links_.begin(); linkIter != this->model.links_.end(); linkIter++){
        for(int i = 0; i < linkIter->second->child_joints.size(); i++) {
            if(this->jointExists(linkIter->second->parent_joint))
                parentIndex = this->findJointIndex(linkIter->second->parent_joint->name);
            else
                parentIndex = -1;

            if(this->jointExists(linkIter->second->child_joints[i]))
                childIndex = this->findJointIndex(linkIter->second->child_joints[i]->name);
            else
                childIndex = -1;

            if(parentIndex >= 0)
                p = &(this->joints[parentIndex]);
            else
                p = NULL;

            if(childIndex >= 0)
                c = &(this->joints[childIndex]);
            else
                c = NULL;

            tmp_link = Link::CreateLink(linkIter->second->name, p,c);
            HomTransform ht = this->parseTransform(linkIter->second->child_joints[i]->name);
            ht.setName(tmp_link.getName());
            tmp_link.setFrame(ht);
            this->links.push_back(tmp_link);
        }
        if(linkIter->second->child_joints.size() < 1){
            c = NULL;
            if(this->jointExists(linkIter->second->parent_joint)){
                parentIndex = this->findJointIndex(linkIter->second->parent_joint->name);
                p = &(this->joints[parentIndex]);
                tmp_link = Link::CreateLink(linkIter->second->name, p,c);
                this->links.push_back(tmp_link);
            }
            else{
                std::cout << "There exists links without any joints" << std::endl;
            }
        }
    }
}

HomTransform FKParser::parseTransform(std::string _jointName){
    urdf::JointConstSharedPtr jt = this->model.getJoint(_jointName);
    
    Eigen::Vector3f trans = Eigen::Vector3f(jt->parent_to_joint_origin_transform.position.x,
                                            jt->parent_to_joint_origin_transform.position.y,
                                            jt->parent_to_joint_origin_transform.position.z);
    std::cout << "Trans: " << trans(0) << " " << trans(1) << " " << trans(2) << std::endl;
    
    Eigen::Vector4f quat = Eigen::Vector4f( jt->parent_to_joint_origin_transform.rotation.w,
                                            jt->parent_to_joint_origin_transform.rotation.x,
                                            jt->parent_to_joint_origin_transform.rotation.y,
                                            jt->parent_to_joint_origin_transform.rotation.z );
    
    std::cout << "For joint :" << _jointName << std::endl;
    std::cout << "Their quat [" << quat(0) << ", " << quat(1) << ", " << quat(2) << ", " << quat(3) << "]" << std::endl;
    HomTransform ht;
    ht.setRotationFromQuaternion(quat);
    ht.setTranslation(trans);
    std::cout << "My quat: [" << ht.getQuaternion().transpose() << "]" << std::endl;
    std::cout << "My RotationM: " << ht.getRotation() << std::endl << std::endl;
    tf::Quaternion quatt;
    
    Eigen::Vector4f angleAxis = ht.getAngleAxis();
    Eigen::Vector3f v_diff = Eigen::Vector3f(jt->axis.x, jt->axis.y, jt->axis.z) - angleAxis.bottomRows(3);
    if( v_diff.norm()< 0.00001)
        this->jv_origin.push_back(angleAxis(0));
    else
        this->jv_origin.push_back(angleAxis(0)*-1);
    quatt.setRotation(tf::Vector3(angleAxis(1), angleAxis(2), angleAxis(3)), angleAxis(0));
    std::cout << "tf quat: {" << quatt.w() << ", " << quatt.x() << ", " << quatt.y() << "," << quatt.z() << "]\n";
    return ht;
}

void FKParser::sortLinks(){
    
    std::vector<Link> sorted_links;
    std::map<int, int> sortingMap;
    int j = 0;
    /*Find root*/
    for(int i = 0; i < this->links.size(); i++){
        if(this->links[i].getParent() == NULL){
            sortingMap[j] = i;
            j++;
        }
            
    }
    if(sortingMap.size() > 1)
        ROS_ERROR("There are multiple root links in this parse tree :(");
    
    
    while(j < this->links.size()){
        for(int k = 0; k < this->links.size(); k++){
                if(this->links[sortingMap[j-1]].getChild() != NULL){
                    if(this->links[k].getParent()->getName() == this->links[sortingMap[j-1]].getChild()->getName()){
                        sortingMap[j] = k;
                        j++;
                        break;
                    }
                }
        }
    }
    
    for(int k = 0; k < sortingMap.size(); k++){
        sorted_links.push_back(this->links[sortingMap[k]]);
    }
    this->links.swap(sorted_links);
    
}

int FKParser::findJointIndex(std::string _jointName){
    for (int i = 0; i < this->joints.size(); i++){
        if(this->joints[i].getName() == _jointName)
            return i;
    }
    std::cout << _jointName << " was not found in joints" << std::endl;
    return -1;
}


bool FKParser::jointExists(urdf::JointSharedPtr ptr){
    if(ptr != NULL)
        return 1;
    return 0;
}

Joint::JointType FKParser::translateType(urdf::JointSharedPtr jointPtr){
    if(jointPtr->type == urdf::Joint::PRISMATIC)
        return Joint::Prismatic;
    if(jointPtr->type == urdf::Joint::REVOLUTE)
        return Joint::Revolute;
    if(jointPtr->type == urdf::Joint::FIXED)
        return Joint::Fixed;
    return Joint::Unknown;
    
}

std::vector<Link> FKParser::getLinks(){
    return this->links;
}