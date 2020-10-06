/* 
 * File:   HomTra.h
 * Author: anton
 *
 * Created on September 6, 2020, 2:54 PM
 */

#ifndef HOMTRA_H
#define HOMTRA_H

#include <eigen3/Eigen/Core>
#include <iostream>
#include <cmath>

class HomTransform{
private:
    Eigen::Matrix4f m_transform;
    Eigen::Matrix3f m_rotation;
    Eigen::Vector3f v_translation;
    
public:
    
    HomTransform(){
        this->m_transform = Eigen::Matrix4f::Zero(4,4);
        this->m_rotation = Eigen::Matrix3f::Zero(3,3);
        this->v_translation = Eigen::Vector3f(0,0,0);
        this->m_transform(3,3) = 1.0f;
    }
    static HomTransform fromRotationAndTranslation(Eigen::Matrix3f R, Eigen::Vector3f t);
    static HomTransform fromZYXEulerAngles(double za, double ya, double xa);
    
    HomTransform operator*(const HomTransform &other);
    Eigen::Vector3f operator*(const Eigen::Vector3f &other);
    
    HomTransform inverse();
    
    HomTransform identity();
        
    void setRotation(Eigen::Matrix3f r);
    void setRotationFromAngleAxis(double angle, Eigen::Vector3f axis);
    void setTranslation(Eigen::Vector3f t);
    void setTransformation(Eigen::Matrix4f t);
    
    Eigen::Matrix3f getRotation();
    Eigen::Vector3f getTranslation();
    Eigen::Matrix4f getTransformation();
    Eigen::Vector3f getRPY();
    Eigen::Vector3f getRPY(HomTransform transform);
    Eigen::Vector4f getAngleAxis();
    Eigen::Vector4f getQuaternion();
    void print();
};



#endif /* HOMTRA_H */

