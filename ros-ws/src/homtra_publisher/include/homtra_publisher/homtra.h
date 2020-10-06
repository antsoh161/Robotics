/* 
 * File:   HomTra.h
 * Author: anton
 *
 * Created on September 6, 2020, 2:54 PM
 */
#ifndef HOMTRA_H
#define HOMTRA_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <cmath>
#include <random>
#include <tf/tf.h>

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
    static HomTransform fromRandom();
    static tf::Transform homtraToTF(HomTransform ht);
    
    HomTransform operator*(const HomTransform &other);
    Eigen::Vector3f operator*(const Eigen::Vector3f &other);
    
    HomTransform inverse();
    
    HomTransform identity();
        
    void setRotation(Eigen::Matrix3f r);
    void setRotationFromAngleAxis(double angle, Eigen::Vector3f axis);
    void setRotationFromQuaternion(Eigen::Vector4f quat);
    void setTranslation(Eigen::Vector3f t);
    void setTransformation(Eigen::Matrix4f t);
    
    Eigen::Matrix3f getRotation();
    Eigen::Vector3f getTranslation();
    Eigen::Matrix4f getTransformation();
    Eigen::Vector3f getRPY();
    Eigen::Vector3f getRPY(HomTransform transform);
    Eigen::Vector4f getAngleAxis();
    Eigen::Vector4f getQuaternion();
    bool isOrthonormal(Eigen::Matrix3f rotation);
    static bool isEqual(Eigen::MatrixXf a, Eigen::MatrixXf b);
    static bool isApprox(Eigen::MatrixXf a, Eigen::MatrixXf b);
    void print();
};



#endif /* HOMTRA_H */

