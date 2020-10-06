/* 
 * File:   main.cpp
 * Author: Anton Söderlund
 *
 * Created on September 6, 2020, 1:53 PM
 */

#include <cstdlib>
#include <eigen3/Eigen/Core>
#include "HomTra.h"

void testFunction();

int main(int argc, char** argv) {
    //Eigen::Matrix3f R = Eigen::Matrix3f::Random();
    //Eigen::Vector3f t = Eigen::Vector3f::Random();
    
    Eigen::Matrix3f R = Eigen::Matrix3f();
    R(0,0) = 0; R(0,1) = -1; R(0,2) = 0;
    R(1,0) = 1; R(1,1) = 0; R(1,2) = 0;
    R(2,0) = 0; R(2,1) = 0; R(2,2) = 1;
    
    Eigen::Vector3f t = Eigen::Vector3f(0.17322050, 0.2, 0);
    
    HomTransform transform = HomTransform::fromRotationAndTranslation(R, t);
    //HomTransform transform2 = HomTransform::fromRotationAndTranslation(R, t);
    //transform.print();
    
    transform = transform.fromZYXEulerAngles(0.1, 0.2 ,0.3);
    //HomTransform tt = transform*transform2;
    
    Eigen::Vector3f tv = Eigen::Vector3f(0.3, 0.1, 0);
    tv = transform.inverse()*tv;
    
    Eigen::Vector3f axis = Eigen::Vector3f(1,0,0);
    
    transform.setRotationFromAngleAxis(-M_PI/2,axis);
    //std::cout << transform.getAngleAxis();
    
    std::cout << transform.getQuaternion();
    
    //transform.print();
    //tt.print();
    
    //testFunction();

    return 0;
}

void testFunction(){
    Eigen::Matrix3f R = Eigen::Matrix3f();
    R(0,0) = 0; R(0,1) = -1; R(0,2) = 0;
    R(1,0) = 1; R(1,1) = 0; R(1,2) = 0;
    R(2,0) = 0; R(2,1) = 0; R(2,2) = 1;
    
    Eigen::Vector3f t = Eigen::Vector3f(0.17322050, 0.2, 0);
    
    
}