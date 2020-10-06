/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <homtra_publisher/homtra.h>

 HomTransform HomTransform::fromRotationAndTranslation(Eigen::Matrix3f R, Eigen::Vector3f t){
        HomTransform *ht = new HomTransform();
        ht->m_rotation = R;
        ht->v_translation = t;
        ht->m_transform.topLeftCorner(3,3) = R;
        ht->m_transform.topRightCorner(3,1) = t;
        ht->m_transform(3,3) = 1.0f;
        return *ht;
    } 
 
 HomTransform HomTransform::fromRandom(){

     std::default_random_engine generator;
     std::random_device rd;
     generator.seed(rd());
     std::uniform_real_distribution<double> angle_distr(0, 2*M_PI);
     std::uniform_real_distribution<double> trans_distr(0.0,1.0);
     
     double za = angle_distr(generator);
     double ya = angle_distr(generator);
     double xa = angle_distr(generator);
     
     auto trans = std::bind(trans_distr, generator);
     
     Eigen::Vector3f translation;
     translation(0) = trans_distr(generator);
     translation(1) = trans_distr(generator);
     translation(2) = trans_distr(generator);
     
     HomTransform ht = HomTransform::fromZYXEulerAngles(za,ya,xa);
     ht.setTranslation(translation);

     //std::cout << "Randomed euler angles: [" << za << ", " << ya << ", " << xa << "]" << std::endl;
     //std::cout << "Randomed translation: " << translation << std::endl;
     
     return ht;
 }

void HomTransform::setRotationFromAngleAxis(double angle, Eigen::Vector3f axis){
    Eigen::Matrix3f rotation;
    
    Eigen::Matrix3f rrt = axis*axis.transpose();
     
    Eigen::Matrix3f I_rrt = ( this->identity().getRotation() - rrt)*cos(angle);
     
     Eigen::Matrix3f skewSymmAxis = Eigen::Matrix3f::Zero(3,3);
     
     skewSymmAxis(0,1) = -1*axis(2);
     skewSymmAxis(1,0) = axis(2);
     skewSymmAxis(0,2) = axis(1);
     skewSymmAxis(2,0) = -1*axis(1);
     skewSymmAxis(1,2) = -1*axis(0);
     skewSymmAxis(2,1) = axis(0);
     
     rotation = rrt + I_rrt + skewSymmAxis*sin(angle);
     
     this->setRotation(rotation);
     
 }
/*Luca lecture 10, time 50:58*/
/*Returns vector4f = [angle, [x,y,z]]*/
Eigen::Vector4f HomTransform::getAngleAxis(){
    Eigen::Vector3f axis;
    double rx_diff, ry_diff, rz_diff;
    rz_diff = this->m_rotation(1,0) - this->m_rotation(0,1);
    rz_diff *= rz_diff;
    
    ry_diff = this->m_rotation(0,2) - this->m_rotation(2,0);
    ry_diff *= ry_diff;
    
    rx_diff = this->m_rotation(1,2) - this->m_rotation(2,1);
    rx_diff *= rx_diff;
    
    double trace_sum = this->m_rotation(0,0) + this->m_rotation(1,1) + this->m_rotation(2,2) - 1;
    
    double angle = atan2( sqrt(rz_diff + ry_diff + rx_diff), trace_sum );
    
    if(angle != 0){
        double rx,ry,rz;
        rx = this->m_rotation(2,1) - this->m_rotation(1,2);
        ry = this->m_rotation(0,2) - this->m_rotation(2,0);
        rz = this->m_rotation(1,0) - this->m_rotation(0,1);
        double denom = 1/(2*sin(angle));
        axis(0) = denom*rx;
        axis(1) = denom*ry;
        axis(2) = denom*rz;
        axis.normalize();
    }
    else{
        std::cout << "axis = [0,0,0]" << std::endl;
        axis = Eigen::Vector3f(1,0,0); // angle = 0 so axis doesnt matter
    }
    Eigen::Vector4f angleAxis;
    angleAxis << angle, axis;
    return angleAxis;
}

void HomTransform::setRotationFromQuaternion(Eigen::Vector4f quat){
    if(quat(0) == 1 && quat.norm() == 1){
        this->m_rotation = Eigen::Matrix3f::Identity();
        return;
    }
    
    double angle = acos(quat(0))*2.0;
    double denom = 1.0/ (sin(angle/2));
    Eigen::Vector3f axis = denom * quat.bottomRows(3);
    
    this->setRotationFromAngleAxis(angle,axis);
}

Eigen::Vector4f HomTransform::getQuaternion(){
    Eigen::Vector4f angleAxis = this->getAngleAxis();
    double angle = angleAxis(0);
    Eigen::Vector4f quaternion;
    quaternion << angleAxis*sin(angle/2.0);
    quaternion(0) = cos(angle/2.0);
    return quaternion;
}

HomTransform HomTransform::fromZYXEulerAngles(double za, double ya, double xa){
    Eigen::Matrix3f rotation; 
    
    rotation(0,0) = cos(za) * cos(ya);
    rotation(0,1) = cos(za)*sin(ya)*sin(xa) - cos(xa)*sin(za);
    rotation(0,2) = sin(za)*sin(xa) + cos(za)*cos(xa)*sin(ya);
    rotation(1,0) = cos(ya)*sin(za);
    rotation(1,1) = cos(za)*cos(xa) + sin(za)*sin(ya)*sin(xa);
    rotation(1,2) = cos(xa)*sin(za)*sin(ya) - cos(za)*sin(xa);
    rotation(2,0) = -sin(ya);
    rotation(2,1) = cos(ya)*sin(xa);
    rotation(2,2) = cos(ya)*cos(xa);
    
    HomTransform *ht = new HomTransform();
    ht->setRotation(rotation);
    return *ht;
}
 
 
 
 /*Transformation multiplication*/
 HomTransform HomTransform::operator*(const HomTransform &other){
     Eigen::Matrix3f new_rotation = this->m_rotation*other.m_rotation;
     Eigen::Vector3f new_translation = this->m_rotation*other.v_translation + this->v_translation; // R_t * t_ba + t_ab 
 
     return HomTransform::fromRotationAndTranslation(new_rotation,new_translation);
 }
 
 
Eigen::Vector3f HomTransform::operator*(const Eigen::Vector3f &other){
    Eigen::Vector4f v_tmp = Eigen::Vector4f(other(0),other(1),other(2),1.0);
    v_tmp = this->m_transform * v_tmp;
    
    return Eigen::Vector3f(v_tmp(0),v_tmp(1),v_tmp(2));
}
 
 HomTransform HomTransform::inverse(){
     Eigen::Matrix3f R_transposed = this->m_rotation.transpose();
     Eigen::Vector3f new_translation = -1*R_transposed*this->v_translation;
     return HomTransform::fromRotationAndTranslation(R_transposed, new_translation);
 }
 
 HomTransform HomTransform::identity(){
     Eigen::Matrix3f i_rotation = Eigen::Matrix3f::Identity();
     Eigen::Vector3f I_translation = Eigen::Vector3f(0,0,0);
     return HomTransform::fromRotationAndTranslation(i_rotation, I_translation);
 }
 
void HomTransform::setRotation(Eigen::Matrix3f r){
    this->m_rotation = r;
    this->m_transform.topLeftCorner(3,3) = r;
}
void HomTransform::setTranslation(Eigen::Vector3f t){
    this->v_translation = t;
    this->m_transform.topRightCorner(3,1) = t;
}
void HomTransform::setTransformation(Eigen::Matrix4f t){
    this->m_transform = t;
}

Eigen::Matrix3f HomTransform::getRotation(){
    return this->m_rotation;
}
Eigen::Vector3f HomTransform::getTranslation(){
    return this->v_translation;
}
Eigen::Matrix4f HomTransform::getTransformation(){
    return this->m_transform;
}

Eigen::Vector3f HomTransform::getRPY(){
    double roll, pitch, yaw;
    yaw = atan2(this->m_rotation(1,0), this->m_rotation(0,0));
    double square1, square2;
    square1 = this->m_rotation(2,1) * this->m_rotation(2,1);
    square2 = this->m_rotation(2,2) * this->m_rotation(2,2);
    pitch = atan2(-1.0f * this->m_rotation(2,0), sqrt(square1 + square2));
    roll = atan2(this->m_rotation(2,1), this->m_rotation(2,2));
    
    return Eigen::Vector3f(roll,pitch,yaw);
}

void HomTransform::print(){
    std::cout << "------------------------------------------------------" << std::endl;
    std::cout << "Rotation: " << std::endl << this->m_rotation << std::endl;
    std::cout << "Translation: " << std::endl <<  this->v_translation << std::endl;
    std::cout << "Transformation: " << std::endl << this->m_transform << std::endl;
    std::cout << "Inverse: " << std::endl << this->inverse().m_transform << std::endl;
    std::cout << "Identity: " << std::endl << this->identity().m_transform << std::endl;
    std::cout << "RPY: " << std::endl << this->getRPY() << std::endl;
    std::cout << "Quat [w x y z]: " << std::endl << this->getQuaternion() << std::endl;
    std::cout << "AngleAxis [angle, axis]: " << std::endl << this->getAngleAxis() << std::endl;
    std::cout << "------------------------------------------------------" << std::endl;
}

bool HomTransform::isOrthonormal(Eigen::Matrix3f rotation){
    double dot_product = 0;
    Eigen::Vector3f x = rotation.topLeftCorner(3,1);
    Eigen::Vector3f y = rotation.middleCols(1,1);
    Eigen::Vector3f z = rotation.topRightCorner(3,1);
    
    if(x.isOrthogonal(y))
        std::cout << "x // y" << std::endl;
    if(x.isOrthogonal(z))
        std::cout << "x // z" << std::endl;
    if(y.isOrthogonal(z))
        std::cout << "y // <" << std::endl;
    
    return true;
}

tf::Transform HomTransform::homtraToTF(HomTransform ht){
    tf::Transform tf_transform;
    tf::Quaternion quat;
    
    Eigen::Vector3f translation = ht.getTranslation();
    Eigen::Vector4f angleAxis = ht.getAngleAxis();
 
    quat.setRotation(tf::Vector3(angleAxis(1), angleAxis(2), angleAxis(3)),angleAxis(0));
    tf_transform.setRotation(quat);
    tf_transform.setOrigin(tf::Vector3(translation(0), translation(1), translation(2)));
    
    return tf_transform;
    
}

bool HomTransform::isEqual(Eigen::MatrixXf a, Eigen::MatrixXf b){
    a -= b;
    if(a.norm() == 0)
        return true;
    else{
        std::cout << "Norm of a - b: " << a.norm() << std::endl;
        return false;
    }
}

bool HomTransform::isApprox(Eigen::MatrixXf a, Eigen::MatrixXf b){
    if(a.isApprox(b))
        return true;
    else
        return false;
}