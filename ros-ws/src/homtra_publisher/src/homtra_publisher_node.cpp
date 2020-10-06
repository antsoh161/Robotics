#include <homtra_publisher/homtra_publisher_node.h>
#include <homtra_publisher/homtra.h>

void unitTesting();

int main(int argc, char* argv[]){
    
    ros::init(argc, argv, "homtra_publisher");
    HomtraPubNode _ht_node;
    
    HomTransform frame1 = HomTransform::fromRandom();
    //frame1.print();
    HomTransform frame2 = HomTransform::fromRandom();
    //frame2.print();
    /*Bring frame 1 onto frame 2*/
    HomTransform frame12 = frame2.inverse()*frame1;
    //frame12.print();
    
    /*Origin of frame1 expressed in frame2*/
    //Eigen::Vector3f o = frame2.inverse()*frame1*Eigen::Vector3f(0,0,0);

    HomTransform f1 = HomTransform::fromRandom();
    HomTransform f2 = f1;
    f1.print();
    f2.print();
    
    
    /*tf transforms*/
    tf::Transform frame1_tf, frame2_tf, frame12_tf;
    
    frame1_tf = HomTransform::homtraToTF(frame1);
    frame2_tf = HomTransform::homtraToTF(frame2);
    frame12_tf = HomTransform::homtraToTF(frame12);
    
    
    Eigen::Vector4f angleAxis = frame12.getAngleAxis();
    double angle = angleAxis(0);
    
    std::vector<tf::Transform> samples;
    int num_samples = 10;
    
    Eigen::Vector3f frame12_trans = frame12.getTranslation();
    Eigen::Vector3f tmp_trans;
    HomTransform tmp_frame;
    
    for(int i = 0; i < num_samples; i++){
        double a = i * angle/(double)num_samples;
        tmp_trans(0) = i * frame12_trans(0)/(double)num_samples;
        tmp_trans(1) = i * frame12_trans(1)/(double)num_samples;
        tmp_trans(2) = i * frame12_trans(2)/(double)num_samples;
        
        tmp_frame.setRotationFromAngleAxis(a, angleAxis.bottomRows(3));
        tmp_frame.setTranslation(tmp_trans);
        samples.push_back(HomTransform::homtraToTF(tmp_frame));
        if(i == 0)
            tmp_frame.print();
    }
    
    unitTesting();
    
    std::string frame_name = "frame";
    
    while(ros::ok()){
        _ht_node.publishTransform(frame1_tf,"world", "frame1");
        _ht_node.publishTransform(frame2_tf, "world", "frame2");
        //_ht_node.publishTransform(frame12_tf, "frame2", "frame12");
        
        for(int i = 0; i < num_samples; i++){
            std::string frame_name = "stepFrame";
            frame_name += std::to_string(i);
            _ht_node.publishTransform(samples.at(i),"frame2",frame_name);
        }
        ros::spinOnce();
    }
    ros::shutdown();
    
    return 0;
}

double doubleRand(double a, double b){
    std::default_random_engine generator;
     std::random_device rd;
     generator.seed(rd());
     std::uniform_real_distribution<double> distr(a, b);
     return distr(generator);
}

void HomtraPubNode::publishTransform(tf::Transform t, std::string frame_id, std::string child_frame){
    this->tf_broadcaster.sendTransform(tf::StampedTransform(t, ros::Time::now(), frame_id,child_frame));
}

void printComparison(std::string matrixType, Eigen::MatrixXf a, Eigen::MatrixXf b){
    std::cout << "Comparing: " << matrixType << std::endl;
    
    if(HomTransform::isEqual(a,b))
        std::cout << matrixType << " matrices are exactly equivalent";
    else if(HomTransform::isApprox(a,b))
        std::cout << matrixType << " matrices are approximately equivalent";
    else
        std::cout << matrixType << " are not equivalent";
    std::cout << std::endl << "----------------" << std::endl;;
            
}

void unitTesting(){
    std::cout << "Running unit tests" << std::endl;
    /*Random translation and rotation*/
    float angle = doubleRand(0, 2*M_PI);
    Eigen::Vector3f trans = Eigen::Vector3f(doubleRand(0,1), doubleRand(0,1), doubleRand(0,1));
    
    /*Build eigen transform*/
    Eigen::Transform<float, 3, Eigen::Affine> eigen_t;
    eigen_t = Eigen::Translation<float, 3>(trans); 
    
    /*My Transform*/
    HomTransform homtra_t;
    homtra_t.setTranslation(trans);
    
    /* Angle axis comparison*/
    Eigen::Vector3f axis = Eigen::Vector3f::Random();
    axis.normalize();
    
    eigen_t.rotate(Eigen::AngleAxis<float>(angle,axis));
    homtra_t.setRotationFromAngleAxis(angle,axis);
    
    printComparison("Translation",homtra_t.getTranslation(), eigen_t.translation().matrix() );
    printComparison("Rotation", homtra_t.getRotation(), eigen_t.rotation().matrix() );
    printComparison("Angle axis rotation", homtra_t.getRotation(), eigen_t.rotation().matrix().block(0,0,3,3));
    /*Inverse*/
    HomTransform homtra_tinv = homtra_t.inverse();
    printComparison("Inverse of transformation ", homtra_tinv.getTransformation(), eigen_t.matrix().inverse());
    /*Identity*/
    printComparison("Identity", homtra_t.identity().getTransformation(), eigen_t.Identity().matrix());
    
    /*Quaternion*/
    Eigen::Quaternion<float> eigen_quat;
    eigen_quat = Eigen::AngleAxis<float>(angle,axis);
    Eigen::Vector4f eigen_quatv = Eigen::Vector4f(eigen_quat.w(), eigen_quat.x(), eigen_quat.y(), eigen_quat.z());
    Eigen::Vector4f homtra_quat = homtra_t.getQuaternion();
    printComparison("Quaternion", eigen_quatv, homtra_quat);
    printComparison("Quaternion2",-1*eigen_quatv,homtra_quat);
    
    /*From ZYX euler angles*/
    Eigen::Vector3f euler_angles =  Eigen::Vector3f(doubleRand(0,2*M_PI), doubleRand(0,2*M_PI), doubleRand(0,2*M_PI));
    homtra_t = HomTransform::fromZYXEulerAngles(euler_angles(0), euler_angles(1), euler_angles(2));
    Eigen::Matrix3f eigen_euler;
    eigen_euler = Eigen::AngleAxis<float>(euler_angles(0),Eigen::Vector3f::UnitZ())
            * Eigen::AngleAxis<float>(euler_angles(1),Eigen::Vector3f::UnitY())
            * Eigen::AngleAxis<float>(euler_angles(2),Eigen::Vector3f::UnitX());
    printComparison("From ZYX Angles", homtra_t.getRotation(), eigen_euler);
    
    
    
    /**/
}

