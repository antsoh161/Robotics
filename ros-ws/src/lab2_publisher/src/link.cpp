#include <lab2_publisher/link.h>

Link::Link(std::string _name, Joint *_parent, Joint *_child) :
    name(_name),    
    parent(_parent),
    child(_child)
    {
    }

Link Link::CreateLink(std::string _name, Joint *_parent, Joint *_child){
    Link link(_name, _parent, _child);
    HomTransform frame; //Todo
    return link;
}

void Link::updateFrameFromDHDisplacement(double d){
    this->frame.updateFromDH_D(d);
}
void Link::updateFrameFromDHAngle(double theta){
    this->frame.updateFromDHTheta(theta);
}

void Link::setName(std::string _name){
    this->name = _name;
}

void Link::setLength(float _length){
    this->length=_length;
}

void Link::setChild(Joint _child){
    this->child = &_child;
}

void Link::setParent(Joint _parent){
    this->parent = &_parent;
}

void Link::linkJoints(Joint _parent, Joint _child){
    this->parent = &_parent;
    this->child = &_child;
}

void Link::linkTip(Joint _parent){
    this->parent = &_parent;
}

void Link::setFrame(HomTransform _frame){
    this->frame = _frame;
}

void Link::setPose(Eigen::Vector3f _position, Eigen::Vector4f _quat){
    this->pose.position = _position;
    this->pose.quaternion = _quat;
}

std::string Link::getName(){
    return this->name;
}

Pose Link::getPose(){
    return this->pose;
}

float Link::getLength(){
    return this->length;
}
Joint *Link::getParent(){
    return this->parent;
}
Joint *Link::getChild(){
    return this->child;
}

HomTransform Link::getFrame(){
    return this->frame;
}

Eigen::Vector3f Link::getOffset(){
    return this->offset;
}

/*
void Chain::addJointToEnd(Joint tip){
    this->chain.push_back(tip);
    this->nrJoints++;
}

void Chain::removeTip(){
    this->chain.pop_back();
    this->nrJoints--;
}
void Chain::reBase(Joint base){
    this->chain.insert(this->chain.begin(), base);
    this->nrJoints++;
}
void Chain::removeJointAt(int i){
    this->chain.erase(this->chain.begin() + i);
    this->nrJoints--;
}

void Chain::addJointAt(int i, Joint joint){
    this->chain.insert(this->chain.begin() + i, joint);
    this->nrJoints++;
}

void Chain::printChain(int verbose){
    std::vector<Joint>::iterator it;
    if(verbose == 0){
    for(it = this->chain.begin(); it != this->chain.end(); it++){
        std::cout << it->getName();
        if(it != this->chain.end() - 1)
            std::cout << "----";
    }
    std::cout << std::endl;
    }
    if(verbose == 1){
        for(it = this->chain.begin(); it != this->chain.end(); it++){
        std::cout << it->getName() << std::endl << "";
           
    }
        
    }
    
}
 */