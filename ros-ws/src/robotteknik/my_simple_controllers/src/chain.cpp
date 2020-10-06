#include <my_simple_controllers/chain.h>


Chain::Chain(std::string _name) : 
    name(_name),
    nrJoints(0),
    nrLinks(0)
    {
    }

Chain Chain::fromUrdf(std::string urdfParam){
    Chain *chain = new Chain("Chain");
    chain->parseUrdf(urdfParam);
    chain->chain.swap(chain->parser.links);
    for(int i = 0; i < chain->chain.size(); i++){
        HomTransform *ptr = &chain->chain[i].frame;
        chain->homtra_transforms.push_back(ptr);
        chain->jointValueOrigins = chain->parser.jv_origin;
    }
    chain->runFK();
    return *chain;
}

void Chain::parseUrdf(std::string urdfParam){
    this->parser = FKParser(urdfParam);
    this->parser.parse();
}


void Chain::swapChain(std::vector<Link> _links){
    this->chain.swap(_links);
}

void Chain::addLinkToEnd(Link tip,  Eigen::Vector3f origin){
    HomTransform A_to_B;
    A_to_B = A_to_B.identity();
    A_to_B.setTranslation(origin);
    
    std::vector<Link>::iterator it;
    for(it = this->chain.begin(); it != this->chain.end(); it++){
        A_to_B = A_to_B * it->getFrame();    
        
    }
    
    Eigen::Vector3f position = A_to_B * Eigen::Vector3f::Zero();
    Eigen::Vector4f quat = A_to_B.getQuaternion();
    
    std::cout << "-------" <<std::endl << position << std::endl << quat << std::endl;
    
    /*Global pose*/
    tip.setPose(position, quat);
    
    this->chain.push_back(tip);
    this->nrLinks++;
}

void Chain::removeTip(){
    this->chain.pop_back();
    this->nrLinks--;
}
void Chain::reBase(Link base){
    this->chain.insert(this->chain.begin(), base);
    this->nrLinks++;
}
void Chain::removeLinkAt(int i){
    this->chain.erase(this->chain.begin() + i);
    this->nrLinks--;
}

void Chain::addLinkAt(int i, Link link){
    this->chain.insert(this->chain.begin() + i, link);
    this->nrLinks++;
}

void Chain::printChain(int verbose){
    std::cout << "Size of chain: " << this->chain.size() << std::endl;
    std::vector<Link>::iterator it;
    Joint *p, *c;
    if(verbose == 0){
        for(it = this->chain.begin(); it != this->chain.end(); it++){
            p = it->getParent();
            c = it->getChild();
            if(p != NULL)
                std::cout << "\tParent: " << p->getName() << std::endl;
            std::cout << "\t| " << "\n\t| " << it->getName() << "\n\tv\n";
            if(c != NULL)
                std::cout << "\tChild: " << c->getName() << std::endl;
        }
    }
    std::cout << std::endl;
    
    std::cout << "Poses\n";
    for(int i = 0; i < this->chain.size(); i++){
        Pose p = this->chain[i].getPose();
        std::cout << "Pose of link " << this->chain[i].getName() << std::endl;
        std::cout << p.position.transpose() << std::endl;
        std::cout << p.quaternion.transpose() << std::endl;
    }
}

void Chain::updateJoint(std::string _jointName, double _jointValue){
    Link tmp_link;
    for(int i = 0; i < this->chain.size(); i++){
        //tmp_link = this->chain[i];
        if(chain[i].getChild() != NULL){
            if(chain[i].getChild()->getName() == _jointName){
                chain[i].updateFromJointValueAxis(_jointValue, chain[i].getChild()->getActuationAxis(), chain[i].getChild()->getType());
                //chain[i].frame = *this->origin_transforms[i] * chain[i].frame; //Apply origin because urdf_parser doesnt ???
                this->homtra_transforms.at(i) = &chain[i].frame;
            }
        }
    }
}

void Chain::runFK(){
    HomTransform A_to_B;
    A_to_B = A_to_B.identity();
//    std::vector<Link>::iterator it;
//    for(it = this->chain.begin(); it != this->chain.end(); it++){
//        A_to_B = A_to_B * it->getFrame();
//        //A_to_B.setTranslation(it->getOffset());
//        Eigen::Vector3f position = A_to_B * Eigen::Vector3f::Zero();
//        Eigen::Vector4f quat = A_to_B.getQuaternion();
//        it->setPose(position, quat);
//    }
    
    for(int i = 0; i < this->homtra_transforms.size(); i++){
        A_to_B = A_to_B * *this->homtra_transforms[i];
        Eigen::Vector3f pos = A_to_B * Eigen::Vector3f::Zero();
        Eigen::Vector4f quat = A_to_B.getQuaternion();
        this->chain[i].setPose(pos,quat);
    }
    //this->baseToEndEffector = A_to_B;
}

HomTransform Chain::getEndEffectorTransform(){
    //return this->baseToEndEffector;
    return *(this->homtra_transforms.back());
}

std::vector<HomTransform*> Chain::getAllFrames(){
    return this->homtra_transforms;
}
std::string Chain::getName(){
    return this->name;
}

double Chain::getJointValueOrigin(int i){
    return this->jointValueOrigins[i];
}