#include <iostream>
#include <vector>
#include <urdf_parser/urdf_parser.h>
#include <urdf/model.h>
#include "link.h"
#include "joint.h"
#include "FKParser.h"
#ifndef CHAIN_H
#define CHAIN_H

class Chain{
private:
    std::string name;
    std::vector<Link> chain;
    std::vector<HomTransform*> homtra_transforms;
    std::vector<double> jointValueOrigins;
    FKParser parser;
    HomTransform baseToEndEffector;
    int nrJoints;
    int nrLinks;
public:
    Chain(std::string _name="");
    static Chain fromUrdf(std::string urdfParam);
    std::string getName();
    void addLinkToEnd(Link tip, Eigen::Vector3f origin=Eigen::Vector3f::Zero());
    void updateJoint(std::string _jointName, double _jointValue);
    void removeTip();
    void reBase(Link base);
    void removeLinkAt(int i);
    void addLinkAt(int i, Link joint);
    void runFK();
    void printChain(int verbose=0);
    void swapChain(std::vector<Link> _links);
    void parseUrdf(std::string urdfParam);
        std::vector<HomTransform*> getAllFrames();
    HomTransform getEndEffectorTransform();
    double getJointValueOrigin(int i);
};

#endif /* CHAIN_H */