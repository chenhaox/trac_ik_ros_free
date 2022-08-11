//
// Created by haochen on 22/08/11.
//

#include "urdf/model.h"
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <exception>
#include "kdl/tree.hpp"
#include <kdl/frames_io.hpp>
#include "kdl_parser.cpp"
#include "../trac_ik/nlopt_ik.hpp"
#include <mutex>
#include <memory>

using namespace urdf;

int main() {
    std::ifstream file("/home/haochen/Desktop/code/src/test/yumi.urdf");

    if (!file.is_open()) {
        std::cout << "cannot open the urdf file" << std::endl;

    }
    std::string xml_string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    std::cout << xml_string << std::endl;
    urdf::Model robot_model;
    auto p = robot_model.initString(xml_string);
    std::cout << p << std::endl;
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
        std::cout << "Failed to extract kdl tree from xml robot description";
    std::string base_link = "yumi_base_link";
    std::string tip_link = "yumi_link_7_r";
    KDL::Chain chain;
    if (!tree.getChain(base_link, tip_link, chain))
        std::cout << "Couldn't find chain " << base_link.c_str() << " to " << tip_link.c_str() << std::endl;
    std::vector<KDL::Segment> chain_segs = chain.segments;

    urdf::JointConstSharedPtr joint;

    std::vector<double> l_bounds, u_bounds;

    KDL::JntArray lb, ub;

    lb.resize(chain.getNrOfJoints());
    ub.resize(chain.getNrOfJoints());

    uint joint_num = 0;
    for (unsigned int i = 0; i < chain_segs.size(); ++i) {
        joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            joint_num++;
            float lower, upper;
            int hasLimits;
            if (joint->type != urdf::Joint::CONTINUOUS) {
                if (joint->safety) {
                    lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
                    upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
                } else {
                    lower = joint->limits->lower;
                    upper = joint->limits->upper;
                }
                hasLimits = 1;
            } else {
                hasLimits = 0;
            }
            if (hasLimits) {
                lb(joint_num - 1) = lower;
                ub(joint_num - 1) = upper;
            } else {
                lb(joint_num - 1) = std::numeric_limits<float>::lowest();
                ub(joint_num - 1) = std::numeric_limits<float>::max();
            }
            std::cout << "src: IK Using joint " << joint->name << " " << lb(joint_num - 1) << " "
                      << ub(joint_num - 1);
        }
    }
    assert(chain.getNrOfJoints() == lb.data.size());
    assert(chain.getNrOfJoints() == ub.data.size());
    std::unique_ptr<KDL::ChainJntToJacSolver> jacsolver;
    jacsolver.reset(new KDL::ChainJntToJacSolver(chain));

}