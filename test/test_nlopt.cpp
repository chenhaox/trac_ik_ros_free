//
// Created by haochen on 22/08/11.
//
#include "../trac_ik/nlopt_ik.hpp"
#include <mutex>
#include <memory>
#include <iostream>

std::unique_ptr<KDL::ChainJntToJacSolver> jacsolver;
std::unique_ptr<KDL::ChainIkSolverPos_TL> iksolver;
int main(){
    std::cout<<"test"<<std::endl;
}