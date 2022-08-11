//
// Created by haochen on 22/08/11.
//

#ifndef TRAC_IK_KDL_PARSER_HPP
#define TRAC_IK_KDL_PARSER_HPP

#include <kdl/tree.hpp>
#include <urdf_model/model.h>

namespace kdl_parser {
    bool treeFromUrdfModel(const urdf::ModelInterface &robot_model, KDL::Tree &tree);
}
#endif //TRAC_IK_KDL_PARSER_HPP
