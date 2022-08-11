//
// Created by haochen on 22/08/11.
//
#include "src/trac_ik.hpp"

int main() {
    std::cout << "TEST" << std::endl;
    TRAC_IK::TRAC_IK iksolve("yumi_body", "yumi_link_7_r",
                             "D:\\chen\\dev\\trac_ik_build\\test\\yumi.urdf",
                             0.005,
                             1e-5,
                             TRAC_IK::Speed);
    KDL::JntArray result;
    KDL::JntArray q(7);
    q(0) = -0.34906585;
    q(1) = -1.57079633;
    q(2) = -2.0943951;
    q(3) = 0.52359878;
    q(4) = 0.;
    q(5) = 0.6981317;
    q(6) = 0.;


    auto r = iksolve.CartToJnt(
            q,
            KDL::Frame(KDL::Rotation::Quaternion(0.62807112, -0.75853344, 0.11074588, 0.13374991),
                       KDL::Vector(-0.02743993, -0.33416814, 0.26049456)),
            result);
    std::cout << "result" << r << " | " << result(0) << "," << result(1) << "," << result(2) << "," << result(3) << "," << result(4) << ","
              << result(5) << "," << result(6)
              << std::endl;
}

//np.array([0.11055943340536382,-1.7745341313644831,-2.088530406574881,0.5317516210362633,-0.22859509185686763,-0.4601651803822982,-0.4273526410371647,])
//0.110557,-1.77454,-2.08853,0.531752,-0.228595,-0.460168,-0.427349