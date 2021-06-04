/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2021, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Utilities used accross this package.
 */

#include <sstream>
#include "dg_tools/utils.hpp"

namespace dg_tools
{
Eigen::VectorXd se3_to_xyzquat(const pinocchio::SE3& in)
{
    Eigen::VectorXd out = Eigen::VectorXd::Zero(7);
    // Extract position.
    out.head<3>() = in.translation();
    // Extract orientation.
    Eigen::Quaterniond quaternion = Eigen::Quaterniond(in.rotation());
    quaternion.normalize();
    out(3) = quaternion.x();
    out(4) = quaternion.y();
    out(5) = quaternion.z();
    out(6) = quaternion.w();
    return out;
}

pinocchio::SE3 xyzquat_to_se3(Eigen::Ref<const Eigen::VectorXd> in)
{
    if(in.size() != 7)
    {
        std::ostringstream oss;
        oss << "dg_tools::se3_to_xyzquat(): "
            << "input size ("
            << in.size()
            << ") is wrong, must be 7.";
        throw std::runtime_error(oss.str());
    }

    // create the output.
    pinocchio::SE3 out;
    
    // extract the orientation.
    pinocchio::SE3::Quaternion quat;
    quat.x() = in(3);
    quat.y() = in(4);
    quat.z() = in(5);
    quat.w() = in(6);
    quat.normalize();
    // convert to pinocchio::SE3
    out.translation() = in.head<3>();
    out.rotation() = quat.toRotationMatrix();
    return out;
}

}  // namespace dg_tools