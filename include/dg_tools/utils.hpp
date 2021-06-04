/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2021, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Utilities used accross this package.
 */

#pragma once

#include <pinocchio/spatial/se3.hpp>

namespace dg_tools
{
Eigen::VectorXd se3_to_xyzquat(const pinocchio::SE3& in);

pinocchio::SE3 xyzquat_to_se3(Eigen::Ref<const Eigen::VectorXd> in);

}  // namespace dg_tools