/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief dynamic-graph plugin that allow you to offset a SE3 value.
 */

#include <dynamic-graph/all-commands.h>  // Commands
#include <dynamic-graph/factory.h>       // DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN

#include <dg_tools/se3_offset.hpp>
#include <pinocchio/math/rpy.hpp>

#include <iostream>

namespace dg_tools
{
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SE3Offset, "SE3Offset");

SE3Offset::SE3Offset(const std::string& name)
    : dynamicgraph::Entity(name),
      offest_sin_(
          NULL,
          "SE3Offset(" + name + ")::input(xyzquat_vector)::se3_offset_sin"),
      se3_sin_(NULL, "SE3Offset(" + name + ")::input(xyzquat_vector)::se3_sin"),
      xyzquat_sout_(boost::bind(&SE3Offset::output_callback, this, _1, _2),
                    offest_sin_,
                    "SE3Offset(" + name + ")::output(xyzquat_vector)::se3_sout")
{
    // Register signals.
    this->signalRegistration(offest_sin_ << se3_sin_ << xyzquat_sout_);

    // Register commands.
    this->addCommand("update_offset",
                     dynamicgraph::command::makeCommandVoid0(
                         *this,
                         &SE3Offset::update_offset,
                         dynamicgraph::command::docCommandVoid0(
                             "At the next iteration the offset will "
                             "take the input signal value.")));

    this->addCommand("set_which_dofs",
                     dynamicgraph::command::makeCommandVoid1(
                         *this,
                         &SE3Offset::set_which_dofs,
                         dynamicgraph::command::docCommandVoid1(
                             "Set which dofs are going to be offsetted "
                             "[x, y, z, roll, pitch, yaw].",
                             "vector (of 1 or 0 representing the dof to "
                             "offset [x, y, z, roll, pitch, yaw])")));

    // Initialize internal variables.
    do_offset_ = false;
    offset_.setIdentity();
    quaternion_.setIdentity();
    xyz_.setZero();
    rpy_.setZero();
    se3_input_.setIdentity();
    se3_output_.setIdentity();
    which_dofs_.setZero(6);
}

void SE3Offset::update_offset()
{
    do_offset_ = true;
}

void SE3Offset::set_which_dofs(const dynamicgraph::Vector& which_dofs)
{
    if (which_dofs.size() != 6)
    {
        printf("The input 'which_dofs' is not a vector of dim 6.");
        return;
    }
    bool zeros_or_ones = true;
    for (Eigen::Index i = 0; i < 6; ++i)
    {
        zeros_or_ones =
            zeros_or_ones && ((which_dofs[i] == 1) || (which_dofs[i] == 0));
    }
    if (!zeros_or_ones)
    {
        printf("The input 'which_dofs' does not contain only 0 or 1.");
        return;
    }

    which_dofs_ = which_dofs;
}

dynamicgraph::Vector& SE3Offset::output_callback(dynamicgraph::Vector& out,
                                                 int time)
{
    // Extract offset when needed.
    if (do_offset_)
    {
        const dynamicgraph::Vector& offset_value = offest_sin_.access(time);

        xyz_ = offset_value.head<3>();
        quaternion_.x() = offset_value(3);
        quaternion_.y() = offset_value(4);
        quaternion_.z() = offset_value(5);
        quaternion_.w() = offset_value(6);
        rpy_ = pinocchio::rpy::matrixToRpy(quaternion_.toRotationMatrix());
        rpy_ = rpy_.array() * which_dofs_.tail<3>().array();

        // set the offset.
        offset_.translation() = xyz_.array() * which_dofs_.head<3>().array();
        offset_.rotation() = pinocchio::rpy::rpyToMatrix(rpy_);

        do_offset_ = false;
    }

    // Extract SE3 input.
    const dynamicgraph::Vector& xyzquat_input = se3_sin_.access(time);
    quaternion_.x() = xyzquat_input(3);
    quaternion_.y() = xyzquat_input(4);
    quaternion_.z() = xyzquat_input(5);
    quaternion_.w() = xyzquat_input(6);
    se3_input_.translation() = xyzquat_input.head<3>();
    se3_input_.rotation() = quaternion_.toRotationMatrix();

    // Apply offset.
    se3_output_ = offset_.actInv(se3_input_);

    // extract output and return.
    if (out.size() != 7)
    {
        out.setZero(7);
    }
    out.head<3>() = se3_output_.translation();
    quaternion_ = se3_output_.rotation();
    out(3) = quaternion_.x();
    out(4) = quaternion_.y();
    out(5) = quaternion_.z();
    out(6) = quaternion_.w();


    std::cout << "\noffset_\n" << offset_ << std::endl;
    std::cout << "in_\n" << se3_input_ << std::endl;
    std::cout << "out_\n" << se3_output_ << std::endl;
    return out;
}

}  // namespace dg_tools