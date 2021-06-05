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

#include <dg_tools/create_world_frame.hpp>
#include <iostream>
#include <pinocchio/math/rpy.hpp>

namespace dg_tools
{
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(CreateWorldFrame, "CreateWorldFrame");

CreateWorldFrame::CreateWorldFrame(const std::string& name)
    : dynamicgraph::Entity(name),
      frame_sin_(
          NULL,
          "CreateWorldFrame(" + name + ")::input(xyzquat_vector)::frame_sin"),
      world_frame_sout_(
          boost::bind(&CreateWorldFrame::output_callback, this, _1, _2),
          frame_sin_,
          "CreateWorldFrame(" + name +
              ")::output(xyzquat_vector)::world_frame_sout")
{
    // Register signals.
    this->signalRegistration(frame_sin_ << world_frame_sout_);

    // Register commands.
    this->addCommand("update",
                     dynamicgraph::command::makeCommandVoid0(
                         *this,
                         &CreateWorldFrame::update,
                         dynamicgraph::command::docCommandVoid0(
                             "At the next iteration the world_frame will "
                             "take the input signal value.")));

    this->addCommand("set_which_dofs",
                     dynamicgraph::command::makeCommandVoid1(
                         *this,
                         &CreateWorldFrame::set_which_dofs,
                         dynamicgraph::command::docCommandVoid1(
                             "Set which dofs [x, y, z, roll, pitch, yaw] are "
                             "going to be set to preserved"
                             "from the input frame.",
                             "vector of 1 or 0 representing the dof to "
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

void CreateWorldFrame::update()
{
    do_offset_ = true;
}

void CreateWorldFrame::set_which_dofs(const dynamicgraph::Vector& which_dofs)
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

dynamicgraph::Vector& CreateWorldFrame::output_callback(
    dynamicgraph::Vector& out, int time)
{
    // Extract offset when needed.
    if (do_offset_)
    {
        const dynamicgraph::Vector& offset_value = frame_sin_.access(time);

        xyz_ = offset_value.head<3>();
        quaternion_.x() = offset_value(3);
        quaternion_.y() = offset_value(4);
        quaternion_.z() = offset_value(5);
        quaternion_.w() = offset_value(6);
        quaternion_.normalize();
        rpy_ = pinocchio::rpy::matrixToRpy(quaternion_.toRotationMatrix());
        rpy_ = rpy_.array() * which_dofs_.tail<3>().array();

        // set the offset.
        offset_.translation() = xyz_.array() * which_dofs_.head<3>().array();
        offset_.rotation() = pinocchio::rpy::rpyToMatrix(rpy_);

        do_offset_ = false;
    }

    // extract output and return.
    if (out.size() != 7)
    {
        out.setZero(7);
    }
    out.head<3>() = offset_.translation();
    quaternion_ = offset_.rotation();
    out(3) = quaternion_.x();
    out(4) = quaternion_.y();
    out(5) = quaternion_.z();
    out(6) = quaternion_.w();

    return out;
}

}  // namespace dg_tools