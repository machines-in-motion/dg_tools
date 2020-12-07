/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief dynamic-graph plugin that allow you to offset a SE3 value.
 */

#pragma once

#include <dynamic-graph/entity.h>                 // Entity.
#include <dynamic-graph/linear-algebra.h>         // Vector and Matrix.
#include <dynamic-graph/signal-ptr.h>             // Input signals.
#include <dynamic-graph/signal-time-dependent.h>  // Output signals.

#include <pinocchio/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace dg_tools
{
/**
 * @brief This class has for purpose to create an SE3 frame from an existing
 * one zeroing some of the component of this frame. For example when you want
 * to start a robot at a certain position se3_vicon you want to acquire the
 * vicon position at the starting popint and extract the position and yaw of
 * frame and zero the roll and pitch. This entity extract the information and
 * spit out an XYZQUAT vector from the input data.
 *
 */
class CreateWorldFrame : public dynamicgraph::Entity
{
public:
    /**
     * @brief Construct a new CreateWorldFrame object
     *
     * @param name
     */
    CreateWorldFrame(const std::string& name);

    /// @brief Name of the plugin class.
    static const std::string CLASS_NAME;

    /**
     * @brief Get Class Name.
     *
     * @return const std::string&
     */
    virtual const std::string& getClassName(void) const
    {
        return CLASS_NAME;
    }

    /**
     * @brief At the next iteration, update the projection with the input
     * signal.
     */
    void update();

    /**
     * @brief Set which dofs are going to be zeroed from the offset [x, y, z,
     * roll, pitch, yaw].
     *
     * @param which_dofs is a set of 6 flags. 0 means no offset is applied in
     * the direction. 1 means the offset will apply. No other value will be
     * accepted. The flags are order like so: [x, y, z, roll, pitch, yaw]
     */
    void set_which_dofs(const dynamicgraph::Vector& which_dofs);

public:  // Signals.
    /// @brief Input signal that will offset the output.
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> frame_sin_;

    /// @brief Output signal.
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> world_frame_sout_;

private:  // methods
    /**
     * @brief Output signal callback function.
     *
     * @param output_torque
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& output_callback(dynamicgraph::Vector& out, int time);

private:  // Attributes.
    /// @brief Check when we should update the internal offset_ object.
    std::atomic_bool do_offset_;

    /// @brief Internal offset.
    pinocchio::SE3 offset_;

    /// @brief Intermediate variable.
    Eigen::Quaterniond quaternion_;

    /// @brief Intermediate variable.
    Eigen::Vector3d xyz_;

    /// @brief Intermediate variable.
    Eigen::Vector3d rpy_;

    /// @brief SE3 from xyzquat input.
    pinocchio::SE3 se3_input_;

    /// @brief SE3 from xyzquat input.
    pinocchio::SE3 se3_output_;

    /// @brief Offset direction. Determine which dof.
    dynamicgraph::Vector which_dofs_;
};

}  // namespace dg_tools