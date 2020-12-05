/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Expose the vicon_client_entity dynamic_graph module to python.
 */

#include "dg_tools/ComImpedanceControl/ComImpedanceController.hpp"
#include "dg_tools/ComImpedanceControl/reactive_lqr_controller.hpp"
#include "dg_tools/control/calibrator.hpp"
#include "dg_tools/control/control_pd.hpp"
#include "dg_tools/data/history_recorder.hpp"
#include "dg_tools/data/memory_replay.hpp"
#include "dg_tools/data/previous_value.hpp"
#include "dg_tools/data/upsampler.hpp"
#include "dg_tools/operator.hpp"
#include "dg_tools/se3_offset.hpp"
#include "dg_tools/smooth-reach.hh"
#include "dg_tools/test_stand_control/power_jump.hpp"

typedef boost::mpl::vector<dynamicgraph::sot::ComImpedanceControl,
                           dynamicgraph::sot::ReactiveLQRController,
                           dynamicgraph::sot::Calibrator,
                           dynamicgraph::sot::PDController,
                           dg_tools::HistoryRecorder,
                           dg_tools::MemoryReplay,
                           dg_tools::PreviousValue,
                           dg_tools::Upsampler,
                           dg_tools::Division_of_double,
                           dg_tools::PoseQuaternionToPoseRPY,
                           dg_tools::PoseRPYToPoseQuaternion,
                           dg_tools::Sinus,
                           dg_tools::VectorIntegrator,
                           dg_tools::SE3Offset,
                           dynamicgraph::sot::SmoothReach,
                           dynamicgraph::sot::PowerJumpControl>
    entities_t;
