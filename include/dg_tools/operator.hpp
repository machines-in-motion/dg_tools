/**
 * @file VectorToQuaternion.hpp
 * @author Julian Viereck
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-08-05
 */

#pragma once

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;

/* SOT */
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/entity.h>

#include <sot/core/matrix-geometry.hh>


/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (Operator_EXPORTS)
#    define Operator_EXPORTS __declspec(dllexport)
#  else
#    define Operator_EXPORTS  __declspec(dllimport)
#  endif
#else
#  define Operator_EXPORTS
#endif

namespace dg_tools {

  /* --------------------------------------------------------------------- */
  /* --- CLASS ----------------------------------------------------------- */
  /* --------------------------------------------------------------------- */

  /**
   * @brief Converts PoseQuaternion into PoseRPY data.
   */
  class Operator_EXPORTS PoseQuaternionToPoseRPY: public dg::Entity
  {
    public:

      PoseQuaternionToPoseRPY( const std::string & name );

      static const std::string CLASS_NAME;
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      dg::SignalPtr<dg::Vector,int> data_inputSIN;
      dg::SignalTimeDependent<dg::Vector,int> data_outSOUT;

      dg::Vector& data_out_callback(dg::Vector& history, int time);
  };

  /**
   * @brief Converts Quaternion into RotationMatrixYaw with data.
   */
  class Operator_EXPORTS QuaternionToMatrixYaw: public dg::Entity
  {
    public:

      QuaternionToMatrixYaw( const std::string & name );

      static const std::string CLASS_NAME;
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      dg::SignalPtr<dg::Vector,int> sin_;
      dg::SignalTimeDependent<dg::Matrix,int> sout_;
      dynamicgraph::sot::VectorQuaternion quat_;
      Eigen::Vector3d rpy_;

      dg::Matrix& data_out_callback(dg::Matrix& s, int time);
  };

  /**
   * @brief Converts Quaternion into RotationMatrixYaw with data.
   */
  class Operator_EXPORTS QuaternionToQuaternionYaw: public dg::Entity
  {
    public:

      QuaternionToQuaternionYaw( const std::string & name );

      static const std::string CLASS_NAME;
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      dg::SignalPtr<dg::Vector,int> sin_;
      dg::SignalTimeDependent<dg::Vector,int> sout_;
      dynamicgraph::sot::VectorQuaternion quat_;
      Eigen::Vector3d rpy_;
      Eigen::Vector3d unit_x_;
      Eigen::Vector3d unit_x_rotated_;

      dg::Vector& data_out_callback(dg::Vector& s, int time);
  };

  /**
   * @brief Given input data x, compute y = sin(x).
   */
  class Operator_EXPORTS SinEntity: public dg::Entity
  {
    public:

      SinEntity( const std::string & name );

      static const std::string CLASS_NAME;
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      dg::SignalPtr<double,int> data_inputSIN;
      dg::SignalTimeDependent<double,int> data_outSOUT;

      double& data_out_callback(double& history, int time);
  };

  /**
   * @brief Given input data sin1 and sin2, compute y = sin1/sin2.
   */
  class Operator_EXPORTS Division_of_double: public dg::Entity
  {
    public:

      Division_of_double( const std::string & name );

      static const std::string CLASS_NAME;
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      dg::SignalPtr<double,int> data_input1SIN;
      dg::SignalPtr<double,int> data_input2SIN;
      dg::SignalTimeDependent<double,int> data_outSOUT;

      double& data_out_callback(double& history, int time);
  };


  /**
   * @brief Given input data, sum up the input data over time.
   */
  class Operator_EXPORTS VectorIntegrator: public dg::Entity
  {
    public:

      VectorIntegrator( const std::string & name );

      static const std::string CLASS_NAME;
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      bool init_;
      dg::Vector sum_;

      dg::SignalPtr<dg::Vector,int> data_inputSIN;
      dg::SignalTimeDependent<dg::Vector,int> data_outSOUT;

      dg::Vector& data_out_callback(dg::Vector& out, int time);
  };

} // namespace dg_tools


