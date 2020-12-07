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

#include <pinocchio/spatial/se3.hpp>

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
   * @brief Converts PoseRPY into PoseQuaternion data.
   */
  class Operator_EXPORTS PoseRPYToPoseQuaternion: public dg::Entity
  {
    public:

      PoseRPYToPoseQuaternion( const std::string & name );

      static const std::string CLASS_NAME;
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      dg::SignalPtr<dg::Vector,int> data_inputSIN;
      dg::SignalTimeDependent<dg::Vector,int> data_outSOUT;

      dg::Vector& data_out_callback(dg::Vector& history, int time);
  };

  /**
   * @brief Given input data x, compute y = sin(x).
   */
  class Operator_EXPORTS Sinus: public dg::Entity
  {
    public:

      Sinus( const std::string & name );

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

  /**
   * @brief Given input sin1 (se3 poseQuat) and sin2 (a vector to apply it,
   * xyzquat or 6d), compute y = sin1.act(sin2).
   */
  class Operator_EXPORTS Multiply_poseQuaternion_vector: public dg::Entity
  {
    public:

      Multiply_poseQuaternion_vector( const std::string & name );

      static const std::string CLASS_NAME;
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      dg::SignalPtr<dg::Vector,int> sin1_;
      dg::SignalPtr<dg::Vector,int> sin2_;
      dg::SignalTimeDependent<dg::Vector,int> sout_;

      dg::Vector& data_out_callback(dg::Vector& out, int time);
  };

  /**
   * @brief Given input sin1 (se3 poseQuat) and sin2 (a vector to apply it,
   * xyzquat or 6d), compute y = sin1.actInv(sin2).
   */
  class Operator_EXPORTS MultiplyInv_poseQuaternion_vector: public dg::Entity
  {
    public:

      MultiplyInv_poseQuaternion_vector( const std::string & name );

      static const std::string CLASS_NAME;
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      dg::SignalPtr<dg::Vector,int> sin1_;
      dg::SignalPtr<dg::Vector,int> sin2_;
      dg::SignalTimeDependent<dg::Vector,int> sout_;

      dg::Vector& data_out_callback(dg::Vector& out, int time);
  };


} // namespace dg_tools


