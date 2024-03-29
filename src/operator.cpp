/**
 * Copyright 2019 Max Planck Society. All rights reserved.
 * Julian Viereck
 */

#include "dg_tools/utils.hpp"
#include "dg_tools/operator.hpp"

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/math/quaternion.hpp"
#include <iostream>

using namespace dg_tools;

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PoseQuaternionToPoseRPY, "PoseQuaternionToPoseRPY");

PoseQuaternionToPoseRPY::PoseQuaternionToPoseRPY( const std::string & name )
 :Entity(name)
 ,data_inputSIN(NULL,"PoseQuaternionToPoseRPY("+name+")::input(vector)::sin")
 ,data_outSOUT( boost::bind(&PoseQuaternionToPoseRPY::data_out_callback,this,_1,_2),
         data_inputSIN,
        "PoseQuaternionToPoseRPY("+name+")::output(vector)::sout" )
{
  Entity::signalRegistration(data_inputSIN << data_outSOUT);
}

dg::Vector& PoseQuaternionToPoseRPY::data_out_callback(dg::Vector& out, int time)
{
    const dg::Vector& input = data_inputSIN(time);
    dynamicgraph::sot::VectorQuaternion quat(input(6), input(3), input(4), input(5));

    out.resize(6);
    out.head<3>() = input.head<3>();
    quat.normalize();
    out.tail<3>() = pinocchio::rpy::matrixToRpy(quat.toRotationMatrix());
    return out;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PoseRPYToPoseQuaternion, "PoseRPYToPoseQuaternion");

PoseRPYToPoseQuaternion::PoseRPYToPoseQuaternion( const std::string & name )
 :Entity(name)
 ,data_inputSIN(NULL,"PoseRPYToPoseQuaternion("+name+")::input(vector)::sin")
 ,data_outSOUT( boost::bind(&PoseRPYToPoseQuaternion::data_out_callback,this,_1,_2),
         data_inputSIN,
        "PoseRPYToPoseQuaternion("+name+")::output(vector)::sout" )
{
  Entity::signalRegistration(data_inputSIN << data_outSOUT);
}

dg::Vector& PoseRPYToPoseQuaternion::data_out_callback(dg::Vector& out, int time)
{
    const dg::Vector& input = data_inputSIN(time);
    Eigen::Quaterniond q = Eigen::AngleAxisd(input(5), Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(input(4), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(input(3), Eigen::Vector3d::UnitX());
    out.resize(7);
    out.head<3>() = input.head<3>();
    out(3) = q.x();
    out(4) = q.y();
    out(5) = q.z();
    out(6) = q.w();
    return out;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RPYToRotationMatrix, "RPYToRotationMatrix");

RPYToRotationMatrix::RPYToRotationMatrix( const std::string & name )
 :Entity(name)
 ,data_inputSIN(NULL,"RPYToRotationMatrix("+name+")::input(vector)::sin")
 ,data_outSOUT( boost::bind(&RPYToRotationMatrix::data_out_callback,this,_1,_2),
         data_inputSIN,
        "RPYToRotationMatrix("+name+")::output(matrix)::sout" )
{
  Entity::signalRegistration(data_inputSIN << data_outSOUT);
}

dg::Matrix& RPYToRotationMatrix::data_out_callback(dg::Matrix& out, int time)
{
    const dg::Vector& input = data_inputSIN(time);
    Eigen::Quaterniond q = Eigen::AngleAxisd(input(2), Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(input(1), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(input(0), Eigen::Vector3d::UnitX());
    out.resize(3, 3);
    out = q.toRotationMatrix();
    return out;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Sinus, "Sinus");

Sinus::Sinus( const std::string & name )
 :Entity(name)
 ,data_inputSIN(NULL,"Sinus("+name+")::input(double)::sin")
 ,data_outSOUT( boost::bind(&Sinus::data_out_callback,this,_1,_2),
         data_inputSIN,
        "Sinus("+name+")::output(double)::sout" )
{
  Entity::signalRegistration(data_inputSIN << data_outSOUT);
}

double& Sinus::data_out_callback(double& out, int time)
{
    out = sin(data_inputSIN.access(time));
    return out;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Division_of_double, "Division_of_double");

Division_of_double::Division_of_double( const std::string & name )
 :Entity(name)
 ,data_input1SIN(NULL,"Division_of_double("+name+")::input(double)::sin1")
 ,data_input2SIN(NULL,"Division_of_double("+name+")::input(double)::sin2")
 ,data_outSOUT( boost::bind(&Division_of_double::data_out_callback,this,_1,_2),
         data_input1SIN << data_input2SIN,
        "Division_of_double("+name+")::output(double)::sout" )
{
  Entity::signalRegistration(data_input1SIN << data_input2SIN << data_outSOUT);
}

double& Division_of_double::data_out_callback(double& out, int time)
{
    out = data_input1SIN.access(time) / data_input2SIN.access(time);
    return out;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(VectorIntegrator, "VectorIntegrator");

VectorIntegrator::VectorIntegrator( const std::string & name )
 :Entity(name)
 ,init_(true)
 ,data_inputSIN(NULL,"VectorIntegrator("+name+")::input(vector)::sin")
 ,data_outSOUT( boost::bind(&VectorIntegrator::data_out_callback,this,_1,_2),
         data_inputSIN,
        "VectorIntegrator("+name+")::output(vector)::sout" )
{
  Entity::signalRegistration(data_inputSIN << data_outSOUT);

  data_outSOUT.setDependencyType(
        dynamicgraph::TimeDependency<int>::ALWAYS_READY);
}

dg::Vector& VectorIntegrator::data_out_callback(dg::Vector& out, int time)
{
  const dg::Vector& input = data_inputSIN(time);
  out.resize(input.size());
  if (init_) {
    init_ = false;
    sum_.resize(input.size());
    sum_.setZero(input.size());
  }

  // TODO: Avoid hard-coded sampling rate here.
  sum_ += 0.001 * input;
  out = sum_;
  return out;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Multiply_poseQuaternion_vector,
                                   "Multiply_poseQuaternion_vector");

Multiply_poseQuaternion_vector::Multiply_poseQuaternion_vector(
    const std::string & name )
 :Entity(name)
 ,sin1_(
    NULL,"Multiply_poseQuaternion_vector("+name+")::input(poseQuaternion)::sin1")
 ,sin2_(
    NULL,"Multiply_poseQuaternion_vector("+name+")::input(vector)::sin2")
 ,sout_(
    boost::bind(&Multiply_poseQuaternion_vector::data_out_callback,this,_1,_2),
    sin1_ << sin2_,
    "Multiply_poseQuaternion_vector("+name+")::output(double)::sout" )
{
  Entity::signalRegistration(sin1_ << sin2_ << sout_);
}

dg::Vector& Multiply_poseQuaternion_vector::data_out_callback(
    dg::Vector& out, int time)
{
    const dg::Vector& xyzquat_in1 = sin1_.access(time);
    const dg::Vector& vector_in2 = sin2_.access(time);

    // convert the xyzquat into a pinocchio::SE3 object
    pinocchio::SE3 se3_in1 = xyzquat_to_se3(xyzquat_in1);

    if(vector_in2.size() == 6)
    {
        if(out.size() != 6)
        {
            out.setZero(6);
        }
        out = se3_in1.toActionMatrix() * vector_in2;
    }
    else if(vector_in2.size() == 7)
    {
        if(out.size() != 7)
        {
            out.setZero(7);
        }
        pinocchio::SE3 se3_in2 = xyzquat_to_se3(vector_in2);
        pinocchio::SE3 se3_out = se3_in1.act(se3_in2);
        out = se3_to_xyzquat(se3_out);
    }
    return out;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(MultiplyInv_poseQuaternion_vector,
                                   "MultiplyInv_poseQuaternion_vector");

MultiplyInv_poseQuaternion_vector::MultiplyInv_poseQuaternion_vector(
    const std::string & name )
 :Entity(name)
 ,sin1_(
    NULL,"MultiplyInv_poseQuaternion_vector("+name+")::input(poseQuaternion)::sin1")
 ,sin2_(
    NULL,"MultiplyInv_poseQuaternion_vector("+name+")::input(vector)::sin2")
 ,sout_(
    boost::bind(&MultiplyInv_poseQuaternion_vector::data_out_callback,this,_1,_2),
    sin1_ << sin2_,
    "MultiplyInv_poseQuaternion_vector("+name+")::output(double)::sout" )
{
  Entity::signalRegistration(sin1_ << sin2_ << sout_);
}

dg::Vector& MultiplyInv_poseQuaternion_vector::data_out_callback(
    dg::Vector& out, int time)
{
    const dg::Vector& xyzquat_in1 = sin1_.access(time);
    const dg::Vector& vector_in2 = sin2_.access(time);

    // convert the xyzquat into a pinocchio::SE3 object
    pinocchio::SE3 se3_in1 = xyzquat_to_se3(xyzquat_in1);

    if(vector_in2.size() == 6)
    {
        if(out.size() != 6)
        {
            out.setZero(6);
        }
        out = se3_in1.toActionMatrixInverse() * vector_in2;
    }
    else if(vector_in2.size() == 7)
    {
        if(out.size() != 7)
        {
            out.setZero(7);
        }
        pinocchio::SE3 se3_in2 = xyzquat_to_se3(vector_in2);
        pinocchio::SE3 se3_out = se3_in1.actInv(se3_in2);
        out = se3_to_xyzquat(se3_out);
    }
    return out;
}
