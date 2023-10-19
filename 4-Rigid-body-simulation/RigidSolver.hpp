// ----------------------------------------------------------------------------
// RigidSolver.hpp
//
//  Created on: 18 Dec 2020
//      Author: Kiwon Um
//        Mail: kiwon.um@telecom-paris.fr
//
// Description: Simple Rigid Body Solver (DO NOT DISTRIBUTE!)
//
// Copyright 2020-2023 Kiwon Um
//
// The copyright to the computer program(s) herein is the property of Kiwon Um,
// Telecom Paris, France. The program(s) may be used and/or copied only with
// the written permission of Kiwon Um or in accordance with the terms and
// conditions stipulated in the agreement/contract under which the program(s)
// have been supplied.
// ----------------------------------------------------------------------------

#ifndef _RIGIDSOLVER_HPP_
#define _RIGIDSOLVER_HPP_

#include <glm/ext/matrix_transform.hpp>

#include "Vector3.hpp"
#include "Matrix3x3.hpp"
#include "Quaternion.hpp"

struct BodyAttributes {
  BodyAttributes() :
    X(0, 0, 0), R(Mat3f::I()), P(0, 0, 0), L(0, 0, 0),
    V(0, 0, 0), omega(0, 0, 0), F(0, 0, 0), tau(0, 0, 0),
    Q(Quaternion(1.0, Vec3f(0, 0, 0)))  {}

  glm::mat4 worldMat() const
  {
    return glm::mat4(           // column-major
      R(0,0), R(1,0), R(2,0), 0,
      R(0,1), R(1,1), R(2,1), 0,
      R(0,2), R(1,2), R(2,2), 0,
      X[0],   X[1],   X[2],   1);
  }

  tReal M;                      // mass
  Mat3f I0, I0inv;              // inertia tensor and its inverse in body space
  Mat3f Iinv;                   // inverse of inertia tensor

  // rigid body state
  Vec3f X;                      // position
  Mat3f R;                      // rotation
  Vec3f P;                      // linear momentum
  Vec3f L;                      // angular momentum

  // auxiliary quantities
  Vec3f V;                      // linear velocity
  Vec3f omega;                  // angular velocity
  Vec3f alpha;                  // angular acceleration
  Quaternion Q;

  // force and torque
  Vec3f F;                      // force
  Vec3f tau;                    // torque

  // mesh's vertices in body space
  std::vector<Vec3f> vdata0;
};

class Box : public BodyAttributes {
public:
  explicit Box(
    const tReal w=1.0, const tReal h=1.0, const tReal d=1.0, const tReal dens=10.0,
    const Vec3f v0=Vec3f(0, 0, 0), const Vec3f omega0=Vec3f(0, 0, 0)) :
    width(w), height(h), depth(d)
  {
    V = v0;                     // initial velocity
    omega = omega0;             // initial angular velocity

    // TODO: calculate physical attributes
    M = dens * w * h * d; //density*volume (width(w), height(h), depth(d))

    I0 = Mat3f(Vec3f(M * (pow(h, 2) + pow(d, 2)) / 12.f, //inertia tensor in body space
                     M * (pow(w, 2) + pow(d, 2)) / 12.f,
                     M * (pow(w, 2) + pow(h, 2)) / 12.f));
  
    I0inv = I0.inverse();     //inverse inertia tensor  in body space
    Iinv = R * I0inv * R.transpose(); //inverse inertia tensor
    
    // vertices data (8 vertices)
    vdata0.push_back(Vec3f(-0.5*w, -0.5*h, -0.5*d));
    vdata0.push_back(Vec3f( 0.5*w, -0.5*h, -0.5*d));
    vdata0.push_back(Vec3f( 0.5*w,  0.5*h, -0.5*d));
    vdata0.push_back(Vec3f(-0.5*w,  0.5*h, -0.5*d));

    vdata0.push_back(Vec3f(-0.5*w, -0.5*h,  0.5*d));
    vdata0.push_back(Vec3f( 0.5*w, -0.5*h,  0.5*d));
    vdata0.push_back(Vec3f( 0.5*w,  0.5*h,  0.5*d));
    vdata0.push_back(Vec3f(-0.5*w,  0.5*h,  0.5*d));
  }

  // rigid body property
  tReal width, height, depth;
};

class RigidSolver {
public:
  explicit RigidSolver(
    BodyAttributes *body0=nullptr, const Vec3f g=Vec3f(0, 0, 0)) :
    body(body0), _g(g), _step(0), _sim_t(0) {}

  void init(BodyAttributes *body0)
  {
    body = body0;
    _step = 0;
    _sim_t = 0;
  }

  void step(const tReal dt)
  {
    std::cout << "t=" << _sim_t << " (dt=" << dt << ")" << std::endl;

    computeForceAndTorque();
    
    // TODO: time integration
   
    body->V = body->P / body->M;          // velocity --> taking the formula from Linear Momentum (slide 13)
    body->X += dt * body->V;     // position -> se la non la metto mi resta fermo , ruota e basta
    body->P += dt * body->F;     // linear momentum 
    //
    body->L += dt * body->tau;   // angular momentum 

    //// rotation
    body->omega = body->Iinv * body->L;  // angular velocity
    ////OR
   //// body->alpha = body->Iinv * body->F; // angular acceleration
   //// body->omega += dt * body->alpha; // angular velocity
   body->R = body->R + dt * body->omega.crossProductMatrix() * body->R;  // rotation matrix
   //body->Q += Quaternion(0, body->omega) * body->Q * dt / 2;
   //body->R = body->Q.normalize().ToRotMat();
    //

    ++_step;
    _sim_t += dt;
  }

  BodyAttributes *body;

private:
  void computeForceAndTorque() //fuori da qui non devo fare nulla xk me lo aggiorna ogni secondo, ogni dt!
  {
    // TODO: force and torque calculation
      //body->tau = Vec3f(0, 0, 0); 
    //  body->F = body->M * _g;
    // TODO: instance force at the very first step

    if(_step==1) {
        body->F = body->M * _g;
        body->F += Vec3f(0.015f, 0.015f, 0.03f);
        body->tau = crossProductMatrix(body->R * body->vdata0[0]) * body->F;
    }

    if (_step == 2) {
        body->F = body->M * _g;
        body->tau = Vec3f(0, 0, 0);
      }
  }

  // simulation parameters
  Vec3f _g;                     // gravity
  tIndex _step;                 // step count
  tReal _sim_t;                 // simulation time
};

#endif  /* _RIGIDSOLVER_HPP_ */
