// File:          smorphi_6ds.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include "base.h"
#include <webots/DistanceSensor.hpp>
#include <icecream.hpp>
#include "SmorphiDef.hpp"
#include <iostream>
#include "algorithm"
#include "webots/InertialUnit.hpp"
#include "webots/Lidar.hpp"
// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
std::unique_ptr<Base> Base::instance = nullptr;
std::once_flag Base::initInstanceFlag;
int main(int argc, char **argv)
{

  // create the Robot instance.
  Robot *robot = new Robot();

  // Init Keyboard
  Keyboard *key = new Keyboard();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // Enable Keyboard
  key->enable(timeStep);

  Motor *m_fl = robot->getMotor("m_fl");
  Motor *m_fr = robot->getMotor("m_fr");
  Motor *m_rl = robot->getMotor("m_rl");
  Motor *m_rr = robot->getMotor("m_rr");

  // Distance Sensor
  DistanceSensor *d1_forw = robot->getDistanceSensor("ds1_forward");
  DistanceSensor *d2_forw = robot->getDistanceSensor("ds2_forward");

  DistanceSensor *d1_back = robot->getDistanceSensor("ds1_backward");
  DistanceSensor *d2_back = robot->getDistanceSensor("ds2_backward");

  DistanceSensor *d1_left = robot->getDistanceSensor("ds1_left");
  DistanceSensor *d2_left = robot->getDistanceSensor("ds2_left");

  DistanceSensor *d1_right = robot->getDistanceSensor("ds1_right");
  DistanceSensor *d2_right = robot->getDistanceSensor("ds2_right");

  d1_forw->enable(timeStep);
  d2_forw->enable(timeStep);

  d1_back->enable(timeStep);
  d2_back->enable(timeStep);

  d1_left->enable(timeStep);
  d2_left->enable(timeStep);

  d1_right->enable(timeStep);
  d2_right->enable(timeStep);

  InertialUnit *imu = robot->getInertialUnit("IMU");
  imu->enable(timeStep);

  Lidar *lidar = robot->getLidar("lidar");
  lidar->enable(timeStep);

  Base &base_ = Base::getInstance();
  base_.init_motor(m_fl, m_fr, m_rl, m_rr);

  constexpr double constant_vel = 0.3;
  constexpr double setpoint = 0.05;
  constexpr double k_p = 5.0;
  constexpr double correction_min = -constant_vel;
  constexpr double correction_max = constant_vel;
  // Angles corresponding to each sensor
  constexpr double angles[] = {0.52, -0.52, 2.61, -2.61, -1.04, -2.09, 1.04, 2.09};

  // Lidar Component
  int horizonalResolution = lidar->getHorizontalResolution();
  double fov = lidar->getFov();

  // Main loop:
  while (robot->step(timeStep) != -1)
  {
    const double *rpy = imu->getRollPitchYaw();
    IC(rpy[2]);
    // Read all distance sensors
    // double distances[] = {
    //     d1_forw->getValue(), d2_forw->getValue(),
    //     d1_back->getValue(), d2_back->getValue(),
    //     d1_right->getValue(), d2_right->getValue(),
    //     d1_left->getValue(), d2_left->getValue()};

    const float *range = lidar->getRangeImage();

    // // Find the closest obstacle
    // int min_idx = 0;
    // double min_distance = distances[0];
    // for (int i = 1; i < 8; ++i)
    // {
    //   if (distances[i] < min_distance)
    //   {
    //     min_distance = distances[i];
    //     min_idx = i;
    //   }
    // }
    //  double angle_to_wall = angles[min_idx];
    // Find the closest Obstacle based on Lidar

    int min_idx = -1;
    double min_distance = std::numeric_limits<float>::infinity();

    for (int i = 0; i < horizonalResolution; i++)
    {
      if (range[i] - lidar->getMinRange() < min_distance)
      {
        min_distance = range[i] - lidar->getMinRange();
        min_idx = i;
      }
    }

    double angle_to_wall = fov / 2.0 + min_idx * (fov / (horizonalResolution - 1));
    // Shift angle by +π/2 to make 0° = front
    // angle_to_wall += M_PI / 2;

    // Wrap to [0, 2π)
    if (angle_to_wall < 0)
      angle_to_wall += 2 * M_PI;
    else if (angle_to_wall >= 2 * M_PI)
      angle_to_wall -= 2 * M_PI;
    IC(angle_to_wall * 180  / M_PI);
    // angle_to_wall *= 180 / M_PI;
    IC(min_distance, angle_to_wall, horizonalResolution, min_idx);
    double error = setpoint - min_distance;

    // Compute forward vector (perpendicular to obstacle)
    double v_to_wall_x = constant_vel * cos(angle_to_wall);
    double v_to_wall_y = constant_vel * sin(angle_to_wall);
    IC(v_to_wall_x, v_to_wall_y);
    // Perpendicular direction (forward movement)
    double v_forward_x = v_to_wall_y;
    double v_forward_y = -v_to_wall_x;

    // Correction to maintain setpoint distance
    double correction_x = std::clamp(k_p * error * cos(angle_to_wall), correction_min, correction_max);
    double correction_y = std::clamp(k_p * error * sin(angle_to_wall), correction_min, correction_max);
    IC(correction_x, correction_y);
    // Final command: move forward while correcting distance to wall
    double v_command_x = v_forward_x - correction_x;
    double v_command_y = v_forward_y - correction_y;

    double errorYaw = -1.57 - rpy[2];
    double out = 1.0 * errorYaw;
    IC(v_command_x, v_command_y);
    // base_.base_move(correction_x, -correction_y, out);
    base_.base_move(v_command_x, -v_command_y, out);

    // base_.base_move(0, 0, 0);
  }

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
