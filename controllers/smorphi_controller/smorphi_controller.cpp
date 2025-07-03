// File:          smorphi_controller.cpp
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

enum LastMove
{
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  FORWARD_LEFT,
  FORWARD_RIGHT,
  BACKWARD_LEFT,
  BACKWARD_RIGHT,
  NONE
};

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

  DistanceSensor *d1_back = robot->getDistanceSensor("ds1_backward");

  DistanceSensor *d1_left = robot->getDistanceSensor("ds1_left");

  DistanceSensor *d1_right = robot->getDistanceSensor("ds1_right");

  d1_forw->enable(timeStep);

  d1_back->enable(timeStep);

  d1_left->enable(timeStep);

  d1_right->enable(timeStep);

  Base &base_ = Base::getInstance();
  base_.init_motor(m_fl, m_fr, m_rl, m_rr);

  Velocity_t vel_robot;
  int last_key = -1;

  LastMove last_move = NONE; // Global or class-level variable

  double error = 0;
  constexpr double constant_vel = 0.3;
  constexpr double setpoint = 0.1;
  constexpr double k_p = 5.0;
  constexpr double correction_min = -constant_vel;
  constexpr double correction_max = constant_vel;
  // Angles corresponding to each sensor
  constexpr double angles[] = {0.0, M_PI, - M_PI / 2, M_PI / 2};

  while (robot->step(timeStep) != -1)
  {

    auto k = key->getKey();

    if (k >= 0 and k != last_key)
    {
      if (k == 73) // keyboard I
      {
        vel_robot.vx += 0.05;
        vel_robot.vy = 0;
        vel_robot.vtheta = 0;

        base_.base_move(vel_robot.vx, vel_robot.vy, vel_robot.vtheta);
      }
      else if (k == 77) // Keyboard M
      {
        vel_robot.vx -= 0.05;
        vel_robot.vy = 0;
        vel_robot.vtheta = 0;

        base_.base_move(vel_robot.vx, vel_robot.vy, vel_robot.vtheta);
      }
      else if (k == 74) // Keyboard J
      {
        vel_robot.vx = 0;
        vel_robot.vy += 0.05;
        vel_robot.vtheta = 0;

        base_.base_move(vel_robot.vx, vel_robot.vy, vel_robot.vtheta);
      }
      else if (k == 76) // Keyboard L
      {
        vel_robot.vx = 0;
        vel_robot.vy -= 0.05;
        vel_robot.vtheta = 0;

        base_.base_move(vel_robot.vx, vel_robot.vy, vel_robot.vtheta);
      }
      else if (k == Keyboard::LEFT) // Keyboard LEFT
      {
        vel_robot.vx = 0;
        vel_robot.vy = 0;
        vel_robot.vtheta -= 0.05;
        base_.base_move(vel_robot.vx, vel_robot.vy, vel_robot.vtheta);
      }

      else if (k == Keyboard::RIGHT) // Keyboard RIGHT
      {
        vel_robot.vx = 0;
        vel_robot.vy = 0;
        vel_robot.vtheta += 0.05;
        base_.base_move(vel_robot.vx, vel_robot.vy, vel_robot.vtheta);
      }

      else if (k == 85) // Keyboard U
      {
        vel_robot.vx += 0.05;
        vel_robot.vy += 0.05;
        vel_robot.vtheta = 0;
        base_.base_move(vel_robot.vx, vel_robot.vy, vel_robot.vtheta);
      }

      else if (k == 79) // Keyboard O
      {
        vel_robot.vx += 0.05;
        vel_robot.vy -= 0.05;
        vel_robot.vtheta = 0;
        base_.base_move(vel_robot.vx, vel_robot.vy, vel_robot.vtheta);
      }

      else if (k == 78) // Keyboard N
      {
        vel_robot.vx -= 0.05;
        vel_robot.vy += 0.05;
        vel_robot.vtheta = 0;
        base_.base_move(vel_robot.vx, vel_robot.vy, vel_robot.vtheta);
      }

      else if (k == 44) // Keyboard ,
      {
        vel_robot.vx -= 0.05;
        vel_robot.vy -= 0.05;
        vel_robot.vtheta = 0;
        base_.base_move(vel_robot.vx, vel_robot.vy, vel_robot.vtheta);
      }

      else
      {
        vel_robot.vx = 0;
        vel_robot.vy = 0;
        vel_robot.vtheta = 0;
        base_.base_move(vel_robot.vx, vel_robot.vy, vel_robot.vtheta);
      }
    }

    // IC(d1_forw->getValue(), d2_forw->getValue());
    // IC(d1_back->getValue(), d2_back->getValue());
    // IC(d1_left->getValue(), d2_left->getValue());
    // IC(d1_right->getValue(), d2_right->getValue());

    // Mapping Logic
    // double distance_forw = std::min(d1_forw->getValue(), d2_forw->getValue());
    // double distance_back = std::min(d1_back->getValue(), d2_back->getValue());
    // double distance_left = std::min(d1_left->getValue(), d2_left->getValue());
    // double distance_right = std::min(d1_right->getValue(), d2_right->getValue());

    double distances[] = {
        d1_forw->getValue(),
        d1_back->getValue(),
        d1_right->getValue(),
        d1_left->getValue(),
    };

       // Find the closest obstacle
    int min_idx = 0;
    double min_distance = distances[0];
    for (int i = 1; i < sizeof(distances) / sizeof(distances[0]); ++i) {
      if (distances[i] < min_distance) {
        min_distance = distances[i];
        min_idx = i;
      }
    }

    double angle_to_wall = angles[min_idx];
    double error = setpoint - min_distance;

    // Compute forward vector (perpendicular to obstacle)
    double v_to_wall_x = constant_vel * cos(angle_to_wall);
    double v_to_wall_y = constant_vel * sin(angle_to_wall);

    // Perpendicular direction (forward movement)
    double v_forward_x = v_to_wall_y;
    double v_forward_y = -v_to_wall_x;

    // Correction to maintain setpoint distance
    double correction_x = std::clamp(k_p * error * cos(angle_to_wall), correction_min, correction_max);
    double correction_y = std::clamp(k_p * error * sin(angle_to_wall), correction_min, correction_max);

    // Final command: move forward while correcting distance to wall
    double v_command_x = v_forward_x - correction_x;
    double v_command_y = v_forward_y - correction_y;

    base_.base_move(v_command_x, v_command_y, 0);
    // base_.base_move(0, 0, 0);

    // IC(distance_forw, distance_back, distance_left, distance_right);

    // Standar Move
    // Default: try to move forward
    // base_.base_move(0.1, 0, 0);

    // 1. If front, left, and back are blocked → move right
    IC(last_move);

    // if (distance_forw < 0.1 && distance_left < 0.1 && distance_right > 0.1)
    // {
    //   std::cout << "[DEBUG] Blocked F+L, Right clear → Strafe right" << std::endl;
    //   base_.base_move(-0.05, -0.1, 0);
    //   last_move = BACKWARD_RIGHT;
    // }

    // // 2. If front, right, and back are blocked → move left
    // else if (distance_forw < 0.1 && distance_right < 0.1 && distance_left > 0.1)
    // {
    //   std::cout << "[DEBUG] Blocked F+R, Left clear → Strafe left" << std::endl;
    //   base_.base_move(-0.05, 0.1, 0);
    //   last_move = BACKWARD_LEFT;
    // }

    // else if (distance_back < 0.1 && distance_left < 0.1 && distance_right > 0.1)
    // {
    //   std::cout << "[DEBUG] Blocked B+L, Right clear → Strafe right" << std::endl;
    //   base_.base_move(0.05, -0.1, 0);
    //   last_move = FORWARD_RIGHT;
    // }

    // // 2. If front, right, and back are blocked → move left
    // else if (distance_back < 0.1 && distance_right < 0.1 && distance_left > 0.1)
    // {
    //   std::cout << "[DEBUG] Blocked B+R, Left clear → Strafe left" << std::endl;
    //   base_.base_move(0.05, 0.1, 0);
    //   last_move = FORWARD_LEFT;
    // }

    // else if (distance_forw < 0.1 && distance_back > 0.1 && last_move != FORWARD)
    // {
    //   std::cout << "[DEBUG] F blocked, B clear → Move back" << std::endl;
    //   base_.base_move(-0.1, 0, 0);
    //   last_move = BACKWARD;
    // }

    // else if (distance_back < 0.1 && distance_forw > 0.1 && IC(last_move != BACKWARD))
    // {
    //   std::cout << "[DEBUG] B blocked, F clear → Move forward" << std::endl;
    //   base_.base_move(0.1, 0, 0);
    //   last_move = FORWARD;
    // }

    // // 3. If front and right blocked, but left is clearer → forward-left
    // else if (distance_forw < 0.1 && distance_right < 0.1 && distance_left > 0.1 && last_move != FORWARD_LEFT)
    // {
    //   std::cout << "[DEBUG] Forward+Right blocked, Left clear → Move forward-left" << std::endl;
    //   base_.base_move(0.05, 0.1, 0);
    //   last_move = FORWARD_LEFT;
    // }

    // // 4. If front and left blocked, but right is clearer → forward-right
    // else if (distance_forw < 0.1 && distance_left < 0.1 && distance_right > 0.1 && last_move != FORWARD_RIGHT)
    // {
    //   std::cout << "[DEBUG] Forward+Left blocked, Right clear → Move forward-right" << std::endl;
    //   base_.base_move(0.05, -0.1, 0);
    //   last_move = FORWARD_RIGHT;
    // }

    // 7. If all directions blocked → spin or stop
    // else if (distance_forw < 0.1 && distance_back < 0.1 && distance_left < 0.1 && distance_right < 0.1)
    // {
    //   std::cout << "[DEBUG] All directions blocked → Spin or stop" << std::endl;
    //   base_.base_move(0, 0, 0.1); // rotate
    // }
    // IC(distance_back, distance_forw, distance_left, distance_right);
    // // IC(m_fl->getVelocity(), m_fr->getVelocity());
    // IC(m_rl->getVelocity(), m_rr->getVelocity());

    // else if()

    // base_.base_move(0, 0, 0);
    // 77 M , 76 L , J 74, Space 32 STOP
    last_key = k;
    // IC(k);
    // IC(robot->getName());
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
