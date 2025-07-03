#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include "icecream.hpp"

using namespace webots;

// Variable default
#define base_speed 4.0
#define SAFE_DIST 200
#define DANGER_DIST 50
#define MAX_SPEED 1.57
#define NUM_SENSORS 8

// PID parameters
float kp = 0.15, kd = 0.05, ki = 0.001;
float error[NUM_SENSORS] = {0}, previousError[NUM_SENSORS] = {0}, integral[NUM_SENSORS] = {0};
float corrections[NUM_SENSORS] = {0};
double left_speed = 0, right_speed = 0;

int main(int argc, char **argv)
{
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  // Motor Initialization
  Motor *wheels[2];
  char wheels_names[2][8] = {"r_wheel", "l_wheel"};
  for (int i = 0; i < 2; i++)
  {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }

  // Distance Sensor Initialization
  DistanceSensor *ds[NUM_SENSORS];
  char dsNames[8][32] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(timeStep);
  }

  double dist_val[NUM_SENSORS];
  double last_time = 0;

  while (robot->step(timeStep) != -1)
  {
    // Get Elapsed Time
    double dt = robot->getTime() - last_time;

    // Read sensor values and calculate corrections
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      dist_val[i] = ds[i]->getValue();
      error[i] = SAFE_DIST - dist_val[i];

      // PID calculations
      integral[i] += error[i] * dt;
      float derivative = (error[i] - previousError[i]) / dt;
      corrections[i] = kp * error[i] + ki * integral[i] + kd * derivative;

      // Clamp Output PID
      if (corrections[i] > MAX_SPEED)
        corrections[i] = MAX_SPEED;
      else if (corrections[i] < -MAX_SPEED)
        corrections[i] = -MAX_SPEED;
      // std::cout << "Output Sensor " << i << ":" << corrections[i] << " Error Sensor " << i << ":" << error[i] << " Integral Sensor " << i << ":" << integral[i] << " Derivative Sensor " << i << ":" << derivative << std::endl;
      previousError[i] = error[i];
    }

    // Reset wheel speeds to base speed

    double front_val = (corrections[0] + corrections[7]) / 2.0;
    double left_val = (corrections[5] + corrections[6]) / 2.0;
    double right_val = (corrections[1] + corrections[2]) / 2.0;

    // Default base forward motion
    left_speed = base_speed;
    right_speed = base_speed;

    // If left is free:
    //     Turn Left
    // Else if left is occupied and straight is free:
    //     Go Straight
    // Else if left and straight are occupied:
    //     Turn Right
    // Else if left/right/straight are occupied or you crashed:
    //     Turn 180 degrees

    // if (dist_val[5] < SAFE_DIST || dist_val[6] < SAFE_DIST)
    // {
    //   // Wall or obs in the left
    //   IC("left");
    //   left_speed -= left_val;
    //   right_speed += left_val;
    // }

    // else if ((dist_val[5] > SAFE_DIST || dist_val[6] > SAFE_DIST) && (dist_val[0] < 80 || dist_val[7] < 80))
    // {
    //   IC("Straight");

    //   left_speed = front_val;
    //   right_speed = front_val;
    // }

    // else if ((dist_val[5] > SAFE_DIST || dist_val[6] > SAFE_DIST) && (dist_val[0] > 80 || dist_val[7] > 80))
    // {
    //   IC("turn_right");

    //   left_speed += left_val;
    //   right_speed -= left_val;
    //   IC(right_speed, left_speed);
    // }

    // else{
    //   IC("turn 180");
    //   left_speed = -1.0;
    //   right_speed = -1.0;
    // }


    // if (dist_val[0] > 80 || dist_val[7] > 80)
    // {
    //   if (left_val > right_val)
    //   {
    //     // Left is more blocked → Turn right
    //     left_speed = 1;
    //     right_speed = -1.0;
    //   }
    //   else
    //   {
    //     // Right is more blocked → Turn left
    //     left_speed = -1.0;
    //     right_speed = 1.0;
    //   }
    // }

    // else if (dist_val[5] > SAFE_DIST || dist_val[6] > SAFE_DIST)
    // {
    //   // Wall or obs in the left
    //   left_speed += left_val;
    //   right_speed -= left_val;
    // }

    // else if (dist_val[2] > SAFE_DIST || dist_val[1] > SAFE_DIST)
    // {
    //   // Obstacle on the right
    //   left_speed -= right_val;
    //   right_speed += right_val;
    // }

    // // Obstacle avoidance logic
    if (dist_val[0] > 80 || dist_val[7] > 80)
    {
      if (left_val > right_val)
      {
        // Left is more blocked → Turn right
        left_speed = 1;
        right_speed = -1.0;
      }
      else
      {
        // Right is more blocked → Turn left
        left_speed = -1.0;
        right_speed = 1.0;
      }
    }

    else if (dist_val[5] > SAFE_DIST || dist_val[6] > SAFE_DIST)
    {
      // Wall or obs in the left
      left_speed += left_val;
      right_speed -= left_val;
    }

    else if (dist_val[2] > SAFE_DIST || dist_val[1] > SAFE_DIST)
    {
      // Obstacle on the right
      left_speed -= right_val;
      right_speed += right_val;
    }

    // Clamp speeds to avoid exceeding MAX_SPEED
    if (left_speed > base_speed)
      left_speed = base_speed;
    else if (left_speed < -base_speed)
      left_speed = -base_speed;

    if (right_speed > base_speed)
      right_speed = base_speed;
    else if (right_speed < -base_speed)
      right_speed = -base_speed;

    // Set wheel speeds
    wheels[0]->setVelocity(left_speed);
    wheels[1]->setVelocity(right_speed);
    IC(left_speed, right_speed);
    IC(dist_val[0], dist_val[7]);
    IC(dist_val[5], dist_val[6]);
    IC(dist_val[1], dist_val[2]);
    IC(right_val, front_val, left_val);
    IC(corrections[5], corrections[6]);
    // wheels[0]->setVelocity(0);
    // wheels[1]->setVelocity(0);

    last_time = robot->getTime();
  };

  // Enter here exit cleanup code.
  delete robot;
  return 0;
}
