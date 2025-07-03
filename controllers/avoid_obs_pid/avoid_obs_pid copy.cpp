#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

using namespace webots;

//Variable default
#define base_speed 4.0
#define SAFE_DIST 80
#define DANGER_DIST 50
#define MAX_SPEED 1.57
#define NUM_SENSORS 8

// PID parameters
float kp = 0.15, kd = 0.05, ki = 0.001;
float error[NUM_SENSORS] = {0}, previousError[NUM_SENSORS] = {0}, integral[NUM_SENSORS] = {0};
float corrections[NUM_SENSORS] = {0};
double left_speed = 0, right_speed = 0;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  // Motor Initialization
  Motor *wheels[2];
  char wheels_names[2][8] = {"r_wheel", "l_wheel"}; 
  for (int i = 0; i < 2; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }

  // Distance Sensor Initialization
  DistanceSensor *ds[NUM_SENSORS];
  char dsNames[8][32] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
  for (int i = 0; i < NUM_SENSORS; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(timeStep);
  }

  double dist_val[NUM_SENSORS];
  double last_time = 0;

  while (robot->step(timeStep) != -1) {
    //Get Elapsed Time
    double dt = robot->getTime() - last_time;

    // Read sensor values and calculate corrections
    for (int i = 0; i < NUM_SENSORS; i++) {
      dist_val[i] = ds[i]->getValue();
      error[i] = SAFE_DIST - dist_val[i];

      // PID calculations
      integral[i] += error[i] * dt;
      float derivative = (error[i] - previousError[i]) / dt;
      corrections[i] = kp * error[i] + ki * integral[i] + kd * derivative;
     
      //Clamp Output PID
      if (corrections[i] > MAX_SPEED) corrections[i] = MAX_SPEED;
      else if (corrections[i] < -MAX_SPEED) corrections[i] = -MAX_SPEED;
      std::cout << "Output Sensor " << i << ":" << corrections[i] << " Error Sensor " << i << ":" << error[i] << " Integral Sensor " << i << ":" << integral[i] << " Derivative Sensor " << i << ":" << derivative << std::endl;  
      previousError[i] = error[i];
    }

    // Reset wheel speeds to base speed
    left_speed = base_speed;
    right_speed = base_speed;

    // Obstacle avoidance logic
    if (dist_val[0] > SAFE_DIST  || dist_val[7] > SAFE_DIST) {
      // Obstacle in front
        left_speed -= base_speed + corrections[0];
        right_speed += base_speed + corrections[7];
    }
 
    if (dist_val[5] > SAFE_DIST || dist_val[6] > SAFE_DIST) {
      // Obstacle on the left
      left_speed +=  corrections[5];
      right_speed -= corrections[6] ;
    }

    if (dist_val[2] > SAFE_DIST || dist_val[1] > SAFE_DIST) {
      // Obstacle on the right
      left_speed -=   corrections[2];
      right_speed +=  corrections[1] ;
   
    }
    
    // Clamp speeds to avoid exceeding MAX_SPEED
    if (left_speed > base_speed ) left_speed = base_speed;
    else if (left_speed < -base_speed) left_speed = -base_speed;

    if (right_speed > base_speed) right_speed = base_speed;
    else if (right_speed < -base_speed) right_speed = -base_speed;

    // Set wheel speeds
    wheels[0]->setVelocity(left_speed);
    wheels[1]->setVelocity(right_speed);

    last_time = robot->getTime();
  };

  // Enter here exit cleanup code.
  delete robot;
  return 0;
}
