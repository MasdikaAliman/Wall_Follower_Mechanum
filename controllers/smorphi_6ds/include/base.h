#ifndef BASE_HPP
#define BASE_HPP

#include <memory>
#include <mutex>
#include <vector>
#include <iostream>

#include <webots/Motor.hpp>

#define SPEED 4.0
#define MAX_SPEED 0.3
#define SPEED_INCREMENT 0.05
#define DISTANCE_TOLERANCE 0.001
#define ANGLE_TOLERANCE 0.001

// robot geometry
#define WHEEL_RADIUS 0.05
#define LX 0.07 // longitudinal distance from robot's COM to wheel [m].
#define LY 0.07 // lateral distance from robot's COM to wheel [m].

class Base
{

private:
    // Private constructor to prevent instantiation
    Base()
    {
        std::cout << "Base instance created." << std::endl;
    }

    static void initBase()
    {
        instance.reset(new Base);
    }

private:
    static std::unique_ptr<Base> instance;
    static std::once_flag initInstanceFlag;

    std::vector<webots::Motor *> motor_;
    double robot_vx, robot_vy, robot_vtheta;

public:
    ~Base()
    {
        std::cout << "Base instance destroyed." << std::endl;
    }
    // Static method to get the instance of the Singleton class
    static Base &getInstance()
    {
        std::call_once(initInstanceFlag, &Base::initBase);
        return *instance;
    }

    // Delete copy constructor and assignment operator to prevent copies
    Base(const Base &) = delete;
    Base &operator=(const Base &) = delete;

    void init_motor(webots::Motor *fl, webots::Motor *fr, webots::Motor *rl, webots::Motor *rr)
    {
        this->motor_.reserve(4);
        this->motor_.push_back(fr);
        this->motor_.push_back(fl);
        this->motor_.push_back(rr);
        this->motor_.push_back(rl);
    }

    void base_reset()
    {
        static double speeds[4] = {0.0, 0.0, 0.0, 0.0};
        base_set_wheel_speeds_helper(speeds);
        robot_vx = 0.0;
        robot_vy = 0.0;
        robot_vtheta = 0.0;
    }

    void base_set_wheel_speeds_helper(double speeds[4])
    {
        int i;
        for (i = 0; i < 4; i++)
            base_set_wheel_velocity(motor_[i], speeds[i]);
    }
    void base_set_wheel_velocity(webots::Motor *m, double velocity)
    {
        m->setPosition(INFINITY);
        m->setVelocity(velocity);
    }
    void base_forwards()
    {
        static double speeds[4] = {SPEED, SPEED, SPEED, SPEED};
        base_set_wheel_speeds_helper(speeds);
    }
    void base_backwards()
    {
        static double speeds[4] = {-SPEED, -SPEED, -SPEED, -SPEED};
        base_set_wheel_speeds_helper(speeds);
    }
    void base_turn_left()
    {
        static double speeds[4] = {-SPEED, SPEED, -SPEED, SPEED};
        base_set_wheel_speeds_helper(speeds);
    }
    void base_turn_right()
    {
        static double speeds[4] = {SPEED, -SPEED, SPEED, -SPEED};
        base_set_wheel_speeds_helper(speeds);
    }
    void base_strafe_left()
    {
        static double speeds[4] = {SPEED, -SPEED, -SPEED, SPEED};
        base_set_wheel_speeds_helper(speeds);
    }
    void base_strafe_right()
    {
        static double speeds[4] = {-SPEED, SPEED, SPEED, -SPEED};
        base_set_wheel_speeds_helper(speeds);
    }

    void base_move(double vx, double vy, double vtheta)
    {
        double speeds[4];
        speeds[0] = 1 / WHEEL_RADIUS * (vx + vy + 0.085 * vtheta + (LX + LY) * vtheta);
        speeds[1] = 1 / WHEEL_RADIUS * (vx - vy - 0.085 * vtheta - (LX + LY) * vtheta);
        speeds[2] = 1 / WHEEL_RADIUS * (vx - vy - 0.085 * vtheta + (LX + LY) * vtheta);
        speeds[3] = 1 / WHEEL_RADIUS * (vx + vy + 0.085 * vtheta - (LX + LY) * vtheta);
        base_set_wheel_speeds_helper(speeds);
    }
    void base_forwards_increment()
    {
        robot_vx += SPEED_INCREMENT;
        robot_vx = robot_vx > MAX_SPEED ? MAX_SPEED : robot_vx;
        base_move(robot_vx, robot_vy, robot_vtheta);
    }
    void base_backwards_increment()
    {
        robot_vx -= SPEED_INCREMENT;
        robot_vx = robot_vx < -MAX_SPEED ? -MAX_SPEED : robot_vx;
        base_move(robot_vx, robot_vy, robot_vtheta);
    }
    void base_turn_left_increment()
    {
        robot_vtheta += SPEED_INCREMENT;
        robot_vtheta = robot_vtheta > MAX_SPEED ? MAX_SPEED : robot_vtheta;
        base_move(robot_vx, robot_vy, robot_vtheta);
    }
    void base_turn_right_increment()
    {
        robot_vtheta -= SPEED_INCREMENT;
        robot_vtheta = robot_vtheta < -MAX_SPEED ? -MAX_SPEED : robot_vtheta;
        base_move(robot_vx, robot_vy, robot_vtheta);
    }
    void base_strafe_left_increment()
    {
        robot_vy += SPEED_INCREMENT;
        robot_vy = robot_vy > MAX_SPEED ? MAX_SPEED : robot_vy;
        base_move(robot_vx, robot_vy, robot_vtheta);
    }
    void base_strafe_right_increment()
    {
        robot_vy -= SPEED_INCREMENT;
        robot_vy = robot_vy < -MAX_SPEED ? -MAX_SPEED : robot_vy;
        base_move(robot_vx, robot_vy, robot_vtheta);
    }
};

#endif