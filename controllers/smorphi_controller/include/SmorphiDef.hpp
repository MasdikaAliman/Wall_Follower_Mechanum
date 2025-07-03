#ifndef SMOR_DEF_HPP
#define SMOR_DEF_HPP

struct Velocity_t
{
    Velocity_t()
    {
        vx = 0;
        vy = 0;
        vtheta = 0;
    }

    Velocity_t(double vx_, double vy_, double vtheta_)
        : vx(vx_), vy(vy_), vtheta(vtheta_)
    {
    }
    // Operator tambah (+)
    Velocity_t operator+(const Velocity_t& other) const {
        return Velocity_t(vx + other.vx, vy + other.vy, vtheta + other.vtheta);
    }

    // Operator kurang (-)
    Velocity_t operator-(const Velocity_t& other) const {
        return Velocity_t(vx - other.vx, vy - other.vy, vtheta - other.vtheta);
    }

    // Operator kali (*)
    Velocity_t operator*(const Velocity_t& other) const {
        return Velocity_t(vx * other.vx, vy * other.vy, vtheta * other.vtheta);
    }

    // Operator bagi (/)
    Velocity_t operator/(const Velocity_t& other) const {
        return Velocity_t(vx / other.vx, vy / other.vy, vtheta / other.vtheta);
    }

    double vx, vy, vtheta;
};

#endif