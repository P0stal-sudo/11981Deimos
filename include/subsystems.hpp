#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "subsystems.hpp"
#include "EZ-Template/tracking_wheel.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::Motor intake(10);
inline pros::MotorGroup lady_brown({11,-12});

inline ez::tracking_wheel horizontal_tracker(-19, 2, 0);

inline ez::tracking_wheel vertical_tracker(-20, 2, -2);
// inline ez::tracking_wheel right_tracker(14, 1.625, 4.0, 1.0);
// inline ez::tracking_wheel left_tracker(15, 1.625, 4.0, 1.0);
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');