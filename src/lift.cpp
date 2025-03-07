#include "pros/rotation.hpp"
#include "main.h"

inline pros::Rotation rotate_sens(4);

const int num_states = 3;

int states[num_states] = {0,-2300,-14500};

int curr_state = 0;

int target = 0;

void up_state() {
    curr_state += 1;
    if (curr_state == 3) {
        curr_state=0;
    }
    target = states[curr_state];
}

void down_state() {
    curr_state -= 1;
    if (curr_state == -1) {
        curr_state=2;
    }
    target = states[curr_state];
}

void lift_control() {    
    double error = target - rotate_sens.get_position();
    double kP = 0.012;
    double velocity = kP * error;
    lady_brown.move(velocity);
}