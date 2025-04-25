#include "pros/rotation.hpp"
#include "main.h"

inline pros::Rotation rotate_sens(4);

const int num_states = 4;

int states[num_states] = {0,2300,14500,25000};

int curr_state = 0;

int target = 0;

void up_state() {
    curr_state += 1;
    if (curr_state == 4) {
        curr_state=0;
    }
    target = states[curr_state];
}

void down_state() {
    curr_state -= 1;
    if (curr_state == -1) {
        curr_state=3;
    }
    target = states[curr_state];
}

void set_state(int state) {
    curr_state = state;
     
    target = curr_state;
}

void lift_control() {    
    double error = target - rotate_sens.get_position();
    double kP = 0.012;
    double velocity = kP * error;
    lady_brown.move(velocity);
}