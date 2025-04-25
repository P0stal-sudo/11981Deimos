#include "main.h"
//0 means no sort, 1 throws blue rings, and 2 throws red ring
int throw_color = 0;

pros::Optical optical(5);

void change_color(int new_color) {
    throw_color = new_color;
}

void controller_change_color() {
    throw_color += 1;
    if (throw_color == 2) {
        throw_color = 0;
    }
}

void color_sort() {
    double target = 60;
    if (throw_color == 1) {
        target = 240;
    }
    if (throw_color == 2) {
        target = 0;
    }
    if (optical.get_hue() >= target-10 && optical.get_hue() <= target + 10) {
        pros::delay(200);
        intake.move(0);
        pros::delay(50);
        intake.move(-127);
    }
}