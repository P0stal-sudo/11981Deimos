#include "EZ-Template/util.hpp"
#include "pneumatics.hpp"
#include "lift.cpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

const int red = 1;
const int blue = 2;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void negative_6_alliance() {
  chassis.odom_xyt_set(-24_in, 12_in, 180_deg);

  set_state(2);
  pros::delay(500);
  intake.move(-127);

  chassis.pid_odom_set({{-24_in, 48_in, 180_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-44_in, 68_in, -45_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{-52_in, 68_in, -60_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  
  chassis.pid_odom_set({{-24_in, 48_in, -90_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{-48_in, 48_in, -90_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  doinker.set(true);

  chassis.pid_odom_set({{-64_in, 12_in, -135_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  doinker.set(false);

  chassis.pid_drive_set(12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-48_in, 6_in, 135_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  
  chassis.pid_odom_set({{-48_in, 6_in, 135_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{-24_in, 48_in, 45_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait(); 

  set_state(2);

  chassis.pid_drive_set(8_in, DRIVE_SPEED);
}

void positive_4_goal_rush(){
  chassis.odom_xyt_set(60_in, 12_in, 180_deg);

  set_state(2);
  pros::delay(500);
  intake.move(-127);

  chassis.pid_odom_set({{60_in, 48_in, 150_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{54_in, 68_in, 150_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  mogo.set(true);
  pros::delay(500);

  chassis.pid_odom_set({{48_in, 48_in, 180_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  doinker.set(true);

  chassis.pid_odom_set({{64_in, 12_in, 135_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  doinker.set(false);

  chassis.pid_drive_set(12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{48_in, 6_in, -135_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();  
  
  chassis.pid_odom_set({{24_in, 48_in, -45_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait(); 

  set_state(2);

  chassis.pid_drive_set(8_in, DRIVE_SPEED);
  chassis.pid_wait();
}

void positive_4_alliance_awp() {
  chassis.odom_xyt_set(12_in, 12_in, -135_deg);

  set_state(2);
  pros::delay(500);
  intake.move(-127);

  chassis.pid_odom_set({{24_in, 48_in, 180_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  mogo.set(true);

  pros::delay(500);

  chassis.pid_odom_set({{48_in, 48_in, 90_deg}, fwd, DRIVE_SPEED});

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  doinker.set(true);

  chassis.pid_odom_set({{64_in, 12_in, 135_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  doinker.set(false);

  chassis.pid_drive_set(12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{48_in, 6_in, -135_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{24_in, 48_in, -45_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait(); 

  set_state(2);

  chassis.pid_drive_set(8_in, DRIVE_SPEED);
  chassis.pid_wait();
}

void negative_6_alliance_awp() {
  chassis.odom_xyt_set(-12_in, 12_in, 135_deg);

  set_state(2);

  pros::delay(500);

  intake.move(-127);

  chassis.pid_odom_set({{-24_in, 48_in, 180_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  mogo.set(true);

  pros::delay(500);

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-44_in, 68_in, -45_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{-52_in, 68_in, -60_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  
  chassis.pid_odom_set({{-24_in, 48_in, -90_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{-48_in, 48_in, -90_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  doinker.set(true);

  chassis.pid_odom_set({{-64_in, 12_in, -135_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  doinker.set(false);

  chassis.pid_drive_set(12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-48_in, 6_in, 135_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{-24_in, 48_in, 45_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait(); 

  set_state(2);

  chassis.pid_drive_set(8_in, DRIVE_SPEED);
  chassis.pid_wait();
}
/*
void negative_8_solo_awp() {
  chassis.odom_xyt_set(-12_in, 12_in, 135_deg);

  set_state(2);

  pros::delay(500);

  intake.move(-127);

  chassis.pid_odom_set({{-24_in, 48_in, 180_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  mogo.set(true);

  pros::delay(500);

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-44_in, 68_in, -45_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{-52_in, 68_in, -60_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  
  chassis.pid_odom_set({{-24_in, 48_in, -90_deg}, rev, DRIVE_SPEED}); 
  chassis.pid_wait();

  chassis.pid_odom_set({{-48_in, 48_in, -90_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  
  intake_elev.set(false);

  mogo.set(false);

  chassis.pid_odom_set({{0_in, 24_in, 90_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  intake.move(0);

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{24_in, 48_in, -135_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  mogo.set(true);

  pros::delay(500);

  intake.move(-127);
  
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  set_state(2);

  chassis.pid_drive_set(8_in, DRIVE_SPEED);
  chassis.pid_wait();
}
*/
void skills() {

// first mogo

  chassis.odom_xyt_set(-12_in, 12_in, 135_deg);

  set_state(2);
  
  pros::delay(500);

  set_state(0);

  intake.move(-127);

  chassis.pid_odom_set({{-24_in, 24_in, -90_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  mogo.set(true);

  pros::delay(500);

  chassis.pid_odom_set({{-48_in, 24_in, -90_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{-60_in, 24_in, -90_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{-48_in, 24_in, -90_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-48_in, 12_in, 180_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  
  chassis.pid_odom_set({{-48_in, 24_in, 180_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-48_in, 48_in, 0_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{-48_in, 72_in, 0_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-24_in, 48_in, 90_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{-60_in, 12_in, 45_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  mogo.set(false);

//2nd mogo

  chassis.pid_odom_set({{24_in, 24_in, 90_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  mogo.set(true);

  pros::delay(500);

  chassis.pid_odom_set({{48_in, 24_in, 90_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{60_in, 24_in, 90_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{48_in, 24_in, 90_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{48_in, 12_in, 180_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{48_in, 24_in, 180_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{48_in, 48_in, 0_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{-48_in, 72_in, 0_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{24_in, 48_in, -90_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{60_in, 12_in, -45_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  mogo.set(false);

  chassis.pid_odom_set({{0_in, 72_in, -45_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  pros::delay(100);

  intake.move(0);

  chassis.pid_odom_set({{-24_in, 96_in, -22_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  
  intake.move(-127);

  pros::delay(100);

  intake.move(0);

  chassis.pid_odom_set({{0_in, 96_in, -135_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();
  
  mogo.set(true);

  pros::delay(500);

  intake.move(-127);

  chassis.pid_odom_set({{-48_in, 96_in, -90_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-48_in, 108_in, 0_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{-48_in, 120_in, 0_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{-60_in, 108_in, -135_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-60_in, 120_in, 135_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  mogo.set(false);

  // 4th mogo + wall stake

  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-24_in, 120_in, 135_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  mogo.set(true);

  pros::delay(500);

  set_state(1);

  intake.move(0);
  
  chassis.pid_odom_set({{48_in, 108_in, 135_deg}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  mogo.set(false);

  chassis.pid_odom_set({{24_in, 96_in, 0_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  
  set_state(2);
}