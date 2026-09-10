#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cstdio>
#include "auton.h"

extern lemlib::Chassis chassis;
extern pros::Motor lift;
extern pros::Motor claw;
extern pros::Distance backdistance; 
extern pros::Distance frontdistance;
extern pros::Distance leftdistance;
extern pros::Distance rightdistance;

float getBack() {
    return (backdistance.get()/25.4) + 0; // change these values to adjust for the distance from the sensor to the center of the robot
}

float getFront() {
    return (frontdistance.get()/25.4) + 0; // change these values to adjust for the distance from the sensor to the center of the robot
}

float getLeft() {
    return (leftdistance.get()/25.4) + 0; // change these values to adjust for the distance from the sensor to the center of the robot
}

float getRight() {
    return (rightdistance.get()/25.4) + 0; // change these values to adjust for the distance from the sensor to the center of the robot
}

//preliminary auton using no sensors
void noSensorAuton() {
//     //robot switches toggle before compmleting rest of routine
//     chassis.move(-10);
//     pros::c::delay(250);
//     chassis.move(10);
//     pros::c::delay(250);
//     //robot moves towards alliance goal
//     chassis.move(-24);
//     chassis.turn(90);
//     lift.move_absolute(300);
//     chassis.move(-20);
//     //robot scores on alliance goal
//     lift.move_absolute(200);
//     claw.move(120);
//     pros::c::delay(100);
//     claw.stop();
//     //robot backs away from alliance goal and moves towards nearby stack
//     chassis.move(20);
//     chassis.turn(-45);
//     chassis.setDriveVelocity(50);
//     chassis.drive(-23);
//     //robot picks up nearby stack and moves towards alliance goal
//     claw.move(-120);
//     pros::c::delay(100);
//     claw.stop
//     chassis.setDriveVelocity(100);
//     chassis.turn(-135);
//     lift.move_absolute(300);
//     chassis.move(20);
//     //robot scores on alliance goal
//     lift.move_absolute(200);
//     claw.move(120);
//     pros::c::delay(100);
//     claw.stop();
}

//main 15 second auton

void mainAuton() {
    chassis.setPose(chassis.getPose().x, chassis.getPose().y, getFront(), chassis.getPose().theta); //insert values as x, y, sensor reading, theta
    //move away from toggle
    chassis.moveToPoint(-48, 0, 2000);
    //move towards alliance goal
    chassis.turnToHeading(-90, 2000);
    lift.move_absolute(300, 110);
    chassis.setPose(chassis.getPose().x, chassis.getPose().y, getLeft(), chassis.getPose().theta); //insert values as x, y, sensor reading, theta
    chassis.moveToPoint(-48, 22, 1000, {.forwards = false});
    //score in alliance goal
    lift.move_absolute(200, 110);
    claw.move(120);
    pros::c::delay(100);
    claw.brake();
    //move towards nearby stack
    chassis.moveToPoint(-48, 12, 1000);
    chassis.moveToPose(-23, -23, 135, 1000, {.forwards = false});
    //grab nearby stack
    claw.move(-120);
    pros::c::delay(100);
    claw.brake();
    //move towards alliance goal
    chassis.moveToPoint(-48, 12, 1000);
    chassis.turnToPoint(-48, 24, 1000);
    chassis.moveToPoint(-48, 22, 1000);
    lift.move_absolute(300, 110);
    //score on alliance goal
    lift.move_absolute(200, 110);
    claw.move(120);
    pros::c::delay(100);
    claw.brake();
    printf("Auton complete");
    printf("Final X: %f", chassis.getPose().x);
    printf("Final Y: %f", chassis.getPose().y);
}

void pidTest() {
     //insert values as x, y, sensor reading, theta
}