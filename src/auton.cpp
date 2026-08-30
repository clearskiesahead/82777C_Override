#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "main.h"
#include <cstdio>
#include "auton.h"

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
    //robot switches toggle before compmleting rest of routine
    chassis.move(-10, inches);
    pros::c::delay(250);
    chassis.move(10, inches);
    pros::c::delay(250);
    //robot moves towards alliance goal
    chassis.move(-24, inches);
    chassis.turn(90, degrees);
    lift.move_absolute(300, degrees);
    chassis.move(-20, inches);
    //robot scores on alliance goal
    lift.move_absolute(200, degrees);
    claw.move(120);
    pros::c::delay(100);
    claw.stop();
    //robot backs away from alliance goal and moves towards nearby stack
    chassis.move(20, inches);
    chassis.turn(-45, degrees);
    chassis.setDriveVelocity(50, percent);
    chassis.drive(-23, inches);
    //robot picks up nearby stack and moves towards alliance goal
    claw.move(-120);
    pros::c::delay(100);
    claw.stop
    chassis.setDriveVelocity(100, percent);
    chassis.turn(-135, degrees);
    lift.move_absolute(300, degrees);
    chassis.move(20, inches);
    //robot scores on alliance goal
    lift.move_absolute(200, degrees);
    claw.move(120);
    pros::c::delay(100);
    claw.stop();
}

//main 15 second auton

void mainAuton() {
    chassis.setPose(); //insert values as x, y, sensor reading, theta
    chassis.moveToPoint()
}