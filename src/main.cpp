#include "main.h" 
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "main.h"
#include "auton.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motor_group({1, 2}, pros::MotorGearset::blue);
pros::MotorGroup right_motor_group({3, 4}, pros::MotorGearset::blue);
pros::Motor intake(5);
pros::MotorGroup lift({7, 8}, pros::MotorGearset::green);
pros::Motor claw(6, pros::E_MOTOR_GEARSET_36, false);
pros::Motor rotationMech(9, pros::E_MOTOR_GEARSET_36, false);

pros::Imu imu(10);


enum class LiftState {
    ONE
    TWO
    THREE
    FOUR
};

current_lift_state = LiftState::ONE;

double getDegreesForState(LiftState state) {
    switch (state) {
        case LiftState::Bottom: return 0.0;
        case LiftState::Low:    return 300.0;
        case LiftState::High:   return 600.0;
        case LiftState::Top:    return 900.0;
        default:                return 0.0;
    }
}

// Cycle up through the presets
void cycleLiftUp() {
    if (selectedState == LiftState::Bottom)      selectedState = LiftState::Low;
    else if (selectedState == LiftState::Low)    selectedState = LiftState::High;
    else if (selectedState == LiftState::High)   selectedState = LiftState::Top;
}

// Cycle Down through the presets
void cycleLiftDown() {
    if (selectedState == LiftState::Top)         selectedState = LiftState::High;
    else if (selectedState == LiftState::High)   selectedState = LiftState::Low;
    else if (selectedState == LiftState::Low)    selectedState = LiftState::Bottom;
}

void moveLiftToState(LiftState state) {
    double targetDegrees = getDegreesForState(LiftState state);
    lift.move(targetDegrees, degrees);
}

void handleClaw() {
    int clawState = 0; // 0 for closed, 1 for open
    if ((clawState == 0) && (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))) {
        clawState = 1;
        claw.move(120);
        wait(0.5, seconds); 
        claw.stop(brake);
    }
    if ((clawState == 1) && (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))) {
        clawState = 0;
        claw.move(-120);
        wait(0.5, seconds); 
        claw.stop(brake);
    }
}

void pullFromIntake() {
    claw.move(120);
    wait(0.5, seconds); 
    claw.stop(brake);
    rotationMech.move_absolute(90, degrees);
}


lemlib::OdomSensors sensors(&imu);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        // move the robot
        chassis.curvature(leftY, leftX);

        // control the intake
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(120);
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(-120);
        } else {
            intake.brake();
        };


        //move the lift
        controller.ButtonL1.pressed(onUpPressed);   
        controller.ButtonL2.pressed(onDownPressed); 
        controller.ButtonA.pressed(onGoPressed);    

        controller.ButtonX.pressed(handleClaw);
        controller.ButtonX 
       
        }

        // delay to save resources
        pros::delay(25);
    }