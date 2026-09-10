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

pros::MotorGroup left_motor_group({3, 4}, pros::MotorGearset::blue);
pros::MotorGroup right_motor_group({1, 2}, pros::MotorGearset::blue);
pros::Motor intake(5);
pros::MotorGroup lift({6, -7}, pros::MotorGearset::green);
pros::Motor claw(8, pros::v5::MotorGears::red);
pros::Motor rotationMech(9, pros::v5::MotorGears::red);
pros::Distance backdistance('A');
pros::Distance frontdistance('B');
pros::Distance leftdistance('C');
pros::Distance rightdistance('D');

pros::Imu imu(20);


enum class LiftState {
    Bottom,
    Low,
    High,
    Top
};

LiftState current_lift_state = LiftState::Bottom;

int rotationMechState = 1;
int clawState = 0;

// double getDegreesForState(LiftState state) {
//     switch (state) {
//         case LiftState::Bottom: return 0.0;
//         case LiftState::Low:    return 300.0;
//         case LiftState::High:   return 900.0;
//         case LiftState::Top:    return 1200.0;
//         default:                return 0.0;
//     }
// }

// void moveLiftToState(LiftState state) {
//     double targetDegrees = getDegreesForState(state);
//     lift.move_absolute(targetDegrees, 50);
// }

// // Cycle up through the presets and move the lift to the new preset
// void cycleLiftUp() {
//     if (current_lift_state == LiftState::Bottom)      current_lift_state = LiftState::Low;
//     else if (current_lift_state == LiftState::Low)    current_lift_state = LiftState::High;
//     else if (current_lift_state == LiftState::High)   current_lift_state = LiftState::Top;
//     moveLiftToState(current_lift_state);
// }

// // Cycle down through the presets and move the lift to the new preset
// void cycleLiftDown() {
//     if (current_lift_state == LiftState::Top)         current_lift_state = LiftState::High;
//     else if (current_lift_state == LiftState::High)   current_lift_state = LiftState::Low;
//     else if (current_lift_state == LiftState::Low)    current_lift_state = LiftState::Bottom;
//     moveLiftToState(current_lift_state);
// }

void zeroLift() {
    lift.move(-70);
    rotationMech.move_absolute(90, 100);
    claw.move(120);
    pros::delay(500);
    claw.brake();
    lift.brake();

}

void handleClaw() { // 0 for closed, 1 for open
    if (clawState == 0) {
        clawState = 1;
        claw.move(120);
        pros::delay(500);
        claw.brake();
    } else {
        clawState = 0;
        claw.move(-120);
        pros::delay(500);
        claw.move(-60);
    }
}

void handleRotation() {
    if (rotationMechState == 0) {
        rotationMechState = 1;
        rotationMech.move_absolute(-270, 100);
    } else {
        rotationMechState = 0;
        rotationMech.move_absolute(0, 100);
    }
}

void pullFromIntake() {
    clawState = 1;
    handleClaw();
    lift.move_absolute(50, 100);
    pros::delay(200);
    rotationMech.move_absolute(-270, 100);
    claw.move(-120);
    pros::delay(100);
    claw.move(-60);
    rotationMech.move_absolute(90, 100);
    rotationMech.brake();
}


lemlib::OdomSensors sensors(nullptr, // no vertical tracking wheel
                             nullptr, // no second vertical tracking wheel
                             nullptr, // no horizontal tracking wheel
                             nullptr, // no second horizontal tracking wheel
                             &imu);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              257, // drivetrain rpm is 360
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
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // actively hold position instead of coasting after claw.brake()
    rotationMech.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // resist gravity/external torque once at target
    chassis.calibrate(); // calibrate sensors

    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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
void autonomous() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.moveToPoint(10, 0, 5000);
    chassis.turnToHeading(90, 5000);
}

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
        // get left y (throttle) and right x (turn) positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        // turn is negated because swapping the left/right motor ports (3/4 <-> 1/2) reversed turn direction
        chassis.arcade(-rightX, leftY);

        // control the intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(120);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(-120);
        } else {
            intake.brake();
        };

        //move the lift
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        //     cycleLiftUp();
        // } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        //     cycleLiftDown();
        // }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            handleClaw();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            handleRotation();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            lift.move(90);  // Move Up (Full power)
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            lift.move(-90); // Move Down (Full power)
        } 
        else {
            lift.brake();      // Automatically brakes due to HOLD mode
        }


        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            zeroLift();
        }

        // delay to save resources
        pros::delay(25);
    }
}