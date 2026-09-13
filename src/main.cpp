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

lemlib::TrackingWheel horizontal_tracking_wheel(&right_motor_group, lemlib::Omniwheel::NEW_4, -5.75, 600);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&left_motor_group, lemlib::Omniwheel::NEW_4, -2.5, 600);

pros::Motor intake(5);
pros::MotorGroup lift({6, -7}, pros::MotorGearset::green);
pros::Motor claw(8, pros::v5::MotorGears::red);
pros::Motor rotationMech(9, pros::v5::MotorGears::red);
pros::Distance backdistance('A');
pros::Distance frontdistance('B');
pros::Distance leftdistance('C');
pros::Distance rightdistance('D');

pros::Imu imu(20);

lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group, 10, lemlib::Omniwheel::NEW_4, 257, 2);


enum class LiftState {
    Bottom,
    Low,
    High,
    Top
};

LiftState current_lift_state = LiftState::Bottom;

int rotationMechState = 0; // 0 = resting at 0 (down), matching the mechanism's actual position at boot
int clawState = 0;

void zeroLift() {
    lift.move(-70);
    rotationMech.move_absolute(90, 100);
    rotationMechState = 0; // 90 is closer to the down (0) reference than up (-270)
    claw.move(120);
    pros::delay(500);
    claw.brake();
    clawState = 1; // claw ends up open
    lift.brake();

}

// Closes the claw at full power until it stalls against the pin (or a timeout elapses),
// then backs off to a lower holding voltage so it doesn't keep fighting something it already has.
void closeClawUntilStall() {
    const double stallVelocityThreshold = 5;  // RPM; below this counts as "not moving"
    const int stallReadingsNeeded = 5;        // consecutive low-velocity readings before declaring a stall
    const int rampUpGraceMs = 150;            // ignore the initial near-zero velocity while it's still starting to move
    const int maxCloseMs = 1500;              // safety timeout in case nothing is actually gripped

    claw.move(-120);
    pros::delay(rampUpGraceMs);

    int stalledReadings = 0;
    int elapsedMs = rampUpGraceMs;
    while (elapsedMs < maxCloseMs) {
        double actualVelocity = claw.get_actual_velocity();
        if (actualVelocity < 0) actualVelocity = -actualVelocity;

        if (actualVelocity < stallVelocityThreshold) {
            stalledReadings++;
            if (stalledReadings >= stallReadingsNeeded) break;
        } else {
            stalledReadings = 0;
        }

        pros::delay(10);
        elapsedMs += 10;
    }

    claw.brake(); // stop pushing once stalled (or after the timeout); HOLD brake mode keeps the grip
}

void handleClaw() { // 0 for closed, 1 for open
    if (clawState == 0) {
        clawState = 1;
        claw.move(120);
        pros::delay(500);
        claw.brake();
    } else {
        clawState = 0;
        closeClawUntilStall();
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
    // set claw to open
    clawState = 1;
    handleClaw();
    lift.move_absolute(50, 100);
    //move lift to position for claw to intake transition
    pros::delay(200);
    //face claw downwward
    rotationMech.move_absolute(-270, 100);
    rotationMechState = 1; // now at the up (-270) reference
    //grab from intake then apply constant claw pressure
    lift.move_absolute(0, 100);
    claw.move(-120);
    pros::delay(100);
    claw.move(-60);
    clawState = 0; // claw ends up closed
    lift.move_absolute(100, 100);
    rotationMech.move_absolute(90, 100);
    rotationMechState = 0; // 90 is closer to the down (0) reference than up (-270)
    rotationMech.brake();
}


lemlib::OdomSensors sensors(&vertical_tracking_wheel, // no vertical tracking wheel
                             nullptr, // no second vertical tracking wheel
                             nullptr, // no horizontal tracking wheel
                             nullptr, // no second horizontal tracking wheel
                             &imu);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
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
    claw.set_current_limit(1200); // cap current draw (default 2500mA) so sustained stall/hold pressure runs cooler
    rotationMech.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // resist gravity/external torque once at target

    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.calibrate(); 

    // forces the brain to pause here until the IMU is completely done calibrating
    while (imu.is_calibrating()) {
        pros::delay(10);
    }

    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); 
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); 
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); 
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
    printf("Cord-x: %f\n", chassis.getPose().x);
    printf("Cord-y: %f\n", chassis.getPose().y);
    printf("Heading: %f\n", chassis.getPose().theta);
    pros::delay(20);

    chassis.setPose(0, 0, 0);
    printf("Cord-x: %f\n", chassis.getPose().x);
    printf("Cord-y: %f\n", chassis.getPose().y);
    printf("Heading: %f\n", chassis.getPose().theta);
    pros::delay(20);

    pros::delay(3000);
    // turn to face heading 90 with a very long timeout
    chassis.moveToPoint(24, 0, 5000, {.maxSpeed = 50});
    printf("Cord-x: %f\n", chassis.getPose().x);
    printf("Cord-y: %f\n", chassis.getPose().y);
    printf("Heading: %f\n", chassis.getPose().theta);
    pros::delay(20);

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
            lift.move_velocity(100);  // Move Up (closed-loop RPM, consistent speed regardless of gravity)
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            lift.move_velocity(-70); // Move Down (closed-loop RPM, consistent speed regardless of gravity)
        }
        else {
            lift.brake();      // Automatically brakes due to HOLD mode
        }


        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            zeroLift();
        }

        pros::lcd::print(0, "X: %f", chassis.getPose().x); 
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); 
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); 
        
        printf("Cord-x: %f\n", chassis.getPose().x);
        printf("Cord-y: %f\n", chassis.getPose().y);
        printf("Heading: %f\n", chassis.getPose().theta);

        // delay to save resources
        pros::delay(25);
    }
}