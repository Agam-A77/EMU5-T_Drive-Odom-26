#include "main.h"
#include "lemlib/api.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-10, 9, -8, 7});
pros::MotorGroup right_mg({-20, 19, -18, 17});

pros::Imu imu(11);
pros::Rotation horizontal_encoder(2);
pros::Rotation vertical_encoder(1);

//pros::adi::Encoder vertical_encoder('C', 'D', true);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, 0);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -3.34646);

lemlib::Drivetrain drivetrain(
    &left_mg,
    &right_mg,
    11.65, //track width deistance between the left and right centre of wheels
    lemlib::Omniwheel::NEW_4,
    600, //max speed of wheels in rpm
    8 // horizontal drift, 2 for omni and 8 for traction wheels
);

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

lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,
    nullptr,
    &horizontal_tracking_wheel,
    nullptr,
    &imu
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);


void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Calibrating IMU...");

    chassis.calibrate();

    pros::Task screen_task([]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(3, "Value: %f", horizontal_encoder.get_position());
            pros::delay(50);
        }
    });
}
void print_position(float  x, float y, float angle) {
    pros::lcd::print(0, "X: %f", x);
    pros::lcd::print(1, "Y: %f", y);
    pros::lcd::print(2, "Theta: %f", angle);
}
void disabled() {}
void competition_initialize() {}


void autonomous() {
    chassis.setPose(0, 0, 0);
}


void opcontrol() {
    while (true) {
        int dir = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        left_mg.move(turn - dir);
        right_mg.move(dir + turn);

        pros::delay(20);
    }
}