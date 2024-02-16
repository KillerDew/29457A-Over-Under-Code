#include "main.h"
#include "lemlib/api.hpp"
#include "autoSelect/selection.h"


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
std::shared_ptr<OdomChassisController> chassis;
std::shared_ptr<AsyncPositionController<double, double>> IntakeController;
std::shared_ptr<AsyncVelocityController<double, double>> IntakeController_;
pros::Controller Master (pros::E_CONTROLLER_MASTER);
lemlib::Drivetrain_t drivetrain{
	&motors_left,
	&motors_right,
	12,
	3.25,
	200
};
pros::IMU imu (6);
lemlib::OdomSensors_t odomSensors{
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	&imu
};
lemlib::ChassisController_t lateralController{
	8, //Kp
	30, //Kd
	1, // small error range
	100, // small error timeout
	3, // large error range
	500, // large error timeout
	5 //slew rate
};
lemlib::ChassisController_t angularController{
	8, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
lemlib::Chassis lemChassis (drivetrain, lateralController, angularController, odomSensors);
pros::ADIDigitalOut Wings (DIGITAL_A);
void initialize() {
	pros::lcd::initialize();
	selector::init();
	chassis = 
		ChassisControllerBuilder()
			.withMotors(
				{-18, -20, -19},
				{8, 9, 10}
			)
			.withDimensions(AbstractMotor::gearset::green, {{3.75_in, 12_in}, imev5GreenTPR})
			.withOdometry({{3.25_in, 12_in}, quadEncoderTPR}, StateMode::FRAME_TRANSFORMATION)
			.buildOdometry();
	IntakeController_ = 
		AsyncVelControllerBuilder()
			.withMotor(-14)
			.withMotor(15)
			.withMaxVelocity(200)
			.build();
	

	
	


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
void competition_initialize() {
}

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
	lemChassis.calibrate();
	lemChassis.moveTo(0, 100, 5, 200);
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
pros::Motor Catapult (7);
pros::MotorGroup motors_left({18, 19, 20});
pros::MotorGroup motors_right({8, 9, 10});
pros::MotorGroup IntakeMotors({-14, 15});
double CataSpeed = 0.4;
double IntakeSpeed = 1;
void opcontrol() {
	autonomous();
	Catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Catapult.set_gearing(pros::E_MOTOR_GEARSET_36);
	bool BPressed = false;
	bool Extended = false;
	while(true) {
		// Retrieve the necessary joystick values
		int leftX = -Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		int leftY = -Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		
		// Move the left side of the robot
		motors_left.move(leftX + leftY);
		
		// Move the right side of the robot 
		motors_right.move(leftX - leftY);

		if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			Catapult.move_velocity(-100 * CataSpeed);
		}
		else{
			Catapult = 0;
		}
		if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			IntakeMotors.move(-127 * IntakeSpeed);
		}
		else if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			IntakeMotors.move(127 * IntakeSpeed);
		}
		else{
			IntakeMotors = 0;
		}
		if (!BPressed){
			if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
				BPressed = true;
				Extended = !Extended;
			}else{
				BPressed = false;
			}
		}
		Wings.set_value(Extended);
		pros::delay(20);
	}
}