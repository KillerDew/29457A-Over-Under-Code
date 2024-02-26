#include "main.h"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/impl/chassis/controller/chassisControllerBuilder.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <memory>

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

// Defining Controller
pros::Controller Master(pros::E_CONTROLLER_MASTER);
// Defining Left and Right Drive groups
pros::MotorGroup LeftDrive(
    {-19, -18, -17});
pros::MotorGroup RightDrive(
    {9, 8, 7});

// Defining intake motors
pros::MotorGroup Intake(
    {1, 11});
// Defining Catapult
pros::Motor Catapult(-20, pros::v5::MotorGears::red);
// Deifining Pneumatics
pros::adi::DigitalOut Wing(1);
pros::adi::DigitalOut BalanceMech(2);
// Defining Speeds as a decimal value with 1 being 100% percent:
double CatapultSpeed = 0.4;
double TurnSpeed = .8;
double LatDriveSpeed = 1;
double IntakeSpeed = 1;
// Deadzones
double TurnDeadzone = 0.05;
double DriveDeadzone = 0.05;



// Autonomous Definitions:
std::shared_ptr<OdomChassisController> chassis = 
	ChassisControllerBuilder()
		.withMotors({-17, -18, -19},{7, 8, 9})
		.withDimensions({AbstractMotor::gearset::blue, (60.0/36.0)}, {{4_in, 13.6_in}, imev5BlueTPR})
		.withOdometry()
		.buildOdometry();
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");

  pros::lcd::register_btn1_cb(on_center_button);
  LeftDrive.set_gearing_all(pros::E_MOTOR_GEARSET_06);
  RightDrive.set_gearing_all(pros::E_MOTOR_GEARSET_06);
  Intake.set_reversed(true, 0);
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
 **/
void autonomous() {
  chassis->setMaxVelocity(500);
	chassis -> setState({0_cm, 0_cm, 45_deg});
  Wing.set_value(true);
  pros::delay(80);
  chassis -> moveDistance(-30_cm);
  Wing.set_value(false);
  pros::delay(80);
  chassis -> turnToAngle(0_deg);
  chassis -> moveDistance(-65_cm );
  chassis -> moveDistance(20_cm);
  chassis -> turnToAngle(-40_deg);
  chassis -> moveDistance(-30_cm);
  chassis -> moveDistance(10_cm);
  chassis->setMaxVelocity(100);
  chassis -> turnToAngle(0_deg);
  chassis -> moveDistance(10_cm);
  chassis -> turnToAngle(40_deg);
  chassis->setMaxVelocity(500);  
  chassis -> moveDistance(65_cm);
  chassis -> turnToAngle(140_deg);
  chassis -> moveDistance(50_cm);
  chassis -> turnToAngle(145_deg);
  Intake.move( 127);
  chassis->setMaxVelocity(100);
  chassis -> moveDistance(25_cm);
  chassis -> moveDistance(-10_cm);
  chassis -> turnToAngle(240_deg);
  pros::delay(80);
  Intake.move( -127);
  chassis->setMaxVelocity(100);
  chassis -> turnToAngle(260_deg);
  chassis -> moveDistance(30_cm);
  Intake.move( 127);


  

  return;
	chassis -> driveToPoint({44_cm, 40_cm});
  Intake.move(-127);
  chassis -> driveToPoint({120_cm, 180_cm});
  chassis -> moveDistance(10_cm);
  chassis -> turnToPoint({60_cm, 170_cm});
  Intake.move(127);
  pros::delay(500);
  Intake.move(-127);
  chassis -> driveToPoint({120_cm, 170_cm});
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
  autonomous();
  Catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  Intake.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  bool WingExtended = false;
  bool Balance = false;
  double elapsed = 0;
  while (true) {
    DriveCommands DCs;
    double Y = Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    double X = -Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
    DCs = CurvatureDrive(Y, X, TurnSpeed, LatDriveSpeed, TurnDeadzone, DriveDeadzone);
    LeftDrive.move(DCs.left * 127);
    RightDrive.move(DCs.right * 127);

    if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      Catapult.move_velocity(100 * CatapultSpeed);
    } else {
      Catapult = 0;
    }

    if (Master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
      WingExtended = !WingExtended;
      Wing.set_value(WingExtended);
    }
    if (Master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      Balance = !Balance;
      Wing.set_value(Balance);
    }

    if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      Intake.move_velocity(-200 * IntakeSpeed);
    }else if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      Intake.move_velocity(200 * IntakeSpeed);
    }else{
      Intake = 0;
    }
    if (DCs.left == 0 && DCs.right ==0){
      elapsed += 20;
    }else {
      elapsed = 0;
    }
    if (elapsed > 500){
      LeftDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
      RightDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    }else{
      LeftDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
      RightDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    }
    pros::delay(20);
  }
}