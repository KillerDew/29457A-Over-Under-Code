#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pid.hpp"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/impl/chassis/controller/chassisControllerBuilder.hpp"
#include "okapi/impl/device/rotarysensor/IMU.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <memory>
#include "lemlib/api.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

// Defining Controller
pros::Controller Master(pros::E_CONTROLLER_MASTER);

// --Defining Left and Right Drive groups--
pros::MotorGroup LeftDrive(
  // Left motors must be reversed in a drivetrain
    {-19, -18, -17});
pros::MotorGroup RightDrive(
    {9, 8, 7});
// --

// Defining intake motors
pros::MotorGroup Intake(
    {-1, 11});

// Defining Catapult
pros::Motor Catapult(-20, pros::E_MOTOR_GEAR_100);

// --Deifining Pneumatics--:
// Wing
pros::ADIDigitalOut Wing(1);
// Balance mechanism
pros::ADIDigitalOut BalanceMech('b');
//--

// Defining Speeds as a decimal value with 1 being 100% percent:
double CatapultSpeed = 0.4;
double TurnSpeed = .8;
double LatDriveSpeed = 1;
double IntakeSpeed = 1;

// Deadzones (where 1 is 100%, 0 is 0%)
double TurnDeadzone = 0.05;
double DriveDeadzone = 0.05;

// Defining IMU
pros::Imu imu (3);

enum DriveType{
  Tank,
  Arcade,
  Curvature,
  RC
};
lemlib::Drivetrain Lem_drivetrain{
  &LeftDrive,
  &RightDrive,
  12,
  3.25,
  360,
  5
};
lemlib::OdomSensors Lem_Sensors(
  nullptr,
  nullptr,
  nullptr,
  nullptr,
  &imu
);
lemlib::ControllerSettings Lat_Controller{
  10, // proportional gain (kP)
  0, // integral gain (kI)
  3, // derivative gain (kD)
  3, // anti windup
  1, // small error range, in inches
  100, // small error range timeout, in milliseconds
  3, // large error range, in inches
  500, // large error range timeout, in milliseconds
  20 // maximum acceleration (slew)
};
lemlib::ControllerSettings angularController{
  2, // proportional gain (kP)
  0, // integral gain (kI)
  10, // derivative gain (kD)
  3, // anti windup
  1, // small error range, in degrees
  100, // small error range timeout, in milliseconds
  3, // large error range, in degrees
  500, // large error range timeout, in milliseconds
  0 // maximum acceleration (slew)
};

lemlib::Chassis lemChassis(Lem_drivetrain, Lat_Controller, angularController, Lem_Sensors);

// --Autonomous Definitions--
// OdomChassisContoller - a built in okapilib class that alllws us to define a drivetrain with builtin odometry for autonomous.
std::shared_ptr<OdomChassisController> chassis = 
	ChassisControllerBuilder()
		.withMotors({-17, -18, -19},{7, 8, 9})
		.withDimensions({AbstractMotor::gearset::blue, (60.0/36.0)}, {{4_in, 13.6_in}, imev5BlueTPR})
		.withOdometry()
		.buildOdometry();
// --

// Initialize function - called when program begins - used for calibrating IMU and setting certain gearing configuraations.
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "29457A!");
  
  LeftDrive.set_gearing(pros::E_MOTOR_GEARSET_06);
  RightDrive.set_gearing(pros::E_MOTOR_GEARSET_06);
  pros::delay(1000);
  lemChassis.calibrate();
  BalanceMech.set_value(false);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
 // Currently not in use.
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
 // Currently not in use.
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
 **/

// FarSide - defines wether we are on far side (offensive, true) or near side (defensive, false)
bool FarSide = false;

void NewAuton(){
  return; // TODO: Do new auton
}

void autonomous() {
  NewAuton();
  if (FarSide){
    // Far side auton
    // Setting Chassis 
    chassis->setMaxVelocity(600);
    chassis -> setState({0_cm, 0_cm, 45_deg});

    // --Descoring triball--
    Wing.set_value(true);
    pros::delay(80);
    chassis -> moveDistance(-30_cm);
    Wing.set_value(false);
    // --

    // --Scoring 2 triballs--
    pros::delay(80);
    chassis -> turnToAngle(0_deg);
    chassis -> moveDistance(-65_cm );
    chassis -> moveDistance(30_cm);
    chassis -> turnToAngle(-40_deg);
    chassis -> moveDistance(-40_cm);
    // --
    //Experimenting with combining IMU (commented out for competition): 
    //double heading = imu.get();
    //chassis ->setState({0_ft, 0_ft, QAngle(heading)});

    // --Advanced auton - may be scrapped due to inconsistency--
    chassis -> moveDistance(25_cm);
    chassis -> turnToAngle(80_deg);
    chassis -> moveDistance(87.5_cm);
    chassis-> turnToAngle(196_deg);
    Intake = 127;
    chassis->setMaxVelocity(300);
    chassis -> moveDistance(55_cm);
    chassis->setMaxVelocity(600);
    chassis-> turnToAngle(350_deg);
    Intake = -127;
    // --
  }     
  else{
    // Near side auton
    // --Setting drivetrain--
    chassis-> setState({0_ft, 0_ft, 0_deg});
    chassis -> setMaxVelocity(600);
    // -- 

    // Descore triball--
    Wing.set_value(true);
    chassis -> turnToAngle(260_deg);
    chassis -> moveDistance(10_cm);
    chassis -> moveDistance(-10_cm);
    // --

    // Scoring alliance triball
    chassis -> turnToAngle(290_deg);
    chassis -> moveDistance(-30_cm);
    Wing.set_value(false);
    pros::delay(100);
    chassis -> moveDistance(-70_cm);
    chassis -> moveDistance(10_cm);
    chassis -> turnToAngle(10_deg);
    chassis -> moveDistance(-15_cm);
    // --

    // --Movement for AWP--
    chassis -> moveDistance(20_cm);
    chassis-> turnToAngle(300_deg);
    chassis -> moveDistance(50_cm);
    chassis-> turnToAngle(315_deg);

    // --
  }
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
DriveType DriverType = Curvature;
void opcontrol() {
  // MUST BE COMMENTED OUT FOR COMPETITION:
  autonomous();
  //--
  // --Setting Brake modes--
  Catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  Intake.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
  // --

  // --efining runtime variables--
  bool WingExtended = false;
  bool Balance = true;
  double elapsed = 0;
  // --

  // Driver control loop
  while (true) {
    // Normally, while true loops are a big no, however, in this case there is a delay at the end of the loop and the competition controller can cancel the task.
    /*
      ! DEPRACATED
    ///DCs, custom class that contains left and right motor velocities. (See CheesyDrive.cpp).
    DriveCommands DCs;
    ///Getting joystick inputs
    double Y = Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    double X = -Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
    ///Using CurvatureDrive function (CheesyDrive.cpp), uses specialised maths to alter how turning affects the robot. Returns DC
    DCs = CurvatureDrive(Y, X, TurnSpeed, LatDriveSpeed, TurnDeadzone, DriveDeadzone);
    ///Move left and right motors based on DCs.
    LeftDrive.move(DCs.left * 127);
    RightDrive.move(DCs.right * 127);
    ! --
    */
    int throttleL = Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int throttleR = Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    int turnL = -Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

    if (DriverType == Arcade){
      lemChassis.arcade(throttleL, turnL);
    }else if (DriverType == Tank){
      lemChassis.tank(throttleL, throttleR);
    }else if (DriverType == Curvature){
      lemChassis.curvature(throttleL, turnL);
    }
    // TODO: Implement RC (not nescessary)

    // If X is held down, run catapult - else stop cata
    if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      Catapult.move_velocity(100 * CatapultSpeed);
    } else {
      Catapult.move_velocity(0);
    }

    // If R1 is pressed down (single action), Toggle wings
    if (Master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
      WingExtended = !WingExtended;
      Wing.set_value(WingExtended);
    }
    // If Y is pressed, toggle balance mech
    if (Master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      Balance = !Balance;
      BalanceMech.set_value(!Balance);
    }

    // If L2 is pressed down, run Intake backwards, or if R2 is pressed, run intake forwards else stop intake.
    if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      Intake.move_velocity(-200 * IntakeSpeed);
    }else if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      Intake.move_velocity(200 * IntakeSpeed);
    }else{
      Intake.move_velocity(0);
    }
    /*
    ! DEPRECATED
    TODO: Fix hold timer
    /// -- If there is no activity, a timer starts that sets the motors to HOLD after 0.5s, so we are more stable --
    if (DCs.left == 0 && DCs.right ==0){
      elapsed += 20;
    }else {
      elapsed = 0;
    }
    */

    if (elapsed > 500){
      LeftDrive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
      RightDrive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    }else{
      LeftDrive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
      RightDrive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    }
    // --
    // Delay is CRITICAL for a while true loop to work.
    pros::delay(20);
  }
}