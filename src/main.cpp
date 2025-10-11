#include "main.h"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
// Motor intake(7);

// Negative turn is left
//Positive turn is right


Motor lIntake(7); // lower
Motor mfIntake (-14); // middle/transition
Motor mbIntake (20);
Motor uIntake(11); // upper
MotorGroup intakes({7,12,15,-14});
Imu inertial(19);

Motor leftFront = (3);
Motor rightFront = (6);
MotorGroup left_mg({-1, -2, -3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
MotorGroup right_mg({4, 5, 6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

adi::Pneumatics piston('a', false); //
adi::Pneumatics matchLoad('b', false);
adi::Pneumatics descore('c', false);
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
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	inertial.reset(true);
	pros::lcd::register_btn1_cb(on_center_button);
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


int speedLimit(int speed){
	if( speed > 90){
		return 90;
	}
	else if( speed < -90){
		return -90;
	}
	else{
		return speed;
	}
}

 int conversionTool (float inches){
	
	double wheelDiameter = 2.75,
		Pi = 3.1415,
		gearRatio = 4.0/3.0, // wheel gear / motor gear
		ticksPerRev = 300.0;

		return inches / (Pi * wheelDiameter) * ticksPerRev * gearRatio;
 }


void moveEncoder(int target, int time){ // forward and backward
	double kp = 1.6;
	double pastError;
	double derivative;
	double kd = 1.5; 
	int STF = millis();
	leftFront.tare_position(); // get position from this wheel
	double error = target - leftFront.get_position();
	while(fabs(target -leftFront.get_position()) > 0.5 && millis() - STF < time ){
		error = target -leftFront.get_position();
		left_mg.move(error * kp + derivative * kd);
		right_mg.move(error * kp - derivative * kd);
		pros::lcd::print(1, "error = %d",error ); // print error on screen (should be 0 if everything went well)
	}
	left_mg.move(0);// stop if no command is given
	right_mg.move(0);
}
void driveStraight(int distance, int time, int smallTimeOut){ // autocorrect/ forceful correct when driving in autonomous
	int ticks = conversionTool(distance);
	double kp = 0.6;//0.15;//double kp = 0.05; for auton
	double rkp = 1; // 0.1
	inertial.tare_rotation();
	double pastError;
	double derivative;
	double kd = 2.4; 

	int STF = millis(); // start time of function
	right_mg.tare_position();
	double rError = inertial.get_rotation();
	double error = ticks - right_mg.get_position();
	double tTime = millis();

	while(millis() - STF < time && millis() - tTime < smallTimeOut){//millis() - tTime < 500 , millis() - STF < time
		error = ticks - right_mg.get_position();
		derivative = pastError- error;
		rError = inertial.get_rotation();
		left_mg.move(speedLimit(error * kp - derivative * kd)  - (rError * rkp));
		right_mg.move(speedLimit(error * kp - derivative * kd)  + (rError * rkp));
		if(fabs(error) > 10){
 			tTime = millis(); 
		}
		pros::lcd::print(1, "error = %d",error);
		pros::lcd::print(2, "rError = %lf",inertial.get_rotation());
		pros::lcd::print(3, "ticks = %d", ticks);
		pastError = error;
		delay(20);
	}
	left_mg.move(0);
	right_mg.move(0);
}

void turnInertial(int degrees, int time, int smallTimeOut){ // turning, measured by the inertial sensor so it can be precise: counter clockwise --> negative/ clockwise --> positive
	inertial.tare_rotation();
	double kp = 2;//1.6;// auton 1.1
	double error = degrees - inertial.get_rotation();
	double pastError;
	double derivative;
	double kd = 3;
	int STF = millis();
	double tTime = millis();

	while(millis() - STF < time && millis() - tTime < smallTimeOut){// millis() - STF < time && millis() - tTime < 50
		error = degrees - inertial.get_rotation();
		derivative = pastError - error;
		left_mg.move(error * kp - derivative * kd);
		right_mg.move(-error * kp + derivative * kd);
		if(fabs(error) > 0.5){
 			tTime = millis(); 
		}
		pros::lcd::print(1, "error = %d",error);
		
		delay(20);
		pastError = error;
	}
	left_mg.move(0);
	right_mg.move(0);
}

void autonomous() {
	//right corners
		// matchLoad.extend();
		// driveStraight(32, 1000, 600);
		// turnInertial(93,600,400);
		// lIntake.move(127);
		// mfIntake.move(127);
		// mbIntake.move(127);
		// uIntake.move(-20);
		// driveStraight(10,800,400);
		// turnInertial(10,400,300);
		// turnInertial(-10, 400, 300);
		// delay(1200);
		// driveStraight(-16,1000, 400);
		// matchLoad.retract();
		// turnInertial(170,800,400);
		// driveStraight(7, 1000, 400);
		// turnInertial(12,800,400);
		// driveStraight(3, 1000, 400);
		// uIntake.move(127);
		// delay(1800);

			// piston.extend();
			// driveStraight(-10,1000, 500);
			// driveStraight(14,800,400);

		// driveStraight(-10,1000, 500);
		// turnInertial(-60,600,400);
		// driveStraight(25,800,500);



	// left corners
	//check config
		matchLoad.extend();
		driveStraight(33, 1000, 600);
		turnInertial(-90,600,400);
		lIntake.move(127);
		mfIntake.move(127);
		mbIntake.move(127);
		uIntake.move(-20);
		driveStraight(12,800,400);
		turnInertial(-15,400,300);
		turnInertial(15, 400, 300);
		delay(1200);
		driveStraight(-16,1000, 400);
		matchLoad.retract();
		// turnInertial(-170,800,400);
		// driveStraight(7, 1000, 400);
		// turnInertial(-12,800,400);
		// driveStraight(3, 1000, 400);
		// uIntake.move(127);
		// delay(1800);

		// driveStraight(-10,1000, 500);
		// turnInertial(60,600,400);
		// matchLoad.extend();
		// driveStraight(25,800,500);


		




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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	bool clampState = 0;
	
	while (true) {

		// Arcade control scheme
		int turn = master.get_analog(ANALOG_RIGHT_X);    // Gets amount forward/backward from left joystick
		int dir = master.get_analog(ANALOG_LEFT_Y);  // Gets the turn left/right from right joystick
		left_mg.move(dir + turn);                      // Sets left motor voltage
		right_mg.move(dir - turn);                     // Sets right motor voltage





		if (master.get_digital(DIGITAL_R1)){
			mfIntake.move(127);//127 is the max speed
			mbIntake.move(127);
			uIntake.move(127);
			lIntake.move(127);
			if(master.get_digital(DIGITAL_LEFT)) {
				uIntake.move(0);
			}
			if(master.get_digital(DIGITAL_UP)) {
				mfIntake.move(-127);
				mbIntake.move(-127);

			}
		}
		else if (master.get_digital(DIGITAL_R2)){
			mfIntake.move(-90);//-127
			mbIntake.move(-90);
			uIntake.move(-90);
			lIntake.move(-90);
		}
		else {
			mfIntake.move (0);
			mbIntake.move(0);
			uIntake.move(0);
			lIntake.move(0);
		}

		if (master.get_digital_new_press(DIGITAL_L2)){
			piston.toggle();
		}
		
		if (master.get_digital(DIGITAL_L1)) {
			lIntake.move(127);
			mfIntake.move(127);
			mbIntake.move(127);
			uIntake.move(-20);
		}

		if (master.get_digital(DIGITAL_RIGHT)){
			mfIntake.move(-127);
			mbIntake.move(127);
			uIntake.move(127);
			lIntake.move(127);
		}

		if (master.get_digital_new_press(DIGITAL_A)){
			matchLoad.toggle();
		}

		if (master.get_digital_new_press(DIGITAL_Y)){
			descore.toggle();
		}

				delay(20); 
		                 // Run for 20 ms then update
	}
}