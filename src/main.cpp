#include "main.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include <cstdint>
#include "lemlib/api.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"


// Negative turn is left
//Positive turn is right


Motor Intake(-3); // lower
Motor uIntake(6); // upper

Imu inertial(5);
Rotation rotation_sensor(15);//placeholder port

// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&rotation_sensor, lemlib::Omniwheel::NEW_325, -2.5);

Motor leftFront = (10);
Motor rightFront = (-4);
MotorGroup left_mg({10, -14, -21});    // Creates a motor group with forwards ports 1 & 3 and reversed port 
MotorGroup right_mg({-4, 13, 16});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              600 * 0.75, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

adi::Pneumatics matchLoad('e', false);
adi::Pneumatics descore('c', false);
adi::Pneumatics transition('a', false);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &inertial // inertial sensor
);

lemlib::ControllerSettings lateral_controller(0.25, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0.5, // derivative gain (kD)
                                              2, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              6, // derivative gain (kD)
                                              2, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
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
	pros::lcd::initialize();
	inertial.reset(true);
	pros::lcd::register_btn1_cb(on_center_button);

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
	
	double wheelDiameter = 3.25,
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
	double kp = 0.25;// 0.6 for others
	double rkp = 0.1; // 0.1
	inertial.tare_rotation();
	double pastError;
	double derivative;
	double kd = 0.5; // 2.4 others

	int STF = millis(); // start time of function
	right_mg.tare_position();
	double rError = inertial.get_rotation();
	double error = ticks - right_mg.get_position();
	double tTime = millis();

	while( millis() - STF < time && millis() - tTime < smallTimeOut){//millis() - tTime < 500 , millis() - STF < time || millis() - STF < time && millis() - tTime < smallTimeOut
		
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
	double kp = 1.6;//1.6;// auton 1.1
	double error = degrees - inertial.get_rotation();
	double pastError;
	double derivative;
	double kd = 6;
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

void hold(int time, int speed) {
	leftFront.move(speed);
	rightFront.move(speed);
	delay(time);
	leftFront.move(0);
	rightFront.move(0);
}

void autonomous() {

	// Comment out other auton codes when doing each
	// use project.pros to name each code, assign to different slots
	// 1 tile length = 24 inches

	//right corners
		
		 
		// ---------------------------------below!!!!
		descore.extend();
		driveStraight(33, 1000, 600);
		delay(700);
		turnInertial(92,800,500);
		delay(300);
		driveStraight(-7,800,400); //backup
		matchLoad.extend();
		delay(300);
		Intake.move(127); //run Intake
		uIntake.move(0); //store
		driveStraight(14,800,400);
		hold(1200,45); //load from loader
		driveStraight(-29,1200, 800);
		matchLoad.retract();
		uIntake.move(127);
		delay(2600); // score in high goal

		uIntake.move(0);



		driveStraight(16,1000, 500);
		turnInertial(130,600,400); // turn 
		driveStraight(25,1000,800);
		delay(1000); // load blocks on the ground
		driveStraight(15,800,500);
		uIntake.move(-127); // score on low goal
		Intake.move(-127);
		
		delay(2500);
	

//------------------------------------------------------------------------------


// 	// left corners
// 	//check config-- below!!!!

// 		// ---------------------------------below!!!!
// 		descore.extend();
// 		driveStraight(33, 1000, 600);
// 		delay(700);
// 		turnInertial(-92,800,500);
// 		delay(300);
// 		driveStraight(-7,800,400); //backup
// 		matchLoad.extend();
// 		delay(300);
// 		Intake.move(127); //run Intake
// 		uIntake.move(0); //store
// 		driveStraight(14,800,400);
// 		hold(1200,45); //load from loader
// 		driveStraight(-29,1200, 800);
// 		matchLoad.retract();
// 		uIntake.move(127);
// 		delay(2600); // score in high goal

// 		uIntake.move(0);

// 		driveStraight(16,1000, 500);
// 		turnInertial(-130,600,400); // turn 
// 		driveStraight(27,1000,800);
// 		delay(1000); // load blocks on the ground
// 		turnInertial(180,800,500); //turn
// 		driveStraight(-15,800,500);
// 		uIntake.move(-127); // score on low goal
// 		Intake.move(-127);
		
// 		delay(2500);
	

// 	//-------------------------------------------------------------------
	
// 	//Skills

// 		//set 1: left 2 loaders, 1 long goal , clear & park
// 		descore.extend();
// 		driveStraight(33, 1000, 600);
// 		delay(700);
// 		turnInertial(-92,800,500);
// 		delay(300);
// 		driveStraight(-7,800,400); //backup
// 		matchLoad.extend();
// 		delay(300);
// 		Intake.move(127); //run Intake
// 		uIntake.move(0); //store
// 		driveStraight(14,800,400);
// 		hold(1200,45); //load from loader
// 		driveStraight(-10,1000, 400);
// 		matchLoad.retract();
// 		Intake.move(0);

// 		uIntake.move(0);
// 		turnInertial(135,600,400);
// 		driveStraight(19,600,300);
// 		turnInertial(35,500,250);
// 		driveStraight(72,4000,3000); // drive to other side
// 		turnInertial(90,400,300);
// 		driveStraight(-15,600,500);// hard reset
// 		driveStraight(16,800,600);
// 		delay(1000);

		
// 		delay(1000);
// 		turnInertial(-90,400,300);
// 		driveStraight(-20,1000,800);
// 		Intake.move(127);
// 		uIntake.move(127);
// 		delay(2600); // score in high goal 1- 1
// 		matchLoad.extend();
// 		delay(300);
// 		uIntake.move(0);

// 		driveStraight(28,1000,800);
// 		hold(1000,15); //load from loader 2
// 		driveStraight(-30,1200, 800);
// 		matchLoad.retract();
// 		Intake.move(127);
// 		uIntake.move(127);
// 		delay(2600); // score in high goal 1-2
// 		Intake.move(0);
// 		uIntake.move(0);

// 		driveStraight(-8,1000, 500);// drive back
// 		turnInertial(90,400,300);
// 		driveStraight(20, 3000,2500);
// 		turnInertial(75,400,300);
// 		delay(1000);
// 		driveStraight(90, 3000,2500); //come back
// 		turnInertial(-90,400,300);
// 		uIntake.move(127); // clear parking
// 		driveStraight(30,800,700);
// 		driveStraight(-16,500,400);// backup & park



// 		//--------------------------------------------------------------------------
// 		//Set 2: Full: 4 loaders, 2 goals, clear and park

// 		descore.extend();
// 		driveStraight(33, 1000, 600);
// 		delay(700);
// 		turnInertial(-92,800,500);
// 		delay(300);
// 		driveStraight(-7,800,400); //backup
// 		matchLoad.extend();
// 		delay(300);
// 		Intake.move(127); //run Intake
// 		uIntake.move(0); //store
// 		driveStraight(14,800,400);
// 		hold(1200,45); //load from loader
// 		driveStraight(-10,1000, 400);
// 		matchLoad.retract();
// 		Intake.move(0);

// 		uIntake.move(0);
// 		turnInertial(135,600,400);
// 		driveStraight(19,600,300);
// 		turnInertial(35,500,250);
// 		driveStraight(72,4000,3000); // drive to other side
// 		turnInertial(90,400,300);
// 		driveStraight(-15,600,500);// hard reset
// 		driveStraight(16,800,600);
// 		delay(1000);

		
// 		delay(1000);
// 		turnInertial(-90,400,300);
// 		driveStraight(-20,1000,800);
// 		Intake.move(127);
// 		uIntake.move(127);
// 		delay(2600); // score in high goal 1- 1
// 		matchLoad.extend();
// 		delay(300);
// 		uIntake.move(0);

// 		driveStraight(28,1000,800);
// 		hold(1000,15); //load from loader 2
// 		driveStraight(-30,1200, 800);
// 		matchLoad.retract();
// 		Intake.move(127);
// 		uIntake.move(127);
// 		delay(2600); // score in high goal 1-2
// 		Intake.move(0);
// 		uIntake.move(0);
// 		driveStraight(18,800,600);
// 		turnInertial(90,600,500);
// 		driveStraight(80,1600,1400); // go to loader 3
// 		turnInertial(-90,600,500);	

// 		matchLoad.extend();
// 		Intake.move(127);
// 		driveStraight(28,1000,800);
// 		hold(1000,15); //load from loader 3
// 		driveStraight(-27,1200, 800);
// 		matchLoad.retract();	

// 		Intake.move(0);
// 		uIntake.move(0);
// 		turnInertial(135,600,400);
// 		driveStraight(19,600,300);
// 		turnInertial(35,500,250);
// 		//descore.extend();
// 		driveStraight(72,4000,3000); // drive to other side
// 		turnInertial(90,400,300);
// 		driveStraight(-15,600,500);// hard reset
// 		driveStraight(16,800,600);
// 		delay(1000);

// 		turnInertial(-90,400,300);
// 		driveStraight(-20,1000,800);
// 		Intake.move(127);
// 		uIntake.move(127);
// 		delay(2600); // score in high goal 2- 1
// 		matchLoad.extend();
// 		delay(300);
// 		uIntake.move(0);

// 		driveStraight(28,1000,800);
// 		hold(1000,15); //load from loader 4
// 		driveStraight(-28,800,700);
// 		uIntake.move(127);
// 		delay(2600); // score in high goal 2- 2
// 		matchLoad.retract();
// 		uIntake.move(0);

// 		driveStraight(20,800,700);
// 		turnInertial(-90,600,400);
// 		driveStraight(-20,800,700);
// 		turnInertial(90,600,400);
// 		driveStraight(10,800,700);
// 		turnInertial(90,600,400);

// 		uIntake.move(127); // clear parking
// 		driveStraight(30,800,700);
// 		driveStraight(-16,500,400);// backup & park

		

// //----------------------------------------------
// //backup & park

// 		uIntake.move(127); // clear parking
// 		Intake.move(127);
// 		driveStraight(30,800,700);
// 		driveStraight(-16,500,400);// backup & park



//-------------------------------------------------------------------------

// chassis.turnToHeading(90, ); //turning- degrees
// chassis.turnToPoint(float x, float y, int timeout); // move with coordinates
// chassis.setPose(0,0,0); // set current position as 0

// chassis.swingToHeading(float theta, int timeout);// swing to face x degrees
// chassis.swingToPoint(float x, float y, int timeout); // swing to face the point 

// chassis.moveToPoint(float x, float y, int timeout);// move to (x,y)- straight
// chassis.moveToPose(float x, float y, float theta, int timeout)//  move to (x,y)- with turn


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
	pros::lcd::set_text(1, "Test!");
	
	uIntake.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	while (true) {

		// Arcade control scheme
		int turn = master.get_analog(ANALOG_RIGHT_X) * 0.85;    // Gets amount forward/backward from left joystick
		int dir = master.get_analog(ANALOG_LEFT_Y)* 0.85;  // Gets the turn left/right from right joystick
		left_mg.move(dir + turn);                      // Sets left motor voltage
		right_mg.move(dir - turn);                     // Sets right motor voltage



		//
			if (master.get_digital(DIGITAL_R1)) { // long goal store // S L1
				Intake.move(127);// up
				uIntake.move(0);// up
				// transition.extend();
			}
			else if (master.get_digital(DIGITAL_R2)) { // outtake 
				Intake.move(-127); // down
				uIntake.move(-127); // down
			
			}

			else if (master.get_digital (DIGITAL_L1)) { //scoring long goal
				Intake.move(127);// up
				uIntake.move(127);// up
			}
			else if (master.get_digital(DIGITAL_L2)) { //middle
				transition.retract();
				Intake.move (127);
				uIntake.move(-127);
			}

			else {
				Intake.move(0);
				uIntake.move(0);
			}
		

		if (master.get_digital_new_press(DIGITAL_Y)){ // matchload //S X
			matchLoad.toggle();
		}

		if (master.get_digital_new_press(DIGITAL_X)){ // descore // SY
			descore.toggle(); 
		}

		if (master.get_digital_new_press(DIGITAL_B)){ // transition in L2 
			transition.toggle();
		}

		
				delay(20); 
		                 // Run for 20 ms then update
	}
}
