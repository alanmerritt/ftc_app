package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto;

/**
 * Created by MerrittAM on 1/25/2019.
 */
@Autonomous(name = "Auto_Depot", group = "Autonomous")
public class Auto_Depot extends Auto {
	@Override
	public void runOpMode() throws InterruptedException {
		
		initialize();
		
		waitForStart();
		
		//!!! Lowering and repositioning. !!!
		lowerBot();
		
		sleep(1000);
		
		driveLeft(6, 1, this);
		
		sleep(500);
		
		driveForward(3, .5, this);
		
		sleep(500);
		
		driveRight(4, .5, this);
		
		sleep(500);
		
		//!!! ----------------------- !!!
		
		
		driveForward(60, .9, this);
		
		robot.markerDropperdeposit();
		
		sleep(1000);
		
		driveBackward(6, .4, this);
		
		sleep(500);
		
		
//
//		rotateCW(45, .4);
//
//		sleep(500);
		
//		driveBackward(6, .4, this);
//
//		sleep(500);
//
//		driveLeft(12, .4, this);
//
//		sleep(500);
		
		//!!! Drive to crater. !!!
		
		driveBackward(30, .9, this);
		
//		double power = -.75;
//
//		//While none of the robots have reached the target.
//		while(!isStopRequested() && robot.gyro.getOrientation().secondAngle > -5)
//		{
//
//			//Drive the robot forward.
//			robot.runMotors(power, -power, -power, power);
//
//		}
//
//		//Stop the motors and reset the encoders.
//		robot.runMotors(0, 0, 0, 0);
//
		//!!! ------------------------------ !!!
		
		sleep(500);
		
		driveLeft(50, .9, this);
		
		sleep(500);
		
		
		
	}
}
