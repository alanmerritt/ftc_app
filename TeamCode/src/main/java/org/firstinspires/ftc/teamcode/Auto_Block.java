package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by MerrittAM on 11/8/2018.
 *
 */
@Autonomous(name = "Auto Block", group = "Autonomous")
public class Auto_Block extends Auto {
	
	
	
	@Override
	public void runOpMode() throws InterruptedException {
		
		initialize();
		
		waitForStart();
		
		driveForward(23, this);
		
		sleep(500);
		
		//Drive left to the leftmost object.
		
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		double distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);
		while(!isStopRequested() && robot.frontLeft.getCurrentPosition()/robot.INCH < 12)
		{
			
			robot.runMotors(-.5, -.5, .5, .5);
			
			
			
			
			telemetry.addData("Distance Traveled", robot.frontLeft.getCurrentPosition() / robot.INCH);
			
			telemetry.update();
			
		}
		robot.runMotors(0, 0, 0, 0);
		
		sleep(500);
		
		
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		while(!isStopRequested() && robot.frontLeft.getCurrentPosition()/robot.INCH > -12 && !isYellow())
		{
			
			robot.runMotors(.5, .5, -.5, -.5);
			
			telemetry.addData("Distance Traveled", robot.frontLeft.getCurrentPosition() / robot.INCH);
			
			telemetry.update();
			
		}
		robot.runMotors(0, 0, 0, 0);
		
		sleep(500);
		
		
	}
}
