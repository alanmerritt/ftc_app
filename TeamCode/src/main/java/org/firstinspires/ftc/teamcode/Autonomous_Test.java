package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by MerrittAM on 11/8/2018.
 *
 */
@Autonomous(name = "Autonomous Test", group = "Autonomous")
public class Autonomous_Test extends Auto {
	
	
	
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
		while(!isStopRequested() && robot.frontLeft.getCurrentPosition()/robot.INCH < 18 &&
				(Double.isNaN(distance) ? true : distance < 10))
		{
			
			robot.runMotors(-.5, -.5, .5, .5);
			distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);
			
			
			telemetry.addData("Distance Sensor", distance);
			telemetry.addData("Distance Traveled", robot.frontLeft.getCurrentPosition() / robot.INCH);
			
			telemetry.update();
			
		}
		robot.runMotors(0, 0, 0, 0);
		
		sleep(500);
		
		//Center the sensor on the object.
		
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		while(!isStopRequested() && (robot.frontLeft.getCurrentPosition()/robot.INCH > -1 ||
				distance < 2.5))
		{
			
			robot.runMotors(-.35, -.35, .35, .35);
			distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);
			
			telemetry.addLine("Additional Adjustment");
			telemetry.addData("Distance Sensor", distance);
			telemetry.addData("Distance Traveled", robot.frontLeft.getCurrentPosition() / robot.INCH);
			
			telemetry.update();
			
		}
		robot.runMotors(0, 0, 0, 0);
		
		
		sleep(500);
		
		//Drive closer to object
		while(!isStopRequested() && (Double.isNaN(distance) ? false : distance > 2.5))
		{
			
			robot.runMotors(.15, -.15, -.15, .15);
			distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);
			
			
			telemetry.addData("Distance Sensor", distance);
			
			telemetry.update();
			
		}
		robot.runMotors(0, 0, 0, 0);
		
		telemetry.addData("Red", robot.colorSensor.red());
		telemetry.addData("Green", robot.colorSensor.green());
		telemetry.addData("Blue", robot.colorSensor.blue());
		telemetry.update();
		
		sleep(2000);
		
		if(isYellow())
		{
			driveForward(2, this);
			sleep(500);
			driveForward(-3, this);
		}
		else {
			
			driveForward(-3, this);
			
			robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			robot.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);
			while (!isStopRequested() && robot.frontLeft.getCurrentPosition() / robot.INCH < 18 &&
					(Double.isNaN(distance) ? true : distance < 10)) {
				
				robot.runMotors(.5, .5, -.5, -.5);
				distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);
				
				
				telemetry.addData("Distance Sensor", distance);
				telemetry.addData("Distance Traveled", robot.frontLeft.getCurrentPosition() / robot.INCH);
				
				telemetry.update();
				
			}
			robot.runMotors(0, 0, 0, 0);
			
		}
		
		
	}
}
