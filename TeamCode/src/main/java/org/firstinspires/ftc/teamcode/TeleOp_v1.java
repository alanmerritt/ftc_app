package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by MerrittAM on 11/4/2018.
 * TeleOp v1.
 */
@TeleOp(name = "TeleOp v1", group = "TeleOp")
public class TeleOp_v1 extends OpMode {
	
	private Robot robot;
	
	//Edge detection for switching drive modes.
	private boolean driveButton = false;
	private boolean driveButtonLast = false;
	private boolean driveMode = false;
	

	
	@Override
	public void init() {
		
		robot = new Robot(this);
		
		
	}
	
	@Override
	public void loop() {
		
		double x = gamepad1.left_stick_x;
		double y = gamepad1.left_stick_y;
		double r = gamepad1.right_stick_x;
		
		//Update edge detection variable.
		driveButton = gamepad1.right_bumper;
		
		//If the button has been pressed, switch drive modes.
		if(driveButton && !driveButtonLast) {
			
			driveMode = !driveMode;
			
		}
		
		//Run a different drive mode depending on the mode.
		if (driveMode) {
			robot.driverCentricDrive(x, y, r);
			telemetry.addData("Drive Mode", "Field Centric");
		} else {
			robot.robotCentricDrive(x, y, -r);
			telemetry.addData("Drive Mode", "Robot Centric");
		}
		
		//Reset the gyroscope offset.
		if(gamepad1.y)
		{
			robot.offset = robot.gyro.getYaw();
		}
		
		//Run the arm.
		robot.runArm(gamepad2.left_stick_y);
		
		//Update the edge detection variables.
		driveButtonLast = driveButton;
		
		robot.extendanator.setPower(gamepad2.right_stick_y);
		
		
		if(gamepad2.a)
		{
			robot.collectorIntake();
		}
		else if(gamepad2.b)
		{
			robot.collectorDeposit();
		}
		else
		{
			robot.collectorServoStop();
		}
		
//		robot.collectorRotateForward(gamepad2.right_trigger);
//		robot.collectorRotateBackward(gamepad2.left_trigger);
//
//		telemetry.addData("Collector rotation", robot.collectorCurrentRotation);
//
		
		robot.collectorCurrentRotation += ((gamepad1.left_trigger - gamepad1.right_trigger) + (gamepad2.left_trigger - gamepad2.right_trigger))/150;
		robot.collectorCurrentRotation = Range.clip(robot.collectorCurrentRotation, 0, 1);
		robot.collectorRotator.setPosition(robot.collectorCurrentRotation);
		
		telemetry.addData("Left Trigger", gamepad1.left_trigger);
		telemetry.addData("Current Rotation", robot.collectorCurrentRotation);
		
//		telemetry.addData("Gyro Z (first)", robot.gyro.getOrientation().firstAngle);
//		telemetry.addData("Gyro Y (second)", robot.gyro.getOrientation().secondAngle);
//		telemetry.addData("Gyro X (third)", robot.gyro.getOrientation().thirdAngle);
		
		telemetry.update();
		
	}
}
