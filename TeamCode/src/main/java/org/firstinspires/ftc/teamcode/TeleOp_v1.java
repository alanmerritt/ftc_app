package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by MerrittAM on 11/4/2018.
 * TeleOp v1.
 */
@TeleOp(name = "TeleOp v1", group = "TeleOp")
public class TeleOp_v1 extends OpMode {
	
	private Robot robot;
	
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
		
		driveButton = gamepad1.right_bumper;
		
		if(driveButton && !driveButtonLast) {
			
			driveMode = !driveMode;
			
		}
		
		if (driveMode) {
			robot.driverCentricDrive(x, y, r);
		} else {
			robot.robotCentricDrive(x, y, r);
		}
		
		if(gamepad1.y)
		{
			robot.offset = robot.gyro.getYaw();
		}
		
		driveButtonLast = driveButton;
		
	}
}
