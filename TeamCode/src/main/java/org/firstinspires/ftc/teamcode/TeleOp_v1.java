package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
	private DcMotor extendanator;

	
	@Override
	public void init() {
		
		robot = new Robot(this);
		
		extendanator= hardwareMap.dcMotor.get("extendanator");

		
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
			telemetry.addData("Drive Mode", "Driver Centric");
		} else {
			robot.robotCentricDrive(x, y, r);
			telemetry.addData("Drive Mode", "Field Centric");
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
		
		extendanator.setPower(gamepad2.right_stick_y);
		
		if(gamepad2.a)
		{

			robot.collectorServo1.setPower(1);
			robot.collectorServo2.setPower(-1);

		}

		telemetry.addData("Gyro Z (first)", robot.gyro.getOrientation().firstAngle);
		telemetry.addData("Gyro Y (second)", robot.gyro.getOrientation().secondAngle);
		telemetry.addData("Gyro X (third)", robot.gyro.getOrientation().thirdAngle);
		
		telemetry.update();
		
	}
}
