package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by MerrittAM on 12/18/2018.
 */
@TeleOp(name = "Gear Reduction Test", group = "TeleOp")
public class Gear_Reduction_Test extends OpMode {
	
	DcMotor[] motors = new DcMotor[3];
	
	@Override
	public void init() {
		
		motors[0] = hardwareMap.dcMotor.get("motor0");
		motors[1] = hardwareMap.dcMotor.get("motor1");
		motors[2] = hardwareMap.dcMotor.get("motor2");
		
	}
	
	@Override
	public void loop() {
	
		double power = gamepad1.left_stick_y;
		
		for(int i = 0; i < motors.length; i++)
		{
			motors[i].setPower(power);		}
	
	}
}
