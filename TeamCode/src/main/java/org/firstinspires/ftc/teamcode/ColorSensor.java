package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by MerrittAM on 11/9/2018.
 */
@TeleOp(name = "Color Sensor", group = "TeleOp")
public class ColorSensor extends OpMode {
	
	Robot robot;
	
	@Override
	public void init() {
		
		robot = new Robot(this);
		
	}
	
	@Override
	public void loop() {
		
		telemetry.addData("Red", robot.colorSensor.red());
		telemetry.addData("Green", robot.colorSensor.green());
		telemetry.addData("Blue", robot.colorSensor.blue());
		telemetry.addData("Distance", robot.distanceSensor.getDistance(DistanceUnit.INCH));
		
		telemetry.update();
		
	}
}
