package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by MerrittAM on 11/17/2018.
 */
@Autonomous(name = "Auto Crater Side To Depot", group = "Autonomous")
public class Auto_Crater_Side_To_Depot extends Auto {
	
	@Override
	public void runOpMode() throws InterruptedException {
		
		initialize();
		
		//robot.markerDropper.setPosition(robot.MARKER_DROPPER_CLOSED);
		
		waitForStart();
		
		driveForward(-20, this);
		
		while(robot.gyro.getYaw() < 60 && opModeIsActive())
		{
			robot.runMotors(.3, .3, .3, .3);
		}
		
		driveForward(-90, this);
		
		sleep(1000);
		
		//robot.markerDropper.setPosition(robot.MARKER_DROPPER_CLOSED);
		
		sleep(1000);
		
		while(robot.gyro.getYaw() < 90 && opModeIsActive())
		{
			robot.runMotors(.3, .3, .3, .3);
		}
		
		driveForward(-90, this);
		
		
		
	}
	
	void rotate(double degrees, double speed)
	{
		
		double yaw = robot.gyro.getTotalYaw();
		while(yaw > -degrees && !isStopRequested()) { //175
			robot.gyro.updateYaw();
			yaw = robot.gyro.getTotalYaw();
			telemetry.addData("Yaw", yaw);
			robot.runMotors(.3, -.3, -.3, .3);
		}
	
	}
	
}