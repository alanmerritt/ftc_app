package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by MerrittAM on 11/17/2018.
 */
@Autonomous(name = "Auto Blue Corner Drop", group = "Autonomous")
public class Auto_Blue_Corner_Drop extends Auto {
	
	@Override
	public void runOpMode() throws InterruptedException {
		
		initialize();
		
		//robot.markerDropper.setPosition(robot.MARKER_DROPPER_CLOSED);
		robot.markerDropperretract();
		waitForStart();
		
		driveForward(-67, this);
		robot.runMotors(0, 0, 0, 0);
		
		robot.markerDropperdeposit();
		sleep(1000);
		robot.markerDropperretract();
		
		//robot.markerDropper.setPosition(robot.MARKER_DROPPER_OPEN);
		sleep(1000);
		
		while(robot.gyro.getYaw() > -40 && opModeIsActive())
		{
			robot.runMotors(-.3, -.3, -.3, -.3);
		}
		
		driveForward(75, this);
		robot.runMotors(0, 0, 0, 0);
		
	}
}
