package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by MerrittAM on 11/10/2018.
 */
@Autonomous(name = "Auto Drop Relic", group = "Autonomous")
public class Auto_Drop_Relic extends Auto {
	@Override
	public void runOpMode() throws InterruptedException {
		
		initialize();
		
		double markerDropperOpen = 1;
		double markerDropperClosed = .5;
		
		robot.markerDropper.setPosition(markerDropperClosed);
		
		waitForStart();
		
		driveForward(-55, this);
		
		sleep(1000);
		
		robot.markerDropper.setPosition(markerDropperOpen);
		
		sleep(1000);
		
		driveForward(3, this);
		
	}
}
