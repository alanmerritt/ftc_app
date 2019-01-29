package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto;

/**
 * Created by MerrittAM on 1/24/2019.
 */
@Autonomous(name = "Auto_Corner", group = "Autonomous")
public class Auto_Corner extends Auto {
	
	@Override
	public void runOpMode() throws InterruptedException {
		
		//Initialize robot systems.
		initialize();
		
		waitForStart();
		
		lowerBot();
		
		sleep(1000);
		
//		driveBackward(2, 1, this);
		
		driveLeft(6, 1, this);
		
		sleep(1000);
		
		//Drive forward to the corner.
		driveForward(67, .8, this);
		
		//Drop the marker.
		robot.markerDropperdeposit();
		
		sleep(1000);
		
		//Reverse to drag the marker off the mechanism.
		driveBackward(3, .3, this);
		
		//Retract the mechanism.
		robot.markerDropperretract();
		
		//Rotate toward the opponent's crater.
		rotateCCW(40, .3);
		
		//Drive backward.
		driveBackward(12, .5, this);
		
		//Drive and bump into the wall to prevent running into the elements.
		driveLeft(6, .4, this);
		
		//Drive to the crater.
		driveBackward(63, .9, this);
		
	}
	
}
