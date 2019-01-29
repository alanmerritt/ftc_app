package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto;

/**
 * Created by MerrittAM on 1/26/2019.
 */
@Autonomous(name = "Full Crater",group = "Autonomous")
public class Full_Crater extends Auto {
	
	public void runOpMode() {
	
	initialize();
	
	waitForStart();
	
	lowerBot();
	
	sleep(2000);
	
	driveLeft(6,1,this);
	
	sleep(2000);
	
	driveForward(3,1,this);
	
	sleep(2000);
	
	driveRight(5,1,this);
	
	sleep(2000);
	
	knockoffcenter();
	
	    driveBackward(25,1,this);
		
		sleep(2000);
		
		driveLeft(45,1,this);
		
		sleep(2000);
		
		rotateCCW(90,1);
		
		sleep(2000);
		
		knockoffcenter();
		
		driveForward(13,1,this);
		
		robot.markerDropperdeposit();
		
		sleep(2000);
		
		rotateCW(25,.5);
		
		sleep(2000);
		
		driveForward(20,1,this);
		
		
		
		
		
		
		
		robot.markerDropperdeposit();
		
		
	}
	
	void knockoffcenter(){
	
	driveForward(20,.5,this);
		
		sleep(200);
		
	driveBackward(20,.5,this);
	
	
	
	}
	
	
	
}
