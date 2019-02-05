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
	
	sleep(200);
	
	driveLeft(6,1,this);
	
	sleep(200);
	
	driveForward(3,1,this);
	
	sleep(200);
	
	driveRight(5,1,this);
	
	sleep(200);
	
	robot.markerDropperretract();
	
			//knockoffcenter();
		
		knockoffright();
		
		sleep(200);
		
		rotateCCW(45,1);
		
		sleep(200);
		
		driveForward(25,1,this);
		
		sleep(200);
		
		rotateCCW(100,1);
		
		sleep(200);
		
		driveRight(23,.4,this);
		
		sleep(200);
		
		driveLeft(8,1,this);
		
		sleep(200);
		
		driveForward(55,1,this);
		
		sleep(200);
		
		robot.markerDropperdeposit();
		
		driveForward(5,1,this);
		
		sleep(100);
		
		driveBackward(5,1,this);
		
		sleep(100);
		
		driveForward(5,1,this);
		
		sleep(100);
		
		driveBackward(5,1,this);
		
		sleep(100);
		
		driveBackward(30,1,this);
		
		sleep(200);
		
		driveRight(4,1,this);
		
		sleep(200);
		
		driveBackward(20,1,this);
		
		sleep(200);
		
		driveBackward(12,.4,this);
	}
	
	void knockoffcenter(){
	
	driveForward(25,.5,this);
		
		sleep(200);
		
	driveBackward(15,.5,this);
	
	
	
	}
	
	void knockoffright()
	{
	    
	    driveForward(5,1,this);
		
	    sleep(200);
	    
		driveRight(15,1,this);
	    
	    sleep(200);
	    
	    driveForward(15,.5,this);
	    
	    sleep(200);
	    
	    driveBackward(15,.5,this);
	    
	    sleep(200);
	    
	    driveLeft(15,1,this);
	
	}
	
	
	
}
