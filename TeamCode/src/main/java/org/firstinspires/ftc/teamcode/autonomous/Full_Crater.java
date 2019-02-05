package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto;

/**
 * Created by MerrittAM on 1/26/2019.
 */
@Autonomous(name = "Full Crater",group = "Autonomous")
public class Full_Crater extends Auto {
	
	//TODO: !!!Tristin!!! Put some comments in the code!!!!!!
	
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
	
	//Start detecting the elements.
	activateElementDetection();
	
	//Detect the elements. Save the position (LEFT, CENTER,
	//RIGHT, or UNKNOWN) into the position variable.
	ElementPosition position = detectElement();
	
	//Deactivate element detection.
	deactivateElementDetection();
	
	//If the position is LEFT, run the knockOffLeft method.
	if(position == ElementPosition.LEFT)
	{
		//TODO: Add knockOffLeft() method.
		
	} //If the position is RIGHT, run the knockOffRight method.
	else if(position == ElementPosition.RIGHT)
	{
		knockOffRight();
	} //If the position is CENTER, run the knockOffCenter method.
	//If the position could not be determined, run the knockOffCenter method as a default.
	else if(position == ElementPosition.CENTER || position == ElementPosition.UNKNOWN)
	{
		knockOffCenter();
	}
	
			//knockoffcenter();
		
//		knockoffright();
		
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
	
	void knockOffCenter(){
	
	driveForward(25,.5,this);
		
		sleep(200);
		
	driveBackward(15,.5,this);
	
	
	
	}
	
	void knockOffRight()
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
