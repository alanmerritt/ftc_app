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
	
	//rest of this line is for unhooking
	lowerBot();
	   sleep(200);
	
	driveLeft(6,1,this);
	   sleep(200);
	
	driveForward(3,1,this);
	   sleep(200);

	   driveRight(6,1,this);
	      sleep(200);




	   robot.collectorServo1.setPower(1);
	   robot.collectorServo2.setPower(-1);

	   sleep(5000);

	   robot.collectorServo1.setPower(0);
		robot.collectorServo2.setPower(0);


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
		knockOffLeft();
		
	} //If the position is RIGHT, run the knockOffRight method.
	else if(position == ElementPosition.RIGHT)
	{
		knockOffRight();
	} //If the position is CENTER, run the knockOffCenter method.
	//If the position could not be determined, run the knockOffCenter method as a default.
	else if(position == ElementPosition.CENTER)
	{
		knockOffCenter();
	}
	else
	{

		telemetry.addLine("Sherman you broke me i have a wife and kids how could you do this(Warning!!! position could not be determined)!!!");
		telemetry.update();

		knockOffCenter();

	}
	   //all of the methods end at the same spot to run the same line
			//knockoffcenter();
		
//		knockoffright();
		
		   sleep(200);
		
		rotateCCW(45,1);
		   sleep(200);
		
		driveLeft(3,1,this);
		   sleep(200);

		driveForward(25,1,this);
		   sleep(200);
		
		rotateCCW(100,1);
		   sleep(200);
		
		driveRight(35,.7,this);
		//robot is straighting its self out on the wall
		   sleep(200);
		
		driveLeft(8,1,this);
		//drives left so it doesnt get stuck on the wall
		   sleep(200);
		
		driveForward(55,1,this);
		   sleep(200);
		
		rotateCW(180,1);
		   sleep(200);

		   driveBackward(5,1,this);
		      sleep(200);

		   robot.markerDropperdeposit();
		//drops the team marker
		//in case the team marker gets stuck on the dropper it shakes it off
		driveBackward(10,1,this);
		   sleep(100);
		
		driveForward(5,1,this);
		   sleep(100);

		driveBackward(5,1,this);
		   sleep(100);
		
		driveForward(5,1,this);
		   sleep(100);
		//ending the shake off

		driveBackward(30,1,this);
		   sleep(200);
		
		rotateCCW(180,1);
		   sleep(200);

		   driveLeft(4,1,this);
		//puts it back up against the wall
		   sleep(200);

		driveRight(2,.3,this);
		   sleep(200);

		driveForward(20,1,this);
		   sleep(200);
		
		driveForward(15,.4,this);
		   sleep(200);
	}
	
	void knockOffCenter(){//justs drives forwards and backwards
	
		driveForward(25,.5,this);
		   sleep(200);
		
	driveBackward(15,.5,this);
	}

	void knockOffRight()
	{
	    //justs drives forward and knocks of the middle\
		// 	driveForward(5,1,this);
	    sleep(200);
	    
		driveForward(2,.5,this);
	       sleep(200);

	    driveRight(15,1,this);
		   sleep(200);
	    
	    driveForward(20,.5,this);
	       sleep(200);
	    
	    driveBackward(15,.5,this);
	       sleep(200);
	    
	    driveLeft(15,1,this);
	       sleep(200);
	}

	void knockOffLeft()//make sure it doesnt hit the landers leg
	{

	driveForward(5,1,this);
	   sleep(200);

	driveLeft(15,1,this);
	   sleep(200);

	driveForward(20,1,this);
	   sleep(200);

	driveBackward(20,1,this);
	   sleep(200);

	driveRight(15,1,this);
	   sleep(200);
	}
	//squiggle is the end of the entire code


}