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

	activateElementDetection();

	telemetry.addLine("Ready to go!");
	telemetry.update();

	waitForStart();
	
	//rest of this line is for unhooking
	lowerBot();
	   sleep(100);
	
	driveLeft(6,1,this);
	   sleep(100);
	
	driveForward(3,1,this);
	   sleep(100);

	   driveRight(6,1,this);
	      sleep(100);
		
	      driveLeft(3,.5,this);
	
	      //Realign.
	      rotateCCW(2, .2);
	      
		//Start detecting the elements.
//	activateElementDetection();
	
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

		telemetry.addLine("This is Shermans fault(Warning!!! position could not be determined)!!!");
		telemetry.update();

		knockOffCenter();

	}
	   //all of the methods end at the same spot to run the same line
			//knockoffcenter();
		
//		knockoffright();
		
		rotateCCW(50,1);
		   sleep(100);
		
		driveRight(5,1,this);
		   sleep(100);
		   
		   driveForward(35,1,this);
		   sleep(100);
		
		rotateCCW(130,1);
		   sleep(100);

		   driveRight(7,1,this);
		      sleep(100);


		      driveForward(50,1,this);
		   sleep(100);	   //Lower arm.
		   robot.runArm(.5);
		   sleep(1000);
		   robot.stopArm();
		
			robot.extendanator.setPower(-.5);
			sleep(500);
			robot.extendanator.setPower(0);
		   
		   //Rotate the wrist.
		   robot.collectorRotator.setPosition(1);
		   
		   //Run the collector out.
		   robot.collectorDeposit();
		   
		   sleep(1000);
		   
		   //Stop the collector.
		   robot.collectorServoStop();

		driveBackward(50,1,this);
		   sleep(100);
		   
		driveBackward(10,.5,this);
		
	}
	
	
	
	
	void knockOffCenter(){//justs drives forwards and backwards
	
		rotateCCW(2,.2);
		   sleep(100);
		
		   driveLeft(3,1,this);
		      sleep(100);
		   
		driveForward(30,.5,this);
		   sleep(100);
		
	driveBackward(20,.5,this);
	}

	void knockOffRight()
	{
		driveForward(15,.5,this);
	       sleep(100);

	    driveRight(15,1,this);
		   sleep(100);
	    
	    driveForward(20,.5,this);
	       sleep(100);
	    
	    driveBackward(15,.5,this);
	       sleep(100);
	    
	    driveLeft(15,1,this);
	       sleep(100);

	    driveBackward(5,1,this);
	       sleep(100);
	}

	void knockOffLeft()//make sure it doesnt hit the landers leg
	{

	driveForward(10,1,this);
	   sleep(100);

	driveLeft(15,1,this);
	   sleep(100);

	driveForward(20,1,this);
	   sleep(100);

	driveBackward(15,1,this);
	   sleep(100);

	driveRight(15,1,this);
	   sleep(100);
	}
	//squiggle is the end of the entire code


}