package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto;

/**
 * Created by MerrittAM on 1/25/2019.
 */
@Autonomous(name = "Auto_Depot", group = "Autonomous")
public class Auto_Depot extends Auto {
	@Override
	public void runOpMode() throws InterruptedException {
		
		initialize();
		
		activateElementDetection();
		
		waitForStart();
		//!!! Lowering and repositioning. !!!
		lowerBot();
		sleep(100);
		
		driveLeft(6, 1, this);
		sleep(100);
		
		driveForward(3, .5, this);
		sleep(100);
		
		driveRight(4, .5, this);
		sleep(100);
		
		
		//robot.markerDropperretract();
		//Start detecting the elements.
//	activateElementDetection();
		
		//Detect the elements. Save the position (LEFT, CENTER,
		//RIGHT, or UNKNOWN) into the position variable.
		ElementPosition position = detectElement();
		
		//Deactivate element detection.
		deactivateElementDetection();
		
		//If the position is LEFT, run the knockOffLeft method.
		if (position == ElementPosition.LEFT) {
			knockOffLeft();
			
		} //If the position is RIGHT, run the knockOffRight method.
		else if (position == ElementPosition.RIGHT) {
			knockOffRight();
		} //If the position is CENTER, run the knockOffCenter method.
		//If the position could not be determined, run the knockOffCenter method as a default.
		else if (position == ElementPosition.CENTER) {
			knockOffCenter();
		} else {
			
			telemetry.addLine("This is Shermans fault(Warning!!! position could not be determined)!!!");
			telemetry.update();
			
			knockOffCenter();
			
		}
		
		driveForward(10,1,this);
		
		rotateCW(45,1);
		   sleep(100);
		   
		driveForward(50,1,this);
		   sleep(100);
		   
		rotateCW(50,1);
		   sleep(100);
		   
		driveLeft(25,1,this);
		   sleep(100);
		   
		driveForward(35,1,this);
		   sleep(100);
		
		
		
		
	}
		void knockOffCenter(){//justs drives forwards and backwards
			
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
		
		void knockOffLeft()  //make sure it doesnt hit the landers leg
		{
			
			driveForward(10,1,this);
			sleep(100);
			
			driveLeft(10,.7,this);
			sleep(100);
			
			driveForward(30,1,this);
			sleep(100);
			
			driveBackward(20,1,this);
			sleep(100);
			
			driveRight(15,1,this);
			sleep(100);
		}
		//squiggle is the end of the entire code
	
	
}
