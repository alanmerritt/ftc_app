package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by MerrittAM on 11/17/2018.
 */

@Autonomous(name = "Auto Park On Crater", group = "Autonomous")
public class Auto_Park_On_Crater extends Auto {
	
	
	@Override
	public void runOpMode() throws InterruptedException {
		
		initialize();
		
		waitForStart();
		
		driveForward(-30, this);
		
	}
}
