package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by MerrittAM on 11/15/2018.
 */

@Autonomous(name = "Auto Navigation Test", group = "Autonomous")
public class Auto_Navigation_Test extends Auto {
	@Override
	public void runOpMode() throws InterruptedException {
		
		initialize();
		
		waitForStart();
		
		beginTracking();
		
		while(opModeIsActive())
		{
		
			runVision();
			telemetry.update();
		
		}
		
	}
}
