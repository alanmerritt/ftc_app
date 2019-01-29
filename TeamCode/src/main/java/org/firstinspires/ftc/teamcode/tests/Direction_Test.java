package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto;

/**
 * Created by MerrittAM on 1/25/2019.
 */
@Autonomous(name = "Direction Test", group = "Autonomous")
public class Direction_Test extends Auto {
	@Override
	public void runOpMode() throws InterruptedException {
		
		initialize();
		
		waitForStart();
		
		driveLeft(1000, 1, this);
		
	}
}
