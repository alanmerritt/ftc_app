package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto;

/**
 * Created by MerrittAM on 2/7/2019.
 */
@Autonomous(name = "TensorFlow Recognition Test", group = "Autonomous")
public class TensorFlow_Recognition_Test extends Auto {
	
	
	@Override
	public void runOpMode() throws InterruptedException {
		
		initialize();
		
		waitForStart();
		
		activateElementDetection();
		
		while(!isStopRequested())
		{
			
			ElementPosition position = detectElement();
			switch(position)
			{
				case LEFT:
					telemetry.addData("Position", "Left");
					break;
				case CENTER:
					telemetry.addData("Position", "Center");
					break;
				case RIGHT:
					telemetry.addData("Position", "Right");
					break;
				default:
					telemetry.addData("Position", "Unknown");
					break;
			}
			
			telemetry.update();
			
		}
		
		deactivateElementDetection();
		
	}
}
