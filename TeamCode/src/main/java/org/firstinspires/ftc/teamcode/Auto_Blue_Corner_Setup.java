package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

/**
 * Created by MerrittAM on 11/14/2018.
 *
 */
@Autonomous(name = "Auto Blue Corner Setup", group = "Autonomous")
public class Auto_Blue_Corner_Setup extends Auto {
	
	String calibrationFilePath = "/sdcard/FIRST/Auto_Blue_Setup.txt";
	
	@Override
	public void runOpMode() throws InterruptedException {
		
		initialize();
		
		
		String dataToWrite = "";
		
		waitForStart();
		
		beginTracking();
		
		//Left position.
		while(opModeIsActive() && !gamepad1.a)
		{
			telemetry.addLine("Move to left position. Press A to continue.");
			updateLocation();
			telemetry.addData("Location", getRobotLocation());
			telemetry.update();
		}
		
		//Add the data to the data to write.
		dataToWrite += getRobotLocation().x + "," + getRobotLocation().y + ",";
		
		sleep(1000);
		
		//Middle position.
		while(opModeIsActive() && !gamepad1.a)
		{
			telemetry.addLine("Move to center position. Press A to continue.");
			updateLocation();
			telemetry.addData("Location", getRobotLocation());
			telemetry.update();
		}
		
		//Add the data to the data to write.
		dataToWrite += getRobotLocation().x + "," + getRobotLocation().y + ",";
		
		sleep(1000);
		
		//Right position.
		while(opModeIsActive() && !gamepad1.a)
		{
			telemetry.addLine("Move to right position. Press A to continue.");
			updateLocation();
			telemetry.addData("Location", getRobotLocation());
			telemetry.update();
		}
		
		//Add the data to the data to write.
		dataToWrite += getRobotLocation().x + "," + getRobotLocation().y;
		
		sleep(1000);
		
		try {
			
			FileOutputStream dataout = new FileOutputStream(calibrationFilePath);
			
			//Write all the data.
			dataout.write(dataToWrite.getBytes());
			
			dataout.close();
			
			telemetry.addData("Data Written", dataToWrite);
			
		} catch (FileNotFoundException ex)
		{
			telemetry.addLine("Unable to open file.");
		}
		catch (IOException ex)
		{
			telemetry.addLine("Unable to write data.");
		}
		
		telemetry.update();
		
	}
}
