package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;

/**
 * Created by MerrittAM on 11/14/2018.
 *
 */
@Autonomous(name = "Auto Blue Corner", group = "Autonomous")
public class Auto_Blue_Corner extends Auto {
	
	@Override
	public void runOpMode() throws InterruptedException {
	
		initialize();
		
		Vector left = new Vector();
		Vector center = new Vector();
		Vector right = new Vector();
		
		double leftR = 0;
		double centerR = 0;
		double rightR = 0;
		
		String calibrationFilePath = "/sdcard/FIRST/Auto_Blue_Setup.txt";
		
		try {
			
			//Load the data.
			
			FileInputStream datain = new FileInputStream(calibrationFilePath);
			String raw = "";
			int c;
			while((c = datain.read()) != -1)
			{
				raw +=  (char)c;
			}
			
			String[] data = raw.split(",");
			
			left.x = Double.valueOf(data[0]);
			left.y = Double.valueOf(data[1]);
			leftR = Double.valueOf(data[2]);
			center.x = Double.valueOf(data[3]);
			center.y = Double.valueOf(data[4]);
			centerR = Double.valueOf(data[5]);
			right.x = Double.valueOf(data[6]);
			right.y = Double.valueOf(data[7]);
			rightR = Double.valueOf(data[8]);
			telemetry.addData("Data Loaded", raw);
			
		} catch(FileNotFoundException ex)
		{
			telemetry.addLine("File could not be opened.");
		}
		catch (IOException ex)
		{
			telemetry.addLine("Unable to read data.");
		}
		telemetry.update();
		
		waitForStart();
		
		beginTracking();
		
		
		moveToPosition(left);
		
		telemetry.addLine("Movements Complete.");
		telemetry.update();
		sleep(1000);
		
		
	}
	
	
	
}
