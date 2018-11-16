package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by MerrittAM on 11/12/2018.
 * Program to test the computer vision navigation.
 */
@Autonomous(name = "Computer Vision Navigation Test", group = "Autonomous")
public class Computer_Vision_Navigation_Test extends LinearOpMode {
	
	//The last location matrix.
	OpenGLMatrix lastLocation = null;
	
	private VuforiaLocalizer vuforia;
	private VuforiaLocalizer.Parameters parameters;
	
	private int cameraMonitorViewId;
	
	private VuforiaTrackables roverRuckusTrackables;
	private VuforiaTrackable bluePerimiter;
	
	@Override
	public void runOpMode() {
		
		cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
		
		//Set the license key.
		parameters.vuforiaLicenseKey = "ARS60u//////AAAAGatg5bzrIElJruCZEPDqKr8mksRb99R0GEdJMfM4xVotZyXhiShn+ToKcAK2foRmNGNekn6uvxjmkdjbOlFvoQhDYJVBYvFF3afgz8aWcqo+WkdT3pXqnEcrPtMd4bz/CuC65ajgco231Ca7iUjqk7tuzv5Zg5gUpAfE2FulF0GIq6sXboe5OqrDxCLG+tA6oF24zuzFCEGZHUs8PDL3NwoA2KKbZttdoE13Kvqq9+AgBrqWeIYwefx9nkzWmn81QXHFd68APHaKyKT1PNxWKEK9aDL5vp4LRiG17AaBaGpXedpKVN4/o6GAWJm2zCOLwOb1aSMP8u7hDTXxsax0iMyEFYpzhshDar3HwD4xNy28";
		
		//Set the direction of the camera.
		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
		
		//Instantiate the vuforia.
		vuforia = ClassFactory.getInstance().createVuforia(parameters);
		
		//Load the file of trackables.
		roverRuckusTrackables = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
		//Get the first trackable - blue perimiter.
		bluePerimiter = roverRuckusTrackables.get(0);
		//Set the name.
		bluePerimiter.setName("BluePerimiter");
		
		final float mmPerInch = 24.4f;
		final float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;
		
		//Set the location of the target on the field.
		OpenGLMatrix bluePerimiterLocationOnField = OpenGLMatrix.translation(-mmFTCFieldWidth / 2, 0, 0)
				.multiplied(Orientation.getRotationMatrix(
						AxesReference.EXTRINSIC, AxesOrder.XZX,
						AngleUnit.DEGREES, 90, 90, 0
				));
		bluePerimiter.setLocation(bluePerimiterLocationOnField);
		
		//Set the location of the phone on the robot.
		OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
				.translation(0, 0, 0)
				.multiplied(Orientation.getRotationMatrix(
						AxesReference.EXTRINSIC, AxesOrder.YZY,
						AngleUnit.DEGREES, -90, 0, 0
				));
		
		//Factor in the phone position information.
		((VuforiaTrackableDefaultListener) bluePerimiter.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
		
		
		VectorF bluePerimiterTranslationVector = new VectorF(0, 0, mmFTCFieldWidth/2);
		
		waitForStart();
		
		
		roverRuckusTrackables.activate();
		
		while (opModeIsActive()) {
			
			//Determine whether or not the vumark is visible.
			telemetry.addData(bluePerimiter.getName(), ((VuforiaTrackableDefaultListener) bluePerimiter.getListener()).isVisible() ? "Visible" : "Not Visible");
			
			OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)bluePerimiter.getListener()).getPose();
			
			if(pose != null)
			{
				
				
				VectorF location = pose.getTranslation();
				location.add(bluePerimiterTranslationVector);
				
				telemetry.addLine("getPose");
				telemetry.addData("X", location.get(0));
				telemetry.addData("Y", location.get(1));
				telemetry.addData("Z", location.get(2));
			
			}
			
//			OpenGLMatrix loc = ((VuforiaTrackableDefaultListener)bluePerimiter.getListener()).getUpdatedRobotLocation();
//
//			if(loc != null)
//			{
//
//				VectorF location = loc.getTranslation();
//
//				telemetry.addLine("getUpdatedRobotLocation");
//				telemetry.addData("X", location.get(0));
//				telemetry.addData("Y", location.get(1));
//				telemetry.addData("Z", location.get(2));
//
//			}
			
			
			telemetry.update();
			
		}
		
	}
	
}
