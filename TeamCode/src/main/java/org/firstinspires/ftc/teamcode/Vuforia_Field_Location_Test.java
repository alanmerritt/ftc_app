package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Trackable;
import com.vuforia.Vuforia;

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

import java.util.ArrayList;

/**
 * Created by MerrittAM on 11/13/2018.
 * Program to test the Vuforia computer vision
 * to determine the location on the field.
 */
@Autonomous(name = "Vuforia Field Location Test", group = "Autonomous")
public class Vuforia_Field_Location_Test extends LinearOpMode {
	
	private VuforiaLocalizer vuforia;
	private VuforiaLocalizer.Parameters parameters;
	
	private int cameraMonitorViewId;
	
	private VuforiaTrackables roverRuckusTrackables;
	private ArrayList<VuforiaTrackable> trackables;
	
	//private VectorF bluePerimiterLocationOnField;
	
	@Override
	public void runOpMode() throws InterruptedException {
		
		initializeVuforia();
		
		float mmPerInch = 24.5f;
		float mmFTCFieldWidth = (12*12-2)*mmPerInch;
		
		OpenGLMatrix bluePerimiterLocationOnField = OpenGLMatrix
				.translation(0, mmFTCFieldWidth/2, 0)
				.multiplied(Orientation.getRotationMatrix(
						AxesReference.EXTRINSIC, AxesOrder.XZX,
						AngleUnit.DEGREES, 90, 0, 0
				));
		trackables.get(0).setLocation(bluePerimiterLocationOnField);
		
		OpenGLMatrix redPerimiterLocationOnField = OpenGLMatrix
				.translation(-mmFTCFieldWidth/2, 0, 0)
				.multiplied(Orientation.getRotationMatrix(
						AxesReference.EXTRINSIC, AxesOrder.XZX,
						AngleUnit.DEGREES, 90, 180, 0
				));
		trackables.get(1).setLocation(redPerimiterLocationOnField);
		
		OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
				.translation(0, 0, 0)
				.multiplied(Orientation.getRotationMatrix(
						AxesReference.EXTRINSIC, AxesOrder.YZY,
						AngleUnit.DEGREES, 0, 0, 0
				));
		
		((VuforiaTrackableDefaultListener)trackables.get(0).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
		
		waitForStart();
		
		//Start looking for the trackables.
		roverRuckusTrackables.activate();
		
		OpenGLMatrix lastLocation = null;
		
		while(opModeIsActive())
		{
			
//			//Iterate through each trackable.
//			for(int i = 0; i < trackables.size(); i++)
//			{
//
//				//Get a trackable
//				VuforiaTrackable currentTrackable = trackables.get(i);
//				//Get the location of the trackable.
//				OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)currentTrackable.getListener()).getPose();
//
//				if(pose != null)
//				{
//					//Get a vector containing the location of the trackable.
//					VectorF location = pose.getTranslation();
//					Orientation orientation = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//
//					telemetry.addLine(currentTrackable.getName());
//
//					telemetry.addLine("Position");
//					telemetry.addData("X", location.get(0));
//					telemetry.addData("Y", location.get(1));
//					telemetry.addData("Z", location.get(2));
//
//					telemetry.addLine("Rotation");
//					telemetry.addData("X", orientation.firstAngle);
//					telemetry.addData("Y", orientation.secondAngle);
//					telemetry.addData("Z", orientation.thirdAngle);
//
//				}
//
//			}
			
			for(int i = 0; i < 2; i++) {
				
				VuforiaTrackable trackable = trackables.get(i);
				
				OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
				
//				if (robotLocationTransform != null) {
//					lastLocation = robotLocationTransform;
//				}
				
				if (robotLocationTransform != null) {
					VectorF locate = robotLocationTransform.getTranslation();
					Orientation orient = Orientation.getOrientation(robotLocationTransform, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
					
					telemetry.addLine(trackable.getName());
					telemetry.addLine("Location");
					telemetry.addData("X", locate.get(0));
					telemetry.addData("Y", locate.get(1));
					telemetry.addData("Z", locate.get(2));
					telemetry.addLine("Orientation");
					telemetry.addData("X", orient.firstAngle);
					telemetry.addData("Y", orient.secondAngle);
					telemetry.addData("Z", orient.thirdAngle);
					telemetry.addLine();
					
				}
				
			}
			
			telemetry.update();
			
		}
		
		
	}
	
	String format(OpenGLMatrix transformationMatrix) {
		return transformationMatrix.formatAsTransform();
	}
	
	/**
	 * Initializes the Vuforia computer vision.
	 */
	private void initializeVuforia()
	{
		
		
		//Tells the robot controller to show what it sees in the camera on the screen.
		cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
		//Set the license key.
		parameters.vuforiaLicenseKey = "ARS60u//////AAAAGatg5bzrIElJruCZEPDqKr8mksRb99R0GE" +
				"dJMfM4xVotZyXhiShn+ToKcAK2foRmNGNekn6uvxjmkdjbOlFvoQhDYJVBYvFF3afgz8aWcqo" +
				"+WkdT3pXqnEcrPtMd4bz/CuC65ajgco231Ca7iUjqk7tuzv5Zg5gUpAfE2FulF0GIq6sXboe5" +
				"OqrDxCLG+tA6oF24zuzFCEGZHUs8PDL3NwoA2KKbZttdoE13Kvqq9+AgBrqWeIYwefx9nkzWm" +
				"n81QXHFd68APHaKyKT1PNxWKEK9aDL5vp4LRiG17AaBaGpXedpKVN4/o6GAWJm2zCOLwOb1aS" +
				"MP8u7hDTXxsax0iMyEFYpzhshDar3HwD4xNy28";
		//Set the camera direction.
		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
		
		
		
		//Instantiate the Vuforia.
		vuforia = ClassFactory.getInstance().createVuforia(parameters);
		
		//This allows vuforia to track multiple images at once.
		Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
		
		
		//Load the trackables.
		roverRuckusTrackables = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
		
		trackables = new ArrayList<VuforiaTrackable>();
		
		//Add each trackable to the list of trackables.
		//---------------------------------------------
		VuforiaTrackable bluePerimiter = roverRuckusTrackables.get(0);
		bluePerimiter.setName("BluePerimiter");
		trackables.add(bluePerimiter);
		
		VuforiaTrackable redPerimiter = roverRuckusTrackables.get(1);
		redPerimiter.setName("RedPerimiter");
		trackables.add(redPerimiter);
		
		VuforiaTrackable frontPerimiter = roverRuckusTrackables.get(2);
		frontPerimiter.setName("FrontPerimiter");
		trackables.add(frontPerimiter);
		
		VuforiaTrackable backPerimiter = roverRuckusTrackables.get(3);
		backPerimiter.setName("BackPerimiter");
		trackables.add(backPerimiter);
		//---------------------------------------------
		
	}
	
}
