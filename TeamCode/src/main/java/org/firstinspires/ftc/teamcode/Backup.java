package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.HINT;
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
 * Created by MerrittAM on 11/9/2018.
 * Autonomous program base class.
 */

public abstract class Backup extends LinearOpMode {
	
	Robot robot;
	
	private VuforiaLocalizer vuforia;
	private VuforiaLocalizer.Parameters parameters;
	
	private int cameraMonitorViewId;
	
	private VuforiaTrackables roverRuckusTrackables;
	private ArrayList<VuforiaTrackable> trackables;
	
	final float MM_PER_INCH = 24.5f;
	final float MM_FTC_FIELD_WIDTH = (12*12-2)* MM_PER_INCH;
	
	final float TARGET_ACCURACY = 10;
	
	Vector robotLocation = new Vector();
	double robotRotation = 0;
	
	boolean hasSeenTarget = false;
	
	void initialize()
	{
		robot = new Robot(this);
		
		initializeVuforia();
		setUpVisionTargets();
		
		OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackables.get(0).getListener()).getUpdatedRobotLocation();
		
		telemetry.addLine("Location matrix got.");
		telemetry.update();
		
		//updateLocation();
		
		telemetry.addLine("Initial location updated");
		telemetry.update();
		
	}
	
	public void driveForward(double distance, LinearOpMode linearOp)
	{
		
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_TO_POSITION);
		
		robot.setTargets((int) distance * robot.INCH);
		
		while(!linearOp.isStopRequested() &&
				(robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
						robot.backRight.isBusy() && robot.backLeft.isBusy()))
		{
			
			linearOp.telemetry.addLine("Driving forward.");
			linearOp.telemetry.update();
			
			robot.runMotors(.5, -.5, -.5, .5);
			
		}
		
		robot.runMotors(0, 0, 0, 0);
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		
	}
	
	protected boolean isYellow()
	{
		
		double r = robot.colorSensor.red();
		double g = robot.colorSensor.green();
		double b = robot.colorSensor.blue();
		
		if(r > 95 && g > 95 && b < 95)
		{
			return true;
		}
		else
		{
			return false;
		}
		
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
		
		
		telemetry.addLine("Parameters set up.");
		telemetry.update();
		
		//Instantiate the Vuforia.
		vuforia = ClassFactory.getInstance().createVuforia(parameters);
		
		telemetry.addLine("Vuforia instantiated.");
		telemetry.update();
		
		//This allows vuforia to track multiple images at once.
		Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
		
		
		//Load the trackables.
		roverRuckusTrackables = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
		
		telemetry.addLine("Trackables Loaded");
		telemetry.update();
		
		trackables = new ArrayList<VuforiaTrackable>();
		
		telemetry.addLine("Trackables array initialized.");
		telemetry.update();
		
		//Add each trackable to the list of trackables.
		//---------------------------------------------
		VuforiaTrackable bluePerimiter = roverRuckusTrackables.get(0);
		bluePerimiter.setName("BluePerimiter");
		trackables.add(bluePerimiter);
		
		telemetry.addLine("Blue perimiter added.");
		telemetry.update();
		
		VuforiaTrackable redPerimiter = roverRuckusTrackables.get(1);
		redPerimiter.setName("RedPerimiter");
		trackables.add(redPerimiter);
		
		telemetry.addLine("Red perimiter added.");
		telemetry.update();
		
		VuforiaTrackable frontPerimiter = roverRuckusTrackables.get(2);
		frontPerimiter.setName("FrontPerimiter");
		trackables.add(frontPerimiter);
		
		telemetry.addLine("Front perimiter added.");
		telemetry.update();
		
		VuforiaTrackable backPerimiter = roverRuckusTrackables.get(3);
		backPerimiter.setName("BackPerimiter");
		trackables.add(backPerimiter);
		
		telemetry.addLine("Back perimiter added.");
		telemetry.update();
		
		//---------------------------------------------
		
	}
	
	/**
	 * Sets the location of the vision targets
	 * on the field and the phone on the robot.
	 */
	private void setUpVisionTargets()
	{
		
		telemetry.addLine("Setting up vision targets.");
		telemetry.update();
		
		OpenGLMatrix bluePerimiterLocationOnField = OpenGLMatrix
				.translation(0, MM_FTC_FIELD_WIDTH /2, 0)
				.multiplied(Orientation.getRotationMatrix(
						AxesReference.EXTRINSIC, AxesOrder.XZX,
						AngleUnit.DEGREES, 90, 0, 0
				));
		trackables.get(0).setLocation(bluePerimiterLocationOnField);
		telemetry.addLine("Blue perimiter location set.");
		telemetry.update();
		
		OpenGLMatrix redPerimiterLocationOnField = OpenGLMatrix
				.translation(-MM_FTC_FIELD_WIDTH /2, 0, 0)
				.multiplied(Orientation.getRotationMatrix(
						AxesReference.EXTRINSIC, AxesOrder.XZX,
						AngleUnit.DEGREES, 90, 180, 0
				));
		trackables.get(1).setLocation(redPerimiterLocationOnField);
		
		telemetry.addLine("Red perimiter location set.");
		telemetry.update();
		
		OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
				.translation(0, 0, 0)
				.multiplied(Orientation.getRotationMatrix(
						AxesReference.EXTRINSIC, AxesOrder.YZY,
						AngleUnit.DEGREES, 0, 0, 0
				));
		
		telemetry.addLine("Phone location determined.");
		telemetry.update();
		
		sleep(1000);
		
		((VuforiaTrackableDefaultListener)trackables.get(0).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
		//((VuforiaTrackableDefaultListener)trackables.get(1).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
		
		telemetry.addLine("Phone data set.");
		telemetry.update();
		
		sleep(1000);
		
	}
	
	/**
	 * Begins tracking the vision targets.
	 */
	void beginTracking()
	{
		roverRuckusTrackables.activate();
	}
	
	void testVision()
	{
		
		VuforiaTrackable trackable = trackables.get(0);
		
		OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
		
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
		else
		{
			telemetry.addLine("Vision target not located.");
		}
		
	}
	
	void updateLocation()
	{
		
		//Get the trackable.
		// TODO: 11/14/2018 Implement locating for all trackables.
		VuforiaTrackable trackable = trackables.get(0);
		
		OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
		
		if(robotLocationTransform != null) {
			
			if(!hasSeenTarget)
			{
				robot.offset = Orientation.getOrientation(robotLocationTransform, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
				
				telemetry.addLine("Gyro orientation determined");
				
				hasSeenTarget = true;
				
			}
			
			VectorF loc = robotLocationTransform.getTranslation();
			telemetry.addData("Location", loc.toString());
			robotLocation.x = loc.get(0);
			robotLocation.y = loc.get(1);
			
		}
		else
		{
			
			telemetry.addLine("Unable to locate target.");
			
			robotLocation.x = 0;
			robotLocation.y = 0;
		}
		
	}
	
	Vector getRobotLocation()
	{
		return robotLocation;
	}
	
	void moveToPosition(Vector target)
	{
		
		//Calculate the distance to the target.
		Vector distanceToTarget = Vector.sub(target, robotLocation);
		
		//Drive the robot while it is not within the target threshold.
		while(opModeIsActive() && Vector.dist(robotLocation, distanceToTarget) < TARGET_ACCURACY) {
			
			telemetry.addLine("Driving to Target.");
			telemetry.addData("Target Coordinates", target.toString());
			
			//Continuously update the position of the robot.
			updateLocation();
			
			telemetry.addData("Robot Location", robotLocation.toString());
			
			//Continuously recalculate the distance to the target.
			distanceToTarget = Vector.sub(target, robotLocation);
			
			telemetry.addData("Distance to Target", distanceToTarget.toString());
			
			//Determine the power needed to drive the robot in the correct direction.
			Vector power = distanceToTarget.copy();
			power.normalize();
			
			telemetry.addData("Power", power.toString());
			
			//Drive the robot in driver-centric mode toward the target.
			robot.driverCentricDrive(power.x, power.y, 0);
			
			telemetry.update();
			
		}
		robot.robotCentricDrive(0, 0, 0);
		
	}
	
}
