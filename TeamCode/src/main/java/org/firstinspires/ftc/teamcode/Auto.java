package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

public abstract class Auto extends LinearOpMode {
	
	protected Robot robot;
	
	//--- Vuforia Computer Vision ---
	
	private VuforiaLocalizer vuforia;
	private VuforiaLocalizer.Parameters parameters;
	
	private int cameraMonitorViewId;
	
	private VuforiaTrackables roverRuckusTrackables;
	private ArrayList<VuforiaTrackable> trackables;
	
	//-------------------------------
	
	//--- Vuforia Location Variables ---
	
	final float MM_PER_INCH = 24.5f;
	final float MM_FTC_FIELD_WIDTH = (12*12-2)* MM_PER_INCH;
	
	final float TARGET_ACCURACY = 70;
	
	Vector robotLocation = new Vector();
	
	boolean hasSeenTarget = false;
	
	Vector[] recentData = new Vector[10];
	
	//----------------------------------
	
	public void initialize()
	{
		
		//Initialize the robot.
		robot = new Robot(this);
		
		//Retract the marker dropper.
		robot.markerDropperretract();
		
		//--- Prepare the Vuforia. ---
		initializeVuforia();
		
		OpenGLMatrix bluePerimiterLocationOnField = OpenGLMatrix
				.translation(0, MM_FTC_FIELD_WIDTH/2, 0)
				.multiplied(Orientation.getRotationMatrix(
						AxesReference.EXTRINSIC, AxesOrder.XZX,
						AngleUnit.DEGREES, 90, 0, 0
				));
		trackables.get(0).setLocation(bluePerimiterLocationOnField);
		
		OpenGLMatrix redPerimiterLocationOnField = OpenGLMatrix
				.translation(-MM_FTC_FIELD_WIDTH/2, 0, 0)
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
		
		for(int i = 0; i < recentData.length; i++)
		{
			recentData[i] = null;
		}
		
		//----------------------------
		
		telemetry.addLine("Ready.");
		telemetry.update();
		
	}
	
	@Deprecated
	public void driveForward(double distance, LinearOpMode linearOp)
	{
		//Reset the motor encoders.
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_TO_POSITION);
		
		//Set the targets.
		robot.setTargets((int) distance * robot.INCH);
		
		//While none of the robots have reached the target.
		while(!linearOp.isStopRequested() &&
				(robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
						robot.backRight.isBusy() && robot.backLeft.isBusy()))
		{
			
			linearOp.telemetry.addLine("Driving forward.");
			linearOp.telemetry.addData("Distance", robot.frontLeft.getCurrentPosition());
			linearOp.telemetry.update();
			
			//Drive the robot forward.
			robot.runMotors(.9, -.9, -.9, .9);
			
		}
		
		//Stop the motors and reset the encoders.
		robot.runMotors(0, 0, 0, 0);
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		
	}
	
	/**
	 * Drives the robot forward.
	 * @param distance The distance to travel.
	 * @param power The motor power.
	 * @param linearOp The OpMode that is running the robot (use this).
	 */
	public void driveForward(double distance, double power, LinearOpMode linearOp)
	{
		
		//Reset the motor encoders.
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_TO_POSITION);
		
		//Set the targets.
		robot.setTargets((int) distance * robot.INCH);
		
		//While none of the robots have reached the target.
		while(!linearOp.isStopRequested() &&
				(robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
						robot.backRight.isBusy() && robot.backLeft.isBusy()))
		{
			
			linearOp.telemetry.addLine("Driving forward.");
			linearOp.telemetry.addData("Distance", robot.frontLeft.getCurrentPosition());
			linearOp.telemetry.update();
			
			//Drive the robot forward.
			robot.runMotors(power, -power, -power, power);
			
		}
		
		//Stop the motors and reset the encoders.
		robot.runMotors(0, 0, 0, 0);
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		
	}
	
	
	public void driveForwardUntilConditionMet(double distance, double power, LinearOpMode linearOp, boolean condition)
	{
		
		//Reset the motor encoders.
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_TO_POSITION);
		
		//Set the targets.
		robot.setTargets((int) distance * robot.INCH);
		
		//While none of the robots have reached the target.
		while(!linearOp.isStopRequested() &&
				(robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
						robot.backRight.isBusy() && robot.backLeft.isBusy()))
		{
			
			linearOp.telemetry.addLine("Driving forward.");
			linearOp.telemetry.addData("Distance", robot.frontLeft.getCurrentPosition());
			linearOp.telemetry.update();
			
			//Drive the robot forward.
			robot.runMotors(power, -power, -power, power);
			
		}
		
		//Stop the motors and reset the encoders.
		robot.runMotors(0, 0, 0, 0);
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		
	}
	
	/**
	 * Drives the robot backward.
	 * @param distance The distance to travel.
	 * @param power The motor power.
	 * @param linearOp The OpMode that is running the robot (use this).
	 */
	public void driveBackward(double distance, double power, LinearOpMode linearOp)
	{
		
		//Reset the motor encoders.
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_TO_POSITION);
		
		//Set the targets.
		robot.setTargets((int) -distance * robot.INCH);
		
		//While none of the robots have reached the target.
		while(!linearOp.isStopRequested() &&
				(robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
						robot.backRight.isBusy() && robot.backLeft.isBusy()))
		{
			
			linearOp.telemetry.addLine("Driving backward.");
			linearOp.telemetry.addData("Distance", robot.frontLeft.getCurrentPosition());
			linearOp.telemetry.update();
			
			//Drive the robot backward.
			robot.runMotors(-power, power, power, -power);
			
		}
		
		
		
		//Stop the motors and reset the encoders.
		robot.runMotors(0, 0, 0, 0);
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		
	}
	
	/**
	 * Drives the robot left.
	 * @param distance The distance to travel.
	 * @param power The motor power.
	 * @param linearOp The OpMode that is running the robot (use this).
	 */
	public void driveLeft(double distance, double power, LinearOpMode linearOp)
	{
		
		//Reset the motor encoders.
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_TO_POSITION);
		
		//Set the targets.
		int target = (int)(distance*robot.INCH);
		robot.setTargets(-target, target, -target, target);
		
		//While none of the robots have reached the target.
		while(!linearOp.isStopRequested() &&
				(robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
						robot.backRight.isBusy() && robot.backLeft.isBusy()))
		{
			
			linearOp.telemetry.addLine("Driving left.");
			linearOp.telemetry.addData("Distance", robot.frontLeft.getCurrentPosition());
			linearOp.telemetry.update();
			
			//Drive the robot backward.
			robot.runMotors(power, power, power, power);
			
		}
		
		
		
		//Stop the motors and reset the encoders.
		robot.runMotors(0, 0, 0, 0);
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		
	}
	
	/**
	 * Drives the robot right.
	 * @param distance The distance to travel.
	 * @param power The motor power.
	 * @param linearOp The OpMode that is running the robot (use this).
	 */
	public void driveRight(double distance, double power, LinearOpMode linearOp)
	{
		
		//Reset the motor encoders.
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_TO_POSITION);
		
		//Set the targets.
		int target = (int)(distance*robot.INCH);
		robot.setTargets(target, -target, target, -target);
		
		//While none of the robots have reached the target.
		while(!linearOp.isStopRequested() &&
				(robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
						robot.backRight.isBusy() && robot.backLeft.isBusy()))
		{
			
			linearOp.telemetry.addLine("Driving right.");
			linearOp.telemetry.addData("Distance", robot.frontLeft.getCurrentPosition());
			linearOp.telemetry.update();
			
			//Drive the robot backward.
			robot.runMotors(-power, -power, power, power);
			
		}
		
		
		
		//Stop the motors and reset the encoders.
		robot.runMotors(0, 0, 0, 0);
		robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		
	}
	
	public void rotateCCW(double degrees, double power)
	{
		
		//Get the absolute value of the inputs to
		//make sure no one does anything funny.
		double deg = Math.abs(degrees);
		double pow = Math.abs(power);
		
		//Drive the robot while the correct yaw value is not reached.
		while(robot.gyro.getYaw() > -deg && opModeIsActive())
		{
			robot.runMotors(pow, pow, pow, pow);
		}
		
		robot.stopDrive();
		
	}
	
	public void rotateCW(double degrees, double power)
	{
		
		//Get the absolute value of the inputs to
		//make sure no one does anything funny.
		double deg = Math.abs(degrees);
		double pow = Math.abs(power);
		
		//Drive the robot while the correct yaw value is not reached.
		while(robot.gyro.getYaw() < deg && opModeIsActive())
		{
			robot.runMotors(-pow, -pow, -pow, -pow);
		}
		
		robot.stopDrive();
		
	}
	
	public void lowerBot()
	{
		
		while(!isStopRequested() && robot.gyro.getOrientation().secondAngle < 0)
		{
			robot.runArm(-.5);
			telemetry.addData("Angle", robot.gyro.getOrientation().secondAngle);
			telemetry.update();
		}
		
//		robot.runArm(-.5);
//		sleep(1250);
		
		robot.stopArm();
		
	}
	
	public boolean isYellow()
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
	 * Begins tracking the vision targets.
	 */
	public void beginTracking()
	{
		roverRuckusTrackables.activate();
	}
	
	/*
	void moveToPosition(Vector target)
	{
		
		//Calculate the distance to the target.
		Vector distanceToTarget = Vector.sub(target, robotLocation);
		
		telemetry.addData("Distance to target", Vector.dist(robotLocation, distanceToTarget));
		telemetry.update();
		
		sleep(1000);
		
		//Drive the robot while it is not within the target threshold.
		while(opModeIsActive() && Vector.dist(robotLocation, target) > TARGET_ACCURACY) {
			
			telemetry.addLine("Driving to Target.");
			
			telemetry.addData("Distance to target (linear)", Vector.dist(robotLocation, target));
			
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
			power.div(3);
			
			telemetry.addData("Power", power.toString());
			
			//Drive the robot in driver-centric mode toward the target.
			robot.driverCentricDrive(power.x, power.y, 0);
			
			telemetry.update();
			
		}
		
		telemetry.addLine("Target Reached.");
		telemetry.update();
		robot.robotCentricDrive(0, 0, 0);
		
	}
	*/
	
	void moveToPosition(Vector target)
	{
	
		updateLocation();
		double linearDistanceToTarget = Vector.dist(target, getRobotLocation());
		while(opModeIsActive() && linearDistanceToTarget > TARGET_ACCURACY)
		{
			
			updateLocation();
			
			telemetry.addData("Target", target.toString());
			telemetry.addData("Location", getRobotLocation().toString());
			linearDistanceToTarget = Vector.dist(target, getRobotLocation());
			telemetry.addData("Linear Distance to Target", linearDistanceToTarget);
			Vector direction = Vector.sub(target, getRobotLocation());
			telemetry.addData("Direction", direction.toString());
			direction.normalize();
			if(linearDistanceToTarget > 100) {
				direction.div(3);
			}
			else
			{
				direction.div(5);
			}
			telemetry.addData("Direction (scaled)", direction.toString());
			
			telemetry.addLine();
			
			telemetry.addLine();
			telemetry.addData("Calculated Direction", direction.toString());
			telemetry.addData("Driver Direction", new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y).toString());
			
			//if(gamepad1.a) {
				robot.driverCentricDrive(-direction.x, direction.y, 0);
			//}
			//else
			//{
				//robot.driverCentricDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, 0);
			//}
			
			telemetry.addLine();
			
			//telemetry.addData("Robot Offset", robot.offset);
			//telemetry.addData("Target Seen", hasSeenTarget);
			
		}
		robot.driverCentricDrive(0, 0, 0);
	
	}
	
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
	
	void updateLocation() {
		
		VuforiaTrackable trackable = trackables.get(0);
		
		OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
		
		if (robotLocationTransform != null) {
			
			VectorF locate = robotLocationTransform.getTranslation();
			
			if(!hasSeenTarget) {
				Orientation orient = Orientation.getOrientation(robotLocationTransform, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
				robot.offset = -90 + orient.thirdAngle;
				hasSeenTarget = true;
				telemetry.addData("Offset", robot.offset);
				telemetry.addLine("Target detected.");
				telemetry.update();
			}
			
			robotLocation.x = locate.get(0);
			robotLocation.y = locate.get(1);
			
		}
		else
		{
			telemetry.addLine("Target not detected.");
			telemetry.update();
		}
		
	}
	
	public Vector getRobotLocation() {
		return robotLocation;
	}
	
	/**
	 * Dampens irregularities in the location data
	 * by averaging the location of previous location
	 * reads and throwing out irregular location values.
	 * @return A "smoother" location value.
	 */
	private Vector dampenLocation()
	{
		
		//Shift the data through the array.
		for(int i = recentData.length-1; i >= 1; i++)
		{
			recentData[i] = recentData[i-1];
		}
		recentData[0] = robotLocation.copy();
		
		//Loop through the data and calculate the average.
		Vector average = new Vector();
		int amount = 0;
		for(int i = 0; i < recentData.length; i++)
		{
			if(recentData[i] != null)
			{
				average.add(recentData[i]);
				amount++;
			}
		}
		
		if(amount != 0)
		{
			average.div(amount);
			return average;
		}
		else
		{
			return new Vector(0, 0);
		}
	
	}
	
	/**
	 * Drives the robot to a certain location on the field.
	 * @param target The location to drive the robot to.
	 */
	void moveToPosition(Vector target, double rotationTarget)
	{
		
		//Update the location of the robot before starting.
		updateLocation();
		//Calculate the linear distance to the target.
		double linearDistanceToTarget = Vector.dist(target, getRobotLocation());
		//Travel until within range of the target.
		while(opModeIsActive() && linearDistanceToTarget > TARGET_ACCURACY)
		{
			
			//Continuously update the position of the robot.
			updateLocation();
			
			telemetry.addData("Target", target.toString());
			telemetry.addData("Location", getRobotLocation().toString());
			
			//Continuously update the linear distance to the target.
			linearDistanceToTarget = Vector.dist(target, getRobotLocation());
			
			telemetry.addData("Linear Distance to Target", linearDistanceToTarget);
			
			//Calculate the vector between the target and the location.
			Vector direction = Vector.sub(target, getRobotLocation());
			
			telemetry.addData("Direction", direction.toString());
			
			//Scale down the vector to a magnitude usable by the robot.
			direction.normalize();
			direction.div(3);
			
			telemetry.addData("Direction (scaled)", direction.toString());
			
			double currentRotation = robot.getAbsoluteRobotRotation();
			double ROTATION_TOLERANCE = 5;
			
			// TODO: 11/16/2018 May need to switch rotation direction signs.
			//Drive the robot using field-centric mode.
			robot.driverCentricDrive(-direction.x, direction.y, 0);
					//(currentRotation > rotationTarget+ROTATION_TOLERANCE ? -.1 :
					//		(currentRotation < rotationTarget-ROTATION_TOLERANCE ? .1 : 0)));
			
			telemetry.addLine();
			
		}
		//Stop the robot.
		robot.driverCentricDrive(0, 0, 0);
		
	}
	
	protected void runVision()
	{
		
		for(int i = 0; i < 2; i++) {
			
			VuforiaTrackable trackable = trackables.get(i);
			
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
			
		}
	
	}
	
	
	
}
