package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

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
	
	//--- Tensorflow Computer Vision ---
	private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
	private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
	private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
	
	private TFObjectDetector tfod;
	
	//----------------------------------
	
	//--- Vuforia Location Variables ---
	final float MM_PER_INCH = 24.5f;
	final float MM_FTC_FIELD_WIDTH = (12*12-2)* MM_PER_INCH;
	
	final float TARGET_ACCURACY = 70;
	
	Vector robotLocation = new Vector();
	
	boolean hasSeenTarget = false;
	
	Vector[] recentData = new Vector[10];
	
	//----------------------------------
	
	protected enum ElementPosition
	{
		LEFT,
		CENTER,
		RIGHT,
		UNKNOWN
	}
	
	public void initialize()
	{
		
		//Initialize the robot.
		robot = new Robot(this);
		
		//Retract the marker dropper.
		robot.markerDropperretract();
		
		//--- Prepare the Vuforia. ---
		initializeVuforia();
		
		//--- Prepare the TensorFlow. ---
		if(ClassFactory.getInstance().canCreateTFObjectDetector())
		{
			initializeTensorFlow();
		}
		else
		{
			telemetry.addLine("TensorFlow unable to be instantiated.");
		}
		
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
	
//	public void rotateCW(double degrees, double power)
//	{
//
//		//Get the absolute value of the inputs to
//		//make sure no one does anything funny.
//		double deg = Math.abs(degrees);
//		double pow = Math.abs(power);
//
//		//Drive the robot while the correct yaw value is not reached.
//		while(robot.gyro.getYaw() > -deg && opModeIsActive())
//		{
//			robot.runMotors(pow, pow, pow, pow);
//		}
//
//		robot.stopDrive();
//
//	}
	
	public void rotateCW(double degrees, double power)
	{
		
		//Get the absolute value of the inputs to
		//make sure no one does anything funny.
		double deg = Math.abs(degrees);
		double pow = Math.abs(power);
		
		//Drive the robot while the correct yaw value is not reached.
		while(robot.gyro.getTotalYaw() > -deg && opModeIsActive())
		{
			robot.gyro.updateYaw();
			robot.runMotors(pow, pow, pow, pow);
		}
		
		robot.stopDrive();
		
	}
	
//	public void rotateCCW(double degrees, double power)
//	{
//
//		//Get the absolute value of the inputs to
//		//make sure no one does anything funny.
//		double deg = Math.abs(degrees);
//		double pow = Math.abs(power);
//
//		//Drive the robot while the correct yaw value is not reached.
//		while(robot.gyro.getYaw() < deg && opModeIsActive())
//		{
//			robot.runMotors(-pow, -pow, -pow, -pow);
//		}
//
//		robot.stopDrive();
//
//	}
	
	public void rotateCCW(double degrees, double power)
	{
		
		//Get the absolute value of the inputs to
		//make sure no one does anything funny.
		double deg = Math.abs(degrees);
		double pow = Math.abs(power);
		
		//Drive the robot while the correct yaw value is not reached.
		while(robot.gyro.getTotalYaw() < deg && opModeIsActive())
		{
			robot.gyro.updateYaw();
			robot.runMotors(-pow, -pow, -pow, -pow);
		}
		
		robot.stopDrive();
		
	}
	
	public void lowerBot()
	{
		
		while(!isStopRequested() && robot.gyro.getOrientation().secondAngle > -85)
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
		parameters = new VuforiaLocalizer.Parameters();//(cameraMonitorViewId);
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
	
	private void initializeTensorFlow()
	{
		//Shows the camera view on the screen.
		int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		//Set the camera view.
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		
		//Minimum confidence level.
		tfodParameters.minimumConfidence = .4;
		
		//Create an instance of the tensorflow object detector.
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		//Load the models.
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
		
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
	
	protected void activateElementDetection()
	{
		if(tfod != null)
		{
			tfod.activate();
			telemetry.addLine("Element detection activated.");
			telemetry.update();
		}
		else
		{
			telemetry.addLine("!!!Warning! Element detection not activated.!!!");
			telemetry.update();
		}
	}
	
	protected ElementPosition detectElement()
	{
		
		//A timer to end the object detection if it has not returned after a certain time.
		ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
		time.reset();
		
		//The amount of time allotted for the computer vision to run its course.
		final int TIMEOUT = 5000;
		
		//While stop has not been pressed and the timer has not exceeded the limit.
		while(!isStopRequested() && time.time() < TIMEOUT) {
			
			if (tfod != null) {
				
				//Get a list of all the recognized objects.
				List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
				if (updatedRecognitions != null) {
					
					telemetry.addData("# of objects detected", updatedRecognitions.size());
					
					//If three or more objects are detected.
					if (updatedRecognitions.size() >= 3) {
						
						final int ERROR_FLAG = -1;
						
						//Initialize the positions to error flags. If the values of the vectors
						//are still equal to the error flags at the end, we know that something
						//went wrong or the correct information was not available.
						Vector lowestGoldPos = new Vector(ERROR_FLAG, ERROR_FLAG);
						Vector lowestSilverPos = new Vector(ERROR_FLAG, ERROR_FLAG);
						Vector secondLowestSilverPos = new Vector(ERROR_FLAG, ERROR_FLAG);
						
						//Loop through all the recognized objects.
						for (Recognition recognition : updatedRecognitions) {
							
							//If it is a gold element, check if it is the lowest.
							if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
								//If there is a gold object lower on the screen
								//(higher y value), set its position as the lowest so far.
								if (lowerThan(recognition, lowestGoldPos)) {
									lowestGoldPos.x = recognition.getLeft();
									lowestGoldPos.y = recognition.getTop();
								}
							} else {
								//Check if it is lower than the lowest so far.
								if (lowerThan(recognition, lowestSilverPos)) {
									//The current lowest silver is now the second lowest.
									secondLowestSilverPos = lowestSilverPos.copy();
									
									//Update the lowest position.
									lowestSilverPos.x = recognition.getLeft();
									lowestSilverPos.y = recognition.getTop();
									
								} //Check if it is higher than the lowest, but lower than the second.
								else if (lowerThan(recognition, secondLowestSilverPos)) {
									secondLowestSilverPos.x = recognition.getLeft();
									secondLowestSilverPos.y = recognition.getTop();
								}
								
							}
							
							//If the values have not changed, then there was not enough information.
							//If they have changed, then we can determine the positions.
							if (lowestGoldPos.x != ERROR_FLAG &&
									lowestSilverPos.x != ERROR_FLAG &&
									secondLowestSilverPos.x != ERROR_FLAG) {
								
								//If the x position of the gold is less than the x positions of both
								//silvers, then the gold element is on the left.
								if (lowestGoldPos.x < lowestSilverPos.x && lowestGoldPos.x < secondLowestSilverPos.x)
								{
									//Gold element is in LEFT position.
									return ElementPosition.LEFT;
								} //If the x position of the gold is greater than the x positions of
								//both silvers, then the gold element is on the right.
								else if (lowestGoldPos.x > lowestSilverPos.x && lowestGoldPos.x > secondLowestSilverPos.x)
								{
									//Gold element is in RIGHT position.
									return ElementPosition.RIGHT;
								} //If the x position of the gold is not greater than both or less
								//than both, then the gold element is in the middle.
								else
								{
									//Gold element is in CENTER position.
									return ElementPosition.CENTER;
								}
								
								
							}
							
						}
						
						
					} else {
						telemetry.addLine("Not enough objects detected.");
					}
					
				}
				
			} else {
				telemetry.addLine("!!!Warning! Problem with TensorFlow.!!!");
			}
			
			telemetry.update();
			
		}
		
		return ElementPosition.UNKNOWN;
		
	}
	
	private boolean lowerThan(Recognition recognition, Vector check)
	{
		if(recognition.getTop() > check.y)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	protected void deactivateElementDetection()
	{
		if(tfod != null)
		{
			tfod.shutdown();
			telemetry.addLine("Element detection deactivated.");
			telemetry.update();
		}
		else
		{
			telemetry.addLine("!!!Warning! Element detection not deactivated.!!!");
			telemetry.update();
		}
	}
	
}
