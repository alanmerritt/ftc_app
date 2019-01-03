package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

/**
 * Created by MerrittAM on 11/7/2018.
 * Class that contains common code for controlling
 * the robot.
 */

public class Robot {
	
	final int ENCODER_TICKS_PER_REVOLUTION = 1120;
	final int WHEEL_DIAMETER = 4; //in.
	final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; //in.
	//Proportion to convert inches to encoder ticks.
	public final int INCH = (int)(ENCODER_TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE);
	
	public DcMotor frontLeft;
	public DcMotor frontRight;
	public DcMotor backRight;
	public DcMotor backLeft;
	
	public Gyro gyro;
	
	
	public ColorSensor colorSensor;
	public DistanceSensor distanceSensor;

	public Servo markerDropper;
	public final double MARKER_DROPPER_OPEN = 1;
	public final double MARKER_DROPPER_CLOSED = .2;
	public Servo phoneMount;
	
	private OpMode opmode;
	
	
	
	public double offset;
	
	/**
	 * Initializes the robot.
	 * @param _opmode The opMode that the robot is being controlled by.
	 */
	public Robot(OpMode _opmode) {
		
		this.opmode = _opmode;
		
		frontLeft = opmode.hardwareMap.dcMotor.get("fl");
		frontRight = opmode.hardwareMap.dcMotor.get("fr");
		backRight = opmode.hardwareMap.dcMotor.get("br");
		backLeft = opmode.hardwareMap.dcMotor.get("bl");
		
		gyro = new Gyro(opmode);
		
		colorSensor = opmode.hardwareMap.get(ColorSensor.class, "color_distance_sensor");
		distanceSensor = opmode.hardwareMap.get(DistanceSensor.class, "color_distance_sensor");
		
		markerDropper = opmode.hardwareMap.servo.get("markerDropper");
		
		offset = 0;
		
		phoneMount = opmode.hardwareMap.servo.get("phoneMount");
		
		phoneMount.setPosition(.5);
		
	}
	
	/**
	 * Drives the robot in "Robot Centric" mode, where
	 * forward on the joystick causes the robot to move
	 * forward relative to itself.
	 * @param x Value for left/right movement.
	 * @param y Value for forward/back movement.
	 * @param r Value for rotational movement.
	 */
	public void robotCentricDrive(double x, double y, double r)
	{
		
		frontLeft.setPower(Range.clip(-y + x - r, -1, 1));
		backLeft.setPower(Range.clip(-y - x - r, -1, 1));
		frontRight.setPower(Range.clip(y + x - r, -1, 1));
		backRight.setPower(Range.clip(y - x - r, -1, 1));
		
	}
	
	/**
	 * Drives the robot in "Driver Centric" or "Field Centric" mode,
	 * where forward on the joystick causes the robot to move
	 * forward relative to a set orientation (usually the direction
	 * the driver is facing).
	 * @param x The value for left/right movement.
	 * @param y The value for forward/back movement.
	 * @param r The value for rotational movement.
	 */
	public void driverCentricDrive(double x, double y, double r)
	{
	
		Vector inputVector = new Vector(x, y);
		opmode.telemetry.addData("Input vector", inputVector.toString());
		Vector rotatedVector = rotateVector(inputVector, gyro.getYaw() - offset);
		opmode.telemetry.addData("RotatedVector", rotatedVector.toString());
		robotCentricDrive(rotatedVector.x, rotatedVector.y, r);
		
		opmode.telemetry.update();
	
	}
	
	/**
	 * Rotates a vector by a specified rotation value.
	 * @param v The vector to rotate.
	 * @param rotation The amount to rotate the vector.
	 * @return A new vector that has been rotated.
	 */
	private Vector rotateVector(Vector v, double rotation)
	{
		
		//Convert the vector to polar form.
		double length = Math.sqrt(v.x*v.x + v.y*v.y);
		double theta = Math.atan2(v.y, v.x);
		
		opmode.telemetry.addLine();
		opmode.telemetry.addData("Length", length);
		opmode.telemetry.addData("Theta", theta);
		
		//Add the rotation to the angle of the polar vector.
		theta += rotation * Math.PI / 180.0;
		
		opmode.telemetry.addData("Rotated Theta", theta);
		
		//Create a new vector to return.
		Vector rotated = new Vector();
		
		//Convert the polar vector back to cartesian form
		//and set rotated to the resulting vector.
		rotated.x = length*Math.cos(theta);
		rotated.y = length*Math.sin(theta);
		
		//Return the rotated vector.
		return rotated;
		
	}
	
	/**
	 * Sets the run mode of the motor.
	 * @param mode The mode of the motor.
	 */
	public void setModes(DcMotor.RunMode mode) {
		frontLeft.setMode(mode);
		frontRight.setMode(mode);
		backRight.setMode(mode);
		backLeft.setMode(mode);
	}
	
	/**
	 * Sets the encoder targets of all the motors.
	 * @param target The encoder target.
	 */
	public void setTargets(int target) {
		frontLeft.setTargetPosition(target);
		frontRight.setTargetPosition(-target);
		backRight.setTargetPosition(-target);
		backLeft.setTargetPosition(target);
	}
	
	/**
	 * Drives each motor at a set power.
	 * @param fl Power for front left motor.
	 * @param fr Power for front right motor.
	 * @param br Power for back right motor.
	 * @param bl Power for back left motor.
	 */
	public void runMotors(double fl, double fr, double br, double bl)
	{
		frontLeft.setPower(fl);
		frontRight.setPower(fr);
		backRight.setPower(br);
		backLeft.setPower(bl);
	}
	
	public double getAbsoluteRobotRotation()
	{
		return gyro.getYaw() - offset;
	}
	
}
