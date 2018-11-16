package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by MerrittAM on 9/30/2017.
 * Class used for easily accessing the imu
 * builtin to the Rev Expansion Hub.
 *
 * It also provides a method that returns the total
 * rotation of the robot, which makes it easy to
 * rotate the robot past 180 or -180.
 *
 * It assumes that the imu is named "imu" in
 * the robot configuration file.
 */
public class Gyro {
	
	BNO055IMU imu;
	
	private double totalYaw = 0;
	private double lastYaw = 0;
	
	/**
	 * Initializes the gyroscope.
	 * @param opMode The opmode, needed for hardware mapping.
	 */
	public Gyro(OpMode opMode) {
		
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json";
		parameters.loggingEnabled = true;
		parameters.loggingTag = "IMU";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
		
		imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters);
		
	}
	
	/**
	 * Gets the rotational orientation of the robot.
	 * @return The rotation of the robot. Angles are in ZYX order.
	 */
	public Orientation getOrientation() {
		
		return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		
	}
	
	/**
	 * Gets the yaw value straight from the gyroscope.
	 * @return The yaw value.
	 */
	public double getYaw() {
		
		return getOrientation().firstAngle;
		
	}
	
	/**
	 * This updates the yaw value for the total rotation.
	 * It <i><b>MUST</b></i> run whenever the robot is turning so
	 * that the value can update properly.
	 */
	public void updateYaw() {
		
		//Get the yaw directly from the gyroscope.
		double yaw = getYaw();
		
		//Get the difference between the current yaw value and the previous one.
		double diff = yaw - lastYaw;
		
		//If the difference is greater than 180, it means that it has jumped from negative
		//to positive, so subtract 360 to find the actual difference.
		if(diff >= 180) {
			diff -= 360;
		}
		//Likewise, if the difference is less than -180, it means that it has jumped from positive
		//to negative, so add 360 to find the actual difference.
		if(diff < -180) {
			diff += 360;
		}
		
		//Add the difference to the total yaw.
		totalYaw += diff;
		
		//Set the last yaw to the current yaw.
		lastYaw = yaw;
		
	}
	
	/**
	 * Gets the total yaw. While the gyroscope returns a value from -180 to 180,
	 * this will return the total amount that the robot has rotated.
	 * @return The total yaw value.
	 */
	public double getTotalYaw() {
		
		return totalYaw;
		
	}
	
}