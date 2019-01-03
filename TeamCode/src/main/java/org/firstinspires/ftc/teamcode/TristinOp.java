package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by MerrittAM on 11/29/2018.
 */
@TeleOp(name = "TristinOp", group = "TeleOp")
public class TristinOp extends OpMode {

private Gyroscope imu;
private DcMotor frontRight;
private DcMotor frontLeft;
private DcMotor backRight;
private DcMotor backLeft;
private DcMotor wormDrive1;
private DcMotor wormDrive2;
private DcMotor wormDrive3;
private Servo servoTest;



    public void init()  {
 
        imu = hardwareMap.get(Gyroscope.class,"imu");
        
        frontLeft=hardwareMap.get(DcMotor.class, "fl");
        frontRight=hardwareMap.get(DcMotor.class,"fr");
        backRight=hardwareMap.get(DcMotor.class,"br");
        backLeft=hardwareMap.get(DcMotor.class,"bl");
        
        wormDrive1=hardwareMap.get(DcMotor.class,"wormDrive1");
        wormDrive2=hardwareMap.get(DcMotor.class,"wormDrive2");
        wormDrive3=hardwareMap.get(DcMotor.class,"wormDrive3");
        
        
       
        servoTest=hardwareMap.get(Servo.class,"markerServo");
 
    }

public void loop() {
    
    double leftPower=-gamepad1.left_stick_y;
    double rightPower=gamepad1.right_stick_y;
    double y=gamepad1.left_stick_y;
    double x=gamepad1.left_stick_x;
    double r=gamepad1.right_stick_x;
    
    double wormGearPower=gamepad2.right_stick_y;
    
    
    frontLeft.setPower(-y-r+x);
    frontRight.setPower(y-r+x);
    backLeft.setPower(-y-r-x);
    backRight.setPower(y-r-x);
    
            wormDrive1.setPower(wormGearPower);
            wormDrive2.setPower(wormGearPower);
            wormDrive3.setPower(wormGearPower);
            
            
    if (gamepad1.dpad_up)
            servoTest.setPosition(1);
         
       if (gamepad1.dpad_down)
             servoTest.setPosition(0);
    
}




}
