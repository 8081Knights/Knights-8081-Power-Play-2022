package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

public class RobotCommands {
    HardwareSoftware robot = new HardwareSoftware();
    public boolean home = true;
    public boolean out = false;
    public boolean back = false;


    DcMotorEx frontRight    = null;
    DcMotorEx backRight     = null;
    DcMotorEx backLeft      = null;
    DcMotorEx frontLeft     = null;
//    DcMotorEx encoder       = null;
//    DcMotorEx strafeEncoder = null;
//    DcMotorEx turnEncoder   = null;

    DcMotorEx armDrive = null;

    DcMotorEx leftSlide = null;
    DcMotorEx rightSlide = null;

    //Servo clawRotate = null;
    Servo claw = null;
    Servo clawElbow = null;
    Servo clawWrist = null;


    //Drive Constants
    int tickPerIn = 1000;

    int armOut = 1000;
    int armBack = 1500;


    public void init(HardwareSoftware robot){
        frontRight    = robot.frontRight();
        backRight     = robot.backRight();
        backLeft      = robot.backLeft();
        frontLeft     = robot.frontLeft();
//    Dcx encoder       = null;
//    Dcx strafeEncoder = null;
//    Dcx turnEncoder   = null;

        armDrive = robot.armDrive();

        leftSlide = leftSlide;
        rightSlide = rightSlide;

        claw = robot.claw();
        clawElbow = robot.clawElbow();
        clawWrist = robot.clawElbow();

    }
    public void Drive(int speed, int distance){
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(distance*tickPerIn);
        frontRight.setTargetPosition(distance*tickPerIn);
        backLeft.setTargetPosition(distance*tickPerIn);
        backRight.setTargetPosition(distance*tickPerIn);

        frontLeft.setVelocity(speed);
        frontRight.setVelocity(speed);
        backLeft.setVelocity(speed);
        backRight.setVelocity(speed);


    }

    public void armOut(){
        armDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armDrive.setTargetPosition(armOut);
        out = true;

        armDrive.setVelocity(2000);
    }
    public void armBack(){
        armDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armDrive.setTargetPosition(armBack);


        armDrive.setVelocity(2000);
    }


    public void armHome(){
        armDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armDrive.setTargetPosition(10);

    }

    public void slideChange(int height){
        leftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftSlide.setTargetPosition(height);
        rightSlide.setTargetPosition(height);

        leftSlide.setVelocity(2000);
        rightSlide.setVelocity(2000);
    }
}
