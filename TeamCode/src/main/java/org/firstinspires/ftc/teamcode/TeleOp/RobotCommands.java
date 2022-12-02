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


    DcMotorEx frontRight   ;
    DcMotorEx backRight    ;
    DcMotorEx backLeft     ;
    DcMotorEx frontLeft    ;
//    DcMotorEx encoder       = null;
//    DcMotorEx strafeEncoder = null;
//    DcMotorEx turnEncoder   = null;

    DcMotorEx armDrive ;

    DcMotorEx leftSlide;
    DcMotorEx rightSlide;

    //Servo clawRotate = null;
    Servo clawGrab;
    Servo clawElbow;
    Servo clawWrist;


    //Drive Constants
    int tickPerIn = 1000;

    int armOut = 500;
    int armBack = 750;


    public void init(HardwareSoftware robot){
        frontRight    = robot.frontRight();
        backRight     = robot.backRight();
        backLeft      = robot.backLeft();
        frontLeft     = robot.frontLeft();

        leftSlide = robot.leftSlide();
        rightSlide = robot.rightSlide();
//    Dcx encoder       = null;
//    Dcx strafeEncoder = null;
//    Dcx turnEncoder   = null;

        armDrive = robot.armDrive();

        leftSlide = leftSlide;
        rightSlide = rightSlide;

        clawGrab = robot.clawGrab();
        clawElbow = robot.clawElbow();
        clawWrist = robot.clawElbow();

    }
    public void Drive(int speed, int distance){
        frontLeft.setTargetPosition(distance*tickPerIn);
        frontRight.setTargetPosition(distance*tickPerIn);
        backLeft.setTargetPosition(distance*tickPerIn);
        backRight.setTargetPosition(distance*tickPerIn);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        frontLeft.setVelocity(speed);
        frontRight.setVelocity(speed);
        backLeft.setVelocity(speed);
        backRight.setVelocity(speed);


    }

    public void armOut(){
        armDrive.setTargetPosition(armOut);

        armDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);




        armDrive.setVelocity(2000);

        clawWrist.setPosition(0);
        clawElbow.setPosition(0);

    }
    public void armBack(){
        armDrive.setTargetPosition(armBack);

        armDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);



        armDrive.setVelocity(2000);

        clawWrist.setPosition(1);
        clawElbow.setPosition(0);
    }


    public void armHome(){
        armDrive.setTargetPosition(0);

        armDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

//        clawWrist.setPosition(0);
        clawElbow.setPosition(0.5);

        if(armDrive.getCurrentPosition() == 0){
            return;

        }
        else{
            leftSlide.setVelocity(2000);
            rightSlide.setVelocity(2000);
            return;
        }



    }

    public void slideChange(int height){

        leftSlide.setTargetPosition(height);
        rightSlide.setTargetPosition(height);


        leftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


    }

}
