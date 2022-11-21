package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

public class RobotCommands {
    HardwareSoftware robot = new HardwareSoftware();
    public boolean home = true;
    public boolean out = false;
    public boolean back = false;


    //Drive Constants
    int tickPerIn = 1000;

    int armOut = 1000;
    int armBack = 1500;


    public void Drive(int speed, int distance){
        robot.frontLeft().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft().setTargetPosition(distance*tickPerIn);
        robot.frontRight().setTargetPosition(distance*tickPerIn);
        robot.backLeft().setTargetPosition(distance*tickPerIn);
        robot.backRight().setTargetPosition(distance*tickPerIn);

        robot.frontLeft().setVelocity(speed);
        robot.frontRight().setVelocity(speed);
        robot.backLeft().setVelocity(speed);
        robot.backRight().setVelocity(speed);


    }

    public void armOut(){
        robot.armDrive().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armDrive().setTargetPosition(armOut);
        out = true;

        robot.armDrive().setVelocity(2000);
    }
    public void armBack(){
        robot.armDrive().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armDrive().setTargetPosition(armBack);


        robot.armDrive().setVelocity(2000);
    }


    public void armHome(){
        robot.armDrive().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.armDrive().setTargetPosition(10);
        home = true;
    }
}
