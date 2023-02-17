package org.firstinspires.ftc.teamcode.TeleOp;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareSoftware;

public class RobotCommands {
    HardwareSoftware robot = new HardwareSoftware();
    ModernRoboticsI2cGyro gyro;

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

    int armOut = 710;
    int armMid = 250;
    int armBack = 1625;
// EA i moved arm out a little bit higher than previous(650)

    public void init(HardwareSoftware robot){
        gyro = robot.gyro();
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
        sleep(100);
        armDrive.setTargetPosition(armOut);

        armDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);




        armDrive.setVelocity(2000);

//        clawWrist.setPosition(0);
//        clawElbow.setPosition(0);

    }

    public void armMid(){
        sleep(100);
        armDrive.setTargetPosition(armMid);

        armDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);




        armDrive.setVelocity(2000);
//
//        clawWrist.setPosition(0);
//        clawElbow.setPosition(0);

    }
    public void armBack(){
        sleep(100);
        armDrive.setTargetPosition(armBack);

        armDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);



        armDrive.setVelocity(2000);

//        clawWrist.setPosition(1);
//        clawElbow.setPosition(0);
    }


    public void armHome(){
        armDrive.setTargetPosition(0);

        armDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

//        clawWrist.setPosition(0);
//        clawElbow.setPosition(0.5);

        if(armDrive.getCurrentPosition() == 0){
            return;

        }
        else{
            armDrive.setVelocity(1000);
            return;
        }



    }

    //Method to handle and truncate linear slide movement
    public void slideChange(int height, int speed, int tolerance){


        //Set target height for linear slides
        leftSlide.setTargetPosition(height);
        rightSlide.setTargetPosition(height);


        //Tell the linear slides to move to the correct position
        leftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if((leftSlide.getCurrentPosition() >= height - tolerance || leftSlide.getCurrentPosition() <= height + tolerance) && (rightSlide.getCurrentPosition() >= height - tolerance || rightSlide.getCurrentPosition() <= height + tolerance)){
            leftSlide.setVelocity(0);
            rightSlide.setVelocity(0);


        }
        else{

            leftSlide.setVelocity(speed);
            rightSlide.setVelocity(speed);

        }



    }

    public void turnGyro(double angle, double speed){
            double angleError = 10;
            double currentAngle = gyro.getHeading();
            boolean left = false;
            double target = currentAngle + angle;

            if(target >= 360){
                target -= 360;
            }

            if(angle < 0){
                left = true;
                if (target < 0){
                    target += 360;
                }
            }



            while ((currentAngle > (target + angleError) || currentAngle < (target - angleError))) {

                frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                if(left){
                    frontLeft.setPower(-speed);
                    backLeft.setPower(speed);
                    frontRight.setPower(speed);
                    backRight.setPower(-speed);

                }

                else{
                    frontLeft.setPower(speed);
                    backLeft.setPower(-speed);
                    frontRight.setPower(-speed);
                    backRight.setPower(speed);

                }

                currentAngle = gyro.getHeading();
                // telemetry.addData("heading", "%3d deg", currentAngle);
                //
                // telemetry.update();

            }


            frontLeft.setPower(0);
            backRight.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);


    }

    public void initGyro(){
        gyro.calibrate();
    }

}
