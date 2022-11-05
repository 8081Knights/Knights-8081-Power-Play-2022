package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareSoftware {

    HardwareMap hw = null;
    DcMotorEx frontRight    = null;
    DcMotorEx backRight     = null;
    DcMotorEx backLeft      = null;
    DcMotorEx frontLeft     = null;
    DcMotorEx encoder       = null;
    DcMotorEx strafeEncoder = null;
    DcMotorEx turnEncoder   = null;

    DcMotorEx armDrive = null;

    DcMotorEx leftSlide = null;
    DcMotorEx rightSlide = null;

    //Servo clawRotate = null;
    Servo claw = null;

    int tickPerIn = 1000;
    int armMax = 1000;
    int armMin = 10;

    int low = 500;
    int mid = 1000;
    int high = 1500;



    public void init(HardwareMap ahw){
        hw = ahw;

        frontRight = hw.get(DcMotorEx.class, "frontRight");
        backRight = hw.get(DcMotorEx.class, "backRight");
        backLeft = hw.get(DcMotorEx.class, "backLeft");
        frontLeft = hw.get(DcMotorEx.class, "frontLeft");
        encoder = hw.get(DcMotorEx.class, "encoder");
        strafeEncoder = hw.get(DcMotorEx.class, "strafeEncoder");
        turnEncoder = hw.get(DcMotorEx.class, "turnEncoder");

        armDrive = hw.get(DcMotorEx.class, "armDrive");

        leftSlide = hw.get(DcMotorEx.class, "leftSlide");
        rightSlide = hw.get(DcMotorEx.class, "rightSlide");

        leftSlide.setDirection(DcMotorEx.Direction.REVERSE);
        rightSlide.setDirection(DcMotorEx.Direction.REVERSE);

        //clawRotate = hw.get(Servo.class, "clawRotate");
        claw = hw.get(Servo.class, "claw");

        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
//        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
//        backLeft.setDirection(DcMotorEx.Direction.FORWARD);



        armDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        encoder.setPower(0);
        strafeEncoder.setPower(0);
        turnEncoder.setPower(0);


        armDrive.setPower(0);

        armDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);






    }

    public void Drive(int velocity, int distance, int timeOut){

        frontRight().setTargetPosition(distance*tickPerIn);
        frontLeft().setTargetPosition(distance*tickPerIn);
        backRight().setTargetPosition(distance*tickPerIn);
        backLeft().setTargetPosition(distance*tickPerIn);

        sleep(300);

        frontRight().setVelocity(velocity);
        frontLeft().setVelocity(velocity);
        backRight().setVelocity(velocity);
        backLeft().setVelocity(velocity);

        sleep(timeOut);

        frontRight().setVelocity(0);
        frontLeft().setVelocity(0);
        backRight().setVelocity(0);
        backLeft().setVelocity(0);



    }

    public void armUp(int speed){
        armDrive().setTargetPosition(armMax);

        sleep(300);

        armDrive().setVelocity(speed);

        sleep(1000);

        armDrive().setVelocity(0);

    }
    public void armDown(int speed){
        armDrive().setTargetPosition(armMin);

        sleep(300);

        armDrive().setVelocity(speed);

        sleep(1000);

        armDrive().setVelocity(0);

    }

    public void lowGoal(int speed){
        leftSlide().setTargetPosition(low);
        rightSlide().setTargetPosition(low);

        sleep(300);

        leftSlide().setVelocity(speed);
        rightSlide().setVelocity(speed);

        sleep(1000);

    }
    public void midGoal(int speed){
        leftSlide().setTargetPosition(mid);
        rightSlide().setTargetPosition(mid);

        sleep(300);

        leftSlide().setVelocity(speed);
        rightSlide().setVelocity(speed);

        sleep(1000);

    }
    public void highGoal(int speed){
        leftSlide().setTargetPosition(high);
        rightSlide().setTargetPosition(high);

        sleep(300);

        leftSlide().setVelocity(speed);
        rightSlide().setVelocity(speed);

        sleep(1000);

    }
//
//    public void clawFlip(){
//        if(clawRotate().getPosition() == 0){
//            clawRotate().setPosition(1);
//        }
//        else if(clawRotate().getPosition() == 1){
//            clawRotate().setPosition(0);
//        }
//    }

    public void clawChange(){
        if(claw().getPosition() == 0){
            claw.setPosition(1);
        }
        else if(claw.getPosition() == 1){
            claw.setPosition(0);
        }
    }


    public DcMotorEx frontRight(){
        return frontRight;
    }

    public DcMotorEx backRight(){
        return backRight;
    }

    public DcMotorEx frontLeft(){
        return frontLeft;
    }

    public DcMotorEx backLeft(){
        return backLeft;
    }

    public DcMotorEx encoder() {return encoder;}

    public DcMotorEx strafeEncoder() {return strafeEncoder;}

    public DcMotorEx turnEncoder() {return turnEncoder;}

    public DcMotorEx armDrive(){ return armDrive;}

    public DcMotorEx leftSlide(){ return leftSlide;}

    public DcMotorEx rightSlide(){return rightSlide;}

    //public Servo clawRotate(){return clawRotate;}

    public Servo claw(){return claw;}

}
