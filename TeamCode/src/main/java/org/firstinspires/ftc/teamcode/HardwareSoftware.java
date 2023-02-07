package org.firstinspires.ftc.teamcode;



import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FusionFramework.MaxbotixUltrasonicI2c;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class HardwareSoftware {

    HardwareMap hw = null;
    DcMotorEx frontRight    = null;
    DcMotorEx backRight     = null;
    DcMotorEx backLeft      = null;
    DcMotorEx frontLeft     = null;
//    DcMotorEx encoder       = null;
//    DcMotorEx strafeEncoder = null;
//    DcMotorEx turnEncoder   = null;

    DcMotorEx armDrive = null;

    BNO055IMU imu = null;

    DcMotorEx leftSlide = null;
    DcMotorEx rightSlide = null;

    //Servo clawRotate = null;
    Servo clawGrab = null;
    Servo clawElbow = null;
    Servo clawWrist = null;

    Servo bumper1 = null;
    Servo bumper2 = null;

    //Gyro variable
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    OpenCvCamera frontWebCam;

    MaxbotixUltrasonicI2c   right_ultrasonic = null;
    MaxbotixUltrasonicI2c   left_ultrasonic = null;


    int tickPerIn = 1000;
    int armMax = 1000;
    int armMin = 10;


    int low = 500;
    int mid = 1000;
    int high = 1500;



    public void init(HardwareMap ahw){
        hw = ahw;

//        imu = hw.get(BNO055IMU.class, "imu");


        frontRight = hw.get(DcMotorEx.class, "frontRight");
        backRight = hw.get(DcMotorEx.class, "backRight");
        backLeft = hw.get(DcMotorEx.class, "backLeft");
        frontLeft = hw.get(DcMotorEx.class, "frontLeft");
//        encoder = hw.get(DcMotorEx.class, "encoder");
//        strafeEncoder = hw.get(DcMotorEx.class, "strafeEncoder");
//        turnEncoder = hw.get(DcMotorEx.class, "turnEncoder");

        modernRoboticsI2cGyro = hw.get(ModernRoboticsI2cGyro.class, "gyro");


        armDrive = hw.get(DcMotorEx.class, "armDrive");

        leftSlide = hw.get(DcMotorEx.class, "leftSlide");
        rightSlide = hw.get(DcMotorEx.class, "rightSlide");

        leftSlide.setDirection(DcMotorEx.Direction.REVERSE);
        rightSlide.setDirection(DcMotorEx.Direction.REVERSE);
        armDrive.setDirection(DcMotorEx.Direction.FORWARD);


        clawGrab = hw.get(Servo.class, "claw");
        clawWrist = hw.get(Servo.class, "clawWrist");
        clawElbow = hw.get(Servo.class, "clawElbow");

        bumper1 = hw.get(Servo.class, "bumper1");
        bumper2 = hw.get(Servo.class, "bumper2");

        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        encoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        strafeEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        turnEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);

        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

//        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
//        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
//        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRight.setDirection(DcMotorSimple.Direction.FORWARD);




        armDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armDrive.setTargetPositionTolerance(5);

        leftSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftSlide.setTargetPositionTolerance(5);
        rightSlide.setTargetPositionTolerance(5);




        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
//        encoder.setPower(0);
//        strafeEncoder.setPower(0);
//        turnEncoder.setPower(0);

        armDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        armDrive.setPower(0);

        armDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
        frontWebCam = OpenCvCameraFactory.getInstance().createWebcam(hw.get(WebcamName.class, "frontWebcam"), cameraMonitorViewId);

        right_ultrasonic = hw.get(MaxbotixUltrasonicI2c.class, "right_ultrasonic");

        left_ultrasonic = hw.get(MaxbotixUltrasonicI2c.class, "left_ultrasonic");



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

    public OpenCvCamera getFrontWebCam() { return frontWebCam; }
//
//    public DcMotorEx encoder() {return encoder;}
//
//    public DcMotorEx strafeEncoder() {return strafeEncoder;}
//
//    public DcMotorEx turnEncoder() {return turnEncoder;}

    public DcMotorEx armDrive(){ return armDrive;}


    public DcMotorEx leftSlide(){ return leftSlide;}

    public DcMotorEx rightSlide(){return rightSlide;}

    //public Servo clawRotate(){return clawRotate;}

    public Servo clawGrab(){return clawGrab;}
    public Servo clawWrist(){return clawWrist;}
    public Servo clawElbow(){return clawElbow;}

    public Servo bumper1(){return bumper1;}
    public Servo bumper2(){return bumper2;}

    public ModernRoboticsI2cGyro gyro(){ return modernRoboticsI2cGyro;}

    public BNO055IMU getImu(){
        return imu;
    }

    public MaxbotixUltrasonicI2c get_right_ultrasonic(){ return right_ultrasonic; }
    public MaxbotixUltrasonicI2c get_left_ultrasonic(){ return left_ultrasonic; }

}
