package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name="scrimmage teleop")
public class scrimmageTeleop extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();

    int maxSpeed = 2000;
    boolean armUp = false;

    float v1 = 0;
    float v2 = 0;
    float v3 = 0;
    float v4 = 0;

    double drive;
    double strafe;
    double twist;

    int linearSpeed = 1500;

    int maxArmHeight = 800;

    boolean l = false;
    boolean m = false;
    boolean h = false;
    boolean min = true;


    int low = 500;
    int mid = 1000;
    int high = 1500;

    @Override
    public void init() {

        robot.init(hardwareMap);


//        robot.leftSlide().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightSlide().setMode(DcMotor.RunMode.RUN_TO_POSITION);



    }

    @Override
    public void loop() {
//        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
//        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//        double rightX = gamepad1.right_stick_x;
//        final double v1 = r * Math.cos(robotAngle) + rightX;
//        final double v2 = r * Math.sin(robotAngle) - rightX;
//        final double v3 = r * Math.sin(robotAngle) + rightX;
//        final double v4 = r * Math.cos(robotAngle) - rightX;

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;



        double speeds[] = {
                (y + x + rx),
                (y - x + rx),
                (y - x - rx),
                (y + x - rx)
        };

        if(speeds[0] + speeds[1] + speeds[2] + speeds[3] >= 0.2) {
            robot.frontRight().setPower(speeds[0] * 0.8);
            robot.frontLeft().setPower(speeds[1] * 0.8);
            robot.backRight().setPower(speeds[2] * 0.8);
            robot.backLeft().setPower(speeds[3] * 0.8);
        }
        else if(speeds[0] + speeds[1] + speeds[2] + speeds[3] < 0.2){
            robot.frontRight().setPower(0);
            robot.frontLeft().setPower(0);
            robot.backRight().setPower(0);
            robot.backLeft().setPower(0);
        }


//        if(gamepad1.right_stick_y > 0.1 || gamepad1.right_stick_y < -0.1) {
//            v1 = gamepad1.right_stick_y;
//            v2 = gamepad1.right_stick_y;
//        }
//        if(gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1) {
//            v3 = gamepad1.left_stick_y;
//            v4 = gamepad1.left_stick_y;
//        }

//
//        robot.frontRight().setPower(v1);
//        robot.frontLeft( ).setPower(v3);
//        robot.backRight( ).setPower(v2);
//        robot.backLeft(  ).setPower(v4);

        if(gamepad2.a){
            robot.clawGrab().setPosition(1);
        }
        if(gamepad2.b){
            robot.clawGrab().setPosition(0.4);
        }


        if(gamepad2.right_trigger > 0.1){

            armUp = true;
        }
        if(gamepad2.left_trigger > 0.1){

            armUp = false;
        }


        //Linear Slide Logic
        if(gamepad2.dpad_down){
            l =true;

            h = false;
            m = false;
            min = false;
        }
        if(gamepad2.dpad_up){
            h = true;

            l = false;
            m = false;
            min = false;
        }
        if(gamepad2.dpad_left){
            m = true;

            h = false;
            l = false;
            min = false;
        }
        if(gamepad2.dpad_right){
            min = true;

            h = false;
            l = false;
            m = false;
        }



        if(l){


            robot.leftSlide().setTargetPosition(low);
            robot.rightSlide().setTargetPosition(low);

            robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.leftSlide().setVelocity(linearSpeed);
            robot.rightSlide().setVelocity(linearSpeed);

        }
        if(m){
            robot.leftSlide().setTargetPosition(mid);
            robot.rightSlide().setTargetPosition(mid);

            robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.leftSlide().setVelocity(linearSpeed);
            robot.rightSlide().setVelocity(linearSpeed);

        }
        if(h){
            robot.leftSlide().setTargetPosition(high);
            robot.rightSlide().setTargetPosition(high);

            robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.leftSlide().setVelocity(linearSpeed);
            robot.rightSlide().setVelocity(linearSpeed);

        }
        if(min){
            robot.leftSlide().setTargetPosition(0);
            robot.rightSlide().setTargetPosition(0);

            robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.leftSlide().setVelocity(linearSpeed);
            robot.rightSlide().setVelocity(linearSpeed);
        }



        if(armUp){
            robot.armDrive().setTargetPosition(-maxArmHeight);
            robot.armDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armDrive().setVelocity(1000);
        }
        else{
            robot.armDrive().setTargetPosition(0);



            robot.armDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armDrive().setVelocity(1000);


        }





//        if(armUp){
//            robot.armUp(1000);
//            //robot.clawRotate().setPosition(1);
//        }
//        else if(!armUp){
//            robot.armDown(1000);
//            //robot.clawRotate().setPosition(0);
//        }
    }
}
