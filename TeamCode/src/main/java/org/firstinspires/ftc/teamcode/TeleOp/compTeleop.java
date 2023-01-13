package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HardwareSoftware;



@TeleOp(name = "Comp Tele")
public class compTeleop extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();
    RobotCommands commands = new RobotCommands();


    double speedMult = 0.7;
    boolean x = false;
    boolean y = false;


    int scoreHeight = 200;
    int pickHeight = 0;

    int slideTolerance = 10;


    enum slideHeight{
        Ground,
        Low,
        Middle,
        High,
        HighB,
        Nothing,
        Home,
        score,
        Pickup,
        stack1,
        stack2,
        stack3
    }

    slideHeight slide = slideHeight.Nothing;
    slideHeight prevPos = slide;

    int low = 350;
    int mid = 1000;
    int high = 1550;

    int stack1 = 30;
    int stack2 = 60;
    int stack3 = 90;

    @Override
    public void init() {

        robot.init(hardwareMap);
        commands.init(robot);

    }

    @Override
    public void loop() {
        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = gamepad1.left_stick_y;
        double strafe = gamepad1.right_stick_x;
        double twist  = gamepad1.left_stick_x;



        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (-drive + strafe + twist),
                (-drive - strafe + twist),
                (drive - strafe + twist),
                (drive + strafe + twist)

        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        robot.frontLeft().setPower(speeds[0]*speedMult);
        robot.frontRight().setPower(speeds[1]*speedMult);
        robot.backLeft().setPower(speeds[2]*speedMult);
        robot.backRight().setPower(speeds[3]*speedMult);



        if (gamepad1.a){
            slide = slideHeight.Home;
            robot.clawElbow().setPosition(1);
            robot.clawWrist().setPosition(0);
            robot.clawGrab().setPosition(0);
            pickHeight = 0;

        }
        else if(gamepad1.b){
            slide = slideHeight.Home;
            robot.clawElbow().setPosition(0.4);  //it was 45, but it was slightly too slow
            robot.clawWrist().setPosition(0);
            robot.clawGrab().setPosition(0.55);
            pickHeight = 0;

        }
        else if(gamepad1.dpad_right){
            slide = slideHeight.Ground;
            robot.clawWrist().setPosition(0);
            robot.clawElbow().setPosition(0.4);
        }
        else if(gamepad2.dpad_right){
            slide = slideHeight.HighB;
            robot.clawWrist().setPosition(1);
            robot.clawElbow().setPosition(1);
        }
        else if(gamepad1.dpad_down){
            slide = slideHeight.Low;
            robot.clawWrist().setPosition(0);
            robot.clawElbow().setPosition(0);
        }
        else if(gamepad1.dpad_left){
            slide = slideHeight.Middle;
            robot.clawWrist().setPosition(0);
            robot.clawElbow().setPosition(0);
        }
        else if(gamepad1.dpad_up){
            slide = slideHeight.High;
            robot.clawWrist().setPosition(0);
            robot.clawElbow().setPosition(0);
        }

        else if(gamepad2.dpad_down){
            slide = slideHeight.stack1;
        }



        else if(gamepad1.left_bumper){
            robot.clawGrab().setPosition(0.55);
        }
        else if(gamepad1.right_bumper){
            robot.clawGrab().setPosition(0);
        }

//        else if(gamepad1.y){
//            speedMult = .2;
//        }
        else if(gamepad1.x) {

            x = true;
        }

        else if(x && !gamepad1.x){
            x = false;

            if(speedMult == .7){
                speedMult = .2;
            }
            else{
                speedMult = .7;
            }
        }
        else if(x && !gamepad1.x){
            x = false;
            if(speedMult == 0.7){
                speedMult = 0.2;
            }
            else {
                speedMult = 0.7;
            }


        }

        else if(gamepad1.y){
            y = true;
        }
        else if (y && !gamepad1.y){
            slide = slideHeight.Pickup;
            pickHeight += 250;
            y = false;
        }

        else if(gamepad1.right_trigger > .1){
            slide = slideHeight.score;
        }




        switch(slide){
            case Ground:
                robot.leftSlide().setTargetPosition(100);
                robot.rightSlide().setTargetPosition(100);



                robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                commands.armOut();
                prevPos = slide;

                if(robot.leftSlide().getCurrentPosition() == 0 && robot.rightSlide().getCurrentPosition() == 0){
                    break;

                }
                else{
                    robot.leftSlide().setVelocity(2000);
                    robot.rightSlide().setVelocity(2000);
                    break;
                }


            case Low:
                robot.leftSlide().setTargetPosition(low);
                robot.rightSlide().setTargetPosition(low);


                robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                commands.armOut();
                prevPos = slide;
                robot.leftSlide().setVelocity(2000);
                robot.rightSlide().setVelocity(2000);
                if(robot.leftSlide().getCurrentPosition() ==  low && robot.rightSlide().getCurrentPosition() == low){
                    break;

                }
                else{
                    robot.leftSlide().setVelocity(2000);
                    robot.rightSlide().setVelocity(2000);
                    break;
                }

            case Middle:
                robot.leftSlide().setTargetPosition(mid);
                robot.rightSlide().setTargetPosition(mid);


                robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                commands.armHigh();
                prevPos = slide;
                if(robot.leftSlide().getCurrentPosition() == mid && robot.rightSlide().getCurrentPosition() == mid){
                    break;

                }
                else{
                    robot.leftSlide().setVelocity(2000);
                    robot.rightSlide().setVelocity(2000);
                    break;
                }

            case High:

                robot.leftSlide().setTargetPosition(high);
                robot.rightSlide().setTargetPosition(high);


                robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                commands.armHigh();
                prevPos = slide;

                if(robot.leftSlide().getCurrentPosition() == high && robot.rightSlide().getCurrentPosition() == high){
                    break;

                }
                else{
                    robot.leftSlide().setVelocity(2000);
                    robot.rightSlide().setVelocity(2000);
                    break;
                }

            case HighB:
                robot.leftSlide().setTargetPosition(high);
                robot.rightSlide().setTargetPosition(high);


                robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                commands.armBack();
                prevPos = slide;
                robot.leftSlide().setVelocity(2000);
                robot.rightSlide().setVelocity(2000);

                break;

            case Nothing:
                robot.leftSlide().setTargetPosition(robot.leftSlide().getCurrentPosition());
                robot.rightSlide().setTargetPosition(robot.rightSlide().getCurrentPosition());
                robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                prevPos = slide;

                break;

            case Home:

                robot.leftSlide().setTargetPosition(0);
                robot.rightSlide().setTargetPosition(0);


                robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                commands.armHome();
                prevPos = slide;

                if(robot.leftSlide().getCurrentPosition() == 0 && robot.rightSlide().getCurrentPosition() == 0){
                    break;

                }
                else{
                    robot.leftSlide().setVelocity(2000);
                    robot.rightSlide().setVelocity(2000);
                    break;
                }

            case Pickup:
                robot.leftSlide().setTargetPosition(pickHeight);
                robot.rightSlide().setTargetPosition(pickHeight);


                robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                commands.armHome();
                prevPos = slide;

                if(robot.leftSlide().getCurrentPosition() == pickHeight && robot.rightSlide().getCurrentPosition() == pickHeight){
                    robot.leftSlide().setVelocity(0);
                    robot.rightSlide().setVelocity(0);
                    break;

                }
                else{
                    robot.leftSlide().setVelocity(2000);
                    robot.rightSlide().setVelocity(2000);
                    break;
                }


            case score:
                switch(prevPos){
                    case High:


                        robot.leftSlide().setTargetPosition(high - scoreHeight);
                        robot.rightSlide().setTargetPosition(high - scoreHeight);


                        robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


                        commands.armOut();



                        if((robot.leftSlide().getCurrentPosition() >= high - scoreHeight - slideTolerance && robot.leftSlide().getCurrentPosition() <= high - scoreHeight + slideTolerance) && (robot.rightSlide().getCurrentPosition() >= high - scoreHeight - slideTolerance && robot.rightSlide().getCurrentPosition() <= high - scoreHeight + slideTolerance)){
                            telemetry.addLine("SUCCESS!");
                            robot.clawGrab().setPosition(1);
                            break;

                        }
                        else{
                            robot.leftSlide().setVelocity(2000);
                            robot.rightSlide().setVelocity(2000);
                            break;
                        }



                    case HighB:


                        robot.leftSlide().setTargetPosition(high - scoreHeight);
                        robot.rightSlide().setTargetPosition(high - scoreHeight);


                        robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


                        commands.armBack();


                        if((robot.leftSlide().getCurrentPosition() >= high - scoreHeight - slideTolerance && robot.leftSlide().getCurrentPosition() <= high - scoreHeight + slideTolerance) && (robot.rightSlide().getCurrentPosition() >= high - scoreHeight - slideTolerance && robot.rightSlide().getCurrentPosition() <= high - scoreHeight + slideTolerance)){
                            telemetry.addLine("SUCCESS!");
                            robot.clawGrab().setPosition(1);
                            break;

                        }
                        else{
                            robot.leftSlide().setVelocity(2000);
                            robot.rightSlide().setVelocity(2000);

                        }


                    case Middle:
                        robot.leftSlide().setTargetPosition(mid - scoreHeight);
                        robot.rightSlide().setTargetPosition(mid - scoreHeight);


                        robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


                        commands.armOut();



                        if((robot.leftSlide().getCurrentPosition() >= mid - scoreHeight - slideTolerance && robot.leftSlide().getCurrentPosition() <= mid - scoreHeight + slideTolerance) && (robot.rightSlide().getCurrentPosition() >= mid - scoreHeight - slideTolerance && robot.rightSlide().getCurrentPosition() <= mid - scoreHeight + slideTolerance)){
                            telemetry.addLine("SUCCESS!");
                            robot.clawGrab().setPosition(1);
                            break;

                        }
                        else{
                            robot.leftSlide().setVelocity(2000);
                            robot.rightSlide().setVelocity(2000);

                        }


                    case Low:
                        robot.leftSlide().setTargetPosition(low - scoreHeight);
                        robot.rightSlide().setTargetPosition(low - scoreHeight);


                        robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        robot.leftSlide().setVelocity(2000);
                        robot.rightSlide().setVelocity(2000);

                        commands.armOut();

                        if((robot.leftSlide().getCurrentPosition() >= low - scoreHeight - slideTolerance && robot.leftSlide().getCurrentPosition() <= low - scoreHeight + slideTolerance) && (robot.rightSlide().getCurrentPosition() >= low - scoreHeight - slideTolerance && robot.rightSlide().getCurrentPosition() <= low - scoreHeight + slideTolerance)){
                            telemetry.addLine("SUCCESS!");
                            robot.clawGrab().setPosition(1);
                            break;

                        }
                        else{
                            robot.leftSlide().setVelocity(2000);
                            robot.rightSlide().setVelocity(2000);

                        }



                }


                telemetry.addData("Left Slide: ", robot.leftSlide().getCurrentPosition());
                telemetry.addData("Right Slide: ", robot.rightSlide().getCurrentPosition());
                telemetry.addData("Pick Height: ", pickHeight);



        }



    }
}
