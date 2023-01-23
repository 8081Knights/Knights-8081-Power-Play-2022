package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

//TODO: Test the low position for the linear slides because it uses the truncated RobotCommands method
//TODO: Finish replacing the switch case code with the truncated RobotCommands method after getting low to work correctly and consistently

//Written by the programmers of 8081 Knights of the Lab Table
//Alex, Nathaniel, Sydney, Thomas

//Honorary Mentions:
//Ethan - Helped with tuning and testing


@TeleOp(name = "Comp Tele")
public class compTeleop extends OpMode {


    //Hardware Map object
    HardwareSoftware robot = new HardwareSoftware();

    //Truncated programs object
    RobotCommands commands = new RobotCommands();

    // Handles the percentage of the motors when running the drive train, sets the initial speed to 70%
    double speedMult = 0.85;


    //High and Low percentages for Drive train speed
    //TODO: Can be tuned
    double highDtSpeed = 0.85;
    double lowDtSpeed = 0.2;

    //Storage variable in order to make push button logic to work DONT TOUCH
    boolean x = false;
    boolean y = false;

    //Tunable servo position variables

    //Storage variable How wide to open the claw
    //TODO: Can be tuned
    double clawOpen = 0.55;
    double clawClose = 0;

    //Storage variable for the height of the elbow joint when ready to pick up a cone
    //TODO: Can be tuned
    double elbowMid = 0.38;


    //ALL OF THE STORAGE VARIABLES BELOW RELATE TO THE MOVEMENT AND TUNING OF THE LINEAR SLIDES

    // Height that the linear slides drop when you pull the trigger
    //TODO: Can be tuned however system doesn't work perfectly yet
    int scoreHeight = 500;

    // Height the linear slides go to when picking cones up off the stack
    int pickHeight = 0;

    // Range of which slides can be within the tolerances
    //TODO: Can be tuned but wouldn't recommend
    int slideTolerance = 0;

    //Speed the Slides move at
    //TODO: Can be tuned
    int slideSpeed = 2000;

    //Linear Slide tick heights for each Pole height
    //TODO: Can be tuned
    int low = 1450;
    int mid = 1700;
    int high = 1700;

    // Enumerator for availible positions at which slides can go to DO NOT TOUCH!
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


        // Set the height to Home
        if (gamepad1.a){
            slide = slideHeight.Home;
            robot.clawElbow().setPosition(1);
            robot.clawWrist().setPosition(0);
            robot.clawGrab().setPosition(0);
            pickHeight = 0;

        }
        else if(gamepad1.b){
            slide = slideHeight.Home;
            robot.clawElbow().setPosition(elbowMid);  //it was 45, but it was slightly too slow, it was 40, but it was too low EA
            robot.clawWrist().setPosition(0);
            robot.clawGrab().setPosition(clawOpen);
            pickHeight = 0;

        }

        // Set the height to Ground
        else if(gamepad1.dpad_right){
            slide = slideHeight.Ground;
            robot.clawWrist().setPosition(0);
            robot.clawElbow().setPosition(elbowMid);
        }

        // Set the height to the High Back position
        else if(gamepad2.dpad_right){
            slide = slideHeight.HighB;
            robot.clawWrist().setPosition(1);
            robot.clawElbow().setPosition(1);
        }

        // Set the height to the Low position
        else if(gamepad1.dpad_down){
            slide = slideHeight.Low;
            robot.clawWrist().setPosition(0);
            robot.clawElbow().setPosition(elbowMid);
        }

        // Set the height to the Middle position
        else if(gamepad1.dpad_left){
            slide = slideHeight.Middle;
            robot.clawWrist().setPosition(0);
            robot.clawElbow().setPosition(elbowMid);
        }

        // Set the height to the high position
        else if(gamepad1.dpad_up){
            slide = slideHeight.High;
            robot.clawWrist().setPosition(0);
            robot.clawElbow().setPosition(0);
        }

        /*
        else if(gamepad2.dpad_down){
            slide = slideHeight.stack1;
        } */



        else if(gamepad1.left_bumper){
            robot.clawGrab().setPosition(clawOpen);
        }
        else if(gamepad1.right_bumper){
            robot.clawGrab().setPosition(clawClose);
        }

//        else if(gamepad1.y){
//            speedMult = .2;
//        }

        //Push button logic for Toggling the Speed multiplier
        else if(gamepad1.x) {

            x = true;
        }

        //Changes the speed of the Drive train on the up press of Gamepad 1 X
        else if(x && !gamepad1.x){
            x = false;

            if(speedMult == highDtSpeed){
                speedMult = lowDtSpeed;
            }
            else{
                speedMult = highDtSpeed;
            }
        }

        // Logic for the cone stack -- Makes the slides run to a position
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



        // Handles arm and claw logic -- preset instructions for positioning
        switch(slide){
            case Ground:
                robot.leftSlide().setTargetPosition(100);
                robot.rightSlide().setTargetPosition(100);



                robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                commands.armOut();
                commands.armHome();
                prevPos = slide;

                if(robot.leftSlide().getCurrentPosition() == 0 && robot.rightSlide().getCurrentPosition() == 0){
                    break;

                }
                else{
                    robot.leftSlide().setVelocity(slideSpeed);
                    robot.rightSlide().setVelocity(slideSpeed);
                    break;
                }


            case Low:


                robot.leftSlide().setTargetPosition(low);
                robot.rightSlide().setTargetPosition(low);


                robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                commands.armHome();
                prevPos = slide;
                if(robot.leftSlide().getCurrentPosition() == mid && robot.rightSlide().getCurrentPosition() == mid){
                    break;

                }
                else{
                    robot.leftSlide().setVelocity(slideSpeed);
                    robot.rightSlide().setVelocity(slideSpeed);
                    break;
                }




            case Middle:
                robot.leftSlide().setTargetPosition(mid);
                robot.rightSlide().setTargetPosition(mid);


                robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                prevPos = slide;
                commands.armHome();
                if(robot.leftSlide().getCurrentPosition() == mid && robot.rightSlide().getCurrentPosition() == mid){
                    break;

                }
                else{
                    robot.leftSlide().setVelocity(slideSpeed);
                    robot.rightSlide().setVelocity(slideSpeed);
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
                    robot.leftSlide().setVelocity(slideSpeed);
                    robot.rightSlide().setVelocity(slideSpeed);
                    break;
                }

            case HighB:
                robot.leftSlide().setTargetPosition(high);
                robot.rightSlide().setTargetPosition(high);


                robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                commands.armBack();
                prevPos = slide;
                robot.leftSlide().setVelocity(slideSpeed);
                robot.rightSlide().setVelocity(slideSpeed);

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
                    robot.leftSlide().setVelocity(slideSpeed);
                    robot.rightSlide().setVelocity(slideSpeed);
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
                    robot.leftSlide().setVelocity(slideSpeed);
                    robot.rightSlide().setVelocity(slideSpeed);
                    break;
                }


            case score:
                switch(prevPos){
                    case High:


                        robot.leftSlide().setTargetPosition(high - scoreHeight);
                        robot.rightSlide().setTargetPosition(high - scoreHeight);


                        robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        commands.armHigh();
                        prevPos = slide;
                        if(robot.leftSlide().getCurrentPosition() == mid && robot.rightSlide().getCurrentPosition() == mid){
                            break;

                        }
                        else{
                            robot.leftSlide().setVelocity(slideSpeed);
                            robot.rightSlide().setVelocity(slideSpeed);
                            break;
                        }




                    case HighB:


                        robot.leftSlide().setTargetPosition(high - scoreHeight);
                        robot.rightSlide().setTargetPosition(high - scoreHeight);

                        robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        commands.armBack();
                        prevPos = slide;
                        if(robot.leftSlide().getCurrentPosition() == mid && robot.rightSlide().getCurrentPosition() == mid){
                            break;

                        }
                        else{
                            robot.leftSlide().setVelocity(slideSpeed);
                            robot.rightSlide().setVelocity(slideSpeed);
                            break;
                        }


                    case Middle:
                        robot.leftSlide().setTargetPosition(mid - scoreHeight);
                        robot.rightSlide().setTargetPosition(mid - scoreHeight);


                        robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        commands.armHigh();
                        prevPos = slide;
                        if(robot.leftSlide().getCurrentPosition() == mid && robot.rightSlide().getCurrentPosition() == mid){
                            break;

                        }
                        else{
                            robot.leftSlide().setVelocity(slideSpeed);
                            robot.rightSlide().setVelocity(slideSpeed);
                            break;
                        }
                    case Low:
                        robot.leftSlide().setTargetPosition(low - scoreHeight);
                        robot.rightSlide().setTargetPosition(low - scoreHeight);


                        robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        prevPos = slide;
                        if(robot.leftSlide().getCurrentPosition() == mid && robot.rightSlide().getCurrentPosition() == mid){
                            break;

                        }
                        else{
                            robot.leftSlide().setVelocity(slideSpeed);
                            robot.rightSlide().setVelocity(slideSpeed);
                            break;
                        }



                }


                telemetry.addData("Left Slide: ", robot.leftSlide().getCurrentPosition());
                telemetry.addData("Right Slide: ", robot.rightSlide().getCurrentPosition());
                telemetry.addData("Pick Height: ", pickHeight);
                telemetry.update();



        }





    }


//    //Method to handle and truncate linear slide movement
//    public void slideChange(int height, int speed, int tolerance){
//
//
//        //Set target height for linear slides
//        robot.leftSlide().setTargetPosition(height);
//        robot.rightSlide().setTargetPosition(height);
//
//
//        //Tell the linear slides to move to the correct position
//        robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        if((robot.leftSlide().getCurrentPosition() >= height - tolerance || robot.leftSlide().getCurrentPosition() <= height + tolerance) && (robot.rightSlide().getCurrentPosition() >= height - tolerance || robot.rightSlide().getCurrentPosition() <= height + tolerance)){
//            robot.leftSlide().setVelocity(0);
//            robot.rightSlide().setVelocity(0);
//
//
//        }
//        else{
//
//            robot.leftSlide().setVelocity(speed);
//            robot.rightSlide().setVelocity(speed);
//
//        }
//
//
//
//    }
}
