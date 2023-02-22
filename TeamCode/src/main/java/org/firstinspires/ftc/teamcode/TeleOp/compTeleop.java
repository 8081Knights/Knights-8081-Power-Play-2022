package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

//TODO: Test the low position for the linear slides because it uses the truncated RobotCommands method
//TODO: Finish replacing the switch case code with the truncated RobotCommands method after getting low to work correctly and consistently

//Written by the programmers of 8081 Knights of the Lab Table
//Alex, Nathaniel, Sydney, Thomas

//Honorary Mentions:
//Ethan - Helped with tuning and testing


@TeleOp(name = "1. Ethan Teleop")
public class compTeleop extends OpMode {


    //Hardware Map object
    HardwareSoftware robot = new HardwareSoftware();

    //Truncated programs object
    RobotCommands commands = new RobotCommands();

    //Timer Variable
    ElapsedTime timer = new ElapsedTime();

    double voltage;

    // Handles the percentage of the motors when running the drive train, sets the initial speed to 70%
    double speedMult = 1;


    //High and Low percentages for Drive train speed
    //TODO: Can be tuned
    double highDtSpeed = 1;
    double lowDtSpeed = 0.3;

    //Storage variable in order to make push button logic to work DONT TOUCH
    boolean x = false;
    boolean y = false;
    boolean y2 = false;
    boolean leftTrigger = false;

    //Tunable servo position variables

    //Storage variable How wide to open the claw
    //TODO: Can be tuned
    double clawOpen = 0.55;
    double clawClose = 0;

    //Storage variable for the height of the elbow joint when ready to pick up a cone
    //TODO: Can be tuned
    double elbowMid = 0.5;

    //Variable that changes where the claw elbow goes when the left trigger is pressed
    double clawScore = 0.8;



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
    int low = 1350;
    int mid = 1675;
    int high = 1675;

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

        //Initialize Hardware Map
        robot.init(hardwareMap);

        //Initialize pre programmed commands file
        commands.init(robot);

        timer.reset();
        timer.startTime();

        //Make sure the claw is properly placed
        robot.clawElbow().setPosition(1);
        robot.clawWrist().setPosition(0);
        robot.clawGrab().setPosition(0);


        //Initialize and Calibrate the Gyro
        robot.gyro().initialize();
        robot.gyro().calibrate();

        while(robot.gyro().isCalibrating()){

            telemetry.addLine("Gyro is calibrating");
            telemetry.update();

        }

        telemetry.clear();
        telemetry.addLine("Ready to Drive");

        telemetry.update();


    }

    @Override
    public void loop() {

        while(robot.gyro().isCalibrating()){
            robot.Blinkin().setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            telemetry.addLine("Gyro is Calibrating!!");
            telemetry.update();
        }

        //Turn Variable for Headless Robot Logic
        double driveTurn = -gamepad1.right_stick_x;
        //driveVertical = -gamepad1.right_stick_y;
        //driveHorizontal = gamepad1.right_stick_x;

        //Drive X and Y for Headless
        double gamepadXCoordinate = gamepad1.left_stick_x; //this simply gives our x value relative to the driver
        double gamepadYCoordinate = -gamepad1.left_stick_y; //this simply gives our y vaue relative to the driver



        //the inverse tangent of opposite/adjacent gives us our gamepad degree
        double robotDegree = Math.toRadians(robot.gyro().getHeading());


        //Final X and Y for corrected driving (Field Centric Drive Logic)
        double rotX = gamepadXCoordinate * Math.cos(robotDegree) - gamepadYCoordinate * Math.sin(robotDegree);
        double rotY = gamepadXCoordinate * Math.sin(robotDegree) + gamepadYCoordinate * Math.cos(robotDegree);


        //Denominator makes sure the motors are never set past 1 power
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(driveTurn), 1);

        //Power Variables
        double frontLeftPower = ((rotY + rotX - driveTurn) / denominator)*speedMult;
        double backLeftPower = ((-rotY + rotX + driveTurn) / denominator)*speedMult;
        double frontRightPower = ((rotY + rotX + driveTurn) / denominator)*speedMult;
        double backRightPower = ((-rotY + rotX - driveTurn) / denominator)*speedMult;

        //Set Power to Motors
        robot.frontRight().setPower(frontRightPower);
        robot.frontLeft().setPower(frontLeftPower);
        robot.backRight().setPower(backRightPower);
        robot.backLeft().setPower(backLeftPower);


        // Set the height to Home
        if (gamepad1.a){
            slide = slideHeight.Home;
            robot.clawElbow().setPosition(1);
            robot.clawWrist().setPosition(0);
            robot.clawGrab().setPosition(clawOpen);
            pickHeight = 0;
            speedMult = highDtSpeed;

        }
        else if(gamepad1.b){
            slide = slideHeight.Home;
            robot.clawElbow().setPosition(elbowMid);  //it was 45, but it was slightly too slow, it was 40, but it was too low EA
            robot.clawWrist().setPosition(0);
            robot.clawGrab().setPosition(clawOpen);
            pickHeight = 0;
            speedMult = highDtSpeed;

        }

        // Set the height to Ground
        else if(gamepad1.dpad_right){
            slide = slideHeight.Ground;
            robot.clawWrist().setPosition(0);
            robot.clawElbow().setPosition(elbowMid);
            speedMult = lowDtSpeed;

        }

        // Set the height to the High Back position
        else if(gamepad2.dpad_right){
            slide = slideHeight.HighB;
            robot.clawWrist().setPosition(1);
            robot.clawElbow().setPosition(1);
            speedMult = lowDtSpeed;

        }

        // Set the height to the Low position
        else if(gamepad1.dpad_down){
            slide = slideHeight.Low;
            robot.clawWrist().setPosition(0);
            robot.clawElbow().setPosition(elbowMid);
            speedMult = lowDtSpeed;

        }

        // Set the height to the Middle position
        else if(gamepad1.dpad_left){
            slide = slideHeight.Middle;
            robot.clawWrist().setPosition(0);
            robot.clawElbow().setPosition(elbowMid);
            speedMult = lowDtSpeed;

        }

        // Set the height to the high position
        else if(gamepad1.dpad_up){
            slide = slideHeight.High;
            robot.clawWrist().setPosition(0);
            robot.clawElbow().setPosition(0);
            speedMult = lowDtSpeed;

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
        else if(gamepad1.left_trigger > .1){
            leftTrigger = true;
        }
        else if(leftTrigger && gamepad1.left_trigger < .1){
            leftTrigger = false;

            if(robot.clawElbow().getPosition() > 0.5){
                robot.clawElbow().setPosition(0.42);
            }
            else{
                robot.clawElbow().setPosition(clawScore);
            }
        }

        else if(y2 && !gamepad2.y){
            if(robot.bumper2().getPosition() > 0){
                robot.bumper1().setPosition(1);
                robot.bumper2().setPosition(0);
            }
            else{
                robot.bumper1().setPosition(0.5);
                robot.bumper2().setPosition(0.5);
            }
        }

        else if(gamepad1.x && gamepad1.b){
            robot.gyro().calibrate();

        }

        //Touch Sensor Voltage
        voltage =  robot.FSR().getVoltage();

        //LED code
        if(timer.time() >= 90 && timer.time() <= 120 ){
            robot.Blinkin().setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        } else {
            robot.Blinkin().setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
        }

        //Activate LED Change when cone picked up
        if(voltage > .5){
            robot.Blinkin().setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
        }


     /*   else if(gamepad1.left_trigger > .1){
            slide = slideHeight.score;
        }
*/


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
                commands.armMid();
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
                commands.armOut();
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
                        commands.armOut();
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
                        commands.armMid();
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
                        commands.armHome();
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
