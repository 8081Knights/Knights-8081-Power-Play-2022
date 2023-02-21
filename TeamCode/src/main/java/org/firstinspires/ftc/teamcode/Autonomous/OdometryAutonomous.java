package org.firstinspires.ftc.teamcode.Autonomous;

import static com.qualcomm.robotcore.hardware.DistanceSensor.distanceOutOfRange;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FusionFramework.DriveTrainIntf;
import org.firstinspires.ftc.teamcode.FusionFramework.DriveTrainIntfTest;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.teamcode.TeleOp.RobotCommands;
import org.firstinspires.ftc.teamcode.TeleOp.compTeleop;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Autonomous(name="OdometryAutonomous")
public class OdometryAutonomous extends LinearOpMode {

    static final int SLIDE_POSITION_LOW    = 950;
    static final int SLIDE_POSITION_LOW_SCORE       = 1200;
    static final int SLIDE_POSITION_MEDIUM_SCORE    = 1300;
    static final int SLIDE_POSITION_CONE_STACK_4    = 310; // 260 works but on rim
    static final int SLIDE_POSITION_CONE_STACK_3    = 200;
    static final int SLIDE_POSITION_CONE_STACK_2    = 100;
    static final int SLIDE_POSITION_CONE_STACK_1    = 0;
    static final int SLIDE_ERROR_MARGIN      = 8;
    static final int ARM_ERROR_MARGIN      = 10;

    static final double ELBOW_POSITION_UP   = 1.0;
    static final double ELBOW_POSITION_CONE_SCORE   = 0.70;
    static final double ELBOW_POSITION_CONE_GRAB   = 0.50;
    static final double ELBOW_POSITION_DOWN  = 0.0;


    static final double WRIST_POSITION_UP   = 0.0;

    static final double CLAW_OPEN   = 1.0;
    static final double CLAW_CLOSE  = 0.0;

    HardwareSoftware robot = new HardwareSoftware();
    RobotCommands commands = new RobotCommands();
    DriveTrainIntf drivetrain;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // April Tag ID's from the 36h11 family
    static final int ID_TAG_POSITION_1 = 5; // Signal Sleeve Position 1
    static final int ID_TAG_POSITION_2 = 18; // Single Sleeve Position 2
    static final int ID_TAG_POSITION_3 = 29;  // Signal Sleeve Position 3

    AprilTagDetection tagOfInterest = null;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        commands.init(robot);
        commands.initGyro();
        camera = robot.getFrontWebCam();
        drivetrain = new DriveTrainIntf(robot); // init the drive train manager
        drivetrain.setMotorMode( DcMotor.RunMode.RUN_USING_ENCODER );

        //calibrate gyro
        while(robot.gyro().isCalibrating()){
            telemetry.addLine("Calibrating Gyro Don't Move!!");
            telemetry.update();
        }
        telemetry.addLine("Gyro Calibrated, OK to Move.");
        telemetry.update();

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    switch(tag.id)
                    {
                        case ID_TAG_POSITION_1:
                        case ID_TAG_POSITION_2:
                        case ID_TAG_POSITION_3:
                            tagFound = true;
                            tagOfInterest = tag;
                            break;
                        default:
                            tagFound = false;
                            break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
            //Display encoder values
            int[] e = drivetrain.getBackEncoderValues(); // read back encoders
            telemetry.addLine("\n \\ENCODER BACK WHEELS");
            telemetry.addLine(String.format("\n Left Encoders=%d", e[DriveTrainIntf.LEFT_ENCODER]));
            telemetry.addLine(String.format("\nRight Encoders=%d", e[DriveTrainIntf.RIGHT_ENCODER]));

            //telemetry.addLine(String.format("\nLeft Ultrasonic=%d", robot.get_left_ultrasonic().getLastRange() ));
            //telemetry.addLine(String.format("\nRight Ultrasonic=%d", robot.get_right_ultrasonic().getLastRange() ));
            telemetry.update();

            // Tell ultrasonics to get the range
            //robot.get_left_ultrasonic().measureRange();
            //robot.get_right_ultrasonic().measureRange();

            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        }
        else
        {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */
            drivetrain.setMotorForwardDirection(true);

            // grab cone
            robot.clawGrab().setPosition(CLAW_CLOSE);
            //45 degrees so don't run into claw mount point
            sleep(50);
            robot.clawGrab().setPosition(CLAW_CLOSE);  // repeat claw grab to be sure
            robot.clawElbow().setPosition(ELBOW_POSITION_CONE_SCORE);
            // move claw into center of the robot
            robot.clawWrist().setPosition(0);
            sleep(50);
            robot.clawGrab().setPosition(CLAW_CLOSE);   // repeat claw grab to be sure

            //move forward a bit before turing
            long encoder_inc = 48;
            int[] e = drivetrain.getBackEncoderValues();
            drivetrain.Drive(0.55);
            // change condition
            drivetrain.check_condition_encoder_distance( this,
                    e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
            drivetrain.stopAll();
            // Pause to prevent wheel slippage and jerking
            try {
                Thread.sleep(50);
            } catch (InterruptedException exc) {
                //ignore
            }

            //turn 45 degrees toward low pole
            drivetrain.gyroTurn ( this, 0.4, -40, 1.50);
            drivetrain.stopAll();
            // Pause to prevent wheel slippage and jerking
            try {
                Thread.sleep(25);
            } catch (InterruptedException exc) {
                //ignore
            }

            //score on the low pole
            //******************************************
            slides_move_to_position(SLIDE_POSITION_LOW, true); // raise the slides
            armToPosition(320); // position claw and arm

            slides_move_to_position(0, false);
            robot.clawGrab().setPosition(CLAW_OPEN);
            sleep(25);

            //turn back towards 0 degrees
            drivetrain.gyroTurn ( this, 0.4, 0, 1.50);
            drivetrain.stopAll();
            // Pause to prevent wheel slippage and jerking
            sleep(25);

            // Drive forward toward the cone stack
            encoder_inc = drivetrain.calcEncoderValueFromCentimeters(55);
            e = drivetrain.getBackEncoderValues();
            // Do autonomous code to move to Location 2
            drivetrain.Drive(0.50);
            // change condition
            drivetrain.check_condition_encoder_distance( this,
                    e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
            //  was working with this => check_condition_Time( 1200 );
            drivetrain.stopAll();

            // Raise the claw
            robot.clawElbow().setPosition(ELBOW_POSITION_UP); // full up

            //turn right towards cone stack
            drivetrain.gyroTurn ( this, 0.4, 91, 1.50);
            drivetrain.stopAll();
            // Pause to prevent wheel slippage and jerking
            sleep(25);

            // Move forward a bit to align between poles
            encoder_inc = 80; // was100 but it sometimes hit the pole
            e = drivetrain.getBackEncoderValues();
            // Do autonomous code to move to Location 2
            drivetrain.Drive(0.50);
            // change condition
            drivetrain.check_condition_encoder_distance( this,
                    e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
            drivetrain.stopAll();

            // Strafe into position in front of cone stack
            encoder_inc = 1140; //short at 1100 //short at 1000; 1200 was too long
            e = drivetrain.getBackEncoderValues();
            // Do autonomous code to move to Location 2
            drivetrain.StrafeLeft(0.65);
            // change condition
            drivetrain.check_condition_encoder_distance( this,
                    e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
            //  was working with this => check_condition_Time( 1200 );
            drivetrain.stopAll();

            sleep (50);

            //Reset turn position after the strafe
            drivetrain.gyroTurn ( this, 0.4, 90, 1.50);
            drivetrain.stopAll();
            // Pause to prevent wheel slippage and jerking
            sleep (50);


            // Set up arm to grab a cone
            //******************************************
            slides_move_to_position(SLIDE_POSITION_CONE_STACK_4, true); // raise the slides
            //move claw down ready to score
            robot.clawGrab().setPosition(CLAW_OPEN);
            sleep(20);
            robot.clawElbow().setPosition(ELBOW_POSITION_CONE_GRAB);
            //sleep(150); // need to wait time for elbow to drop into position to get cone


            //move forward toward cone stack
            encoder_inc = 850; //875 was great for position
            telemetry.update();
            e = drivetrain.getBackEncoderValues();
            // Do autonomous code to move to Location 2
            drivetrain.Drive(0.50);
            // change condition
            drivetrain.check_condition_encoder_distance( this,
                    e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
            drivetrain.stopAll();


            //move bumbers down
            //robot.bumper1().setPosition(0); //was 1 //not working
            // sleep(20);
            robot.bumper2().setPosition(1);
/*
            // get the distance sensor
            double dis = robot.get_front_distance_sensor().getDistance( DistanceUnit.CM);
            if ( dis == distanceOutOfRange ) {
                telemetry.addLine(" Distance Sensor: OUT OF RANGE\n" );
            } else {
                telemetry.addLine(String.format("Distance Sensor:  = %.2f\n", dis));
            }
            telemetry.update();
            */

            // grab a cone
            sleep(50); // pause to ensure we are in position
            robot.clawGrab().setPosition(CLAW_CLOSE);
            sleep(275);
            // pull up the cone
            // Set up arm to grab a cone
            //******************************************
            robot.clawElbow().setPosition(ELBOW_POSITION_CONE_SCORE);
            slides_move_to_position(SLIDE_POSITION_LOW_SCORE, true); // raise the slides

            // move backwards
            //move backward toward scoring pole
            encoder_inc = drivetrain.calcEncoderValueFromCentimeters(22);
            e = drivetrain.getBackEncoderValues();
            drivetrain.Drive(-0.50);  // backwards will automatically decrement the encoder value in DriveTrain.addCountToEncoder()
            // change condition
            drivetrain.check_condition_encoder_distance( this,
                    e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
            drivetrain.stopAll();

            //Turn toward the pole
            drivetrain.gyroTurn ( this, 0.4, 162, 1.50); // 156 to the left
            drivetrain.stopAll();

            // Lower the claw for scoring
            robot.clawElbow().setPosition(ELBOW_POSITION_CONE_GRAB); // full up
            // Pause to prevent wheel slippage and jerking
            sleep (50);


            // ********************************************************************88
            // ********************************************************************88
            // Score on Low Pole
            slides_move_to_position(SLIDE_POSITION_LOW, true); // raise the slides
            slides_move_to_position(0, false);

            robot.clawGrab().setPosition(CLAW_OPEN);
            sleep(25);
            // Raise the claw
            robot.clawElbow().setPosition(ELBOW_POSITION_UP); // full up
            sleep (150);
            // End score on low pole
            // ********************************************************************88
            // ********************************************************************88

            // back up a little

            // turn toward the cones
            drivetrain.gyroTurn ( this, 0.4, 90, 1.50); // 156 to the left

            // Set up arm to grab a cone
            //******************************************
            slides_move_to_position(SLIDE_POSITION_CONE_STACK_3, true); // raise the slides
            //move claw down ready to score
            robot.clawGrab().setPosition(CLAW_OPEN);
            sleep(20);
            robot.clawElbow().setPosition(ELBOW_POSITION_CONE_GRAB);
            //sleep(150); // need to wait time for elbow to drop into position to get cone

            // move forward to cone stack
            encoder_inc = 380;  // 400 is OK, 375 better
            e = drivetrain.getBackEncoderValues();
            drivetrain.Drive(0.50);  // backwards will automatically decrement the encoder value in DriveTrain.addCountToEncoder()
            // change condition
            drivetrain.check_condition_encoder_distance( this,
                    e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
            drivetrain.stopAll();

            // grab a cone
            sleep(150); // pause to ensure we are in position
            robot.clawGrab().setPosition(CLAW_CLOSE);
            sleep(275);
            // pull up the cone
            // Set up arm to grab a cone
            //******************************************
            robot.clawElbow().setPosition(ELBOW_POSITION_CONE_SCORE);
            slides_move_to_position(SLIDE_POSITION_LOW_SCORE, true); // raise the slides


            //move backward toward scoring pole
            encoder_inc = 1000;
            e = drivetrain.getBackEncoderValues();
            drivetrain.Drive(-0.50);  // backwards will automatically decrement the encoder value in DriveTrain.addCountToEncoder()
            // change condition
            drivetrain.check_condition_encoder_distance( this,
                    e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
            drivetrain.stopAll();

            //Turn toward the pole
            drivetrain.gyroTurn ( this, 0.4, 180, 2.50); // 156 to the left
            drivetrain.stopAll();

            // raise arm to score on medium pole
            armToPosition(650);
            // Turn with cone over the pole
            drivetrain.gyroTurn ( this, 0.4, 235, 2.50);
            drivetrain.stopAll();

            slides_move_to_position(SLIDE_POSITION_LOW  , true); // raise the slides

            armToPosition(400);
            robot.clawElbow().setPosition(0.40);

            // Pause to prevent wheel slippage and jerking
            try {
                Thread.sleep(5000);
            } catch (InterruptedException exc) {
                //ignore
            }
           /*
            switch(tagOfInterest.id) {

                case ID_TAG_POSITION_1:
                {
                    // Do autonomous code to move to Location 1
                    // ***********************************************************************************
                    // move forward off the wall and push the cone out of the way
                    //************************************************************************************
                    encoder_inc = 1500; //
                    e = drivetrain.getBackEncoderValues();
                    drivetrain.Drive(0.55);
                    // change condition
                    drivetrain.check_condition_encoder_distance( this,
                            e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
                    drivetrain.stopAll();
                    // Pause to prevent wheel slippage and jerking
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException exc) {
                        //ignore
                    }
                    // ******************************************************************************
                    // Now move backwards into correct position
                    // *******************************************************************************
                    encoder_inc = (420);  // was 340, 380
                    e = drivetrain.getBackEncoderValues();
                    drivetrain.Drive(-0.40); // DO NOT INCREASE SPEED - it will change all the distance sensor and encoder values due to wheel slippage
                    // use ultrasonic to measure distance
                    drivetrain.check_condition_encoder_distance( this,
                            e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
                    drivetrain.stopAll();
                    // Pause to prevent wheel slippage and jerking
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException exc) {
                        //ignore
                    }
                    // ************************************************************************************
                    // turn using the encoders by rotating right
                    //**************************************************************************************
                    encoder_inc = (885); //drivetrain.calcEncoderValueFromCentimeters(59.158);; // drivetrain.calcEncoderValueFromCentimeters(30); //shortest arc length distance
                    e = drivetrain.getBackEncoderValues();
                    drivetrain.RotateRight(0.50); // DO NOT INCREASE SPEED - it will change all the distance sensor and encoder values due to wheel slippage
                    drivetrain.check_condition_encoder_turning( this,
                            e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc, true);
                    drivetrain.stopAll();
                    // Pause to prevent wheel slippage and jerking
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException exc) {
                        //ignore
                    }
                    // ************************************************************************************
                    // Move backwards into location 1
                    //**************************************************************************************
                    encoder_inc = 840;
                    e = drivetrain.getBackEncoderValues();
                    // Do autonomous code to move to Location 2
                    drivetrain.Drive(-0.40);
                    // change condition
                    drivetrain.check_condition_encoder_distance( this,
                            e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
                    drivetrain.stopAll();
                }
                break;

                case ID_TAG_POSITION_2:
                {
                    DcMotorEx od_x = robot.odometryX();
                    //resets odometry encoders to 0
                    od_x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    //run to position 2
                    encoder_inc = drivetrain.calcEncoderValueFromCentimeters(100);
                    e = drivetrain.getBackEncoderValues();
                    // Do autonomous code to move to Location 2
                    drivetrain.Drive(0.50);
                    // change condition
                    drivetrain.check_condition_encoder_distance( this,
                            e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
                    //  was working with this => check_condition_Time( 1200 );
                    drivetrain.stopAll();

                    //read odometry encoder
                    long od_xticks = od_x.getCurrentPosition();

                    //print out value of odometry
                    telemetry.addLine("odometry reading: " + String.valueOf(od_xticks));

                    //odometry update
                    telemetry.update();

                    // Turn 90 degrees toward coones
                    drivetrain.gyroTurn ( this, 0.6, 90, 2.0);
                }
                break;
                case ID_TAG_POSITION_3:
                {
                    // Do autonomous code to move to Location 3
                    // ***********************************************************************************
                    // move forward off the wall and push the cone out of the way
                    //****************************************** ******************************************
                    encoder_inc = 1500; //
                    e = drivetrain.getBackEncoderValues();
                    // Do autonomous code to move to Location 2
                    drivetrain.Drive(0.55);
                    // change condition
                    drivetrain.check_condition_encoder_distance( this,
                            e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
                    drivetrain.stopAll();
                    // Pause to prevent wheel slippage and jerking
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException exc) {
                        //ignore
                    }
                    // ******************************************************************************
                    // Now move backwards into correct position
                    // *******************************************************************************
                    encoder_inc = (320);
                    e = drivetrain.getBackEncoderValues();
                    drivetrain.Drive(-0.40); // DO NOT INCREASE SPEED - it will change all the distance sensor and encoder values due to wheel slippage
                    // use ultrasonic to measure distance
                    drivetrain.check_condition_encoder_distance( this,
                            e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
                    drivetrain.stopAll();
                    // Pause to prevent wheel slippage and jerking
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException exc) {
                        //ignore
                    }

                    // ************************************************************************************
                    // turn using the encoders by rotating left
                    //**************************************************************************************
                    encoder_inc = (885); //drivetrain.calcEncoderValueFromCentimeters(59.158);; // drivetrain.calcEncoderValueFromCentimeters(30); //shortest arc length distance
                    e = drivetrain.getBackEncoderValues();
                    drivetrain.RotateLeft(0.50); // DO NOT INCREASE SPEED - it will change all the distance sensor and encoder values due to wheel slippage
                    drivetrain.check_condition_encoder_turning( this,
                            e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc, false);
                    drivetrain.stopAll();
                    // Pause to prevent wheel slippage and jerking
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException exc) {
                        //ignore
                    }

                    // ************************************************************************************
                    // Move backwards into location 3
                    //**************************************************************************************
                    encoder_inc = 1000;
                    e = drivetrain.getBackEncoderValues();
                    // Do autonomous code to move to Location 2
                    drivetrain.Drive(-0.45);
                    // change condition
                    drivetrain.check_condition_encoder_distance( this,
                            e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
                    drivetrain.stopAll();
                }
                break;
            } */

            robot.clawElbow().setPosition(1);
            robot.clawWrist().setPosition(0);
            robot.clawGrab().setPosition(0.55);
            sleep(1000);
            try {
                Thread.sleep(1500);
            } catch (InterruptedException exe) {
                //ignore
            }

        }

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public boolean check_condition_Time( long time )
    {
        ElapsedTime eTimer = new ElapsedTime(ElapsedTime.MILLIS_IN_NANO);
        eTimer.reset();

        long now = eTimer.time(TimeUnit.MILLISECONDS);

        while (( now < 8000 ) && (!isStopRequested() ) ) {  // maximum wait time is 8 seconds

            if ( now < time) {

                try {
                    Thread.sleep(25); // time condition has not yet been met
                } catch (InterruptedException e) {
                    // ignore interrupt
                }
            } else {
                break;
            }
            now = eTimer.time(TimeUnit.MILLISECONDS);
        }
        return true;
    }


    public void slides_move_to_position( int pos, boolean brake)
    {
        robot.leftSlide().setTargetPosition(pos);
        robot.rightSlide().setTargetPosition(pos);

        if ( brake) {
            robot.leftSlide().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightSlide().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            robot.leftSlide().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rightSlide().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftSlide().setVelocity(2000);
        robot.rightSlide().setVelocity(2000);

        int ls = robot.leftSlide().getCurrentPosition();
        int rs = robot.rightSlide().getCurrentPosition();
        do {
            if (ls == pos) {
                robot.leftSlide().setVelocity(0);
            }
            if (rs == pos) {
                robot.rightSlide().setVelocity(0);
            }
            sleep(25);
            ls = robot.leftSlide().getCurrentPosition();
            rs = robot.rightSlide().getCurrentPosition();

            telemetry.addLine(String.format("slides_move_to_position Left: %d  Right: %d", ls, rs ));
            telemetry.update();

        } while( ( (rs < (pos-SLIDE_ERROR_MARGIN)) || (rs > (pos+SLIDE_ERROR_MARGIN)) )  &&
                 ( (ls < (pos-SLIDE_ERROR_MARGIN)) || (ls > (pos+SLIDE_ERROR_MARGIN)) )   );

//        robot.leftSlide().setVelocity(0);
//        robot.rightSlide().setVelocity(0);

    }

    public void armHome()
    {
        DcMotorEx armDrive = robot.armDrive();
        armDrive.setTargetPosition(0);

        armDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armDrive.setVelocity(1000);

         // set to home position
        robot.clawElbow().setPosition(1); // full up
        robot.clawWrist().setPosition(0);

        while(armDrive.getCurrentPosition() != 0){
            sleep(25);
        }

        armDrive.setVelocity(0);
    }


    //  Used to set the Arm to 45% and claw forward to grab or drop
    //
    public void arm45Degrees()
    {
        int target = 250;
        DcMotorEx armDrive = robot.armDrive();
        armDrive.setTargetPosition(target);

        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armDrive.setVelocity(1500);

        // set the claw to ~40%
        robot.clawElbow().setPosition(0.38);  //elbowMid - it was 45, but it was slightly too slow, it was 40, but it was too low EA
        robot.clawWrist().setPosition(0);

        int pos = armDrive.getCurrentPosition();
        while(pos != target){
            if (( pos > (target-ARM_ERROR_MARGIN)) && (pos < (target+ARM_ERROR_MARGIN)) ) {
                break;
            }
            sleep(25);
            pos = armDrive.getCurrentPosition();
        }

        armDrive.setVelocity(0);
    }


    public void armToPosition(int target)
    {
        DcMotorEx armDrive = robot.armDrive();
        armDrive.setTargetPosition(target);

        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armDrive.setVelocity(1500);

        // set the claw to ~40%
        robot.clawElbow().setPosition(ELBOW_POSITION_CONE_SCORE);
        robot.clawWrist().setPosition(WRIST_POSITION_UP);

        int pos = armDrive.getCurrentPosition();
        while(pos != target){
            if (( pos > (target-ARM_ERROR_MARGIN)) && (pos < (target+ARM_ERROR_MARGIN)) ) {
                break;
            }
            sleep(25);
            pos = armDrive.getCurrentPosition();
        }

        //armDrive.setVelocity(0);
    }


    /* Control Variables
        // reset positions
        robot.clawElbow().setPosition(1); // full up
        robot.clawWrist().setPosition(0);  //  side to side movement
        robot.clawGrab().setPosition(0);

        // Positions to grab a cone
        robot.clawElbow().setPosition(0.38);  //elbowMid - it was 45, but it was slightly too slow, it was 40, but it was too low EA
        robot.clawWrist().setPosition(0);
        robot.clawGrab().setPosition(0.55);  // claw open

     */
}

