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

@Autonomous(name="multiConeAutoLeft")
public class multiConeAutoLeft extends LinearOpMode{
    static final int SLIDE_POSITION_LOW    = 950;
    static final int SLIDE_POSITION_LOW_SCORE       = 1200;
    static final int SLIDE_POSITION_MEDIUM_SCORE    = 1290;
    static final int SLIDE_POSITION_HIGH_SCORE      = 1460;  // was 1490 and too high
    static final int SLIDE_POSITION_CONE_STACK_4    = 330; //was 310 // 260 works but on rim
    static final int SLIDE_POSITION_CONE_STACK_3    = 200;
    static final int SLIDE_POSITION_CONE_STACK_2    = 100;
    static final int SLIDE_POSITION_CONE_STACK_1    = 0;
    static final int SLIDE_ERROR_MARGIN      = 8;
    static final int ARM_ERROR_MARGIN      = 10;

    static final double ELBOW_POSITION_UP   = 1.0;
    static final double ELBOW_POSITION_CONE_SCORE   = 0.38;
    static final double ELBOW_POSITION_CONE_GRAB   = 0.50;
    static final double ELBOW_POSITION_LOW_POLE = 0.70;
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
            long encoder_inc = 50; //40 & 38 too short //35 too short //32 too short // 28 too short // 23 too short // 200 was too short // too far forward 32 //48 too far forward
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
            drivetrain.gyroTurn ( this, 0.4, 45, 1.50); //was 40 but turning too little
            drivetrain.stopAll();
            // Pause to prevent wheel slippage and jerking
            try {
                Thread.sleep(25);
            } catch (InterruptedException exc) {
                //ignore
            }

            //score on the low pole
            //******************************************
            robot.clawElbow().setPosition(ELBOW_POSITION_CONE_SCORE); //0.70
            slides_move_to_position(SLIDE_POSITION_LOW, true); // raise the slides
            armToPosition(270); //288 290 & 295 too much //300 too much //305 too much // 308 too much // 293 290 was sometimes short//280 & 270 too short //250 to short //290 too far // position claw and arm

            slides_move_to_position(0, false);
            robot.clawGrab().setPosition(CLAW_OPEN);
//            robot.clawElbow().setPosition(ELBOW_POSITION_UP);
            armHome();
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
            drivetrain.gyroTurn ( this, 0.4, -90, 1.50);
            drivetrain.stopAll();
            // Pause to prevent wheel slippage and jerking
            sleep(25);

            // Move forward a bit to align between poles
            encoder_inc = 60; // was 80 & 100 but it sometimes hit the pole
            e = drivetrain.getBackEncoderValues();
            // Do autonomous code to move to Location 2
            drivetrain.Drive(0.50);
            // change condition
            drivetrain.check_condition_encoder_distance( this,
                    e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
            drivetrain.stopAll();

            // realign robot so strafing is good
            drivetrain.gyroTurn ( this, 0.4, -90, 1.50);
            drivetrain.stopAll();
            // Strafe into position in front of cone stack
                   // -1120 was a bit too far sometimes
            encoder_inc = -1075; // must be negative in order to strafe right // 1100 was too far, 1050 was way too short
            e = drivetrain.getBackEncoderValues();
            // Do autonomous code to move to Location 2
            drivetrain.StrafeRight(0.55);
            // change condition
            drivetrain.check_condition_encoder_distance( this,
                    e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
            //  was working with this => check_condition_Time( 1200 );
            drivetrain.stopAll();

            //Reset turn position after the strafe
            drivetrain.gyroTurn ( this, 0.4, -90, 1.50);
            drivetrain.stopAll();
            // Pause to prevent wheel slippage and jerking
            sleep (50);


            // Set up arm to grab a cone
            //******************************************
            slides_move_to_position(SLIDE_POSITION_CONE_STACK_4 + 60, true); // raise the slides
            //move claw down ready to score
            robot.clawGrab().setPosition(CLAW_OPEN);
            sleep(20);
            robot.clawElbow().setPosition(ELBOW_POSITION_CONE_GRAB);
            //sleep(150); // need to wait time for elbow to drop into position to get cone


            //move forward toward cone stack
            encoder_inc = 795; //785, 775 was a bit short
            telemetry.update();
            e = drivetrain.getBackEncoderValues();
            drivetrain.Drive(0.50);
            // change condition
            drivetrain.check_condition_encoder_distance( this,
                    e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
            drivetrain.stopAll();


            //*****************************************************************************************
            //*****************************************************************************************
            //*****************************************************************************************
            // Grab the FIRST cone
            //*****************************************************************************************
            //*****************************************************************************************
            //*****************************************************************************************
            sleep(50); // pause to ensure we are in position
            robot.clawGrab().setPosition(CLAW_CLOSE);
            sleep(500);
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
            // Raise the claw for scoring
            robot.clawElbow().setPosition(ELBOW_POSITION_LOW_POLE);
            // turn to palce cone over the pole
            drivetrain.gyroTurn ( this, 0.4, -156, 1.50); // was 162 too short // 156 to the left
            drivetrain.stopAll();

            // Lower the claw for scoring
            robot.clawElbow().setPosition(ELBOW_POSITION_CONE_SCORE); // full up
            // Pause to prevent wheel slippage and jerking
            sleep (50);


            // ********************************************************************88
            // ********************************************************************88
            // Score on Low Pole
            slides_move_to_position(SLIDE_POSITION_LOW_SCORE, true); //50 too low // raise the slides
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
            drivetrain.gyroTurn ( this, 0.4, -90, 1.50); // 156 to the left

            // Set up arm to grab a cone
            //******************************************
            slides_move_to_position(SLIDE_POSITION_CONE_STACK_3 + 40, true); // raise the slides
            //move claw down ready to score
            robot.clawGrab().setPosition(CLAW_OPEN);
            sleep(20);
            robot.clawElbow().setPosition(ELBOW_POSITION_CONE_GRAB); // was
            //sleep(150); // need to wait time for elbow to drop into position to get cone

            // move forward to cone stack
            encoder_inc = 400;  // 380 misses sometimes
            e = drivetrain.getBackEncoderValues();
            drivetrain.Drive(0.50);  // backwards will automatically decrement the encoder value in DriveTrain.addCountToEncoder()
            // change condition
            drivetrain.check_condition_encoder_distance( this,
                    e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
            drivetrain.stopAll();

            //*****************************************************************************************
            //*****************************************************************************************
            //*****************************************************************************************
            // Grab the SECOND cone
            //*****************************************************************************************
            //*****************************************************************************************
            //*****************************************************************************************
            sleep(150); // pause to ensure we are in position
            robot.clawGrab().setPosition(CLAW_CLOSE);
            sleep(500);
            robot.clawElbow().setPosition(ELBOW_POSITION_CONE_SCORE);
            slides_move_to_position(SLIDE_POSITION_LOW_SCORE, true); // raise the slides

            //move backward away from stack
            encoder_inc = 100; //70 too short //40 too short //30 too short //900 when all the way back //1000 too far
            e = drivetrain.getBackEncoderValues();
            drivetrain.Drive(-0.50);  // backwards will automatically decrement the encoder value in DriveTrain.addCountToEncoder()
            // change condition
            drivetrain.check_condition_encoder_distance( this,
                    e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
            drivetrain.stopAll();

            sleep(100);

            //strafe left to move away from ground junction
            encoder_inc = -50; //can only measure one wheel position with strafing so it a little wiggy
            e = drivetrain.getBackEncoderValues();
            // Do autonomous code to move to Location 2
            drivetrain.StrafeLeft(0.55);
            // change condition
            drivetrain.check_condition_encoder_distance( this,
                    e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
            //  was working with this => check_condition_Time( 1200 );
            drivetrain.stopAll();
            //
            //turn forward
            drivetrain.gyroTurn ( this, 0.4, -1, 1.50); // was 162 too short // 156 to the left
            drivetrain.stopAll();

            try {
                Thread.sleep(100);
            } catch (InterruptedException exc) {
                //ignore
            }

            //move slides and arm down
            robot.clawElbow().setPosition(ELBOW_POSITION_UP);
            sleep(25);
            DcMotorEx armDrive = robot.armDrive();
            armDrive.setTargetPosition(0);
            slides_move_to_position(0, false);


            //turn forward
            drivetrain.gyroTurn ( this, 0.4, 0, 1.50); // was 162 too short // 156 to the left
            drivetrain.stopAll();

            switch(tagOfInterest.id) {

                case ID_TAG_POSITION_1:
                    // strafe to adjust into the square
                    encoder_inc = 150;  // was 70 & too short
                    e = drivetrain.getBackEncoderValues();
                    drivetrain.StrafeLeft(0.55);
                    // change condition
                    drivetrain.check_condition_encoder_distance( this,
                            e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
                    drivetrain.stopAll();
                    break;

                case ID_TAG_POSITION_2:

                    //strafe into position 2
                    encoder_inc = -850;
                    e = drivetrain.getBackEncoderValues();
                    drivetrain.StrafeRight(0.55);
                    // change condition
                    drivetrain.check_condition_encoder_distance( this,
                            e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
                    drivetrain.stopAll();
                    break;

                case ID_TAG_POSITION_3:
                    //strafe into position 2 first
                    encoder_inc = -850;
                    e = drivetrain.getBackEncoderValues();
                    drivetrain.StrafeRight(0.55);
                    // change condition
                    drivetrain.check_condition_encoder_distance( this,
                            e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
                    drivetrain.stopAll();

                    // and then Strafe into position 3
                    encoder_inc = -1150; // -1250 was a bit too far
                    e = drivetrain.getBackEncoderValues();
                    drivetrain.StrafeRight(0.65);
                    // change condition
                    drivetrain.check_condition_encoder_distance( this,
                            e[DriveTrainIntf.LEFT_ENCODER], e[DriveTrainIntf.RIGHT_ENCODER], encoder_inc);
                    drivetrain.stopAll();
                    break;
            }
            sleep(50);
            //turn forward
            drivetrain.gyroTurn ( this, 0.4, 0, 1.50); // was 162 too short // 156 to the left
            drivetrain.stopAll();

            slides_move_to_position_even( 0, false);
            robot.clawElbow().setPosition(ELBOW_POSITION_UP);
            robot.clawWrist().setPosition(WRIST_POSITION_UP);
            robot.clawGrab().setPosition(CLAW_CLOSE);
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


    public void slides_move_to_position_even( int pos, boolean brake)
    {
        if (pos < 1) pos = 1; // prevent divide by zero error
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
            // check if the slides are going up at uneven speeds
            if ( Math.abs(rs - ls) > 28  ) {
                double err = ((rs-ls)/10)/pos;
                if (rs > ls) {
                    // slow the rs side of slide
                    robot.rightSlide().setVelocity(2000 - (err*2000));
                } else {
                    // slow the ls side of slide
                    robot.leftSlide().setVelocity(2000 + (err*2000));
                }
            }
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

        int pos = armDrive.getCurrentPosition();
        int target = 0;
        while(pos != target){
            if ( pos < (target+ARM_ERROR_MARGIN) ) {
                break;
            }
            sleep(25);
            pos = armDrive.getCurrentPosition();
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
