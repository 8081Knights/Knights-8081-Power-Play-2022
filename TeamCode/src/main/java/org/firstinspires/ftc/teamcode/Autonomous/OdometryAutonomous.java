package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FusionFramework.DriveTrainIntf;
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

        // Tell ultrasonics to get the range
        robot.get_left_ultrasonic().measureRange();
        robot.get_right_ultrasonic().measureRange();

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

            telemetry.addLine(String.format("\nLeft Ultrasonic=%d", robot.get_left_ultrasonic().getLastRange() ));
            telemetry.addLine(String.format("\nRight Ultrasonic=%d", robot.get_right_ultrasonic().getLastRange() ));
            telemetry.update();

            // Tell ultrasonics to get the range
            robot.get_left_ultrasonic().measureRange();
            robot.get_right_ultrasonic().measureRange();

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
            robot.clawGrab().setPosition(0);
            // move claw into center of the robot
            robot.clawWrist().setPosition(0);

            //move forward a bit before turing
            long encoder_inc = 34;
            int[] e = drivetrain.getBackEncoderValues();
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

            //turn 45 degrees toward low pole
            drivetrain.gyroTurn ( this, 0.4, -45, 1.50);
            drivetrain.stopAll();
            // Pause to prevent wheel slippage and jerking
            try {
                Thread.sleep(500);
            } catch (InterruptedException exc) {
                //ignore
            }

            //score on the low pole
            //******************************************
            score_low_position();
            sleep(15000);

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

    public void score_low_position()
    {
        robot.leftSlide().setTargetPosition(1350);
        robot.rightSlide().setTargetPosition(1350);


        robot.leftSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.rightSlide().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armHome();
        robot.leftSlide().setVelocity(2000);
        robot.rightSlide().setVelocity(2000);

        do {
            if (robot.leftSlide().getCurrentPosition() == 1350) {
                robot.leftSlide().setVelocity(0);
            }
            if (robot.rightSlide().getCurrentPosition() == 1350) {
                robot.rightSlide().setVelocity(0);
            }
        } while(robot.leftSlide().getCurrentPosition() != 1350 && robot.rightSlide().getCurrentPosition() != 1350);

        robot.leftSlide().setVelocity(0);
        robot.rightSlide().setVelocity(0);

    }

    public void armHome()
    {
        DcMotorEx armDrive = robot.armDrive();
        armDrive.setTargetPosition(0);

        armDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armDrive.setVelocity(1000);

        while(armDrive.getCurrentPosition() != 0){
            sleep(25);
        }

        armDrive.setVelocity(0);
    }
}

