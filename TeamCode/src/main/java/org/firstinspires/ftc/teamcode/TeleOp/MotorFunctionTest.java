package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FusionFramework.DriveTrainIntfTest;
import org.firstinspires.ftc.teamcode.FusionFramework.HardwareSoftwareTest;
import org.firstinspires.ftc.teamcode.TeleOp.RobotCommands;

@Disabled
    @TeleOp(name = "Motor Functionality Test 1")
    public class MotorFunctionTest extends LinearOpMode
    {
        // Class Members for the robot hardware
        HardwareSoftwareTest robot = new HardwareSoftwareTest();
        DriveTrainIntfTest drivetrain;


        @Override
        public void runOpMode() throws InterruptedException
        {
            robot.init(hardwareMap);
            drivetrain = new DriveTrainIntfTest(robot); // init the drive train manager
            drivetrain.setMotorMode( DcMotor.RunMode.RUN_USING_ENCODER );


            waitForStart();

            // Turn on Motors
            drivetrain.Drive(0.30); // run at 30% velocity

            while (opModeIsActive()) {
                /*
                 * Send some stats to the telemetry
                 */

                //Display encoder values
                int[] f = drivetrain.getFrontEncoderValues(); // read front encoders
                int[] b = drivetrain.getBackEncoderValues(); // read back encoders
                telemetry.addLine(  "\n ** ENCODER FRONT WHEELS **\n");
                telemetry.addLine(String.format("    Left  =%d\n", f[DriveTrainIntfTest.LEFT_ENCODER]));
                telemetry.addLine(String.format("    Right =%d\n", f[DriveTrainIntfTest.RIGHT_ENCODER]));
                telemetry.addLine(  "\n ** ENCODER BACK WHEELS  **\n");
                telemetry.addLine(String.format("    Left  =%d\n", b[DriveTrainIntfTest.LEFT_ENCODER]));
                telemetry.addLine(String.format("    Right =%d\n", b[DriveTrainIntfTest.RIGHT_ENCODER]));
                telemetry.update();

                sleep(100);
            }
        }
    }
