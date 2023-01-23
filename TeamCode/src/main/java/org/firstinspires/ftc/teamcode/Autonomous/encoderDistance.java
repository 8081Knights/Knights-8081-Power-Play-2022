package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FusionFramework.DriveTrainIntf;
import org.firstinspires.ftc.teamcode.HardwareSoftware;

import java.util.concurrent.TimeUnit;


@Autonomous(name="Encoder Distance Test", group="Sydney")
public class encoderDistance extends OpMode {

    HardwareSoftware robot;
    DriveTrainIntf drivetrain;

    DcMotorEx m_RFDrive = null;
    DcMotorEx m_RRDrive = null;
    DcMotorEx m_LRDrive = null;
    DcMotorEx m_LFDrive = null;

    private boolean TicksChosen = false;
    private long left_ticks = 0;
    private long right_ticks = 0;
    private int cursorToggle = 0;
    private boolean XbuttonPressed = false;
    private boolean AbuttonPressed = false;
    private boolean YbuttonPressed = false;
    private boolean dpadPressed = false;
    private boolean LBumperbuttonPressed = false;
    private boolean RBumperbuttonPressed = false;

    private boolean settingLeftEncoder = true;
    private boolean settingRightEncoder = false;

    double velocity;

    boolean hasRunAlready = false;

    @Override
    public void init() {

        robot = new HardwareSoftware();
        robot.init(hardwareMap);

        m_LFDrive = robot.frontLeft();
        m_RFDrive = robot.frontRight();
        m_LRDrive = robot.backLeft();
        m_RRDrive = robot.backRight();

         drivetrain = new DriveTrainIntf(robot);

        right_ticks = 54;
        left_ticks = 54;
        velocity = 0.3;

    }
    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // Once a file is chosen, then there is no more changes
        if (TicksChosen) return;

        // This method will be called repeatedly when the INIT button is pressed.

        // Display the instructions and current filename
        telemetry.addLine("Set the number of encoder ticks you want the motors to run.");
        telemetry.addLine("dpad left/right +/- 1 to number under cursor - dpad up/down +/- 5 to number");
        telemetry.addLine("Bumpers moves cursor to left or right encoder values");
        telemetry.addLine("START the game when you've completed construction of the encoder counts.");

        switch (cursorToggle) {
            case 0:
            case 1:
                telemetry.addLine("\n Current Ticks:  LEFT: " + left_ticks + "   RIGHT: " + right_ticks + " " );
                break;
            case 2:
                if (settingLeftEncoder)
                    telemetry.addLine("\n Current Ticks:  LEFT:[" + left_ticks + "]  RIGHT: " + right_ticks + " " );
                else if (settingRightEncoder)
                    telemetry.addLine("\n Current Ticks:  LEFT: " + left_ticks + "   RIGHT:[" + right_ticks + "]" );
                break;
            default:
                if (settingLeftEncoder)
                    telemetry.addLine("\n Current Ticks:  LEFT:*" + left_ticks + "*  RIGHT: " + right_ticks + " ");
                else if (settingRightEncoder)
                    telemetry.addLine("\n Current Ticks:  LEFT: " + left_ticks + "   RIGHT:*" + right_ticks + "*" );
                cursorToggle = -1;
                break;
        }
        cursorToggle +=1;

        // Update the index based on the state of the dpad
        if (gamepad1.dpad_up) {
            // ignore if dpad was just pressed
            if (!dpadPressed) {
                if (settingLeftEncoder) {
                    left_ticks += 5;
                    right_ticks += 5;
                } else if (settingRightEncoder)
                    right_ticks += 5;
                // slow down loop timing so human has time to remove finger from button
                dpadPressed = true;
            }

        } else if (gamepad1.dpad_down) {
            // ignore if dpad was just pressed
            if (!dpadPressed) {
                if (settingLeftEncoder) {
                    left_ticks -= 5;
                    right_ticks -= 5;
                } else if (settingRightEncoder)
                    right_ticks -= 5;
                // slow down loop timing so human has time to remove finger from button
                dpadPressed = true;
            }

        } else if (gamepad1.dpad_left) {
            // ignore if dpad was just pressed
            if (!dpadPressed) {
                if (settingLeftEncoder) {
                    left_ticks -= 1;
                    right_ticks -= 1;
                } else if (settingRightEncoder)
                    right_ticks -= 1;
                // slow down loop timing so human has time to remove finger from button
                dpadPressed = true;
            }

        } else if (gamepad1.dpad_right) {
            // ignore if dpad was just pressed
            if (!dpadPressed) {
                if (settingLeftEncoder) {
                    left_ticks += 1;
                    right_ticks += 1;
                } else if (settingRightEncoder)
                    right_ticks += 1;
                // slow down loop timing so human has time to remove finger from button
                dpadPressed = true;
            }

        } else if ( gamepad1.left_bumper) {
            if (!LBumperbuttonPressed ) {
                // if Left Bumper button is pressed then jump to the left encoder setting
                settingLeftEncoder = true;
                settingRightEncoder = false;
                LBumperbuttonPressed = true;

            }
        } else if ( gamepad1.right_bumper) {
            if (!RBumperbuttonPressed ) {
                // if Left Bumper button is pressed then jump to the left encoder setting
                settingLeftEncoder = false;
                settingRightEncoder = true;
                RBumperbuttonPressed = true;

            }

        } else {
            // turn off all button presses
            XbuttonPressed = false;
            dpadPressed = false;

            LBumperbuttonPressed = false;
            RBumperbuttonPressed = false;
        }

    }

    @Override
    public void loop() {
        if (hasRunAlready) return;

        ElapsedTime eTimer = new ElapsedTime(ElapsedTime.MILLIS_IN_NANO);
        eTimer.reset();

        long now = eTimer.time(TimeUnit.MILLISECONDS);

        // get the current encoder values for all wheels
        int fr[] = drivetrain.getFrontEncoderValues();
        int bk[] = drivetrain.getBackEncoderValues();

        // Now set the motors to run to position

        int LFencodervalue = (int)(fr[DriveTrainIntf.LEFT_ENCODER]+left_ticks);
        int RFencodervalue = (int)(fr[DriveTrainIntf.RIGHT_ENCODER]+right_ticks);
        int LBencodervalue = (int)(bk[DriveTrainIntf.LEFT_ENCODER]+left_ticks);
        int RBencodervalue = (int)(bk[DriveTrainIntf.RIGHT_ENCODER]+right_ticks);

        m_LFDrive.setTargetPosition(LFencodervalue);
        m_RFDrive.setTargetPosition(RFencodervalue);
        m_LRDrive.setTargetPosition(LBencodervalue);
        m_RRDrive.setTargetPosition(RBencodervalue);

        m_LFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_RFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_LRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_RRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set the velocity for each motor
        long left_direction = (left_ticks < 0)?-1:1;
        long right_direction = (right_ticks < 0)?-1:1;

        // Now turn on the motors
        m_LFDrive.setVelocity( (velocity  * DriveTrainIntf.MOTOR_MAX_VELOCITY) *(left_direction) );
        m_RFDrive.setVelocity( (velocity  * DriveTrainIntf.MOTOR_MAX_VELOCITY) *(left_direction));
        m_LRDrive.setVelocity( (velocity  * DriveTrainIntf.MOTOR_MAX_VELOCITY) *(left_direction));
        m_RRDrive.setVelocity( (velocity  * DriveTrainIntf.MOTOR_MAX_VELOCITY) *(left_direction));

        // drive until all motors are at their position
        int motors_at_position = 0;
        do {
            // Check if we have to exit
            if ( (now > 8000) ) {
                // we must exit
                drivetrain.stopAll();
                return;
            }
            if (m_LFDrive.getCurrentPosition() == LFencodervalue) {
                // stop the motor
                m_LFDrive.setVelocity(0);
                // increment count that one motor hit its target
                motors_at_position |= 1; // flag that LF motor hit its mark
            }

            if (m_RFDrive.getCurrentPosition() == RFencodervalue) {
                // stop the motor
                m_RFDrive.setVelocity(0);
                // increment count that one motor hit its target
                motors_at_position |= 2; // set it 2
            }

            if (m_LRDrive.getCurrentPosition() == LBencodervalue) {
                // stop the motor
                m_LRDrive.setVelocity(0);
                // increment count that one motor hit its target
                motors_at_position |= 4; // set it 3
            }

            if (m_RRDrive.getCurrentPosition() == RBencodervalue) {
                // stop the motor
                m_RRDrive.setVelocity(0);
                // increment count that one motor hit its target
                motors_at_position |= 8; // set it 4
            }
            // wait for the encoders to get move a bit
            try {
                Thread.sleep(15); // time condition has not yet been met
            } catch (InterruptedException e) {
                // ignore interrupt
            }

        } while ( motors_at_position < 0x0F);
        hasRunAlready = true;
    }
}
