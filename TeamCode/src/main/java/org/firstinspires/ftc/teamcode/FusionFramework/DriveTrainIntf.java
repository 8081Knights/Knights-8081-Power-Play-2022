package org.firstinspires.ftc.teamcode.FusionFramework;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.teamcode.FusionFramework.GyroSensor;

public class DriveTrainIntf {

    private HardwareSoftware robot;
    private GyroSensor       gyro;

    DcMotorEx m_RFDrive    = null;
    DcMotorEx m_RRDrive     = null;
    DcMotorEx m_LRDrive      = null;
    DcMotorEx m_LFDrive     = null;

    static final double TURN_CONSTANT =   0.50;  // This determines the speed of the turn (higher value is faster turn)
    static final double ROTATE_CONSTANT =   0.75;

    // Settings for the Condition Completion Checks
    private double vel_LF = 0;  // Left  Front Wheel velocity setting
    private double vel_RF = 0;  // Right Front Wheel velocity setting
    private double vel_LR = 0;  // Left  Rear  Wheel velocity setting
    private double vel_RR = 0;  // Right Rear  Wheel velocity setting

    public DriveTrainIntf(HardwareSoftware hs) {

        robot = hs;
        gyro = null;

        m_LFDrive = robot.frontLeft();
        m_RFDrive = robot.frontRight();
        m_LRDrive = robot.backLeft();
        m_RRDrive = robot.backRight();

    }

    public void setGyro(GyroSensor g) { gyro = g; }

    public void stopAll() {
        m_LFDrive.setVelocity(0);
        m_RFDrive.setVelocity(0);
        m_LRDrive.setVelocity(0);
        m_RRDrive.setVelocity(0);
    }

    // Drive - drive straight in either forward or backward direction
    // forward - positive power ... backward is negative power value
    public void Drive(double velocity) {
        Drive( velocity, velocity );
    }

    // Rotate Left - stay in place but rotate robot left
    public void RotateLeft(double velocity) {
        stopAll(); // first ensure the robot is stopped
        double fwdSpeed  = TURN_CONSTANT * velocity;
        double revSpeed  = -1 * ROTATE_CONSTANT * velocity;
        /// Left-Front, Right-Front, Left-Rear, Right-Rear
        Drive( revSpeed, fwdSpeed, revSpeed, fwdSpeed );
    }

    // Rotate Right - stay in place but rotate robot right
    public void RotateRight(double velocity) {
        stopAll(); // first ensure the robot is stopped
        double fwdSpeed  = TURN_CONSTANT * velocity;
        double revSpeed  = -1 * ROTATE_CONSTANT * velocity;
        /// Left-Front, Right-Front, Left-Rear, Right-Rear
        Drive( fwdSpeed, revSpeed, fwdSpeed, revSpeed );
    }

    public void Drive(double velL, double velR) {
        if (velL < 0.0  && velR < 0.0) {
            setMotorForwardDirection(false);
            // change power to positive value
            velL = velL * -1;
            velR = velR * -1;
        } else if (velL >= 0.0  && velR >= 0.0) {
            setMotorForwardDirection(true);
        } else {
            // else this might be a turn command ???
            setMotorForwardDirection(true);
        }

        // convert relative vel to velocity based on max velocity
        vel_LF = vel_LR = velL * MOTOR_MAX_VELOCITY;
        vel_RF = vel_RR = velR * MOTOR_MAX_VELOCITY;

        // Don't power the front wheels
        m_LFDrive.setVelocity(vel_LF);
        m_RFDrive.setVelocity(vel_RF);
        m_LRDrive.setVelocity(vel_LR);
        m_RRDrive.setVelocity(vel_RR);
    }

    /// Left-Front, Right-Front, Left-Rear, Right-Rear
    public void Drive(double velLF, double velRF, double velLR, double velRR)
    {
        // ensure we are set to move forward on all motors
        setMotorForwardDirection(true);

        // convert relative vel to velocity based on max velocity
        vel_LF = velLF * MOTOR_MAX_VELOCITY;
        vel_RF = velRF * MOTOR_MAX_VELOCITY;
        vel_LR = velLR * MOTOR_MAX_VELOCITY;
        vel_RR = velRR * MOTOR_MAX_VELOCITY;

        // Power all wheels
        m_LFDrive.setVelocity(vel_LF);
        m_RFDrive.setVelocity(vel_RF);
        m_LRDrive.setVelocity(vel_LR);
        m_RRDrive.setVelocity(vel_RR);
    }

    /// Left-Front, Right-Front, Left-Rear, Right-Rear
    public void DriveByPower(double vel_LF, double vel_RF, double vel_LR, double vel_RR)
    {
        // ensure we are set to move forward on all motors
        setMotorForwardDirection(true);

        // Power all wheels
        m_LFDrive.setPower(vel_LF);
        m_RFDrive.setPower(vel_RF);
        m_LRDrive.setPower(vel_LR);
        m_RRDrive.setPower(vel_RR);
    }

    public boolean check_condition_Time(ElapsedTime eTimer, long time )
    {
        while ( eTimer.milliseconds() < 8000 ) {  // maximum wait time is 8 seconds
            if (eTimer.milliseconds() < time) {
                long sleep_time = (long)((time - eTimer.milliseconds()) / 2); // sleep half of the wait time left

                try {
                    Thread.sleep(sleep_time); // time condition has not yet been met
                } catch (InterruptedException e) {
                    // ignore interrupt
                }
            }
        }
        return true;
    }


    // *****************************************************8
    //  Code for handling the motor positions
    // ******************************************************

    // Maximum velocity of slowest motor under load is 1340 (calculated from test_Motor_MaxVelocity
    public static final int MOTOR_MAX_VELOCITY = 1340;
    // F = 32767 / maxV = 32767 / 1340 = 24.45298507
    // P = 0.1 * F  = 2.44529
    // I = 0.1 * P  = 0.244529
    // D = 0
    public static final float VELOCITY_PIDF_P = (float)2.44529;
    public static final float VELOCITY_PIDF_I = (float)0.244529;
    public static final float VELOCITY_PIDF_D = (float)0;
    public static final float VELOCITY_PIDF_F = (float)24.45298507;

    // A motor's maximum velocity is determined by running the motor at max power  motor.setPower(1.0);
    // and then reading the motor's velocity   v = motor.getVelocity();
    // the maximum value recorded when repeating this in a loop while running the motor under load is it's maximum velocity (when battery is at max too)
    // for multiple motors on a drive train, chose the LOWEST max power for all the motors, so you don't burn a motor out
    // REV ultraplanetary HD Hex motor is max rated at 6000 rpm under no load - which is 6000/60 = 100 ticks per second
    public static final int DCMOTOR_MAX_VELOCITY = 90; // the maximum ticks per second - determine by testing the motor

    /* setMotorMode()
    Set up the motors to run in a particular mode, such as:
        RUN_WITHOUT_ENCODER - just run but do not use encoders at all
        RUN_USING_ENCODERS - run using encoders you can check to see haow far motors have run
        RUN_TO_POSITION - use encoders so you can run to a particular position based on encoder reading
     */
    public void setMotorMode(DcMotor.RunMode mode) {
        m_LFDrive.setMode(mode);
        m_RFDrive.setMode(mode);
        m_LRDrive.setMode(mode);
        m_RRDrive.setMode(mode);

        switch (mode) {
            case RUN_WITHOUT_ENCODER:
            case STOP_AND_RESET_ENCODER:
            case RUN_WITHOUT_ENCODERS:
            case RESET_ENCODERS:
                break;
            case RUN_USING_ENCODER:
            case RUN_TO_POSITION:
            case RUN_USING_ENCODERS:
                /* change coefficients using methods included with DcMotorEx class. */
                PIDFCoefficients pidNew = new PIDFCoefficients(VELOCITY_PIDF_P, VELOCITY_PIDF_I, VELOCITY_PIDF_D, VELOCITY_PIDF_F);
                m_LFDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
                m_RFDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
                m_LRDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
                m_RRDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
                break;
        }
    }
    private boolean currentlyInForwardDirection = true;

    public void setMotorForwardDirection(boolean mode) {
        if (mode == true) {  // move robot FORWARD
            m_LFDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            m_RFDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            m_LRDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            m_RRDrive.setDirection(DcMotorSimple.Direction.FORWARD );
            currentlyInForwardDirection = true;
        } else {   // Move robot BACKWARD
            m_LFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            m_RFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            m_LRDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            m_RRDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            currentlyInForwardDirection = false;
        }
    }

    public void setMotorToBrakeOnStop(boolean mode) {
        if (mode == true) {
            m_LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m_RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m_LRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m_RRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            m_LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m_RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m_LRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m_RRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT );
        }
    }
}
