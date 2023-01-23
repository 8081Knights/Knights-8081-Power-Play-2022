package org.firstinspires.ftc.teamcode.FusionFramework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.teamcode.FusionFramework.GyroSensor;

import java.util.concurrent.TimeUnit;

public class DriveTrainIntf {

    private HardwareSoftware robot;
    private GyroSensor gyro;

    DcMotorEx m_RFDrive = null;
    DcMotorEx m_RRDrive = null;
    DcMotorEx m_LRDrive = null;
    DcMotorEx m_LFDrive = null;

    static final double TURN_CONSTANT = 0.50;  // This determines the speed of the turn (higher value is faster turn)
    static final double TURN_CONSTANT_SLOW = 0.65;

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

        // ensure we are set to move forward on all motors
        setMotorForwardDirection(true);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODERS);

    }

    public void setGyro(GyroSensor g) {
        gyro = g;
    }

    public void stopAll() {
        m_LFDrive.setVelocity(0);
        m_RFDrive.setVelocity(0);
        m_LRDrive.setVelocity(0);
        m_RRDrive.setVelocity(0);
    }

    // Drive - drive straight in either forward or backward direction
    // forward - positive power ... backward is negative power value
    public void Drive(double velocity) {
        // Don't do anything here -- call the one main drive function
        /// Left-Front, Right-Front, Left-Rear, Right-Rear
        Drive(velocity, velocity, velocity, velocity);
    }


    public void Drive(double velL, double velR) {
        // Don't do anything here -- call the one main drive function
        /// Left-Front, Right-Front, Left-Rear, Right-Rear
        Drive(velL, velR, velL, velR);
    }


    // Rotate Left - stay in place but rotate robot left
    public void RotateLeft(double velocity) {
        stopAll(); // first ensure the robot is stopped
        vel_LF = -1 * TURN_CONSTANT_SLOW * velocity* MOTOR_MAX_VELOCITY;;
        vel_LR = -1 * TURN_CONSTANT * velocity* MOTOR_MAX_VELOCITY;;
        vel_RF = TURN_CONSTANT * velocity* MOTOR_MAX_VELOCITY;;
        vel_RR = TURN_CONSTANT_SLOW * velocity* MOTOR_MAX_VELOCITY;;
        /// Left-Front, Right-Front, Left-Rear, Right-Rear
        Drive(); // call private drive() routine to turn on motors
    }

    // Rotate Right - stay in place but rotate robot right
    public void RotateRight(double velocity) {
        stopAll(); // first ensure the robot is stopped
        vel_LF = TURN_CONSTANT * velocity* MOTOR_MAX_VELOCITY;;
        vel_LR = TURN_CONSTANT_SLOW * velocity* MOTOR_MAX_VELOCITY;;
        vel_RF = -1 * TURN_CONSTANT_SLOW * velocity* MOTOR_MAX_VELOCITY;;
        vel_RR = -1 * TURN_CONSTANT * velocity* MOTOR_MAX_VELOCITY;;
        /// Left-Front, Right-Front, Left-Rear, Right-Rear
        Drive(); // call private drive() routine to turn on motors
    }

    /// Left-Front, Right-Front, Left-Rear, Right-Rear
    public void Drive(double velLF, double velRF, double velLR, double velRR) {
        // convert relative vel to velocity based on max velocity
        vel_LF = velLF * MOTOR_MAX_VELOCITY;
        vel_RF = velRF * MOTOR_MAX_VELOCITY;
        vel_LR = velLR * MOTOR_MAX_VELOCITY;
        vel_RR = velRR * MOTOR_MAX_VELOCITY;

        /// Left-Front, Right-Front, Left-Rear, Right-Rear
        Drive(); // call private drive() routine to turn on motors
    }

    private void Drive() {
        // Power all wheels from the saved velocity in our state
        // This function should only be used inside drivetrain AFTER
        //    any calculations on max speed and direction (negative or positive velocity)
        m_LFDrive.setVelocity(vel_LF);
        m_RFDrive.setVelocity(vel_RF);
        m_LRDrive.setVelocity(vel_LR);
        m_RRDrive.setVelocity(vel_RR);
    }

    /// Left-Front, Right-Front, Left-Rear, Right-Rear
    public void DriveByPower(double vel_LF, double vel_RF, double vel_LR, double vel_RR) {
        // ensure we are set to move forward on all motors
        setMotorForwardDirection(true);

        // Power all wheels
        m_LFDrive.setPower(vel_LF);
        m_RFDrive.setPower(vel_RF);
        m_LRDrive.setPower(vel_LR);
        m_RRDrive.setPower(vel_RR);
    }

    /* **********************************************************************************************
    *************************************************************************************************
         CHECK_CONDITION_TIME
    This routine returns TRUE when the time requested has completed (time is in milliseconds).
    The maximum wait time is 8 seconds and this routine will exit (with false) if more than 8 seconds tranpires.
    The granularity of the loop is at most 25 milliseconds, so waiting less than 25 milliseconds will not
    work well with this routine. increments greater than 25 milliseconds will also not have good consistency.
    You can change the sleep time to a smaller (or larger) number to change the granularity.
    Example: 75 milliseconds is request is good; 60 milliseconds will more often wait 75 milliseconds
             because 75 is a multiple of 25. Changin the sleep time to 10 milliseconds will give you about
             granularity of 10 milliseconds (so requests for 10, 30, 60 will be OK, but 55 and 75 will not).
    If this routine returns FALSE, then a timeout occured or stop was commanded by the OpMode (the caller)
    ************************************************************************************************
    ************************************************************************************************ */
    public boolean check_condition_Time(LinearOpMode caller, long time) {
        ElapsedTime eTimer = new ElapsedTime(ElapsedTime.MILLIS_IN_NANO);
        eTimer.reset();

        long now = eTimer.time(TimeUnit.MILLISECONDS);

        while ((now < 8000) && (!caller.isStopRequested())) {  // maximum wait time is 8 seconds

            if (now < time) {

                try {
                    Thread.sleep(25); // time condition has not yet been met
                } catch (InterruptedException e) {
                    // ignore interrupt
                }
            } else {
                // wait time has been met
                return true;
            }
            now = eTimer.time(TimeUnit.MILLISECONDS);
        }
        return false;
    }

    /* **********************************************************************************************
    *************************************************************************************************
         CHECK_CONDITION_ENCODER_DISTANCE
    This routine returns TRUE when any one of the REAR ENCODERS of the robot hit a particular mark.
    This routines works for both forward and backward motions.
    But this routine will return TRUE if either encoder (not both) hit the count mark.
    'count' should be calculated using the routine calcEncoderValueFromCentimeters() prior to calling this routine.
    If this routine returns FALSE, then a timeout occured or stop was commanded by the OpMode (the caller)
    ************************************************************************************************
    ************************************************************************************************ */
    public boolean check_condition_encoder_distance(
            LinearOpMode caller, long left, long right, long count) {

        int[] vals;
        // Use special methods to determine whether the encoder count
        // must be added or subtracted from the encoder start value
        // We use the current velocity of the wheel (which will be positive if moving forward, negative if moving backwards)
        // and the current encoder count (left or right) and
        // the increment count we need the encoder to count to
        long left_count = addCountToEncoder(vel_LR, left, count);
        long right_count = addCountToEncoder(vel_RR, right, count);

        // set timer for maximum loop running
        ElapsedTime eTimer = new ElapsedTime(ElapsedTime.MILLIS_IN_NANO);
        eTimer.reset();

        do {
            // read the encoders-
            vals = getBackEncoderValues();

            // This loop will end if EITHER left or right encoder hits the count mark
            // We determine whether the encoders should be moving forward or backwards
            // based on the comparison of the initial encoder value (left or right)
            // to the count value we calculated (left_count or right_count)

            // We end if the right encoder has hit it's mark
            if (right < right_count) {
                // motor is moving forward, so encoder value should be greater than the target count
                if (vals[RIGHT_ENCODER] >= (right_count - DC_MOTOR_TICK_ERROR)) {
                    return true;
                }
            } else {
                // motor is moving backwards, so encoder value should be less than the target count
                if (vals[RIGHT_ENCODER] <= (right_count + DC_MOTOR_TICK_ERROR)) {
                    return true;
                }
            }

            // OR We end if the left encoder has hit it's mark
            if (left < left_count) {
                // motor is moving forward, so encoder value should be greater than the target count
                if (vals[LEFT_ENCODER] >= (left_count - DC_MOTOR_TICK_ERROR)) {
                    return true;
                }
            } else {
                // motor is moving backwards, so encoder value should be less than the target count
                if (vals[LEFT_ENCODER] <= (left_count + DC_MOTOR_TICK_ERROR)) {
                    return true;
                }
            }
            // Sleep for a VERY SHORT time so we can check the encoder position often
            // this reduces the errors we get in movement
            try {
                Thread.sleep(5); // time condition has not yet been met
            } catch (InterruptedException e) {
                // ignore interrupt
            }
        } while ((eTimer.milliseconds() < 6000) && (!caller.isStopRequested()));  // maximum wait time is 8 seconds
        return false; // return false to indicate we ran out of time or stop was requested
    }

    /* **********************************************************************************************
    *************************************************************************************************
         CHECK_CONDITION_ENCODER_TURNING
    This routine returns TRUE when the REAR ENCODER of the outside wheel hits a particular count.
    This routine should be used to determine if a turn has completed based on a single encoder value.
    It isn't the most accurate, but it is a quick test of rotation.
    When turning or rotating left, the RIGHT encoder is used (since it should move the most)
    When turning or rotating right, the LEFT encoder is used.
    ************************************************************************************************
    ************************************************************************************************ */
    public boolean check_condition_encoder_turning(
            LinearOpMode caller, long left, long right, long count, boolean turn_left) {

        int[] vals;
        // Use special methods to determine whether the encoder count
        // must be added or subtracted from the encoder start value
        long left_count = addCountToEncoder(vel_LR, left, count);
        long right_count = addCountToEncoder(vel_RR, right, count);

        // set timer for maximum loop running
        ElapsedTime eTimer = new ElapsedTime(ElapsedTime.MILLIS_IN_NANO);
        eTimer.reset();

        do {
            // read the encoders-
            vals = getBackEncoderValues();

            if (turn_left) { // check right encoder -- it's the outside wheel
                // We end if the right encoder has hit it's mark
                if (right < right_count) {
                    // motor is moving forward, so encoder value should be greater than the target count
                    if (vals[RIGHT_ENCODER] >= (right_count - DC_MOTOR_TICK_ERROR)) {
                        return true;
                    }
                } else {
                    // motor is moving backwards, so encoder value should be less than the target count
                    if (vals[RIGHT_ENCODER] <= (right_count + DC_MOTOR_TICK_ERROR)) {
                        return true;
                    }
                }
            } else {
                // OR We end if the left encoder has hit it's mark
                if (left < left_count) {
                    // motor is moving forward, so encoder value should be greater than the target count
                    if (vals[LEFT_ENCODER] >= (left_count - DC_MOTOR_TICK_ERROR)) {
                        return true;
                    }
                } else {
                    // motor is moving backwards, so encoder value should be less than the target count
                    if (vals[LEFT_ENCODER] <= (left_count + DC_MOTOR_TICK_ERROR)) {
                        return true;
                    }
                }
            }
            try {
                Thread.sleep(5); // time condition has not yet been met
            } catch (InterruptedException e) {
                // ignore interrupt
            }
        } while ((eTimer.milliseconds() < 6000) && (!caller.isStopRequested()));  // maximum wait time is 8 seconds
        return false; // return false to indicate we ran out of time or stop was requested
    }

	/* **********************************************************************************************
	*************************************************************************************************
	     CHECK_CONDITION_ULTRASONIC_DISTANCE
	This routine returns TRUE when the provided ultrasonic distance sensor has a reading that
	indicates the "distance" requested. The value "forward" determines whether the reading should
	be less than the distance (when forward=true) or the reading should be greater than distance
	(when forward=false). Be careful to think about how the reading will change based from the
	selected sensor and the direction the robot is moving - forward DOES NOT necessarilly mean the
	robot is moving forward -- it means the robot is moving forward WITH RESPECT to the position of
	the sensor.
	This routine is BEST USED when moving straight inline with the direction of the ultrasonic
	Objects in the peripheral path MAY CAUSE INTERFERENCE with the readings so it is best used
	when movving off a wall or toward a wall with no poles or other obstacles around
	The ulstrasonic reports the SHORTEST distance to the CLOSEST object, not necessarilly the biggest object
	************************************************************************************************
	************************************************************************************************ */

    public boolean check_condition_ultrasonic_distance(
            LinearOpMode caller, double distance, MaxbotixUltrasonicI2c sensor, boolean forward) {

        sensor.measureRange(); // get the sensor to measure pur range

        // set timer for maximum loop running
        ElapsedTime eTimer = new ElapsedTime(ElapsedTime.MILLIS_IN_NANO);
        eTimer.reset();

        do {

            try {
                Thread.sleep(90); // time condition has not yet been met
            } catch (InterruptedException e) {
                // ignore interrupt
            }

            double reading = sensor.getLastRange();

            // We end if the right encoder has hit it's mark
            if (forward) { // distance should be getting smaller
                if (distance <= reading) {
                    return true;
                }
            } else {
                // motor is moving backwards, so encoder value should be less than the target count
                if (distance >= reading) {
                    return true;
                }
            }

            caller.telemetry.addLine(String.format("\nleft Ultrasonic=%d", robot.get_left_ultrasonic().getLastRange()));
            caller.telemetry.update();

            sensor.measureRange(); // get the sensor to measure pur range

        } while ((eTimer.milliseconds() < 6000) && (!caller.isStopRequested()));  // maximum wait time is 8 seconds
        return false; // return false to indicate we ran out of time or stop was requested
    }


    // *****************************************************8
    //  Code for handling the motor positions & encoders
    // ******************************************************

    // Maximum velocity of slowest motor under load is 1340 (calculated from test_Motor_MaxVelocity
    public static final int MOTOR_MAX_VELOCITY = 1340;
    // F = 32767 / maxV = 32767 / 1340 = 24.45298507
    // P = 0.1 * F  = 2.44529
    // I = 0.1 * P  = 0.244529
    // D = 0
    public static final float VELOCITY_PIDF_P = (float) 2.44529;
    public static final float VELOCITY_PIDF_I = (float) 0.244529;
    public static final float VELOCITY_PIDF_D = (float) 0;
    public static final float VELOCITY_PIDF_F = (float) 24.45298507;

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
            m_RRDrive.setDirection(DcMotorSimple.Direction.FORWARD);
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
            m_RRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }


    // This method calculates the correct value the encoder
    // should be set to after "inc" ticks, taking into account the
    // current velocity "vel" and whether is currently going forward
    // or backwards
    private long addCountToEncoder(double vel, long base, long inc) {
        // determine if the robot is configured to go backwards
        if (currentlyInForwardDirection) {
            // Determine Left Drive count addition
            if (vel > 0) { // we are moving left-rear forwards
                base = base + inc;
            } else {
                base = base - inc; // subtract encoder count because motor is moving backwards
            }

        } else { // Robot is currently running in backwards-mode

            if (vel > 0) { // we are moving left-rear backwards
                base = base - inc;
            } else {
                base = base + inc; // subtract encoder count because motor is moving backwards
            }
        }
        return base;
    }

    public static final int LEFT_ENCODER = 0;
    public static final int RIGHT_ENCODER = 1;

    public int[] getBackEncoderValues() {
        int[] retVal = new int[2];
        retVal[LEFT_ENCODER] = m_LRDrive.getCurrentPosition();
        retVal[RIGHT_ENCODER] = m_RRDrive.getCurrentPosition();

        return retVal;
    }

    public int[] getFrontEncoderValues() {
        int[] retVal = new int[2];
        retVal[LEFT_ENCODER] = m_LFDrive.getCurrentPosition();
        retVal[RIGHT_ENCODER] = m_RFDrive.getCurrentPosition();

        return retVal;
    }

    static final double WHEEL_DIAMETER = 9.5; // in centimeters
    static final double WHEEL_CIRCUMFERENCE = 30.80;  // calculated by measurement
    static final long DCMOTOR_TICK_COUNT = (560); // 28 ticks per motor revolution * 20:1 gearbox
    static final long DC_MOTOR_TICK_ERROR = 28;

    /*
    Provide the difference between the Ending Encoder Value and the Starting Encoder Value
    and this function calculates the distance travelled based on the wheel diameter
    CAUTION: Wheel slippage and turning will cause this calculation to be erroneous, but is a fair estimate
    If this function is used on non-driven or odometry wheels, this calculation will be more accurate
     */
    public float calcDistanceFromEncoderValue(int numberEncoderTicks) {
        return (float) (WHEEL_CIRCUMFERENCE * (float) (numberEncoderTicks / DCMOTOR_TICK_COUNT));
    }

    public long calcEncoderValueFromCentimeters(double centimeters) {
        return Math.round(centimeters / WHEEL_CIRCUMFERENCE) * DCMOTOR_TICK_COUNT;
    }

}