package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FusionFramework.DriveTrainIntf;
import org.firstinspires.ftc.teamcode.FusionFramework.MaxbotixUltrasonicI2c;
import org.firstinspires.ftc.teamcode.HardwareSoftware;

import java.util.concurrent.TimeUnit;

public class UltrasonicPoleDetection {

    DcMotorEx m_RFDrive = null;
    DcMotorEx m_RRDrive = null;
    DcMotorEx m_LRDrive = null;
    DcMotorEx m_LFDrive = null;

    public UltrasonicPoleDetection(HardwareSoftware robot) {

        m_LFDrive = robot.frontLeft();
        m_RFDrive = robot.frontRight();
        m_LRDrive = robot.backLeft();
        m_RRDrive = robot.backRight();

    }

    static final long[] distance_table = {
            300,  // move -12 CM
            300,  // move -11 CM
            300,  // move -10 CM
            300,  // move -9 CM
            300,  // move -8 CM
            300,  // move -7 CM
            300,  // move -6 CM
            300,  // move -5 CM
            300,  // move -4 CM
            200, // move -3 CM
            125, // move -2 CM
            50,  // move -1 CM
            0, // move 0 CM
            50,  // move 1 CM
            125, // move 2 CM
            200, // move 3 CM
            300,  // move 4 CM
            300,  // move 5 CM
            300,  // move 6 CM
            300,  // move 7 CM
            300,  // move 8 CM
            300,  // move 9 CM
            300,  // move 10 CM
            300,  // move 11 CM
            300  // move 12 CM
    };
    static final long distance_table_offset = 12;

    public boolean positionTOPole(LinearOpMode caller,
                                  double distance,  // desired distance to pole
                                  MaxbotixUltrasonicI2c left_sensor,
                                  MaxbotixUltrasonicI2c right_sensor,
                                  double velocity,
                                  DriveTrainIntf drivetrain) {
        ElapsedTime eTimer = new ElapsedTime(ElapsedTime.MILLIS_IN_NANO);
        eTimer.reset();

        long now = eTimer.time(TimeUnit.MILLISECONDS);

        drivetrain.getBackEncoderValues();
        drivetrain.getFrontEncoderValues();

        while ((now < 8000) && (!caller.isStopRequested())) {  // maximum wait time is 8 seconds

            // request input from the two ultrasonic sensors on front
            left_sensor.measureRange();
            right_sensor.measureRange();

            // wait for the sensors to get reading
            try {
                Thread.sleep(90); // time condition has not yet been met
            } catch (InterruptedException e) {
                // ignore interrupt
            }

            // read the current values
            double left = left_sensor.getLastRange();
            double right = right_sensor.getLastRange();

            // Check if we are done
            if (( Math.abs(left - right) <= 1.0 ) && // within 1 CM of each other
                    (Math.abs(left-distance) <= 2.0) ) {  // within 2 CM of the distance to pole
                return true;
            }
            // get the current encoder values for all wheels
            int fr[] = drivetrain.getFrontEncoderValues();
            int bk[] = drivetrain.getBackEncoderValues();

            // Move forward or backward with left wheels based on the distance - left (use move to position)
            long left_correction = Math.round(left - distance);  // negative value tells how far to move back
            // make sure that we aren't asking to move larger than our table size
            if ( left_correction > distance_table_offset ) {
                left_correction  = distance_table_offset;
            } else if ( left_correction < (-1*distance_table_offset) ) {
                left_correction = distance_table_offset;
            }
            long left_ticks = distance_table[(int)(left_correction+distance_table_offset)];


            // move forward or backward with the right wheels based on distance - right (use move to position)
            long right_correction = Math.round(right - distance);  // negative value tells how far to move back
            // make sure that we aren't asking to move larger than our table size
            if ( right_correction > distance_table_offset ) {
                right_correction  = distance_table_offset;
            } else if ( right_correction < (-1*distance_table_offset) ) {
                right_correction = distance_table_offset;
            }
            long right_ticks = distance_table[(int)(right_correction+distance_table_offset)];

            // Now set the motors to run to position
            m_LFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m_RFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m_LRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m_RRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            int LFencodervalue = (int)(fr[DriveTrainIntf.LEFT_ENCODER]+left_ticks);
            int RFencodervalue = (int)(fr[DriveTrainIntf.RIGHT_ENCODER]+right_ticks);
            int LBencodervalue = (int)(bk[DriveTrainIntf.LEFT_ENCODER]+left_ticks);
            int RBencodervalue = (int)(bk[DriveTrainIntf.RIGHT_ENCODER]+right_ticks);

            m_LFDrive.setTargetPosition(LFencodervalue);
            m_RFDrive.setTargetPosition(RFencodervalue);
            m_LRDrive.setTargetPosition(LBencodervalue);
            m_RRDrive.setTargetPosition(RBencodervalue);

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
                if ( (now > 8000) || (caller.isStopRequested())) {
                    // we must exit
                    drivetrain.stopAll();
                    return false;
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
        }
        return false; // failed to position to pole
    }
}
